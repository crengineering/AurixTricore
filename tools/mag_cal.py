#!/usr/bin/env python3
"""Hard-iron / soft-iron calibration for the MMC5983MA, over XCP.

Why this is needed
------------------
The magnetometer does not measure Earth's field. It measures Earth's field PLUS
the board's own magnetics -- the DC-DC converter, the copper pour, anything
ferrous nearby. That addition is constant IN THE SENSOR FRAME, so as the board
turns it traces out a sphere that is OFF-CENTRE. The tell is that |B| changes
when the board is merely rotated, and it is a heading error that varies with
heading, which no amount of filtering removes.

Measured on this board 2026-08-26, uncalibrated: |B| swung between 0.428 and
0.984 gauss depending on orientation, against a true field of about 0.48 G in
Munich. That is the whole reason Ahrs.c applies the correction and Xcp_Nvm
stores it.

What it fits
------------
An axis-aligned ellipsoid:

    a*x^2 + b*y^2 + c*z^2 + d*x + e*y + f*z = 1

which is the general quadric with no cross terms. The centre of that ellipsoid
is the HARD-IRON offset; the ratio of its semi-axes is the SOFT-IRON scale. The
cross terms are deliberately left out -- a full 3x3 soft-iron matrix needs a
much better-conditioned dataset than a hand rotation gives, and on a board
whose distortion is dominated by a permanent magnet nearby the diagonal part is
almost all of it.

Usage
-----
    python tools/mag_cal.py                 # collect, fit, print. Writes nothing.
    python tools/mag_cal.py --seconds 90
    python tools/mag_cal.py --write         # also store to NVM and SAVE
                                            # (and update calibration/board.json)
    python tools/mag_cal.py --decl 3.9      # set magnetic declination too
    python tools/mag_cal.py --export F.json # persist the fit to a file
    python tools/mag_cal.py --restore-from calibration/board.json
                                            # push a saved set back to NVM +
                                            # SAVE, then read it back. This is
                                            # the restore step after a reflash
                                            # (flash.bat erases DFLASH).

While it collects, turn the board slowly through as many orientations as you
can -- all six faces up, and tumble it between them. The fit needs the samples
spread over the sphere; rotating about one axis only gives a ring, and a ring
does not determine a centre in three dimensions. The tool tells you if the
coverage was too poor to trust.
"""
from __future__ import annotations

import argparse
import csv
import datetime
import json
import pathlib
import struct
import sys
import time

import numpy as np

from pyxcp.master import Master
from pyxcp.config import create_application_from_config

HOST, PORT = "192.168.0.10", 5555

ALL_OCTANTS = [(i, j, k) for i in (-1, 1) for j in (-1, 1) for k in (-1, 1)]

# A sample only counts toward an octant if it is genuinely OUT in that
# direction, not merely on the noisy side of the mean. The threshold is a
# fraction of the FIELD MAGNITUDE rather than of the observed spread -- which
# matters more than it sounds: with the board sitting still, the spread IS the
# noise, so a spread-relative test finds all eight octants in a stationary
# dataset and cheerfully reports 100% coverage. That very nearly let a garbage
# calibration be written to flash.
OCTANT_FRAC = 0.25


def octants(p: np.ndarray, centre: np.ndarray) -> set:
    d = p - centre
    thresh = OCTANT_FRAC * float(np.median(np.linalg.norm(p, axis=1)))
    sig = np.where(np.abs(d) > thresh, np.sign(d), 0.0).astype(int)
    return {tuple(t) for t in sig if 0 not in t} & set(ALL_OCTANTS)

# Xcp_Data: raw (uncorrected) field, see Measurements.h
MAG_X = 0x70030090

# Xcp_Nvm: where the result goes, see Nvm.h
NVM_COMMAND = 0x70030204
NVM_MAG_OFF_X = 0x70030210
NVM_MAG_SCALE_X = 0x7003021C
NVM_MAG_DECL = 0x70030228
NVM_CMD_SAVE = 0x45564153  # "SAVE"


def read_mag(x):
    raw = bytes(x.shortUpload(12, MAG_X, 0))
    return struct.unpack("<fff", raw[:12])


def write_f32(x, addr, value):
    x.setMta(addr, 0)
    x.download(struct.pack("<f", float(value)))


def write_u32(x, addr, value):
    x.setMta(addr, 0)
    x.download(struct.pack("<I", int(value)))


def read_f32s(x, addr, n):
    raw = bytes(x.shortUpload(4 * n, addr, 0))
    return struct.unpack("<" + "f" * n, raw[: 4 * n])


# The versioned home of the board's calibration (gap fix 2026-08-29): DFLASH
# is erased on every reflash, so the values must exist OUTSIDE the chip too.
BOARD_JSON = pathlib.Path(__file__).resolve().parent.parent / "calibration" / "board.json"


def export_fit(path, centre, scale, residual, cov, samples, decl, note):
    data = {
        "date": datetime.date.today().isoformat(),
        "note": note,
        "hard_iron_G": {"x": float(centre[0]), "y": float(centre[1]), "z": float(centre[2])},
        "soft_iron": {"x": float(scale[0]), "y": float(scale[1]), "z": float(scale[2])},
        "declination_deg": decl,
        "fit_residual_pct": round(float(residual), 2),
        "octant_coverage_pct": round(100.0 * cov, 0),
        "samples": int(samples),
    }
    p = pathlib.Path(path)
    p.parent.mkdir(parents=True, exist_ok=True)
    p.write_text(json.dumps(data, indent=2) + "\n", encoding="utf-8")
    print(f"fit exported to {p}")


def restore_from(conf, path):
    """Push a saved calibration set to NVM + SAVE, then read it back.

    This is the board-discipline restore step after any reflash: verify the
    read-back matches the file before trusting heading."""
    data = json.loads(pathlib.Path(path).read_text(encoding="utf-8"))
    hi, si = data["hard_iron_G"], data["soft_iron"]
    with Master("eth", config=conf) as x:
        x.connect()
        for i, axis in enumerate("xyz"):
            write_f32(x, NVM_MAG_OFF_X + 4 * i, hi[axis])
            write_f32(x, NVM_MAG_SCALE_X + 4 * i, si[axis])
        if data.get("declination_deg") is not None:
            write_f32(x, NVM_MAG_DECL, data["declination_deg"])
        write_u32(x, NVM_COMMAND, NVM_CMD_SAVE)
        off = read_f32s(x, NVM_MAG_OFF_X, 3)
        scl = read_f32s(x, NVM_MAG_SCALE_X, 3)
    want = [hi["x"], hi["y"], hi["z"], si["x"], si["y"], si["z"]]
    got = list(off) + list(scl)
    ok = all(abs(w - g) < 1e-4 for w, g in zip(want, got))
    print(f"restored from {path} ({data.get('date')}, note: {data.get('note')})")
    print(f"read-back offsets {off}  scales {scl}")
    print("read-back MATCHES the file -- heading may be trusted." if ok else
          "!! READ-BACK MISMATCH -- do not trust heading; investigate before flying.")
    if not ok:
        sys.exit(1)


def fit_sphere(p: np.ndarray):
    """Least-squares SPHERE: hard iron only, no soft iron.

    Deliberately the primary fit. |p - c|^2 = r^2 expands to

        x^2 + y^2 + z^2 = 2*cx*x + 2*cy*y + 2*cz*z + (r^2 - |c|^2)

    which is LINEAR in the four unknowns and stays well conditioned even when
    the rotation was uneven -- which a hand rotation always is. The ellipsoid
    fit below needs the three SQUARED terms to be independently determined, and
    that wants coverage a person tumbling a board rarely achieves.

    Hard iron is the dominant error anyway: it shifts the sphere bodily, while
    soft iron only distorts its shape by a few percent.
    """
    x, y, z = p[:, 0], p[:, 1], p[:, 2]
    A = np.column_stack([2 * x, 2 * y, 2 * z, np.ones_like(x)])
    b = (x * x) + (y * y) + (z * z)
    sol, *_ = np.linalg.lstsq(A, b, rcond=None)
    centre = sol[:3]
    r2 = sol[3] + float(centre @ centre)
    if r2 <= 0:
        return None, None, None, "sphere fit degenerate (non-positive radius)"

    scale = np.ones(3)
    norms = np.linalg.norm(p - centre, axis=1)
    residual = 100.0 * norms.std() / norms.mean()
    return centre, scale, residual, None


def fit_ellipsoid(p: np.ndarray):
    """Least-squares axis-aligned ellipsoid: hard iron AND per-axis soft iron.

    Falls back to fit_sphere when the data cannot support it. That is not a
    corner case -- it is the normal outcome of a hand rotation, and the reason
    this returns a usable answer instead of refusing.
    """
    x, y, z = p[:, 0], p[:, 1], p[:, 2]
    # Design matrix for a*x^2 + b*y^2 + c*z^2 + d*x + e*y + f*z = 1
    A = np.column_stack([x * x, y * y, z * z, x, y, z])
    b = np.ones_like(x)
    coef, *_ = np.linalg.lstsq(A, b, rcond=None)
    a, bb, c, d, e, f = coef

    if min(a, bb, c) <= 0:
        return fit_sphere(p)

    centre = np.array([-d / (2 * a), -e / (2 * bb), -f / (2 * c)])
    # Substituting the centre back gives the right-hand side of the centred form
    k = 1.0 + (a * centre[0] ** 2) + (bb * centre[1] ** 2) + (c * centre[2] ** 2)
    if k <= 0:
        return fit_sphere(p)

    radii = np.sqrt(k / np.array([a, bb, c]))
    # Scale each axis onto the MEAN radius, so the corrected magnitude stays in
    # gauss rather than being normalised to 1 -- |B| remains a physical check.
    scale = radii.mean() / radii

    # Reject an implausible soft-iron correction rather than apply it. Real soft
    # iron on a board like this is a few percent; anything past 1.5x means the
    # quadratic terms were not actually determined by the data.
    if (scale.max() / scale.min()) > 1.5:
        return fit_sphere(p)

    corrected = (p - centre) * scale
    norms = np.linalg.norm(corrected, axis=1)
    residual = 100.0 * norms.std() / norms.mean()
    return centre, scale, residual, None


def coverage(p: np.ndarray, centre: np.ndarray) -> float:
    """Fraction of the sphere's octants that got a sample genuinely out there."""
    return len(octants(p, centre)) / 8.0


def report(p, centre, scale, residual):
    """Print the fit. Shared by the live path and --load so a re-fit is
    directly comparable with the run it came from."""
    cov = coverage(p, centre)
    raw_norm = np.linalg.norm(p, axis=1)
    corrected = np.linalg.norm((p - centre) * scale, axis=1).mean()

    print(f"\nsamples          {len(p)}")
    print(f"raw |B|          {raw_norm.min():.4f} .. {raw_norm.max():.4f} G "
          f"(spread {100 * (raw_norm.max() - raw_norm.min()) / raw_norm.mean():.0f}%)")
    print(f"octant coverage  {cov * 100:.0f}%")
    print(f"\nhard iron  [G]   X {centre[0]:+.5f}  Y {centre[1]:+.5f}  Z {centre[2]:+.5f}")
    print(f"soft iron        X {scale[0]:.5f}  Y {scale[1]:.5f}  Z {scale[2]:.5f}")
    print(f"corrected |B|    {corrected:.4f} G")
    print(f"residual         {residual:.2f} %   (under 2% is a good fit)")
    print("  ^ this is the LOCAL FIELD MAGNITUDE and the check on")
    print("    MMC5983_COUNTS_PER_GAUSS: ~0.48 G in Munich. A clean 2x or 4x")
    print("    error here is a scaling bug, not a calibration problem.")
    return cov


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--seconds", type=float, default=60.0, help="collection time")
    ap.add_argument("--rate", type=float, default=0.05, help="sample interval [s]")
    ap.add_argument("--write", action="store_true", help="store to NVM and SAVE")
    ap.add_argument("--decl", type=float, default=None,
                    help="magnetic declination [deg east] to store as well")
    ap.add_argument("--save", default="mag_points.csv",
                    help="where to dump the raw samples (always written)")
    ap.add_argument("--load", default=None,
                    help="re-fit a previous dump instead of collecting")
    ap.add_argument("--export", default=None, metavar="FILE",
                    help="persist the fit result as JSON (a --write always "
                         f"also updates {BOARD_JSON.name} in calibration/)")
    ap.add_argument("--restore-from", default=None, metavar="FILE",
                    help="write a saved calibration set to NVM + SAVE and "
                         "verify by read-back (the after-reflash restore)")
    args = ap.parse_args()

    conf = create_application_from_config({
        "Transport": {"Eth": {"host": HOST, "port": PORT,
                              "protocol": "UDP", "ipv6": False}},
    })

    if args.restore_from:
        restore_from(conf, args.restore_from)
        return

    if args.load:
        # Re-fit a previous dump with no hardware attached. This is what makes
        # a saved run worth saving: the fit can be revisited and argued with
        # long after the board has moved.
        p = np.array([[float(r["magX"]), float(r["magY"]), float(r["magZ"])]
                      for r in csv.DictReader(open(args.load, encoding="utf-8"))])
        centre, scale, residual, err = fit_ellipsoid(p)
        if err:
            sys.exit(f"fit failed: {err}")
        cov = report(p, centre, scale, residual)
        if args.export:
            export_fit(args.export, centre, scale, residual, cov, len(p),
                       args.decl, f"re-fit of {args.load}")
        return

    pts = []
    with Master("eth", config=conf) as x:
        x.connect()
        print(f"Collecting for {args.seconds:.0f} s -- turn the board slowly "
              f"through ALL orientations (every face up, and tumble between them).")
        t_end = time.time() + args.seconds
        last_print = 0.0
        n_oct = 0
        while True:
            pts.append(read_mag(x))
            time.sleep(args.rate)
            left = t_end - time.time()

            # Live coverage feedback. The centre is not known yet, so the
            # running mean stands in for it -- crude, but it is only steering
            # the operator, and it converges toward the real centre as the
            # rotation evens out. This is the point of the whole display:
            # turning a blind two-minute chore into something with a progress
            # bar, so a run cannot silently end up unfittable.
            arr = np.array(pts)
            mid = arr.mean(axis=0)
            octs = octants(arr, mid)
            n_oct = len(octs)

            if (left - last_print < -0.5) or (last_print == 0.0):
                last_print = left
                bar = "".join("#" if o in octs else "." for o in ALL_OCTANTS)
                hint = "ALL CORNERS HIT" if n_oct == 8 else "keep tumbling"
                # flush: without it a redirected stdout is block-buffered and
                # the progress line does not appear until the buffer fills,
                # which defeats the entire purpose of a live display.
                print(f"  corners {n_oct}/8 [{bar}]  {len(pts):5d} samples  "
                      f"{max(left, 0.0):4.0f}s left  -- {hint}      ",
                      end="\r", flush=True)

            # Stop on time, but never before every corner has been seen: the
            # fit cannot succeed without them, so ending early only wastes the
            # rotation that was already done.
            if (left <= 0.0) and (n_oct == 8):
                break
            if left <= -30.0:
                # Give up rather than trap the operator (or sit there until the
                # XCP connection times out and buries the reason in a traceback).
                print(f"\n\nstopping at {n_oct}/8 corners after 30 s of grace.")
                break
        print()

        p = np.array(pts, dtype=float)

        # Dump BEFORE fitting, unconditionally. A rotation is physical effort
        # that cannot be replayed, and an earlier version discarded the samples
        # whenever the fit failed -- which cost two runs, and then cost the
        # ability to re-verify a run that SUCCEEDED, because the reference fit
        # in docs/MMC5983MA.md disagreed with it and there was nothing left to
        # check. Keep the data; the fit is cheap to redo, the tumbling is not.
        with open(args.save, "w", newline="", encoding="utf-8") as fh:
            wr = csv.writer(fh)
            wr.writerow(["magX", "magY", "magZ"])
            wr.writerows(p)
        print(f"raw samples saved to {args.save}")

        centre, scale, residual, err = fit_ellipsoid(p)
        if err:
            print(f"\nfit failed: {err}", file=sys.stderr)
            print("per-axis spread of what was collected:", file=sys.stderr)
            for i, ax in enumerate("XYZ"):
                print(f"  {ax}  {p[:, i].min():+.4f} .. {p[:, i].max():+.4f} "
                      f"(std {p[:, i].std():.4f})", file=sys.stderr)
            print(f"re-fit later with:  --load {args.save}", file=sys.stderr)
            sys.exit(1)

        cov = coverage(p, centre)
        raw_norm = np.linalg.norm(p, axis=1)

        print(f"\nsamples          {len(p)}")
        print(f"raw |B|          {raw_norm.min():.4f} .. {raw_norm.max():.4f} G "
              f"(spread {100*(raw_norm.max()-raw_norm.min())/raw_norm.mean():.0f}%)")
        print(f"octant coverage  {cov*100:.0f}%")
        print(f"\nhard iron  [G]   X {centre[0]:+.5f}  Y {centre[1]:+.5f}  Z {centre[2]:+.5f}")
        print(f"soft iron        X {scale[0]:.5f}  Y {scale[1]:.5f}  Z {scale[2]:.5f}")
        print(f"corrected |B|    {np.linalg.norm((p-centre)*scale, axis=1).mean():.4f} G")
        print(f"residual         {residual:.2f} %   (under 2% is a good fit)")

        if cov < 0.99:
            print("\n!! Coverage is incomplete: some octants of the sphere were never "
                  "visited, so the centre in those directions is extrapolated. "
                  "Re-run and tumble the board more.")
        if residual > 5.0:
            print("\n!! Residual is high. Either the rotation was too restricted, or "
                  "something nearby is magnetic and moved with the board.")

        if args.write:
            if cov < 0.99 or residual > 5.0:
                sys.exit("\nrefusing to --write a fit this poor; re-collect first")
            for i, addr in enumerate(range(NVM_MAG_OFF_X, NVM_MAG_OFF_X + 12, 4)):
                write_f32(x, addr, centre[i])
            for i, addr in enumerate(range(NVM_MAG_SCALE_X, NVM_MAG_SCALE_X + 12, 4)):
                write_f32(x, addr, scale[i])
            if args.decl is not None:
                write_f32(x, NVM_MAG_DECL, args.decl)
            write_u32(x, NVM_COMMAND, NVM_CMD_SAVE)
            print("\nwritten to NVM and SAVE issued -- survives a power cycle.")
            print("Verify: rotate the board and watch MagFieldCorrected "
                  "(0x7003054C) stay put.")
            # A set that went to NVM is the board's truth -- persist it
            # OUTSIDE the chip too, unconditionally: flash.bat erases DFLASH,
            # and calibration/board.json is what --restore-from reads back.
            export_fit(BOARD_JSON, centre, scale, residual, cov, len(p),
                       args.decl, "live fit, written to NVM")
        else:
            print("\n(nothing written -- re-run with --write to store it)")
        if args.export:
            export_fit(args.export, centre, scale, residual, cov, len(p),
                       args.decl, "live fit" + (", written to NVM" if args.write else ""))


if __name__ == "__main__":
    main()
