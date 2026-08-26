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
    python tools/mag_cal.py --decl 3.9      # set magnetic declination too

While it collects, turn the board slowly through as many orientations as you
can -- all six faces up, and tumble it between them. The fit needs the samples
spread over the sphere; rotating about one axis only gives a ring, and a ring
does not determine a centre in three dimensions. The tool tells you if the
coverage was too poor to trust.
"""
from __future__ import annotations

import argparse
import csv
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
    args = ap.parse_args()

    conf = create_application_from_config({
        "Transport": {"Eth": {"host": HOST, "port": PORT,
                              "protocol": "UDP", "ipv6": False}},
    })

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
        centre, scale, residual, err = fit_ellipsoid(p)
        if err:
            sys.exit(f"fit failed: {err}\nCollect again with more tumbling.")

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
        else:
            print("\n(nothing written -- re-run with --write to store it)")


if __name__ == "__main__":
    main()
