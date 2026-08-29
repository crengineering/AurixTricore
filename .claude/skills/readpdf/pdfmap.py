#!/usr/bin/env python
"""pdfmap - locate and extract material from large PDFs without reading them page by page.

Subcommands:
  info   <pdf>                     page count, title, whether text is extractable
  toc    <pdf> [-n N]              table of contents (embedded outline, else scraped)
  offset <pdf> --printed P         calibrate printed page number -> PDF page index
  grep   <pdf> PATTERN [-C n]      regex search over full text, reports PDF page numbers
  text   <pdf> -f A -l B           extract a PDF page range as layout-preserved text

Page numbers on stdout are always PDF indices (1-based) unless labelled "printed".
"""
import argparse
import io
import re
import subprocess
import sys
from pathlib import Path

HAVE_PDFTOTEXT = None


def _pdftotext(pdf, first=None, last=None, layout=True):
    """Extract text via pdftotext. Returns '' if the tool is unavailable."""
    global HAVE_PDFTOTEXT
    cmd = ["pdftotext"]
    if layout:
        cmd.append("-layout")
    if first:
        cmd += ["-f", str(first)]
    if last:
        cmd += ["-l", str(last)]
    cmd += [str(pdf), "-"]
    try:
        out = subprocess.run(cmd, capture_output=True, timeout=300)
        HAVE_PDFTOTEXT = True
        return out.stdout.decode("utf-8", "replace")
    except (FileNotFoundError, subprocess.TimeoutExpired):
        HAVE_PDFTOTEXT = False
        return ""


def _pypdf_text(pdf, first, last):
    import pypdf
    r = pypdf.PdfReader(str(pdf))
    return "\f".join(
        r.pages[i].extract_text() or "" for i in range(first - 1, min(last, len(r.pages)))
    )


def page_text(pdf, first, last):
    t = _pdftotext(pdf, first, last)
    return t if t.strip() else _pypdf_text(pdf, first, last)


def n_pages(pdf):
    import pypdf
    return len(pypdf.PdfReader(str(pdf)).pages)


def cmd_info(a):
    import pypdf
    r = pypdf.PdfReader(str(a.pdf))
    n = len(r.pages)
    meta = r.metadata or {}
    sample = page_text(a.pdf, 1, min(3, n))
    print(f"file      : {Path(a.pdf).name}")
    print(f"pages     : {n}")
    print(f"title     : {(meta.get('/Title') or '?').strip()}")
    print(f"outline   : {'yes' if _outline(r) else 'no (scrape the printed TOC)'}")
    print(f"text layer: {'yes' if len(sample.strip()) > 200 else 'NO - scanned? needs OCR'}")
    print(f"extractor : {'pdftotext' if HAVE_PDFTOTEXT else 'pypdf'}")


def _outline(reader):
    """Flatten an embedded outline into [(level, title, pdf_page)]."""
    out = []

    def walk(items, depth=0):
        for it in items:
            if isinstance(it, list):
                walk(it, depth + 1)
                continue
            try:
                pg = reader.get_destination_page_number(it) + 1
                out.append((depth, str(it.title).strip(), pg))
            except Exception:
                pass

    try:
        walk(reader.outline)
    except Exception:
        return []
    return out


# "3.12   Power Supply Current . . . . . . . 446"
TOC_LINE = re.compile(r"^\s*(\d+(?:\.\d+)*)\s+(.+?)[\s.]{3,}(\d{1,4})\s*$")


def cmd_toc(a):
    import pypdf
    r = pypdf.PdfReader(str(a.pdf))
    ol = _outline(r)
    if ol:
        for depth, title, pg in ol:
            print(f"{'  ' * depth}{title}   [pdf p{pg}]")
        return
    # No embedded outline: scrape the printed contents pages.
    scan = min(a.scan, len(r.pages))
    for line in page_text(a.pdf, 1, scan).splitlines():
        m = TOC_LINE.match(line)
        if m:
            num, title, pg = m.groups()
            print(f"{num:<10} {title.strip():<62} printed p{pg}")


def cmd_offset(a):
    """Find delta where pdf_index = printed + delta, by locating the printed number."""
    total = n_pages(a.pdf)
    target = str(a.printed)
    hits = []
    for idx in range(max(1, a.printed - 5), min(total, a.printed + 40) + 1):
        t = page_text(a.pdf, idx, idx)
        tail = "\n".join(t.strip().splitlines()[-4:])
        head = "\n".join(t.strip().splitlines()[:3])
        if re.search(rf"(?<!\d){re.escape(target)}(?!\d)", tail + head):
            hits.append(idx)
    if not hits:
        print(f"no page found showing printed number {target}; try a distinctive heading with `grep`")
        return
    for h in hits:
        print(f"printed {target} -> pdf page {h}   (delta {h - a.printed:+d})")


def cmd_grep(a):
    total = n_pages(a.pdf)
    rx = re.compile(a.pattern, 0 if a.case else re.I)
    shown = 0
    # Extract in chunks so one bad page cannot cost the whole run.
    CHUNK = 50
    for start in range(1, total + 1, CHUNK):
        end = min(start + CHUNK - 1, total)
        pages = page_text(a.pdf, start, end).split("\f")
        for off, ptext in enumerate(pages):
            idx = start + off
            if idx > total:
                break
            lines = ptext.splitlines()
            for ln, line in enumerate(lines):
                if rx.search(line):
                    print(f"--- pdf p{idx}:{ln + 1}")
                    lo, hi = max(0, ln - a.context), min(len(lines), ln + a.context + 1)
                    for cl in lines[lo:hi]:
                        print(f"    {cl.rstrip()}")
                    shown += 1
                    if shown >= a.max:
                        print(f"... stopped at {a.max} hits (use --max to raise)")
                        return


def cmd_text(a):
    total = n_pages(a.pdf)
    last = min(a.last, total)
    if last - a.first > 60:
        sys.exit(f"refusing {last - a.first + 1} pages at once; extract <=60 and distill as you go")
    sys.stdout.write(page_text(a.pdf, a.first, last))


def main():
    p = argparse.ArgumentParser(prog="pdfmap", description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest="cmd", required=True)

    s = sub.add_parser("info"); s.add_argument("pdf"); s.set_defaults(fn=cmd_info)

    s = sub.add_parser("toc"); s.add_argument("pdf")
    s.add_argument("-n", "--scan", type=int, default=12, help="pages to scan for a printed TOC")
    s.set_defaults(fn=cmd_toc)

    s = sub.add_parser("offset"); s.add_argument("pdf")
    s.add_argument("--printed", type=int, required=True)
    s.set_defaults(fn=cmd_offset)

    s = sub.add_parser("grep"); s.add_argument("pdf"); s.add_argument("pattern")
    s.add_argument("-C", "--context", type=int, default=2)
    s.add_argument("--max", type=int, default=40)
    s.add_argument("--case", action="store_true", help="case sensitive")
    s.set_defaults(fn=cmd_grep)

    s = sub.add_parser("text"); s.add_argument("pdf")
    s.add_argument("-f", "--first", type=int, required=True)
    s.add_argument("-l", "--last", type=int, required=True)
    s.set_defaults(fn=cmd_text)

    a = p.parse_args()
    if not Path(a.pdf).exists():
        sys.exit(f"no such file: {a.pdf}")
    a.fn(a)


if __name__ == "__main__":
    try:
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    except Exception:
        pass
    main()
