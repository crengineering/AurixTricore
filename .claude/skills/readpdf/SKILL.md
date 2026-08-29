---
name: readpdf
description: Turn a PDF (datasheet, reference manual, erratasheet, board manual) into a distilled, grep-able markdown note under docs/ — so it never has to be read again. Use whenever a question would otherwise be answered by opening a PDF, or when the user says "read this datasheet", "what does the manual say about X", "make notes on this PDF".
---

# readpdf — a PDF becomes a note, once

A 548-page datasheet costs a fortune to read and the knowledge dies with the
session. The job here is to spend that cost **once** and leave behind a
markdown note that answers the question next time for ~200 lines of context.

**The deliverable is always a file under `docs/`.** Answering the user's
question without writing the note means the next session pays again — that is
the failure mode this skill exists to prevent.

Helper: `python .claude/skills/readpdf/pdfmap.py` (referred to below as `pdfmap`).
Available locally: `pdftotext`, `pypdf`, `fitz`. Never `Read` a PDF page-by-page
until step 4 says to.

---

## The loop

### 1. Orient — is there already a note?

```bash
ls docs/*.md
```

If a note for this document exists, **extend it**; don't start a second one.
Then:

```bash
python .claude/skills/readpdf/pdfmap.py info <pdf>
```

Check `text layer`. If it says `NO - scanned`, text extraction is useless — skip
to the visual path (step 4) and say so up front.

### 2. Locate — never scan linearly

```bash
pdfmap toc  <pdf>                      # embedded outline -> exact PDF pages
pdfmap toc  <pdf> -n 20                # no outline: scrapes the printed TOC
pdfmap grep <pdf> "SOPE|stopOnPacket" -C 3
```

`toc` output labelled `[pdf p N]` is a real PDF index — use it directly.
Output labelled `printed p N` is the number printed on the page, which is
offset from the PDF index by front matter. Calibrate once:

```bash
pdfmap offset <pdf> --printed 446      # -> "printed 446 -> pdf page 452 (delta +6)"
```

Apply that delta to every printed number from the same document.

`grep` is the workhorse for "does this manual say anything about X" — it
searches the whole document for a few hundred tokens of output.

### 3. Extract — narrow, in passes

```bash
pdfmap text <pdf> -f 452 -l 468
```

Capped at 60 pages per call, deliberately. Extract one section, distill it into
the note, then extract the next. Do **not** dump a whole chapter into context
and hope to summarise it at the end.

### 4. Fall back to eyes only when text fails

Text extraction reliably loses: **pin-out tables, schematics, board layout
figures, timing diagrams, register bit-field boxes.** Vector labels come out
scattered across the page with no spatial relationship — on this project's
board manual, the connector labels (X501, X702, X703) extract as isolated words
with no usable geometry, which is exactly why `docs/PINNING.md` still marks the
X702 hole numbers unverified.

When `grep`/`text` gives you scrambled or empty output for a figure:

```
Read(file_path="docs/<file>.pdf", pages="47-52")   # max 20 pages/request
```

That renders the pages visually. It is expensive — use it for the specific
figure, never for prose you could have extracted.

### 5. Write the note

Target `docs/<TOPIC>.md`. Match the house style of the existing notes
(`docs/BMP581.md`, `docs/MMC5983MA.md`, `docs/ILLD_NOTES.md`): **95–330 lines,
dense, tables over paragraphs, grep-able headings.**

Every note starts with provenance:

```markdown
# <Topic>

Source: `docs/infineon-tc39x-datasheet-en.pdf` §3.12 (pdf p452–468, printed 446–462)
Distilled 2026-08-08. Re-extract if the source revision changes.
```

Then, in order of how often it will be needed:

1. **Answers** — the numbers, register values, pin names, limits. Concrete.
2. **Traps** — anything that contradicts the obvious reading, defaults that
   lie, errata that apply. This is the highest-value section; give it a table.
3. **Cross-references** — link to `docs/PINNING.md`, `docs/ILLD_NOTES.md`, the
   driver that uses this.
4. **Gaps** — what you did *not* extract, so the next session knows whether to
   trust the note's silence. Never omit this section.

Rules:

- **Distil, do not dump.** Raw extracted text in a note is a failure — it is as
  expensive to read as the PDF and less trustworthy.
- **Copy exact values verbatim** — register addresses, bit positions, min/max
  ratings, part numbers. Paraphrasing a number is how errors enter.
- **Mark what you inferred** vs. what the document states. If a figure was
  unreadable, write "not verified — figure did not extract", never a guess.
- Record the erratasheet's verdict alongside a feature if one applies. On this
  project `infineon-aurix-tc39x-bd-step-erratasheet-en.pdf` is a standing
  cross-check for any peripheral note.

### 6. Close the loop

- Add one line to `docs/` navigation if an index exists.
- If the note changes how the project must be built or wired, say so plainly in
  the response — a note nobody acts on is not done.
- Save a memory only for a finding that surprised you (a default that lies, a
  figure that contradicts the text), not for "read the datasheet".

---

## Worked example

> "What's the DTS accuracy and does anything in the errata affect it?"

```bash
pdfmap info  docs/infineon-tc39x-datasheet-en.pdf          # 548p, outline yes
pdfmap toc   docs/infineon-tc39x-datasheet-en.pdf | grep -i temp
                                                            # 3.11 Temperature Sensor, printed 445
pdfmap offset docs/infineon-tc39x-datasheet-en.pdf --printed 445
pdfmap text  docs/infineon-tc39x-datasheet-en.pdf -f 451 -l 453
pdfmap grep  docs/infineon-aurix-tc39x-bd-step-erratasheet-en.pdf "DTS|temperature sensor" -C 3
```

→ write `docs/DTS.md` with the accuracy table, the errata verdict, and a
cross-link to the `IfxDts` section of `docs/ILLD_NOTES.md`.

---

## This project's PDFs

| File | Pages | Contains |
|---|---|---|
| `infineon-tc39x-datasheet-en.pdf` | 548 | pin config per package, electrical limits, ADC/DSADC, oscillator, temp sensor, power |
| `infineon-aurix-tc39x-bd-step-erratasheet-en.pdf` | 308 | **check this for every peripheral** before trusting the datasheet |
| `infineon-triboardmanual-tc3x9-um-en.pdf` | 57 | board connectors, jumpers, schematic, power — the X501/X702/X703 headers |
| `infineon-tc39x-bd-step-datasheet-addendum-datasheet-en.pdf` | 25 | BD-step deltas |

Note: the TC3xx **User Manual** (peripheral register descriptions — STM, GTM,
QSPI, I2C chapters) is **not in this repo**. Register-level questions cannot be
answered from these files; use `docs/ILLD_NOTES.md`, the iLLD headers, or ask
the user to add the manual.

The `... (1).pdf` duplicate is byte-identical to the addendum — ignore it.
