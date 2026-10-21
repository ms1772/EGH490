#!/usr/bin/env python3
"""Render the flight run-sheets as self-contained Word documents.

Each output bundles one run-sheet with the common crew material appended as
annexes, so a single file can be opened in a meeting or carried to a bench
without chasing cross-references.

Outputs land in the project folder and are deliberately NOT committed: the
repository carries code, not written documents.

This file lives in the repo on purpose. It used to be reachable only by an
absolute path into a session-scoped temp scratchpad, so every builder that
imported it broke the moment that folder was cleaned -- on the morning of a
flight, the day the run-sheet needed a change.
"""

from __future__ import annotations

import re
import sys
from datetime import date
from pathlib import Path

from docx import Document
from docx.enum.section import WD_SECTION
from docx.enum.table import WD_TABLE_ALIGNMENT
from docx.enum.text import WD_ALIGN_PARAGRAPH, WD_BREAK
from docx.oxml import OxmlElement
from docx.oxml.ns import qn
from docx.shared import Cm, Pt, RGBColor

# Derived from this file's own location, not hard-coded: this module now lives
# in the repo and has to keep working wherever the repo is checked out.
REPO = Path(__file__).resolve().parents[2]
RUNSHEETS = REPO / "flight_ops" / "runsheets"
COMMON = RUNSHEETS / "common"
OUT_DIR = REPO.parent

NAVY = RGBColor(0x00, 0x00, 0x00)
ACCENT = RGBColor(0x1A, 0x1A, 0x1A)
MUTED = RGBColor(0x59, 0x59, 0x59)
CODE_BG = "F4F4F4"
QUOTE_BG = "FFFFFF"

MONO = "Consolas"
BODY = "Calibri"


# --------------------------------------------------------------------------- #
# low-level helpers
# --------------------------------------------------------------------------- #

def shade(element, hex_fill: str) -> None:
    """Apply a background fill to a paragraph or table cell."""
    shd = OxmlElement("w:shd")
    shd.set(qn("w:val"), "clear")
    shd.set(qn("w:color"), "auto")
    shd.set(qn("w:fill"), hex_fill)
    element.get_or_add_pPr().append(shd) if hasattr(element, "get_or_add_pPr") \
        else element.append(shd)


def shade_paragraph(par, hex_fill: str) -> None:
    par.paragraph_format.space_before = Pt(4)
    par.paragraph_format.space_after = Pt(4)
    shd = OxmlElement("w:shd")
    shd.set(qn("w:val"), "clear")
    shd.set(qn("w:color"), "auto")
    shd.set(qn("w:fill"), hex_fill)
    par._p.get_or_add_pPr().append(shd)


def left_bar(par, hex_color: str) -> None:
    """A coloured left border -- used for block quotes."""
    borders = OxmlElement("w:pBdr")
    left = OxmlElement("w:left")
    left.set(qn("w:val"), "single")
    left.set(qn("w:sz"), "18")
    left.set(qn("w:space"), "8")
    left.set(qn("w:color"), hex_color)
    borders.append(left)
    par._p.get_or_add_pPr().append(borders)


INLINE = re.compile(r"(\*\*.+?\*\*|`[^`]+`|\*[^*]+\*|~~.+?~~)")


def add_inline(par, text: str, base_bold: bool = False,
               base_italic: bool = False, color: RGBColor | None = None) -> None:
    """Write text into a paragraph, honouring **bold**, `code`, *italic*, ~~strike~~."""
    text = re.sub(r"\[([^\]]+)\]\([^)]+\)", r"\1", text)   # links -> label
    for piece in INLINE.split(text):
        if not piece:
            continue
        run = par.add_run()
        run.bold, run.italic = base_bold, base_italic
        if color is not None:
            run.font.color.rgb = color
        if piece.startswith("**") and piece.endswith("**"):
            run.text, run.bold = piece[2:-2], True
        elif piece.startswith("~~") and piece.endswith("~~"):
            run.text, run.font.strike = piece[2:-2], True
        elif piece.startswith("`") and piece.endswith("`"):
            run.text = piece[1:-1]
            run.font.name = MONO
            run.font.size = Pt(9.5)
            run.font.color.rgb = ACCENT
        elif piece.startswith("*") and piece.endswith("*") and len(piece) > 2:
            run.text, run.italic = piece[1:-1], True
        else:
            run.text = piece


def split_row(line: str) -> list[str]:
    return [c.strip() for c in line.strip().strip("|").split("|")]


# --------------------------------------------------------------------------- #
# markdown -> docx
# --------------------------------------------------------------------------- #

def render_markdown(doc: Document, lines: list[str]) -> None:
    i = 0
    while i < len(lines):
        raw = lines[i].rstrip()
        stripped = raw.strip()

        # fenced code
        if stripped.startswith("```"):
            i += 1
            block = []
            while i < len(lines) and not lines[i].strip().startswith("```"):
                block.append(lines[i].rstrip())
                i += 1
            i += 1
            if block:
                par = doc.add_paragraph()
                par.paragraph_format.left_indent = Cm(0.4)
                run = par.add_run("\n".join(block))
                run.font.name = MONO
                run.font.size = Pt(9)
                shade_paragraph(par, CODE_BG)
            continue

        # horizontal rule -> skip (markdown separators, not content)
        if re.fullmatch(r"-{3,}|_{3,}|\*{3,}", stripped):
            i += 1
            continue

        if not stripped:
            i += 1
            continue

        # table
        if stripped.startswith("|") and i + 1 < len(lines) and \
                re.match(r"^\s*\|[\s:|-]+\|\s*$", lines[i + 1]):
            header = split_row(stripped)
            i += 2
            rows = []
            while i < len(lines) and lines[i].strip().startswith("|"):
                rows.append(split_row(lines[i].strip()))
                i += 1
            table = doc.add_table(rows=1, cols=len(header))
            table.style = "Table Grid"
            table.alignment = WD_TABLE_ALIGNMENT.LEFT
            for idx, cell_text in enumerate(header):
                cell = table.rows[0].cells[idx]
                cell.text = ""
                par = cell.paragraphs[0]
                add_inline(par, cell_text, base_bold=True, color=NAVY)
                shade(cell._tc.get_or_add_tcPr(), "E8EDF2")
            for row in rows:
                cells = table.add_row().cells
                for idx in range(len(header)):
                    cells[idx].text = ""
                    add_inline(cells[idx].paragraphs[0],
                               row[idx] if idx < len(row) else "")
            for row in table.rows:
                for cell in row.cells:
                    for par in cell.paragraphs:
                        par.paragraph_format.space_before = Pt(2)
                        par.paragraph_format.space_after = Pt(2)
                        for run in par.runs:
                            if run.font.size is None:
                                run.font.size = Pt(9.5)
            doc.add_paragraph()
            continue

        # headings
        m = re.match(r"^(#{1,6})\s+(.*)$", stripped)
        if m:
            level = len(m.group(1))
            title = m.group(2).strip()
            par = doc.add_paragraph()
            par.paragraph_format.space_before = Pt(14 if level <= 2 else 10)
            par.paragraph_format.space_after = Pt(4)
            add_inline(par, title, base_bold=True,
                       color=NAVY if level <= 2 else ACCENT)
            for run in par.runs:
                run.bold = True
                run.font.size = Pt({1: 18, 2: 14, 3: 12, 4: 11}.get(level, 10.5))
            if level == 1:
                bdr = OxmlElement("w:pBdr")
                bottom = OxmlElement("w:bottom")
                bottom.set(qn("w:val"), "single")
                bottom.set(qn("w:sz"), "4")
                bottom.set(qn("w:space"), "3")
                bottom.set(qn("w:color"), "999999")
                bdr.append(bottom)
                par._p.get_or_add_pPr().append(bdr)
            i += 1
            continue

        # block quote
        if stripped.startswith(">"):
            block = []
            while i < len(lines) and lines[i].strip().startswith(">"):
                block.append(lines[i].strip().lstrip(">").strip())
                i += 1
            par = doc.add_paragraph()
            par.paragraph_format.left_indent = Cm(0.5)
            par.paragraph_format.space_after = Pt(6)
            add_inline(par, " ".join(t for t in block if t), base_italic=True)
            for run in par.runs:
                run.font.size = Pt(10)
            continue

        # checkbox
        m = re.match(r"^[-*]\s+\[([ xX])\]\s+(.*)$", stripped)
        if m:
            par = doc.add_paragraph()
            par.paragraph_format.left_indent = Cm(0.6)
            par.paragraph_format.space_after = Pt(2)
            box = par.add_run("\u2612  " if m.group(1).lower() == "x" else "\u2610  ")
            box.font.size = Pt(12)
            add_inline(par, m.group(2))
            i += 1
            continue

        # bullets
        m = re.match(r"^(\s*)[-*]\s+(.*)$", raw)
        if m:
            depth = len(m.group(1)) // 2
            par = doc.add_paragraph(style="List Bullet" if depth == 0 else "List Bullet 2")
            par.paragraph_format.space_after = Pt(2)
            add_inline(par, m.group(2))
            i += 1
            continue

        # numbered
        m = re.match(r"^(\s*)(\d+)\.\s+(.*)$", raw)
        if m:
            depth = len(m.group(1)) // 2
            par = doc.add_paragraph(style="List Number" if depth == 0 else "List Number 2")
            par.paragraph_format.space_after = Pt(2)
            add_inline(par, m.group(3))
            i += 1
            continue

        # plain paragraph
        par = doc.add_paragraph()
        par.paragraph_format.space_after = Pt(6)
        add_inline(par, stripped)
        i += 1


# --------------------------------------------------------------------------- #
# document assembly
# --------------------------------------------------------------------------- #

def new_document() -> Document:
    doc = Document()
    style = doc.styles["Normal"]
    style.font.name = BODY
    style.font.size = Pt(10.5)
    for section in doc.sections:
        section.page_width, section.page_height = Cm(21.0), Cm(29.7)
        section.left_margin = section.right_margin = Cm(2.0)
        section.top_margin = section.bottom_margin = Cm(1.8)
    return doc


def add_cover(doc: Document, title: str, subtitle: str, scope: str) -> None:
    """A plain title block at the top of page one -- no separate cover page."""
    par = doc.add_paragraph()
    run = par.add_run(title)
    run.bold, run.font.size = True, Pt(17)

    par = doc.add_paragraph()
    par.paragraph_format.space_after = Pt(2)
    run = par.add_run(subtitle)
    run.font.size, run.font.color.rgb = Pt(11), MUTED

    par = doc.add_paragraph()
    par.paragraph_format.space_after = Pt(12)
    run = par.add_run(f"{scope}   Generated {date.today().isoformat()}")
    run.font.size, run.font.color.rgb = Pt(9), MUTED


def build(out_name: str, title: str, subtitle: str, scope: str,
          main: Path, annexes: list[tuple[str, Path]]) -> Path:
    doc = new_document()
    add_cover(doc, title, subtitle, scope)
    render_markdown(doc, main.read_text(encoding="utf-8").splitlines())

    for annex_title, path in annexes:
        doc.add_paragraph().add_run().add_break(WD_BREAK.PAGE)
        par = doc.add_paragraph()
        run = par.add_run(annex_title)
        run.bold, run.font.size, run.font.color.rgb = True, Pt(11), MUTED
        render_markdown(doc, path.read_text(encoding="utf-8").splitlines())

    out = OUT_DIR / out_name
    try:
        doc.save(out)
    except PermissionError:
        # Almost always "the file is open in Word". Write alongside rather
        # than failing the whole run, and let the caller say so.
        out = out.with_name(out.stem + "_UPDATED" + out.suffix)
        doc.save(out)
    return out


def main() -> int:
    annexes = [
        ("ANNEX A", COMMON / "crew_brief.md"),
        ("ANNEX B", COMMON / "emergency_actions.md"),
        ("ANNEX C", COMMON / "callouts.md"),
    ]
    missing = [p for _, p in annexes if not p.exists()]
    for path in (RUNSHEETS / "indoor_o134.md", RUNSHEETS / "outdoor_serf.md"):
        if not path.exists():
            missing.append(path)
    if missing:
        for path in missing:
            print(f"missing source: {path}", file=sys.stderr)
        return 2

    written = [
        build("EGH490_Flight_Runsheet_Indoor_O134.docx",
              "Indoor Flight Run-Sheet",
              "O-134 \u2014 Motion-Capture Localisation, Netted Volume",
              "Operational procedure. Not a risk assessment.",
              RUNSHEETS / "indoor_o134.md", annexes),
        build("EGH490_Flight_Runsheet_Outdoor_SERF.docx",
              "Outdoor Flight Run-Sheet",
              "SERF Brisbane \u2014 GPS Localisation, Open Site",
              "Operational procedure. Not a risk assessment, not an approvals record.",
              RUNSHEETS / "outdoor_serf.md", annexes),
    ]
    explainer = Path(__file__).with_name("mocap_explainer.md")
    if explainer.exists():
        written.append(
            build("Mocap_To_Drone_How_It_Works.docx",
                  "How Motion Capture Gets Into The Drone",
                  "OptiTrack to flight controller — what connects to what",
                  "Personal reference.",
                  explainer, []))

    for path in written:
        print(f"wrote {path}  ({path.stat().st_size / 1024:.0f} KB)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
