"""Build results/presentation/P6R_presentation.pptx.

Start from the .pptx export of the Google Slides deck (all slides, images and styling kept as they are)
and add three tables to slide 4, "Forward Kinematic models of P6R": the URDF chain, the DH table and the
PoE screws of the P6R, with the numbers of the report (mount at z = 0.42 m).

Run from the repo root:   python scripts/build_presentation.py
"""
import sys
from pathlib import Path
from lxml import etree
from pptx import Presentation
from pptx.util import Inches, Pt
from pptx.enum.text import PP_ALIGN, MSO_ANCHOR
from pptx.dml.color import RGBColor

sys.stdout.reconfigure(encoding="utf-8")
ROOT = Path(__file__).resolve().parent.parent
SRC = ROOT / "results" / "presentation" / "google_export_original.pptx"
OUT = ROOT / "results" / "presentation" / "P6R_presentation.pptx"
NS = "http://schemas.openxmlformats.org/drawingml/2006/main"
NO_STYLE = "{2D5ABB26-0587-4C30-8999-92F81FD0307C}"          # "No Style, No Grid"

FONT_PT = 8                 # table text (compact tables, chosen by the author)
LABEL_PT = 10               # heading above each table
ROW_H = 0.21                # inches
TOP_LABEL, TOP_TABLE = 1.18, 1.45
X_LEFT = 0.62               # left edge of the first table: its text lines up with the slide title
RED = "FF0000"              # the slide joint x, same red as "P6R" on slide 2

# ------------------------------------------------------------------ table contents (report model)
# URDF: only the moving joints are shown (x and q1..q6); the fixed mount, base and flange rows are left out.
URDF = [["joint", "xyz [m]", "rpy [rad]"],
        ["x", "(0, 0, 0)", "(0, 0, 0)"],
        ["q1", "(0, 0, 0.1807)", "(0, 0, 0)"],
        ["q2", "(0, 0, 0)", "(π/2, 0, 0)"],
        ["q3", "(−0.6127, 0, 0)", "(0, 0, 0)"],
        ["q4", "(−0.57155, 0, 0.17415)", "(0, 0, 0)"],
        ["q5", "(0, −0.11985, 0)", "(π/2, 0, 0)"],
        ["q6", "(0, 0.11655, 0)", "(π/2, π, π)"]]
DH = [["joint", "θ", "d [m]", "a [m]", "α"],
      ["x", "0", "x", "0", "−π/2"],
      ["q1", "q1 + π/2", "0.1807", "0", "π/2"],
      ["q2", "q2", "0", "−0.6127", "0"],
      ["q3", "q3", "0", "−0.57155", "0"],
      ["q4", "q4", "0.17415", "0", "π/2"],
      ["q5", "q5", "0.11985", "0", "−π/2"],
      ["q6", "q6", "0.11655", "0", "0"]]
SCREW = [["joint", "ω", "point r on axis [m]"],
         ["x", "0", "v = (1, 0, 0)"],
         ["q1", "(0, 0, 1)", "(0, 0, 0.6007)"],
         ["q2", "(0, 1, 0)", "(0, 0, 0.6007)"],
         ["q3", "(0, 1, 0)", "(0.6127, 0, 0.6007)"],
         ["q4", "(0, 1, 0)", "(1.18425, 0.17415, 0.6007)"],
         ["q5", "(0, 0, −1)", "(1.18425, 0.17415, 0.48085)"],
         ["q6", "(0, 1, 0)", "(1.18425, 0.2907, 0.48085)"]]

# (label, rows, column widths [in], right-aligned columns)
TABLES = [
    ("URDF chain", URDF, [0.38, 1.30, 0.70], []),
    ("DH table: Rz(θ) Tz(d) Tx(a) Rx(α)", DH, [0.38, 0.58, 0.50, 0.56, 0.36], [2, 3]),
    ("Screws (PoE), home pose", SCREW, [0.38, 0.56, 1.54], []),
]
GAP = 0.25


def set_cell(cell, text, bold=False, align=PP_ALIGN.LEFT, header=False, color=None):
    cell.margin_left = cell.margin_right = Inches(0.04)
    cell.margin_top = cell.margin_bottom = Inches(0.02)
    cell.vertical_anchor = MSO_ANCHOR.MIDDLE
    tf = cell.text_frame
    tf.word_wrap = False
    p = tf.paragraphs[0]
    p.alignment = align
    r = p.add_run()
    r.text = text
    r.font.size = Pt(FONT_PT)
    r.font.bold = bold
    if color is not None:
        r.font.color.rgb = RGBColor.from_string(color)
    # borders: rows only, no column borders, no fill
    tcPr = cell._tc.get_or_add_tcPr()
    for tag in ("lnL", "lnR", "lnT"):
        ln = etree.SubElement(tcPr, f"{{{NS}}}{tag}", w="0")
        etree.SubElement(ln, f"{{{NS}}}noFill")
    ln = etree.SubElement(tcPr, f"{{{NS}}}lnB", w="9525" if header else "6350", cap="flat", cmpd="sng", algn="ctr")
    fill = etree.SubElement(ln, f"{{{NS}}}solidFill")
    etree.SubElement(fill, f"{{{NS}}}srgbClr", val="7F7F7F" if header else "D9D9D9")
    etree.SubElement(ln, f"{{{NS}}}prstDash", val="solid")
    etree.SubElement(tcPr, f"{{{NS}}}noFill")


def add_text(slide, x, y, w, h, text, size, bold=False, color=None):
    tb = slide.shapes.add_textbox(Inches(x), Inches(y), Inches(w), Inches(h))
    tf = tb.text_frame
    tf.word_wrap = True
    tf.margin_left = tf.margin_right = Inches(0.05)
    tf.margin_top = tf.margin_bottom = Inches(0.02)
    for i, line in enumerate(text if isinstance(text, list) else [text]):
        p = tf.paragraphs[0] if i == 0 else tf.add_paragraph()
        r = p.add_run()
        r.text = line
        r.font.size = Pt(size)
        r.font.bold = bold
        if color is not None:
            r.font.color.rgb = RGBColor.from_string(color)
    return tb


prs = Presentation(SRC)
slide = prs.slides[3]
assert slide.shapes[0].text_frame.text.strip() == "Forward Kinematic models of P6R"

total_w = sum(sum(t[2]) for t in TABLES) + GAP * (len(TABLES) - 1)
x = X_LEFT
print(f"tables: total width {total_w:.2f} in, from x = {x:.2f} to {x + total_w:.2f} in")
for k, (label, rows, widths, right_cols) in enumerate(TABLES):
    w = sum(widths)
    lab = add_text(slide, x, TOP_LABEL, w + GAP - 0.05, 0.25, label, LABEL_PT, bold=True)
    lab.name = f"s4_label_{k}"
    lab.text_frame.margin_left = Inches(0.04)
    gf = slide.shapes.add_table(len(rows), len(widths), Inches(x), Inches(TOP_TABLE), Inches(w), Inches(ROW_H * len(rows)))
    gf.name = f"s4_table_{k}"
    tbl = gf.table
    tbl.first_row, tbl.horz_banding = True, False
    tbl._tbl.tblPr.find(f"{{{NS}}}tableStyleId").text = NO_STYLE
    for c, cw in enumerate(widths):
        tbl.columns[c].width = Inches(cw)
    for r, row in enumerate(rows):
        tbl.rows[r].height = Inches(ROW_H)
        for c, text in enumerate(row):
            set_cell(tbl.cell(r, c), text, bold=(r == 0 or c == 0),
                     align=PP_ALIGN.RIGHT if (c in right_cols) else PP_ALIGN.LEFT, header=(r == 0),
                     color=RED if row[0] == "x" else None)
    print(f"  {label:36s} x = {x:.2f} in, w = {w:.2f} in, rows {len(rows)}, bottom of table {TOP_TABLE + ROW_H * len(rows):.2f} in")
    x += w + GAP

# ------------------------------------------------------------------ validation paragraph under the tables
# numbers from scripts/validating.py: pairwise differences of the three P6R models
# runs: (text, bold, superscript)
VALIDATION = [("Validation. ", True, False),
              ("The three models return the same flange pose. At a test configuration "
               "they differ by less than 3 × 10", False, False), ("−16", False, True),
              (" m and 2 × 10", False, False), ("−14", False, True),
              (" deg. Over 1000 random configurations the largest difference is 1.3 × 10", False, False), ("−15", False, True),
              (" m in position and 3 × 10", False, False), ("−14", False, True),
              (" deg in orientation, which is machine precision.", False, False)]
Y_VALID = TOP_TABLE + ROW_H * 8 + 0.32
tb = slide.shapes.add_textbox(Inches(X_LEFT), Inches(Y_VALID), Inches(total_w), Inches(0.75))
tb.name = "s4_validation"
tf = tb.text_frame
tf.word_wrap = True
tf.margin_left = tf.margin_right = Inches(0.04)
tf.margin_top = tf.margin_bottom = Inches(0.02)
for text, bold, sup in VALIDATION:
    r = tf.paragraphs[0].add_run()
    r.text = text.replace(" × 10", " × 10")    # keep "1.3 × 10^-15" on one line
    r.font.size = Pt(12)                       # the deck's body size
    r.font.bold = bold
    if sup:
        r.font._element.set("baseline", "30000")
print(f"validation paragraph at y = {Y_VALID:.2f} in, width {total_w:.2f} in")

out = Path(sys.argv[1]) if len(sys.argv) > 1 else OUT
try:
    prs.save(out)
except PermissionError:
    out = out.with_name(out.stem + "_new.pptx")
    prs.save(out)
    print("the deck is open in another program: saved under a new name instead")
print(f"written {out}  ({len(prs.slides)} slides)")
