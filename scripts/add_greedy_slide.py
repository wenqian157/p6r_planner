"""Fill the empty slide 13 ("Path planning and Redundancy plane (x, s)") of P6R_presentation_complete.pptx with
the greedy vs branch-aware (DP) comparison, slow base k = 0.42 (Report section 5.6, drafts/section5_output_slow_onegreedy.txt).

Run from the repo root:  python scripts/add_greedy_slide.py [in.pptx] [out.pptx] [figures_dir]
"""
import sys
from pathlib import Path
from lxml import etree
from pptx import Presentation
from pptx.util import Inches, Pt
from pptx.enum.text import PP_ALIGN, MSO_ANCHOR
from pptx.dml.color import RGBColor

ROOT = Path(__file__).resolve().parent.parent
SRC = Path(sys.argv[1]) if len(sys.argv) > 1 else ROOT / "results" / "presentation" / "P6R_presentation_complete.pptx"
OUT = Path(sys.argv[2]) if len(sys.argv) > 2 else SRC.with_name("P6R_presentation_complete_v2.pptx")
FIG = Path(sys.argv[3]) if len(sys.argv) > 3 else ROOT / "results" / "figures"
NS = "http://schemas.openxmlformats.org/drawingml/2006/main"
NO_STYLE = "{2D5ABB26-0587-4C30-8999-92F81FD0307C}"
GREY, RED, BLUE = "595959", "D62728", "1F77B4"
X0 = 0.62


def add_text(slide, name, x, y, w, h, paras, size=11, color=None, bullets=False, space=3):
    """paras: list of paragraphs; each is a str or a list of (text, bold[, color]) runs."""
    tb = slide.shapes.add_textbox(Inches(x), Inches(y), Inches(w), Inches(h))
    tb.name = name
    tf = tb.text_frame
    tf.word_wrap = True
    tf.margin_left = tf.margin_right = Inches(0.05)
    tf.margin_top = tf.margin_bottom = Inches(0.02)
    for i, para in enumerate(paras):
        p = tf.paragraphs[0] if i == 0 else tf.add_paragraph()
        p.alignment = PP_ALIGN.LEFT
        if i:
            p.space_before = Pt(space)
        runs = [(para, False)] if isinstance(para, str) else para
        if bullets:
            pPr = p._p.get_or_add_pPr()
            pPr.set("marL", str(int(Inches(0.15))))
            pPr.set("indent", str(-int(Inches(0.15))))
            etree.SubElement(pPr, f"{{{NS}}}buChar", char="•")
        for run in runs:
            text, bold = run[0], run[1]
            c = run[2] if len(run) > 2 else color
            r = p.add_run()
            r.text = text
            r.font.name = "Arial"
            r.font.size = Pt(size)
            r.font.bold = bold
            if c:
                r.font.color.rgb = RGBColor.from_string(c)
    return tb


def add_table(slide, name, x, y, widths, rows, row_h=0.22, size=10, right=()):
    gf = slide.shapes.add_table(len(rows), len(widths), Inches(x), Inches(y), Inches(sum(widths)),
                                Inches(row_h * len(rows)))
    gf.name = name
    tbl = gf.table
    tbl._tbl.tblPr.find(f"{{{NS}}}tableStyleId").text = NO_STYLE
    for c, cw in enumerate(widths):
        tbl.columns[c].width = Inches(cw)
    for r, row in enumerate(rows):
        tbl.rows[r].height = Inches(row_h)
        for c, text in enumerate(row):
            cell = tbl.cell(r, c)
            cell.margin_left = cell.margin_right = Inches(0.04)
            cell.margin_top = cell.margin_bottom = Inches(0.01)
            cell.vertical_anchor = MSO_ANCHOR.MIDDLE
            p = cell.text_frame.paragraphs[0]
            p.alignment = PP_ALIGN.RIGHT if c in right else PP_ALIGN.LEFT
            run = p.add_run()
            run.text = text
            run.font.size = Pt(size)
            run.font.bold = (r == 0 or c == 0)
            if r == 1 and c == 0:
                run.font.color.rgb = RGBColor.from_string(RED)
            if r == 2 and c == 0:
                run.font.color.rgb = RGBColor.from_string(BLUE)
            tcPr = cell._tc.get_or_add_tcPr()
            for tag in ("lnL", "lnR", "lnT"):
                ln = etree.SubElement(tcPr, f"{{{NS}}}{tag}", w="0")
                etree.SubElement(ln, f"{{{NS}}}noFill")
            ln = etree.SubElement(tcPr, f"{{{NS}}}lnB", w="9525" if r == 0 else "6350")
            fill = etree.SubElement(ln, f"{{{NS}}}solidFill")
            etree.SubElement(fill, f"{{{NS}}}srgbClr", val="7F7F7F" if r == 0 else "D9D9D9")
            etree.SubElement(tcPr, f"{{{NS}}}noFill")


prs = Presentation(SRC)
s = prs.slides[12]
assert s.shapes[0].text_frame.text.strip().startswith("Path planning"), s.shapes[0].text_frame.text
assert len(s.shapes) == 1, "slide 13 is not empty any more"
s.shapes[0].width = Inches(9.2)          # title on one line, above the subtitle

add_text(s, "g_sub", X0, 0.98, 9.0, 0.26,
         ["Why the planner must see the whole map: branch-aware (DP) vs greedy, slow base"], size=12)
s.shapes.add_picture(str(FIG / "s5_sx_map_dp_vs_greedy_slow_onegreedy.png"),
                     Inches(0.45), Inches(1.32), Inches(6.35)).name = "g_img"
add_text(s, "g_cap", 0.45, 4.27, 6.35, 0.22,
         ["Figure. One panel per IK branch (posture). Both planners start at x = −1 m; base speed ≤ 0.42 × tool speed."],
         size=9, color=GREY)

add_text(s, "g_read_head", 6.95, 1.3, 2.85, 0.25, [[("How to read a panel", True)]], size=11)
add_text(s, "g_read", 6.95, 1.58, 2.85, 2.9, [
    [("→ s: ", True), ("progress along the rectangle (0 → 1). Dotted lines = corners.", False)],
    [("↑ x: ", True), ("base position on the rail.", False)],
    [("Green: ", True, "2CA02C"), ("this posture reaches the tool pose from this x, not singular.", False)],
    [("Grey: ", True, "7F7F7F"), ("out of reach (or singular).", False)],
    [("Red: ", True, RED), ("DP plan, the cheapest complete path, chosen seeing the whole map.", False)],
    [("Blue dashed: ", True, BLUE), ("greedy, the cheapest next step only.", False)],
    [("× ", True, BLUE), ("where greedy gets stuck: no allowed next step.", False)],
], size=9.5, space=2)

add_table(s, "g_table", X0, 4.6, [1.75, 0.45, 0.45, 0.45, 0.45, 0.45, 0.45], [
    ["k = base / tool speed", "1.25", "0.94", "0.62", "0.50", "0.42", "0.31"],
    ["DP (whole map)", "6/8", "6/8", "5/8", "3/8", "3/8", "0/8"],
    ["greedy (one step)", "6/8", "6/8", "3/8", "3/8", "0/8", "0/8"],
], row_h=0.22, size=9, right=(1, 2, 3, 4, 5, 6))
add_text(s, "g_take", 5.15, 4.55, 4.65, 0.75, [
    [("Same cost and limits, only the lookahead differs. ", True),
     ("A slow base must start climbing early to pass the grey wedge at s ≈ 0.5; greedy can't see it coming. "
      "Possible only because the analytical IK gives all branches everywhere.", False)]], size=9.5)

SCRIPT = (
    "[~1:15] Both planners solve the same problem on the map you just saw: choose the base position x at every point "
    "of the path, on one IK branch, with the same limits and the same cost, joint motion plus base motion. The only "
    "difference: the DP, in red, sees the whole map and picks the cheapest complete path. Greedy, in blue, only takes "
    "the cheapest next step.\n"
    "With a fast base both finish the same six branches; greedy is just 10 to 46 percent more expensive. Here the "
    "base is slow, at most 0.42 times the tool speed. On the first side the tool moves along +x and the green band "
    "climbs with it. To pass above the grey wedge at s = 0.5, the base must climb at full speed from the start. The "
    "DP sees the wedge and does that, on three branches. Greedy sometimes takes a slightly smaller, cheaper step, "
    "arrives a few centimetres too low, and gets stuck at the tip: the blue cross. Greedy finishes zero branches.\n"
    "So the lookahead matters, and it is only possible because the analytical IK gives every branch at every point.")

STUDY = """

----- STUDY NOTES (not to read out) -----
WHAT EACH PANEL IS: one IK branch = one arm posture family (S shoulder side, W wrist flip, E elbow up/down). The planner must stay on the same branch for the whole path (switching would mean passing through a singularity).
AXES: s = path progress around the 4.4 m rectangle (0 = start, 1 = back at start). Dotted vertical lines = corners at s = 0.36, 0.50, 0.86. On the first long side (s 0 to 0.36) the tool moves 1.6 m along +x, so the green band climbs; on the return it comes back down. x = base position, -1 to +1 m.
DOTS: green = the branch has an IK root at (s, x) and sigma_min(J_red) >= 0.03. Grey below the band = base too far behind the tool (out of reach). Grey above = base too far ahead. The grey wedge with its tip at s = 0.5 is the moment the tool is at the far corner: on the three red branches the base must be at about x = -0.1 m or higher there.
LINES: every line starts at x = -1 m, s = 0 (bottom left). Slope of a line = how fast the base moves. Slow base: slope at most 0.42 (1.7 cm of base per 4 cm of path). Allowed steps: base <= that, every joint <= 0.5 rad, no joint crossing +-180 deg.
RED (DP) only appears on 3 branches: S+ W- E+, S+ W- E-, S- W+ E-. On the other 5 even the DP has no complete plan with a slow base.
BLUE X, branch by branch (greedy, slow base):
  S+ W+ E+: stops at s = 0.31, the green band climbed away faster than the base can follow (out of reach).
  S+ W+ E-: stops at s = 0.10, joint q4 would have to cross +-180 deg (our joint limit). Line so short you mostly see only the x.
  S+ W- E+: stops at s = 0.50, at the tip of the wedge, a few cm below where the DP passes.
  S+ W- E-: same, s = 0.50.
  S- W+ E+: stops at s = 0.04, q2 would have to cross +-180 deg. Again almost only the x is visible.
  S- W+ E-: stops at s = 0.49, at the wedge tip.
  S- W- E+: stops at s = 0.26 although the dots around are green: q2 would cross +-180 deg. The seam is invisible in the dots, it acts between them.
  S- W- E-: stops at s = 0.31, band out of reach (like S+ W+ E+).
WHY ONLY AN X SOMETIMES: the greedy line is there, but it stops after 1-3 steps (seam), so the x covers it.
KEY DIFFERENCE: on the 3 branches with a red line, red and blue almost overlap until s ~ 0.45. Greedy sometimes takes a slightly smaller base step because it is cheaper right now; by s = 0.5 it is a few cm too low and there is no green neighbour it can reach. The DP knows the wedge is coming and stays at the top of what the base can reach.
SECOND GREEDY RULE: picking the safest step (max sigma_min) instead of the cheapest fails the same way (0/8), so it is the missing lookahead, not the rule.
FAST BASE (backup slide): both complete 6/8; greedy costs 10-46 % more; on the best branch it rides the reach edge (sigma_min 0.033 vs 0.172).
WHY ANALYTICAL IK MATTERS: it gives all 8 branches at every dot, so the whole map exists and the DP can search it. A numerical IK gives one root near its seed and behaves like a local (greedy) rule.
"""
s.notes_slide.notes_text_frame.text = SCRIPT + STUDY

prs.save(OUT)
print("written", OUT)
