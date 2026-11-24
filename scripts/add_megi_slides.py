"""Megi's part of P6R_presentation.pptx: Analytical IK solver and Reachable-position maps.

Fills the empty slides 7-10 and inserts three more after them (7 slides in total, before "Path planning").
Content and numbers: results/Report_merged_v2.md, sections 3.1-3.9 and 4.1-4.4 (same numbers as Report_final).
Same look as Wenqian's slides: bold title kept, 12 pt subtitle at y = 0.98 in, Arial body, Arial 10 grey captions,
row-only table borders.

Run from the repo root:  python scripts/add_megi_slides.py [in.pptx] [out.pptx] [figures_dir]
"""
import copy
import sys
import tempfile
from pathlib import Path
from lxml import etree
from PIL import Image
from pptx import Presentation
from pptx.util import Inches, Pt
from pptx.enum.text import PP_ALIGN, MSO_ANCHOR
from pptx.dml.color import RGBColor

ROOT = Path(__file__).resolve().parent.parent
SRC = Path(sys.argv[1]) if len(sys.argv) > 1 else ROOT / "results" / "presentation" / "P6R_presentation.pptx"
OUT = Path(sys.argv[2]) if len(sys.argv) > 2 else SRC.with_name("P6R_presentation_with_Megi.pptx")
FIG = Path(sys.argv[3]) if len(sys.argv) > 3 else ROOT / "results" / "figures"
NS = "http://schemas.openxmlformats.org/drawingml/2006/main"
NO_STYLE = "{2D5ABB26-0587-4C30-8999-92F81FD0307C}"
GREY = "595959"
X0 = 0.62

# crops of the Module 3 screenshot: the 8 single-solution views
TMP = Path(tempfile.mkdtemp())
_m3 = Image.open(FIG / "module3_ik_solutions.png")
_m3.crop((0, 672, 2000, 1358)).save(TMP / "m3_views.png")


# ------------------------------------------------------------------ helpers
def set_cell(cell, text, bold=False, header=False, size=10, align=PP_ALIGN.LEFT, color=None):
    cell.margin_left = cell.margin_right = Inches(0.04)
    cell.margin_top = cell.margin_bottom = Inches(0.01)
    cell.vertical_anchor = MSO_ANCHOR.MIDDLE
    tf = cell.text_frame
    tf.word_wrap = True
    tf.paragraphs[0].alignment = align
    r = tf.paragraphs[0].add_run()
    r.text = text
    r.font.size = Pt(size)
    r.font.bold = bold
    if color:
        r.font.color.rgb = RGBColor.from_string(color)
    tcPr = cell._tc.get_or_add_tcPr()
    for tag in ("lnL", "lnR", "lnT"):
        ln = etree.SubElement(tcPr, f"{{{NS}}}{tag}", w="0")
        etree.SubElement(ln, f"{{{NS}}}noFill")
    ln = etree.SubElement(tcPr, f"{{{NS}}}lnB", w="9525" if header else "6350", cap="flat", cmpd="sng", algn="ctr")
    fill = etree.SubElement(ln, f"{{{NS}}}solidFill")
    etree.SubElement(fill, f"{{{NS}}}srgbClr", val="7F7F7F" if header else "D9D9D9")
    etree.SubElement(ln, f"{{{NS}}}prstDash", val="solid")
    etree.SubElement(tcPr, f"{{{NS}}}noFill")


def add_table(slide, name, x, y, widths, rows, row_h=0.24, size=10, right=(), highlight=None):
    gf = slide.shapes.add_table(len(rows), len(widths), Inches(x), Inches(y), Inches(sum(widths)),
                                Inches(row_h * len(rows)))
    gf.name = name
    tbl = gf.table
    tbl.first_row, tbl.horz_banding = True, False
    tbl._tbl.tblPr.find(f"{{{NS}}}tableStyleId").text = NO_STYLE
    for c, cw in enumerate(widths):
        tbl.columns[c].width = Inches(cw)
    for r, row in enumerate(rows):
        tbl.rows[r].height = Inches(row_h)
        for c, text in enumerate(row):
            set_cell(tbl.cell(r, c), text, bold=(r == 0), header=(r == 0), size=size,
                     align=PP_ALIGN.RIGHT if c in right else PP_ALIGN.LEFT,
                     color="C00000" if (highlight and r in highlight) else None)
    return gf


def add_text(slide, name, x, y, w, h, paras, size=12, color=None, bullets=False, space=4):
    """paras: list of paragraphs; each is a str or a list of (text, bold) runs."""
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
            pPr.set("marL", str(int(Inches(0.17))))
            pPr.set("indent", str(-int(Inches(0.17))))
            etree.SubElement(pPr, f"{{{NS}}}buChar", char="•")
        for text, bold in runs:
            r = p.add_run()
            r.text = text
            r.font.name = "Arial"
            r.font.size = Pt(size)
            r.font.bold = bold
            if color:
                r.font.color.rgb = RGBColor.from_string(color)
    return tb


def subtitle(slide, text):
    tb = slide.shapes.add_textbox(Inches(X0), Inches(0.98), Inches(9.0), Inches(0.24))
    tb.name = "megi_subtitle"
    tf = tb.text_frame
    tf.word_wrap = True
    tf.margin_left = tf.margin_right = Inches(0.05)
    tf.margin_top = tf.margin_bottom = Inches(0.02)
    tf.paragraphs[0].alignment = PP_ALIGN.LEFT
    r = tf.paragraphs[0].add_run()
    r.text = text
    r.font.size = Pt(12)


def picture(slide, name, file, x, y, w=None, h=None):
    path = file if Path(file).is_absolute() else FIG / file
    pic = slide.shapes.add_picture(str(path), Inches(x), Inches(y),
                                   Inches(w) if w else None, Inches(h) if h else None)
    pic.name = name
    return pic


def caption(slide, x, y, w, text):
    add_text(slide, "megi_caption", x, y, w, 0.22, [text], size=10, color=GREY)


def notes(slide, text):
    slide.notes_slide.notes_text_frame.text = text


# ------------------------------------------------------------------ deck layout
# 10 min talk, 5 min per student: Megi's part is 4 main slides (7-10, ~1:15 each). The detailed slides go to a
# backup section at the end of the deck, for the 10 min of questions.
prs = Presentation(SRC)
titles = {7: "Analytical IK solver", 8: "Analytical IK solver", 9: "Reachable-position maps", 10: "Reachable-position maps"}
for i, t in titles.items():
    s = prs.slides[i - 1]
    assert s.shapes[0].text_frame.text.strip() == t, (i, s.shapes[0].text_frame.text)
    assert len(s.shapes) == 1, f"slide {i} is not empty any more"
M = {k: prs.slides[k + 5] for k in (1, 2, 3, 4)}          # main slides 7, 8, 9, 10
template = prs.slides[6]


def backup_slide(title):
    s = prs.slides.add_slide(template.slide_layout)
    s.shapes._spTree.append(copy.deepcopy(template.shapes[0]._element))
    s.shapes[0].text_frame.paragraphs[0].runs[0].text = title
    return s


divider = backup_slide("Backup slides")
add_text(divider, "backup_note", X0, 1.3, 9.0, 0.6,
         ["Details for the questions: IK formulas and labels, the 8 solutions, validation, base coverage, "
          "orientation and singular cells."], size=12, color=GREY)
S = {7: backup_slide("Backup: IK solver, the three groups"),
     8: backup_slide("Backup: IK branches and labels"),
     9: backup_slide("Backup: all 8 IK solutions of one pose"),
     10: backup_slide("Backup: IK validation"),
     11: M[3],
     12: backup_slide("Backup: moving the base"),
     13: backup_slide("Backup: orientation and singular cells")}

# ================================================================== 7: problem and the three groups
s = S[7]
subtitle(s, "Fixed x: the P6R is a UR12e with a shifted target, solved in three groups")
add_text(s, "s7_intro", X0, 1.30, 9.0, 0.5, [
    [("⁰T₆ = (Tₓ(x) T_mount T_base)⁻¹ T* T_tool⁻¹. ", True),
     ("x no longer appears in the formulas: it only changes the target the arm sees. The UR wrist is offset "
      "(d5, d6 ≠ 0), so axes 4, 5, 6 do not meet and Pieper's split does not apply. Axes 2, 3, 4 are parallel: "
      "³J₅ is block triangular, and its three blocks are the three groups.", False)]], size=11)
add_table(s, "s7_table", X0 + 0.04, 2.05, [0.95, 1.55, 3.6, 1.1, 1.75], [
    ["group", "quantity used", "solution", "no root if", "2 roots merge (label)"],
    ["1: q1", "position of O5", "q1 = atan2(p5y, p5x) ± arccos(d4/r) + π/2", "r < d4", "r = d4, ρ = 0  (S±)"],
    ["2: q5, q6", "orientation, given q1", "cos q5 = (p·n − d4)/d6,  n = (sin q1, −cos q1, 0)", "|cos q5| > 1",
     "sin q5 = 0  (W±)"],
    ["3: q2, q3, q4", "planar 3R in arm plane", "cos q3 = (|p13|² − a2² − a3²)/(2 a2 a3)", "|cos q3| > 1",
     "sin q3 = 0  (E±)"],
], row_h=0.27, size=10)
add_text(s, "s7_details", X0, 3.22, 5.2, 2.1, [
    [("Group 1. ", True), ("O5 = p − d6 z6 is fixed by the target. The wrist sits d4 beside the arm plane, so the "
                           "plane is a tangent from axis 1 to a circle of radius d4 around O5: two tangents.", False)],
    [("Group 2. ", True), ("q6 = atan2(−sgn(s5)(y6·n), sgn(s5)(x6·n)). At sin q5 = 0 only q4 + q6 is defined: "
                           "the solver sets q6 = 0.", False)],
    [("Group 3. ", True), ("Peel off A1, A5, A6: ¹T₄ = A2 A3 A4. Law of cosines for q3, then q2; q4 is the "
                           "remaining in-plane angle.", False)],
], size=10, bullets=True, space=4)
add_text(s, "s7_example", 6.05, 3.22, 3.65, 2.0, [
    [("Example (pose of 1.4, x = 0.5 m)", True)],
    "O5 = (−0.877, −0.332, 0.426) m,  r = 0.9375 m",
    "atan2 = −159.3°,  arccos(d4/r) = 79.3°",
    [("q1 = 10.0° (S+)  or  −148.6° (S−)", True)],
], size=10, space=3)
notes(s, "With the base at x, the world target is moved into frame 0 of the arm. From there it is a pure 6R problem: x "
         "does not appear in the formulas any more, it only changes the target the arm sees.\n"
         "The UR wrist is offset, so axes 4, 5 and 6 do not meet in one point and the classic Pieper split does not "
         "apply. What the UR has instead is three parallel axes, 2, 3 and 4. That makes the Jacobian block triangular, "
         "and the three blocks are the three groups of the IK, solved in order.\n"
         "Group 1: O5, where the last two wrist axes cross, is fixed by the target. The wrist sits d4 to the side of the "
         "arm plane, so the plane is a tangent from axis 1 to a circle of radius d4: two tangents, shoulder plus or "
         "minus. No root if O5 is closer than d4 to axis 1. In the example, q1 is 10 degrees or minus 148.6.\n"
         "Group 2: with q1 known, the normal n of the arm plane is known, and q5 comes from the distance of the flange "
         "to that plane: two roots, the wrist flip. q6 follows. At sin q5 = 0, q4 and q6 turn about the same axis, so "
         "we set q6 = 0.\n"
         "Group 3: peel off the outer transforms, and what is left is a planar 2R for q2 and q3 plus q4 as the "
         "remaining angle: two roots, elbow plus or minus.")

# ================================================================== 8: branches, labels, aspects
s = S[8]
subtitle(s, "8 branches, labelled by the signs of det J_red = a2 a3 sin q3 sin q5 ρ")
add_text(s, "s8_left", X0, 1.30, 4.55, 4.0, [
    [("At most 2 × 2 × 2 = 8 branches. ", True), ("A branch is dropped when an arccos argument leaves [−1, 1].", False)],
    [("Angles wrapped to (−π, π]: ", True), ("with limits ±π each branch gives exactly one joint vector. On a path the "
                                            "±π seam acts as a joint limit (section 5).", False)],
    [("Label = sign triple ", True), ("(sign ρ, sign sin q5, sign sin q3) = (S, W, E).", False)],
    [("Why these names without a shoulder–wrist split? ", True),
     ("The three binary choices are made at shoulder pan q1, elbow q3 and wrist 2 q5. S: O5 in front of / behind "
      "the shoulder axis. E: bend of the planar 2R (its \"up\" flips with S). W: computed from the flange position, "
      "the coupling a spherical wrist would remove.", False)],
], size=10.5, bullets=True, space=5)
add_table(s, "s8_merge", 5.35, 1.32, [1.95, 1.2, 1.15], [
    ["root formula", "roots merge", "det J_red factor"],
    ["q1 = … ± arccos(d4/r)", "r = d4", "ρ = 0 (shoulder)"],
    ["q5 = ± arccos(…)", "cos q5 = ±1", "sin q5 = 0 (wrist)"],
    ["q3 = ± arccos(…)", "cos q3 = ±1", "sin q3 = 0 (elbow)"],
], row_h=0.25, size=9)
add_text(s, "s8_right", 5.35, 2.42, 4.35, 2.9, [
    [("IK and det J_red are the same three conditions: ", True),
     ("branches appear and disappear in pairs, every count is even.", False)],
    [("Not cuspidal. ", True), ("The 3 factors cut joint space into 8 aspects with exactly one solution each "
                               "(0 mismatches in 14 426 solutions).", False)],
    [("Label persistence: ", True), ("100 random joint paths × 300 steps: same label in 99.72 % of 194 256 steps. "
                                    "The 545 switches are next to a singular surface: shoulder 68.4 %, wrist 23.9 %, "
                                    "elbow 7.7 %.", False)],
    [("→ fix the branch at the start; follow it by continuity and monitor σ_min.", True)],
], size=10.5, bullets=True, space=5)
notes(s, "Three groups with two roots each give at most 8 branches. We wrap every angle to plus or minus pi, so with "
         "the project limits each branch is exactly one joint vector. On a path that pi seam acts as a joint limit, "
         "which comes back in the planning part.\n"
         "Each branch is labelled by a sign triple: the signs of the three factors of det J_red. The UR has no "
         "shoulder-wrist decoupling, but it still has exactly three binary choices, made at the shoulder pan, the "
         "elbow and wrist 2, so we keep the names; they are just the signs.\n"
         "The table on the right is the key point: each group has one arccos, and its two roots merge exactly where one "
         "factor of det J_red is zero. So the IK and the singularities are the same three conditions, and branch counts "
         "are always even.\n"
         "The robot is not cuspidal: each of the 8 aspects holds exactly one solution, checked on 14 426 solutions. "
         "So a label can only change by crossing a singularity. Along random joint paths the label is kept in 99.7 % of "
         "the steps, and every switch is next to a singular surface, mostly the shoulder. For planning: fix the branch, "
         "follow it by continuity and watch sigma_min.")

# ================================================================== 9: all 8 solutions of one pose
s = S[9]
subtitle(s, "All 8 solutions of one flange pose (x = 0.5 m, q0 = [10, −60, 90, −120, −90, 30]°)")
add_table(s, "s9_table", X0 + 0.04, 1.30, [1.35, 0.6, 0.6, 0.55, 0.62, 0.48, 0.62, 0.85, 1.15], [
    ["branch", "q1", "q2", "q3", "q4", "q5", "q6", "det J_red", "above deck"],
    ["1  S+ W+ E+", "10.0", "−37.7", "50.9", "76.8", "90", "−150.0", "−0.250", "+19 cm"],
    ["2  S+ W+ E−", "10.0", "11.3", "−50.9", "129.6", "90", "−150.0", "0.250", "+11 cm"],
    ["3  S+ W− E+ (q0)", "10.0", "−60.0", "90.0", "−120.0", "−90", "30.0", "0.323", "+19 cm"],
    ["4  S+ W− E−", "10.0", "26.0", "−90.0", "−26.0", "−90", "30.0", "−0.323", "−1 cm"],
    ["5  S− W+ E+", "−148.6", "154.0", "90.0", "−154.0", "90", "51.4", "0.323", "−4 cm"],
    ["6  S− W+ E−", "−148.6", "−120.0", "−90.0", "−60.0", "90", "51.4", "−0.323", "+19 cm"],
    ["7  S− W− E+", "−148.6", "168.7", "50.9", "50.4", "−90", "−128.6", "−0.250", "+10 cm"],
    ["8  S− W− E−", "−148.6", "−142.3", "−50.9", "103.2", "−90", "−128.6", "0.250", "+19 cm"],
], row_h=0.215, size=9, right=(1, 2, 3, 4, 5, 6, 7, 8), highlight=(4, 5))
add_text(s, "s9_note", X0, 3.32, 9.0, 0.42, [
    [("Same pose, 8 different postures; |FK − T| ≤ 1.9·10⁻¹⁵ for all. ", False),
     ("Collisions are not part of the IK: ", True),
     ("with link radii 6–7.5 cm, solutions 4 and 5 pass through the base, 2 and 7 clear it by a few cm.", False)]],
    size=10)
picture(s, "s9_img", TMP / "m3_views.png", X0, 3.78, w=4.85)
caption(s, X0 + 4.95, 4.6, 3.9, "Figure. The 8 solutions in the course simulator (Module 3), numbered as in the "
                                "table. Angles in degrees.")
notes(s, "This is one flange pose, with the base at x = 0.5 m, and all 8 solutions. The shoulder sides differ by about "
         "159 degrees in q1; within each side the wrist flips and the elbow goes up or down. det J_red has the same "
         "magnitude in pairs, with the sign telling the aspect. All 8 reach the pose to machine precision.\n"
         "The last column matters physically: the IK does not know about the base. Approximating the arm by its joint "
         "origins and a link radius of 6 to 7.5 cm, solutions 4 and 5 go through the UMR deck and 2 and 7 clear it by "
         "only a few centimetres. So all 8 are valid IK solutions, but only about half are usable from this base "
         "position. A collision check is outside the project.")

# ================================================================== 10: validation and the base position
s = S[10]
subtitle(s, "Validation, and the base position selects the branches")
add_table(s, "s10_val", X0 + 0.04, 1.30, [2.75, 2.0], [
    ["check (2000 random configurations, ±π)", "result"],
    ["input recovered, max |q_true − branch|", "6.4·10⁻¹¹ rad"],
    ["every branch reaches the pose, max |FK − T|", "3.4·10⁻¹²"],
    ["branches per pose  {2 : 4 : 6 : 8}", "51 : 268 : 98 : 1583"],
    ["label vs sign triple read from q", "0 / 14 426 mismatches"],
    ["vectorised: 200 000 samples", "2.4·10⁻¹⁰ rad, ≈ 59 000 poses/s"],
    ["Module 3 (independent FK), 8 solutions", "0.055 mm*, 0.0000°"],
], row_h=0.25, size=9.5)
add_text(s, "s10_foot", X0, 3.1, 4.8, 0.45, [
    "* the target was typed with 4 decimals; its rounding is exactly 0.055 mm, the same for every row: not an IK error."],
    size=9, color=GREY)
picture(s, "s10_img_x", "ik_branches_vs_x.png", X0 + 0.9, 3.62, w=5.0)
add_text(s, "s10_x", 5.75, 1.30, 3.95, 1.9, [
    [("Same world target, base slid along x:", True)],
    "x < 0.13 m: 0 branches (out of reach)",
    "0.13 ≤ x < 0.38 m: 4 branches (S+ W− E±, S− W+ E±)",
    "x ≥ 0.38 m: all 8",
    [("Branches come and go in elbow pairs: fixing x decides which branches exist.", True)],
], size=10, space=3)
notes(s, "Validation. On 2000 random configurations the input is always recovered, and every returned branch reaches "
         "the pose, both to around 1e-11. The branch counts are always even: 2, 4, 6 or 8, never odd. The label read "
         "back from the joint vector matches its branch every time. The vectorised version gives the same result on "
         "200 000 samples and runs at about 59 000 poses per second, which is what makes the maps and the planner cheap.\n"
         "As an independent check, we loaded the 8 solutions into the course simulator, Module 3, which has its own FK. "
         "All show 0.055 mm, which is exactly the rounding of the target we typed in, and zero orientation error.\n"
         "Bottom: the same world target while the base slides. Below 0.13 m nothing, then four branches, above 0.38 m "
         "all eight. So the base position does not just shift the arm, it decides which branches exist. The maps "
         "extend this from one pose to a whole grid.")

# ================================================================== 11: branch-count map, base fixed
s = S[11]
subtitle(s, "Branch-count map, base fixed: one IK call per grid point")
add_text(s, "s11_setup", X0, 1.28, 9.0, 0.45, [
    [("Setup: ", True), ("tool axis pointing down, flange at z = 0.9 m (shoulder axis at 0.60 m), grid 0.02 m over "
                         "[−2.4, 2.4] × [−1.4, 1.4] m = 33 981 IK calls per map. Colour = number of branches with a root. "
                         "Real UR12e limits (±2π, elbow ±π) remove no branch.", False)]], size=10)
picture(s, "s11_img", "map_down_branches_x0.png", X0, 1.80, w=5.6)
caption(s, X0, 4.33, 5.6, "Figure. Per-branch map, tool down, x = 0 (box: UMR footprint).")
add_text(s, "s11_head", 6.4, 1.80, 3.35, 0.3, [[("Borders = singular families of 2.2", True)]], size=11)
add_text(s, "s11_points", 6.4, 2.12, 3.35, 3.2, [
    [("Inner hole r < d4 = 0.174 m (0): ", True), ("shoulder, O5 cannot come closer to axis 1, no q1 root.", False)],
    [("Outer edge 1.24 m (0): ", True), ("elbow, planar 2R fully stretched, |cos q3| > 1.", False)],
    [("Disc < 1.0 m (8), ring 1.0–1.24 m (4): ", True),
     ("tool vertical ⇒ q5 = ±90°, the wrist link d5 points away from or towards axis 1. Only the outward wrist "
      "(S+ W− E±, S− W+ E±) reaches the ring: Δr ≈ 2 d5 = 0.24 m.", False)],
    [("No wrist family: ", True), ("tool vertical, n horizontal ⇒ cos q5 = 0 everywhere.", False)],
], size=10, bullets=True, space=4)
add_text(s, "s11_bottom", X0, 4.62, 5.6, 0.6, [
    "Counts change only in pairs. Each branch is an annulus between the shoulder hole and its elbow edge."],
    size=10)
notes(s, "[~1:15] For every grid point at z = 0.9 m, with the tool pointing down, we run the analytical IK once and count the "
         "branches. That is about 34 000 IK calls per map. With the real UR12e limits no branch is removed, so this is "
         "the geometric count.\n"
         "Every border is one of the three singular families from the Jacobian part. The inner hole of radius d4 is the "
         "shoulder family: O5 cannot get closer to axis 1 than d4. The outer edge at 1.24 m is the elbow, fully "
         "stretched. In between, with a vertical tool q5 is plus or minus 90 degrees, so the wrist link points either "
         "outward or inward; only the outward choice reaches the outer ring, and the two radii differ by two times d5. "
         "That is why the disc has 8 branches and the ring only 4. And with the tool vertical there is no wrist "
         "singularity at all, cos q5 is zero everywhere.")

# ================================================================== 12: moving the base
s = S[12]
subtitle(s, "Moving the base translates the map; the overlay shows where x is forced")
picture(s, "s12_img", "map_down_overlay.png", X0, 1.28, w=4.3)
caption(s, X0, 4.33, 4.3, "Figure. Branch count for x = −1, 0, +1 m and RGB overlay (box: task strip).")
add_text(s, "s12_shift", 5.15, 1.28, 4.6, 1.15, [
    [("Pure translation: ", True), ("the maps at x = ±1 m equal the x = 0 map shifted by ±1 m (exact). Same counts at "
                                   "every x: 0 IK 22 129, 4 IK 4 176, 8 IK 7 676 cells; reach 0.179–1.241 m from axis 1.",
                                   False)],
    [("The slide adds reach along x only. ", True), ("For a fixed target it changes which region the target falls in.",
                                                      False)],
], size=10, bullets=True, space=4)
add_table(s, "s12_cov", 5.2, 2.6, [3.0, 1.2], [
    ["task strip x ∈ [−1.5, 1.5], y ∈ [0.5, 0.9] m", "covered"],
    ["base at x = −1 m", "50.8 %"],
    ["base at x = 0 m", "67.5 %"],
    ["base at x = +1 m", "50.1 %"],
    ["union of the three", "100.0 %"],
    ["covered by one x only (x forced)", "34.3 %"],
], row_h=0.24, size=10, right=(1,))
add_text(s, "s12_concl", 5.15, 4.13, 4.6, 1.1, [
    [("No single base position covers the strip; a third of it is covered by one x only. ", True),
     ("Along a path the base has to move. The 1.6 m rectangle of section 5 fits a parked base at x = 0, "
      "but not the start position x = −1 m.", False)]], size=10)
notes(s, "Moving the base does not change the shape of the map at all, it translates it. We checked: the shifted map is "
         "identical cell by cell, and the counts are the same for every x. So the slide only adds reach along x.\n"
         "The useful information is in the overlay. On a generic 3 m task strip, one base position covers at most "
         "two thirds, minus one and plus one only half each. The three together cover everything, but a third of the "
         "strip is reachable from one base position only: there, the base position is forced. That is exactly what the "
         "planner has to find along a path. For the rectangle of section 5, a parked base at x = 0 works, but the start "
         "position x = minus 1 does not.")

# ================================================================== 13: orientation and which singular cells the slide rescues
s = S[13]
subtitle(s, "Flange orientation, and which singular cells the slide rescues (J_red vs J)")
picture(s, "s13_tilt", "reach_map_x0_tilt.png", X0, 1.28, w=5.0)
caption(s, X0, 2.62, 5.0, "Figure. Branch count at x = 0, tool tilted 0, 30, 60, 90° towards +x.")
add_text(s, "s13_orient", 5.75, 1.28, 4.0, 1.6, [
    [("Tilting breaks the symmetry: ", True), ("the 4-branch ring narrows and almost disappears at 90°; the d5 offset "
                                             "is no longer horizontal.", False)],
    [("Tool horizontal (+Y): ", True), ("the wrist family appears as thin wedges (tool ∥ axes 2–4). Singular area "
                                       "0.26 vs 0.07 m² per branch.", False)],
], size=10, bullets=True, space=4)
add_table(s, "s13_rescue", X0 + 0.04, 2.98, [0.85, 1.75, 1.95], [
    ["family", "tool down: cells, rescued", "tool horizontal (+Y)"],
    ["shoulder", "416 cells, 93.3 %", "438 cells, 90.0 %"],
    ["elbow", "960 cells, 88.3 %", "992 cells, 95.6 %"],
    ["wrist", "0 cells", "3960 cells, 12.5 %"],
], row_h=0.24, size=10, highlight=(3,))
add_text(s, "s13_rescue_txt", X0, 4.0, 4.6, 1.3, [
    [("Singular cell: σ_min(J_red) < 0.05; rescued: σ_min(J) ≥ 0.05. ", False),
     ("Map version of type A / type B: ", True),
     ("the elbow loses a radial translation, which the slide supplies along ±X but not ±Y. The wrist loses a "
      "rotation, which a prismatic joint cannot give.", False)]], size=10)
picture(s, "s13_sigma", "map_down_sigma_2.png", 5.4, 3.05, w=4.35)
caption(s, 5.4, 4.88, 4.35, "Figure. S+ W− E+, tool down: σ_min(J_red) with x fixed vs σ_min(J) with the slide free.")
notes(s, "Orientation. Tilting the tool breaks the rotational symmetry: the four-branch ring narrows, and with the tool "
         "horizontal it has almost disappeared. The two reach classes only exist because the wrist offset lies in the "
         "horizontal plane when the tool points down. With a horizontal tool the third family, the wrist, appears as thin "
         "wedges, and the singular area is about four times larger.\n"
         "Then we asked, cell by cell: where the arm alone is singular, is the whole body with the slide regular? For "
         "shoulder and elbow cells, 88 to 96 percent are rescued. On the right you see why: near the elbow edge the arm "
         "loses a radial translation; the slide gives it back along plus-minus X, the bright lobes, but not along Y. "
         "For the wrist only 12.5 percent: the wrist loses a rotation, and a prismatic joint cannot supply a rotation. "
         "That is the type A / type B result from the Jacobian slide, now on the whole map.\n"
         "This leads to the path planning: Wenqian.")


# ================================================================== MAIN 1 (slide 7): the solver in one slide
s = M[1]
subtitle(s, "Fixed x: a 6R problem, solved in three groups that match det J_red")
add_text(s, "m1_intro", X0, 1.30, 9.0, 0.45, [
    [("The base only shifts the target: ", True),
     ("⁰T₆ = (Tₓ(x) T_mount T_base)⁻¹ T* T_tool⁻¹. The UR wrist is offset, so no spherical-wrist split; "
      "axes 2, 3, 4 are parallel, so the IK follows the three blocks of ³J₅.", False)]], size=11)
add_table(s, "m1_table", X0 + 0.04, 1.95, [1.2, 3.75, 1.5, 2.4], [
    ["group", "two roots", "merge when", "label = sign of"],
    ["1: q1", "q1 = atan2(p5y, p5x) ± arccos(d4/r) + π/2", "r = d4", "ρ  (shoulder S±)"],
    ["2: q5, q6", "cos q5 = (p·n − d4)/d6", "cos q5 = ±1", "sin q5  (wrist W±)"],
    ["3: q2, q3, q4", "cos q3 = (|p13|² − a2² − a3²)/(2 a2 a3)", "cos q3 = ±1", "sin q3  (elbow E±)"],
], row_h=0.28, size=10.5)
add_text(s, "m1_points", X0, 3.2, 9.0, 2.1, [
    [("At most 2 × 2 × 2 = 8 branches, ", True), ("labelled (S, W, E).", False)],
    [("The IK and det J_red = a2 a3 sin q3 sin q5 ρ are the same three conditions: ", True),
     ("roots merge exactly on a singular surface, so branches appear in pairs.", False)],
    [("Not cuspidal: ", True), ("one solution per aspect (14 426 solutions checked). A branch is kept along a path "
                               "until it meets a singularity (99.7 % of 194 256 steps).", False)],
], size=12, bullets=True, space=6)
notes(s, "[~1:15] With the base at x, the base only shifts the target, so the problem is a 6R IK. The UR wrist is "
         "offset, so the classic wrist split does not apply. But axes 2, 3 and 4 are parallel, and the IK follows the "
         "three blocks of the Jacobian. Group 1 gives q1, two tangents: shoulder plus or minus. Group 2 gives q5, the "
         "wrist flip. Group 3 is a planar 2R: elbow up or down. So at most 8 branches.\n"
         "The key point: each group has one arccos, and its two roots merge exactly where one factor of det J_red is "
         "zero. So the IK and the singularities are the same three conditions, and we label each branch by the three "
         "signs. The robot is not cuspidal, one solution per aspect, so a branch keeps its label until it hits a "
         "singularity. That is what the planner uses later.")

# ================================================================== MAIN 2 (slide 8): 8 solutions, validation, base selects branches
s = M[2]
subtitle(s, "8 solutions of one pose; the base position decides which branches exist")
picture(s, "m2_img_m3", TMP / "m3_views.png", X0, 1.30, w=5.3)
caption(s, X0, 3.14, 5.3, "Figure. The 8 solutions in the course simulator (Module 3), x = 0.5 m.")
add_text(s, "m2_val", 6.1, 1.30, 3.65, 1.95, [
    [("Validation", True)],
    "2000 random poses: input recovered to 6.4·10⁻¹¹ rad, all branches reach the pose.",
    "Module 3: all 8 match (0.000°).",
    "Vectorised: ≈ 59 000 poses/s.",
    [("Collisions are not in the IK: ", True), ("solutions 4 and 5 pass through the base.", False)],
], size=10.5, space=3)
picture(s, "m2_img_x", "ik_branches_vs_x.png", X0, 3.55, w=5.3)
add_text(s, "m2_x", 6.1, 3.65, 3.65, 1.6, [
    [("Same world target, base slid along x:", True)],
    "x < 0.13 m: none    0.13–0.38 m: 4    > 0.38 m: all 8",
    [("Branches come and go in elbow pairs: x selects the branches.", True)],
], size=10.5, space=4)
notes(s, "[~1:15] Here is one tool pose and its 8 solutions, loaded into the course simulator. Same pose, eight "
         "different postures. We validated the solver on 2000 random configurations: the input is always recovered "
         "and every branch reaches the pose, to 1e-11. The simulator, with its own forward kinematics, agrees. The "
         "vectorised version runs about 59 000 poses per second, which is what makes the maps cheap. Note that the IK "
         "does not know about collisions: two of these go through the base.\n"
         "Bottom: the same world target while the base slides. Below 0.13 m nothing, then four branches, above 0.38 m "
         "all eight. So the base position does not just move the arm, it decides which branches exist.")

# ================================================================== MAIN 4 (slide 10): moving the base and orientation
s = M[4]
subtitle(s, "Moving the base translates the map; orientation changes its shape")
picture(s, "m4_img_overlay", "map_down_overlay.png", X0, 1.28, w=4.2)
caption(s, X0, 4.25, 4.2, "Figure. x = −1, 0, +1 m and their overlay (box: 3 m task strip).")
add_text(s, "m4_cover", 5.05, 1.28, 4.7, 1.6, [
    [("Pure translation ", True), ("(exact): the slide adds reach along x only.", False)],
    [("Task strip: ", True), ("one base position covers ≤ 67.5 %, three together 100 %.", False)],
    [("34 % is reachable from one x only: ", True), ("there the base position is forced.", False)],
], size=11, bullets=True, space=4)
picture(s, "m4_img_tilt", "reach_map_x0_tilt.png", 5.05, 2.95, w=4.7)
caption(s, 5.05, 4.22, 4.7, "Figure. Tool tilted 0, 30, 60, 90° towards +x (x = 0).")
add_text(s, "m4_orient", X0, 4.6, 9.1, 0.7, [
    [("Tilting shrinks the 4-branch ring; a horizontal tool brings the wrist family. ", True),
     ("On singular cells the slide restores full rank for 88–96 % of shoulder/elbow cells but 12.5 % of wrist cells: "
      "a slide cannot give back a lost rotation (type B).", False)]], size=11)
notes(s, "[~1:15] Moving the base does not change the map, it translates it, exactly. The useful part is the overlay. "
         "On a 3 m task strip one base position covers at most two thirds; the three positions together cover all of "
         "it, and a third of the strip is reachable from one base position only. There the base position is forced, "
         "and that is what the planner has to find.\n"
         "Orientation changes the shape: tilting the tool shrinks the four-branch ring, and a horizontal tool makes the "
         "wrist singularity appear. Checking the singular cells: the slide rescues almost all shoulder and elbow "
         "cells, but only 12.5 % of wrist cells, because the wrist loses a rotation and a slide cannot give it back. "
         "That is type B from the Jacobian part.\n"
         "Wenqian will now show how this is used for path planning.")


# ================================================================== conclusions (slide 12, was blank); drop blank slide 13
s = prs.slides[11]
assert len(s.shapes) == 0, "slide 12 is not empty any more"
s.shapes._spTree.append(copy.deepcopy(template.shapes[0]._element))
s.shapes[-1].text_frame.paragraphs[0].runs[0].text = "Conclusions"
add_text(s, "c_q1_head", X0, 1.15, 9.0, 0.3, [[("How does the base position x select the IK branches?", True)]],
         size=12, color="C00000")
add_text(s, "c_q1", X0, 1.47, 9.0, 1.5, [
    "With x fixed the P6R is a UR12e with a shifted target. The IK splits into three groups whose roots merge exactly "
    "on det J_red = 0, so moving the base gains or loses branches in pairs (0, 4 or 8).",
    "Not cuspidal: one solution per aspect, so a branch stays valid until a singularity.",
    "The slide rescues most shoulder and elbow singularities, almost no wrist ones: it cannot supply a rotation.",
], size=11, bullets=True, space=3)
add_text(s, "c_q2_head", X0, 2.7, 9.0, 0.3, [[("How can x and the branch be planned jointly?", True)]],
         size=12, color="C00000")
add_text(s, "c_q2", X0, 3.02, 9.0, 1.5, [
    "Each branch is a region in the (s, x) plane; an execution is a curve through it with slope bounded by the base "
    "speed. Obstacles: reach limits, the shoulder hole, the ±π joint seam.",
    "DP over the whole plane (possible because the analytical IK gives all branches everywhere) plans 6 of 8 branches. "
    "Greedy fails on every branch at 0.42× tool speed, where DP still finds 3.",
    "The best branch changes with the task (tilted path): x(s) and the branch must be planned together, per task.",
], size=11, bullets=True, space=3)
add_text(s, "c_limits", X0, 4.95, 9.0, 0.35, [
    [("Limitations: ", True), ("no collision model, base along x only, mount geometry not measured, motion between "
                              "samples not checked.", False)]], size=9.5, color=GREY)
notes(s, "[~0:40] To answer our two questions. First: with the base fixed, the arm has at most 8 branches, and they "
         "appear and disappear in pairs exactly on the singular surfaces, so the base position decides which branches "
         "exist. The slide helps with shoulder and elbow singularities, not with the wrist.\n"
         "Second: each branch is a region in the (s, x) plane, and planning means finding a curve through it. Because "
         "the analytical IK gives every branch everywhere, we can plan over the whole plane, and that beats a greedy "
         "planner when the base is slow. And the best branch depends on the task, so branch and base have to be "
         "planned together. Thank you.")

# ================================================================== backup: greedy vs branch-aware planning
s = backup_slide("Backup: greedy vs branch-aware planning")
subtitle(s, "Same cost |Δq| + 2|Δx| and constraints; the only difference is the lookahead")
add_table(s, "g_table", X0 + 0.04, 1.30, [1.9, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5], [
    ["k = base / tool speed", "1.25", "0.94", "0.62", "0.50", "0.42", "0.31"],
    ["branch-aware DP", "6/8", "6/8", "5/8", "3/8", "3/8", "0/8"],
    ["greedy (min step cost)", "6/8", "6/8", "3/8", "3/8", "0/8", "0/8"],
], row_h=0.25, size=10, right=(1, 2, 3, 4, 5, 6), highlight=None)
add_text(s, "g_fast", 5.85, 1.28, 3.9, 1.0, [
    [("Fast base: ", True), ("same 6 branches, greedy 10–46 % more expensive; on the best branch it rides the reach "
                            "edge (σ_min 0.033 vs 0.172).", False)]], size=10)
picture(s, "g_img", "s5_sx_map_dp_vs_greedy_slow_onegreedy.png", X0, 2.2, w=5.4)
caption(s, X0, 4.7, 5.4, "Figure. Slow base (k = 0.42): DP (red) vs greedy (blue, × = stops).")
add_text(s, "g_slow", 5.85, 2.25, 3.9, 3.0, [
    [("Slow base (k = 0.42): ", True), ("greedy fails on all 8 branches, DP still finds 3.", False)],
    "The green band climbs with the tool. A slow base passes above the grey wedge at s ≈ 0.5 only if it climbs at "
    "full speed from the start.",
    "DP sees the wedge and commits early. Greedy sometimes takes a slightly smaller (cheaper) step, arrives a few cm "
    "too low and stops at the wedge tip.",
    "A second rule (max σ_min each step) fails the same way: the problem is the missing lookahead, not the rule.",
], size=10, bullets=True, space=4)
notes(s, "Both planners pick the base position step by step along the path, with the same constraints and the same cost: "
         "joint motion plus two times base motion. The only difference is that the DP sees the whole map, and greedy "
         "only looks one step ahead.\n"
         "With a fast base both complete the same six branches, but greedy is 10 to 46 percent more expensive, and on "
         "the best branch it does not move the base enough, so the arm is almost stretched.\n"
         "With a slow base greedy fails everywhere at 0.42. The green band climbs with the tool; to pass above the grey "
         "wedge a slow base must climb at full speed from the start. DP sees the wedge and commits early. Greedy "
         "sometimes takes a slightly cheaper smaller step, ends up a few centimetres too low and gets stuck at the tip. "
         "A safety-first greedy rule fails the same way, so it is the lookahead that matters. That is also why the "
         "analytical IK matters: it gives all branches at every point, so the planner can see the whole map; a "
         "numerical IK behaves like a local rule.")

# drop the blank slide 13 (after all slides are added, so part names stay unique)
_ids = prs.slides._sldIdLst
assert len(prs.slides[12].shapes) == 0, "slide 13 is not empty any more"
_el = _ids[12]
prs.part.drop_rel(_el.rId)
_ids.remove(_el)

prs.save(OUT)
print("written", OUT, len(prs.slides), "slides")
