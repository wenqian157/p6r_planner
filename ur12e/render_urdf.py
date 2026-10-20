"""Render ur12e.urdf from the Universal Robots xacro templates without ROS.

Source: https://github.com/UniversalRobots/Universal_Robots_ROS2_Description (rolling)
Files used: urdf/ur.urdf.xacro, urdf/ur_macro.xacro, urdf/inc/ur_common.xacro,
            config/ur12e/*.yaml, meshes/ur10e/* (the UR12e reuses the UR10e meshes).

Run:  python render_urdf.py        (from this folder, in the `dev` conda env)
"""
from pathlib import Path
import xacro
import xacro.substitution_args as sa

ROOT = Path(__file__).resolve().parent
POSIX_ROOT = ROOT.as_posix()

# $(find ur_description) -> this folder
sa._eval_find = lambda pkg: POSIX_ROOT

doc = xacro.process_file(str(ROOT / "urdf" / "ur.urdf.xacro"),
                         mappings={"name": "ur12e", "ur_type": "ur12e"})
text = doc.toprettyxml(indent="  ")

# mesh references -> paths relative to this folder
text = text.replace("package://ur_description/", "")
text = text.replace(f"file://{POSIX_ROOT}/", "")

out = ROOT / "ur12e.urdf"
out.write_text(text, encoding="utf-8")
print("wrote", out)
