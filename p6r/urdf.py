"""URDF loader and reference forward kinematics.

The reference FK chains the URDF transforms verbatim:

    T_root_tip(q) = prod_i  Origin_i  *  Motion_i(q_i)

where Origin_i is the joint's <origin xyz rpy/> and Motion_i is a rotation about the
joint <axis/> (revolute), a translation along it (prismatic), or identity (fixed).
No DH, no screws: this is the ground truth the derived models are checked against.
"""
from dataclasses import dataclass
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np

from .se3 import urdf_origin, T_rot_axis, T_trans_axis


@dataclass
class Joint:
    name: str
    type: str                 # 'revolute' | 'prismatic' | 'fixed' | 'continuous'
    parent: str
    child: str
    xyz: np.ndarray           # origin translation (parent frame)
    rpy: np.ndarray           # origin roll, pitch, yaw
    axis: np.ndarray | None   # joint axis in the child frame (None for fixed)
    lower: float | None       # position limits [rad or m], None if absent
    upper: float | None

    @property
    def moving(self):
        return self.type in ("revolute", "prismatic", "continuous")

    def origin(self):
        return urdf_origin(self.xyz, self.rpy)

    def motion(self, q):
        if self.type in ("revolute", "continuous"):
            return T_rot_axis(self.axis, q)
        if self.type == "prismatic":
            return T_trans_axis(self.axis, q)
        return np.eye(4)


def _vec(el, attr, default):
    if el is None or el.get(attr) is None:
        return np.array(default, float)
    return np.array([float(x) for x in el.get(attr).split()])


def load_joints(urdf_path):
    """All joints of the URDF, keyed by child link."""
    root = ET.parse(str(urdf_path)).getroot()
    joints = {}
    for j in root.findall("joint"):
        o, a, lim = j.find("origin"), j.find("axis"), j.find("limit")
        jt = Joint(
            name=j.get("name"),
            type=j.get("type"),
            parent=j.find("parent").get("link"),
            child=j.find("child").get("link"),
            xyz=_vec(o, "xyz", [0, 0, 0]),
            rpy=_vec(o, "rpy", [0, 0, 0]),
            axis=_vec(a, "xyz", [1, 0, 0]) if a is not None else None,
            lower=float(lim.get("lower")) if lim is not None and lim.get("lower") else None,
            upper=float(lim.get("upper")) if lim is not None and lim.get("upper") else None,
        )
        joints[jt.child] = jt
    return joints


def load_chain(urdf_path, root="world", tip="flange"):
    """Ordered list of joints from `root` link to `tip` link (fixed joints included)."""
    joints = load_joints(urdf_path)
    chain, link = [], tip
    while link != root:
        if link not in joints:
            raise ValueError(f"link '{link}' has no parent joint; chain from '{tip}' never reaches '{root}'")
        chain.append(joints[link])
        link = joints[link].parent
    chain.reverse()
    return chain


class UrdfChain:
    """A serial chain read from a URDF, with reference FK."""

    def __init__(self, urdf_path, root="world", tip="flange"):
        self.path = Path(urdf_path)
        self.root, self.tip = root, tip
        self.joints = load_chain(urdf_path, root, tip)
        self.moving = [j for j in self.joints if j.moving]
        self.n = len(self.moving)
        self.names = [j.name for j in self.moving]
        self.lower = np.array([j.lower if j.lower is not None else -np.inf for j in self.moving])
        self.upper = np.array([j.upper if j.upper is not None else np.inf for j in self.moving])

    def fk(self, q, upto=None):
        """Reference FK: root -> tip (or root -> `upto` link) at joint vector q."""
        q = np.asarray(q, float)
        assert q.shape == (self.n,), f"expected {self.n} joint values, got {q.shape}"
        T, k = np.eye(4), 0
        for j in self.joints:
            T = T @ j.origin()
            if j.moving:
                T = T @ j.motion(q[k])
                k += 1
            if j.child == upto:
                break
        return T

    def frames(self, q):
        """Dict link -> T_root_link at q, for every link on the chain."""
        q = np.asarray(q, float)
        out, T, k = {self.root: np.eye(4)}, np.eye(4), 0
        for j in self.joints:
            T = T @ j.origin()
            if j.moving:
                T = T @ j.motion(q[k])
                k += 1
            out[j.child] = T.copy()
        return out

    def table(self):
        """Printable joint table (what the brief calls 'identify the joint axes and offsets')."""
        rows = [f"{'joint':24s} {'type':9s} {'parent':18s} {'child':18s} {'xyz':30s} {'rpy':30s} {'axis':8s} limits"]
        for j in self.joints:
            xyz = np.array2string(j.xyz, precision=5, suppress_small=True)
            rpy = np.array2string(j.rpy, precision=5, suppress_small=True)
            ax = "-" if j.axis is None else np.array2string(j.axis, precision=0)
            lim = "-" if j.lower is None else f"[{j.lower:+.3f}, {j.upper:+.3f}]"
            rows.append(f"{j.name:24s} {j.type:9s} {j.parent:18s} {j.child:18s} {xyz:30s} {rpy:30s} {ax:8s} {lim}")
        return "\n".join(rows)
