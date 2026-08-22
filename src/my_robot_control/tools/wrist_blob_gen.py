#!/usr/bin/env python3
"""
Swept-volume generator for the spherical-wrist assembly of the MyArm 300 Pi.

WHAT THIS IS FOR
----------------
The arm is an S-R-S manipulator: joints 1,2,3 intersect at the shoulder, joint 4
is the elbow, joints 5,6,7 intersect at the wrist centre. Because of that, the
wrist centre W is a function of q1..q4 ALONE -- the last three joints change
orientation and nothing else.

That lets us plan in a reduced space (q1..q4, or q1,q2,q4) if we can collision
check without knowing q5..q7. The trick is to replace everything distal of the
wrist centre -- link5, link6, link7, flange, gripper, and the carried tube -- by
a single rigid body that encloses every pose those parts can take. Plan against
that body, and any path OMPL accepts is guaranteed collision free for the real
arm, whatever the wrist ends up doing.

The catch is that "every pose" is a sphere of ~0.27 m radius, and the arm's own
reach from the shoulder is only 0.2433 m. A blob that big collides with the
table everywhere and the reduced problem has no solutions at all.

The way out is the task's own orientation constraint. While carrying a tube the
tool axis must stay vertical (uprightConstraint() in simple_move.cpp), which
leaves exactly one free DOF: roll about that vertical axis. Sweeping only that
one DOF gives a flat ANNULUS instead of a fat sphere -- and, crucially, an
annulus that is a solid of revolution about the world vertical. That is why the
reduced URDF attaches the blob through two "align" joints: they hold the blob's
axis vertical as the arm moves, so the same rigid body stays valid everywhere.

So this script sweeps the wrist assembly over the orientation set the constraint
permits, in a frame V that sits at the wrist centre and is aligned with the
world, and emits the enclosing solid of revolution.

OUTPUT
------
  wrist_blob_<tag>.stl        revolved profile, keeps the hole in the annulus
  wrist_blob_<tag>.urdf.xacro <collision> block: cylinder stack, no hole
  wrist_blob_<tag>.json       profile + stats, consumed by the C++ side

The mesh is the accurate one. The cylinder stack is the fallback for collision
backends that convexify meshes, and it is strictly more conservative -- it fills
the hole in the middle of the ring.

USAGE
-----
  python3 wrist_blob_gen.py --tilt 0.30 --tag carry
  python3 wrist_blob_gen.py --full-so3 --tag free     # naive sphere, for comparison
"""

import argparse
import json
import math
import os
import struct
import sys
import xml.etree.ElementTree as ET

import numpy as np

# --- Geometry of the carried payload, mirrored from simple_move.cpp ---------
# Keep these in sync by hand; they are deliberately duplicated rather than
# parsed out of the C++ so that this script stands alone.
TUBE_HEIGHT = 0.13
TUBE_RADIUS = 0.012

# Where the tube sits in the TCP (== link7) frame.
#
# simple_move.cpp expresses the offset in the WORLD frame as (0, -0.135, -0.030),
# which is only correct at the one specific upright yaw it always uses. Rotating
# that offset into the TCP frame through q_upright = RPY(0, -pi/2, pi/2) gives
# the frame-independent version used here:
#
#   R = Rz(pi/2) Ry(-pi/2)  =>  R e_x = +Z_world,  R e_y = -X_world,  R e_z = -Y_world
#   world (0, -0.135, -0.030)  =>  tcp (-0.030, 0, +0.135)
#
# The +0.135 along the TCP's z is the same direction as the wrist-centre ->
# flange offset (0.066), so the two ADD: the tube ends up 0.201 m from the wrist
# centre. That is what makes the swept ring as wide as it is.
TUBE_TCP_OFFSET = np.array([-0.030, 0.0, 0.135])

# Gripper travel, from the SRDF group states "open" and "closed". The URDF
# limits are wider than the gripper is ever driven, so sweeping those instead
# would inflate the blob for poses the mechanism cannot reach.
GRIPPER_OPEN = -0.2
GRIPPER_CLOSED = -1.2

# Links distal of the wrist centre whose pose is pinned once the tool
# orientation is pinned. Swept exactly.
DISTAL_LINKS = [
    "link7", "flange_link", "gripper_base",
    "gripper_arm_gear_right", "gripper_arm_gear_left",
    "gripper_arm_simple_right", "gripper_arm_simple_left",
    "gripper_finger_right", "gripper_finger_left",
]

# Links between the wrist centre and link7. Their pose depends on q5 and q6
# individually, which the tool orientation does NOT determine, so they are
# bounded by a sphere instead of swept exactly. They sit right on the wrist
# centre and are small, so this costs almost nothing.
PROXIMAL_LINKS = ["link5", "link6"]


# ===========================================================================
#  URDF
# ===========================================================================

def rpy_to_R(r, p, y):
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    return Rz @ Ry @ Rx


def iso(R=None, p=None):
    T = np.eye(4)
    if R is not None:
        T[:3, :3] = R
    if p is not None:
        T[:3, 3] = p
    return T


def _vec(s, default=(0.0, 0.0, 0.0)):
    if s is None:
        return np.array(default, dtype=float)
    return np.array([float(v) for v in s.split()], dtype=float)


class Urdf:
    """Just enough URDF for this job: joint frames and collision geometry."""

    def __init__(self, path):
        self.path = path
        root = ET.parse(path).getroot()
        self.joints = {}
        self.links = {}

        for j in root.findall("joint"):
            o = j.find("origin")
            ax = j.find("axis")
            lim = j.find("limit")
            mim = j.find("mimic")
            self.joints[j.get("name")] = {
                "type": j.get("type"),
                "parent": j.find("parent").get("link"),
                "child": j.find("child").get("link"),
                "xyz": _vec(o.get("xyz") if o is not None else None),
                "rpy": _vec(o.get("rpy") if o is not None else None),
                "axis": _vec(ax.get("xyz") if ax is not None else None, (0, 0, 1)),
                "lower": float(lim.get("lower")) if lim is not None and lim.get("lower") else 0.0,
                "upper": float(lim.get("upper")) if lim is not None and lim.get("upper") else 0.0,
                "mimic": None if mim is None else {
                    "joint": mim.get("joint"),
                    "multiplier": float(mim.get("multiplier", 1.0)),
                    "offset": float(mim.get("offset", 0.0)),
                },
            }

        for L in root.findall("link"):
            geoms = []
            for c in L.findall("collision"):
                o = c.find("origin")
                T = iso(rpy_to_R(*_vec(o.get("rpy") if o is not None else None)),
                        _vec(o.get("xyz") if o is not None else None))
                g = c.find("geometry")
                kid = list(g)[0]
                geoms.append({"tag": kid.tag, "attrib": dict(kid.attrib), "T": T})
            self.links[L.get("name")] = geoms

    def joint_T(self, name, q):
        """Parent-frame -> child-frame transform at joint value q."""
        j = self.joints[name]
        T = iso(rpy_to_R(*j["rpy"]), j["xyz"])
        if j["type"] in ("revolute", "continuous"):
            a = j["axis"] / np.linalg.norm(j["axis"])
            K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
            R = np.eye(3) + math.sin(q) * K + (1 - math.cos(q)) * (K @ K)
            T = T @ iso(R=R)
        elif j["type"] == "prismatic":
            T = T @ iso(p=j["axis"] * q)
        return T


# ===========================================================================
#  Meshes
# ===========================================================================

def load_stl_vertices(path):
    """Vertices of a binary or ASCII STL, as an (N,3) array."""
    with open(path, "rb") as f:
        data = f.read()

    # A binary STL is 84 + 50*n bytes. Check that before trusting the header
    # text, because plenty of binary STLs start with the word "solid".
    if len(data) >= 84:
        n = struct.unpack("<I", data[80:84])[0]
        if len(data) == 84 + 50 * n and n > 0:
            raw = np.frombuffer(data, dtype=np.uint8, count=50 * n, offset=84)
            raw = raw.reshape(n, 50)
            # bytes 12..48 of each record are the three vertices
            tri = raw[:, 12:48].copy().view(np.float32).reshape(n * 3, 3)
            return tri.astype(np.float64)

    verts = []
    for line in data.decode("ascii", "ignore").splitlines():
        p = line.split()
        if len(p) == 4 and p[0] == "vertex":
            verts.append([float(p[1]), float(p[2]), float(p[3])])
    if not verts:
        raise ValueError(f"no triangles found in {path}")
    return np.array(verts)


def resolve_package_path(uri, pkg_roots):
    if not uri.startswith("package://"):
        return uri
    rest = uri[len("package://"):]
    pkg, _, tail = rest.partition("/")
    for root in pkg_roots:
        cand = os.path.join(root, pkg, tail)
        if os.path.exists(cand):
            return cand
    raise FileNotFoundError(f"cannot resolve {uri}")


def box_points(size, n=6):
    """Surface samples of a box. Corners alone are not enough once the box is
    swept, because the swept hull is taken per-point."""
    sx, sy, sz = (np.array(size) / 2.0)
    ax = [np.linspace(-s, s, n) for s in (sx, sy, sz)]
    pts = []
    for axis in range(3):
        grid = np.meshgrid(*[ax[k] for k in range(3) if k != axis], indexing="ij")
        flat = [g.ravel() for g in grid]
        for face in (-1, 1):
            col = np.full(flat[0].shape, face * (sx, sy, sz)[axis])
            cols = list(flat)
            cols.insert(axis, col)
            pts.append(np.stack(cols, axis=1))
    return np.unique(np.concatenate(pts), axis=0)


def cylinder_points(radius, length, n_theta=32, n_z=5, axis="z"):
    th = np.linspace(0, 2 * math.pi, n_theta, endpoint=False)
    z = np.linspace(-length / 2, length / 2, n_z)
    T, Z = np.meshgrid(th, z, indexing="ij")
    a, b, c = radius * np.cos(T).ravel(), radius * np.sin(T).ravel(), Z.ravel()
    if axis == "z":
        return np.stack([a, b, c], axis=1)
    if axis == "x":
        return np.stack([c, a, b], axis=1)
    if axis == "y":
        return np.stack([b, c, a], axis=1)
    raise ValueError(axis)


def link_points(urdf, link, pkg_roots, decimate):
    """Collision-geometry sample points of a link, in the link frame."""
    out = []
    for g in urdf.links.get(link, []):
        tag, at, T = g["tag"], g["attrib"], g["T"]
        if tag == "mesh":
            v = load_stl_vertices(resolve_package_path(at["filename"], pkg_roots))
            scale = _vec(at.get("scale"), (1, 1, 1))
            v = v * scale
            if decimate > 1 and len(v) > decimate:
                idx = np.linspace(0, len(v) - 1, decimate).astype(int)
                v = v[idx]
        elif tag == "box":
            v = box_points(_vec(at["size"]))
        elif tag == "cylinder":
            v = cylinder_points(float(at["radius"]), float(at["length"]))
        elif tag == "sphere":
            r = float(at["radius"])
            v = cylinder_points(r, 0.0, 32, 1)
            v = np.concatenate([v, [[0, 0, r], [0, 0, -r]]])
        else:
            continue
        out.append((T[:3, :3] @ v.T).T + T[:3, 3])
    if not out:
        return np.zeros((0, 3))
    return np.concatenate(out)


# ===========================================================================
#  Wrist kinematics
# ===========================================================================

class Wrist:
    """
    Poses of everything distal of the wrist centre, expressed in the frame V:
    origin at the wrist centre, axes aligned with the world.

    The chain from the wrist centre is

        T_V_link7 = Rz(q5) . Tj6 . Rz(q6) . Tj7 . Rz(q7)

    pre-multiplied by whatever orientation the first four joints produce. Since
    the rotation part works out to R_w . Rz(q5) . Ry(-q6) . Rz(q7), the wrist is
    a textbook ZYZ Euler set and we can invert it in closed form.
    """

    def __init__(self, urdf):
        self.u = urdf
        # joint5's origin transform is what carries the frame onto the wrist
        # centre; its rotation is folded into R_w, so only the offsets of
        # joint6 and joint7 matter downstream.
        self.T_j6 = iso(rpy_to_R(*urdf.joints["joint6"]["rpy"]), urdf.joints["joint6"]["xyz"])
        self.T_j7 = iso(rpy_to_R(*urdf.joints["joint7"]["rpy"]), urdf.joints["joint7"]["xyz"])

    @staticmethod
    def Rz(q):
        c, s = math.cos(q), math.sin(q)
        return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])

    def chain(self, q5, q6, q7):
        """Wrist-centre frame -> {link5, link6, link7} frames, with the wrist
        centre frame taken as identity (i.e. R_w folded out)."""
        T5 = iso(R=self.Rz(q5))                     # link5
        T6 = T5 @ self.T_j6 @ iso(R=self.Rz(q6))    # link6
        T7 = T6 @ self.T_j7 @ iso(R=self.Rz(q7))    # link7
        return T5, T6, T7

    def solve_zyz(self, M):
        """All (q5,q6,q7) with Rz(q5) Ry(-q6) Rz(q7) == M."""
        sols = []
        c = float(np.clip(M[2, 2], -1.0, 1.0))
        beta = math.acos(c)
        for b in (beta, -beta):
            if abs(math.sin(b)) < 1e-8:
                # Gimbal lock: only q5+q7 (or q5-q7) is determined. Pick q5=0.
                q5 = 0.0
                q7 = math.atan2(M[1, 0], M[0, 0]) * (1.0 if c > 0 else -1.0)
            else:
                sb = math.sin(b)
                q5 = math.atan2(M[1, 2] / sb, M[0, 2] / sb)
                q7 = math.atan2(M[2, 1] / sb, -M[2, 0] / sb)
            sols.append((q5, -b, q7))
        return sols

    def in_limits(self, q5, q6, q7):
        for name, q in (("joint5", q5), ("joint6", q6), ("joint7", q7)):
            j = self.u.joints[name]
            if not (j["lower"] - 1e-9 <= q <= j["upper"] + 1e-9):
                return False
        return True


def upright_orientations(tilt_tol, n_roll, n_tilt, n_tilt_dir):
    """
    Tool orientations the carry constraint allows, as rotation matrices in the
    world frame.

    The constraint is: the TCP x-axis (the tube's long axis) points up, within
    `tilt_tol`. Roll about that axis is free and is what generates the ring.
    """
    base = rpy_to_R(0.0, -math.pi / 2, math.pi / 2)   # q_upright from simple_move.cpp
    assert np.allclose(base @ np.array([1.0, 0, 0]), [0, 0, 1], atol=1e-6)

    out = []
    tilts = [0.0] if n_tilt <= 1 else np.linspace(0.0, tilt_tol, n_tilt)
    for roll in np.linspace(0, 2 * math.pi, n_roll, endpoint=False):
        # roll about the WORLD vertical, which is the tool axis
        Rroll = rpy_to_R(0, 0, roll)
        for t in tilts:
            if t == 0.0:
                out.append(Rroll @ base)
                continue
            for d in np.linspace(0, 2 * math.pi, n_tilt_dir, endpoint=False):
                axis = np.array([math.cos(d), math.sin(d), 0.0])   # tip about horizontal
                K = np.array([[0, -axis[2], axis[1]],
                              [axis[2], 0, -axis[0]],
                              [-axis[1], axis[0], 0]])
                Rt = np.eye(3) + math.sin(t) * K + (1 - math.cos(t)) * (K @ K)
                out.append(Rt @ Rroll @ base)
    return out


# ===========================================================================
#  Sweep
# ===========================================================================

def sweep(urdf, args, pkg_roots):
    wrist = Wrist(urdf)

    # --- points of each link, in its own frame -----------------------------
    pts = {L: link_points(urdf, L, pkg_roots, args.decimate)
           for L in DISTAL_LINKS + PROXIMAL_LINKS}

    # The carried tube, in the link7 frame. Its long axis is the TCP's X --
    # that is the axis uprightConstraint() holds vertical and the one
    # maxTiltAlongTrajectory() measures against, so the cylinder has to be built
    # X-aligned, not Z-aligned.
    if args.tube:
        tube = cylinder_points(TUBE_RADIUS, TUBE_HEIGHT, 24, 5, axis="x") + TUBE_TCP_OFFSET
        pts["__tube__"] = tube

    # --- gripper sub-chain -------------------------------------------------
    # Every gripper joint mimics endeffector_gripper (see the <mimic> tags in
    # the URDF and gripper_mimic.py), so the whole hand is ONE degree of
    # freedom. Sweeping the six joints independently, as if they were free,
    # inflates the blob badly -- it lets a finger point somewhere the mechanism
    # physically cannot put it.
    #
    # The travel that matters is the operating range, open (-0.2) to closed
    # (-1.2), from the SRDF group states -- not the URDF limits, which are wider
    # than the gripper is ever actually driven.
    def gripper_frames(values):
        """link7 frame -> each gripper link frame, one dict per driver value."""
        T_flange = urdf.joint_T("flange_joint", 0.0)
        T_gb = T_flange @ urdf.joint_T("joint7_to_gripper_base", 0.0)

        def q_of(name, driver):
            m = urdf.joints[name]["mimic"]
            if m is None:
                return driver
            assert m["joint"] == "endeffector_gripper", f"unexpected mimic source for {name}"
            return m["multiplier"] * driver + m["offset"]

        out = []
        for driver in values:
            fr = {"link7": np.eye(4), "flange_link": T_flange, "gripper_base": T_gb}
            for jn, child in (("endeffector_gripper", "gripper_arm_gear_right"),
                              ("gripperbase_to_armgearleft", "gripper_arm_gear_left"),
                              ("gripperbase_to_armsimpleright", "gripper_arm_simple_right"),
                              ("gripperbase_to_armsimpleleft", "gripper_arm_simple_left")):
                fr[child] = T_gb @ urdf.joint_T(jn, q_of(jn, driver))
            fr["gripper_finger_right"] = (
                fr["gripper_arm_gear_right"]
                @ urdf.joint_T("armgearright_to_fingerright", q_of("armgearright_to_fingerright", driver)))
            fr["gripper_finger_left"] = (
                fr["gripper_arm_gear_left"]
                @ urdf.joint_T("armgearleft_to_fingerleft", q_of("armgearleft_to_fingerleft", driver)))
            out.append(fr)
        return out

    if args.gripper == "closed":
        drivers = [GRIPPER_CLOSED]
    elif args.gripper == "open":
        drivers = [GRIPPER_OPEN]
    else:
        drivers = np.linspace(GRIPPER_CLOSED, GRIPPER_OPEN, args.n_gripper)
    grip_frames = gripper_frames(drivers)

    # --- accumulate (radius, height) in the V frame ------------------------
    rz = []          # exact sweep, distal links
    stats = {"orientations": 0, "unreachable_note": ""}

    if args.full_so3:
        # Naive interpretation: every wrist configuration is allowed. Sweep q5,q6,q7
        # over their whole ranges and let the result be whatever it is.
        j5, j6, j7 = (urdf.joints[n] for n in ("joint5", "joint6", "joint7"))
        grid = [np.linspace(j["lower"], j["upper"], n)
                for j, n in ((j5, args.n_q5), (j6, args.n_q6), (j7, args.n_q7))]
        combos = [(a, b, c) for a in grid[0] for b in grid[1] for c in grid[2]]
        for (q5, q6, q7) in combos:
            _, _, T7 = wrist.chain(q5, q6, q7)
            for gf in grip_frames:
                for L, P in pts.items():
                    if L in PROXIMAL_LINKS or len(P) == 0:
                        continue
                    base = T7 @ gf[L] if L in gf else T7
                    if L == "__tube__":
                        base = T7
                    q = (base[:3, :3] @ P.T).T + base[:3, 3]
                    rz.append(np.stack([np.hypot(q[:, 0], q[:, 1]), q[:, 2]], axis=1))
        stats["orientations"] = len(combos)
    else:
        # Constraint-aware: only orientations the upright constraint permits.
        #
        # The wrist-centre frame's orientation in the world (R_w) depends on
        # q1..q4, so the SAME tool orientation is produced by different
        # (q5,q6,q7) depending on where the arm is. We do not know R_w here and
        # we do not need it: the pose of everything distal, expressed in V, is
        # fixed by the tool orientation alone. R_w only decides whether the
        # wrist can REACH it. Ignoring that is the conservative choice -- it
        # assumes every roll angle is reachable, which can only make the blob
        # bigger, never smaller.
        Rs = upright_orientations(args.tilt, args.n_roll, args.n_tilt, args.n_tilt_dir)
        for R7 in Rs:
            # Wrist centre -> link7 origin. joint7's origin offset lies ALONG
            # the joint7 axis (that is exactly why the wrist is spherical), so
            # in the world it is simply |offset| along link7's own z axis --
            # no dependence on q5 or q6.
            off = R7[:, 2] * float(np.linalg.norm(wrist.T_j7[:3, 3]))
            T7 = iso(R=R7, p=off)
            for gf in grip_frames:
                for L, P in pts.items():
                    if L in PROXIMAL_LINKS or len(P) == 0:
                        continue
                    base = T7 if L == "__tube__" else T7 @ gf[L]
                    q = (base[:3, :3] @ P.T).T + base[:3, 3]
                    rz.append(np.stack([np.hypot(q[:, 0], q[:, 1]), q[:, 2]], axis=1))
        stats["orientations"] = len(Rs)

    rz = np.concatenate(rz) if rz else np.zeros((0, 2))

    # --- proximal links: bounded by a sphere about the wrist centre --------
    # Their pose depends on q5 and q6 individually, which the tool orientation
    # does not pin down, so we take the worst case over the whole wrist range.
    prox_r = 0.0
    for L in PROXIMAL_LINKS:
        P = pts[L]
        if len(P) == 0:
            continue
        j5, j6 = urdf.joints["joint5"], urdf.joints["joint6"]
        for q5 in np.linspace(j5["lower"], j5["upper"], args.n_q5):
            for q6 in np.linspace(j6["lower"], j6["upper"], args.n_q6):
                T5, T6, _ = wrist.chain(q5, q6, 0.0)
                T = T5 if L == "link5" else T6
                q = (T[:3, :3] @ P.T).T + T[:3, 3]
                prox_r = max(prox_r, float(np.linalg.norm(q, axis=1).max()))

    return rz, prox_r, stats


def build_profile(rz, prox_r, n_bins, margin):
    """Max radius per height bin -> the solid-of-revolution profile."""
    z_lo = min(rz[:, 1].min(), -prox_r)
    z_hi = max(rz[:, 1].max(), prox_r)
    edges = np.linspace(z_lo, z_hi, n_bins + 1)

    idx = np.clip(np.digitize(rz[:, 1], edges) - 1, 0, n_bins - 1)
    r_max = np.zeros(n_bins)
    r_min = np.full(n_bins, np.inf)
    np.maximum.at(r_max, idx, rz[:, 0])
    np.minimum.at(r_min, idx, rz[:, 0])

    # union with the proximal sphere
    for k in range(n_bins):
        zc = max(abs(edges[k]), abs(edges[k + 1]))
        if zc < prox_r:
            r_sphere = math.sqrt(max(prox_r ** 2 - zc ** 2, 0.0))
            if r_sphere > 0:
                r_max[k] = max(r_max[k], r_sphere)
                r_min[k] = 0.0            # sphere is solid: no hole here

    r_min[~np.isfinite(r_min)] = 0.0
    r_min[r_max <= 0] = 0.0

    # a bin with no samples at all contributes nothing
    occupied = r_max > 0
    r_max = np.where(occupied, r_max + margin, 0.0)
    r_min = np.where(occupied, np.maximum(r_min - margin, 0.0), 0.0)
    return edges, r_max, r_min, occupied


# ===========================================================================
#  Output
# ===========================================================================

def write_revolved_stl(path, edges, r_out, r_in, occupied, n_theta=48):
    """Revolve the (r,z) profile into a closed STL. Keeps the central hole."""
    tris = []
    th = np.linspace(0, 2 * math.pi, n_theta, endpoint=False)
    ct, st = np.cos(th), np.sin(th)

    def ring(r, z):
        return np.stack([r * ct, r * st, np.full(n_theta, z)], axis=1)

    def band(r_lo, r_hi, z0, z1):
        # outer wall, inner wall, and the two annular caps of one bin
        o0, o1 = ring(r_hi, z0), ring(r_hi, z1)
        i0, i1 = ring(r_lo, z0), ring(r_lo, z1)
        for k in range(n_theta):
            m = (k + 1) % n_theta
            tris.extend([(o0[k], o1[k], o1[m]), (o0[k], o1[m], o0[m])])   # outer
            tris.extend([(i0[m], i1[m], i1[k]), (i0[m], i1[k], i0[k])])   # inner
            tris.extend([(i0[k], i0[m], o0[m]), (i0[k], o0[m], o0[k])])   # bottom cap
            tris.extend([(o1[k], o1[m], i1[m]), (o1[k], i1[m], i1[k])])   # top cap

    for k in range(len(r_out)):
        if not occupied[k]:
            continue
        band(r_in[k], r_out[k], edges[k], edges[k + 1])

    tris = np.array(tris, dtype=np.float32)
    with open(path, "wb") as f:
        f.write(b"wrist swept volume".ljust(80, b"\0"))
        f.write(struct.pack("<I", len(tris)))
        for t in tris:
            n = np.cross(t[1] - t[0], t[2] - t[0])
            ln = np.linalg.norm(n)
            n = n / ln if ln > 1e-12 else np.zeros(3)
            f.write(struct.pack("<3f", *n))
            for v in t:
                f.write(struct.pack("<3f", *v))
            f.write(b"\0\0")
    return len(tris)


def write_urdf_snippet(path, edges, r_out, occupied, link_name, merge_tol):
    """Cylinder stack. Solid -- it fills the ring's hole, so it is strictly
    more conservative than the mesh."""
    bands = []
    for k in range(len(r_out)):
        if not occupied[k]:
            continue
        z0, z1, r = edges[k], edges[k + 1], r_out[k]
        if bands and abs(bands[-1][2] - r) < merge_tol and abs(bands[-1][1] - z0) < 1e-9:
            bands[-1] = (bands[-1][0], z1, max(bands[-1][2], r))
        else:
            bands.append((z0, z1, r))

    lines = [
        '<?xml version="1.0"?>',
        '<!-- GENERATED by tools/wrist_blob_gen.py. Do not edit by hand. -->',
        '<robot xmlns:xacro="http://www.ros.org/wiki/xacro">',
        f'  <xacro:macro name="{link_name}_geometry">',
    ]
    for (z0, z1, r) in bands:
        lines += [
            '    <collision>',
            f'      <origin xyz="0 0 {(z0 + z1) / 2:.6f}" rpy="0 0 0"/>',
            f'      <geometry><cylinder radius="{r:.6f}" length="{z1 - z0:.6f}"/></geometry>',
            '    </collision>',
        ]
    lines += ['  </xacro:macro>', '</robot>', '']
    with open(path, "w") as f:
        f.write("\n".join(lines))
    return len(bands)


# ===========================================================================

def main():
    here = os.path.dirname(os.path.abspath(__file__))
    ws_src = os.path.abspath(os.path.join(here, "..", ".."))

    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--urdf", default=os.path.join(
        ws_src, "myarm_300_pi", "urdf", "myarm_300_pi_thorgripper.urdf"))
    ap.add_argument("--pkg-root", action="append", default=[ws_src])
    ap.add_argument("--out-dir", default=os.path.join(here, "..", "generated"))
    ap.add_argument("--tag", default="carry", help="suffix for the output files")

    ap.add_argument("--tilt", type=float, default=0.30,
                    help="tool-axis tilt tolerance in rad (TILT_TOLERANCE)")
    ap.add_argument("--full-so3", action="store_true",
                    help="ignore the constraint and sweep the whole wrist range")
    ap.add_argument("--no-tube", dest="tube", action="store_false",
                    help="omit the carried tube (transit-without-payload blob)")

    ap.add_argument("--n-roll", type=int, default=72)
    ap.add_argument("--n-tilt", type=int, default=3)
    ap.add_argument("--n-tilt-dir", type=int, default=8)
    ap.add_argument("--n-q5", type=int, default=12)
    ap.add_argument("--n-q6", type=int, default=12)
    ap.add_argument("--n-q7", type=int, default=12)
    ap.add_argument("--n-gripper", type=int, default=3)
    ap.add_argument("--gripper", choices=("closed", "open", "range"), default="range",
                    help="hand state to sweep; 'range' covers open..closed")

    ap.add_argument("--bins", type=int, default=40)
    ap.add_argument("--margin", type=float, default=0.005, help="safety padding in m")
    ap.add_argument("--decimate", type=int, default=4000,
                    help="max vertices kept per mesh (0 = keep all)")
    ap.add_argument("--merge-tol", type=float, default=0.004)
    args = ap.parse_args()

    os.makedirs(args.out_dir, exist_ok=True)
    urdf = Urdf(args.urdf)

    print(f"URDF   : {args.urdf}")
    print(f"mode   : {'FULL SO(3) (naive)' if args.full_so3 else f'upright, tilt<={args.tilt} rad'}"
          f"{'' if args.tube else '  [no tube]'}")

    rz, prox_r, stats = sweep(urdf, args, args.pkg_root)
    print(f"sampled: {stats['orientations']} wrist orientations, {len(rz)} surface points")
    print(f"proximal (link5/6) bounding sphere about wrist centre: {prox_r * 1000:.1f} mm")

    edges, r_out, r_in, occupied = build_profile(rz, prox_r, args.bins, args.margin)

    R = float(r_out.max())
    z_lo = float(edges[:-1][occupied].min())
    z_hi = float(edges[1:][occupied].max())
    # Two different questions. The per-bin hole is what the revolved mesh
    # actually keeps; the through-hole is the radius of a vertical cylinder
    # that is free at EVERY height, which is the one that decides whether a
    # tube or the mixer can sit inside the ring without a false collision.
    hole_max = float(r_in[occupied].max()) if occupied.any() else 0.0
    hole_through = float(r_in[occupied].min()) if occupied.any() else 0.0

    # volumes, for the report
    dz = edges[1] - edges[0]
    v_mesh = float(np.sum(np.pi * (r_out[occupied] ** 2 - r_in[occupied] ** 2) * dz))
    v_stack = float(np.sum(np.pi * r_out[occupied] ** 2 * dz))
    v_sphere = 4.0 / 3.0 * math.pi * max(R, abs(z_lo), abs(z_hi)) ** 3

    print()
    print(f"  max radius from wrist centre : {R * 1000:.1f} mm")
    print(f"  height span                  : {z_lo * 1000:.1f} .. {z_hi * 1000:.1f} mm")
    print(f"  inner hole, per-slice max    : {hole_max * 1000:.1f} mm")
    print(f"  inner hole, free through-bore: {hole_through * 1000:.1f} mm")
    print(f"  volume, revolved mesh        : {v_mesh * 1e6:.0f} cm^3")
    print(f"  volume, cylinder stack       : {v_stack * 1e6:.0f} cm^3")
    print(f"  volume, naive bounding sphere: {v_sphere * 1e6:.0f} cm^3"
          f"   ({v_sphere / v_mesh:.1f}x the mesh)")

    stl = os.path.join(args.out_dir, f"wrist_blob_{args.tag}.stl")
    xac = os.path.join(args.out_dir, f"wrist_blob_{args.tag}.urdf.xacro")
    jsn = os.path.join(args.out_dir, f"wrist_blob_{args.tag}.json")

    n_tri = write_revolved_stl(stl, edges, r_out, r_in, occupied)
    n_cyl = write_urdf_snippet(xac, edges, r_out, occupied, "wrist_blob", args.merge_tol)

    with open(jsn, "w") as f:
        json.dump({
            "tag": args.tag,
            "mode": "full_so3" if args.full_so3 else "upright",
            "tilt_tolerance": None if args.full_so3 else args.tilt,
            "with_tube": bool(args.tube),
            "margin": args.margin,
            "max_radius": R,
            "z_min": z_lo, "z_max": z_hi,
            "max_hole_radius": hole_max,
            "through_hole_radius": hole_through,
            "volume_mesh": v_mesh, "volume_stack": v_stack, "volume_sphere": v_sphere,
            "profile": [
                {"z0": float(edges[k]), "z1": float(edges[k + 1]),
                 "r_out": float(r_out[k]), "r_in": float(r_in[k])}
                for k in range(len(r_out)) if occupied[k]
            ],
        }, f, indent=2)

    print()
    print(f"  wrote {stl}  ({n_tri} triangles)")
    print(f"  wrote {xac}  ({n_cyl} cylinders)")
    print(f"  wrote {jsn}")


if __name__ == "__main__":
    sys.exit(main())
