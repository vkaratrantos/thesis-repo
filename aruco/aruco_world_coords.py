#!/usr/bin/env python3
"""
ArUco -> WORLD coordinates, using what you actually know about the scene.

WHAT WAS WRONG BEFORE
---------------------
The previous node published

    marker_base -> camera_link -> marker_N

where `marker_base` was the ANCHOR MARKER'S OWN OpenCV frame. Nothing connected
that to the world/robot frame, so no marker ever had a world coordinate. The
known anchor pose -- the one hard fact in the whole scene -- was never used for
anything except being inverted.

It also estimated a SEPARATE rotation for every marker, then trusted each
marker's own translation. But all seven markers share one orientation. That
means:

  * the rotation is ONE 3-DoF quantity measured 7 x N times, not 7 unknowns.
    Estimating it jointly makes it far better than any single 30 mm marker can
    ever be on its own;
  * once the rotation is FIXED, solving for a marker's translation is a LINEAR
    least-squares problem. The planar pose ambiguity -- the flip this codebase
    spends most of its complexity fighting -- is purely a rotation ambiguity.
    Fix the rotation and it is gone. No ambiguity pool, no LOW CONFIDENCE
    markers, no medoid-vs-EMA argument.

HOW THIS WORKS
--------------
1. Collect N frames. Only frames where the ANCHOR is visible are kept, since a
   frame without the anchor cannot be referenced to the world.

2. Estimate ONE rotation R (marker frame -> camera frame) shared by all
   markers. Both IPPE solutions of every marker in every frame vote; the
   consensus over ~7*150 samples decides the flip, instead of a per-sample
   reprojection-error ratio that is exactly what fails on a head-on view.

3. With R fixed, solve each marker's position per frame by linear least squares
   on its four corners. If COPLANAR is on, solve instead for the marker's
   IN-PLANE offset (a, b) from the anchor directly -- 2 unknowns instead of 3.
   That drops the depth direction entirely, which is where nearly all of a
   small marker's error lives.

4. Express each marker as an offset from the ANCHOR, in the anchor's frame:

       d_i = R^T (t_i - t_anchor)

   Any error common to the whole camera pose -- and camera-pose error IS mostly
   common-mode -- cancels in that difference. That is the entire point of
   having an anchor, and the old node threw it away.

5. Map to the world through the anchor's KNOWN pose:

       p_i_world = ANCHOR_XYZ_WORLD + R_world_anchor @ d_i

   The camera's absolute pose is never trusted; only the relative geometry is.

CONFIGURE TWO THINGS (see below): ANCHOR_XYZ_WORLD, and the anchor's
orientation via MARKER_X_IN_WORLD / MARKER_Y_IN_WORLD.

CONTROLS (in the OpenCV window)
    r : re-scan     q : quit
"""

import json
import os
import sys
import time

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster

# --------------------------------------------------------------------------
# Scene configuration -- THIS is what makes world coordinates possible
# --------------------------------------------------------------------------
WORLD_FRAME = "base"           # robot tree is world -> base (coincident); there
                               # is NO "base_link" frame in this robot
ANCHOR_ID = 0
ANCHOR_XYZ_WORLD = (0.0, -0.07, 0.0)

# The anchor's ORIENTATION, given as where the marker's own axes point in the
# world. A marker's OpenCV frame is:
#     +x  across the face, to the right as you read the marker
#     +y  up the face
#     +z  out of the face, towards the viewer
# So a marker lying FLAT ON A TABLE, face up, with its top edge towards world
# +y, is ("+x", "+y")  -- identity.
# A marker on a VERTICAL rack facing world +x, upright, is ("+y", "+z").
MARKER_X_IN_WORLD = "+x"
MARKER_Y_IN_WORLD = "+y"

# True if all markers sit on ONE flat surface with the anchor. This is a strong
# constraint and worth using when it holds -- it removes the depth error
# entirely. The out-of-plane diagnostic is reported either way, so you can see
# whether the assumption is justified before believing it.
COPLANAR = True

# --------------------------------------------------------------------------
# Camera / detection configuration
# --------------------------------------------------------------------------
MARKER_SIZE = 0.03          # metres. Depth scales LINEARLY with this.
CAMERA_INDEX = 1
CALIB_PROFILE = "4"
REQ_WIDTH, REQ_HEIGHT = 1920, 1080

FRAMES_REQUIRED = 150       # frames WITH THE ANCHOR before solving
SCAN_TIMEOUT_S = 20.0
MIN_FRAMES = 10             # refuse to solve with fewer than this

ROT_OUTLIER_DEG = 12.0      # reject rotation votes this far from consensus
TRANS_OUTLIER_M = 0.02      # reject per-frame offsets this far from the median

OUT_JSON = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        "marker_world_coords.json")

_AXES = {"+x": (1, 0, 0), "-x": (-1, 0, 0),
         "+y": (0, 1, 0), "-y": (0, -1, 0),
         "+z": (0, 0, 1), "-z": (0, 0, -1)}


# --------------------------------------------------------------------------
# Small maths helpers
# --------------------------------------------------------------------------
def rot_from_axes(marker_x, marker_y):
    """Rotation taking a vector in the MARKER frame to the WORLD frame.

    Its columns are the marker's own axes written in world coordinates, which
    is why naming those two axes is enough to pin the orientation down.
    """
    ax = np.array(_AXES[marker_x], dtype=np.float64)
    ay = np.array(_AXES[marker_y], dtype=np.float64)
    if abs(float(ax @ ay)) > 1e-9:
        raise ValueError(f"marker axes {marker_x} and {marker_y} are not "
                         f"perpendicular")
    return np.column_stack([ax, ay, np.cross(ax, ay)])


def mat_to_quat(R):
    """3x3 rotation matrix -> (x, y, z, w). Shepperd's method, branch-stable."""
    t = np.trace(R)
    if t > 0.0:
        s = np.sqrt(t + 1.0) * 2.0
        w, x = 0.25 * s, (R[2, 1] - R[1, 2]) / s
        y, z = (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        w, x = (R[2, 1] - R[1, 2]) / s, 0.25 * s
        y, z = (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        w, x = (R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s
        y, z = 0.25 * s, (R[1, 2] + R[2, 1]) / s
    else:
        s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        w, x = (R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s
        y, z = (R[1, 2] + R[2, 1]) / s, 0.25 * s
    q = np.array([x, y, z, w], dtype=np.float64)
    return q / np.linalg.norm(q)


def quat_to_mat(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w)],
        [2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y)],
    ], dtype=np.float64)


def rvec_to_quat(rvec):
    R, _ = cv2.Rodrigues(np.asarray(rvec, dtype=np.float64).reshape(3, 1))
    return mat_to_quat(R)


def quat_angle_deg(a, b):
    """Angle between two orientations, in degrees. Sign-insensitive."""
    d = abs(float(np.dot(a, b)))
    return float(np.degrees(2.0 * np.arccos(np.clip(d, -1.0, 1.0))))


def average_quaternions(qs):
    """Markley's eigenvector average. Handles the q/-q sign ambiguity itself."""
    Q = np.asarray(qs, dtype=np.float64)
    _, vecs = np.linalg.eigh(Q.T @ Q)
    q = vecs[:, -1]
    return q / np.linalg.norm(q)


def quat_medoid(Q):
    """The sample with the smallest total angular distance to all the others.

    Vectorised, because with 7 markers x 150 frames the pairwise loop the old
    node used would be a million Python-level arccos calls.
    """
    C = np.clip(np.abs(np.asarray(Q) @ np.asarray(Q).T), -1.0, 1.0)
    return Q[int(np.argmin((2.0 * np.arccos(C)).sum(axis=1)))]


# --------------------------------------------------------------------------
# The two linear solves. Both take the rotation as GIVEN.
# --------------------------------------------------------------------------
def solve_translation(R, obj, npts, iters=2):
    """Least-squares marker position t, given a fixed rotation R.

    With X = R p + t and normalised image point (u, v) = (X0/X2, X1/X2), each
    corner contributes two equations that are LINEAR in t:

        t0 - u*t2 = -(Rp)0 + u*(Rp)2
        t1 - v*t2 = -(Rp)1 + v*(Rp)2

    The raw form minimises an algebraic error weighted by depth; one reweight
    by 1/z turns it into the reprojection error to first order.
    """
    rp = (R @ np.asarray(obj, dtype=np.float64).T).T
    n = len(rp)
    w = np.ones(n)
    t = np.zeros(3)
    for _ in range(iters):
        A = np.zeros((2 * n, 3))
        b = np.zeros(2 * n)
        for k in range(n):
            u, v = npts[k]
            p = rp[k]
            A[2 * k] = (w[k], 0.0, -u * w[k])
            b[2 * k] = w[k] * (-p[0] + u * p[2])
            A[2 * k + 1] = (0.0, w[k], -v * w[k])
            b[2 * k + 1] = w[k] * (-p[1] + v * p[2])
        t = np.linalg.lstsq(A, b, rcond=None)[0]
        z = rp[:, 2] + t[2]
        if np.any(z <= 1e-6):
            return None
        w = 1.0 / z
    return t


def solve_inplane(R, obj, npts, t_anchor, iters=2):
    """In-plane offset (a, b) from the anchor, given fixed R and the anchor's
    position in this frame.

    Writing t = t_anchor + a*R[:,0] + b*R[:,1] bakes "this marker is on the
    anchor's plane" into the model, leaving TWO unknowns instead of three. The
    discarded direction is the plane normal -- the depth-like direction that
    carries almost all of a 30 mm marker's error. What comes out is exactly the
    quantity we want anyway: the offset from the anchor, in the anchor's frame.
    """
    e0, e1 = R[:, 0], R[:, 1]
    rp = (R @ np.asarray(obj, dtype=np.float64).T).T + np.asarray(t_anchor)
    n = len(rp)
    w = np.ones(n)
    ab = np.zeros(2)
    for _ in range(iters):
        A = np.zeros((2 * n, 2))
        b = np.zeros(2 * n)
        for k in range(n):
            u, v = npts[k]
            p = rp[k]
            A[2 * k] = (w[k] * (e0[0] - u * e0[2]), w[k] * (e1[0] - u * e1[2]))
            b[2 * k] = -w[k] * (p[0] - u * p[2])
            A[2 * k + 1] = (w[k] * (e0[1] - v * e0[2]), w[k] * (e1[1] - v * e1[2]))
            b[2 * k + 1] = -w[k] * (p[1] - v * p[2])
        ab = np.linalg.lstsq(A, b, rcond=None)[0]
        z = rp[:, 2] + ab[0] * e0[2] + ab[1] * e1[2]
        if np.any(z <= 1e-6):
            return None
        w = 1.0 / z
    return ab


def robust_median(V):
    """Median with one outlier-rejection pass. Returns (median, spread_mm, n)."""
    V = np.asarray(V, dtype=np.float64)
    med = np.median(V, axis=0)
    keep = np.linalg.norm(V - med, axis=1) < TRANS_OUTLIER_M
    if keep.sum() >= 3:
        V = V[keep]
    return (np.median(V, axis=0),
            float(np.max(np.std(V, axis=0)) * 1000.0),
            int(len(V)))


# --------------------------------------------------------------------------
# Node
# --------------------------------------------------------------------------
class ArucoWorldCoords(Node):
    def __init__(self):
        super().__init__("aruco_world_coords")

        here = os.path.dirname(os.path.abspath(__file__))
        json_file = os.path.join(here, "camera_params.json")
        if not os.path.exists(json_file):
            self.get_logger().error(f"{json_file} not found")
            sys.exit(1)
        with open(json_file) as f:
            params = json.load(f)["camera_parameters"][CALIB_PROFILE]
        self.K = np.array(params["camera_matrix"], dtype=np.float64)
        self.D = np.array(params["dist_coeffs"], dtype=np.float64)

        self.R_wa = rot_from_axes(MARKER_X_IN_WORLD, MARKER_Y_IN_WORLD)
        self.q_wa = mat_to_quat(self.R_wa)
        self.p_wa = np.array(ANCHOR_XYZ_WORLD, dtype=np.float64)

        self.cap = cv2.VideoCapture(CAMERA_INDEX)
        if not self.cap.isOpened():
            self.get_logger().error(f"could not open camera index {CAMERA_INDEX}")
            sys.exit(1)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, REQ_WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, REQ_HEIGHT)
        self._verify_resolution()

        det_params = cv2.aruco.DetectorParameters()
        det_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.detector = cv2.aruco.ArucoDetector(
            cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50), det_params)

        # Marker-frame corners in OpenCV's detection order (TL, TR, BR, BL).
        s = MARKER_SIZE / 2.0
        self.obj_pts = np.array([[-s, s, 0], [s, s, 0], [s, -s, 0], [-s, -s, 0]],
                                dtype=np.float64)

        self.static_bc = StaticTransformBroadcaster(self)
        self.start_scan()
        self.timer = self.create_timer(0.03, self.loop)

    # ------------------------------------------------------------------
    def _verify_resolution(self):
        w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        implied_w, implied_h = self.K[0, 2] * 2.0, self.K[1, 2] * 2.0
        self.get_logger().info(
            f"capture {w}x{h}; calibration '{CALIB_PROFILE}' implies "
            f"~{implied_w:.0f}x{implied_h:.0f}")
        if abs(w - implied_w) / max(implied_w, 1) > 0.15:
            self.get_logger().error(
                f"RESOLUTION MISMATCH: capturing {w}x{h} but the calibration is "
                f"for ~{implied_w:.0f}x{implied_h:.0f}. Every distance would be "
                f"wrong by ~{implied_w / max(w, 1):.2f}x.")
            sys.exit(1)

    def _normalise(self, corners):
        pts = np.asarray(corners, dtype=np.float32).reshape(-1, 1, 2)
        return cv2.undistortPoints(pts, self.K, self.D).reshape(-1, 2).astype(np.float64)

    # ------------------------------------------------------------------
    def start_scan(self):
        self.frames = []        # [{id: corners(4,2)}], anchor guaranteed present
        self.seen = {}          # id -> frames detected in
        self.result = None
        self.state = "SCANNING"
        self.scan_started = time.time()
        self.get_logger().info("scanning -- hold the scene still")

    def loop(self):
        ok, frame = self.cap.read()
        if not ok:
            return

        corners, ids, _ = self.detector.detectMarkers(
            cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))

        if ids is not None:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            if self.state == "SCANNING":
                obs = {int(i): np.asarray(c, dtype=np.float64).reshape(4, 2)
                       for c, i in zip(corners, ids.ravel())}
                for i in obs:
                    self.seen[i] = self.seen.get(i, 0) + 1
                # A frame without the anchor has nothing to reference against,
                # so it is not merely less useful -- it is unusable.
                if ANCHOR_ID in obs:
                    self.frames.append(obs)

        if self.state == "SCANNING":
            elapsed = time.time() - self.scan_started
            if len(self.frames) >= FRAMES_REQUIRED:
                self.solve()
            elif elapsed > SCAN_TIMEOUT_S:
                if len(self.frames) >= MIN_FRAMES:
                    self.get_logger().warn(
                        f"timed out with {len(self.frames)} anchor frames -- "
                        f"solving anyway")
                    self.solve()
                else:
                    self.get_logger().warn(
                        f"only {len(self.frames)} frames with anchor "
                        f"{ANCHOR_ID} in {elapsed:.0f}s -- the anchor MUST be "
                        f"visible. Still scanning.")
                    self.scan_started = time.time()
        else:
            self._draw_solution(frame)

        self._overlay(frame)
        cv2.imshow("ArUco world coords (r = rescan, q = quit)", frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord("r"):
            self.start_scan()
        elif key == ord("q"):
            rclpy.shutdown()

    # ------------------------------------------------------------------
    def _common_rotation(self):
        """One rotation for all markers, decided by consensus across every
        marker and every frame.

        Each detection offers two IPPE solutions. Rather than picking per
        sample on a reprojection-error ratio -- which is precisely what a
        head-on view makes undecidable -- each sample contributes whichever of
        its solutions agrees with the running consensus. One badly-conditioned
        marker cannot outvote the other six.
        """
        cand = []       # [(marker_id, [quat, ...])]
        for f in self.frames:
            for i, c in f.items():
                img = np.asarray(c, dtype=np.float32).reshape(4, 2)
                try:
                    n, rvecs, _, errs = cv2.solvePnPGeneric(
                        self.obj_pts.astype(np.float32), img, self.K, self.D,
                        flags=cv2.SOLVEPNP_IPPE_SQUARE)
                except cv2.error:
                    continue
                if n < 1:
                    continue
                order = np.argsort(np.asarray(errs).ravel())
                cand.append((i, [rvec_to_quat(rvecs[k]) for k in order]))

        if not cand:
            return None, None, None

        ref = quat_medoid(np.array([qs[0] for _, qs in cand]))
        chosen = []
        for _ in range(10):
            chosen = []
            for _, qs in cand:
                q = max(qs, key=lambda x: abs(float(np.dot(x, ref))))
                chosen.append(q if float(np.dot(q, ref)) >= 0 else -q)
            Qc = np.array(chosen)
            dev = np.degrees(2.0 * np.arccos(
                np.clip(np.abs(Qc @ ref), -1.0, 1.0)))
            keep = dev < ROT_OUTLIER_DEG
            Qk = Qc[keep] if keep.sum() >= max(3, int(0.2 * len(Qc))) else Qc
            new = average_quaternions(Qk)
            if float(np.dot(new, ref)) < 0:
                new = -new
            moved = quat_angle_deg(new, ref)
            ref = new
            if moved < 1e-4:
                break

        # Per-marker deviation from the consensus: the test of "they all have
        # the same orientation". A marker several degrees out is either mounted
        # crooked or is being read from a bad viewpoint, and either way its
        # position estimate deserves suspicion.
        per_marker = {}
        for mid in sorted({i for i, _ in cand}):
            Qi = np.array([q for (i, _), q in zip(cand, chosen) if i == mid])
            per_marker[mid] = quat_angle_deg(quat_medoid(Qi), ref)

        spread = float(np.percentile(
            [quat_angle_deg(q, ref) for q in chosen], 90))
        return ref, per_marker, spread

    # ------------------------------------------------------------------
    def solve(self):
        q_cm, rot_dev, rot_spread = self._common_rotation()
        if q_cm is None:
            self.get_logger().error("no usable detections -- nothing solved")
            self.state = "FAILED"
            return
        R = quat_to_mat(q_cm)

        offsets = {}      # id -> [d in anchor frame]
        out_plane = {}    # id -> [signed distance off the anchor plane]
        anchor_t = []     # anchor position in camera frame, per frame

        for f in self.frames:
            npts = {i: self._normalise(c) for i, c in f.items()}
            t_a = solve_translation(R, self.obj_pts, npts[ANCHOR_ID])
            if t_a is None or t_a[2] <= 0:
                continue
            anchor_t.append(t_a)
            offsets.setdefault(ANCHOR_ID, []).append(np.zeros(3))
            out_plane.setdefault(ANCHOR_ID, []).append(0.0)

            for i, p in npts.items():
                if i == ANCHOR_ID:
                    continue
                # The free solve is computed regardless: even when the plane
                # constraint is applied, its out-of-plane component is the
                # honest measure of how well the flat-scene assumption holds.
                t_i = solve_translation(R, self.obj_pts, p)
                if t_i is None or t_i[2] <= 0:
                    continue
                d_free = R.T @ (t_i - t_a)
                if COPLANAR:
                    ab = solve_inplane(R, self.obj_pts, p, t_a)
                    if ab is None:
                        continue
                    d = np.array([ab[0], ab[1], 0.0])
                else:
                    d = d_free
                offsets.setdefault(i, []).append(d)
                out_plane.setdefault(i, []).append(float(d_free[2]))

        if ANCHOR_ID not in offsets or len(anchor_t) < MIN_FRAMES:
            self.get_logger().error(
                f"anchor solved in only {len(anchor_t)} frames -- not enough")
            self.state = "FAILED"
            return

        t_a_med = np.median(np.array(anchor_t), axis=0)

        markers = {}
        for i in sorted(offsets):
            if len(offsets[i]) < MIN_FRAMES:
                self.get_logger().warn(
                    f"marker {i}: only {len(offsets[i])} usable frames -- skipped")
                continue
            d, noise_mm, n_used = robust_median(offsets[i])
            markers[i] = {
                "d": d,                                     # anchor frame
                "xyz": self.p_wa + self.R_wa @ d,           # world frame
                "noise_mm": noise_mm,
                "n": len(offsets[i]),
                "n_used": n_used,
                "off_plane_mm": float(np.median(out_plane[i]) * 1000.0),
                "rot_dev_deg": rot_dev.get(i, float("nan")),
            }

        # Camera pose in world, purely for visualisation and sanity: it is the
        # anchor's known pose composed with the measured anchor->camera pose.
        R_wc = self.R_wa @ R.T
        p_wc = self.p_wa + self.R_wa @ (-R.T @ t_a_med)

        self.result = {"R_cm": R, "q_cm": q_cm, "t_anchor_cam": t_a_med,
                       "markers": markers, "rot_spread": rot_spread,
                       "R_wc": R_wc, "p_wc": p_wc}
        self.state = "SOLVED"
        self._report()
        self._publish()

    # ------------------------------------------------------------------
    def _report(self):
        r = self.result
        m = r["markers"]
        lines = [
            "",
            f"world frame '{WORLD_FRAME}', anchor {ANCHOR_ID} at "
            f"({self.p_wa[0]:.4f}, {self.p_wa[1]:.4f}, {self.p_wa[2]:.4f}), "
            f"orientation marker-x->{MARKER_X_IN_WORLD} marker-y->{MARKER_Y_IN_WORLD}",
            f"common rotation solved from {len(self.frames)} frames; "
            f"90th-pct sample spread {r['rot_spread']:.2f} deg"
            f"{'  [COPLANAR constraint ON]' if COPLANAR else ''}",
            "",
            "  id   frames  used     X        Y        Z     noise  offplane  rotdev",
            "                       (m)      (m)      (m)     (mm)     (mm)    (deg)",
        ]
        for i in sorted(m):
            d = m[i]
            x, y, z = d["xyz"]
            tag = "  <- anchor (known)" if i == ANCHOR_ID else ""
            lines.append(
                f"  {i:<4} {d['n']:>6} {d['n_used']:>5} "
                f"{x:>8.4f} {y:>8.4f} {z:>8.4f} "
                f"{d['noise_mm']:>7.2f} {d['off_plane_mm']:>8.2f} "
                f"{d['rot_dev_deg']:>7.2f}{tag}")
        lines += [
            "",
            "  noise    frame-to-frame scatter. Bounds the RANDOM error only;",
            "           anything worse than this in reality is BIAS (marker size,",
            "           calibration, or a wrong anchor pose), not filtering.",
            "  offplane distance off the anchor's plane, measured WITHOUT the",
            "           coplanar constraint. If the markers really are on one",
            "           surface this is your depth error -- and with COPLANAR on",
            "           it is exactly what has been removed from the answer.",
            "  rotdev   this marker's rotation vs the shared one. Large means the",
            "           'all same orientation' assumption fails for it.",
        ]
        self.get_logger().info("\n".join(lines))

        missing = sorted(set(self.seen) - set(m))
        if missing:
            self.get_logger().error(
                f"markers {missing} were DETECTED but not solved -- not published")
        bad_rot = sorted(i for i in m if m[i]["rot_dev_deg"] > 5.0)
        if bad_rot:
            self.get_logger().warn(
                f"markers {bad_rot} deviate >5 deg from the shared orientation. "
                f"Either they are not mounted parallel, or they are seen too "
                f"head-on to read reliably.")
        big_off = sorted(i for i in m if abs(m[i]["off_plane_mm"]) > 10.0)
        if COPLANAR and big_off:
            self.get_logger().warn(
                f"markers {big_off} sit >10 mm off the anchor plane when solved "
                f"freely. If they ARE coplanar this is depth error and the "
                f"constraint is doing its job; if they are NOT, set "
                f"COPLANAR = False.")

        dump = {
            "world_frame": WORLD_FRAME,
            "anchor_id": ANCHOR_ID,
            "anchor_xyz_world": list(map(float, self.p_wa)),
            "orientation_quat_xyzw": [float(v) for v in self.q_wa],
            "coplanar_constraint": COPLANAR,
            "frames_used": len(self.frames),
            "markers": {
                str(i): {
                    "xyz": [float(v) for v in m[i]["xyz"]],
                    "offset_in_anchor_frame": [float(v) for v in m[i]["d"]],
                    "noise_mm": m[i]["noise_mm"],
                    "off_plane_mm": m[i]["off_plane_mm"],
                    "rot_dev_deg": m[i]["rot_dev_deg"],
                    "frames": m[i]["n"],
                } for i in sorted(m)
            },
        }
        with open(OUT_JSON, "w") as f:
            json.dump(dump, f, indent=2)
        self.get_logger().info(f"wrote {OUT_JSON}")

    # ------------------------------------------------------------------
    def _publish(self):
        stamp = self.get_clock().now().to_msg()
        out = [self._tf(stamp, WORLD_FRAME, "camera_link",
                        self.result["p_wc"], mat_to_quat(self.result["R_wc"]))]
        # Every marker is a direct child of the world frame, with the anchor's
        # known orientation -- not a child of the camera. Nothing downstream
        # has to route through an estimated camera pose to get a coordinate.
        for i, d in self.result["markers"].items():
            out.append(self._tf(stamp, WORLD_FRAME, f"marker_{i}",
                                d["xyz"], self.q_wa))
        self.static_bc.sendTransform(out)
        self.get_logger().info(
            f"published {len(out)} static transforms "
            f"({len(out) - 1} markers + camera) in '{WORLD_FRAME}'")

    @staticmethod
    def _tf(stamp, parent, child, t, q):
        m = TransformStamped()
        m.header.stamp = stamp
        m.header.frame_id = parent
        m.child_frame_id = child
        m.transform.translation.x = float(t[0])
        m.transform.translation.y = float(t[1])
        m.transform.translation.z = float(t[2])
        m.transform.rotation.x = float(q[0])
        m.transform.rotation.y = float(q[1])
        m.transform.rotation.z = float(q[2])
        m.transform.rotation.w = float(q[3])
        return m

    # ------------------------------------------------------------------
    def _draw_solution(self, frame):
        """Draw the SOLVED pose, so a wrong answer is visible against the live
        green outline instead of hidden behind it."""
        if not self.result:
            return
        R = self.result["R_cm"]
        rvec, _ = cv2.Rodrigues(R)
        t_a = self.result["t_anchor_cam"]
        for d in self.result["markers"].values():
            t = t_a + R @ d["d"]
            try:
                cv2.drawFrameAxes(frame, self.K, self.D, rvec,
                                  t.reshape(3, 1), MARKER_SIZE * 0.8)
            except cv2.error:
                pass

    def _overlay(self, frame):
        h = frame.shape[0]
        if self.state == "SCANNING":
            txt = [f"SCANNING  {len(self.frames)}/{FRAMES_REQUIRED} anchor frames"
                   f"  ({time.time() - self.scan_started:.1f}s)"]
            for i in sorted(self.seen):
                txt.append(f"  id {i}: seen {self.seen[i]}")
            if ANCHOR_ID not in self.seen:
                txt.append(f"  ANCHOR {ANCHOR_ID} NOT VISIBLE - nothing can be solved")
            colour = (0, 200, 255)
        elif self.state == "SOLVED":
            txt = [f"SOLVED in '{WORLD_FRAME}'  (r = rescan)"]
            for i in sorted(self.result["markers"]):
                d = self.result["markers"][i]
                x, y, z = d["xyz"]
                txt.append(f"  id {i}: {x:+.3f} {y:+.3f} {z:+.3f} m"
                           f"   +/-{d['noise_mm']:.1f} mm")
            for i in sorted(set(self.seen) - set(self.result["markers"])):
                txt.append(f"  id {i}: NOT SOLVED - not published")
            colour = (0, 255, 0)
        else:
            txt = ["FAILED - see log  (r = rescan)"]
            colour = (0, 0, 255)

        y = h - 20 - 22 * len(txt)
        for line in txt:
            cv2.putText(frame, line, (12, y), cv2.FONT_HERSHEY_SIMPLEX,
                        0.6, (0, 0, 0), 4, cv2.LINE_AA)
            cv2.putText(frame, line, (12, y), cv2.FONT_HERSHEY_SIMPLEX,
                        0.6, colour, 1, cv2.LINE_AA)
            y += 22


def main():
    rclpy.init()
    node = ArucoWorldCoords()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cap.release()
        cv2.destroyAllWindows()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
