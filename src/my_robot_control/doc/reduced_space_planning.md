# Reduced-search-space motion planning for the MyArm 300 Pi

Technical record of the second-approach planning work: what was built, what was
measured, and what is still open.

Robot: Elephant Robotics MyArm 300 Pi, 7 DOF, ROS 2 Humble, MoveIt 2.5.9.
Task: liquid transfer — pick a test tube from a rack, carry it upright, pour
into a mixer. The orientation constraint exists because the tube holds liquid.

---

## 1. Motivation

The existing pipeline plans transits with OMPL RRTConnect over all seven joints,
under an orientation path constraint that keeps the tube upright
(`uprightConstraint()` in `simple_move.cpp`). It is slow and unreliable.

Measured on 30 task transits at the arm's own stations (see §6):

| | success | planning time | worst tool tilt |
|---|---|---|---|
| 7-DOF, no constraint (control) | 96.7 % | 0.66 s | 3.057 rad |
| 7-DOF, upright constraint | 60.0 % | 10.03 s | 0.418 rad |

The constraint alone costs a **15× slowdown and 37 points of success rate**. The
control column exists to establish that: without the constraint the same queries
are easy, so the difficulty is the constraint and not the scene, the robot or
the configuration.

The 0.418 rad figure is also worth noting on its own — the current pipeline
produces transits that tilt the tube up to 24°, against a nominal tolerance of
0.30 rad.

---

## 2. Kinematic structure

Everything that follows depends on one property of the arm, verified
numerically against the URDF rather than assumed.

**The arm is S-R-S** (spherical shoulder, revolute elbow, spherical wrist):

- Joints 1, 2, 3 intersect at a single point — the shoulder, at
  `(0, 0, 0.1695)`. Pairwise common-normal distance between all three axes: 0.
- Joints 5, 6, 7 intersect at a single point — the wrist centre. Same test,
  same result: 0.

Link lengths follow: upper arm `L1 = 0.1155 m`, forearm `L2 = 0.1278 m`, so the
wrist centre lies between 0.1347 m and 0.2433 m from the shoulder.

Two consequences carry the whole approach:

1. **The wrist centre is a function of `q1..q4` alone.** Joints 5–7 change
   orientation and nothing else. Position planning is therefore a 4-DOF problem.
2. **`|W − S|` depends on `q4` alone**, because a spherical shoulder can rotate
   the upper-arm/forearm triangle but never change its shape. The elbow angle
   falls out of one law of cosines.

Folding in the URDF's fixed rpy offsets, both joint triples turn out to be ZYZ
Euler sets:

```
R_shoulder = Rz(q1) Ry(q2) Rz(q3)
R_wrist    = Rz(q5) Ry(-q6) Rz(q7)
```

so both invert in closed form. Validation:

| check | result |
|---|---|
| analytic FK vs MoveIt FK (wrist centre) | max error 1.3 µm |
| position IK round-trip | 5000 / 5000, residual 0 m |
| wrist ZYZ IK round-trip | 3000 / 3000, residual 1e-5 |

The 1.3 µm residual is **not** an approximation in the derivation. The URDF
writes every right angle as `1.5708`, which is 3.7e-6 rad short of π/2. Replaying
the same FK with exact π/2 brings the disagreement to 1e-16 m. The closed form is
exact; the URDF is rounded.

**Joint 3 is the redundancy.** Locking it costs almost nothing: voxelising the
reachable wrist-centre set at 15 mm over 3 M samples gives 16312 voxels with
`q3` free and 15655 with `q3 = 0` — a **5 % loss**, all at the workspace
boundary. So `{q1, q2, q4}` is a genuine 3-DOF positioner.

---

## 3. Approach A — swept wrist volume (4 DOF)

This is the professor's suggestion implemented literally: reduce the search to
joints 1–4 and replace everything distal of the wrist centre with a single rigid
body enclosing every pose those parts can take.

### 3.1 Construction

`tools/wrist_blob_gen.py` sweeps the real link5/6/7, flange, gripper and tube
geometry out of the URDF and emits a solid of revolution.

The naive reading — sweep *all* wrist orientations — gives a sphere of radius
~0.27 m on an arm whose entire reach is 0.2433 m. Useless: it collides with the
table everywhere.

The task's own constraint rescues it. While carrying, the tool axis must stay
vertical, which leaves exactly **one** free degree of freedom: roll about that
axis. Sweeping only that gives a flat **annulus** rather than a sphere.

Crucially the annulus is a solid of revolution about the **world** vertical, not
about any axis fixed in link 4. So in the reduced model it is attached through
two continuous "align" joints, outside the planning group, which the validity
checker sets at every state so the blob's axis stays on world z. Welding it to
link 4 would be wrong and would force the useless sphere.

### 3.2 Measured geometry

| | constraint-aware (tilt 0.30) | naive full SO(3) |
|---|---|---|
| max radius from wrist centre | 236.6 mm | 238.2 mm |
| height span | 250 mm | 407 mm |
| volume | 26.4 L | 49.3 L |

The constraint buys **1.9× less volume**, essentially all of it vertical — the
radius is unchanged, because the ring's width is set by the gripper-plus-tube
reach, which no constraint can shrink.

Sensitivity to the tilt tolerance:

| tilt tolerance | max radius | volume |
|---|---|---|
| 0.05 rad | 222.5 mm | 16.3 L |
| 0.10 rad | 226.4 mm | 20.0 L |
| 0.20 rad | 232.6 mm | 23.0 L |
| 0.30 rad | 236.6 mm | 26.3 L |

Keeping the ring's central hole matters too. Emitting the blob as a revolved
mesh rather than a solid cylinder stack recovers a third of the lost workspace
in the band that matters: at wrist height 0.28 m, 61.6 % → 75.0 %; at 0.30 m,
81.5 % → 91.0 %.

### 3.3 Result: not viable on this robot

Workspace admitted by the blob, by wrist height:

| wrist z | admitted (4-DOF) | admitted (3-DOF) |
|---|---|---|
| 0.18 m | 0 % | 0 % |
| 0.20 m | 54 % | 55 % |
| 0.26 m | 66 % | 66 % |
| 0.30 m | 91 % | 91 % |
| ≥ 0.32 m | 100 % | 100 % |

In isolation the planner is fast — 2.6 ms for 4-DOF, 1.5 ms for 3-DOF — and the
3-DOF variant costs essentially nothing relative to 4-DOF, which answers the
"3 or 4?" question directly.

But on the **actual task transits** it scored **0 / 30**.

The reason is geometric and specific to this arm. The gripper plus carried tube
reaches **201 mm** from the wrist centre, on an arm with **243 mm** of total
reach. The swept ring is therefore nearly as wide as the robot. Parked above the
tube rack it always contains a neighbouring tube, and every configuration is
rejected.

This is a real, quantified negative result: the rigid-body surrogate is sound in
principle and not viable at this robot's scale. A larger arm, or a shorter
end-effector, would change the conclusion.

---

## 4. Approach B — exact 5-DOF manifold planning

The fix is to stop *bounding* the wrist's remaining freedom and start
*searching* it.

The roll was never genuinely unknown — only unknown to a 4-DOF planner. Add it
as a fifth search dimension and the whole robot becomes determined:

```
q1..q4     ->  wrist centre                      (S-R-S decomposition)
phi        ->  tool orientation = upright(phi)
q5,q6,q7   ->  closed-form ZYZ inversion of that orientation
```

Every point of the 5-D space maps to a complete 7-DOF configuration. That buys
three things at once:

- Collision checking sees the **real** gripper and tube. No surrogate, no
  conservatism, nothing engulfing the rack.
- The upright constraint holds **exactly**, by construction, at every sample —
  not approximately by rejection sampling or numerical projection.
- The search is still 5-dimensional rather than 7. Locking the arm angle gives
  4, which is the professor's lower number.

This is planning directly on the constraint manifold using an exact analytic
chart. MoveIt's `ConstrainedPlanningStateSpace` attempts the same thing
numerically by projection; on this arm that path succeeds 60 % of the time.

### 4.1 Results

Same 30 task transits, same scene, same carried tube:

| | success | planning time | tool tilt |
|---|---|---|---|
| swept volume (4-DOF) | 0 % | — | — |
| 7-DOF constrained | 60 % | ~10 000 ms | up to 0.418 rad |
| **5-DOF manifold** | **100 %** | **~31 ms** | **0.000 rad** |

Stable across heights, and the 4-D variant is faster again:

| transit z | 5-D | 4-D (arm angle locked) |
|---|---|---|
| 0.20 m | 100 %, 33 ms | 100 %, 26 ms |
| 0.24 m | 100 %, 29 ms | 100 %, 17 ms |
| 0.28 m | 100 %, 32 ms | 100 %, 18 ms |

Note the 0.20 m row: that is *below* the height at which the swept-volume
approach can operate at all.

On the live task through the fake-driver stack, carrying transits plan in
20–140 ms with a planned tilt of 0.0002–0.0005° and a measured executed tilt of
0.001–0.94°.

It also fails fast: when a goal is genuinely impossible the manifold planner
reports it in ~20 ms, where the 7-DOF ladder spends ~3 s working through its
tight, loose and verified rungs to reach the same conclusion.

---

## 5. Implementation issues worth recording

Two are genuine subtleties of planning on an analytic chart, and both produced
symptoms that looked like something else entirely.

### 5.1 The 2π wrist wrap

`q5` and `q7` come from `atan2`, so they live in (−π, π] and wrap. Across that
wrap the **orientation** is perfectly continuous — `Rz(q)` and `Rz(q − 2π)` are
the same rotation — but the **joint value** jumps by 2π, and a controller
replaying those waypoints executes a full 360° wrist spin, sweeping the tube
through every orientation on the way.

The symptom: the planner honestly reported 0.0005° of tilt while the arm
visibly tumbled through 173°. It was right about every waypoint and wrong about
the motion *between* two of them.

Fix: unwrap the wrist joints against the previous waypoint, with a joint-limit
check. Measured effect on the same move: **173.7° → 8.9°**.

### 5.2 The joint-7 seam

Joint 7's limit is exactly ±π, so a path crossing that seam asks it to rotate
through the one point it cannot reach. Checking this only at extraction time —
after the search — means throwing away a finished plan and falling back.

Fix: a custom `ompl::base::MotionValidator` that walks each candidate edge,
unwraps the wrist against the previous state, and rejects the edge if that would
leave the joint limits. The seam becomes part of the search and RRTConnect
routes around it. Cost ~2–3× in planning time (89 ms vs 31 ms offline), and it
recovers the cases that previously fell back. Measured effect on a carrying
transit: **19.2° → 0.001°**.

### 5.3 A quarter-turn convention error

The wrist-centre-to-flange direction is `(sin φ, −cos φ, 0)`, **not**
`(cos φ, sin φ, 0)` — the base upright orientation `RPY(0, −π/2, π/2)` maps the
tool's z axis onto world −y. Getting this wrong is silent when φ is swept over a
full circle, because the *set* of wrist centres is unchanged. It only bites when
a specific φ must correspond to a specific wrist centre, which is exactly what a
planner carrying φ as a state dimension needs. There is now a single
`flangeDirection()` helper so the convention cannot drift again.

### 5.4 Unstable wrist branch index

`wristIk()` drops out-of-limit ZYZ branches, so `sols[0]` can be the positive
branch at one state and the negative branch at a neighbour. With the roll as a
state dimension that makes the state→configuration map discontinuous. Added
`wristIkBranch()` with a stable index; the planner fixes the branch once per
plan from the start state.

---

## 6. Benchmark methodology

The comparison in §1 and §4.1 is only meaningful because of four corrections,
each of which would otherwise have produced a flattering but false result.

**Infeasible queries.** The first query set used transit heights up to 0.32 m,
where the tube stations are past the arm's 0.2433 m reach. Both methods fail
there — but the reduced planner rejects in microseconds during goal IK while
`move_group` burns its entire budget first. That manufactures a large fake
speedup. Both endpoints must now be provably reachable and collision-free on the
full model, or the pair is dropped.

**An empty world.** `move_group`'s planning scene starts empty; it is
`simple_move` that publishes the table, tubes and mixer. Benchmarking without
publishing them had the 7-DOF side planning through open air while the reduced
side planned against real obstacles. The benchmark now publishes the scene and
attaches the carried tube before snapshotting.

**A handicapped baseline.** `simple_move.cpp` configures
`setGoalPositionTolerance(0.005)`, `setGoalOrientationTolerance(0.02)` and
`setNumPlanningAttempts(5)`. Running the baseline on MoveIt's defaults — 1e-4 m
and a single attempt — makes it look far worse than it is. Those are now the
benchmark defaults.

**A control column.** `7dof-nocon` runs the same queries with no orientation
constraint. It is not a candidate method — it is free to tip the tube, and does,
reaching 3.057 rad. It exists to separate "the constraint is what makes this
hard" from "the benchmark is misconfigured". At 96.7 % and 0.66 s it settles
that question.

One further check: MoveIt's state-space selector was instrumented at DEBUG level
to confirm `enforce_constrained_state_space: true` is actually taking effect.
It logs `ConstrainedPlanningJointModel` on every constrained request and
`JointModel` on the control, so the baseline is genuinely running the
constrained state space and not silently falling back to rejection sampling.

---

## 7. What is deployed

`transitUpright()` in `simple_move.cpp` dispatches: the 5-DOF manifold planner
first, `moveFreeSpaceUpright()` (the original constrained 7-DOF pipeline)
unchanged as the fallback. Four call sites. `moveFreeSpaceUpright()` itself is
untouched.

Architecturally it cannot be worse than the original: a manifold failure costs
~20 ms before falling back to exactly the previous behaviour.

Runtime switches:

| parameter | effect |
|---|---|
| `use_manifold:=false` | restores the original behaviour exactly |
| `manifold_lock_arm_angle:=true` | 4-D variant (locks joint 3) |
| `manifold_time:=<s>` | planning budget, default 2.0 |

Orientation-keeping applies **while carrying only**, which is the case that can
spill.

---

## 8. Open item

Extending orientation-keeping to empty-gripper transits was implemented and
**reverted**. The transit itself succeeded, but the arm arrived at the standoff
in a configuration from which the subsequent straight-line descent stalled
between 10 % and 80 % of the way down.

The cause is an invariant in `approachTarget()`: `standoff_state` is computed by
seeding IK from `grasp_state`, so the standoff configuration is deliberately
close to the grasp configuration and the descent is a short joint move. Planning
to the standoff **pose** instead leaves the choice of configuration to the
planner, and a different configuration at the same pose can be one the descent
cannot start from.

Attempts that did **not** fix it: exact goal tolerances; filtering goal
candidates to within 0.6 rad and then 0.12 rad of `standoff_state`; a
single-goal `planToConfiguration()`. An apparent pattern — "1 goal state →
descent succeeds, 627 → fails" — turned out to be noise; a later run showed a
single goal state failing at 10.6 %.

The next step is diagnostic, not a code change: set
`CollisionRequest::contacts = true` and dump the contact pairs at the first
blocked descent waypoint, to establish what is actually colliding. That was
never established, and three fixes were built on a hypothesis about it instead.

This is cosmetic for the task — an empty gripper has nothing to spill.

---

## 9. File map

| path | role |
|---|---|
| `tools/wrist_blob_gen.py` | sweeps the wrist assembly, emits the blob |
| `tools/make_reduced_model.py` | generates the reduced URDF/SRDF and the RViz preview model |
| `include/.../srs_kinematics.hpp` | closed-form S-R-S kinematics, self-testing |
| `include/.../reduced_planner.hpp` | Approach A: 3/4-DOF swept-volume planner |
| `include/.../manifold_planner.hpp` | Approach B: 5-DOF exact manifold planner |
| `include/.../path_lifter.hpp` | wrist-centre path → 7-DOF, plus full-model validation |
| `src/test_reduced.cpp` | offline validation of Approach A |
| `src/test_manifold.cpp` | offline validation of Approach B |
| `src/benchmark_reduced.cpp` | head-to-head benchmark (needs `move_group`) |
| `src/preview_reduced.cpp` | RViz preview: real arm inside the swept volume |
| `launch/moveit_headless.launch.py` | headless stack for benchmarking |
| `launch/preview_reduced.launch.py` | RViz preview |

### Reproducing the numbers

```bash
# Approach A geometry and coverage
python3 tools/wrist_blob_gen.py --tag carry --tilt 0.30
python3 tools/make_reduced_model.py --geometry mesh
ros2 run my_robot_control test_reduced --gen <share>/generated \
  --full-urdf <urdf> --full-srdf <srdf>

# Approach B
ros2 run my_robot_control test_manifold --urdf <urdf> --srdf <srdf> --transit-z 0.24

# Head-to-head (needs move_group)
ros2 launch my_robot_control moveit_headless.launch.py
ros2 run my_robot_control benchmark_reduced --ros-args -p gen_dir:=<share>/generated
```

Note: MoveIt is a source build in `~/ws_moveit2` — source it, or `move_group`
fails to load `trac_ik` and the OMPL plugin. After editing headers under
`include/my_robot_control/`, rebuild clean; an incremental build once left
`simple_move.cpp` compiled against a stale struct layout, silently reading
garbage for two newly added fields.
