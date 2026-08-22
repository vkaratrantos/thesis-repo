# Working notes, phase 2 — descent-stall bug, planner fixes, visualisation, GUI sequencing

> **Chronologically this follows `session_notes_phase1.md`**, which covers the
> professor's original message, the swept-volume approach and why it failed
> (0/30), the closed-form S-R-S kinematics, the benchmark against the constrained
> 7-DOF baseline, and the construction of the manifold planner this file starts
> from. The "already tried and failed in an earlier session" list in §0 below is
> written up in full there, in phase 1 §6.1.

Informal notes, roughly chronological. Includes the things that worked and the
things that didn't, because several of the dead ends are more informative than
the fixes. All numbers here are measured on the running system, not estimated.

---

## 0. Starting point

The 5-DOF manifold planner already existed and worked: carrying transits planned
in ~30 ms with ~0.0002° tool tilt, against ~10 s and up to 24° for the older
constrained 7-DOF pipeline. Descents were at 100%.

One open bug: **orientation-keeping only applied while the gripper was HOLDING a
tube.** Extending it to empty-gripper transits had been tried and reverted — the
transit itself succeeded and the arm arrived correctly above the tube, but the
straight-line descent that followed stalled somewhere between 10% and 80% of the
way down.

Things already tried and failed in an earlier session:
- exact goal tolerances (removing the 5 mm / 0.02 rad slack)
- filtering goal states to within 0.6 rad, then 0.12 rad, of `standoff_state`
- `planToConfiguration()` with a single explicit goal configuration
- the apparent pattern "1 goal state → descent works, 627 → fails" turned out to
  be noise; a later run showed 1 goal state failing at 10.6%

---

## 1. Diagnosis: what is actually blocking the descent

### 1.1 The instrumentation

The instruction was to stop guessing and find out what was actually colliding —
set `CollisionRequest::contacts = true` and dump the contact pairs at the first
blocked waypoint.

Added three things to `simple_move.cpp`:

- **`IkValidator::contactPairs()`** — re-runs the collision check with contacts
  enabled and returns the touching body pairs plus penetration depths.
  `isStateColliding()` only answers yes/no, and `setFromIK()` swallows even that.
- **`diagnoseBlockedWaypoint()`** — the important one, see below.
- **a `u` runtime toggle** on `/gui_commands` — flips between the shipped
  behaviour and the reverted one without a rebuild, so both can be exercised
  against the same scene in one session.

The reason for `diagnoseBlockedWaypoint()`: `solveWaypoint()` returns a bare
`false`, but there are **three** distinct ways to get there:

1. no IK solution exists at that pose at all (reach / singularity)
2. solutions exist but every one is in collision
3. solutions exist and are collision-free, but sit further than `max_joint_step`
   (0.30 rad) from the previous waypoint — a null-space jump, i.e. a continuity
   rejection, not an obstacle

These require opposite fixes, and (3) is invisible to any amount of collision
debugging. So the diagnostic re-solves IK with the collision check switched off
and grades every solution against the three gates separately.

### 1.2 The result

Across many runs and ~160 raw IK solutions:

```
out of joint bounds .......... 1
in collision ................. 0        <-- every time
collision-free but too far ... 23   (limit 0.3 rad)
should have passed ........... 0
```

**Nothing was ever in collision.** The collision hypothesis, which had driven
every previous fix attempt, was simply wrong.

### 1.3 The actual mechanism

Joint values at the stall, printed against their limits:

```
joint5   2.96693  ->  0.166719   delta -2.80021   limits [-2.967, 2.967]
```

joint5 arrives **saturated against its own upper stop** — 2.962 measured, about
0.005 rad of headroom. The descent needs it to keep going; it can't, so IK jumps
to the mirror wrist branch ~2.8 rad away, and the continuity check correctly
rejects that.

Side-by-side, same target, same run structure:

| | joint5 on arrival | headroom to limit | descent |
|---|---|---|---|
| shipped (joint target) | 0.065 | 2.90 rad | **100%** |
| upright (reverted change) | 2.962 | **0.005 rad** | 11.2% |

The clincher: in the failing run `standoff_state` itself held the *good* branch
(joint5 = 0.070). The upright transit went to 2.962 anyway — a 2.89 rad gap.

This also explained the earlier "noise": on one attempt the arm arrived within
0.054 rad of `standoff_state` and still stalled, because by then `standoff_state`
had inherited the saturated branch from the previous failed attempt's start
state. Arrival distance was never the variable. **Branch was.**

### 1.4 Two process notes worth recording

- **My own diagnostic was initially misleading.** It printed a "CONTINUITY
  REJECTION" verdict whenever any solution was too far, even when other
  solutions had passed all gates. Fixed so the verdict depends on whether
  anything passed.
- **Stale ROS processes broke the experiment.** A partial teardown left
  duplicate node names in the graph, which broke DDS discovery — published
  commands silently never reached the node, and a run looked like "the bug did
  not reproduce" when in fact nothing had run. The test driver was changed to
  verify teardown and to confirm each command was actually queued before
  proceeding. The "always restart the full stack" rule matters more than it
  looks, and it's the *partial* restart that bites.

---

## 2. The fix — five defects, stacked

Each one hid the next, which is why single-shot attempts kept missing.

### 2.1 Choose the standoff for descendability, not just existence

`standoff_state` used to be any collision-free IK solution at the standoff pose.
Now each candidate is:
- screened for joint-limit headroom (`LIMIT_HEADROOM_MIN`), and
- verified by **dry-running the actual descent from it** (`descentDryRun`)
  before the arm commits to going there.

This needed `planRobustCartesian()` to accept an explicit start state, so a
descent can be planned from a configuration the arm has not moved to yet.

### 2.2 Mirror-wrist IK seeds — the load-bearing part

IK seeded from a pinned configuration returns pinned configurations. Measured,
all four original seed strategies came back 1e-6 … 0.09 rad from the stop, while
a perfectly good configuration existed on the other branch with 0.23 rad to
spare.

`mirrorWristSeed()` seeds from `(q5 ± π, −q6, q7 ± π)` — the same wrist
orientation expressed on the other ZYZ branch. In testing this is the generator
that rescues nearly every otherwise-failing approach. It is not "more retries":
re-rolling seeds on the arm's own branch produces the same pinned solutions
however many times you try.

### 2.3 The goal hint was a hard filter, and that was worse than useless

`ManifoldPlanner::Options::goal_hint_radius` (0.12 rad) was applied as an
elimination. Measured on the live task it discarded **all 14** goal candidates,
so the planner reported "no goal configuration", and the caller fell back to the
constrained 7-DOF pipeline — which plans to the *pose* with no hint at all and
lands wherever it likes.

> A preference strict enough to eliminate the planner enforcing it just hands
> the decision to something with no preference.

Changed to a **ranking**: candidates inside the radius win; if none are, keep the
closest `goal_hint_fallback_count` (8) instead of failing. Also added
`goal_limit_headroom`, which rejects goal configurations pressed against a joint
stop outright — that one is not a preference, since there is nowhere to go from
such a configuration.

### 2.4 Manifold trajectories were silently rejected at execution

This one was invisible and had been there the whole time.

The manifold path starts at the **projection** of the current state onto the
upright manifold — the planner packs the start into (q1..q4, φ) and rebuilds the
wrist in closed form, so any part of the real configuration that is off-manifold
is discarded. `move_group` refuses to execute a trajectory whose first point is
more than `allowed_start_tolerance` (0.01 rad) from the measured state:

```
Invalid Trajectory: start point deviates from current robot state more than 0.01
```

`[manifold] ok` had already been printed by then — it is the *execution* that is
rejected — so the caller silently fell back to the 7-DOF pipeline. **The manifold
planner was never actually used for empty-gripper transits.** While carrying, the
arm is already on the manifold from the previous upright move and the projection
is a no-op, which is why it only surfaced now.

Fix: bridge explicitly from the real configuration to the planned start,
checking every intermediate state, and refuse when the gap is a full wrist flip
(π) rather than a small projection.

### 2.5 Verify arrival, don't assume it

After the transit, if the arm landed more than 0.05 rad from the intended
standoff *and* the descent from where it actually is would fail, it reconfigures
in place — a planned joint-target move (not naive interpolation, which was
measured sweeping the wrist through the rack), with an explicit tilt limit when
carrying.

This fired live at **10.6%** and at **78.1%** — the two ends of the originally
reported 10–80% band.

### 2.6 Results

| | before | after |
|---|---|---|
| m3 from home | stalls 10–15%, 0/3 attempts | 100% |
| m3 after m1 / after m5 | fails all 3 attempts | 100% |
| m1, m2, m4, m5 | — | 100% |
| carrying → mixer, → home | 100% | 100%, unchanged |

Carrying transits still plan on the manifold at ~70–180 ms with ~0.0003° tilt.

### 2.7 Known limitation (worth writing up honestly)

The manifold planner fixes the wrist branch **per plan, by design** — that is
what keeps the state→configuration map continuous. A transit whose start and
goal sit on opposite branches (home → tube 3) cannot be represented, and
correctly falls back to constrained 7-DOF.

Measured at tube 3's standoff: on the arm's own branch, of 14 arm-IK solutions,
8 were rejected by wrist limits and 6 were pinned against a joint stop — zero
usable goals. The rack essentially forces the other branch.

The two branches meet at q6 = 0 (the wrist singularity), so a branch-changing
chart is possible future work. This is a real finding, not just a workaround: a
reduced-space planner using an exact analytic chart commits the arm to one wrist
branch, and the workcell / home pose have to be laid out compatibly.

---

## 3. Verifying the professor's kinematic assumption

The professor's first message asked to confirm the robot has a spherical wrist
(last three axes intersecting) so that position and orientation can be solved
almost independently — position from the first joints, orientation from the last
three.

**Confirmed from the URDF.** The axes of joint5, joint6 and joint7 all intersect
at one point — the origin of link5's frame, 127.8 mm out from link4.

The geometry is deceptive and worth explaining in the report: joint7's origin is
`xyz="0 0.066 0"`, a 66 mm offset that looks like it breaks the intersection. It
doesn't — the offset lies **along joint7's own axis direction**, so the axis line
still passes through the wrist centre. A naive "are the offsets zero?" check
gives the wrong answer.

Numerically, over 2000 random wrist configurations, max distance from the common
point to any of the three axes:

- **3×10⁻¹⁷ m** with exact π/2 — machine epsilon, i.e. exactly spherical by design
- **2.4×10⁻⁷ m** with the URDF as written, because it says `1.5708` instead of
  `1.5707963…`. That 3.67×10⁻⁶ rad rounding over the 66 mm lever predicts
  2.424×10⁻⁷ m, matching the measurement to four significant figures.

Consistency check: that 66 mm is exactly the `flange_offset_` the manifold
planner computes, and 66 + 135 (gripper) = the 201 mm figure used in the
swept-volume argument.

**The professor's caveat turned out to be the hard part.** They wrote that "the
only thing you must take into account is the joint limits" — and the entire
descent-stall bug was exactly that: wrist joint limits in the decoupled solution.

---

## 4. Terminology clarification (for the write-up and the viva)

"5-DOF planner" is loose and invites a question. Better: **a planner whose search
space is 5-dimensional.**

- The arm still has 7 joints and the planner outputs full 7-joint configurations.
- What is 5-dimensional is the space OMPL samples: `(q1, q2, q3, q4, φ)`.
- Of those, **four are joints**; φ is the tube's remaining orientation freedom
  (rotation about the vertical). `q5, q6, q7` are not searched at all — they come
  from the closed-form ZYZ inversion.

Why 5 is forced, not chosen — two derivations that agree:

```
7 joints − 2 constraints (tube axis vertical fixes 2 of 3 orientation DOF) = 5
3 (wrist-centre position) + 1 (arm redundancy) + 1 (roll φ)               = 5
```

Analogy for the viva: a point constrained to a sphere lives in 3D, but the sphere
is 2-dimensional — you locate any point with latitude and longitude. Same here:
`(q1..q4, φ)` are coordinates on the constraint surface.

`lock_arm_angle` gives a genuine 4-D search (`q1, q2, q4, φ`) — the professor's
literal number — at a measured ~5% cost in wrist-centre workspace.

**Relation to the swept-volume approach.** Both reduce the arm search to 4
joints. The difference is what happens to φ:

| | arm joints searched | what happens to φ | result |
|---|---|---|---|
| swept volume | 4 | swept (treated as unknown) | 0/30 |
| manifold | 4 | searched (treated as a decision) | 100% |

The swept volume pays for every value φ *might* take; the gripper plus tube
reaches 201 mm from the wrist centre on an arm of 243 mm reach, so the swept
solid is nearly as wide as the robot and always contains a neighbouring tube.
Promoting φ from an *uncertainty the collision checker must absorb* to a
*decision the planner makes* costs one extra search dimension and makes the
collision check exact. That is the intellectual bridge between the two
approaches, and it is worth presenting that way rather than as a rejection of
the professor's suggestion.

---

## 5. Visualisation and timing (this took several wrong turns)

### 5.1 First hypothesis — wrong

Observation: motion looked choppy in RViz. My first hypothesis was that the
manifold planner produced coarse trajectories (it logs `max joint step`, and one
run showed 0.175 rad = 10°).

**Measured the opposite.** Recording `/joint_states` and per-process CPU:

| move | planner | msg rate | max step per message | CPU (rviz/simple/mg) |
|---|---|---|---|---|
| m3 transit | constrained 7-DOF | 10.0 Hz | — | 52 / 4 / **70%** |
| m3 descent | Cartesian | 10.0 Hz | **0.45 rad (25.8°)** | 50 / 164 / 12% |
| m6 | **5-D manifold** | 10.0 Hz | 0.15 rad (8.6°) | 28 / 108 / 2% |

The 7-DOF path had the *larger* steps, and rviz2 was never CPU-starved (peak
62% of one core out of eight; move_group peaked at 329%, simple_move at 202%).

### 5.2 What was actually wrong (two real bugs)

**(a) Everything reached RViz at exactly 10.0 Hz.** `joint_state_publisher`'s
default `rate` is 10 Hz, and `fake.launch.py` never set it — so the relay
decimated everything before anything was drawn.

**(b) `fake_robot.py` ignored `time_from_start`.** It did `time.sleep(0.02)` per
trajectory point, discarding TOTG's velocity profile entirely. Consequence
beyond appearance: **every trajectory executed at a flat 20 ms per waypoint
regardless of its planned timing, so the velocity/acceleration scaling factors
had no effect on the running system.** Any statement about execution speed before
this point was not true of the real system. (Planning times are unaffected —
those are measured inside `simple_move` before execution.)

Fixed both: `rate: 100` on the relay, and a wall-clock walk in `fake_robot.py`
that interpolates between waypoints at ~100 Hz.

| | before | after |
|---|---|---|
| `/joint_states` rate | 10.0 Hz | **100.0 Hz** |
| max step per update | 0.45 rad (25.8°) | 0.0104 rad (**0.6°**) |
| rviz2 CPU | peak 62%, mean 19% | peak 82%, mean 24% |

43× finer motion. Durations went 3.10 s → 10.40 s etc., i.e. moves now run at
the speed they were planned at.

### 5.3 "Now it's too slow" — a config bug underneath

`joint_limits.yaml` defines `max_velocity 3.0, max_acceleration 3.0` for all
seven joints, but `run_simple_move.launch.py` never passed
`moveit_config.joint_limits` (= `robot_description_planning`). So the TOTG
running *inside* `simple_move` — for manifold transits and all Cartesian
descents — fell back to the default **1 rad/s²**, then scaled by
`ACC_SCALE_TRANSIT = 0.10`. Effective acceleration 0.1 rad/s². Trajectories
planned by `move_group` were unaffected, because move_group loads the limits
itself.

Fixed the launch file, and raised the acceleration scales 0.10/0.05 → 0.30/0.15,
keeping the 2:1 transit:liquid ratio.

Worth noting: motion is entirely acceleration-limited here (duration ∝ 1/√a),
nothing is velocity-limited, so `VEL_SCALE_*` does essentially nothing at these
values. And the *original* 20 ms/waypoint playback was effectively running the
arm at its full dynamic limits — it looked fast because it was unphysical.

### 5.4 Delays *between* movements

Instrumented the approach stages. For m6:

```
grasp IK ...............................  757 ms
standoff z+0.12:  six generators, all "no IK"  3825 ms   <-- all wasted
standoff z+0.084: accepted first try     [ik 404 ms, dry-run 349 ms]
TOTAL BEFORE THE ARM MOVES .............. 5335 ms
[manifold] planned in ...................   21 ms
```

**The manifold planner was 0.4% of the delay.** The mixer's full standoff height
is genuinely out of reach, and the code proved that six times over with six
different seeds. The costs 403 / 604 / 806 ms are exactly `tries × ik_timeout`
(8, 12, 16 × 0.05 s) — when IK fails, every attempt burns its full timeout. This
was partly self-inflicted: the old code had two generators, the mirror-wrist fix
added four more.

Three fixes:
1. **`poseIsReachable()` gate** — one cheap kinematic probe before the expensive
   ladder. 3825 ms → **122 ms**.
2. **`LIMIT_HEADROOM_MIN` 0.15 → 0.05.** 0.15 was too aggressive: it discarded
   candidates with 0.128–0.131 rad of headroom *without testing them*, forcing
   the search through all six generators and down to a lower standoff for no
   reason. The measured populations are far apart — every configuration that
   stalled a descent had headroom under 0.01, every working one 0.18+ — so 0.05
   sits in the empty gap, and marginal cases now go to the dry run, which is the
   authoritative test and costs ~250 ms.
3. **Duplicate skip** — four of six generators were re-finding the same rejected
   configuration.

Result: m6 5335 ms → **1651 ms**; m2 2863 ms → **1591 ms**.

### 5.5 An experiment that failed and was reverted (and the revert may be wrong)

Tried `ik_timeout` 0.05 → 0.02. It gave a clean **2.5×** on IK (grasp IK 757 →
307 ms; total before motion m2 1591 → 944 ms, m6 1651 → 976 ms).

But markers 1 and 5 then failed outright with "NO kinematic solution at the
target at all". Reverted on that basis.

**Caveat that must be recorded:** it was discovered afterwards that the tube
positions had been changed at around the same time, and a later run showed
marker 1 still failing *at the reverted 0.05 timeout* — so the failure was
probably caused by the position change, not the timeout, and the revert may have
been based on a bad attribution. The comment currently in the source claiming
0.02 "broke markers 1 and 5" is not established. It is worth re-testing once
tubes 1 and 5 are reliably in reach; there is a 2.5× IK speedup available if the
attribution was indeed wrong.

(Tubes 1 and 5 sit at x = ±0.17, i.e. 286 mm from the base against 251 mm for
tubes 2/4 and 230 mm for tube 3 — they are the marginal ones by a clear margin.)

### 5.6 The actual FPS problem — nothing to do with any of the above

Symptom: RViz showing "only a few frames per movement". Analysed a screencast
frame by frame: during motion the viewport changed **2.9 times per second**, with
the picture frozen for a median of **504 ms**. RViz's own readout said 29 fps, so
rendering was fine — there was simply nothing new to draw.

Root cause: **there was no `RobotModel` display in `moveit.rviz` at all.** The
only displays were Grid, TF and MotionPlanning, and MotionPlanning draws its
robot from `Planning Scene Topic: monitored_planning_scene` — not from TF. So
`/joint_states` at 100 Hz → TF at 100 Hz → *rendered by nothing*. The visible
robot only redrew when the planning scene was republished, and the only thing
doing that regularly was `simple_move`'s 500 ms collision-object updater. Hence
the 504 ms freezes.

This also means the earlier 10 → 100 Hz fix, while correct and necessary for
execution timing, did not address the visual symptom at all. Lesson: check which
display is drawing the thing before optimising the pipeline feeding it.

Fix: add a `RobotModel` display (renders from `/robot_description` + TF) and turn
off the MotionPlanning Scene Robot visual. Motion became smooth.

**Follow-on problem:** the grasped tube then disappeared, because an attached
collision object lives in the planning scene, not the URDF, so `RobotModel`
cannot draw it. Fixed by raising the planning-scene publish rate on move_group:

```python
{'publish_planning_scene': True,
 'publish_geometry_updates': True,
 'publish_state_updates': True,
 'publish_transforms_updates': True,
 'publish_planning_scene_hz': 30.0}
```

`publish_planning_scene_hz` alone was **not** enough — it is a maximum, and the
scene is only published when it *changes*; without `publish_state_updates`,
robot-state changes trigger nothing. Measured: 2.0 Hz → **28.8 Hz**.

---

## 6. GUI sequencing

### 6.1 Symptom and (wrong) first guess

The GUI's AUTO recipes and MANUAL batch sequences didn't work. My first guess was
the `if (!attached_tube.empty()) { "Already holding..."; continue; }` guard —
every recipe has two tubes and the GUI never sent a release between them.

Tested it instead of assuming, and the real failure came first:

```
[EXECUTING] TASK 1
[-] MTC: TF lookup for marker_1 failed.
    [!] Task failed or aborted.
```

### 6.2 Actual causes

1. **`TASK n` requires real ArUco TF; `m<n>` doesn't.** The GUI sequences sent
   `TASK n`, which goes to the MTC pipeline, which does a hard TF lookup and
   bails. The manual `m<n>` path falls back to predefined tube positions —
   which is exactly why the direct-override buttons worked and sequences didn't.
   Also worth noting: the whole MTC path is separate from, and untested against,
   everything fixed in sections 1–2.
2. **No release between tubes** (the guard above) would have bitten immediately
   afterwards.
3. **The GUI never called `rclpy.spin`.** Publishing works without it, but the
   node could never *receive* anything — so feedback of any kind was impossible
   before this was fixed.

### 6.3 Fix

**`simple_move.cpp`** — added a status channel. It now publishes `OK <cmd>` /
`FAIL <cmd>` on **`/gui_status`**, exactly once per command.

The reporter fires from a **destructor** (`CommandReport`). This matters: the
command loop exits through about a dozen separate `continue` statements, and any
scheme that required each exit point to remember to report would eventually miss
one, leaving a sequence waiting forever. It defaults to `FAIL` and requires a
branch to say otherwise, so an unhandled path fails safe.

**`GUI.py`** —
- `_run_tube(n)` sends `m<n> → c → m6 → p → m<n> → o`, waiting for each result
  and stopping at the first failure or timeout.
- dropped `TASK <n>`, so no camera is needed, and sequences now exercise the
  manifold planner and all the fixes above.
- added a spin thread (prerequisite for receiving anything).
- `sequence_lock` is now `acquire(blocking=False)`, so a second EXECUTE while one
  is running is refused rather than silently queueing.
- `on_close` releases a blocked sequence and joins the spin thread, otherwise
  rclpy aborts with "terminate called without an active exception".

### 6.4 Verified

Two-tube sequence, every step reporting back:

```
=== [1] TUBE 3 ===        === [2] TUBE 2 ===
  OK   m3  (28.1s)          OK   m2  (10.5s)
  OK   c   ( 9.6s)          OK   c   ( 8.3s)
  OK   m6  (12.6s)          OK   m6  (10.8s)
  OK   p   (11.3s)          OK   p   (11.3s)
  OK   m3  (28.3s)          OK   m2  (10.7s)
  OK   o   ( 7.9s)          OK   o   ( 7.8s)
=== SEQUENCE COMPLETE ===
```

Abort path, forced with a pour on an empty gripper:

```
-> step [p]
  FAIL p
!! step p failed -> SEQUENCE ABORTED (later steps skipped)
```

Timeouts are 180 s for moves, 90 s for gripper/pour; real steps took 8–33 s.

---

## 7. Methodological points worth a paragraph in the report

- **Every hypothesis in this session that was not measured turned out to be
  wrong at least once**, including several of mine: the manifold path being
  coarse (it was finer), the `ik_timeout` attribution (probably a confound), and
  the GUI's "already holding" guard (real, but not the active failure). The
  contact-pair instrumentation was what finally settled the original bug after a
  previous session had guessed repeatedly.
- **Failure modes that are silently swallowed are the expensive ones.** Three
  separate bugs here were of that shape: `setFromIK` returning `false` for three
  different reasons; `[manifold] ok` printing before an execution that was then
  rejected; and `/gui_commands` having no reply channel at all.
- **Defaults matter and are invisible.** `joint_state_publisher rate` (10 Hz),
  `publish_planning_scene_hz` (4 Hz), missing `robot_description_planning`
  (1 rad/s² instead of 3.0) — three separate performance problems, all from
  parameters nobody had set.
- **Restart the full stack between tests.** Partial restarts leave duplicate node
  names that break DDS discovery, and stale attached objects in the planning
  scene, both of which produce confidently wrong results.

---

## 8. Files touched

| file | what |
|---|---|
| `my_robot_control/src/simple_move.cpp` | diagnostics, standoff selection, mirror seeds, reachability gate, arrival verification, `/gui_status` reporting, timing instrumentation, speed scales |
| `my_robot_control/include/my_robot_control/manifold_planner.hpp` | `goal_hint_fallback_count`, `goal_limit_headroom`, `limitHeadroom()` |
| `my_robot_control/src/manifold_planner.cpp` | hint as ranking not filter, joint-stop rejection, branch reason reporting |
| `my_robot_control/launch/run_simple_move.launch.py` | pass `moveit_config.joint_limits` |
| `robot_config/launch/fake.launch.py` | `joint_state_publisher rate: 100`, planning-scene publish params |
| `robot_config/config/moveit.rviz` | (manual) add RobotModel display, disable Scene Robot visual |
| `~/my_robot/fake_robot.py` | honour `time_from_start`, interpolate at 100 Hz |
| `~/my_robot/GUI.py` | spin thread, `/gui_status` subscriber, `send_and_wait`, `_run_tube`, abort-on-failure |

---

## 9. Open items

- Re-test `ik_timeout = 0.02` now that tubes 1 and 5 are reachable — likely 2.5×
  on IK, and the current source comment about it is probably wrong.
- The MTC `TASK n` path still requires camera TF; either give it the same
  fallback as `m<n>`, or retire it now that sequences use the manual path.
- A branch-changing chart through q6 = 0 would let empty-gripper approaches use
  the manifold planner instead of falling back to constrained 7-DOF.
- The remaining ~1.6 s pre-motion delay is almost entirely IK (757 ms grasp +
  404 ms standoff); `solveNearestIk` always burns its full try budget to pick the
  *nearest* solution even after finding good ones. An early exit would trade
  posture consistency for speed.
