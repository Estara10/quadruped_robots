# S6-11 revised map set: forward-local obstacle layouts

Version: `s6-11-mapset/1.0`. This version replaces the 24 S6-09 `route_fork` and `irregular_cluster` candidates in the **new main-candidate manifest**. Their XML, IDs, previews, and historical runs remain in the scene catalog and old S6-09 layout. They are not silently reused or rewritten. The 60-scene S6-11 manifest contains 36 retained records plus 24 new maps; it is stratified as 30 development and 30 held-out. New generation seeds are disjoint by split. Policy run seeds are separate and no policy run was performed for this task.

## Why the layouts changed

The change follows the thesis scope: local obstacle avoidance from the current forward range rays. The current Go2 ray producer defines 11 horizontal rays at 9-degree increments from -45 to +45 degrees, with 0.1–6 m range and an origin at body x=-0.05 m (`unitree_mujoco/simulate/src/unitree_sdk2_bridge.h`, ray constants and `ray2d` producer). This describes a forward 90-degree sensing sector. It does not prove the old maps were impossible to traverse. The revised layouts avoid requiring long detours around broad central obstacle groups; they do not prove that the policy can observe or pass them.

The robot clearance rule is unchanged: a conservative 0.65 m planar body envelope plus the existing 0.10 m engineering margin, for at least 0.75 m from the reference centerline to each obstacle. This envelope has not been gait-tested. No density band or safety margin was reduced to obtain the new layouts.

## Quantified local-layout rules

| Rule | Value | Check/meaning |
|---|---:|---|
| Obstacle placement boundary | x=[-0.5, 7.5] m, y=[-3, 3] m | Generation boundary, not a physical wall |
| Reference-path search boundary | x=[0, 7] m, y=[-1.8, 1.8] m | Start `(0,0)` to goal `(7,0)` |
| Start/goal obstacle edge clearance | >=0.95 m | Preserved from the existing map rule |
| Maximum projected obstacle length along x | 0.85 m | Projection uses rotated box bounds or cylinder diameter |
| Maximum projected obstacle width along y | 0.75 m | Same actual-shape projection |
| Minimum centerline-to-obstacle clearance | 0.75 m | 0.65 m envelope + 0.10 m margin; unchanged |
| Maximum reference lateral offset | 1.55 m | A local bypass limit, not a route instruction to policy |
| Reference segment heading | <=50 degrees from +x | Forward progress only; no route requires backing up |
| Grid / continuous shape check | 0.05 m / <=0.025 m | Grid for candidate search; final paths checked against actual rotated boxes/circles |
| Min obstacle spacing by density | sparse 0.28 m; medium 0.16 m; dense 0.10 m | Actual shape-to-shape distance |
| Obstacle height | 0.32–0.62 m | Same range as the existing library; no newly introduced low trip obstacle |

Generated shape parameters are also bounded: cylinders use radius 0.22–0.29 m; axis-aligned or slightly rotated boxes use half lengths x=0.25–0.35 m and y=0.20–0.28 m; short rotated bars use half lengths x=0.27–0.37 m and y=0.13–0.20 m. Their yaw ranges are ±0.22 rad for boxes and ±0.20 rad for bars. After rotation, every object—including density-fill objects—must still satisfy the maximum projected x/y bounds above.

Primary obstacles in the medium/dense short-bypass arrangements are about 1.85 m apart in x; with their bounded x projections this leaves at least about 0.84 m between adjacent x-projection edges before accounting for off-route fill obstacles. The sparse short-bypass pair is farther apart. The forward-offset arrangement alternates obstacle sides at separated x positions. Final route checks use all obstacles, so an arrangement is rejected if fill obstacles intrude into the required Go2 envelope clearance. This geometric space is not a claim about dynamic gait recovery.

## New structures

1. **Short local bypass** (`short_local_bypass`, 12 maps): two or three discrete boxes, cylinders, or short rotated bars cross or approach the start-goal centerline. A forward-monotone reference route passes each obstacle locally and continues in +x. The route is for geometry checking and preview only.
2. **Forward-offset short obstacles** (`forward_offset_short_obstacles`, 12 maps): short obstacles alternate around the forward corridor, producing multiple modest envelope-driven lateral changes while preserving a forward path and room to turn back toward the goal. Some maps do not physically intersect the point-robot start-goal line; their body-envelope clearance check requires the lateral adjustments. This distinction is retained in each map's `direct_line_blocked` field.

Density bands are the existing non-overlapping S6-09 footprint fractions: sparse `[6%,9%)`, medium `[9.5%,12%)`, dense `[13%,20%)`; gaps between bands are deliberate. Density describes projected obstacle area only, not difficulty. Actual spacing, route length, detour ratio, geometric clearance, and obstacle count are also stored.

Each new structure has 2 development and 2 held-out maps per density. Development generation seeds start at 9,110,000; held-out seeds start at 9,210,000. Each candidate uses bounded layout attempts and bounded density-fill attempts, then must pass bounds, endpoint clearance, spacing/no-overlap, direct-shape segment checks, forward progress, lateral-offset, heading, and density checks. Failure exhausts a finite retry limit and rejects that scene; there is no unbounded resampling.

## Version and production binding

The active S6-11 manifest is [`layouts_s6_11.json`](layouts_s6_11.json); the 24 old exclusions and replacement crosswalk are [`legacy_24_excluded.json`](legacy_24_excluded.json) and [`replacement_crosswalk.md`](replacement_crosswalk.md). The production catalog retains all old records and adds unique short ASCII IDs (`s611_*`, fewer than 32 bytes) for new XML. Descriptive filenames and preview names remain long and are not used as binary scene IDs.

MuJoCo ray generation and collision authority both use `abs_scene::selectObstacleGeoms()` from the shared scene catalog. Run setup and the analyzer resolve the same filename-to-ID/root/closure/fingerprint binding. Fingerprints in new catalog rows were computed with the C++ production fingerprint helper linked to the runtime MuJoCo 3.3.3 library, not with the Python MuJoCo 3.4.0 syntax/load check.

The 24 new maps passed a bounded production C++ authority publisher to normal Python reader handshake: all snapshots were LIVE and authoritative, with matching scene ID, root/closure identity, runtime fingerprint and obstacle count. This checks scene loading and identity binding, not robot motion or policy reachability.

## Remaining 36 S6-09 maps

All 36 original maps in `single_side_bypass`, `alternating_bypass`, and `offset_gates` were statically inspected using their actual rotated-shape projections and saved reference paths. The results are in [`retained_36_static_review.md`](retained_36_static_review.md). Every map has at least one obstacle whose projected span exceeds the new 0.85 m local-obstacle limit. The 12 single-side maps have maximum path-segment headings around 28.3 degrees but retain long transverse structures (maximum lateral projection about 3.425 m). The 12 alternating maps have maximum segment headings about 81–82 degrees; the 12 offset-gate maps about 73.5–80 degrees. Their stored reference paths do not backtrack and stay within the 1.55 m lateral bound, but the long barriers/large turns differ from S6-11's local-obstacle rule. The report lists every ID for Director review; none was changed or filtered by policy outcomes.

The field edges are not walls, so a robot could go around an obstacle field from outside its declared bounds. Future runs need actual trajectory and out-of-bounds records, and the experiment must define whether leaving the declared region terminates a run. Reference connectivity does not establish policy reachability, ray visibility after occlusion, or dynamic maneuverability. The held-out split must remain unused for parameter selection and must not be removed based on policy performance.
