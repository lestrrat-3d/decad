# decad file layout: verification

Verification, survey and check files.
The rules for rows live in `docs/layout.md`.

## Layout

### Verification and surveys

| Path | Responsibility |
|---|---|
| `verify.go` | Public verification option tiers and `Document.Verify` orchestration. See verification §1–§3. |
| `verify_pairs.go` | `Verify`'s pair proofs and job list. See interference §2. |
| `report.go` | Public verdict and diagnostic enums, diagnostic and pair result types, and private report adapters. |
| `verify_tolerance.go` | Adapts `internal/tolerance/` to `Verify` readings and diagnostics. See verification §2-§3. |
| `verify_gate.go` | Verify's faceted diameter, carrier boundary, point table and cached pair diameter; `internal/gatediameter/` selects other witnesses. See verification §3. |
| `verify_result.go` | Public survey outcome enums, reports, requests, bounded readings and survey results. |
| `verify_publish.go` | Adapts private surveys to `internal/reportvocab` publication. See verification §1, §6. |
| `clearance.go` | The pair kernel: `clearancePair` and `sheetSolidPair`, plus unplaced axis-aligned box pairs over the `internal/pair/box/` gap proof. See clearance §1-§3/§6. |
| `clearance_planar.go` | The exact planar pair arm for mitred, faceted and coil bodies: interference §3.2. |
| `contact_pair.go` | Pair gates, public contact report types, their conversion from the `internal/reportvocab` report, and the bit-exact pair report memo key and cap. See contact-geometry §6. |
| `contact_box.go` | Source-box admission and public face/measurement mapping. See contact-geometry §4. |
| `contact_faceted_pair.go` | Planar admission, snapshots and convexity cache; publication through `internal/planarband/`. See multibody §9–§10. |
| `contact_faceted_floor.go` | Exact faceted support-face proof, bounded strict separation, and faceted floor sweeps by exact support face or swept-box clearance. See contact-geometry §4 and contact-sweep. |
| `contact_oriented_box.go` | Source face identity and report adapters over `internal/pair/box/` geometry, and publication of oblique box patches. See contact-geometry §4. |
| `contact_clipped_patch.go` | Publishes horizontal box patches from exact polygon clips. See contact geometry §4. |
| `contact_sphere.go` / `contact_sphere_sweep.go` | Sphere-box and sphere-pair contact, rotated sphere-box adapters, and sphere-box sweeps. See contact-sweep §4–§5. |
| `contact_cylinder.go` / `contact_cylinder_sweep.go` | Cylinder contact and sweep adapters over `internal/pair/box/`. See contact-sweep §4–§5. |
| `contact_analytic_manifold.go` | Ruling contact adapters. See contact-geometry §4.5. |
| `contact_sphere_pair_sweep.go` | Sphere-pair sweep. See contact-sweep §4–§5. |
| `contact_sweep_replay.go` | Replay adapters. See contact-sweep §6. |
| `clearance_cells.go` | Pruned cell walk, face-pair adapter and tier adapters; `vertexTier` passes the shared budget to `internal/clearance/tier` and folds its error/refusal state into the pair. See clearance §3–§6. |
| `clearance_geom.go` | `bodyGeom`: carrier dispatch, live body topology and nesting over `internal/clearance/` and `internal/wallsurvey/`. See clearance §2–§3. |
| `point_containment.go` | Public point location and body/face distance through verified meshes. See point-containment design. |
| `survey.go` | Dispatches wall, undercut and radius readers and adapts their readings to public body and face handles. See verification §6. |
| `interference.go` | Pairwise overlap behind `Verify`. See interference §4-§8. |
| `motion.go` / `motion_verify.go` | Public motion inputs, options, requests and reports; pose checks over `internal/motionbound/` and static pair classification through `internal/motionrun/`. See motion-check §2–§6. |
| `motion_verify_pair.go` | Per-pose mover and pair evaluation, interval verdicts and report publication for `VerifyMotion`. See motion-check §5–§8. |
| `linkage_box_cell.go` | Joint-box cell evaluation, classification and report publication, and the bounded transient body cache. See linkage §10, §14. |
| `motion_bound.go` | Reads payload record radii for `internal/motionbound/`. |
| `linkage.go` / `linkage_verify.go` | `VerifyLinkage` and its public report types. See linkage design. |
| `linkage_box.go` | `VerifyJointBox` and its public joint-space inputs and reports. See `docs/linkage-check-design.md`. |
| `linkage_loop.go` | Closed loops and their bar geometry. See `docs/linkage-check-design.md` §15. |
| `linkage_loop_drive.go` | Loop private-scene drive, loop poses and `Schedule`. See `docs/linkage-check-design.md` §15. |
| `linkage_bound.go` | Linkage reach and projection adapters. See linkage §5.2, §5.8. |
| `contact_sweep.go` | Sweeps, public report, request and time types, the request's conversion to the `internal/reportvocab` form, and tracks. See contact-sweep §7. |
| `contact_sweep_run.go` | The pair sweep run: sampling, ideal contact, manifold transfer and departure proof. See contact-sweep §7. |
| `contact_sweep_rotation.go` / `contact_sweep_planar.go` | Rotating sweep adapters, planar sweeps with their support bands, and the sweep memo adapter. See contact-sweep §4, §7 and multibody §10. |
| `contact_sweep_rotation_depart.go` | Rotating pair sweep departure search, bracket refinement and interval clearance over the `internal/pair/box/` and `internal/sweepmemo/` separation proofs. See contact-sweep §4. |
| `contact_sweep_rolling.go` | Rolling pair sweeps and their band tracks. See multibody §10. |
| `swept_box.go` | Whole-path box. See multibody §4.2. |
