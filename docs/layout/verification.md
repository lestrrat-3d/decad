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
| `verify_result.go` | Public survey outcome enums, reports, requests, bounded readings and survey results. |
| `verify_publish.go` | Adapts private surveys to `internal/verifypublish` publication. See verification §1, §6. |
| `internal/clearance/pairkernel/` | The pair kernel over topology nodes: `Pair`, `SheetSolid`, axis-aligned box pairs, the per-call carrier `Cache`, carrier models, the cell walk, the body and pair gate diameters, and the planar disjoint enclosure. See clearance §1–§7 and verification §3. |
| `clearance_planar.go` | The exact planar pair arm for mitred, faceted and coil bodies: interference §3.2. |
| `contact_pair.go` | Pair gates, public contact report aliases of `report/`, the lift from the node report to the handle report, the bit-exact pair report memo key and cap, and the `internal/contactpair/` call. See contact-geometry §6. |
| `internal/planarpose/pose.go` | Planar admission, snapshots and convexity cache; publication through `internal/planarband/`. See multibody §9–§10. |
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
| `contact_sweep.go` | `SweepPair` gates, the sweep hooks over `ContactPair`, the lift of the node report to handles, and the report, request, outcome and track aliases of `report/`. See contact-sweep §7. |
| `swept_box.go` | Whole-path box. See multibody §4.2. |
