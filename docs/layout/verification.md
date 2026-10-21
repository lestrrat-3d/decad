# decad file layout: verification

Verification, survey and check files.
The rules for rows live in `docs/layout.md`.

## Layout

### Verification and surveys

| Path | Responsibility |
|---|---|
| `verify.go` | Public verification option tiers and the `Document.Verify` gate: it runs `internal/verifyrun/` and lifts its report to handles. See verification §1–§3. |
| `report.go` | Public verdict and diagnostic enums, diagnostic and pair result types, and private report adapters. |
| `verify_result.go` | Public survey outcome enums, reports, requests, bounded readings and survey results. |
| `internal/clearance/pairkernel/` | The pair kernel over topology nodes: `Pair`, `SheetSolid`, axis-aligned box pairs, the per-call carrier `Cache`, carrier models, the cell walk, the body and pair gate diameters, and the planar disjoint enclosure. See clearance §1–§7 and verification §3. |
| `contact_pair.go` | Pair gates, public contact report aliases of `report/`, the lift from the node report to the handle report, the bit-exact pair report memo key and cap, and the `internal/contactpair/` call. See contact-geometry §6. |
| `internal/planarpose/pose.go` | Planar admission, snapshots and convexity cache; publication through `internal/planarband/`. See multibody §9–§10. |
| `point_containment.go` | Public point location and body/face distance through verified meshes. See point-containment design. |
| `motion.go` / `motion_verify.go` | Public motion inputs and options, the report aliases of `report/`, and the `VerifyMotion` gate over `internal/motionengine/`. See motion-check §2–§6. |
| `linkage_box_cell.go` | Joint-box cell evaluation, classification and report publication, and the bounded transient body cache. See linkage §10, §14. |
| `linkage.go` / `linkage_verify.go` | `VerifyLinkage` and its public report types. See linkage design. |
| `linkage_box.go` | `VerifyJointBox` and its public joint-space inputs and reports. See `docs/linkage-check-design.md`. |
| `linkage_loop.go` | Closed loops and their bar geometry. See `docs/linkage-check-design.md` §15. |
| `linkage_loop_drive.go` | Loop private-scene drive, loop poses and `Schedule`. See `docs/linkage-check-design.md` §15. |
| `linkage_bound.go` | Linkage reach and projection adapters. See linkage §5.2, §5.8. |
| `contact_sweep.go` | `SweepPair` gates, the sweep hooks over `ContactPair`, the lift of the node report to handles, and the report, request, outcome and track aliases of `report/`. See contact-sweep §7. |
| `swept_box.go` | Whole-path box. See multibody §4.2. |
