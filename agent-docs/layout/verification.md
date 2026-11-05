# decad file layout: verification, surveys and observability

Verification, survey and check files, the wall, undercut and concave-radius survey packages, and the operation
history and diagnostics internals. The rules for rows live in `agent-docs/layout.md`.

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
| `point_containment.go` | Public point location through the exact prism record, Boolean history or a verified mesh, and body/face distance and closest point through verified meshes. See point-containment design. |
| `motion.go` / `motion_verify.go` | Public motion inputs and options, the report aliases of `report/`, and the `VerifyMotion` gate over `internal/motionengine/`. See motion-check §2–§6. |
| `linkage.go` | The linkage aliases of `linkage/`, the `VerifyLinkage` and `VerifyJointBox` gates, and `WithCellBudget`. See linkage design. |
| `contact_sweep.go` | `SweepPair` gates, the sweep hooks over `ContactPair`, the lift of the node report to handles, and the report, request, outcome and track aliases of `report/`. See contact-sweep §7. |
| `swept_box.go` | Whole-path box. See multibody §4.2. |

### Internal packages

| Path | Responsibility |
|---|---|
| `internal/capblendsurvey/` | Cap-blend and Draft undercut readers, patch normal ranges, and cap-blend concave-radius reading. See modify-reach Table DX and draft DD7. |
| `internal/analyticundercut/` | Maps prism, revolve and cup records to their own-step faces and decides their undercut surveys. See verification §6. |
| `internal/survey2d/` | 2D disks, prism readers, walks, corner loops, Bézier carriers. See verification §6. |
| `internal/revolvesurvey/` | Revolve wall, undercut and concave-radius readers. See verification §6. |
| `internal/wallsurvey/` | Prism and revolve wall record preparation and bounded readings; revolve clearance carrier construction and bounds. See verification §6, clearance §2. |
| `internal/radiussurvey/` | Prism, revolve, cup and BRep concave-radius readings. See verification §6. |
| `internal/meshthick/` | Ray-cast wall thickness and pull undercuts of a closed triangle mesh through a bounding-volume tree. See mesh-wall-survey design. |
| `internal/gaptol/` | The gap tolerance a root operation puts on its context, and the default. See gap-tolerance §1. |
| `internal/chaintol/` | The chain tolerance carried by Boolean evaluation contexts. See operation-diagnostics-design. |
| `internal/opdiagnostic/` | Bounded plain-data summaries and aggregation for accepted Boolean and Stitch adjustments. See operation-diagnostics-design. |
| `internal/boolchain/` | Flattens a refused two-body Boolean over mesh Boolean results into source-body literals and runs the regrouped plan; `Expandable` also gates point location through history. See boolean-chain-regroup-design. |
