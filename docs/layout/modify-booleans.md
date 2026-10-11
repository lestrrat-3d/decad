# decad file layout: modify and booleans

Modify operations and the boolean files.
The rules for rows live in `docs/layout.md`.

## Layout

### Modify

| Path | Responsibility |
|---|---|
| `fillet.go` | Fillet entry and section rewrite adapter. See modify §6. |
| `loft_fillet.go` | Rewrites matching loft records at selected analytic and fitted-spline root corners. See loft §17. |
| `loft_fillet_audit.go` | Audits rewritten loft sections and nonadjacent clearance. See loft §17. |
| `loft_fit_fillet.go` | Finds a numerical fit/circle root-fillet candidate. See loft §17. |
| `loft_fit_root_cert.go` | Certifies the ideal fit/circle tangent contact. See loft §17. |
| `loft_fit_arc_proof.go` | Bounds held connectors against the ideal fillet. See loft §17. |
| `loft_fit_carrier_proof.go` | Bounds trimmed carriers against ideal trims. See loft §17. |
| `loft_fit_contact_audit.go` | Excludes recorded fit/connector contacts. See loft §17. |
| `loft_fit_circle_audit.go` | Excludes recorded circle/connector contacts. See loft §17. |
| `loft_fit_ideal_audit.go` | Excludes ideal fit/connector contacts. See loft §17. |
| `loft_chamfer.go` | Builds both outer cap bands on a matching axial loft's held polygon. See loft §18. |
| `cap_edge_cutter.go` | Applies `internal/capedge/` admission and cutters to one oblique prism cap-edge fillet or chamfer. |
| `chamfer.go` | `Body.Chamfer`; cap loops route to `capblend.go`. See modify §7. |
| `modify_options.go` | Public reach option tiers, codec adapters and asymmetric reference resolution (SX3). See modify-reach §2, §6. |
| `tangent_chain.go` | `WithTangentChain` expansion. See modify-reach §5. |
| `fillet_audit.go` | Adapts corner cutbacks to `internal/sectionaudit/` and renders its detailed refusals. See modify §5. |
| `shell.go` | Public `ShellOption`, `ShellSense` and `Body.Shell`: a prism tube, cup or band group, or a side opening. See modify §8. |
| `shell_offset.go` | Adapts Shell section offsets and proofs; audits the record. See modify §7–§9. |
| `shell_cup.go` | `cupPayload`, its stacked record and view. See modify-reach §9.1. |
| `shell_revolve.go` | `Body.Shell` of a revolve. See modify-reach §9.3. |
| `shell_opening.go` | Prism side opening: classify removed faces and dispatch the audited region build. See shell-opening §3–§5. |
| `shell_opening_brep.go` | Records a prism side opening from the section or stack engine. See shell-opening §4. |
| `shell_stacked_boss.go` | Builds and measures the exact two-slab rectangular-boss shell. See modify-general §3.1a. |
| `shell_round_boss.go` | Builds and measures the exact two-slab circular-boss shell. See modify-general §3.1c. |
| `shell_blind_pocket.go` | Builds and measures the exact two-slab rectangular blind-pocket shell. See modify-general §3.1b. |
| `brep_modify.go` / `brep_modify_prism.go` | Brep and stacked modify receivers: dispatch, SB1/SB2, route P. See brep-modify. |
| `brep_shell.go` / `brep_shell_curved.go` | Route S: through-cut shell and one circular tool-wall opening. See modify-general §3. |
| `brep_shell_rim.go` | Route S rims. See modify-general §3.3. |
| `brep_modify_edge.go` | Route E: Table EB, restatement, blends, trims and closure. See brep-modify §5. |
| `brep_modify_loop.go` / `brep_loop_band.go` | Route L: record rewrite, bands and mass. See modify-general §4. |
| `brep_modify_chain.go` | Route L: selected cap-edge chains and partial-loop record adapters over `internal/offset2d/` and `internal/capband/`. |
| `brep_modify_partial_chamfer.go` | Rewrites two adjacent straight edges on part of a brep cap loop, including terminal lines. See modify-general §4.3a. |
| `brep_loop_fillet.go` | Route L's fillet topology and mass adapters over `internal/filletband/`. See loop-fillet. |
| `brep_loop_partial_fillet.go` | Closes partial-loop fillet bands with corner patches and terminal arcs. |
| `brep_loop_partial_chamfer.go` | Closes two selected chamfer strips and integrates their exact planar mass. See modify-general §4.3a. |
| `brep_all_edge_chamfer.go` | Builds and measures the twelve simultaneous outer bevels of an exact cross-drilled box. See modify-general §4.3b. |
| `brep_all_edge_placement.go` | Rebuilds that chamfer after an exact signed-axis placement and maps face roles. See modify-general §4.3b. |

### Booleans

| Path | Responsibility |
|---|---|
| `boolean.go` | Public `Union`/`Cut`/`Intersect` and the `BooleanError` mapping. See evaluator §9. |
| `point_cone_trim.go` | Retains point-loft and cone source faces through coaxial Cut/Intersect. See loft-point design. |
| `bevel_join_source.go` | Matches the live Sketch tooth source to the finite blank root meridian. See loft-point design. |
| `bevel_join.go` | Rebuilds and audits one shared-root bevel blank and tooth Union. See loft-point design. |
| `bevel_join_two.go` | Rebuilds two separate point-loft teeth on one bevel blank root ring. See loft-point design. |
| `loft_cut.go` | Preserves a matching axial loft through a certified coaxial circle-bore Cut. See loft §18. |
| `prism_boolean.go` | Analytic booleans of co-directional prisms. See `docs/prism-boolean-design.md`. |
| `prism_boolean_nesting.go` | Clean-nesting scene adapters. See prism-boolean §4.2. |
| `prism_boolean_blind.go` | Blind and spanning Cuts via the whole-loop match. See `docs/prism-boolean-design.md` §3.2. |
| `stacked_prism.go` | Stacked slabs, walls and measurements. See `docs/stacked-prism-design.md`. |
| `prism_group.go` | Prism-group `Cut` tools and disjoint `Union` results. See general-boolean A5. |
| `stacked_union.go` / `stacked_union_brep.go` | A1 `Union`: slabs or a brep. See general-boolean A1. |
| `prism_boolean_crossing.go` | Cut/Intersect's crossing resolution. See prism-boolean §4.2. |
| `prism_overlap.go` | Prism-boolean §4.5's overlap-area reading for `Verify`. See its doc comment. |
| `brep_payload.go` / `brep_measure.go` | BRep face views, topology and readings. See general-boolean §4. |
| `classb.go` | Class-B admission and result adapters. See general-boolean §3 B. |
| `classb_crossing.go` | Class-B scene and BRep adapters. See general-boolean §5, §10. |
| `surface_trim.go` / `surface_split_revolve.go` | Surface body gates and revolve `Split`. See surface-intersection §2–§3. |
| `boolean_mesh.go` | Prepares an operand's mesh for `internal/meshbool/`. See evaluator §9. |
| `boolean_body.go` | Builds faceted bodies and readings from an audited mesh. See evaluator §9. |
