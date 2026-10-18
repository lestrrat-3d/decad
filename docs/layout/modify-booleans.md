# decad file layout: modify

Modify operations.
The rules for rows live in `docs/layout.md`.

## Layout

### Modify

| Path | Responsibility |
|---|---|
| `fillet.go` | Fillet entry and section rewrite adapter. See modify §6. |
| `prism_union_fillet.go` | Authenticates one untouched outer edge of a faceted prism Union and rebuilds its isolated source fillet. See modify-reach §10.1. |
| `prism_union_chamfer.go` | Reuses an authenticated outer edge and corner separation to rebuild one isolated source chamfer. See modify-reach §10.1. |
| `draft_circle_fillet.go` | Rebuilds a positive-taper circular draft's far cap-rim fillet as a bounded full meridian revolution. See draft §10.4. |
| `draft_circle_chamfer.go` | Rebuilds a positive-taper circular draft's far cap-rim chamfer as a bounded full meridian revolution. See draft §10.5. |
| `draft_circle_shell.go` | Builds the inward cup of a positive-taper circular draft through a bounded meridian revolution. See draft §10.6. |
| `loft_fillet.go` | Rewrites matching loft records at selected analytic and fitted-spline root corners. See loft §17. |
| `oblique_boss_fillet.go` | Admits the junction edge of a complete tilted cylinder boss-root fillet, then builds and proves the mesh through `internal/obliqueboss/` and commits the faceted result. See oblique-boss-fillet design. |
| `cross_cylinder_bore_chamfer.go` / `cross_cylinder_bore_chamfer_mesh.go` | Admits the perpendicular bore-mouth edge, reads source face groups, and asks `internal/crossbore/` to assemble the held mesh. See cross-cylinder-bore-chamfer design. |
| `cross_cylinder_bore_chamfer_proof.go` | Checks the exact mirror pose from source bodies, then asks `internal/crossbore/` for cell, boundary, volume and area proofs. See cross-cylinder-bore-chamfer design. |
| `loft_chamfer.go` | Builds both outer cap bands on a matching axial loft's held polygon. See loft §18. |
| `cap_edge_cutter.go` | Applies `internal/capedge/` admission and cutters to one oblique prism cap-edge fillet or chamfer. |
| `brep_rounded_edge_fillet.go` | Rounds P8's one front top edge after matching its exact rounded, cross-drilled BRep record. See modify-general §4.3c. |
| `chamfer.go` | `Chamfer`; cap loops route to `capblend.go`. See modify §7. |
| `modify_options.go` | Public reach option tiers, codec adapters and asymmetric reference resolution (SX3). See modify-reach §2, §6. |
| `tangent_chain.go` | `WithTangentChain` expansion. See modify-reach §5. |
| `fillet_audit.go` | Renders `internal/sectionaudit/`'s detailed refusals. See modify §5. |
| `shell.go` | Public `ShellOption`, `ShellSense` and `Shell`: prism, Sweep and Revolve dispatch. See modify §8 and Table R. |
| `shell_twist.go` | Admits and commits the inward top-cap Shell of a straight twisted rounded Sweep. See shell-twisted-sweep design. |
| `shell_twist_mesh.go` | Builds the five-face faceted cup from the proven twisted-shell mesh and audits it. See shell-twisted-sweep design. |
| `shell_offset.go` | Binds `internal/offset2d/`'s section offset and its displacement to `shellTol`. See modify §7–§9. |
| `shell_revolve.go` | `Shell` of a revolve. See modify-reach §9.3. |
| `shell_opening.go` | Prism and straight Sweep side openings: classify removed faces and dispatch the audited region build. See shell-opening §3–§5. |
| `shell_opening_brep.go` | Records a prism side opening whose caps are both removed; `breprecord` records the stacked case. See shell-opening §4. |
| `shell_stacked_boss.go` | Admits and records the exact two-slab rectangular-boss shell; `breprecord` builds and measures it. See modify-general §3.1a. |
| `shell_prism_union_boss.go` | Authenticates the faceted nested boss, then audits and commits the inward cup mesh from `prismshell`. See shell-faceted-boss design. |
| `shell_round_boss.go` | Admits and records the exact two-slab circular-boss shell; `breprecord` builds and measures it. See modify-general §3.1c. |
| `shell_blind_pocket.go` | Admits and records the exact two-slab rectangular blind-pocket shell; `breprecord` builds and measures it. See modify-general §3.1b. |
| `shell_counterbore.go` | Admits and records the exact two-level coaxial counterbore shell; `breprecord` builds and measures it. See modify-general §3.1e. |
| `brep_modify.go` / `brep_modify_prism.go` | Brep and stacked modify receivers: dispatch, SB1/SB2, route P. See brep-modify. |
| `brep_shell.go` / `brep_shell_curved.go` | Route S: through-cut shell and one circular tool-wall opening. See modify-general §3. |
| `brep_shell_rim.go` | Route S rims. See modify-general §3.3. |
| `brep_modify_edge.go` | Route E: Table EB, restatement, blends, trims and closure. See brep-modify §5. |
| `brep_modify_loop.go` / `brep_loop_band.go` | Route L: record rewrite and the bands' mesh-boolean admission; `breprecord` builds the bands and their mass. See modify-general §4. |
| `brep_modify_chain.go` | Route L: selected cap-edge chains and partial-loop record adapters over `internal/offset2d/` and `internal/capband/`. |
| `brep_modify_partial_chamfer.go` | Rewrites one or two straight edges on part of a brep cap loop, including terminal lines. See modify-general §4.3a. |
| `brep_loop_fillet.go` | Route L's fillet-band pull survey (DF7). See loop-fillet. |
| `brep_all_edge_chamfer.go` | Admits an exact cross-drilled box and its twelve selected outer edges, then builds the all-edge chamfer through `breprecord`. See modify-general §4.3b. |
