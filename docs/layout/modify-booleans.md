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
| `draft_circle.go` | Admits the live far rim and commits its fillet or chamfer, or the far cap and inward sense for the cup; `internal/draftrecord/` builds the bounded meridian. See draft §10.4–§10.6. |
| `loft_fillet.go` | Rewrites matching loft records at selected analytic and fitted-spline root corners. See loft §17. |
| `oblique_boss_fillet.go` | Admits the junction edge of a complete tilted cylinder boss-root fillet, then builds and proves the mesh through `internal/obliqueboss/` and commits the faceted result. See oblique-boss-fillet design. |
| `cross_cylinder_bore.go` | Recognizes the two circular prism sources a perpendicular cylinder Cut retains, admits the bore-mouth edge chamfer, and asks `internal/crossbore/` for the held mesh and its proofs. See cross-cylinder-bore-chamfer design. |
| `loft_chamfer.go` | Builds both outer cap bands on a matching axial loft's held polygon. See loft §18. |
| `cap_edge_cutter.go` | Applies `internal/capedge/` admission and cutters to one oblique prism cap-edge fillet or chamfer. |
| `brep_rounded_edge_fillet.go` | Rounds P8's one front top edge after matching its exact rounded, cross-drilled BRep record. See modify-general §4.3c. |
| `chamfer.go` | `Chamfer`; cap loops route to `capblend.go`. See modify §7. |
| `modify_options.go` | Public reach option tiers, codec adapters and asymmetric reference resolution (SX3). See modify-reach §2, §6. |
| `shell.go` | Public `ShellOption`, `ShellSense` and `Shell`: prism, Sweep and Revolve dispatch, and the section offset bound to `shellTol`. See modify §7–§9 and Table R. |
| `shell_twist.go` | Admits and commits the inward top-cap Shell of a straight twisted rounded Sweep, and builds its five-face faceted cup from the proven mesh. See shell-twisted-sweep design. |
| `shell_revolve.go` | `Shell` of a revolve. See modify-reach §9.3. |
| `shell_opening.go` | Prism and straight Sweep side openings: classify removed faces, dispatch the audited region build and record the cap-removed prism case. See shell-opening §3–§5. |
| `shell_stacked.go` | Admits and records the exact stacked shells (nested boss, circular boss, rectangular blind pocket, coaxial counterbore); `prismshell` reverses a closed source and `breprecord` builds and measures it. See modify-general §3.1. |
| `shell_prism_union_boss.go` | Authenticates the faceted nested boss and commits the inward cup proved by `prismshell`. See shell-faceted-boss design. |
| `brep_modify.go` | Brep and stacked modify receivers: dispatch, SB1/SB2, route P. See brep-modify. |
| `brep_shell.go` | Route S: live face selection, stage order, result commit, and one circular tool-wall opening. `internal/throughshell/` owns its record geometry. See modify-general §3. |
| `brep_modify_edge.go` | Route E: public edge and reference-face checks, ordered gate dispatch and result commit. `breprecord` reads and rewrites the record and checks reference-face roles. See brep-modify §5. |
| `brep_modify_loop.go` | Route L/V public edge and face adapters and result commit; `breploop` selects and rewrites the record, and `breprecord` builds the bands and mass. See modify-general §4. |
| `brep_modify_partial.go` | Route L: selected cap-edge chains, partial-loop fillet and chamfer rewrites including terminal lines, and `WithTangentChain`'s public handles, selectors and refusals. See modify-reach §5 and modify-general §4.3a. |
| `brep_loop_fillet.go` | Route L's fillet-band pull survey (DF7). See loop-fillet. |
| `brep_all_edge_chamfer.go` | Admits an exact cross-drilled box and its twelve selected outer edges, then builds the all-edge chamfer through `breprecord`. See modify-general §4.3b. |
