# decad file layout: modify

Modify operations.
The rules for rows live in `docs/layout.md`.

## Layout

### Modify

| Path | Responsibility |
|---|---|
| `fillet.go` | Fillet entry and section rewrite adapter; `internal/modifyselect` matches its selected edges. See modify §6. |
| `prism_union_fillet.go` | Authenticates one untouched outer edge of a faceted prism Union and rebuilds its isolated source fillet. See modify-reach §10.1. |
| `prism_union_chamfer.go` | Reuses an authenticated outer edge and corner separation to rebuild one isolated source chamfer. See modify-reach §10.1. |
| `draft_circle.go` | Admits the live far rim and commits its fillet or chamfer, or the far cap and inward sense for the cup; `internal/draftrecord/` builds the bounded meridian. See draft §10.4–§10.6. |
| `loft_chamfer.go` | Builds both outer cap bands on a matching axial loft's held polygon. See loft §18. |
| `cap_edge_cutter.go` | Applies `internal/capedge/` admission and cutters to one oblique prism cap-edge fillet or chamfer. |
| `brep_rounded_edge_fillet.go` | Rounds P8's one front top edge after matching its exact rounded, cross-drilled BRep record. See modify-general §4.3c. |
| `chamfer.go` | `Chamfer`; cap loops route to `capblend.go`. See modify §7. |
| `modify_options.go` | Public reach option tiers, codec adapters and asymmetric reference resolution (SX3). See modify-reach §2, §6. |
| `shell.go` | Public `ShellOption`, `ShellSense` and `Shell`: prism, Sweep and Revolve dispatch, the cap-shell stages over `internal/cupwall/`, and `shellTol`, the tolerance its section offsets pass to `internal/offset2d/`. Prism cap-shell records are `internal/prismshell/`'s. See modify §7–§9 and Table R. |
| `shell_twist.go` | Admits and commits the inward top-cap Shell of a straight twisted rounded Sweep, and builds its five-face faceted cup from the proven mesh. See shell-twisted-sweep design. |
| `shell_opening.go` | Prism and straight Sweep side openings: classify removed faces, dispatch the audited region build and record the cap-removed prism case. See shell-opening §3–§5. |
| `brep_modify.go` | The prism cap faces a modify op classifies against, the modify commit, and the conversions to `internal/brepmodify/`, which owns the brep and stacked receiver routes. See brep-modify. |
| `brep_modify_partial.go` | `WithTangentChain`'s public handles, selectors and refusals over `internal/tangentchain/`. See modify-reach §5. |
