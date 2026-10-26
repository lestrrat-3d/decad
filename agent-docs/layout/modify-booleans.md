# decad file layout: modify

Modify operations.
The rules for rows live in `agent-docs/layout.md`.

## Layout

### Modify

| Path | Responsibility |
|---|---|
| `fillet.go` | `Fillet`: options, selector, tangent chain and thickness gates, then `internal/brepmodify/` builds the fillet. See modify §6. |
| `chamfer.go` | `Chamfer`: options, selector, tangent chain, asymmetric reference and setback gates, then `internal/brepmodify/` builds the chamfer. See modify §7. |
| `chamfer_angle.go` | Public `ChamferAngle`: reads the angle, gates each edge as right-angled, then calls `Chamfer` with the second setback. See chamfer-angle design. |
| `modify_options.go` | Public reach option tiers, codec adapters and asymmetric reference resolution (SX3). See modify-reach §2, §6. |
| `shell.go` | Public `ShellOption`, `ShellSense` and `Shell`: options, selector and thickness gates, then `internal/brepmodify/` builds the shell. Also `shellTol`. See modify §7–§9 and Table R. |
| `brep_modify.go` | The prism cap faces a modify op classifies against, the modify commit, and the conversions to `internal/brepmodify/`, which owns the brep and stacked receiver routes. See brep-modify. |
| `brep_modify_partial.go` | `WithTangentChain`'s public handles, selectors and refusals over `internal/tangentchain/`. See modify-reach §5. |
