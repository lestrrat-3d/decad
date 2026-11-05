# decad file layout: Booleans and modify

Boolean files and modify operations. The rules for rows live in `agent-docs/layout.md`.

## Layout

| Path | Responsibility |
|---|---|
| `boolean.go` | Public `Union`/`Cut`/`Intersect`, error mapping, scale-source marking, and the mesh Boolean over two bodies (`internal/booleaneval`). Routes: `internal/booleandispatch`, `internal/massdispatch`. See evaluator §9, operation-history-design. |
| `boolean_group_blind.go` | Binds root history and handles for bounded blind group Cuts and commits their result. `internal/prismboolean` owns cap selection and geometry proofs. See prism-boolean design. |
| `boolean_group_cache.go` | Replays and compares a cached blind Cut mesh before returning it. See tessellation design. |
| `boolean_group_history.go` | Authenticates ordered public source bodies for a prism group Union; `internal/prismboolean` replays their region and cap proofs. |
| `boolean_group_tessellate.go` | Replays a blind group Cut mesh and verifies its source history. See tessellation design. |
| `boolean_many.go` | Validates and commits multi-body Union through `internal/unionmany`, prism pair chains, and the two-at-a-time fold no certified route takes. See API §8, prism-boolean §4.6–§4.7, union-fold design. |
| `internal/unionmany/` | Orders certified many-operand Union routes and fallback prism pair chains over nodes. See prism-boolean §4.6–§4.7, axial-section-boolean, loft-cylinder-boolean, loft-mirror-boolean, prism-pattern-union and revolve-loft-sector designs. |
| `boolean_regroup.go` | Root history, handle, error and commit adapters for chain regroup and paired Cut continuation. `internal/booleandispatch` owns the private node plan. See boolean-chain-regroup-design. |
| `surface_trim.go` | Public `Trim`, `Extend` and `Split` entry checks and commits; `internal/surfacetrim/` builds them. See surface-intersection §2–§3. |
| `point_plane_scale.go` | ScaledAt: resolves the similarity source by payload kind and commits the image; facetedrecord.ScaleImage admits and scales it. See loft-point design and evaluator §8. |
| `prism_scale.go` | Admits prism and full-Revolve certified similarity sources and transfers their face roles. See evaluator §8. |
| `stack_brep_scale.go` | Admits exact line-only stacks and stack-backed BReps for certified similarity. See evaluator §8. |
| `point_cap_extremes.go` | Resolves Split cap faces and selects the ends using facetedrecord's centroid-order proof. See loft-point design. |
| `internal/boolupstream/` | Attributes an analytic Boolean result's planar faces to the one operand face exact plane identity proves each lies inside, and stores the roles in the payload's `Upstream`. See prism-boolean §11. |
| `internal/capplane/` | Reads a `CapRecord` plane, a sweep axis and plane identity as exact rationals for Loft cap levels and Boolean attribution. See prism-boolean §11, loft §18. |
| `refine.go` | Public `Refine`: re-certifies a held mesh Boolean result under a surface bound; it unwraps the faceted payload, whose `RefinedSurfaceBounds` (`internal/facetedrecord`) runs the `internal/occupancy` certificate. See body-refine §7. |
| `internal/occupancyleaf/` | Reads a Boolean operand payload (circle prism, coaxial stack, recorded Union result) as an occupancy record. See body-refine §4, §6. |

### Modify

| Path | Responsibility |
|---|---|
| `fillet.go` | `Fillet`: options, selector, tangent chain and thickness gates, then `internal/brepmodify/` builds the fillet. See modify §6. |
| `chamfer.go` | `Chamfer`: options, selector, tangent chain, asymmetric reference and setback gates, then `internal/brepmodify/` builds the chamfer. See modify §7. |
| `chamfer_wedges.go` | `chamferByWedges`: several convex edges of an axis-square mesh solid that share corners, chamfered as the solid less each edge's wedge, with the fit check on the grid. See orthogonal Boolean B17. |
| `fillet_chord.go` | Public `FilletChord`: reads the chord, gates each edge as right-angled, then calls `Fillet` with the radius. See fillet-chord design. |
| `edge_dihedral.go` | `Edge.planarDihedral`, the angle across the corner of a convex or concave planar edge that `ChamferAngle` and `FilletChord` share. |
| `chamfer_angle.go` | Public `ChamferAngle`: reads the angle, gates each edge as right-angled, then calls `Chamfer` with the second setback. See chamfer-angle design. |
| `modify_options.go` | Public reach option tiers, codec adapters and asymmetric reference resolution (SX3). See modify-reach §2, §6. |
| `move_face.go` | Public `MoveFace`: gates on a faceted receiver and one selected face, then `internal/orthoshell/` presses the face and `solidFromMeshBody` makes the result. See orthogonal-move design. |
| `quarter_turn.go` | Public `QuarterTurn`: exact quarter turns of a faceted solid about a world axis by exchanging coordinates, each sum proved exact. See orthogonal-boolean design B10. |
| `boolean_orthogonal.go` | Pair handle and result adapters for `internal/booleandispatch`; many-body admission and `internal/orthoshell/` grid assembly. See orthogonal-boolean design. |
| `shell_orthogonal.go` | The orthogonal-polyhedron route of `Shell`: gates on a faceted receiver, then `internal/orthoshell/` builds the walls and `solidFromMeshBody` makes them the result. See orthogonal-shell design. |
| `shell.go` | Public `ShellOption`, `ShellSense` and `Shell`: options, selector and thickness gates, then `internal/brepmodify/` builds the shell. Also `shellTol`. See modify §7–§9 and Table R. |
| `brep_modify.go` | The prism cap faces a modify op classifies against, the modify commit, and the conversions to `internal/brepmodify/`, which owns the brep and stacked receiver routes. See brep-modify. |
| `brep_modify_partial.go` | `WithTangentChain`'s public handles, selectors and refusals over `internal/tangentchain/`. See modify-reach §5. |
