# decad file layout: features

Feature files and the cap-loop chamfer.
The rules for rows live in `agent-docs/layout.md`.

## Layout

### Features

| Path | Responsibility |
|---|---|
| `topology.go` | Topology types from `Body` to `Vertex` and public surface and curve variants. See API §6.1 and evaluator §3. |
| `topology_handles.go` | Born-handle constructors, the node-to-handle mapping of linked topology nodes, `handleAlloc` and `envOf` for internal builders, `bodyResult`, and `placePayload`, the root path into `topo.Payload.Placed`. See evaluator §3, §8. |
| `document.go` | `Document`: live body set, commit, `Remove`, liveness gates, `Transform` and `Copy`; the producer identity spelling and zero-vector predicate. See evaluator §8. |
| `mirror.go` | `MirrorPlane` and `Mirrored`/`MirroredCopy`. See mirror-pattern §4. |
| `mirror_join.go` | `WithJoin`, face selection and document commit around `internal/mirrorjoin/`'s record rewrite. See mirror-pattern §5. |
| `pattern.go` | Public pattern specs, admission, Copy/Union fallback and publication; `internal/patternassembly/` plans frame-keeping instances and disjoint groups. See mirror-pattern §4.3, §6. |
| `surface.go` | `WithSurfaceResult` and the sheet refusal. See surface §2–§4, §7, §11. |
| `feature_options.go` | Public feature option tiers and adapters to `internal/featureoption/`. See surface §4, sweep §2 and loft §2. |
| `patch.go` | `PatchProfile`: records the profile and builds its face through `patchchain.Build`. See surface §5.1. |
| `hole.go` | Public `Hole`, `HoleSpec`, depth and head variants: gates, tool sketches, chained cut and the commit; `internal/holeplan/` computes the tools. See hole design. |
| `rib.go` | Public `Rib`, `Web` and `WithRibOverrun`: line, thickness and extent gates, rib tool extrusion, touch and volume checks. See rib design. |
| `emboss.go` | Public `Emboss` and `EmbossMode`: gates, sketch-plane check, tool extrusion, join and volume checks. See emboss design. |
| `planar_face.go` | Resolves a selector to one analytic planar face and its outward normal; `Hole` and `Emboss` share it. |
| `tool_combine.go` | Shared by `Hole` and `Rib`: chains boolean steps over tool bodies, commits once, and removes the tools. |
| `thicken.go` | Public `Thicken` option, thickness and sheet gates and the commit; `internal/thickenbuild/` dispatches the payload and builds. See surface §16. |
| `offset.go` | `Offset`: option, distance and sheet gates and the commit; `internal/thickenbuild/` builds the second sheet at a normal distance. See surface §17. |
| `patch_body.go` | `PatchEdges` selection and commit; the chain gates, level proof and Rule P live in `internal/patchchain/`. See surface §5.2. |
| `stitch.go` | `Stitch`; builds through `internal/stitchrecord/`. See surface §6.4. |
| `unstitch.go` | `Unstitch` sheet split; builds through `internal/stitchrecord/`. See surface §6.5. |
| `extrude.go` | Generic `Extrude` over a profile or chain, `WithTaper`, linear-extent resolution, the chain ribbon entry point. See evaluator §5 and surface §13.2. |
| `draft.go` | `Draft`, `NeutralPlane`, `Walls` gates, plus the tapered extrude angle and surface-result gates, extent resolution and slab dispatch. See draft §5, §7, §10. |
| `sweep.go` | Generic `Sweep` over a profile or chain: nil, option and seam gates, then `internal/sweepbuild/` builds and root commits. See sweep §3.3, §15.8–§15.9. |
| `sweep_mitre.go` | Mitred options and entry adapters; builds through `sweepmitre.Build`. See sweep §16. |
| `coil.go` | `Coil`, `CoilOption`, Table CS gates. Root `coilPayload` aliases `coilshell.Payload`. See helix §2, §4, §5.6. |
| `revolve.go` | Generic `Revolve` over a profile or chain: `Axis` variants, options, angular extents, and the public axis adapter to `internal/revolveaxis/`. See evaluator §6 and surface §13.2. |
| `stops.go` | Body-relative stop resolution: live stop bodies, face selection and angular stops; `internal/extent` admits a `ToFace` or `ToFaceAngular` face and computes its stop, and resolves through-all stops. See evaluator §5/§6/§11. |
| `internal/extent/stop_revolve.go` | Exact `ToFace` level for a planar Revolve wall or angular cap. See evaluator §5. |
| `loft.go` | Loft and LoftChain entry points and options. Builds through `loftrecord.FromProfiles` and `FromChains`. See loft §2/§4/§10/§16. |
| `loft_sections.go` | Public smooth-section loft entry point and source-face assembly. See loft-sections design. |
| `cone_from_face.go` | `ConeFromFace`: the full tool cone one float step off a Revolve blank's selected conical face. See loft-face-sections design. |
| `loft_faces.go` | The commit of the loft `internal/revolveloftfaces` assembles and proves, shared rational helpers and map aliases, and `LoftFacesToConeTangent` with `ConeTangentEnd`. See loft-face-sections design. |
| `loft_point.go` | Admits a point-section Loft from Sketch and publishes the `internal/pointloft/` payload. See loft-point design. |

### Cap-loop chamfer

| Path | Responsibility |
|---|---|
