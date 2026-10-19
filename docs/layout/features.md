# decad file layout: features

Feature files and the cap-loop chamfer.
The rules for rows live in `docs/layout.md`.

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
| `thicken.go` | Public `Thicken` gates, payload dispatch, patch build, prism tube/annulus and chain-sheet publication, and one-span Sweep role restoration; revolve and chain builders live in `internal/thickenbuild/`. See surface §16. |
| `offset.go` | `Offset`: a second sheet at a normal distance. See surface §17. |
| `patch_body.go` | `PatchEdges` selection, chain gates 1-2 and exact-plane topology adapter; the level proof and Rule P live in `internal/patchchain/`. See surface §5.2. |
| `stitch.go` | `Stitch`; builds through `internal/stitchrecord/`. See surface §6.4. |
| `unstitch.go` | `Unstitch` sheet split; builds through `internal/stitchrecord/`. See surface §6.5. |
| `extrude.go` | Generic `Extrude` over a profile or chain, `WithTaper`, linear-extent resolution, the chain ribbon entry point. See evaluator §5 and surface §13.2. |
| `draft.go` | `Draft`, `NeutralPlane`, `Walls` gates, plus the tapered extrude angle and surface-result gates, extent resolution and slab dispatch. See draft §5, §7, §10. |
| `sweep.go` | Generic `Sweep` over a profile or chain and path admission adapters, including the cardinal line/arc chain and the full-circle path reduction; builds through `internal/sweeprecord/`. See sweep §3.3, §15.8–§15.9. |
| `sweep_mitre.go` | Mitred options, entry adapters and restatement; builds through `sweepmitre.Build`. See sweep §16. |
| `coil.go` | `Coil`, `CoilOption`, Table CS gates. Root `coilPayload` aliases `coilshell.Payload`. See helix §2, §4, §5.6. |
| `revolve.go` | Generic `Revolve` over a profile or chain: `Axis` variants, options, angular extents, and the public axis adapter to `internal/revolveaxis/`. See evaluator §6 and surface §13.2. |
| `revolve_blend.go` | Fillet/Chamfer of revolve meridian junctions. See modify-reach §7. |
| `revolve_cap_edge_blend.go` | Admits the quarter-turn radial cap rim, binds its source faces and publishes the mesh from `internal/revolvemesh/`. See modify-reach §7.2. |
| `stops.go` | Body-relative stop resolution: live stop bodies, face selection, through-all and angular stops; `internal/extent` admits a `ToFace` face and computes its level. See evaluator §5/§6/§11. |
| `internal/extent/stop_revolve.go` | Exact `ToFace` level for a planar Revolve wall or angular cap. See evaluator §5. |
| `loft.go` | Loft and LoftChain entry points and options. Root `chainLoftPayload` aliases `loftrecord.ChainPayload`. See loft §2/§4/§10/§16. |
| `loft_sections.go` | Public smooth-section loft entry point and source-face assembly. See loft-sections design. |
| `cone_from_face.go` | `ConeFromFace`: the full tool cone one float step off a Revolve blank's selected conical face. See loft-face-sections design. |
| `loft_faces.go` | Bounded source-cap selection checks, the commit of the loft `internal/revolveloftfaces` assembles and proves, shared rational helpers and map aliases, and `LoftFacesToConeTangent` with `ConeTangentEnd`. See loft-face-sections design. |
| `loft_point.go` | Admits a point-section Loft from Sketch and publishes the `internal/pointloft/` payload. See loft-point design. |

### Cap-loop chamfer

| Path | Responsibility |
|---|---|
| `capblend.go` | Complete-cap-loop chamfer: selection classification, build gates and the hand-off to `internal/capblend/`. See modify-reach §8.3/§4. |
