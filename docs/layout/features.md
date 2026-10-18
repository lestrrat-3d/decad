# decad file layout: features

Feature files and the cap-loop chamfer.
The rules for rows live in `docs/layout.md`.

## Layout

### Features

| Path | Responsibility |
|---|---|
| `topology.go` | Topology types from `Body` to `Vertex` and public surface and curve variants. See API §6.1 and evaluator §3. |
| `topology_handles.go` | Born-handle constructors, the node-to-handle mapping of linked topology nodes, `handleAlloc` and `envOf` for internal builders, `bodyResult`, and `placePayload`, the root path into `topo.Payload.Placed`. See evaluator §3, §8. |
| `document.go` | `Document`: live body set, commit, `Remove`, liveness gates, `Transform` and `Copy`. See evaluator §8. |
| `mirror.go` | `MirrorPlane` and `Mirrored`/`MirroredCopy`. See mirror-pattern §4. |
| `mirror_join.go` | `WithJoin`, face selection and document commit around `internal/mirrorjoin/`'s record rewrite. See mirror-pattern §5. |
| `pattern.go` | Public pattern specs, entry points and payload adapters. See mirror-pattern §4.3, §6. |
| `surface.go` | `WithSurfaceResult`, sheet refusal and the handle adapter for `internal/topobuild/`'s free-edge chain counts. See surface §2–§4, §7, §11. |
| `feature_options.go` | Public feature option tiers and adapters to `internal/featureoption/`. See surface §4, sweep §2 and loft §2. |
| `patch.go` | `PatchProfile`: records the profile and builds its face through `patchchain.Build`. See surface §5.1. |
| `thicken.go` | `Thicken`. See surface §16. |
| `thicken_prism.go` | Builds the certified wall of a prism sheet. See surface §16.2. |
| `thicken_chain_sweep.go` | Thickens admitted one-arc and cardinal composite chain sheets. See surface §16.8. |
| `offset.go` | `Offset`: a second sheet at a normal distance. See surface §17. |
| `patch_body.go` | `PatchEdges` selection, chain gates 1-2 and exact-plane topology adapter; the level proof and Rule P live in `internal/patchchain/`. See surface §5.2. |
| `denotation.go` | Level and curve token names and comparisons; the document node mints them. |
| `stitch.go` | `Stitch`; builds through `internal/stitchrecord/`. See surface §6.4. |
| `unstitch.go` | `Unstitch` sheet split; builds through `internal/stitchrecord/`. See surface §6.5. |
| `extrude.go` | Generic `Extrude` over a profile or chain, `WithTaper`, linear-extent resolution, the chain ribbon entry point. See evaluator §5 and surface §13.2. |
| `draft.go` | `Draft`, `NeutralPlane`, `Walls` gates. See draft §10. |
| `draft_build.go` | Tapered extrude angle and surface-result gates, extent resolution and slab dispatch. See draft §5, §7. |
| `section_input.go` | `Section`: the profile-or-chain constraint a section-fed operation is generic over. See surface §13.2. |
| `sweep.go` | Generic `Sweep` over a profile or chain and path admission adapters; builds through `internal/sweeprecord/`. See sweep design. |
| `sweep_chain_composite.go` | Admits the cardinal line/arc chain Sweep and builds it through `sweeprecord.ReplayChainWork`. See sweep §15.8. |
| `sweep_arc_chain.go` | Reduces a one-arc chain Sweep to the authenticated chain Revolve wall. See sweep §15.9. |
| `sweep_closed_circle.go` | Checks the exact two-arc full-circle path and evaluates its full Revolve reduction. See sweep §3.3. |
| `sweep_mitre.go` | Mitred options, entry adapters and restatement; builds through `sweepmitre.Build`. See sweep §16. |
| `coil.go` | `Coil`, `CoilOption`, Table CS gates. Root `coilPayload` aliases `coilshell.Payload`. See helix §2, §4, §5.6. |
| `revolve.go` | Generic `Revolve` over a profile or chain: `Axis` variants, options, angular extents. See evaluator §6 and surface §13.2. |
| `revolve_blend.go` | Fillet/Chamfer of revolve meridian junctions. See modify-reach §7. |
| `revolve_cap_edge_blend.go` | Bounded quarter-turn radial cap-rim fillet and chamfer with faceted proofs. See modify-reach §7.2. |
| `revolve_axis.go` | Adapts the public axis to `internal/revolveaxis/`. See evaluator §6. |
| `stops.go` | Body-relative stop resolution. See evaluator §5/§6/§11. |
| `internal/extent/stop_revolve.go` | Exact `ToFace` level for a planar Revolve wall or angular cap. See evaluator §5. |
| `loft.go` | Loft and LoftChain entry points and options. Root `chainLoftPayload` aliases `loftrecord.ChainPayload`. See loft §2/§4/§10/§16. |
| `loft_sections.go` | Public smooth-section loft entry point and source-face assembly. See loft-sections design. |
| `cone_from_face.go` | `ConeFromFace`: the full tool cone one float step off a Revolve blank's selected conical face. See loft-face-sections design. |
| `loft_faces.go` | Bounded source-cap loft admission and held wall assembly. See loft-face-sections design. |
| `loft_faces_tangent.go` | `LoftFacesToConeTangent` and `ConeTangentEnd`: the LoftFaces loft whose last section is the plane exactly tangent to a point cone. See loft-face-sections design. |
| `loft_faces_proof.go` | Exact cap-map, band embedding and occupied-volume proof for face-fed lofts. See loft-face-sections design. |
| `loft_point.go` | Point-section Loft entry, apex fan and proof. See loft-point design. |
| `loft_build.go` | Tessellation adapter for a lofted body. Root `loftPayload` aliases `loftrecord.Payload`. See loft §5, §8, §12. |

### Cap-loop chamfer

| Path | Responsibility |
|---|---|
| `capblend.go` | Complete-cap-loop chamfer: selection classification, build gates and the hand-off to `internal/capblend/`. See modify-reach §8.3/§4. |
