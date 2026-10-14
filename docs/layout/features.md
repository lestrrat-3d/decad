# decad file layout: features

Feature files and the cap-loop chamfer.
The rules for rows live in `docs/layout.md`.

## Layout

### Features

| Path | Responsibility |
|---|---|
| `topology.go` | Topology types from `Body` to `Vertex` and public surface and curve variants. See API §6.1 and evaluator §3. |
| `topology_handles.go` | Born-handle constructors and the node-to-handle mapping of linked topology nodes, and `placePayload`, the root path into `topo.Payload.Placed`. See evaluator §3, §8. |
| `document.go` | `Document`: live body set, commit, `Remove`, liveness gates, `Transform` and `Copy`. See evaluator §8. |
| `mirror.go` | `MirrorPlane` and `Mirrored`/`MirroredCopy`. See mirror-pattern §4. |
| `mirror_join.go` | `WithJoin` adapters over `internal/mirrorjoin/`'s splice and section audit. See mirror-pattern §5. |
| `pattern.go` | Public pattern specs, entry points and payload adapters. See mirror-pattern §4.3, §6. |
| `surface.go` | `WithSurfaceResult`, sheet refusal, shell/lump and free-edge adapters over `internal/surfacegroup/`. See surface §2-§4, §7, §11. |
| `feature_options.go` | Public feature option tiers and adapters to `internal/featureoption/`. See surface §4, sweep §2 and loft §2. |
| `patch.go` | Builds a single planar face from a recorded profile. See surface §5.1. |
| `thicken.go` | `Thicken`. See surface §16. |
| `thicken_prism.go` | Builds the certified wall of a prism sheet. See surface §16.2. |
| `thicken_axis.go` | Assembles thicken sections. See surface §16. |
| `thicken_chain_sweep.go` | Thickens admitted one-arc and cardinal composite chain sheets. See surface §16.8. |
| `offset.go` | `Offset`: a second sheet at a normal distance. See surface §17. |
| `patch_body.go` | `PatchEdges` topology adapter and face build. See surface §5.2. |
| `denotation.go` | Level and curve token names and comparisons; the document node mints them. |
| `stitch_weld.go` | Adapts Stitch topology to Table J. See surface §6.2. |
| `stitch.go` | `Stitch` evaluator and topology adapter. See surface §6.4. |
| `stitch_flux.go` | Stitch face flux, mass and tag adapters. See surface §6.4. |
| `unstitch.go` | `Unstitch` sheet split and placement. See surface §6.5. |
| `extrude.go` | Generic `Extrude` over a profile or chain, `WithTaper`, linear-extent resolution, the chain ribbon build. See evaluator §5 and surface §13.2. |
| `draft.go` | `Draft`, `NeutralPlane`, `Walls` gates. See draft §10. |
| `draft_payload.go` | `draftPayload`, its band view and offset span. See draft §6, §8.1. |
| `draft_build.go` | Tapered extrude gates and assembly. See draft §5, §7. |
| `draft_two_sided.go` | Two-slab draft assembly and placement. See draft §7. |
| `draft_moments.go` | Draft body measurements. See draft §8. |
| `draft_survey.go` | Draft body undercut survey. See draft DD7. |
| `section_input.go` | `Section`: the profile-or-chain constraint a section-fed operation is generic over. See surface §13.2. |
| `sweep.go` | Generic `Sweep` over a profile or chain, path admission adapters and span payloads. See sweep design. |
| `sweep_chain_composite.go` | Certified cardinal line/arc chain sheet, indexed rim join, separation and readings. See sweep §15.8. |
| `sweep_arc_chain.go` | Reduces a one-arc chain Sweep to the authenticated chain Revolve wall. See sweep §15.9. |
| `sweep_twist.go` | Symmetric Line/Arc twist admission, true-shape readings and source-face area proofs. See sweep §17. |
| `sweep_twist_local_area.go` | Local absolute wall-area proof for authenticated Line/Arc twist cells and Boolean subsets. See sweep §17.4. |
| `sweep_twist_stations.go` | Straight-path twist stations, bounded Loft cells and occupied-volume proof. See sweep §§17.3–17.4. |
| `sweep_twist_composite.go` | Twist Loft cell sewing, source wall roles and mesh proof; cardinal composite path. See sweep §§17.2–17.4. |
| `sweep_arc.go` | Adapts `internal/sweeparc/` to the one-span `ArcThrough` reduction and Revolve build. See sweep §3. |
| `sweep_closed_circle.go` | Checks the exact two-arc full-circle path and evaluates its full Revolve reduction. See sweep §3.3. |
| `sweep_closed_racetrack.go` | Admits exact-turn planar circular-section loops and routes the cyclic span build. See sweep §§3.4–3.5. |
| `sweep_composite.go` | Composite Sweep join topology, boundary adapter over `internal/surfacegroup/`, surface-result caps. See sweep PR 4. |
| `sweep_composite_measure.go` | Composite Sweep span replay and body measurement adapters. See `docs/sweep-design.md` PR 4. |
| `sweep_audit.go` | Adapts built span caps, bounds and extents to `internal/compositesweep/`'s separation audit. See sweep §7. |
| `sweep_mitre.go` | Mitred options, entry adapters, payload, placement and restatement. See sweep §16. |
| `sweep_mitre_build.go` | Adapts mitred construction and runs the SM8 audit. See sweep §16.3–§16.4. |
| `sweep_mitre_body.go` | Builds Table BM's topology and publishes §16.6's readings. See sweep §16.5–§16.6. |
| `coil.go` | `Coil`, `CoilOption`, Table CS gates, payload and placement. See helix §2, §4, §5.6. |
| `coil_body.go` | The coil's Table CB topology and Table CM readings. See helix §6–§7. |
| `prism_payload.go` | `prismPayload`, the root wrapper of `prismrecord.Payload`: re-evaluation and root-interface readings. See evaluator §8. |
| `prism_build.go` | `evalPrismContext`, caps, and side faces. See evaluator §5. |
| `prism_extent.go` | Adapts prism extents and box readings. See evaluator §5. |
| `revolve.go` | Generic `Revolve` over a profile or chain: `Axis` variants, options, angular extents. See evaluator §6 and surface §13.2. |
| `revolve_blend.go` | Fillet/Chamfer of revolve meridian junctions. See modify-reach §7. |
| `revolve_cap_edge_blend.go` | Bounded quarter-turn radial cap-rim fillet and chamfer with faceted proofs. See modify-reach §7.2. |
| `revolve_axis.go` | Adapts the public axis and bounded profile scan to `internal/revolveaxis/`. See evaluator §6. |
| `revolve_build.go` | Builds a revolve's solid or sheet body and its measurements. See evaluator §6. |
| `revolve_extent.go` | Adapts revolve extent and box readings to `internal/revolveaxis/`. See evaluator §6. |
| `revolve_denotation.go` | Payload sweep bounds over `internal/revolveangle/` proofs. See evaluator §6 and sweep §3. |
| `stops.go` | Body-relative stop resolution. See evaluator §5/§6/§11. |
| `internal/extent/stop_revolve.go` | Exact `ToFace` level for a planar revolved wall. See evaluator §5. |
| `loft.go` | Loft entry points and chain ribbon assembly. See loft §2/§4/§10/§16. |
| `loft_sections.go` | Public smooth-section loft entry point and source-face assembly. See loft-sections design. |
| `cone_from_face.go` | `ConeFromFace`: the full tool cone one float step off a Revolve blank's selected conical face. See loft-face-sections design. |
| `loft_faces.go` | Bounded source-cap loft admission and held wall assembly. See loft-face-sections design. |
| `loft_faces_tangent.go` | `LoftFacesToConeTangent` and `ConeTangentEnd`: the LoftFaces loft whose last section is the plane exactly tangent to a point cone. See loft-face-sections design. |
| `loft_faces_proof.go` | Exact cap-map, band embedding and occupied-volume proof for face-fed lofts. See loft-face-sections design. |
| `loft_point.go` | Point-section Loft entry, live Sketch source check, apex fan and proof. See loft-point design. |
| `loft_build.go` | Loft payload, evaluation, placement and tessellation adapter. See loft §5, §8, §12. |
| `loft_topology.go` | Builds loft `Body` topology over `internal/loftmesh/` assembly. See loft §5.1, §7. |
| `loft_circle_bore_topology.go` | Proves exact section axes and circle-center lifts before publishing direct Loft curved walls and circular rims; retains the certified through-Cut cylinder path. See loft §18. |
| `loft_moments.go` | Loft mass adapters and chord proofs. See loft §8, §12. |

### Cap-loop chamfer

| Path | Responsibility |
|---|---|
| `capblend.go` | Complete-cap-loop chamfer: `capBlendPayload` and build gates. See modify-reach §8.3/§4. |
| `capblend_geom.go` | `buildCapBand`: band patches and cap edges, also a draft's walls. See modify-reach §8.3. |
| `capblend_contour.go` | Adapts built corner and edge records to `internal/capband/` contour and closure proofs. |
| `capblend_moments.go` | Builds the cap-blend body and adapts its mass readings to `internal/capband/`. See modify-reach §8.4. |
| `capblend_survey.go` | Cap-blend undercut and minimum-radius surveys. See modify-reach Table DX (DX7/DX8). |
| `capblend_normal.go` | Reads band-patch tags for DX7's normal model. |
| `capblend_departure.go` | Reads band-patch tags and built edges for departure bounds. |
| `capblend_admit.go` | Adapts cap-band occupied-volume admission. See tessellation-reach §7. |
