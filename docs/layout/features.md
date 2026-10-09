# decad file layout: features

Feature files and the cap-loop chamfer.
The rules for rows live in `docs/layout.md`.

## Layout

### Features

| Path | Responsibility |
|---|---|
| `topology.go` | Topology types from `Body` to `Vertex`; aliases for `internal/surfacegeom/`. See evaluator §3. |
| `document.go` | `Document`: live body set, commit, `Remove`, liveness gates, placement and duplication. See evaluator §8. |
| `mirror.go` | `MirrorPlane` and `Mirrored`/`MirroredCopy`. See mirror-pattern §4. |
| `mirror_join.go` | `WithJoin` adapters and section audit. See mirror-pattern §5. |
| `pattern.go` | Pattern entry points and payload adapters. See mirror-pattern §6. |
| `surface.go` | `WithSurfaceResult`, sheet refusal, shell/lump and free-edge adapters over `internal/surfacegroup/`. See surface §2-§4, §7, §11. |
| `patch.go` | Builds a single planar face from a recorded profile. See surface §5.1. |
| `thicken.go` | `Body.Thicken`. See surface §16. |
| `thicken_prism.go` | Builds the certified wall of a prism sheet. See surface §16.2. |
| `thicken_axis.go` | Assembles thicken sections. See surface §16. |
| `offset.go` | `Body.Offset`: a second sheet at a normal distance. See surface §17. |
| `patch_body.go` | `Body.Patch` topology adapter and face build. See surface §5.2. |
| `denotation.go` | Mints document tokens. |
| `stitch_weld.go` | Adapts Stitch topology to Table J. See surface §6.2. |
| `stitch.go` | `Stitch` evaluator and topology adapter. See surface §6.4. |
| `stitch_flux.go` | Stitch face flux, mass and tag adapters. See surface §6.4. |
| `unstitch.go` | `Unstitch` sheet split and placement. See surface §6.5. |
| `extrude.go` | `Document.Extrude`, `WithTaper`, linear-extent resolution. See evaluator §5. |
| `draft.go` | `Body.Draft`, `NeutralPlane`, `Walls` gates. See draft §10. |
| `draft_payload.go` | `draftPayload`, its band view and offset span. See draft §6, §8.1. |
| `draft_build.go` | Tapered extrude gates and assembly. See draft §5, §7. |
| `draft_two_sided.go` | Two-slab draft assembly and placement. See draft §7. |
| `draft_moments.go` | Draft body measurements. See draft §8. |
| `draft_survey.go` | Draft body undercut survey. See draft DD7. |
| `sweep.go` | `Document.Sweep`/`SweepChain`, path gates, span payloads. See sweep design. |
| `sweep_arc.go` | Adapts `internal/sweeparc/` to the one-span `ArcThrough` reduction and Revolve build. See sweep §3. |
| `sweep_composite.go` | Composite Sweep join topology, boundary adapter over `internal/surfacegroup/`, surface-result caps. See sweep PR 4. |
| `sweep_composite_measure.go` | Composite Sweep span replay and body measurement adapters. See `docs/sweep-design.md` PR 4. |
| `sweep_audit.go` | Adapts built span caps, bounds and extents to `internal/compositesweep/`'s separation audit. See sweep §7. |
| `sweep_mitre.go` | Mitred options, entry adapters, payload, placement and restatement. See sweep §16. |
| `sweep_mitre_build.go` | Adapts mitred construction, rounding and SM8 audit. See sweep §16.3–§16.4. |
| `sweep_mitre_body.go` | Builds Table BM's topology and publishes §16.6's readings. See sweep §16.5–§16.6. |
| `coil.go` | `Document.Coil`, `CoilOption`, Table CS gates, payload and placement. See helix §2, §4, §5.6. |
| `coil_body.go` | The coil's Table CB topology and Table CM readings. See helix §6–§7. |
| `sweep_transport.go` | Validates the path and adapts spans to `internal/sweeptransport/`. See sweep §3.2. |
| `prism_payload.go` | `prismPayload` and coordinate adapters. See evaluator §5. |
| `prism_build.go` | `evalPrismContext`, caps, and side faces. See evaluator §5. |
| `prism_extent.go` | Adapts prism extents and box readings. See evaluator §5. |
| `revolve.go` | `Document.Revolve`: `Axis` variants, options, angular extents. See evaluator §6. |
| `revolve_blend.go` | Fillet/Chamfer of revolve meridian junctions. See modify-reach §7. |
| `revolve_axis.go` | Resolves the axis, classifies walls and checks contact. Uses `internal/revolveaxis/`. See evaluator §6. |
| `revolve_build.go` | Builds a revolve's solid or sheet body and its measurements. See evaluator §6. |
| `revolve_extent.go` | Adapts revolve extent and box readings to `internal/revolveaxis/`. See evaluator §6. |
| `revolve_denotation.go` | Payload sweep bounds over `internal/revolveangle/` proofs. See evaluator §6 and sweep §3. |
| `stops.go` | Body-relative stop resolution. See evaluator §5/§6/§11. |
| `loft.go` | Loft entry points and chain ribbon assembly. See loft §2/§4/§10/§16. |
| `loft_build.go` | Loft payload, evaluation, placement and tessellation adapter. See loft §5, §8, §12. |
| `loft_topology.go` | Adapts loft assembly and builds `Body` topology. See loft §5.1, §7. |
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
