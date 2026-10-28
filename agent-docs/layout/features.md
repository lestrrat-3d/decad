# decad file layout: features

Feature files and the cap-loop chamfer.
The rules for rows live in `agent-docs/layout.md`.

## Layout

### Features

| Path | Responsibility |
|---|---|
| `topology.go` | Topology types from `Body` to `Vertex` and public surface and curve variants. See API §6.1 and evaluator §3. |
| `topology_handles.go` | Born-handle constructors, the node-to-handle mapping of linked topology nodes, `handleAlloc` and `envOf` for internal builders, `bodyResult`, and `placePayload`, the root path into `topo.Payload.Placed`. See evaluator §3, §8. |
| `container.go` | `Container`, the document's grouping of sketches, planes and bodies; placement of results and `ErrCrossContainer`. See `agent-docs/container-design.md`. |
| `document.go` | `Document`: live body set, atomic operation publication, `Remove`, liveness gates, `Transform` and `Copy`; the producer identity spelling and zero-vector predicate. See evaluator §8 and operation history. |
| `operation_history.go` | Immutable public operation snapshots, body dependency indexes, and face-successor query values. See operation-history-design. |
| `operation_kind.go` | Shared public operation-kind vocabulary and stable string tokens. See operation-history-design and operation-diagnostics-design. |
| `operation_diagnostics.go` | Public operation diagnostic values, typed error accessors and body warning snapshots. See operation-diagnostics-design. |
| `params.go` | `WithParams`, `Document.Params`, `Param`, `Expr`, `ParamInt`, `Bindings`. See param-binding §2. |
| `chain_tolerance.go` | `WithChainTolerance`, the document option that moves the chain-depth refusal. See faceted-vertex-bounds §5. |
| `gap_tolerance.go` | `WithGapTolerance`, `ExactOnly` and the context the root hands the evaluator. See gap-tolerance §1. |
| `mirror.go` | `MirrorPlane` and `Mirrored`/`MirroredCopy`. See mirror-pattern §4. |
| `mirror_join.go` | `WithJoin`, face selection and document commit around `internal/mirrorjoin/`'s record rewrite. See mirror-pattern §5. |
| `arrange_angle.go` | Public `WithArrangeAngle` and its footprint probes: copies turned by each angle step are measured and retired, then the best turn is applied. See arrange design §6. |
| `arrange.go` | Public `Arrange`, `ArrangeBed` (with its layers) and `WithArrangeTurns`: gates, footprints, then one `Transform` per body. `internal/arrangeplan/` packs the footprints. See arrange design. |
| `thread_table.go` | Public `LookupThread` and `ThreadSize`: the metric coarse and Unified pitch tables and the designation parser. See thread design §9. |
| `thread.go` | Public `Thread` and `ThreadSpec`: face and pitch gates, the axis-plane frame, then a private `Coil` and `Cut`. `internal/threadplan/` plans the groove. See thread design. |
| `path_pattern.go` | Public `PathPatternCopies`, `PathPatternSpaced`, `PathPatternJoin`, `PathPatternJoinSpaced`, `WithPathFollow` and `WithPathPivot`: gates, path sampling through `internal/pathsample/`, then one `Copy` per instance. See path-pattern design. |
| `pattern.go` | Public pattern specs, admission, Copy/Union fallback, `PatternJoin` and publication; `internal/patternassembly/` plans frame-keeping instances and disjoint groups. See mirror-pattern §4.3, §6. |
| `surface.go` | `WithSurfaceResult` and the sheet refusal. See surface §2–§4, §7, §11. |
| `feature_options.go` | Public feature option tiers and adapters to `internal/featureoption/`. See surface §4, sweep §2 and loft §2. |
| `patch.go` | `PatchProfile`: records the profile and builds its face through `patchchain.Build`. See surface §5.1. |
| `boss.go` | Public `Boss`, `BossSpec` and `BossBore`: face and size gates, the cylinder tool, then a union and the pilot cut through `combineToolSteps`. See boss design. |
| `boss_shape.go` | The non-round section of a `Boss`: reads a `HoleShape` and sketches it, grown for the draft. See boss design §7. |
| `boss_fillet.go` | Root fillet of a `Boss`: finds the boss's root circle in the result and calls `Fillet`, after a trial on a copy. See boss design §6. |
| `hole.go` | Public `Hole`, `HoleSpec`, depth, head and shape variants: gates, tool sketches, chained cut and the commit; `internal/holeplan/` computes the tools. See hole design. |
| `hole_faceted_site.go` | `facetedHoleSite`: the hole site of a convex planar face of an exact-vertex faceted solid, its normal read exactly and rounded once, its outward side from the centroid. See orthogonal Boolean B20. |
| `rib.go` | Public `Rib`, `RibArc`, `Web`, `WebPath` and `WithRibOverrun`: line, thickness and extent gates, rib tool extrusion, touch and volume checks. See rib design. |
| `text.go` | Public `TextProfiles` and `TextSpec`: gates, then `internal/glyphs/` lays the text out and the contours are drawn into a sketch and filtered to filled regions. See text design. |
| `emboss.go` | Public `Emboss`, `EmbossMode`, `WithEmbossDraft` and `WithEmbossRimFillet`: gates, sketch-plane check, tool extrusion, join and volume checks. See emboss design. |
| `emboss_rim.go` | Rim fillet of an `Emboss` relief: finds the rim edges in the result and calls `Fillet`, after a trial on a copy. See emboss design §7. |
| `planar_face.go` | Resolves a selector to one analytic planar face and its outward normal; `Hole` and `Emboss` share it. |
| `tool_combine.go` | Shared by composite features: chains Boolean steps, of one kind (`combineTools`) or several (`combineToolSteps`), over temporary tools, commits once, and retires tools without public history entries. |
| `thicken.go` | Public `Thicken` option, thickness and sheet gates and the commit; `internal/thickenbuild/` dispatches the payload and builds. See surface §16. |
| `offset.go` | `Offset`: option, distance and sheet gates and the commit; `internal/thickenbuild/` builds the second sheet at a normal distance. See surface §17. |
| `patch_body.go` | `PatchEdges` selection and commit; the chain gates, level proof and Rule P live in `internal/patchchain/`. See surface §5.2. |
| `stitch.go` | `Stitch`; builds through `internal/stitchrecord/`. See surface §6.4. |
| `unstitch.go` | `Unstitch` sheet split; builds through `internal/stitchrecord/`. See surface §6.5. |
| `extrude.go` | Generic `Extrude` over a profile or chain, `WithTaper`, linear-extent resolution, the chain ribbon entry point. See evaluator §5 and surface §13.2. |
| `extrude_tilted_face.go` | `extrudeToTiltedFace`: `Extrude` to a planar face of an exact-vertex faceted solid that is tilted from the sweep, as an extrusion past the face cut by a prism on the face's own outline. See orthogonal Boolean B19. |
| `draft.go` | `Draft`, `NeutralPlane`, `Walls` gates, plus the tapered extrude angle and surface-result gates, extent resolution and slab dispatch. See draft §5, §7, §10. |
| `draft_mesh_prism.go` | `draftMeshPrism`: Draft of every wall of an exact-vertex faceted straight prism, read on the grid and swept with a taper. See orthogonal Boolean B21. |
| `sheet_metal.go` | Public `SheetMetal`, `SheetMetalSpec` and `SheetMetalReport`: gates, the thickened centreline as a sketch outline, `Extrude`, and the flat pattern. See sheet-metal design. |
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
| `loft_between.go` | `LoftBetween` and its `LoftEnd`: dispatches a profile or point end to `Loft` or `LoftFromPoint`, adding no certificate. See loft §3.3. |
| `loft_point.go` | Admits a point-section Loft from Sketch and publishes the `internal/pointloft/` payload. See loft-point design. |

### Cap-loop chamfer

| Path | Responsibility |
|---|---|
