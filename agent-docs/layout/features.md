# decad file layout: features

Feature files and the cap-loop chamfer.
The rules for rows live in `agent-docs/layout.md`.

## Layout

### Features

| Path | Responsibility |
|---|---|
| `topology.go` | Topology types from `Body` to `Vertex` and public surface and curve variants. See API §6.1 and evaluator §3. |
| `topology_handles.go` | Born-handle constructors, node-to-handle mapping, `handleAlloc` (with `ReplayNodeSize`) and `envOf` for internal builders, `bodyResult`, and `placePayload`. See evaluator §3, §8. |
| `container.go` | `Container`, the document's grouping of sketches, planes and bodies; placement of results, `ErrCrossContainer`, `ChildNamed`, `NewUniqueChild` and `VerifyContainers`. See `agent-docs/container-design.md`. |
| `document.go` | `Document`: live body set, atomic operation publication, `RemoveCommand`, liveness gates, `Transform` and `Copy` with `CopyPlacement` and `copyBody`; the producer identity spelling and zero-vector predicate. See evaluator §8 and operation history. |
| `circle_affine_replay.go` | Exact affine differences for replay of placed circle sources. |
| `circle_placement_replay.go` | Ordered placement witness and rounding checks for circle sources. |
| `circle_primitive_seal.go` | Private circle source interface and candidate replay with placement bounds. |
| `circle_source_replay.go` | Circle Extrude and Transform history capture under one replay budget. |
| `operation_history.go` | Immutable public operation snapshots, body dependency indexes, and face-successor query values. See operation-history-design. |
| `operation_kind.go` | Shared public operation-kind vocabulary and stable string tokens. See operation-history-design and operation-diagnostics-design. |
| `operation_diagnostics.go` | Public operation diagnostic values, typed error accessors and body warning snapshots. See operation-diagnostics-design. |
| `operation_command.go` | `runCommand`, the shared `Do` of every operation command, `subjectError`, `subjectFault` and `errNilBody`. See command-api-design §3, §14. |
| `params.go` | `WithParams`, `Document.Params`, `Param`, `Expr`, `ParamInt`, `Bindings`. See param-binding §2. |
| `chain_tolerance.go` | `WithChainTolerance`, the document option that moves the chain-depth refusal. See faceted-vertex-bounds §5. |
| `rim_split.go` | `WithRimSplit`, the document option that keeps a rim's bound beside the rim. See faceted-vertex-bounds §3.7. |
| `gap_tolerance.go` | `WithGapTolerance`, `ExactOnly` and the context the root hands the evaluator. See gap-tolerance §1. |
| `mirror.go` | `MirrorPlane`, `copyMirrorPlane` and the `Mirrored`/`MirroredCopy` commands. See mirror-pattern §4. |
| `mirror_join.go` | The `MirroredJoin` command, face selection and document commit around `internal/mirrorjoin/`'s record rewrite. See mirror-pattern §5. |
| `arrange_angle.go` | `ArrangeCommand.Angle` and its footprint probes: copies turned by each angle step are measured and retired, then the best turn is applied. See arrange design §6. |
| `arrange.go` | The `Arrange` command, `ArrangeBed` (with its layers) and `Turns`: gates, footprints, then one `Transform` per body. `internal/arrangeplan/` packs the footprints. See arrange design. |
| `thread_table.go` | Public `LookupThread` and `ThreadSize`: the metric coarse and Unified pitch tables and the designation parser. See thread design §9. |
| `thread.go` | `Thread`, `ThreadCommand` with its methods, `ThreadForm` and its constructors: face and pitch gates, the axis-plane frame, then a private `Coil` and `Cut`. `internal/threadplan/` plans the groove and its sampling density. See thread design. |
| `path_pattern.go` | The `PathPattern` command, its `PathCount` and `PathSpacing` stations and `Follow` and `FollowAbout`: gates, path sampling through `internal/pathsample/`, then one `Copy` per instance. See path-pattern design. |
| `pattern.go` | The `PatternCopies` and `Patterned` commands, pattern specs, admission, Copy/Union fallback and publication; `internal/patternassembly/` plans frame-keeping instances and disjoint groups. See mirror-pattern §4.3, §6. |
| `surface.go` | The sheet refusal of Table X. See surface §2–§4, §7, §11. |
| `feature_options.go` | Shared settings of the sketch-fed commands: `SurfaceResult`, the extrude taper, the constructor nil faults and the section operands. See command-api §4, §6 and surface §4. |
| `patch.go` | `PatchProfile` and `PatchProfileCommand`: records the profile and builds its face through `patchchain.Build`. See surface §5.1. |
| `boss.go` | `Boss`, `BossShaped`, their commands, `BossBore` and `BossRibs`: face and size gates, the cylinder tool, then a union and the pilot cut through `combineToolSteps`. See boss design. |
| `cut_profile.go` | `CutExtrude`, `CutRevolve` and their commands: one private `Extrude` or `Revolve` tool per profile, then one cut through `combineTools`. See cut-profile design. |
| `boss_shape.go` | The non-round section of a `Boss`: reads a `HoleShape` and sketches it, grown for the draft. See boss design §7. |
| `boss_fillet.go` | Root fillet of a `Boss`: finds the boss's root circle in the result and calls `Fillet`, after a trial on a copy. See boss design §6. |
| `hole.go` | `Hole`, `HoleShaped`, their commands, `HoleThread`, depth, head and shape variants: gates, tool sketches, chained cut and the commit; `internal/holeplan/` computes the tools. See hole design. |
| `hole_faceted_site.go` | `facetedHoleSite`: the hole site of a convex planar face of an exact-vertex faceted solid, its normal read exactly and rounded once, its outward side from the centroid. See orthogonal Boolean B20. |
| `rib.go` | `WebPath` and `WebPathCommand` with `Draft` and `Overrun`: line and arc outlines, thickness and extent gates, rib tool extrusion, touch and volume checks. See rib design. |
| `text.go` | Public `TextProfiles` and `TextSpec`: gates, then `internal/glyphs/` lays the text out and the contours are drawn into a sketch and filtered to filled regions. See text design. |
| `emboss.go` | `Emboss`, `EmbossCommand` with `Draft` and `RimFillet`, and `EmbossMode`: gates, sketch-plane check, tool extrusion, join and volume checks. See emboss design. |
| `emboss_rim.go` | Rim fillet of an `Emboss` relief: finds the rim edges in the result and calls `Fillet`, after a trial on a copy. See emboss design §7. |
| `planar_face.go` | Resolves a selector to one analytic planar face and its outward normal; `Hole` and `Emboss` share it. |
| `tool_combine.go` | Shared by composite features: chains Boolean steps, of one kind (`combineTools`) or several (`combineToolSteps`), over temporary tools, commits once, and retires tools without public history entries. |
| `thicken.go` | Public `Thicken`, `ThickenCommand` with its `Side` method, thickness and sheet gates and the commit; `internal/thickenbuild/` dispatches the payload and builds. See surface §16. |
| `offset.go` | `Offset` and `OffsetCommand` with its `Side` method: distance and sheet gates and the commit; `internal/thickenbuild/` builds the second sheet at a normal distance. See surface §17. |
| `patch_body.go` | `PatchEdgesCommand` over `patchEdges`: selection and commit; the chain gates, level proof and Rule P live in `internal/patchchain/`. See surface §5.2. |
| `stitch.go` | `StitchCommand` over `stitch`; builds through `internal/stitchrecord/`. See surface §6.4. |
| `unstitch.go` | `UnstitchCommand` over `unstitch`, the sheet split; builds through `internal/stitchrecord/`. See surface §6.5. |
| `extrude.go` | `Extrude`, `ExtrudeCommand` with `Taper` and `SurfaceResult`, linear-extent resolution and the chain ribbon entry point. See evaluator §5 and surface §13.2. |
| `extrude_tilted_face.go` | `extrudeToTiltedFace`: `Extrude` to a planar face of an exact-vertex faceted solid that is tilted from the sweep, as an extrusion past the face cut by a prism on the face's own outline. See orthogonal Boolean B19. |
| `draft.go` | `Draft`, `DraftCommand`, `NeutralPlane`, `Walls` gates, plus the tapered extrude angle and surface-result gates, extent resolution and slab dispatch. See draft §5, §7, §10. |
| `draft_mesh_prism.go` | `draftMeshPrism`: Draft of every wall of an exact-vertex faceted straight prism, read on the grid and swept with a taper. See orthogonal Boolean B21. |
| `sheet_metal.go` | `SheetMetal`, `SheetMetalCommand` with `KFactor`, and `SheetMetalReport`: gates, the thickened centreline as a sketch outline, `Extrude`, and the flat pattern. See sheet-metal design. |
| `sweep.go` | `Sweep` and `SweepCommand`: nil and seam gates, then `internal/sweepbuild/` builds and root commits. Also the mitred join and section scale methods, built through `sweepmitre.Build`. See sweep §3.3, §15.8–§15.9, §16. |
| `coil.go` | `Coil`, `CoilCommand` with `LeftHand`, and the Table CS gates. Root `coilPayload` aliases `coilshell.Payload`. See helix §2, §4, §5.6. |
| `revolve.go` | `Revolve`, `RevolveCommand` with `SurfaceResult`, the `Axis` variants, angular extents and the public axis adapter to `internal/revolveaxis/`. See evaluator §6 and surface §13.2. |
| `stops.go` | Body-relative stop resolution: live stop bodies, face selection and angular stops; `internal/extent` admits a `ToFace` or `ToFaceAngular` face and computes its stop, and resolves through-all stops. See evaluator §5/§6/§11. |
| `internal/extent/stop_revolve.go` | Exact `ToFace` level for a planar Revolve wall or angular cap. See evaluator §5. |
| `loft.go` | `Loft`, `LoftChain` and their commands, with `Alignment`, `Splits` and `SurfaceResult`. Builds through `loftrecord.FromProfiles` and `FromChains`. See loft §2/§4/§10/§16. |
| `loft_sections.go` | `LoftSections` and `LoftSectionsCommand` with `CircularCorrespondence`, and the source-face assembly. See loft-sections design. |
| `cone_from_face.go` | `ConeFromFace` and `ConeFromFaceCommand`: the full tool cone one float step off a Revolve blank's selected conical face. See loft-face-sections design. |
| `loft_faces.go` | `LoftFaces`, `LoftFacesToCone` and `LoftFacesCommand`; the commit of the loft `internal/revolveloftfaces` assembles and proves; shared rational helpers. See loft-face-sections design. |
| `loft_point.go` | `LoftFromPoint` and its command: admits a point-section Loft from Sketch and publishes the `internal/pointloft/` payload. See loft-point design. |

### Cap-loop chamfer

| Path | Responsibility |
|---|---|
