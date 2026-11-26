# Operation command API design

Every operation (`agent-docs/api-design.md` §6.3) is a command: a free function builds it from the mandatory
inputs, chained methods set the optional ones, and `Do(ctx)` runs it.

```go
cmd := decad.Fillet(body, decad.Edges(decad.Convex()), units.Millimeters(2))
cmd.TangentChain()            // optional input; returns cmd
result, err := cmd.Do(ctx)    // runs the operation once
```

This document owns the command form: the type shape, the run order of `Do`, argument copying and recording,
method rules, error tags, concurrency, the lint that catches a forgotten `Do`, the family verdicts and the
migration stack. `agent-docs/api-design.md` §6.3 owns which calls are operations. Each operation's companion
design owns its gate order, refusals and build; a command never changes them.

Navigation only, the sections own the rules:

| Question | Section |
|---|---|
| Is this call a command? | §1 |
| What are the type, constructor and method names? | §2 |
| In what order does `Do` check and run? | §3 |
| What may a constructor or method store or record? | §4 |
| How do repeated, exclusive or meaning-changing inputs work? | §5 |
| How do several commands share a method? | §6 |
| What does `Do` return, and how are errors tagged? | §7 |
| Concurrency, consumption, `Clone` | §8 |
| How is a forgotten `Do` caught? | §10 |
| How do root composites call operations? | §11 |
| Which family converts in which PR, and what does a PR carry? | §12–§14 |

## 1. Scope

- **Every operation is a command.** An operation builds, retires or registers a `*Body` (api-design §6.3). It is
  a command whether or not it has optional inputs, `Remove` included.
- **Queries keep the function or method form.** `Tessellate`, `Section`, `SectionOutline`, `LocatePoint`,
  `DistanceToPoint`, `ClosestPoint`, `MassProperties`, `ExtremeFacesByCentroid`, every `Verify*`, `ContactPair`,
  `ProvePairSeparated`, `SweepPair`, `SweptBox`, the mesh pipeline (`NewMeshBody`, `Read*`, `*Mesh`, `*Lattice`,
  `SubDMesh`), the `export` writers, `linkage`, `dynamics`, `parts` and `decadtest` stay as they are. Queries
  keep functional options (`option/v3`): they are re-runnable, commit nothing and share option tiers across
  calls (`TessellateOption` serves `Tessellate`, `ScaleBodyMesh` and the export writers).
- **Document options stay on `New`.** `WithGapTolerance`, `ExactOnly`, `WithChainTolerance`, `WithRimSplit` and
  `WithParams` configure the document, not a call.
- **Builders stay builders.** `Linkage`, `Link`, `parts.Set`, `dynamics.World` and `Timeline` are mutable values
  with their own error models (api-design §6.3). The command rules do not cover them.

## 2. Shape and naming

```go
// UnionCommand joins bodies. Build it with Union.
type UnionCommand struct {
	state    opcommand.State // consumed flag and first recorded fault (§4)
	bodies   []*Body         // copied at Union
	refine   bool            // set by RefineTo
	bound    float64         // the RefineTo bound in millimetres
	boundErr error           // the fault of the latest RefineTo bound, refused at Do (§4, A9)
}

func Union(bodies ...*Body) *UnionCommand
func (c *UnionCommand) RefineTo(maxBound units.Value) *UnionCommand
func (c *UnionCommand) Do(ctx context.Context) (*Body, error)
```

| Rule | Statement |
|---|---|
| N1 | The constructor keeps the operation's name: `Union`, `Fillet`, `Extrude`. |
| N2 | The command type is `<Op>Command`: `UnionCommand`, `FilletCommand`. A type cannot share the function's name in one package, `Operation` is the history record (`operation_history.go`), and `Op` is too short to read. |
| N3 | The constructor takes every mandatory input in today's order, without `ctx` and without options. Every operand is a constructor argument, never a method, so the bodies a call consumes stay visible in one place (api-design invariant #4). |
| N4 | A variadic operand slot is the constructor's last parameter: `Union(bodies ...*Body)`, `Stitch(bodies ...*Body)`, `LoftSections(d, sections ...LoftSection)`. |
| N5 | The constructor returns `*<Op>Command`, never an error. |
| N6 | Each optional input is one exported method named for the input, without a `With` prefix: `RefineTo`, `MatchingFaces`, `TangentChain`, `Taper`, `SurfaceResult`. |
| N7 | `Do(ctx context.Context)` is the only method that reads `ctx`, the document or any body. |
| N8 | A generic constructor keeps its type parameter and returns a non-generic command: `func Extrude[S Section](d *Document, s *sketch.Sketch, sec S, e Extent) *ExtrudeCommand`. The type argument changes no method and no result, so a generic command type would only make callers spell `ExtrudeCommand[*sketch.Profile]`. |
| N9 | Every command embeds `opcommand.State` as an unexported field and its `Do` calls `runCommand` (§3, §15). |

## 3. Do — the run order

`Do` runs these steps in order. `runCommand` (`operation_command.go`) owns steps 1–3 and the error tag; the
command's unexported implementation owns steps 4–6.

1. **Consume.** An atomic compare-and-swap marks the command consumed. A command already consumed returns
   `ErrConsumed` (§7) and runs nothing.
2. **Nil ctx.** A nil `ctx` is `ErrDegenerate`. This keeps the tested contract that a nil `ctx` is refused before
   any operand or option is read (`apitest/context_test.go`).
3. **Recorded fault.** The first fault a constructor or method recorded (§4) is returned.
4. **Gates.** The operation's documented gate order runs unchanged, including every place it reads `ctx.Err()`
   and every refusal table it implements (e.g. `agent-docs/modify-design.md` Table S). A stored method value is
   validated at the gate where today's option decode validated it.
5. **Build.**
6. **Commit.** The result commits atomically (§9).

- **`Do` consumes the command on every outcome.** Success, refusal, a recorded fault, a nil `ctx` and
  cancellation all consume it. A retry builds a new command.
- **Cancellation passes through unwrapped.** `Do` returns `ctx.Err()` itself, so `err == context.Canceled`
  holds (`apitest/context_test.go`). `operationError` already passes `context.Canceled` and
  `context.DeadlineExceeded` through untagged.
- **A wrapper that maps a ctx-taking call's error to a sentinel checks `ctx.Err()` first.** A cancel can land
  inside the call, so mapping its error to `ErrUnsupported` with `%s` would hide `context.Canceled`:
  `if cerr := ctx.Err(); cerr != nil { return nil, cerr }` comes before the mapping (`move_face.go`). A
  pre-canceled `ctx` stops at the first poll and never reaches such a wrapper, so a test of cancellation by
  identity cancels in the middle of the build (§14).
- **Every other error carries the command's `OperationKind`** (§7).

## 4. Arguments — copy, store, record

| Rule | Statement |
|---|---|
| A1 | **Constructors and methods only store copied values.** They never read document state, body liveness, a profile's staleness, `ctx`, or another argument. |
| A2 | A slice argument is copied with `slices.Clone` at the call. A variadic slot is copied too, so a caller's later write to its slice changes nothing. |
| A3 | A selector query is deep-copied at the call, branch by branch, as `ChamferCommand.Asymmetric` copies its reference (`modify_options.go`, `copyFaceQuery` in `selector.go`): `Or`, `Exactly` and `AtLeast` change a query in place (`selector.go`). A selector that is not a decad query is stored as given; the selector gate refuses it at `Do` without calling it. A sealed argument that holds a selector is copied at the call with that selector deep-copied: `NeutralFace.Face` inside the `NeutralPlane` of `Draft` (a `NeutralFace` value or a non-nil `*NeutralFace`), and the same for each later argument that holds one (`ToFace` and the angular stops inside an extent, `MirrorFace`, a spec struct). The pattern is one copy helper per sealed family, named `copy<Type>` and called by the constructor: `copyExtent` and `copyAngularExtent` (`extent.go`) and `copyAxis` (`revolve.go`) copy the selectors in an extent, an angular extent and an `EdgeAxis`. Stack PR 5 adds the helper for `MirrorFace`, and stack PR 6 one for each spec struct that holds a selector (`HoleSpec.Face` and the stops). |
| A4 | Bodies, sketches, profiles, chains, paths and meshes are stored as pointers. Liveness, foreign-document, stale-profile and selector-resolution checks run at `Do` against the live document (`agent-docs/modify-design.md` §4 order). A profile is never snapshotted at build time. |
| A5 | **A constructor or method records a fault only when both hold.** Its own argument shows the fault without reading anything else (A1), and the operation's documented gate order reports that fault ahead of every check that reads document state, liveness, `ctx.Err()` or another argument. Example: `Union` with fewer than two bodies is recorded, because `Union` refuses the count right after the nil-ctx check today. |
| A6 | Every other invalid value is stored and refused by the operation's own gate at `Do`, in today's position. Example: a negative `Fillet` radius is stored, and S15 refuses it after the S17 liveness gate. |
| A7 | Only the first recorded fault is kept. A constructor runs before any method, so its faults win. Where two methods could both record and today's order distinguishes their faults, the later one in today's order is stored under A6 instead, so call order never decides which fault the caller sees. |
| A8 | A recorded fault names its subject: the constructor parameter name (`bodies`, `target`, `tool`). A fault of a method value names the method (`RefineTo`) when the gate refuses it. |
| A9 | **Only constructors call `State.Record`.** `Record` keeps the first fault, so a method that recorded could not be replaced by a second call (M2). A method stores its value and the error validating it produced in its own field, and the gate refuses that error at the position A6 gives. `opcommand.Slot[T]` (`Set(value, err)`, `Get()`) is the shared holder: the last `Set` replaces the value and its error. `UnionCommand.RefineTo` keeps the same two fields inline. |

"Recorded" and "stored" both mean returned by `Do` before any build or commit. Neither ever makes a constructor
or method return an error.

## 5. Methods

| Rule | Statement |
|---|---|
| M1 | A method has a pointer receiver, changes the command and returns the same pointer, so `cmd.TangentChain()` written as a statement changes `cmd` rather than a dropped copy. |
| M2 | **A second call replaces the earlier value.** This holds for every method. A repeated `Asymmetric` replaces the earlier reference and distance, and today's `ErrDegenerate` for a repeated `WithPathCount` or `WithConeTangentEnd` has no command counterpart. `Shell.FaceThickness(normal, t)` names one world axis direction per call, so a repeat for that direction replaces its thickness and calls for other directions add to it. |
| M3 | **Exclusive alternatives are one method taking a sealed variant.** `Copy(src).Place(p CopyPlacement)` takes `CopyTransform`, `CopyRotation` or `CopyLiftTranslation`, so the type system allows one placement. |
| M4 | **A required choice is a sealed constructor argument**, as `Extent` and `PatternSpec` already are: `PathPattern(b, path, stations PathStations)` with `PathCount` or `PathSpacing`. |
| M5 | A dependent option takes its parent's slot: `PathPattern(...).Follow()` or `.FollowAbout(pivot)`. Both write the one follow slot, and the last call wins (M2). |
| M6 | **An option that changes the result kind or what a mandatory argument means is its own constructor.** `WithJoin` becomes `MirroredJoin(b, plane)`, `ShellClosed(b, t)` is the shell that keeps every face and takes no selector, and `WithConeTangentEnd` becomes `LoftFacesToCone(sections, cone, radial)`. |
| M7 | **A twin whose mandatory size means another quantity stays its own constructor.** `FilletChord(b, sel, chord)` sizes by chord, and `ChamferAngle(b, sel, d, reference, angle)` has its own gate order (`chamfer_angle.go`). |
| M8 | **A twin that exists only because the parent had no option slot folds into a method.** the refined union is `Union(bodies...).RefineTo(bound)`, and `LoftSectionsWithCircularCorrespondence(d, c, sections...)` becomes `LoftSections(d, sections...).CircularCorrespondence(c)`. The fold keeps the twin's gates, result identities and history exactly. |
| M9 | A conflict between two different methods is refused at `Do` with today's sentinel: `Taper` with `SurfaceResult` stays `ErrUnsupported` (`extrude.go`). |
| M10 | A method called after `Do` changes the command and nothing else, because `Do` refuses a consumed command. |
| M11 | A method exists on a command only when today's call accepts the option without an unconditional refusal. `CutExtrudeCommand` has no `SurfaceResult`. |

`RefineTo` builds the union directly at the bound for its class: a revolved blank and two or more copies of one
point-Loft tooth. Any other set is `ErrUnsupported`. It never falls back to `Union` then `Refine`, because the history would then depend
on the input. Every error carries `OperationUnion`, and a bound fault names `RefineTo` as its subject.

## 6. Shared method groups and generics

Several operations share an optional input: `SurfaceResult` serves Extrude, Revolve, Sweep and Loft, `TangentChain`
serves Fillet and Chamfer, and `MatchingFaces` serves Cut and Intersect.

- **Each command declares its own one-line method, and the method calls one shared unexported setter.**

  ```go
  type surfaceResultSetting struct{ on bool }

  func (s *surfaceResultSetting) set() { s.on = true }

  // SurfaceResult returns the wall set as a sheet instead of a solid.
  func (c *ExtrudeCommand) SurfaceResult() *ExtrudeCommand { c.surface.set(); return c }
  ```

- **NEVER share a method by embedding.** A method promoted from an embedded struct returns the embedded type, so
  `cmd.SurfaceResult().Taper(a)` would stop compiling. A generic mixin that returns the outer type needs a
  back-pointer from the mixin to its command, and its promoted method has one doc comment for every command.
- The per-type method keeps each command's method set explicit in source and in the generated reference, and lets
  its doc comment state that command's meaning.
- **A Go method set cannot vary by type argument.** `Extrude` over `*sketch.Chain` still offers `Taper()`. The
  chain refusal stays at `Do`, in today's gate position (`extrude.go`). N8 keeps the command type non-generic.

## 7. Results and errors

- **`Do` returns exactly what the function returns today.** `UnionCommand.Do` returns `(*Body, error)`,
  `SplitCommand.Do` returns `([]*Body, error)`, `SheetMetalCommand.Do` returns
  `(*Body, SheetMetalReport, error)` and `RemoveCommand.Do` returns `error`. `runCommand` is generic over one
  result value, and a command with several results passes a struct and unpacks it.
- **There is no shared `Command` interface.** Nothing needs one, and one would force a wrapper type per result.
- **Every error from `Do` except cancellation is tagged with the command's own `OperationKind`.** This covers the
  nil-ctx error. `PathPattern`
  tags `OperationCopy` and `Arrange` tags `OperationTransform` today, and their PRs keep those kinds, since
  `OperationKind` gains no constant in this design.
- **A recorded fault names its subject (A8) in `OperationSubject.Name`.** A gate that refuses a stored method
  value names the method by returning `subjectError(kind, method, err)`; the tag `runCommand` adds keeps it.
- **Subject names follow the constructor's parameter names.** `Union(bodies ...*Body)` reports operand `i` as
  `bodies` with ordinal `i`.
- **History roles keep their recorded values.** `OperationInput.Role` is recorded data read by the regroup and
  point-query code. A Union input keeps the roles `first` and `others` whatever the constructor parameter is
  called, and M8 folds keep history exactly.
- **`ErrConsumed`** is a §12 sentinel (api-design §12, `errors.go`, `internal/decaderr/`). `Do` returns it on a
  consumed command, tagged with the command's kind, `OpDiagInputRejected` and `OperationStageInput`, with no
  subject.
- Error text that names a `With...` option names the method instead.
- The nil-option, zero-option and foreign-option refusals of today's option tiers have no command counterpart.
  Their tests are deleted with the option.

## 8. Concurrency and reuse

- **A command is not safe for concurrent use.** Building, calling methods and `Do` belong to one goroutine.
- **Racing `Do` calls run the operation at most once.** Consumption is an atomic compare-and-swap; every loser
  returns `ErrConsumed`.
- Two commands on one document MUST NOT run `Do` concurrently: `Document` is not safe for concurrent mutation
  (api-design §12). `Remove(b).Do(ctx)` may run concurrently with `Bodies` and `Verify`, as `Remove` may today.
- **There is no `Clone`.** Rebuilding is one expression, and a clone would have to deep-copy selectors again.

## 9. Atomicity

A failed or canceled `Do` leaves `Document.Bodies()`, the operation history and producer numbering as it found
them (api-design §2). `PathPattern` and `Arrange` do not meet this today: `PathPattern` commits each copy through
`Copy` and only retires them on failure (`path_pattern.go`), and a canceled `Arrange` keeps bodies already moved
(`arrange.go`). Stack PR 4 (§13) makes both atomic before stack PR 5 converts them.

## 10. A forgotten `Do`

- **Every constructor is listed in govet's `unusedresult.funcs`** in `.golangci.yml`, under
  `linters.settings.govet.settings.unusedresult.funcs`. Then `decad.Union(a, b)` written as a statement fails lint.
- The setting replaces the analyzer's default list, so the list MUST also carry the standard-library defaults of
  the golangci-lint version CI pins (`golang.org/x/tools` `passes/unusedresult`).
- The booleans PR adds the setting and a root test that lists every exported function returning `*<X>Command`
  and fails when one is missing from the setting. Each later family PR adds its constructors.
- The lint does not catch `cmd := decad.Fillet(...); cmd.TangentChain()` with no `Do`. M1 keeps that a visible
  unused command in review rather than a silent copy.
- The lint does not catch the chained form `decad.Cut(a, b).MatchingFaces()` written as a statement, because the
  statement's call is the method. A repo-wide syntactic Go test fails on any expression statement whose call chain
  starts at a command constructor and does not end in `Do`, and on a constructor call used as a statement. It
  scans `apitest/`, `examples/`, the root tests and `_gallery/`.
- The setting and both checks protect this repository only. A downstream caller's code gets no lint from them.
- The `unusedresult.funcs` comment in `.golangci.yml` names the x/tools version the standard-library entries were
  copied from (v0.52.0, inside golangci-lint v2.15.0). A golangci-lint bump re-copies that list. A stale list never
  fails lint; it only loses the newer standard-library checks.

## 11. Callers inside the root package

- **`Do` is a thin wrapper over an unexported implementation with the same inputs.**

  ```go
  func (c *CutCommand) Do(ctx context.Context) (*Body, error) {
  	return runCommand(ctx, &c.state, OperationCut, c.run)
  }

  func (c *CutCommand) run(ctx context.Context) (*Body, error) {
  	return cut(ctx, c.target, c.tool, c.cfg)
  }
  ```

- **A root composite calls the unexported implementation, never a public command.** `hole.go`, `infill.go`,
  `pattern.go`, `path_pattern.go`, `arrange.go`, `fillet_chord.go` and `chamfer_angle.go` call `cut`, `union`,
  `copyBody` and the like. A composite allocates and consumes no command.
- **A composite may tag the implementation's error with that operation's kind itself, but the outermost command's
  kind wins.** `operationError` re-tags every nested non-selection diagnostic with the kind it is given, and the
  public `Do` applies it once with its own kind. A composite's own tag therefore decides a diagnostic's kind only for
  a caller inside the root that applies no outer tag. Selection diagnostics (`OperationSelectEdges`,
  `OperationSelectFaces`) keep their own kinds. This is the accepted behaviour for composites: `Emboss` reports its
  nested `Union`, `Cut` and rim-fillet refusals as `OperationEmboss`, and `Infill` reports its nested `Union`,
  `Intersect` and `Shell` refusals as `OperationInfill` once stack PR 6 converts it (§12).
- **The rewrite tool never rewrites a root production file.** It reports a bare operation call in a non-test root
  file, and the author moves it onto the implementation by hand.
- The implementation returns untagged errors. Only the public `Do` applies `operationError`, once.
  `SelectionError` and `BooleanError` keep their own kinds, as `operationError` does today.
- **The one exception is the refusal of a stored method value.** The implementation refuses it with
  `subjectError(kind, subject, err)` (§7), so that error is tagged and names its subject. The subject travels in
  the implementation's settings record, next to the value and its error (`chamferSettings.asymSubject`). The
  command's `run` sets the method name. A composite that calls the implementation with a value it computed sets the
  name of the constructor argument that value comes from, or leaves the subject empty, so a diagnostic never names
  a method the composite's own command does not have (`ChamferAngle` names `d`, not `Asymmetric`).
- **A stored method value's fault is named in one of two ways.** Settings that the implementation takes carry the
  subject next to the value and its error (`extrudeSettings.taperSubject`), or the folding code returns a typed
  error that carries the method name and `Do` turns it into the subject (`featureoption.SweepFaultError`, which
  `foldSweepInput` converts with `subjectError`). Either way the diagnostic names the method the caller used.
- The implementation takes a non-nil `ctx`; `Do` refused a nil one.

## 12. Family verdicts

Every operation below becomes a command. The first list names the constructors; the bullets under each family
name its methods and special rules.

| Family | Constructors |
|---|---|
| Booleans | `Union(bodies...)`, `Cut(target, tool)`, `Intersect(a, b)`, `Refine(source, maxBound)` |
| Modify ops | `Fillet`, `Chamfer`, `FilletChord`, `ChamferAngle`, `Shell`, `ShellClosed`, `Draft`, `Offset`, `Thicken`, `MoveFace`, `ConeFromFace` |
| Sketch-fed features | `Extrude`, `Revolve`, `Sweep`, `Coil`, `Loft`, `LoftChain`, `LoftFromPoint`, `LoftSections`, `LoftFaces`, `LoftFacesToCone`, `PatchProfile`, `Emboss`, `WebPath`, `CutExtrude`, `CutRevolve` |
| Placement and patterns | `Transform`, `Copy`, `Mirrored`, `MirroredJoin`, `MirroredCopy`, `ScaledAt`, `QuarterTurn`, `Stretch`, `PatternCopies`, `Patterned`, `PathPattern`, `Arrange` |
| Spec composites | `Hole`, `Boss`, `Thread`, `Infill`, `SheetMetal` |
| Surface, mesh-to-solid and retirement | `PatchEdges`, `Stitch(bodies...)`, `Unstitch`, `Trim`, `Extend`, `Split`, `SolidFromMesh`, `Remove` |

- Booleans: `Union.RefineTo` is the refined union (M8). Cut and Intersect each declare `MatchingFaces` (§6).
- Modify ops: Fillet and Chamfer each declare `TangentChain` (§6). Chamfer has `Asymmetric(reference, other)`.
  Shell and `ShellClosed` each have `Sense` and `FaceThickness`. Offset and Thicken have `Side`. `ShellClosed(b, t)`
  is the closed shell (M6). `FilletChord`, `ChamferAngle`, `Draft`, `MoveFace` and `ConeFromFace` have no method.
- Sketch-fed features: Extrude, Revolve, Sweep and Loft each declare `SurfaceResult` (§6, M11). The other methods
  are `Extrude.Taper`, `CutExtrude.Taper`, `Loft.Alignment`, `Loft.Splits`, `Sweep.MitredJoins`,
  `Sweep.SectionScale`, `Sweep.Twist`, `Coil.LeftHand`, `Emboss.RimFillet`, `Emboss.Draft`, `WebPath.Draft` and
  `WebPath.Overrun`. `LoftChain`, `LoftFromPoint`, `LoftFaces`, `LoftFacesToCone`, `PatchProfile` and `CutRevolve`
  have no method. `LoftSections.CircularCorrespondence` folds its twin (M8). `LoftFacesToCone(sections, cone,
  radial)` replaces the old cone-tangent-end option (M6). `Extrude`, `Revolve` and `Sweep` keep generic
  constructors (N8, §6). A section constructor records a nil document and a nil sketch, section or path (A5),
  and each `Do` of a profile-or-chain section refuses a chain's `SurfaceResult` and `Taper` at the seam gate's
  position. `Sweep.Twist` and `Sweep.SectionScale` store their validation errors, and the count of factors
  against the path is refused at the same gate as the factors themselves, naming `SectionScale`. A composite that
  sweeps a private tool calls `extrudeTool`, which tags the error with `OperationExtrude`; the composite's public
  `Do` then re-tags it with its own kind (§11).
- Placement and patterns: `Copy.Place` takes a sealed placement (M3). `MirroredJoin` replaces `WithJoin` (M6).
  `PathPattern` takes sealed `PathStations` (M4) and has `Follow` and `FollowAbout` (M5). Arrange has `Angle` and
  `Turns`.
- Spec composites: `Infill` has no outer tag today, so the stack PR that converts it re-tags its nested `Union`,
  `Intersect` and `Shell` diagnostics as `OperationInfill` (§11). Each spec struct splits. Required fields become constructor arguments, and each
  zero-means-default field becomes a method whose absence means the default: `SheetMetalSpec.KFactor` zero
  meaning 0.44 becomes a `KFactor` method. Sum-type specs stay arguments. The spec-composite PR names the split
  of each struct in §12.
- Surface, mesh-to-solid and retirement: no methods. `Remove(b).Do(ctx)` returns `error`.
- `PatternSpec` and every `Extent`, `AngularExtent`, `Axis` and `MirrorPlane` stay sealed constructor arguments.

## 13. Migration stack

The stack lands on `main` together. Each PR converts one family with a hard switch: it deletes the old function,
its option tier and its `With...` constructors in the same PR, and no wrapper survives it.

| PR | Content |
|---|---|
| 0 (base) | This document, `internal/opcommand/`, `operation_command.go`, `ErrConsumed`, and the command-form rules in api-design §3, §4, §6.3, §8 and §12 and in `docs/api.md`. It converts no family. |
| 1 | Booleans. It also adds the `unusedresult` setting and its root test (§10) and the rewrite tool (§14). |
| 2 | Modify ops. It proves §3 against Table S and the two-command `TangentChain` method. |
| 3 | Sketch-fed features. It proves N8 and the four-command `SurfaceResult` method. |
| 4 | Atomic `PathPattern` and `Arrange` (§9). It changes behaviour only on failure and cancellation and converts nothing. |
| 5 | Placement and patterns. |
| 6 | Spec composites. They follow PRs 2–5 because they call those operations' implementations (§11). |
| 7 | Surface, mesh-to-solid ops and `Remove`. It deletes the rewrite tool. |

## 14. What each family PR carries

- The conversion of every member of the family, each `Do` over an unexported implementation (§11), and each root
  composite moved onto those implementations.
- Its rows in §12 updated to the shipped names, and its signature blocks and prose in api-design §8, §9 and the
  companion design docs rewritten to the command form. `docs/reference/` is regenerated by the Docs workflow,
  never by a PR.
- `docs/api.md`: every entry of the family, named by constructor and method. `docs/data-structures.md`: every new
  command type. `apitest/api_reference_test.go` checks both.
- `examples/`: every example that calls the family, with its `// Output:` block verified by `go test ./examples/...`.
- `apitest/`, the root tests and `_gallery/` migrated. `_gallery/` builds against the working tree
  (`_gallery/go.mod` `replace`), so it breaks in the same PR otherwise.
- Tests asserting the §3 order for each member: a second `Do` is `ErrConsumed`, a nil `ctx` precedes every operand
  fault, cancellation returns `ctx.Err()` by identity, and the operation's own refusal table holds unchanged.
- A cancellation test that cancels in the middle of the build, not only a pre-canceled `ctx` (§3). The test
  passes a context whose `Err` turns to `context.Canceled` after n checks and moves n one later per run until
  `Do` succeeds. Every run before that returns `context.Canceled` by identity
  (`TestModifyCommandsReturnMidBuildCancellationByIdentity`).
- **The cancellation sweep has a cost rule.** Every family adds one sweep to race shard 12, and a sweep of a build
  that polls `ctx` N times costs about N²/2 checks. A member whose build polls more than a few hundred times runs
  its sweep over the smallest fixture on the same build route (a coil of one turn, a narrower groove) and moves n
  by a stride of about N/10 instead of one, so the sweep makes about ten runs
  (`TestFeatureCommandsReturnMidBuildCancellationByIdentity`). It builds that fixture once and makes a fresh
  command per run, because every canceled run leaves the document unchanged. A member that polls a few hundred
  times or fewer keeps the one-check-later rule.
- **Each family PR updates the "today" sentences it makes false.** A sentence that says what an operation does
  "today" or "leaves untagged today" is rewritten in the PR that changes it, in this document and the companion
  designs.
- A per-member test of where each stored method fault is refused: after the retired-body gate and the sheet
  refusal, before the magnitude gate, and after a nil `ctx`. Every member gets a retired-body row, and each
  member with a validating method gets rows for the method fault against the gates around it
  (`TestModifyCommandsRefuseMethodFaultsAtTheirGate`).
- The family's constructors in `unusedresult.funcs` (§10).
- `.github/test-shards.txt` and `.github/test-shards-apitest.txt` updated for renamed or removed tests
  (`agent-docs/testing.md`).
- The rewrite tool's rules for the family, run over `apitest/`, `examples/`, the root tests and `_gallery/`. The tool
  is `_cmdrewrite/`, its own nested module recorded in `agent-docs/modules.md`, because `gofmt -r` cannot match a
  variadic option list. The rules match the pre-migration names, such as `WithTangentChain` and
  `WithNoOpenings`, and the tool's tests feed old-API code and assert the rewritten output, because the tool must
  migrate calls that land on `main` while the stack is open. A new-name call is not input.
- The Module boundary gate evidence (`agent-docs/module-boundary.md`) in the PR body.

## 15. Shared infrastructure

| Piece | Owner | Behaviour |
|---|---|---|
| `opcommand.State` | `internal/opcommand/state.go` | Zero value is a fresh command. `Record(subject, err)` keeps the first non-nil fault. `Fault()` returns it. `Consume()` reports true to exactly one caller, racing callers included. |
| `runCommand[T]` | `operation_command.go` | Steps 1–3 of §3, then the implementation, then the `operationError` tag. A root adapter: it applies the root-owned `OperationKind`, `OperationError` and `OperationSubject`. |
| `subjectError` | `operation_command.go` | Tags an error with the kind and names the subject on every diagnostic that names none. |
| `opcommand.Slot[T]` | `internal/opcommand/slot.go` | Holds one method's value and the error validating it. `Set(value, err)` replaces both. `Get()` returns them. `IsSet()` reports a call. |
| `ErrConsumed` | `internal/decaderr/errors.go`, `errors.go` | Mapped to `OpDiagInputRejected` at `OperationStageInput` by `operationErrorCode`. |

The tests are `internal/opcommand/state_test.go`, `internal/opcommand/slot_test.go` and `operation_command_test.go`. They cover the first fault
winning, one winner among racing `Consume` calls, `ErrConsumed` on a second `Do` with its tag, consumption on
refusal, consumption before nil ctx, nil ctx before a recorded fault, a recorded fault returned before the
operation runs with its subject, cancellation by identity and the kind tag on operation errors.
