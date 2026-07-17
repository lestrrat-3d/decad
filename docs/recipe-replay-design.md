# Recipe Replay Design

How decad decodes, validates, and evaluates a stored `Recipe` into a new
`Document`. Companion to `docs/api-design.md` (public contract),
`docs/sketch-seam-design.md` (exact profile records), and
`docs/evaluator-design.md` (v1 geometry). References of the form "core §N" are
to `docs/api-design.md`.

This contract has four goals:

- stored recipes rebuild without a live sketch;
- malformed input never becomes a partly built model;
- replay uses the same operation logic as immediate feature calls;
- evaluator choice never enters the exact record of intent.

## 1. Public surface

```go
type RecipeLimits struct {
    MaxBytes            int64
    MaxDepth            int
    MaxSteps            int
    MaxLoops            int
    MaxSegments         int
    MaxCurvePoints      int
    MaxCurveScalars     int
    MaxSelectors        int
    MaxPredicates       int
    MaxRoleBytes        int
    MaxTotalStringBytes int
}

func DefaultRecipeLimits() RecipeLimits

type EvaluationLimits struct {
    MaxFacets           int
    MaxBooleanPairTests int64
    MaxExactFallbacks   int64
}

func DefaultEvaluationLimits() EvaluationLimits

type DecodeRecipeOption interface {
    option.Interface
    decodeRecipeOption()
}

type ValidateRecipeOption interface {
    option.Interface
    validateRecipeOption()
}

type EvaluateOption interface {
    option.Interface
    evaluateOption()
}

// RecipeOption satisfies DecodeRecipeOption, ValidateRecipeOption, and
// EvaluateOption.
type RecipeOption interface {
    DecodeRecipeOption
    ValidateRecipeOption
    EvaluateOption
}

func WithRecipeLimits(RecipeLimits) RecipeOption
func WithEvaluationLimits(EvaluationLimits) EvaluateOption

func DecodeRecipe(r io.Reader, opts ...DecodeRecipeOption) (Recipe, error)
func (r Recipe) Validate(opts ...ValidateRecipeOption) error

// Evaluator is an opaque package-owned evaluator handle.
type Evaluator struct { /* unexported implementation */ }

func V1Evaluator() Evaluator
func (e Evaluator) Name() string
func WithEvaluator(Evaluator) EvaluateOption

func Evaluate(ctx context.Context, r Recipe, opts ...EvaluateOption) (*Document, error)
```

Rules:

- `DecodeRecipe` decodes one JSON recipe and validates it.
- Nil reader → `ErrDegenerate`.
- `Recipe.Validate` is pure. It never changes the recipe or builds geometry.
- `Evaluate` always creates a new `Document`. It never appends into an existing
  document.
- No option selects a destination document.
- No `Load` synonym ships. Call `DecodeRecipe`, inspect if needed, then call
  `Evaluate`.
- No `Replay` synonym ships. `Evaluate` is the one operation that runs exact
  intent through an evaluator.
- No options → default recipe limits + default evaluation limits + v1 evaluator.
- Any nil option passed to decode, validate, or evaluate → `ErrDegenerate`.
- A zero or partly zero limit struct is `ErrDegenerate`. Call the default
  constructor, then change named fields.
- A zero `Evaluator` passed to `WithEvaluator` is `ErrDegenerate` at
  `Evaluate`.
- `Evaluator` is a concrete opaque value, not a user-implemented interface.
  `Document` topology has package-owned construction invariants; an external
  implementation cannot safely build it. Future bundled evaluators return
  other `Evaluator` values.
- Code generators consume `Recipe` directly. They are not evaluators because
  they do not produce a decad `Document`.

`Recipe` contains slices and pointer-form closed variants. `Evaluate` takes a
deep, normalized snapshot before validation. Caller mutation after snapshot
cannot affect the build. Concurrent mutation while `Validate` or `Evaluate`
copies the same recipe is unsupported and is a data race, on the same terms as
concurrent mutation of `Document`.

## 2. Wire envelope and compatibility

Canonical JSON uses this envelope:

```json
{
  "format": "decad.recipe",
  "version": 1,
  "steps": []
}
```

The in-memory `Recipe` stays:

```go
type Recipe struct {
    Steps []Step
}
```

Format metadata belongs to the envelope, not model intent. A decoded recipe is
normalized into the current in-memory shape; callers never branch on a stored
format version.

### 2.1 Version rules

- `json.Marshal(Recipe)` ALWAYS writes `format`, `version`, then `steps`.
- Canonical `steps` is always an array. Nil + empty in-memory slices both encode
  as `[]`.
- `Recipe.MarshalJSON` validates structure without applying resource ceilings.
  A malformed caller-built recipe refuses to encode.
- `json.Unmarshal` into `Recipe` uses the strict decoder and default limits.
- `DecodeRecipe` is the untrusted-reader API and allows explicit limits.
- Existing unversioned `{"steps": ...}` input is legacy version 1 and remains
  accepted. It MUST contain only `steps`.
- Legacy `{"steps":null}` is accepted as empty because the original default Go
  codec emitted it for `Recipe{}`. Versioned input requires an array.
- If either `format` or `version` is present, both are required.
- `format` MUST equal `decad.recipe`.
- `version` MUST equal `1`.
- Unknown versions return `ErrUnsupportedRecipeVersion`. NEVER ignore fields
  from a newer version.
- Every change that adds, removes, or changes the meaning of a wire field bumps
  `version`, even when old decoders could ignore the field.
- Adding an evaluator does NOT bump recipe version.
- Evaluator identity, tessellation tolerance, error bounds, and cached topology
  NEVER enter the envelope.

Legacy input re-encodes as the canonical versioned envelope. No separate
migration API exists for version 1.

### 2.2 Strict JSON

The strict decoder MUST reject:

- an unknown field at any object level;
- a duplicate key at any object level;
- a missing required field;
- an unknown or missing closed-set `kind` tag;
- more than one top-level JSON value;
- non-whitespace after the top-level value;
- nesting past `MaxDepth`;
- input past `MaxBytes`.

Implementation: read through a `MaxBytes+1` limiter, run one token pass that
checks depth + duplicate keys, then run typed decoding with unknown-field
rejection. Do not rely on `encoding/json`'s default last-key-wins or
unknown-field behavior.

Every decoded closed variant is normalized to its value form except selectors,
whose sealed forms are pointers by design. Decoded selectors are newly
allocated. Every slice is newly owned. `StepOpts` receives the same normalize +
clone path as extents, axes, segments, and selectors.

## 3. Validation layers

Validation has three layers. Each layer runs before the next.

| layer | checks | may build geometry? |
|---|---|---|
| wire | envelope, JSON shape, tags, duplicates, limits visible in tokens | no |
| recipe | per-op fields, values, record structure, reference graph, liveness | no |
| evaluator | selector resolution, body-relative stops, supported payloads, geometry construction | yes |

`DecodeRecipe` runs wire + recipe validation. `Recipe.Validate` runs recipe
validation. `Evaluate` snapshots, runs recipe validation again, then runs the
evaluator. Revalidation is required because `Recipe` has exported fields and a
caller may construct or change one without the decoder.

### 3.1 Record checks

Recipe validation checks every reachable value:

- every coordinate, direction component, curve parameter, knot, weight, and
  `Rho` is finite;
- every `units.Value` has the kind required by its field and has a finite
  magnitude;
- magnitude fields are non-negative; signed taper + `ToFace.Offset` retain
  their signed rules;
- every required `PlaneRecord` rebuilds through `r3.NewFrame`;
- every required `TransformRecord` rebuilds through `r3.FromBasis`;
- every loop is non-empty;
- every segment variant satisfies its own field-count, unit, range, and winding
  rules;
- NURBS degree/control/knot/weight counts agree;
- selectors have the correct query kind, valid cardinality, valid predicates,
  finite directions, and valid provenance references;
- role strings are non-empty and within limits;
- nil interface values and typed nil variants are invalid;
- every closed-set value is one of decad's variants.

Profile closure, loop nesting, and trim admission were certified by `sketch`
before the record existed. A decoded `ProfileRecord` carries that exact-record
claim. Validation may disprove a contradiction in the stored fields, but it
MUST NOT recreate the sketch arrangement or use a small residual to bless a
trim. `docs/sketch-seam-design.md` §2.1 owns this trust boundary.

Passing `Validate` means “well-formed recipe IR,” not “supported by this
evaluator.” E.g. a nonzero recorded taper validates, then v1 returns
`ErrUnsupported`.

### 3.2 Per-operation shape

`nil`, empty, and zero below mean the field's absent wire form and Go zero
value. Every field not listed as required or allowed MUST be absent.

| `Op` | `Inputs` | required content | allowed variable content | consumed inputs |
|---|---:|---|---|---:|
| `OpExtrude` | 0+ unique | `Profile`, `Plane`, `Extent`, `ExtrudeOpts` | dependencies named/resolved by extent | 0 |
| `OpRevolve` | 0+ unique | `Profile`, `Plane`, `Angular`, `Axis` | dependencies named by angular extent/axis | 0 |
| `OpUnion` | 2 distinct | none | none | 2 |
| `OpCut` | 2 distinct, `[target, tool]` | none | none | 2 |
| `OpIntersect` | 2 distinct | none | none | 2 |
| `OpFillet` | 1 | one `EdgeSelector`, one length `Value` | none | 1 |
| `OpChamfer` | 1 | one `EdgeSelector`, one length `Value` | none | 1 |
| `OpShell` | 1 | one `FaceSelector`, one length `Value`, `ShellOpts` | none | 1 |
| `OpPlaced` | 1 | nonzero valid `Placement` | none | 1 |

Additional rules:

- `ExtrudeOpts.Taper` MUST be an angle. Zero and nonzero are both valid intent.
- Fillet radius, chamfer distance, and shell thickness MUST be positive lengths.
- `ShellOpts.Sense` MUST be `Inward` or `Outward`.
- Extrude and revolve `Profile.Outer` MUST be non-empty.
- `Extent` is required only for extrude.
- `Angular` + `Axis` are required only for revolve.
- `Placement` is required only for placed.
- `Selectors` are required only for modify operations.
- `Values` are required only for modify operations.
- `Opts` is required only for extrude + shell under the current vocabulary.
- Empty recipe is valid and evaluates to an empty document.

## 4. References and liveness

Every step produces exactly one body. Validation walks steps in order and keeps
one slot per prior step:

```text
slot = { produced body ref, live bool }
```

For step `i`:

1. Every `Inputs[j]` MUST satisfy `0 <= ref < i`.
2. Every input MUST be live at step `i`.
3. `Inputs` MUST contain no duplicate.
4. Nested body references MUST be `StepRef`, backward, live, and present in
   `Inputs`.
5. Provenance references in selector predicates MUST be backward and have a
   non-empty role. They need not be live: a current body's face may carry a
   role from a retired ancestor.
6. Mark the operation's consumed inputs retired.
7. Append step `i` as live.

This state machine derives final `Document.Bodies()` membership and order.
Retiring removes each consumed body from its current live position; committing
appends the produced body. Dependency-only bodies remain in place.

### 4.1 Dependency order

Canonical `Inputs` order is:

1. named extent references in extent walk order;
2. resolved through-all stop bodies in stop order;
3. axis reference;
4. first occurrence wins across the full list.

For extrudes without `ThroughAll`/`ThroughAllSide`, recipe validation can derive
the complete list from nested references and requires exact equality.

For an extrude with a through-all form, recipe validation requires:

- all derived named references appear in canonical prefix order;
- every remaining input is a unique, live, backward reference;
- `Inputs` is non-empty after first-occurrence deduplication.

Whether the recorded bodies actually stop every through-all side needs geometry
and is confirmed by the evaluator. During evaluation, through-all resolution
scans the replayed live body set exactly as an immediate call does, rebuilds the
complete canonical dependency list, and requires exact equality with recorded
`Inputs`. A mismatch is `ErrInvalidRecipe`, not a different sweep.

Revolve has no ambient through-all angular form. Its angular-stop refs followed
by its edge-axis ref are fully checkable before geometry, subject to
first-occurrence deduplication.

## 5. Evaluation boundary

The package owns an internal interface behind `Evaluator`:

```go
type evaluatorImpl interface {
    name() string
    evaluateStep(context.Context, *evaluationState, StepRef, Step) (*Body, []*Body, error)
}
```

This shape is illustrative; it is not public API. Required behavior:

- consume only the normalized `Step` snapshot + bodies produced by earlier
  steps;
- never read a live sketch;
- return one body + exact consumed-body list;
- never append a step or change document liveness itself;
- check context + evaluation budget inside long loops;
- return existing branchable evaluator errors unchanged through wrapping.

One package-owned commit tail appends the normalized step, retires the returned
consumed bodies, and appends the result. Immediate feature calls and recipe
evaluation MUST use the same recorded-step operation helpers. NEVER maintain a
second extrude/revolve/boolean/modify implementation for replay.

### 5.1 Replay algorithm

`Evaluate(ctx, recipe)`:

1. Return `ErrDegenerate` when `ctx == nil`.
2. Return `ctx.Err()` when already canceled/deadlined.
3. Deep-clone + normalize recipe into private memory.
4. Apply recipe limits and validate the full operation/reference graph.
5. Check `ctx.Err()` again, then create private empty `Document` + evaluator
   state.
6. For each step in order:
   - check `ctx.Err()`;
   - resolve input slots and nested refs against current live bodies;
   - recompute geometry-dependent dependencies and compare them with `Inputs`;
   - evaluate the step from its record;
   - commit only that successful step to the private document.
7. Return document only after all steps commit.

Step helpers reuse current record-based paths:

- extrude → frame + recorded extent resolution → `evalPrism`;
- revolve → recorded axis/angular resolution → `evalRevolve`;
- placed → `TransformRecord.Transform` + payload `placed`;
- booleans → shared boolean compute + `buildFacetedBody`;
- fillet/chamfer/shell → selector resolution + current exact section rewrite.

Selectors resolve against evaluator topology at their step. A conforming
alternate evaluator MUST preserve feature roles + query meaning. If its
topology cannot resolve recorded intent under the query's cardinality, it
returns `ErrCardinality`/`ErrUnsupported`; it NEVER chooses a different entity
to force replay through.

### 5.2 Whole-recipe atomicity

Per-step commits happen only in the private document. On any decode,
validation, cancellation, limit, or evaluator failure:

- return `nil, error` from `Evaluate`;
- expose no partially built document or body;
- leave caller recipe unchanged;
- leave every existing document unchanged;
- discard evaluator state.

An evaluator that publishes intermediate bodies, writes external state, or
needs rollback does not satisfy this contract and cannot back an `Evaluator`
handle.

## 6. Errors

Add these sentinels:

```go
var ErrInvalidRecipe = errors.New("decad: invalid recipe")
var ErrUnsupportedRecipeVersion = errors.New("decad: unsupported recipe version")
var ErrResourceLimit = errors.New("decad: resource limit exceeded")
```

Add path-aware errors:

```go
type RecipeError struct {
    StepIndex int    // -1 for envelope/root
    Path      string // JSON-style path, e.g. steps[3].inputs[0]
    Kind      error  // invalid recipe, version, or resource-limit sentinel
    Err       error  // optional specific cause
}

func (*RecipeError) Error() string
func (*RecipeError) Unwrap() error
func (*RecipeError) Is(error) bool

type EvaluationError struct {
    Step StepRef
    Op   OpKind
    Err  error
}

func (*EvaluationError) Error() string
func (*EvaluationError) Unwrap() error
```

`RecipeError.Kind` MUST be exactly one of `ErrInvalidRecipe`,
`ErrUnsupportedRecipeVersion`, or `ErrResourceLimit`. `RecipeError.Is` matches
`Kind` and the optional specific cause. Examples:

- wrong quantity kind → `ErrInvalidRecipe` + `ErrUnitKind`;
- non-finite coordinate → `ErrInvalidRecipe` + `ErrNotFinite`;
- forward/live-state failure → `ErrInvalidRecipe`;
- input/element ceiling → `ErrResourceLimit`;
- unknown format version → `ErrUnsupportedRecipeVersion`.

`EvaluationError` wraps evaluator or context errors without changing their
identity. Examples: `ErrUnsupported`, `ErrCardinality`, `ErrBooleanFailed`,
`context.Canceled`, `context.DeadlineExceeded`.

A geometry-time dependency mismatch is a `RecipeError` at that step because the
record is incomplete or changed. An evaluator's honest inability to build valid
intent remains `EvaluationError` wrapping `ErrUnsupported`.

## 7. Resource limits

Default recipe limits:

| field | default | count |
|---|---:|---|
| `MaxBytes` | 16 MiB | encoded input bytes |
| `MaxDepth` | 32 | JSON container nesting |
| `MaxSteps` | 4,096 | recipe steps |
| `MaxLoops` | 65,536 | all profile loops |
| `MaxSegments` | 262,144 | all loop segments |
| `MaxCurvePoints` | 1,048,576 | control + fit points |
| `MaxCurveScalars` | 1,048,576 | knots + weights |
| `MaxSelectors` | 16,384 | all selector objects, including nested stops/axes |
| `MaxPredicates` | 131,072 | all selector predicates |
| `MaxRoleBytes` | 256 | one provenance role |
| `MaxTotalStringBytes` | 1 MiB | all tags/roles/quantity strings |

`MaxBytes` + `MaxDepth` apply only while decoding JSON. Other fields apply to
decoded and caller-built recipes during `Validate`/`Evaluate`.

Default evaluation limits:

| field | default | count |
|---|---:|---|
| `MaxFacets` | 2,000,000 | facets generated/restated across evaluation |
| `MaxBooleanPairTests` | 50,000,000 | triangle pairs reaching boolean classification |
| `MaxExactFallbacks` | 5,000,000 | adaptive predicates falling back to exact arithmetic |

Rules:

- Limits are hard ceilings. Reaching the ceiling is allowed; exceeding it is
  `ErrResourceLimit`.
- Counters use checked integer addition. Overflow is `ErrResourceLimit`.
- Decoder checks byte/depth/string ceilings before typed allocation.
- Validator checks aggregate counts on caller-built in-memory recipes.
- Evaluator charges work before allocation or expensive calculation.
- Context cancellation is separate from limits and remains the caller's time
  budget.
- Increasing limits never changes geometric meaning. No input is clipped,
  sampled more coarsely, or skipped to fit a budget.
- Huge but finite coordinates are not a resource-limit case. Derived overflow
  is `ErrNotFinite` or `ErrUnsupported`; NEVER clamp a coordinate.

These defaults admit models far larger than current examples while bounding
single-input memory and combinatorial boolean work. Callers handling trusted
larger models opt into larger explicit values.

## 8. Determinism and evaluator changes

Canonical JSON is byte-stable for one normalized recipe:

- struct field order is fixed;
- closed variants use named tags;
- no maps enter the wire form;
- units retain their registered text form;
- legacy input re-encodes in canonical version 1 form.

For the same normalized recipe + evaluator implementation:

- step order, live-body order, `FeatureRef`s, role order, selector traversal,
  and export ordering are deterministic;
- exact measurements reproduce the same values;
- approximate measurements remain within their reported bounds;
- no map iteration decides topology or output order.

A later evaluator MUST reproduce:

- one produced body per step;
- the same body-consumption/liveness graph;
- the same feature roles and selector meaning;
- the same exact intent;
- measurements valid under its own `Exactness` + `Bound`.

It need not reproduce v1's internal payload, face split, mesh, or error bound.
Cross-platform transcendental results and different evaluators are compared by
the reported bounds, not raw float bits.

Evaluator upgrades do not rewrite recipes. A valid recipe that the selected
evaluator cannot build returns `ErrUnsupported` at its first unsupported step.

## 9. Implementation sequence

No public half-replay surface ships. Internal changes may land in this order:

1. strict envelope codec + version 1 legacy reader;
2. deep normalization, including `StepOpts`;
3. resource counters + full recipe validator;
4. shared recorded-step helpers for extrude/revolve/placed;
5. shared recorded-step helpers for booleans + modify operations;
6. private whole-recipe driver + atomic failure tests;
7. public `DecodeRecipe`, `Validate`, evaluator handle, and `Evaluate` together;
8. executable example + full replay/fuzz/property suite.

Public `Evaluate` MUST support every `OpKind` the same release can record.
Valid intent outside v1 geometry reach returns the same `ErrUnsupported` as its
immediate feature call.

## 10. Required tests

### 10.1 Wire

- versioned round-trip for every closed variant;
- legacy unversioned decode → canonical versioned encode;
- missing/partial envelope, unknown format/version;
- unknown field + duplicate key at every object layer;
- trailing value/data, depth boundary, byte boundary;
- pointer forms normalize; typed nils reject;
- canonical bytes stable across repeated encoding.

### 10.2 Validation

- one valid + every invalid field combination for every `OpKind`;
- wrong unit, negative magnitude, non-finite number, invalid frame/transform;
- malformed segment arrays + winding contradictions;
- negative/self/forward/missing/duplicate/reordered refs;
- retired operand/dependency reuse;
- nested body ref absent from `Inputs`;
- backward provenance ref to retired ancestor remains valid;
- empty recipe validates.

### 10.3 Evaluation

- every current operation alone;
- one mixed recipe containing extrude, revolve, stops, placed, boolean, fillet,
  chamfer, and shell;
- direct build vs replay: canonical recipe, final body order, origins, roles,
  selector results, topology checks, and measurements;
- through-all dependencies recompute exactly; changed/missing/extra/order errors
  reject;
- failure at every step index returns nil document + correct step error;
- `ErrUnsupported`, selector, boolean, context, and limit identities survive;
- repeated v1 evaluation is deterministic;
- evaluator snapshot does not alias caller slices/queries/options;
- every executable example's produced recipe validates + replays.

### 10.4 Robustness

- fuzz `DecodeRecipe`: no panic/hang; every accepted value validates;
- fuzz reference mutation: never access outside slot table;
- property: every recipe emitted by successful immediate public calls validates
  and replays;
- exact limit accepted, one-over rejected, counter overflow rejected;
- cancellation inside tessellation, boolean pair loops, exact fallback, and
  modify audits.

User-facing example flow belongs in `examples/`:

```text
build → json encode → DecodeRecipe → Validate → Evaluate → inspect/Verify
```

Assert on body count + computed measurement, not only successful return.
