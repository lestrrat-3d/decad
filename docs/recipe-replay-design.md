# Recipe Replay Design

How decad decodes, validates, and evaluates a stored `Recipe` into a new
`Document`. Companion to `docs/api-design.md` (public contract),
`docs/sketch-seam-design.md` (exact profile records), and
`docs/evaluator-design.md` (v1 geometry). References of the form "core §N" are
to `docs/api-design.md`.

This contract has four goals:

- stored recipes rebuild without the original live sketch; validation may use
  a private reconstructed sketch only to prove the stored profile;
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
    MaxProfilePairTests int64
    MaxRoleBytes        int
    MaxTotalStringBytes int
}

func DefaultRecipeLimits() RecipeLimits

type EvaluationLimits struct {
    MaxFacets           int
    MaxBooleanPairTests int64
    MaxExactFallbacks   int64
    MaxGeometryWork     int64
}

func DefaultEvaluationLimits() EvaluationLimits

type DecodeRecipeOption interface {
    option.Interface
    decodeRecipeOption()
}

type EncodeRecipeOption interface {
    option.Interface
    encodeRecipeOption()
}

type ValidateRecipeOption interface {
    option.Interface
    validateRecipeOption()
}

type EvaluateOption interface {
    option.Interface
    evaluateOption()
}

// RecipeOption satisfies EncodeRecipeOption, DecodeRecipeOption,
// ValidateRecipeOption, and EvaluateOption.
type RecipeOption interface {
    EncodeRecipeOption
    DecodeRecipeOption
    ValidateRecipeOption
    EvaluateOption
}

func WithRecipeLimits(RecipeLimits) RecipeOption
func WithEvaluationLimits(EvaluationLimits) EvaluateOption

func EncodeRecipe(w io.Writer, r Recipe, opts ...EncodeRecipeOption) error
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

- `EncodeRecipe` validates one caller-built recipe, then writes one canonical
  versioned JSON envelope. It buffers at most `MaxBytes` and writes nothing to
  the caller's writer until validation + bounded encoding succeed. Nil writer
  → `ErrDegenerate`.
- `DecodeRecipe` decodes one JSON recipe and validates it.
- Nil reader → `ErrDegenerate`.
- `Recipe.Validate` is pure. It never changes the recipe or builds body
  geometry. It may build a private 2D sketch arrangement only to revalidate a
  stored `ProfileRecord` (§3.1).
- `Evaluate` always creates a new `Document`. It never appends into an existing
  document.
- No option selects a destination document.
- No `Load` synonym ships. Call `DecodeRecipe`, inspect if needed, then call
  `Evaluate`.
- No `Replay` synonym ships. `Evaluate` is the one operation that runs exact
  intent through an evaluator.
- No options → default recipe limits + default evaluation limits + v1 evaluator.
- Any nil option passed to encode, decode, validate, or evaluate →
  `ErrDegenerate`.
- Every field in `RecipeLimits` and `EvaluationLimits` MUST be strictly
  positive. Any zero or negative field is `ErrDegenerate`. Call the default
  constructor, then change named fields.
- A zero `Evaluator` passed to `WithEvaluator` is `ErrDegenerate` at
  `Evaluate`.
- `Evaluator` is a concrete opaque value, not a user-implemented interface.
  `Document` topology has package-owned construction invariants; an external
  implementation cannot safely build it. Future bundled evaluators return
  other `Evaluator` values.
- Code generators consume `Recipe` directly. They are not evaluators because
  they do not produce a decad `Document`.

`Recipe` contains slices and pointer-form closed variants. Every typed entry
path — `EncodeRecipe`, `Recipe.MarshalJSON`, `Recipe.Validate`, and `Evaluate` —
selects + validates its recipe limits before copying, then uses one limit-aware
deep normalizer. The normalizer charges each source aggregate before allocating
or growing its destination and returns `ErrResourceLimit` at the first one-over
element. It charges steps, loops, segments, curve points/scalars, selectors,
predicates, and role/total string bytes. Before copying, it also rejects an
`Inputs` slice longer than its step index and a `Values` slice longer than one
as `ErrInvalidRecipe`; no valid operation can contain either shape. No private
destination slice ever grows beyond the selected limit or those structural
bounds.

The successful result is a private normalized snapshot: every slice and
pointer-form closed variant is newly owned, and caller mutation after snapshot
cannot affect validation, encoding, or evaluation. A partial snapshot is
discarded on failure. Concurrent mutation while a typed entry path copies the
same recipe is unsupported and is a data race, on the same terms as concurrent
mutation of `Document`.

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
- `Recipe.MarshalJSON` runs the full recipe validator, including independent
  stored-profile reconstruction, under `DefaultRecipeLimits`, then emits the
  canonical envelope. A malformed or over-default caller-built recipe refuses
  to encode before unmetered arrangement work, over-limit private growth, or
  an output allocation beyond `MaxBytes`.
- `EncodeRecipe` is the limit-aware encoding path. It runs the same full
  validation and canonical encoder under its selected recipe limits. Callers
  with trusted recipes above the defaults must use it with explicit limits;
  `json.Marshal` has no option channel and never bypasses the defaults.
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

- invalid raw UTF-8;
- a JSON string containing a lone, reversed, or otherwise malformed UTF-16
  surrogate escape sequence;
- an unknown field at any object level;
- a duplicate key at any object level;
- a missing required field;
- an unknown or missing closed-set `kind` tag;
- more than one top-level JSON value;
- non-whitespace after the top-level value;
- nesting past `MaxDepth`;
- input past `MaxBytes`.

Implementation: read at most `MaxBytes` into the retained buffer without
incrementing the limit. If the buffer reaches `MaxBytes`, perform a separate
one-byte probe; any returned byte means over-limit, even when the probe also
returns `io.EOF`. Zero probe bytes with `io.EOF` means the input ends exactly
at the ceiling. Then run one schema-aware raw JSON preflight before any
`encoding/json` call. The preflight validates raw
UTF-8, validates every escaped Unicode scalar (including surrogate pairing),
checks depth + duplicate keys, and charges every recipe aggregate at its schema
position. Charge a step, loop, segment, curve point/scalar, selector, or
predicate before the corresponding typed slice could grow. Charge each stored
profile's checked arrangement-work upper bound against `MaxProfilePairTests`
before typed decoding. Charge decoded UTF-8 bytes for each string before storing
it, including per-role and total string ceilings. Only after the complete
preflight succeeds may typed decoding run with unknown-field rejection. Do not
rely on `encoding/json`: it replaces invalid UTF-8 and malformed surrogate
sequences with U+FFFD, uses last-key-wins for duplicates, and ignores unknown
fields by default.

Every decoded closed variant is normalized to its value form except selectors,
whose sealed forms are pointers by design. Decoded selectors are newly
allocated. Every slice is newly owned. `StepOpts` receives the same normalize +
clone path as extents, axes, segments, and selectors.

## 3. Validation layers

Validation has three layers. Each layer runs before the next.

| layer | checks | may build body geometry? |
|---|---|---|
| wire | envelope, JSON shape, Unicode scalars, tags, duplicates, schema-aware decode limits | no |
| recipe | per-op fields, values, independently proven profile structure, reference graph, liveness | no |
| evaluator | selector resolution, body-relative stops, supported payloads, geometry construction | yes |

`DecodeRecipe` runs wire + recipe validation. `EncodeRecipe`,
`Recipe.MarshalJSON`, and `Recipe.Validate` run bounded normalization + recipe
validation. `Evaluate` runs bounded normalization, recipe validation again,
then the evaluator. Revalidation is required because `Recipe` has exported
fields and a caller may construct or change one without the decoder.

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
- role strings are non-empty, valid UTF-8, and within limits;
- nil interface values and typed nil variants are invalid;
- every closed-set value is one of decad's variants.

A decoded or caller-built `ProfileRecord` carries geometry, not certification.
Recipe validation MUST independently prove its region from the stored analytic
IR before evaluation:

- reconstruct the recorded entities in a private `sketch.Sketch` and ask
  `sketch` to build its arrangement;
- require one valid arranged profile to match the stored outer/hole walks
  exactly: entity definitions, segment order + sense, ranges, closure,
  simplicity, and hole containment/disjointness;
- require every matched partial fragment to have `BoundaryEdge.TExact == true`,
  then apply the seam's reject-only range falsifier;
- reject no match, multiple possible matches, sampled trim, self-contact,
  crossing, broken closure, or invalid nesting as `ErrInvalidRecipe`.

No tolerance or small residual may turn a mismatch into a match. The validator
uses `sketch`'s independent arrangement result; it never trusts the writer's
loop shape or an absent `TExact` claim. The original live sketch is not needed,
and the evaluator still consumes only the validated record. Before invoking an
arrangement or containment walk that cannot accept a counter, compute and
charge a checked worst-case visit bound against `MaxProfilePairTests`; overflow
or a one-over bound is `ErrResourceLimit`. The immediate-call seam and replay
trust boundaries are specified in `docs/sketch-seam-design.md` §2.1.

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
3. Parse + validate options, selecting recipe/evaluation limits + evaluator.
4. Deep-clone + normalize into private memory through the selected recipe
   limits; charge before every destination allocation/growth.
5. Validate the full operation/reference graph on that bounded snapshot.
6. Check `ctx.Err()` again, then create private empty `Document` + evaluator
   state.
7. For each step in order:
   - check `ctx.Err()`;
   - resolve input slots and nested refs against current live bodies;
   - recompute geometry-dependent dependencies and compare them with `Inputs`;
   - evaluate the step from its record;
   - commit only that successful step to the private document.
8. Return document only after all steps commit.

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
    Path      string // JSON-style path; "$" for root, e.g. steps[3].inputs[0]
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
- unproved profile closure, simplicity, nesting, or trim → `ErrInvalidRecipe`;
- unknown format version → `ErrUnsupportedRecipeVersion`.

`EvaluationError` wraps evaluator or context errors without changing their
identity. Examples: `ErrUnsupported`, `ErrCardinality`, `ErrBooleanFailed`,
`context.Canceled`, `context.DeadlineExceeded`.

A geometry-time dependency mismatch is a `RecipeError` at that step because the
record is incomplete or changed. An evaluator's honest inability to build valid
intent remains `EvaluationError` wrapping `ErrUnsupported`.

### 6.1 Deterministic error precedence and traversal

Every public entry point returns the first failure in the order below. It never
validates independent branches in parallel and never replaces an earlier error
with a later, more specific one.

Call-entry order:

1. `Recipe.MarshalJSON` selects `DefaultRecipeLimits`; it has no option gate.
2. `EncodeRecipe` checks the writer, then applies options left-to-right.
3. `DecodeRecipe` checks the reader, then applies options left-to-right.
4. `Recipe.Validate` applies options left-to-right.
5. `Evaluate` checks nil context, then the context's existing error, then
   applies options left-to-right.

At an option position, nil or invalid content fails immediately. Later valid
options of the same kind replace earlier ones. After validation + canonical
encoding succeed, an `EncodeRecipe` writer failure is returned as the writer's
error.

`DecodeRecipe` then uses this wire order:

1. Read at most `MaxBytes` into the retained buffer without incrementing the
   limit. If the buffer reaches the ceiling, perform a separate one-byte
   probe. Any returned byte wins over every defect in the preflight buffer as
   an over-limit input; zero probe bytes with `io.EOF` means exact-limit input.
2. Scan the retained JSON depth-first in source order. At each token, check
   syntax + Unicode scalar validity, charge container depth before entering
   it, check a repeated object key before its schema membership, check schema
   membership, then charge an array element/string before retaining or
   descending into it. The first failing token wins.
3. At each object close, check missing required fields in that object's
   canonical field order. At the root, first choose the envelope shape: neither
   `format` nor `version` means legacy and requires `steps`; either one means
   versioned and requires both plus `steps`. Only after required-field checks
   pass does the versioned form check `format`, then `version`. An incomplete
   envelope therefore fails before an unsupported version; a complete
   unsupported version fails before typed step decode or recipe validation.
4. Run typed decoding only after the whole preflight succeeds, then run the
   typed-recipe order below.

Every typed-recipe path next uses this order:

1. Bounded normalization traverses `Steps` by ascending index and each step in
   canonical field order: `Op`, `Inputs`, `Profile`, `Plane`, `Extent`,
   `Angular`, `Axis`, `Placement`, `Selectors`, `Opts`, `Values`. It charges a
   complete source slice/string before allocating or retaining its destination;
   a failed checked addition reports the path of the first element/byte beyond
   the ceiling. `MaxRoleBytes` is checked before `MaxTotalStringBytes` for one
   role. `MaxDepth` remains decode-only.
2. Nested slices use ascending index. Nested objects use canonical JSON field
   order. A profile visits `Outer`, then `Holes` by index; each loop visits
   `Segments` by index; each segment visits its tagged fields in canonical wire
   order. A selector visits its tag + cardinality before its predicates, and
   predicates by index + canonical field order.
3. Recipe validation walks steps by ascending index. For one step it checks:
   the `Op` tag; required/allowed/absent field shape in the canonical step-field
   order above; non-reference field values in that same order; independent
   profile reconstruction after every stored profile field passes; direct
   `Inputs` references by index; then nested body/provenance references
   depth-first in `Extent`, `Angular`, `Axis`, and `Selectors` order.
4. Only after that step passes does validation apply its consumed-input
   liveness changes and append the produced slot, then visit the next step.
   Thus an earlier step always wins among recipe-semantic failures; bounded
   normalization is the earlier layer and its limit/structural error wins first.
   Within one step a field/value/profile defect wins over a reference/liveness
   defect.

Profile pair-work is charged before an arrangement/containment visit or before
calling an unmetered `sketch` routine. A one-over charge therefore wins over
the geometry result that visit could have produced.

After recipe validation, the marshal/encode paths generate canonical bytes in
field order and charge each output chunk before growing their private buffer.
A one-over `MaxBytes` result is a root `RecipeError` wrapping
`ErrResourceLimit`; `EncodeRecipe` has not called the writer yet.

Bounded normalization + recipe validation do not poll context; they are the
same pure pipeline used by `Recipe.Validate`. After `Evaluate`'s initial context
check, a normalization/recipe error therefore wins over cancellation observed
only at the post-validation check.

After recipe validation, `Evaluate` checks context before creating evaluator
state. It then visits steps in order. At each step it checks context, resolves
and recomputes dependencies, rejects a mismatch as `RecipeError`, then enters
the evaluator. Every long-loop check point tests context first, charges the
applicable evaluation counter second, then performs the visit. The first
observed context error therefore wins over a simultaneous one-over budget at
that check point; a one-over budget wins over any operation error the refused
visit could have produced. Evaluator helpers retain the immediate operation's
documented gate order, and their first failure is wrapped in `EvaluationError`.

## 7. Resource limits

Default recipe limits:

| field | default | count |
|---|---:|---|
| `MaxBytes` | 16 MiB | encoded input/output bytes |
| `MaxDepth` | 32 | JSON container nesting |
| `MaxSteps` | 4,096 | recipe steps |
| `MaxLoops` | 65,536 | all profile loops |
| `MaxSegments` | 262,144 | all loop segments |
| `MaxCurvePoints` | 1,048,576 | control + fit points |
| `MaxCurveScalars` | 1,048,576 | knots + weights |
| `MaxSelectors` | 16,384 | all selector objects, including nested stops/axes |
| `MaxPredicates` | 131,072 | all selector predicates |
| `MaxProfilePairTests` | 50,000,000 | stored-profile arrangement/containment visits |
| `MaxRoleBytes` | 256 | one provenance role |
| `MaxTotalStringBytes` | 1 MiB | all tags/roles/quantity strings |

`MaxDepth` applies only while decoding JSON. `MaxBytes` applies to the bounded
decode read and to canonical encoding before bytes reach the caller. Every
other recipe limit applies during schema-aware decode preflight and during the
bounded normalization used by encode, validate, and evaluate. Both paths
charge before typed slice allocation/growth. `MaxProfilePairTests` is shared
across all stored profiles in the subsequent recipe validation. Charge every
arrangement candidate-pair and loop-containment segment visit, or a checked
worst-case upper bound before calling a `sketch` routine that cannot accept the
counter.

Default evaluation limits:

| field | default | count |
|---|---:|---|
| `MaxFacets` | 2,000,000 | facets generated/restated across evaluation |
| `MaxBooleanPairTests` | 50,000,000 | facet-pair visits before box pruning/classification |
| `MaxExactFallbacks` | 5,000,000 | adaptive predicates falling back to exact arithmetic |
| `MaxGeometryWork` | 50,000,000 | other input-dependent geometry visits |

`MaxBooleanPairTests` is shared by the pre-boolean proximity gate and mesh
boolean. Charge every candidate facet-pair visit before its box test, including
a pair that pruning rejects. A future spatial index may replace the Cartesian
walk only if it charges every index/candidate visit against this ceiling.

`MaxGeometryWork` covers input-dependent geometry loops not charged by the
facet-pair or exact-fallback counters. Current charge sites include every
non-adjacent segment-pair visit in the shared fillet/chamfer/shell section audit
and every segment visit in each nesting classification. Polygon triangulation
charges every segment/vertex visit while choosing or validating a hole bridge,
every ear candidate, and every candidate-versus-vertex visit, for cap and
boolean polygons. Charge before each visit. New nested geometry work MUST use
this counter or add an operation-specific hard ceiling. All evaluation counters
are shared across the whole recipe evaluation.

Rules:

- Every `RecipeLimits` and `EvaluationLimits` field MUST be strictly positive.
  Reject any zero or negative field as `ErrDegenerate` before limit arithmetic,
  reads, allocation, validation, or evaluator state creation.
- Limits are hard ceilings. Reaching the ceiling is allowed; exceeding it is
  `ErrResourceLimit`.
- Counters use checked integer addition. Overflow is `ErrResourceLimit`.
- Decoder rejects invalid Unicode and checks every recipe ceiling during the
  schema-aware preflight, before typed allocation.
- The bounded normalizer checks the same aggregate counts on decoded and
  caller-built in-memory recipes before allocating its private copy.
- Evaluator charges work before allocation or expensive calculation.
- Context cancellation is separate from limits and remains the caller's time
  budget.
- Increasing limits never changes geometric meaning. No input is clipped,
  sampled more coarsely, or skipped to fit a budget.
- Huge but finite coordinates are not a resource-limit case. Derived overflow
  is `ErrNotFinite` or `ErrUnsupported`; NEVER clamp a coordinate.

These defaults admit models far larger than current examples while bounding
single-input memory and input-dependent geometry work. Callers handling
trusted larger models opt into larger explicit values.

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
3. schema-aware decode counters + evaluation counters + full recipe validator;
4. shared recorded-step helpers for extrude/revolve/placed;
5. shared recorded-step helpers for booleans + modify operations;
6. private whole-recipe driver + atomic failure tests;
7. public `EncodeRecipe`, `DecodeRecipe`, `Validate`, evaluator handle, and
   `Evaluate` together;
8. executable example + full replay/fuzz/property suite.

Public `Evaluate` MUST support every `OpKind` the same release can record.
Valid intent outside v1 geometry reach returns the same `ErrUnsupported` as its
immediate feature call.

## 10. Required tests

### 10.1 Wire

- versioned round-trip for every closed variant;
- legacy unversioned decode → canonical versioned encode;
- `Recipe.MarshalJSON` runs full profile validation with default limits;
  `EncodeRecipe` produces identical canonical bytes under the same limits;
- exact-limit + one-over marshal profile-arrangement work, with instrumentation
  proving the one-over path does not enter the private arrangement;
- exact-limit + one-over custom `EncodeRecipe` limits for a recipe above the
  defaults;
- exact-limit + one-over encoded bytes; the one-over `EncodeRecipe` writer
  receives no bytes;
- missing/partial envelope, unknown format/version;
- unknown field + duplicate key at every object layer;
- invalid raw UTF-8 + every malformed surrogate form at every free-string
  schema position, including both provenance-role predicates and every
  quantity-bearing field;
- trailing value/data, depth boundary, byte boundary;
- every `RecipeLimits` field set to zero and to a negative value, one field at
  a time from defaults, returns `ErrDegenerate` through `EncodeRecipe`,
  `DecodeRecipe`, `Recipe.Validate`, and `Evaluate`;
- `DecodeRecipe` with `MaxBytes == math.MaxInt64` accepts a small valid recipe;
  a small malformed recipe returns its wire error rather than overflow,
  panic, or a false `ErrResourceLimit`;
- compact exact-limit/one-over arrays for steps, loops, segments, curve
  points/scalars, selectors, and predicates; instrument typed decoding to prove
  no corresponding slice grows past its limit;
- pointer forms normalize; typed nils reject;
- canonical bytes stable across repeated encoding.

### 10.2 Validation

- one valid + every invalid field combination for every `OpKind`;
- wrong unit, negative magnitude, non-finite number, invalid frame/transform;
- malformed segment arrays + winding contradictions;
- hostile stored profiles: open positive-area walk, self-crossing/touching loop,
  crossing/outside/nested holes, ambiguous arrangement match, and a partial
  range whose reconstructed boundary reports `TExact == false`;
- valid reconstructed whole + exact-partial profiles prove closure, simplicity,
  nesting, and trim admission;
- an adversarial stored profile exhausts `MaxProfilePairTests` promptly before
  starting an unmetered private arrangement;
- exact-limit + one-over caller-built aggregates for every typed-recipe limit;
  instrument the normalizer to prove no destination slice grows beyond the
  selected ceiling;
- overlong caller-built `Inputs` + `Values` slices reject before their private
  destination allocates;
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
- exact-limit + one-over `Evaluate` cases for every in-memory aggregate;
  instrumentation proves the rejected destination never reaches one-over;
- every `EvaluationLimits` field set to zero and to a negative value, one field
  at a time from defaults, makes `Evaluate` return `ErrDegenerate` before
  evaluator state creation;
- multi-defect recipes pin the first step, exact path, `Kind`, and optional
  cause across `DecodeRecipe`, `Validate`, and `Evaluate`, including
  field/value vs reference/liveness and context vs budget cases;
- `ErrUnsupported`, selector, boolean, context, and limit identities survive;
- repeated v1 evaluation is deterministic;
- evaluator snapshot does not alias caller slices/queries/options;
- every executable example's produced recipe validates + replays.

### 10.4 Robustness

- fuzz `DecodeRecipe`: no panic/hang; every accepted value validates;
- fuzz reference mutation: never access outside slot table;
- property: every recipe emitted by successful immediate public calls validates
  and replays;
- exact limit accepted, one-over rejected, counter overflow rejected for every
  recipe + evaluation counter;
- dense box-pruned facet products exhaust `MaxBooleanPairTests` promptly even
  when no pair reaches classification;
- large modify-section audits and adversarial cap polygons exhaust
  `MaxGeometryWork` promptly before completing their quadratic scans;
- cancellation inside tessellation, boolean pair loops, exact fallback, and
  modify audits + triangulation.

User-facing example flow belongs in `examples/`:

```text
build → EncodeRecipe → DecodeRecipe → Validate → Evaluate → inspect/Verify
```

Assert on body count + computed measurement, not only successful return.
