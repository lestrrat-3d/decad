# Payload Verification Design

How `Verify` answers every question for `cupPayload` and `facetedPayload`.
Companion to:

- `docs/verification-design.md` — report meaning, tolerance, absence, status;
- `docs/clearance-design.md` — analytic distance candidates + pair partition;
- `docs/modify-design.md` — exact shell/cup construction;
- `docs/evaluator-design.md` — mesh boolean + evaluator staging.

This document owns payload adaptation, bounded mesh proofs, survey certificates,
implementation order, and required tests. It changes no public signature.

## 1. Coverage contract

`Verify` MUST support every payload the public evaluator builds. Unsupported
answers stay `Suspect` until their increment lands. NEVER turn a missing payload
case into nil, an empty list, or `Sound`.

| Payload | Validity | Pair clearance | `MinWallThickness` | `Undercuts` | `MinRadius` |
|---|---|---|---|---|---|
| `prismPayload` | exact construction proof | analytic kernel | exact 2D reduction | exact normal range | exact curvature |
| `revolvePayload` | exact construction proof | analytic kernel | exact meridian reduction | exact normal range | exact curvature |
| `cupPayload` | exact construction proof | exact analytic adapter (§3) | exact shell theorem (§4) | existing exact cup walk | existing exact cup walk |
| `facetedPayload` | bounded boundary proof (§6) | bounded triangle adapter (§7) | bounded medial survey (§10) | certified normal patches (§8) | certified curvature patches (§9) |

Two payload classes require different treatment:

- `cupPayload` is exact analytic data. Adapt its two recorded regions and three
  axial planes. NEVER tessellate it for verification.
- `facetedPayload` is an approximate boundary backed by a proof certificate.
  Read held polygons exactly, then widen every claim by that certificate.

## 2. Shared proof rules

### 2.1 Outcomes

Every internal verification routine returns one of:

| Outcome | Report effect |
|---|---|
| proven value interval `[lo, hi]` | emit midpoint/half-width `Measurement`; `Exact` iff `lo == hi` |
| proven absence | emit allowed nil/empty answer |
| proven spec failure | emit value/list where defined; `Violating` |
| proven invalidity | `Unsound` |
| undecided | emit no fabricated answer; `Suspect` |

Bounds MUST round outward. Put each new error mechanism in `bounds.go`; NEVER
compute a measurement bound at its call site.

### 2.2 Tolerance vs. proof

`WithTolerance` judges a bounded value after proof:

`Bound <= rel * Ref`

It MUST NOT change:

- validity;
- disjoint/overlap classification;
- undercut membership;
- wall-vs-edge classification at draft allowance;
- feature existence/absence.

Those are predicates. A predicate is proven, refuted, or undecided independently
of caller tolerance. A loose tolerance NEVER turns an unresolved pinch, contact,
normal sign, or missing feature into a pass.

Wall specs use `docs/verification-design.md`'s interval rule after measurement:

- `hi < tool` → `Violating`;
- `lo >= tool` → spec met, then run tolerance gate;
- otherwise → `Suspect`.

### 2.3 Boundary displacement

For payload `X`, define `delta(X)`:

- analytic payload → `0`;
- `facetedPayload` → payload boundary certificate's `Delta` (§5).

For pair `(A, B)`, define `DeltaPair = up(delta(A) + delta(B))`.

Distance between closed sets is 1-Lipschitz in each operand. If held-boundary
distance lies in `[loH, hiH]`, true-boundary distance lies in:

`[max(0, down(loH - DeltaPair)), up(hiH + DeltaPair)]`.

Use this formula once, after held-boundary candidate aggregation. NEVER add the
same payload bound per face, edge, candidate, or refinement level.

## 3. Cup boundary adapter

### 3.1 Record

For open end above closed end, cup solid is:

`(O x [zOuter, zOpen]) \\ (C x (zCav, zOpen])`.

Bottom-open cup is its exact axial mirror. `O` and `C` are recorded
`ProfileRecord`s in one frame. Shell construction proves:

- both regions are simple + correctly nested;
- loop counts match;
- `C` is exact erosion of `O` by `t` for inward shell, or `O` is exact
  dilation of `C` by `t` for outward shell;
- `t = abs(zCav - zOuter) > 0`;
- cavity interval is non-empty;
- every offset feature survives; topology does not change.

### 3.2 `bodyGeom`

Add `bodyGeom.addCupFaces(cp)` by refactoring prism helpers. Build exactly:

| Cup boundary | Carrier/trim | Orientation |
|---|---|---|
| every loop of `O` over outer interval | plane/cylinder side face | natural `O` walk |
| every loop of `C` over cavity interval | plane/cylinder side face | reversed `C` walk |
| kept cap at `zOuter` | planar region `O` | away from material |
| pocket floor at `zCav` | planar region `C` | into cavity |
| `rim(i)` at `zOpen` | planar band between paired loops `O_i`, `C_i` | toward opening |

Requirements:

- Reuse `recordLoops`, `walkElem`, `region2`, `cFace`, `cEdge`, and `addTopology`.
- Build no temporary prism caps. They are not cup faces.
- Keep one rim face per paired loop. Hole/post loop polarity follows
  `evalCup`'s existing topology.
- Add `cupPayload` to `payloadExtent` through `cp.extentAlong(g)`.
- Use outer-prism extrema for support points. Cavity lies inside.
- Use live cup shell witnesses for nesting casts.

All cup carriers, trims, edges, vertices, extents, and witnesses are exact.
Existing analytic clearance cells apply unchanged.

### 3.3 Pair result

Cup/cup, cup/prism, and cup/revolve pairs run normal analytic enumeration.

- Positive boundary lower bound + nesting excluded → proven disjoint.
- Certified analytic contact → `Clearance{Gap: Exact 0}` when requested.
- Nesting/crossing → hand pair to interference path (§11).
- Unresolved analytic cell → `Suspect`; NEVER fall back to cup tessellation.

## 4. Cup wall theorem

### 4.1 Result

For accepted cup shell thickness `t` and draft allowance `alpha`:

- any material junction with dihedral `<= alpha` → `MinWallThickness = Exact 0`;
- otherwise → `MinWallThickness = Exact t`.

A cup always has a wall. `MinWallThickness` is never nil after this proof lands.

### 4.2 Proof

Upper bound:

- Pick any strict interior point of `C`.
- Center a radius-`t/2` ball halfway between kept cap and pocket floor.
- Exact erosion/dilation proves ball fits inside `O`.
- Opposite cap contacts prove ball spans.
- Two opposite contacts make ball maximal.
- Therefore wall reading `<= t`.

Lower bound when no qualifying junction exists:

- Exact erosion/dilation makes every cavity skin an offset skin at distance `t`.
- S11 offset audit proves no feature drops, merges, crosses, or changes nesting.
- A non-corresponding skin pair closer than `t` would make offset regions cross,
  touch, merge, or drop a feature; S11 excludes every case.
- Floor thickness is exactly `t`.
- Vertical-horizontal cup junctions have dihedral `90 degrees`; legal
  `alpha < 90 degrees` excludes them.
- Therefore no spanning maximal ball has diameter below `t`.

Boundary limit:

- A junction with material dihedral `<= alpha` admits spanning balls tending to
  radius zero.
- Closure-under-limits rule makes reading genuine `Exact 0`.

The theorem depends on exact morphology, not recorded user intent. `Verify` MUST
recheck certificate facts it consumes. It MUST NOT return `t` from recipe value
alone.

### 4.3 Algorithm

`cupWall(cp, alpha)`:

1. Compute `t = abs(zCav - zOuter)`; reject internal inconsistency as undecided.
2. Reconfirm equal loop count, strict region nesting, and exact offset relation.
3. Walk every loop of `O` in natural material-left sense.
4. Walk every loop of `C` reversed, matching cavity material-left sense.
5. Run existing `junctionPinch` on each non-smooth junction.
6. Return exact zero on any qualifying pinch; else exact `t`.

Do not count:

- tangent offset joins — material angle is `pi`, not a pinch;
- vertical-horizontal cap/rim junctions — `90 degrees > alpha`;
- sharp concave floor edges in `MinRadius` — radius survey reads face curvature,
  not edge radius.

## 5. Faceted boundary certificate

`Faceted.Bound` exposes one length. Verification needs a stronger internal proof
behind it. Add no public field.

### 5.1 Contract

`facetedPayload` carries `boundaryCert`:

```go
type boundaryCert struct {
    Delta   float64
    Facets  []facetCert
    Local   []localJoinCert
}

type facetCert struct {
    SourceID int
    Normal   normalCert
    Radius   radiusCert
}
```

Names are illustrative; representation stays internal.

Certificate proves:

1. Each held facet maps continuously + orientation-preservingly to one non-empty
   true boundary patch.
2. Map and inverse move every point by at most `Delta` → two-sided Hausdorff
   bound plus correspondence.
3. Maps agree on every certified shared edge/vertex.
4. Each certified vertex fan maps to one local disk; each certified shell fan
   preserves its local orientation.
5. `normalCert` encloses every true patch normal (§8).
6. `radiusCert` encloses every true patch's concave principal radius state (§9).

Raw Hausdorff distance alone is insufficient: a sub-bound handle can stay near a
plane while changing topology. Local bijection/orientation clauses are mandatory.

### 5.2 Creation + composition

Analytic tessellation creates certificates:

- record source surface variant + trim cell internally;
- split parameter cells at normal/curvature sign boundaries;
- prove chord correspondence and sagitta bound per cell;
- make neighboring cell edge maps agree because boundary chording is shared.

Mesh boolean preserves certificates:

- exact facet subdivision gives every child its parent's source cell;
- tightening child parameter range is optional; inheriting parent range is safe;
- stitched shared vertices retain compatible local join certificates;
- tangency/branching refusal remains mandatory;
- final weld composes its rounding displacement into `Delta`;
- a weld that destroys local fan correspondence is `ErrUnsupported`, like a
  whole component welded away.

Second-generation booleans carry operand certificates through the same path.
Rigid placement:

- rotate normal certificates;
- preserve curvature certificates;
- add `rigidRoundAllow` to `Delta`;
- rebuild local fan certificates after reflected winding reversal.

Missing, malformed, or mismatched certificate → faceted validity + every survey
that needs it are undecided. NEVER reconstruct source identity from provenance
roles or fit an analytic surface to polygons.

### 5.3 Public readings

Strengthen existing meanings; add no API:

- `Faceted.Bound` is certificate `Delta`.
- `Vertex.Position().Bound` remains `Delta` or tighter local displacement.
- `Face.NormalAt(p)` on a faceted face finds held facet(s) containing `p`, unions
  their `normalCert`s, and returns `Approximate` direction bound.
- Point outside held face trim → `ErrDegenerate`.
- Certificate too broad for one direction ball → `ErrUnsupported`; verification
  consumes full internal range and remains `Suspect` where needed.

## 6. Faceted validity

### 6.1 Held audit

Audit held mesh exactly:

- every directed edge has one reverse;
- every vertex fan is one disk per shell;
- no collapsed facet;
- no remote facet intersection;
- shell signed volumes are nonzero;
- shell containment is decided by exact parity;
- face loop topology is consistent.

These fields describe held data exactly: `Solid`, `Watertight`, `Manifold`,
`SelfIntersecting`, `Lumps`, `Voids`.

### 6.2 Local vs. remote

Exclude a facet pair from self-clearance only when:

- facets share a vertex; and
- `localJoinCert` proves their whole common vertex fan maps to one local true
  sheet with consistent orientation.

No certificate → pair is remote. Same source id alone NEVER makes a pair local:
one source face can be split into disconnected patches or separated by a narrow
trim.

Also compare nonadjacent face-loop edges. This catches a narrow planar web whose
two trim boundaries approach while cap facets remain one connected sheet.

### 6.3 Feature scale

Build BVH over outward-rounded triangle/edge boxes. BVH only prunes; every remote
candidate whose boxes can beat current minimum MUST reach exact distance code.

Compute certified held feature-scale lower bound `sH` over:

- remote triangle/triangle pairs;
- nonadjacent loop edge/edge pairs;
- distinct shell pairs;
- neck-separating boundary pairs found by same remote primitive pass.

Reuse `triTriDistance`, segment distance, exact incidence, and exact parity.
Replace fixed fractional nudges with `bounds.go` helpers proving float slop.

### 6.4 Verdict

Let `delta = boundaryCert.Delta` and `guard = up(2 * delta)`.

| Held/certificate result | Part validity |
|---|---|
| clean held audit, complete local certificate, `down(sH) > guard` | proven valid |
| exact held defect and `delta == 0` | proven invalid |
| held defect with certified persistence depth `> guard` | proven invalid |
| anything else | undecided |

Persistence depth is minimum boundary displacement required to remove held
crossing, gap, or non-manifold junction. A bare intersection witness has depth
zero and proves no approximate part invalid.

When validity is undecided:

- held predicate fields + counts remain populated as held-data facts;
- `BodyReport.Status = Suspect`;
- region quantities (`Volume`, `Centroid`, opt-in solid surveys) are nil because
  report cannot vouch for a region.

When validity is proven, expose bounded region quantities and run tolerance gate.
Approximate does NOT imply `Suspect`; only failed bound gate does.

## 7. Faceted clearance

### 7.1 `bodyGeom`

Add `bodyGeom.addFacetedFaces(fp)`:

- one `ckPlane` face per held triangle;
- triangle-local orthonormal frame;
- three-line `region2` trim;
- outward normal from held winding;
- exact held triangle box + centroid witness;
- all triangle edges as line `cEdge`s, including internal tessellation edges;
- held vertices once each;
- one live topology witness per shell.

Duplicates are allowed. They cost time, never correctness. Internal triangle
edges are required: a constrained plane minimum can migrate to any triangle
boundary even when boundary is not a public B-rep edge.

Set `bodyGeom.delta = fp.boundaryCert.Delta`. Analytic adapters set zero.

### 7.2 Held interval + true interval

Run existing face/edge/vertex tiers against held geometry. Triangle faces reuse
the plane row. Parity ray casts use exact triangle predicates.

After candidate aggregation, expand once with §2.3 formula.

- `loTrue > 0` + held nesting excluded + both bodies proven valid → disjoint.
- `loTrue == 0` with nonzero `DeltaPair` → undecided unless separate exact contact
  certificate proves touching.
- `DeltaPair == 0` permits existing exact coplanar contact certificate.
- Held nesting/crossing routes to interference path (§11).

Gap measurement uses true interval midpoint + half-width. It is `Exact` only
when held interval is a point and `DeltaPair == 0`.

### 7.3 Extents + pair diameter

- Faceted directional extent = held vertex projection interval expanded by
  `delta`.
- Never call exact coplanar material-side certificate with nonzero extent bound.
- Pair-diameter underestimate remains safe for noise floor; include held support
  points from both bodies. Do not inflate it to loosen tolerance gate.

## 8. Faceted undercuts

### 8.1 Normal certificate

`normalCert` returns, for unit pull `p`:

- enclosing dot interval `[lo, hi]` for every true patch normal;
- `constant` bit + exact value for constant analytic normal;
- `ok` bit.

Plane → exact constant. Cylinder/cone/sphere/torus → closed-form range over source
parameter cell, widened by inherited direction bound. Child facets may inherit
parent range. Placement rotates certificate exactly, then adds direction-rounding
bound.

Length `Delta` alone NEVER supplies normal tilt. A tiny corrugation can fit inside
any positional bound while reversing its normal.

### 8.2 Three-way face decision

For each true patch:

- `constant && value == -1` → exact antiparallel carve-out; clears;
- `lo >= 0` → clears;
- `hi < 0 && lo > -1` → proven opposing patch;
- otherwise → undecided.

Aggregate by public `*Face`:

- any proven opposing patch → list face once;
- all patches clear → face settled clear;
- otherwise → face undecided.

Any listed face proves `Violating`, even if another face is undecided. Empty list
is proven only when every face settles clear. Otherwise empty + `Suspect`.

## 9. Faceted minimum radius

### 9.1 Radius certificate

Each `radiusCert` is one of:

```text
none                  # patch proven to have no concave principal curvature
interval [lo, hi]     # patch proven concave; its minimum radius lies here
unknown
```

Rules:

- Plane → `none`.
- Cylinder/sphere → exact signed radius or `none` from material orientation.
- Cone/torus → closed-form interval over parameter cell.
- Split tessellation cells at curvature-sign transitions. A cell NEVER mixes
  `none` and concave.
- Boolean child may inherit parent interval: lower bounds every subset; upper
  bounds every point in consistently-concave parent, so child remains enclosed.
- Placement preserves interval.

### 9.2 Aggregate

- Any `unknown` → question undecided.
- All `none` → proven nil, no concave feature.
- Otherwise global interval is
  `[min(all lo), min(all hi)]`.
- Emit midpoint/half-width; run tolerance gate.

Do not infer radius from faceted hinge angle or fit circles to vertices. Boolean
rim edges add no radius: `MinRadius` reads face principal curvature.

## 10. Faceted wall thickness

Nearest remote skins alone are not wall thickness. Survey MUST enforce maximal
inscribed ball, material containment, and draft-opposed contacts.

### 10.1 Held medial candidates

Triangulated closed solid's maximal ball has a contact subset of at most four
boundary features: contact directions' convex hull contains origin, and
Caratheodory in 3D reduces support to four.

Enumerate unordered feature tuples of size 2..4 over triangle interiors, edges,
and vertices. BVH prunes tuples whose distance boxes cannot beat current bound.

For each tuple, solve variables `(center, radius)` with certified interval
arithmetic:

- equal distance to tuple features;
- contact lies inside each feature trim;
- radius no greater than distance to every other triangle;
- center lies inside held solid by exact parity;
- contact-direction convex hull contains origin;
- at least two source-normal certificates admit contacts within `alpha` of
  diametrally opposite.

Parallel-face and other positive-dimensional families add stationarity of radius
on family, plus family endpoints. Interval Newton isolates zero-dimensional
roots; branch-and-bound encloses families. Every primitive is linear/quadratic on
held mesh.

Limits:

- max depth 80;
- max active boxes `1 << 20` per body;
- context cancellation checked every 1024 boxes.

Budget exhaustion returns honest remaining lower/upper enclosure. It NEVER drops
a family.

### 10.2 True interval

Let held thickness interval be `[loH, hiH]`. Distance-transform perturbation by
`delta` moves a ball radius by at most `delta`, so diameter interval is:

`[max(0, down(loH - 2*delta)), up(hiH + 2*delta)]`.

Normal certificates decide whether each contact family is a wall:

- proven within allowance → include;
- proven outside → exclude;
- straddles allowance → survey undecided, unless another exact-zero candidate
  already wins globally.

Outcomes:

- at least one proven family + all possible smaller families bounded → emit
  measurement interval;
- every family proven outside allowance → proven nil;
- possible family but no proven existence/value → nil + `Suspect`.

Then apply wall tool interval rule + tolerance gate independently.

## 11. Pair partition + interference dependency

This design proves disjointness and gap. It does not fabricate overlap volume.

Pair flow over proven solids:

1. Bounds-disjoint shortcut → proven disjoint; run payload kernel only when gap
   requested.
2. Run payload clearance adapter.
3. Positive true gap + nesting excluded → disjoint; optional `Clearance` row.
4. Exact touching certificate → disjoint interiors; zero `Clearance` row when
   requested.
5. Nested/crossing/zero-band pair → interference evaluator.
6. Bounded overlap volume → `Interference` row.
7. Interference evaluator undecided → pair omitted from both lists; `Suspect`.

Cup/faceted clearance can land before interference. Until bounded overlap volume
lands, overlapping pairs remain `Suspect` by verification's pair-partition rule.

## 12. Implementation map

| File | Change |
|---|---|
| `bounds.go` | interval expansion, distance slop, direction-bound composition |
| `clearance_geom.go` | `addCupFaces`, `addFacetedFaces`, payload delta |
| `clearance.go` | bounded extent/contact gates + true interval expansion |
| `shell.go` / `shell_cup.go` | expose/recheck exact morphology certificate internally |
| `survey.go` | `cupWall`; dispatch faceted surveys |
| `tessellate.go` | create/carry per-facet source certificates internally |
| `boolean.go` / `boolean_body.go` | compose `boundaryCert`; preserve facet survey metadata |
| `topology.go` | faceted `NormalAt` through certificate; no public type change |
| new `verify_mesh.go` | BVH feature scale + faceted validity |
| new `survey_mesh.go` | faceted undercut/radius/wall kernels |
| `verify.go` | validity-first presence; total tolerance gate; payload outcomes |

## 13. Increments

Each row leaves every later question staged as `Suspect`.

| PR | Lands |
|---|---|
| 1 | cup analytic boundary adapter, cup pair clearance, payload extent/contact gates |
| 2 | cup morphology recheck + exact wall theorem |
| 3 | faceted `boundaryCert` creation/composition + `NormalAt` |
| 4 | faceted held audit, BVH feature scale, validity/presence integration |
| 5 | faceted triangle boundary adapter + bounded clearance |
| 6 | faceted undercut + min-radius certificates/surveys |
| 7 | faceted medial wall survey |

Tolerance-gate implementation MUST land no later than PR 4. Otherwise a proven
valid faceted body still cannot become `Sound` when all bounds pass.

## 14. Required tests

### 14.1 Cup

| Area | Cases |
|---|---|
| boundary adapter | inward/outward; top/bottom opening; reflected/placed; holes/posts; face count + roles |
| clearance | cup/cup, cup/prism, cup/revolve; box-disjoint requested gap; nested; non-box-disjoint but clear; coplanar touch |
| wall | box/cylinder exact `t`; outward rounded cup; holed cup; top/bottom mirror; qualifying section pinch exact zero |
| wall spec | tool below/equal/above `t`; zero pinch violates every legal tool |
| refusals | malformed internal offset relation → `Suspect`, never fabricated `t` |

Every clearance test asserts gap value/bound. Every wall test asserts exact value,
not only status.

### 14.2 Faceted certificate + validity

| Case | Required result |
|---|---|
| ordinary curved boolean, remote feature scale `> 2*delta` | validity proven; quantities present |
| same body under default passing tolerance | `Sound` if no pair/spec issue |
| strict tolerance rejecting one quantity bound | `Suspect` from gate, not validity |
| narrow neck/trim gap `<= 2*delta` | validity undecided; region quantities nil |
| exact all-planar faceted body, `delta == 0` | clean mesh proves validity |
| remote held crossing with persistence `> 2*delta` | `Unsound` |
| crossing without persistence proof | `Suspect` |
| missing/local fan certificate | `Suspect` |
| placement/reflection | same topology verdict; rounding added to bound |
| second-generation boolean | certificates survive subdivision + composition |

Internal tests inject payloads for invalid/missing-certificate cases. Public API
cannot manufacture them.

### 14.3 Faceted clearance

- all-planar `delta == 0` exact gap;
- curved pair: returned interval contains independently sampled true gap;
- mixed analytic/faceted pair;
- held gap just above/below `DeltaPair`;
- nonzero-bound near-touch → undecided, never exact zero;
- nested/overlap route to interference;
- loose/strict `WithTolerance` over same gap interval;
- placement rounding widens but never shrinks interval.

### 14.4 Faceted surveys

| Survey | Cases |
|---|---|
| undercut | proven opposing patch; all-clear; exact antiparallel; perpendicular-straddling `Suspect`; one proven face plus another undecided still `Violating` |
| radius | planar nil; concave cylinder exact interval; torus variable interval; sign-split cell; unknown certificate `Suspect`; second generation |
| wall | parallel skins; curved skins; allowance boundary; exact zero; tool below/above; tool straddle; no-wall proof; budget exhaustion keeps honest interval/`Suspect` |

Property tests compare:

- certificate interval against dense analytic samples from source surfaces;
- BVH-pruned result against brute-force all-pair mesh pass;
- wall branch-and-bound interval against refined independent mesh samples;
- every refined interval nests inside its coarser predecessor.

## 15. Rejected shortcuts

- NEVER tessellate cup verification. Use exact cup payload adapter.
- NEVER return recipe shell thickness without morphology recheck. Use §4 proof.
- NEVER derive normal/curvature bounds from positional `Delta`. Carry source
  certificates.
- NEVER mark every `Approximate` body `Suspect`. Run total tolerance gate.
- NEVER use caller tolerance for validity/contact/predicate admission.
- NEVER emit exact faceted touching with nonzero `DeltaPair`.
- NEVER expose certificate triangles or indices through public API.
- NEVER call a held-mesh all-clear a part all-clear without certificate proof.

## 16. Resolved decisions

- Cup wall needs no stacked 3D solver: exact morphology proves `0` or `t`.
- Faceted bound means two-sided displacement under internal local correspondence,
  not raw one-sided proximity.
- Faceted self-clearance excludes only certified shared-vertex fans.
- Faceted normals/curvature use per-facet source certificates, not surface fitting.
- Triangle facets adapt to existing plane clearance cells; no separate mesh-only
  pair API.
- Validity ignores caller tolerance.
- Faceted touching with nonzero boundary bound remains undecided.

No public API decision remains open for this increment.
