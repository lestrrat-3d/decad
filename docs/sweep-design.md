# Sweep Design

`Sweep` moves one recorded planar profile along an ordered spatial path. The
profile stays normal to the path and uses rotation-minimizing transport. This
document owns the spatial-path vocabulary, frame transport, refusals, topology,
measurements, downstream coverage, and staged implementation.

Companion contracts remain authoritative for their existing areas:

- `docs/api-design.md` owns the public model and sketch-profile seam;
- `docs/sketch-seam-design.md` owns profile authentication and recording;
- `docs/evaluator-design.md` owns topology, commits, and payload dispatch;
- `docs/tessellation-design.md` owns public mesh proofs and boolean admission;
- `docs/verification-design.md` owns report meaning and tolerance gates;
- `docs/spline-design.md` owns free-form profile-segment reach.

Eight tables are normative:

| Table | States | Section |
|---|---|---|
| **P** | spatial path representation and frame transport | §3 |
| **S** | refusals and their sentinels | §5 |
| **B** | payload, topology, faces, and roles | §8 |
| **D** | downstream coverage and staging | §10 |
| **SC** | what refuses a chain sweep | §15.4 |
| **SM** | what refuses a mitred polyline sweep | §16.4 |
| **BM** | a mitred polyline sweep's topology, faces, and roles | §16.5 |
| **DM** | downstream coverage of a mitred polyline sweep | §16.7 |

## 1. Scope

The admitted operation has one closed planar profile and one ordered spatial
path. The path starts in the profile plane, leaves it along the plane's positive
normal, and is position-continuous. Every admitted internal join is tangent.
The section moves by the unique rotation-minimizing rigid motion along each
straight or circular path segment.

The first implementation admits:

- an open path made from `LineTo` and `ArcThrough` segments;
- line, circle, and arc profile boundaries;
- zero total twist;
- tangent internal joins;
- a build whose local curvature gates and global contact audit prove a simple,
  closed solid.

The design also fixes these later increments:

- Tier A free-form profile boundaries follow `docs/spline-design.md` reach;
- nonzero distributed twist builds a certified faceted sweep;
- exact-restatement tessellation admits the payload to export and booleans;
- a future constrained 3D sketch may be recorded into the same `Path` without
  changing `Document.Sweep`.

**§16 owns the one corner mode and the one scale this document admits**: a
`LineTo`-only path whose joins are cut on a stated join plane
(`WithMitredJoins()`), and a dimensionless factor per span end that scales the
section about the span's own axis (`WithSectionScale`), over a `LineSeg`-only
profile. Everything §1 through §14 states about tangent joins, transport and
the analytic span builders is what that section replaces for its own case and
leaves unchanged for every other path.

The following remain outside this design:

- a scale along a path that is not §16's: a scale on an arc span, a scale
  whose section is not `LineSeg`-only, or a scale that varies within a span
  other than linearly;
- guide surfaces, guide rails, and a second profile;
- a corner mode other than §16's join plane: a rounded corner, or a corner
  on an arc span;
- closed paths and their frame holonomy;
- a sweep that lets the section leave the path-normal plane;
- a helical path: `docs/helix-design.md` builds it as `Document.Coil`, a
  screw motion whose section stays in the axis plane, and its §12 states
  why it is not a `Path` segment.

`Loft` remains the operation for two different sections. `Revolve` remains the
direct operation for one angular span. `Sweep` does not infer either operation
from approximate input; its evaluator may reduce an admitted path segment to
their existing builders internally.

**`Document.Sweep` takes a closed profile and always will. The OPEN sketch
chain is `Document.SweepChain`, a separate entry point, and §15 owns it** —
its pairing rule, signature, refusals, result and staging. Everything §1
through §14 states about the path, the transport, the span builders and the
global audit is what that section consumes; what it replaces is the section
alone.

## 2. Public API

```go
func (d *Document) Sweep(
    ctx context.Context,
    s *sketch.Sketch,
    p *sketch.Profile,
    path *Path,
    opts ...SweepOption,
) (*Body, error)

type Path struct { /* immutable */ }

type PathSegment interface {
    pathSegment() // sealed
}

// LineTo joins the preceding path point to End by one straight segment.
type LineTo struct {
    End r3.Vec
}

// ArcThrough joins the preceding path point to End by the unique circular arc
// through Through. Start, Through, and End must be distinct and non-collinear.
type ArcThrough struct {
    Through r3.Vec
    End     r3.Vec
}

func NewPath(start r3.Vec, segments ...PathSegment) (*Path, error)
func (p *Path) Start() r3.Vec
func (p *Path) End() r3.Vec
func (p *Path) Segments() []PathSegment

type SweepOption interface { /* sealed */ }

// WithSweepTwist applies total signed rotation about the transported tangent.
// Twist is distributed in proportion to path arc length. Zero is the default.
func WithSweepTwist(angle units.Value) SweepOption

// WithSurfaceResult omits the two section caps and publishes a sheet body
// instead of a solid (docs/surface-design.md §4). Unlike every other sealed
// SweepOption, a repeated WithSurfaceResult() is idempotent rather than S12's
// stated ErrDegenerate: it carries no payload for a repeat to disagree with.
func WithSurfaceResult() SurfaceResultOption

// WithMitredJoins admits a LineTo-only path whose internal joins are corners:
// the two spans meeting at a join are each cut on the join plane §16.3
// states and share the section polygon on it. §16 owns it.
func WithMitredJoins() SweepOption

// WithSectionScale states one dimensionless factor per path segment, in path
// order: factors[k] is the ratio of the section at the END of span k to the
// authored profile. §16 owns it.
func WithSectionScale(factors ...units.Value) SweepOption
```

```go
path, err := decad.NewPath(
    r3.NewVec(0, 0, 0),
    decad.LineTo{End: r3.NewVec(0, 0, 20)},
    decad.ArcThrough{
        Through: r3.NewVec(5, 0, 25),
        End:     r3.NewVec(10, 0, 20),
    },
)
body, err := doc.Sweep(ctx, s, profile, path)
```

`Path` is spatial: every point is an `r3.Vec` coordinate in millimetres. It is
not tied to a `Document`, body, or live sketch. `NewPath` copies every segment,
and `Segments` returns a copy. A caller may reuse one path in several documents.

`ArcThrough` uses three points because they state a connected circular path
without a derived endpoint. A center/axis/angle input would compute the next
segment's start through trigonometry; the held arc endpoint and the following
segment's exact start could then disagree. The three-point form makes every join
share one recorded coordinate. Its circle, plane, sense, and swept angle are
derived exactly from decad-owned spatial input, with bounded publication into
binary64.

`WithSweepTwist` is accepted at most once. Its angle is a signed displacement,
not a magnitude: a negative value reverses the twist sense. A wrong kind is
`ErrUnitKind`; a non-finite value is `ErrNotFinite`. The first implementation
accepts zero and returns `ErrUnsupported` for a nonzero value.

Both profiles pass through the unchanged sketch seam. `p` MUST be a current,
unaltered, valid profile of `s`. The seam's own sentinel wins before any path
geometry is evaluated.

## 3. Table P — path and frame contract

`Path` records an ordered, directed curve. It never stores a sample as its
geometry.

| P | Rule |
|---|---|
| **P1** | `NewPath` requires a finite start and at least one owned, non-nil concrete segment. It copies the segment values before validation. |
| **P2** | `LineTo` denotes the exact closed line segment from the current point to `End`. Equal endpoints are degenerate. |
| **P3** | `ArcThrough` denotes the oriented circle arc from the current point through `Through` to `End`. The points fix the carrier, sense, and minor/major choice. Repeated or collinear points are degenerate. |
| **P4** | A segment's end is the next segment's start by construction. No proximity weld, endpoint search, or snapping runs. |
| **P5** | An internal join is admitted by Sweep only when the two one-sided tangent rays are exactly codirectional in the structural path geometry. An ordinary position-continuous corner remains a valid `Path`, but Sweep stages it at S6. |
| **P6** | The initial path point lies exactly in the recorded profile plane, and the first tangent is exactly codirectional with the plane's positive normal. The profile need not contain that point. |
| **P7** | The initial frame is the profile's `(U, V, N)`. Lines translate it; arcs rotate it about their axes. Composition is rotation-minimizing transport. |
| **P8** | Zero twist adds no motion. Nonzero twist composes a rotation about the transported tangent by `total * length(prefix) / length(path)`. It never changes the path or section scale. |
| **P9** | A path ending at its start is closed. Closed-path Sweep is staged even when every join is tangent, because the transported end frame need not equal the start frame. No distributed correction is inferred. |

### 3.1 Exact path record

The private `Path` record stores the caller's points verbatim and stores each
segment's derived geometry over exact dyadic or rational arithmetic:

- a line stores its two recorded endpoints;
- an arc stores its three recorded points, exact unnormalised carrier plane,
  exact rational circumcenter and squared radius, and a certified directed
  sweep-angle interval;
- a derived float coordinate carries the outward-rounded gap from the exact
  value it denotes;
- a join tangent is compared by exact signs and products, never by an angular
  tolerance.

`Path.Start()` and `Path.End()` return recorded input coordinates, not computed
positions, so they are bare `r3.Vec` values under core §5.2. Computed carrier
points and transported coordinates remain bounded evaluator readings.

The path constructor validates its own spatial primitives. This does not
re-derive a `sketch` answer: the path is decad-owned geometry, and no upstream
package has classified it.

### 3.2 Rotation-minimizing transport

Let `q(t)` be the path and `(U(t), V(t), T(t))` its transported frame. The
section point whose initial world coordinate is `x` moves as:

```text
X(t, x) = q(t) + R(t) * (x - q(0))
```

where `R(0)` is identity and maps the initial frame onto the transported frame.
This keeps the source profile exactly at its authored plane when `t = 0`, even
when the path start lies outside the profile region.

On a line, `R` is constant. On an arc, `R` is the right-handed rotation about
the arc axis through the arc's swept angle. At a tangent join the two segment
motions have the same position and tangent frame. No Frenet frame is used:
Frenet normals are undefined on a line and flip at an inflection.

## 4. 3D sketches

`sketch.Sketch` is planar, and `r3` deliberately owns coordinates and rigid
motions but no shapes. Sweep therefore does not make either package own a
spatial curve. `Path` is decad's immutable spatial-wire input.

This split gives callers a parametric path now: their Go function computes the
`r3.Vec` values and calls `NewPath`, just as the same function supplies feature
dimensions. It also leaves a stable seam for a later constrained 3D sketch.

A future upstream 3D-sketch capability MUST export an ordered path snapshot
with all of these facts:

- exact segment order and direction;
- shared endpoint identity, not merely nearby endpoint coordinates;
- line and circular-arc structural data;
- a certified tangent verdict at each constrained tangent join;
- revision/staleness and ownership checks equivalent to the planar seam;
- a proven displacement whenever solved coordinates do not exactly realize a
  certified incidence.

When that type exists, decad may add an adapter that records it into `Path`.
The adapter may use an upstream tangent certificate to admit a join, but a
residual may only disprove that certificate. It may never accept a join because
two sampled tangents happen to be close. `Document.Sweep` and the private sweep
payload do not change.

This design does not add an unavailable type to the current dependency graph.
The initial feature consumes only `Path`, `r3`, `units`, and the existing planar
profile seam.

## 5. Table S — refusals

The existing existence rule applies: a requested solid that does not exist is
`ErrDegenerate`; a solid that exists but this evaluator cannot build is
`ErrUnsupported`.

| S | Condition | Sentinel | Permanent |
|---|---|---|---|
| **S1** | nil sketch, profile, or path; empty path; nil or foreign option | `ErrDegenerate` | yes |
| **S2** | profile seam rejection | seam sentinel | seam-owned |
| **S3** | non-finite path point, or arc construction overflows before it can state a finite carrier | `ErrNotFinite` for caller input; `ErrUnsupported` for derived range overflow | input rule is permanent; range ceiling is not |
| **S4** | zero-length line; repeated or collinear `ArcThrough` points | `ErrDegenerate` | yes |
| **S5** | path start is not on the profile plane, or its initial tangent is not codirectional with the positive plane normal | `ErrDegenerate` | yes for this operation's meaning |
| **S6** | an internal path join is not tangent, and the call does not carry `WithMitredJoins()` | `ErrUnsupported` | no; `WithMitredJoins()` defines the one admitted corner mode, and §16.4 owns what refuses it |
| **S7** | path is closed | `ErrUnsupported` | no; closed-frame holonomy is staged |
| **S8** | an arc span's rotation axis crosses the transported profile interior, or boundary contact fails Revolve's exact axis-incidence rule | `ErrDegenerate` | yes; the mapped boundary folds or pinches |
| **S9** | remote patches contact, or neighbours contact beyond their shared boundary | proved contact: `ErrDegenerate`; undecided budget: `ErrUnsupported` | contact is permanent; budget is not |
| **S10** | profile kind is unsupported by one of the path-span builders | `ErrUnsupported` | follows spline reach |
| **S11** | nonzero `WithSweepTwist` before the faceted-twist increment | `ErrUnsupported` | no |
| **S12** | option repeated, or a foreign type embeds the sealed marker | `ErrDegenerate` | yes; `WithSurfaceResult()` is exempt — a repeat is idempotent (docs/surface-design.md §4.1) |
| **S13** | a computed frame, vertex, measurement, or proof bound is non-finite | `ErrUnsupported` | no; numeric ceiling |
| **S14** | fixed facet, station, exact-predicate, or work budget is exhausted | `ErrUnsupported` | no; resource ceiling |
| **S15** | a constructed face, edge, or shell collapses from computed-coordinate rounding while the structural input is nondegenerate | `ErrUnsupported` | no; precision ceiling |

Gate order is normative:

1. Validate context, nils, owned options, and option arity.
2. Authenticate and record the planar profile.
3. Validate and snapshot the path.
4. Check initial placement and tangent joins.
5. Run every span's local line/extrude or arc/revolve existence gates.
6. Preflight the complete topology and audit budgets.
7. Build the candidate payload and run the global contact audit.
8. Compute all four body measurements and their bounds.
9. Commit once.

Every failure leaves document membership and producer numbering unchanged.

## 6. Span construction

The evaluator transports the one `ProfileRecord` through the path in order. A
line span is a straight prism interval. An arc span is a partial revolution of
the transported section about that arc's axis. The builders share their proven
walk, extent, axis-incidence, and measurement machinery with Extrude and
Revolve, but they emit no per-span cap faces.

For each span:

- the input section is the preceding span's exact terminal section record and
  transported frame;
- the output section is the same profile under the span's rigid motion;
- a line uses the path endpoints as its signed extent;
- an arc uses the path's exact center, axis, and directed angle;
- only the first and last sections become caps;
- an internal section becomes shared topology, never material and never a
  hidden coincident face pair.

An arc applies Revolve's half-plane and axis-contact audit to the transported
profile before construction. This is the local injectivity gate: every material
point must remain on one side of the rotation axis, except for an admitted pole
incidence. It prevents a section from folding through the center of curvature.

## 7. Global simplicity audit

Local span validity does not prove a composite sweep valid. A path can return
near an earlier span, and a large section can make two remote swept regions
overlap while the centerline remains simple.

The current composite evaluator uses a conservative separation audit. Adjacent
spans must have certified support on opposite sides of their shared section
plane. A span is supported there when its bound-inflated extent along the
plane's normal proves it on its side, or when its own construction makes its
endpoint cap a supporting plane (a prism, or an exact-turn arc of at most a
half turn) and the extent does not prove it past the plane: the rim pairing
already proves the two caps denote one section, so that extent reading only
falsifies. A placement's rounding therefore cannot refuse a composite sweep
whose spans touch only at their shared sections. Every non-neighbour pair must
have a strict separating gap between its bound-inflated axis-aligned boxes. A pair without either certificate returns
`ErrUnsupported`; the evaluator never infers disjointness from samples.

The shared-grid tessellation increment will replace this conservative subset
with an ephemeral certified facet cover of every lateral patch and both
endpoint caps. That audit will reuse shared boundary stations, source bounds,
exact triangle predicates, fixed work counters, and homotopy sign tests. It
will distinguish proven extra contact from an undecidable displacement band.

The final topology also runs the ordinary directed-edge, vertex-link, positive
face-area, outward-orientation, and positive-volume audits.

## 8. Table B — result

The result is one `sweepPayload` holding the recorded profile, path, transported
span frames, analytic span data, audit certificate, and four cached body
measurements.

For loop `i`, profile segment `j`, and path span `k`:

| Entity | Construction | Role |
|---|---|---|
| start cap | source recorded region in the original sketch plane | `capStart` |
| end cap | transported recorded region in the terminal frame | `capEnd` |
| lateral patch | profile segment `j` transported over path span `k` | `side(k,i,j)` |
| longitudinal edge | profile vertex transported over one path span | no public positional helper |
| internal section edge | transported profile segment at join `k` | no public positional helper |
| grid vertex | profile vertex at a path endpoint | no public positional helper |

Line-path patches have the same analytic surface variants as Extrude. Arc-path
patches have the same analytic surface variants as Revolve: plane, cylinder,
cone, sphere, torus, or `NURBSSurface` when spline reach admits it. A nonzero
twist increment instead stores certified flat facets and reports `Faceted`
surfaces; it never labels a twisted patch with an analytic surface it is not.

Internal section edges remain even when the two incident patches are tangent.
They name the change of path carrier and preserve `side(k,i,j)` provenance.
Adjacent patches merge only when their complete analytic carriers and trims are
the same and merging preserves every origin role.

The body has one lump and one outer shell. Profile holes become void passages
through the sweep; they do not create extra lumps. S9 rejects any mapping that
would change those claims.

**This is the solid's own shape alone.** `WithSurfaceResult()`
(docs/surface-design.md §2.2, §4) omits the two section caps that join a
holed profile's outer wall tube to its hole wall tube, so a surface-result
sweep's holed profile reports one `Lump` per tube instead — the identical
carve-out `docs/surface-design.md` §2.2 already states for the surface-result
prism and revolve builds, and Table D's own composite increment reuses it
unchanged rather than inventing a second rule.

## 9. Measurements and bounds

Each zero-twist span reuses the corresponding Extrude or Revolve closed forms,
with its two temporary section caps excluded. The body combines them as follows:

| Reading | Composition |
|---|---|
| `Volume` | sum of positive span volumes; the global audit proves span interiors disjoint |
| `Centroid` | volume-weighted sum of span first moments |
| `Area` | sum of lateral span areas plus the two endpoint cap areas |
| `Bounds` | componentwise union of every span's proven bounds |

Every span carries the displacement of its derived path carrier, transported
frame, computed stations, and accumulated rigid motion. A reading composes only
the terms that can move the geometry it reads. Bounds round outward at every
sum, product, quotient, and min/max publication.

An `Exact` result requires every contributing closed form to be exactly
representable and every displacement term to be zero. Operation history grants
no exactness. A single positive span bound makes the combined result
`Approximate` with the outward-rounded sum or maximum appropriate to that
quantity.

Nonzero twist uses the certified faceted payload's exact-rational tetrahedron
sum and the same boundary-displacement, area-slack, and occupied-volume proof
discipline as Loft. It never sums untwisted analytic span formulas.

The body tolerance reference reads a proven lower bound on the true diameter.
It starts from the complete held audit vertex set and subtracts twice the
payload's maximum positional displacement, rounded down. A non-positive result
withholds the reference and prevents a false `Sound` report.

## 10. Table D — downstream

| D | Consumer | Status |
|---|---|---|
| **D1** | structural `Verify` + tolerance gate | lands with Sweep. For a solid, the construction and global audit prove validity and all four readings are judged. For a `WithSurfaceResult()` sheet, only the one-span straight reduction carries an equivalent construction proof (it IS a prismPayload build, docs/surface-design.md §9.1); the arc reduction and every composite sheet read `Suspect` — the arc build runs no crossing audit of its own, and the composite build's own boundary/vertex-link audit proves assembled topology, not geometric non-self-intersection |
| **D2** | `Tessellate` / STL / OBJ | a one-span straight solid sweep (no spans, no arc, no surface result) tessellates as the prism it reduced to: its payload's `prism` is the `prismPayload` `evalPrismContext` built, so the prism path chords it and its mesh, `Bound` and every proof are that prism's, with each wall read under the span-prefixed role `side(0,i,j)` that `prefixSweepSpanZeroRole` minted (`tessellate.go`). The arc reduction, every composite path and every surface result stay staged until the shared-span tessellator publishes complete source, area, and boundary proofs |
| **D3** | mesh booleans | the one-span straight solid sweep is an operand: its D2 mesh carries the prism's `volSymDiff` with `symDiffOK == true`. Every other sweep stays staged until D2 also publishes that proof for it |
| **D4** | interference | bounds-disjoint pairs work immediately, and the one-span straight solid sweep reaches the mesh intersection D3 admits. Other pairs stay `Suspect` until D3 or a sweep analytic adapter lands |
| **D5** | clearance | box separation may settle the partition, but `WithClearances` stays `Suspect` until a sweep boundary adapter lands |
| **D6** | `Wall`, `Undercut`, `ConcaveRadius` | `Unavailable` with `DiagUnsupportedSurveyPayload` until non-constant-section proofs land |
| **D7** | `Placed`, `Duplicate`, `PlacedCopy` | re-evaluates the payload under the composed rigid motion and reruns the global audit; every displacement and measurement is recomputed |
| **D8** | modify operations | `ErrUnsupported`; Table R in `docs/modify-design.md` has no `sweepPayload` receiver row |

The tessellator shares one profile station chain across adjacent path spans and
one path station chain across adjacent profile patches. It never builds each
span independently and welds nearby endpoints. Join vertices are shared by
construction.

`sourceBound(face)` contains profile chording, path chording, frame
construction, stored-coordinate, twist, and placement displacement applicable
to that face. `Mesh.Bound` is their maximum. `areaSlack` integrates local
true-vs-held area-density error without cancellation. `volSymDiff` sums the
per-span occupied-volume homotopies; it never substitutes a generic
`delta * area` shortcut.

## 11. Determinism, cancellation, and budgets

Equal profile record, path record, options, and document placement produce the
same topology, roles, measurements, and tessellation order.

`Sweep` checks cancellation at every phase boundary and through path
derivation, station generation, facet-pair classification, exact predicates,
and measurement accumulation. It returns `ctx.Err()` unchanged.

The evaluator uses the existing per-walk chord cap, mesh facet cap, cumulative
facet-work cap, and facet-pair-test cap. It adds one sweep-span cap so topology
and role counts are preflighted before allocation. The defining source constant,
not this design document, owns its numeric value and derivation.

No budget resets per span. Checked arithmetic computes the complete build's
candidate counts before any large allocation or audit.

## 12. Increments

The current evaluator implements the first four increments, with composite
paths admitted when every transported frame is exact and the conservative §7
separation audit closes. Other composite paths remain staged as
`ErrUnsupported`.

| PR | Lands | Still staged |
|---|---|---|
| **1** | `Path`, line/arc recording, exact join classification, public Sweep signatures and option validation | every build returns `ErrUnsupported` |
| **2** | zero-twist one-span line reduction, topology and all four measurements | arc spans, composite paths, downstream beyond D1 |
| **3** | zero-twist one-span arc reduction with Revolve's local gates | composite paths |
| **4** | composite tangent line/arc transport, internal-section topology, global contact audit, D1 and D7 | D2–D6, nonzero twist |
| **5** | shared-grid tessellation and complete proof record; D2 and D3 | analytic clearance and surveys |
| **6** | Tier A free-form profile reach supported by each span builder | Tier B/C profile kinds follow spline staging |
| **7** | nonzero distributed twist as a certified faceted sweep | closed paths, a corner mode or scale other than §16's |
| **8** | sweep boundary adapter for clearance/interference | non-constant-section surveys |
| **M1**, **M2** | §16's mitred polyline sweep; §16.9 owns the two rows | what §16.9 lists |

Each published feature increment lands its four measurements and structural
verification together. An increment never returns a body whose cached reading
is a placeholder.

## 13. Required tests

Every test asserts computed geometry or a proof bound.

- Build a straight square sweep and compare every vertex, face role, volume,
  area, centroid, and bounds with the equivalent Extrude.
- Build a quarter-circle path and compare the same readings with the equivalent
  partial Revolve.
- Build tangent line-arc-line and two-plane arc-arc paths. Assert transported
  frames and join sections, not only face counts.
- Reject an off-plane path start, a reversed initial tangent, every degenerate
  path segment, a non-tangent join, and a closed path with the stated sentinel.
- Put the profile across an arc axis and assert S8 before any document commit.
- Make a remote path span cross an earlier span. Assert S9 and the specific
  non-neighbouring patches the audit classified.
- Put two remote spans just outside and just inside the summed audit bounds.
  Assert deterministic refinement, acceptance only with a separating proof,
  and `ErrUnsupported` in the undecidable band.
- Assert every edge has two incident faces, every vertex link is one cycle, the
  shell is outward, and internal section faces do not exist.
- Assert holes remain void passages and contribute with the correct sign to
  volume and centroid.
- Assert a non-axis-aligned path produces bounds that enclose dense falsifier
  samples. Samples never replace the proof.
- Assert exactness changes only when a published bound changes from zero.
- Assert a failed and canceled call leaves live bodies, order, and next producer
  identity unchanged.
- Assert repeated construction and replay produce bit-identical roles,
  measurements, and mesh order.
- For tessellation, assert shared indices at every span join, positive facets,
  outward winding, complete source-face mapping, `Bound`, `areaSlack`, and
  `volSymDiff` against independent references.
- For a future 3D-sketch adapter, corrupt each upstream certificate separately.
  A large residual rejects it; a small residual never grants admission.

## 14. Companion edits

Landing this design makes these contract edits:

- `docs/api-design.md` §8 adds `Sweep` and points here for its signature and
  path contract;
- `docs/api-design.md` §13 removes Sweep from the non-goals list;
- `docs/layout/design-docs.md` lists this document;
- `docs/evaluator-design.md` §11 points to this document's staged delivery;
- the implementation increment that adds `sweepPayload` adds its row to
  tessellation and payload-verification tables in the same change;
- `doc.go` changes only when an implementation increment changes current
  evaluator support;
- `docs/surface-design.md` §1.2, §13.5, Table R and Table D name §15 as the
  owner of `SweepChain`'s pairing rule, and its §15 carries §15.7's test rows.

No current dependency exposes a 3D sketch. This design consumes none. A future
adapter is a separate design change after an upstream spatial-path snapshot and
its certification contract exist.

## 15. `SweepChain` — sweeping an open sketch chain

`docs/surface-design.md` §13 owns the open chain itself: what `sketch.Chain`
publishes, what `ChainRecord` records, the gates `RecordChain` runs, the four
seam sentinels every chain-fed entry point reuses, and Table G's per-segment
wall set. That document left exactly one question here (§13.5) — whether this
document's composite join topology states a pairing rule for an open walk.

**It does.** §15.1 states why the rule survives, §15.2 states which paths need
no rule at all, and §15.3 through §15.7 state the entry point, the refusals,
the result and the staging that follow.

### 15.1 The join pairs by recorded-segment index, and the loop is not what makes that true

**A composite sweep pairs two adjacent spans' rim edges by POSITION in the
recorded section.** It reads no winding, no closure, no coordinate and no
proximity. The two sections meeting at a join are the ONE recorded section
under two rigid motions, so element `i` of the first denotes the same recorded
generator as element `i` of the second. That identity is the whole content of
the pairing, and §8's internal section edge is what it produces.

**A `ChainRecord` states that index exactly as a `LoopRecord` does.**
`ChainRecord.Segments` is an ordered list over the same ten sealed
`CurveSegment` variants, in `sketch`'s own published walk order
(`docs/surface-design.md` §13.1, §13.3). Two adjacent chain spans transport
that one list, so segment `j`'s terminal rim edge on span `k` pairs with
segment `j`'s initial rim edge on span `k+1`. The rule needs no new proof
because it consumes no new fact: a shared record, not a shared winding, is
what makes index `i` mean one thing on both sides.

**What the loop supplies and the walk does not is the WRAP, and dropping it
mints free ends rather than breaking the pairing.** A closed section of `n`
segments carries `n` rim posts, each shared by the two walls that meet there;
an open walk of `n` segments carries `n+1`, of which post `0` and post `n` are
touched by one wall each. So a chain join sews `n` rim edges and `n+1` rim
vertices where a profile join sews `n` and `n`, by the same index in both
lists. `buildChainSides` (`extrude.go`) already mints that post set for one
span, and `docs/surface-design.md` §13.3 states the same drop for a chain
revolve's own two free ends.

**What the loop supplies and the walk does not is a CAP FACE, and that is an
implementation seam rather than a proof.** The composite build reads each
span's two rim lists off that span's `capStart` and `capEnd` face loops and
refuses a span carrying neither. A chain-fed span mints no cap at all
(`docs/surface-design.md` §13.4), so the increment that builds one reads its
two rim lists off the span's WALL faces instead — wall `j`'s own start rim
edge and end rim edge, in recorded-segment order. That is the identical index,
reached through the face an open walk does have.

**The separation certificate carries over unchanged.** §7's adjacent-span rule
requires each span's material to be certified on its own side of the shared
section plane. It reads the span's analytic extent along that plane's normal
and the span's own endpoint-supporting-plane property, and neither reading asks
whether the section closes: a chain prism span supplies both through
`chainPayload.prism`'s `prismPayload` view, and a chain arc span through
`chainRevolvePayload.revolve`'s `revolvePayload` view. The non-neighbour rule
is a bounded-box separation and reads no section at all. The topology audit
that follows the sew already admits a free rim edge — it requires one or two
incident faces per edge and a vertex link that is one cycle or one path — so a
chain span's own free rims pass it for the reason a surface result's two
omitted outer caps already do.

**What the join does NOT inherit is a solid.** Every profile-fed span builds
closed and the two outer caps are dropped afterwards, which is why a
surface-result composite sweep still carries two cap roles through the pairing.
Every chain span publishes `BodySheet` from its own first face, so the
increment that builds a composite chain sweep pairs two sheets at every join
and never a temporary solid.

### 15.2 A one-span chain sweep states no join at all

A path of one span has no internal join, so §15.1's rule has nothing to
decide there. The whole build is the recorded walk transported over that one
span:

- a **`LineTo`** span is the chain prism `Document.ExtrudeChain` already
  builds, over the path's own signed height instead of a resolved `Extent`;
- an **`ArcThrough`** span is the chain shell `Document.RevolveChain` already
  builds, over the arc's exact centre, axis and directed angle instead of a
  resolved `AngularExtent`.

Both constructions are landed (`docs/surface-design.md` Table D row 6), so a
one-span `SweepChain` was never waiting on a pairing rule. It was waiting on
this section to say so, and on Table SC to state which path shapes it admits.

**The two reductions are not the same call as their chain siblings, and the
difference is the height.** `ExtrudeChain` takes its interval from an `Extent`
the document resolves; a straight chain sweep takes it from the path's two
recorded points, so the interval carries `validateStraightSweepPath`'s own
composed bound — the square root's committed error against the exact rational
squared length, plus the per-component departure of the held sweep vector from
the exact tangent. On an axis-aligned frame every one of those terms is zero
and the two calls publish bit-identical measurements; on any other frame the
chain sweep's `Area` is `Approximate` where the same walk's `ExtrudeChain`
reading may not be.

### 15.3 The entry point

```go
func (d *Document) SweepChain(ctx context.Context, s *sketch.Sketch,
    ch *sketch.Chain, path *Path, opts ...ChainSweepOption) (*Body, error)

// ChainSweepOption configures SweepChain. It is its own sealed tier.
type ChainSweepOption interface { /* sealed */ }
```

**`ChainSweepOption` is a sealed tier of its own rather than `SweepOption`**,
for the reason `docs/surface-design.md` §13.2 states for `ChainExtrudeOption`
and `ChainRevolveOption`: `WithSurfaceResult()` must not compile against a
chain-fed call. A chain sweep is always a sheet — an open walk encloses no
region, so there is no cap to omit and no solid to ask for — and accepting the
option as a no-op would give one option two meanings. `WithSweepTwist` is not a
member either: a nonzero twist is S11 for a profile-fed sweep, and a chain
inherits that staging rather than a second spelling of it. The tier carries no
member in this increment; it exists so a later chain-only option has one to
land on. The rejected alternative is the `SweepOption` tier the staged
signature first landed with, which type-checks `WithSurfaceResult()` against a
call that can never honour it.

The call takes `ctx` because `Document.Sweep` does, and takes the sketch beside
the chain because a chain's geometry is plane-local and the plane is the
sketch's (`docs/api-design.md` §7).

**A chain-fed result is always a sheet**, which is `docs/surface-design.md`
§13.2's answer for `ExtrudeChain` and `RevolveChain` restated with no change:
`Kind()` is `BodySheet` by construction, `IsSolid()` is `false`, and
`Volume()`/`Centroid()` answer `ErrNotSolid` by KIND rather than by soundness.

### 15.4 Table SC — what refuses a chain sweep

Every row lands on `docs/api-design.md` §12's existing vocabulary, and no row
mints a sentinel of its own. Gate order is §5's, with its step 2 reading a
chain through `RecordChain` instead of a profile through `RecordProfile`.

| SC | Condition | Sentinel |
|---|---|---|
| **SC1** | a nil document, context, sketch, chain or path; an empty path; a nil or foreign option | `ErrDegenerate` |
| **SC2** | the chain fails one of `docs/surface-design.md` §13.3's gates | the seam's own: `ErrForeignProfile` / `ErrStaleProfile` / `ErrInvalidProfile` / `ErrUnrecordableProfile` |
| **SC3** | the path start is not in the sketch plane, or its initial tangent is not codirectional with that plane's positive normal | `ErrDegenerate`, S5's rule over a chain's own recorded plane |
| **SC4** | every path condition S3, S4, S7 and S13 through S15 already state — a non-finite point, a degenerate segment, a closed path, a non-finite derived bound, an exhausted budget, a collapse from computed-coordinate rounding | those rows' own sentinels, unchanged |
| **SC5** | an internal path join is not tangent | `ErrUnsupported`, S6 |
| **SC6** | a recorded chain segment outside the line, circle and arc set the span builders admit | `ErrUnsupported`, S10 |
| **SC7** | a composite path, before the increment that builds §15.1's join | `ErrUnsupported` |
| **SC8** | an arc span the chain shell refuses — a walk that does not lie in one closed half-plane of the arc axis, both free ends on that axis, or an interior on-axis junction | exactly what `RevolveChain` answers: `ErrDegenerate`, or `ErrUnsupported` under `docs/surface-design.md` R22 and R33 |
| **SC9** | an arc span, before the increment that builds §15.6's C2 reduction | `ErrUnsupported`, `docs/surface-design.md` R34 |

**SC2 is decided before SC3, and that order is load-bearing.** A foreign,
stale, invalid or unrecordable chain names a repair the caller makes in the
sketch, and reporting a path refusal first would hide it behind geometry the
caller cannot act on. It is also what keeps a staged `ErrUnsupported` from
masking a seam refusal, which is the order the staged signature already runs.

### 15.5 What a chain sweep publishes

| Reading | A chain sweep |
|---|---|
| `Kind()` | `BodySheet`, always and by construction — a decided enum carrying no bound |
| `IsSolid()` | `false`, always |
| `Volume()`, `Centroid()` | `ErrNotSolid`, by kind |
| faces | one wall per recorded segment per path span, after the rim coalescing `Extrude` already runs — `docs/surface-design.md` Table G's own per-kind construction, never a cap and never a closing face |
| `Edges(Free())` | both rims of the walk over every span, plus the one sweep edge at each of the walk's two free ends; a one-span straight sweep of an `n`-segment walk resolves `2n + 2` |
| `Area` | the sum of each wall's own area through `boundedAdd`. A straight span's wall is `boundedMul(segment length, span height)`, the height carrying §15.2's composed path bound; an arc span's is `boundedMul(walkAxisMoment, sweep)` on `docs/surface-design.md` §13.4's own terms. A circular wall carries its proven integral enclosure and a free-form wall `internal/freeform/spline_length.go`'s proven bracket |
| `Bounds` | the recorded walk's per-segment analytic extremes swept over the span's signed interval, from `prism_extent.go` and `revolve_extent.go` verbatim, charging the frame and placement rounding those readings already charge |
| `Verify`'s validity | exactly what an `ExtrudeChain` ribbon or a `RevolveChain` shell over the same recorded walk and interval reads (`docs/surface-design.md` §13.4, §9.1). A composite chain sweep reads `ValidityUndecided`, for D1's own reason: the assembled topology audit proves how the spans sew together, never that the swept walls do not fold back through one another |

No bound in that table is new, because no geometry is: every wall is built by
the per-segment construction its chain sibling already proves, and the one term
a chain sweep adds that neither sibling has is §15.2's path height bound.

### 15.6 Increments

| PR | Lands | Still staged |
|---|---|---|
| **C1** | `Document.SweepChain` over a one-span `LineTo` path: the sealed `ChainSweepOption` tier, Table SC rows SC1 through SC4, SC6 and SC7, the chain prism reduction, and the four readings §15.5 states for it | the arc reduction, every composite path, D2 through D8 |
| **C2** | the one-span `ArcThrough` reduction with the chain shell's own axis gates (SC8) | every composite path |
| **C3** | §15.1's composite join over a tangent line/arc path: the wall-face rim lists, the sew, the separation certificate over chain spans, and the assembled boundary audit | D2 through D8 for a chain sweep |

Every downstream row of Table D reads a chain sweep exactly as it reads a
profile-fed one, and each stays staged on its own terms: a chain sweep's mesh
is `ErrUnsupported` until D2 lands, and the modify operations have no receiver
row for it (D8).

### 15.7 Required tests

Each row asserts on computed geometry, and each bound assertion is shown to
fail before it is trusted. `docs/surface-design.md` §15 carries the rows
themselves, as T190 onward, so the obligations sit beside every other
chain-fed obligation rather than in two places.

## 16. Mitred polyline sweep with a per-span section scale

A branch of a 3D-print tree support is one straight segment after another,
each leaning a different way, thick at the build plate and thin at the tip.
The consumer that states this case (`mtilt`, its tree-support generator)
needs one such branch to be ONE solid body, built without a boolean per
segment, and needs two branches to union where they merge. This section
admits that body: a `LineSeg`-only profile moved along a `LineTo`-only path
whose joins are corners, cut on a join plane at every corner, with the
section scaled by a stated factor across every span. Every vertex the build
holds is an exact rational, rounded once for publication, so every wall is an
exactly planar quad and the body's volume is `Exact`.

### 16.1 Scope

The build admits:

- a path of one or more `LineTo` segments, with `WithMitredJoins()` when it
  has more than one; the joins may be corners or straight continuations;
- a profile whose every segment, on the outer loop and on every hole, is a
  `LineSeg`;
- `WithSectionScale` with one positive factor per segment, or no scale;
- zero twist, and a solid result.

It refuses, with Table SM's rows, an `ArcThrough` segment, a curved or
free-form profile segment, `WithSurfaceResult()`, a nonzero twist and a closed
path. A curved profile segment's image under a span's wall lines is a conic
section, which no admitted surface variant holds exactly; an arc span has no
join plane this construction can state. Neither is permanent.

### 16.2 The options

```go
func WithMitredJoins() SweepOption
func WithSectionScale(factors ...units.Value) SweepOption
```

Each is accepted at most once (S12). `WithSectionScale` takes exactly
`len(path.Segments())` factors, in path order; `factors[k]` is the ratio of
the section at the end of span `k` to the AUTHORED profile, so a tree branch
with radius `r_k` at path point `k` passes `r_k / r_0`. A factor is a
`units.Value` of kind `Dimensionless` (`units.Scalar`): another kind is
`ErrUnitKind`, a non-finite value is `ErrNotFinite`, a factor at or below zero
is `ErrDegenerate`, and a count other than the segment count is
`ErrDegenerate`. An omitted `WithSectionScale` means every factor is `1`.
`WithSectionScale` on a one-span path needs no `WithMitredJoins()`: it builds
a polygonal frustum. On a path of two or more spans it requires
`WithMitredJoins()` (SM4), because the tangent-join composite builder of §6
transports a rigid section and has no scale.

### 16.3 Construction

Write the path points `V_0 … V_N` (`N` spans), `d_k = V_{k+1} − V_k` for
span `k`, the factors `f_1 … f_N` with `f_0 = 1`, and the span ratio
`ρ_k = f_{k+1} / f_k`. Every quantity below is an exact rational over the
recorded floats; nothing is normalised and no square root is taken.

**Section planes.** Every section lies on a plane through its path point:

| Plane | Through | Normal |
|---|---|---|
| `Π_0` | `V_0` | the recorded profile plane's own (P6: `V_0` lies on it, `d_0` is codirectional with its positive normal) |
| `Σ_k`, `0 < k < N` | `V_k` | `m_k = λ_k · d_{k−1} + λ_{k−1} · d_k`, where `λ_j` is the largest `float64` whose square does not exceed the exact `‖d_j‖²` (`internal/proof`'s `DySqrtDown`): the lower end of a certified enclosure at `float64`'s fixed precision |
| `Π_N` | `V_N` | `d_{N−1}` |

`m_k` is a rational vector that the two span directions, weighted by near
lengths, add to: it is the bisector direction up to the enclosure's width,
and the body DENOTES the plane with normal `m_k`, not the exact bisector. No
reading depends on the join plane being the exact bisector; what every
reading depends on is that both spans meeting at `V_k` are cut on the SAME
plane, which they are by construction. Two consecutive spans that are
exactly reversed (`d_{k−1} × d_k = 0` and `d_{k−1} · d_k < 0`, both decided
exactly) have no join plane and are SM5.

**Sections.** `P_0` is the recorded profile: each recorded point `(u, v)` lifts
to `O + u·U + v·V` over the plane record's floats, held as the exact rational
that sum is. For `k ≥ 1`, `P_k` is the image of `P_{k−1}` under span `k−1`'s
wall lines, below. The hole loops map the same way as the outer loop, so a
holed profile sweeps to a tube with void passages.

**Wall lines.** For a vertex `p` of `P_k`, span `k`'s wall line through `p`
runs along

```text
w_k(p) = d_k + (ρ_k − 1) · (p − V_k)
```

and `p`'s image on the next plane (`Σ_{k+1}`, or `Π_N` on the last span) is
`p' = p + s · w_k(p)` with `s` the unique solution of that plane's equation.
`w_k(p) · n = 0` for the next plane's normal `n`, `s ≤ 0`, or, when
`ρ_k < 1`, `s · (1 − ρ_k) ≥ 1` is SM6: the wall line misses the join plane,
meets it behind the section, or meets it at or past the span's apex. SM6's
span-level arm runs first: `d_k` must have a positive dot product with both
its start and its end plane normals. The first sign is every wall line's
own, since `w_k(p)` differs from `d_k` by a vector in the start plane; the
second puts the apex beyond the end plane, so a wall line that would meet
that plane coming back is exactly one that meets it past the apex.

**Why every wall is one exact plane.** For `ρ_k ≠ 1`, every wall line of span
`k` passes through one point, the apex `F_k = V_k + d_k / (1 − ρ_k)`; for
`ρ_k = 1`, every wall line is parallel to `d_k`. Two lines through one point,
or two parallel lines, span one plane, so the quad `p, q, q', p'` over
profile segment `p → q` is exactly planar, whatever planes its two ends lie
on. That is what lets a join plane tilt freely: the mitre changes where the
quad ends, never whether it is flat. The SM6 bound `s · (1 − ρ_k) < 1` keeps
every image on the near side of the plane through `F_k` parallel to the next
section plane, so the map from `P_k` to `P_{k+1}` is a projective map
restricted to one half-space, which carries a simple polygon to a simple
polygon and preserves its vertex order.

**Watertightness.** `P_k`'s vertices are span `k−1`'s end and span `k`'s
start, by identity rather than by weld; the two caps are `P_0`'s and `P_N`'s
regions. Every edge therefore bounds exactly two faces (Table BM), and the
assembled shell is closed by construction. What construction does not prove
is that no two remote faces cross — a path that bends back through its own
earlier span, or a wide section on a tight corner, can make two walls
intersect while every local gate passes — so SM8's crossing audit runs over
the assembled triangle set before commit, exactly as `docs/loft-design.md`
§6 runs it over a loft (`loftCrossingAudit`'s generic entry: sweep-and-prune
over the triangles' boxes, every candidate tested pairwise, and S8 over the
candidate count against `maxFacetPairTestsPerCall`). SM10's `F·(F−1)/2`
preflight against the same ceiling runs first.

**Rounding, once.** Every held vertex is the exact rational rounded to the
nearest `float64` per coordinate. `delta` is the largest 3D distance, over
every vertex, between the rational and its rounding, read exactly and
rounded up (the `PointRoundBound` reading `internal/meshbool` already
performs for an exact point). A rounding that collapses a held triangle the
rationals keep is S15, `ErrUnsupported`. `delta` is what every vertex
`Position()` carries; the mesh carries each vertex's own gap, read the same
way, and `delta` as its `Bound` (DM2); the four body measurements read the
rationals and carry nothing from it (§16.6). The rational vertices stay in the payload: a
placement re-reads them (DM7), and a later exact consumer may read them
without rounding.

**Orientation.** Seed each cap's winding from the recorded loop sense as
`docs/loft-design.md` §5 does, then orient the whole shell once by the sign
of the exact tetrahedron sum over the rational vertices; a negative sum
reverses every triangle. Never orient a wall or a cap on its own.

### 16.4 Table SM — what refuses a mitred polyline sweep

Every row lands on `docs/api-design.md` §12's vocabulary. Gate order is
§5's, with step 5 running SM5 through SM7 per span in path order and step 7
running SM8 in place of §7's separation audit.

| SM | Condition | Sentinel |
|---|---|---|
| **SM1** | `WithMitredJoins()` or `WithSectionScale` with an `ArcThrough` segment | `ErrUnsupported` |
| **SM2** | a profile segment, on any loop, that is not a `LineSeg`, or a `LineSeg` trimmed at a cut parameter, under either option | `ErrUnsupported` |
| **SM3** | a factor of the wrong kind; a non-finite factor; a factor at or below zero; a factor count other than the segment count | `ErrUnitKind`; `ErrNotFinite`; `ErrDegenerate`; `ErrDegenerate` |
| **SM4** | `WithSectionScale` on a path of two or more spans without `WithMitredJoins()` | `ErrUnsupported` |
| **SM5** | two consecutive spans exactly reversed | `ErrDegenerate` |
| **SM6** | a span that does not leave its start plane or reach its end plane forward; a wall line parallel to its end plane, meeting it at or behind its start, or at or past the span's apex | `ErrDegenerate` |
| **SM7** | a wall quad whose exact area is zero, or a section with two coincident vertices | `ErrDegenerate` |
| **SM8** | the crossing audit proves two non-adjacent faces cross; its budget runs out first | `ErrDegenerate`; `ErrUnsupported` (S9's split) |
| **SM9** | `WithSurfaceResult()`, a nonzero `WithSweepTwist`, or a closed path, with either option | `ErrUnsupported` |
| **SM10** | `F·(F−1)/2` over Table BM's `F` exceeds `maxFacetPairTestsPerCall`, or the span count exceeds §11's span cap | `ErrUnsupported` (S14) |

SM2's trimmed-line arm keeps every section vertex a recorded point: a cut
parameter's point is a computed one, and the polygon through it is not the
recorded region exactly. SM7 is unreachable once SM6 passes, because §16.3's
map is a projective map on a convex region holding the section, which keeps
every vertex distinct and every wall quad simple; the build keeps the check.

SM6 names the requested solid's own failure: a join plane that a section's
wall line cannot reach before the apex asks for a frustum cut past its own
point, which is no solid. A caller meets it with a wide section on a sharp
bend and a strong taper; the repair is a longer span, a smaller section, or a
weaker taper, and the error names the span and the section vertex.

### 16.5 Table BM — the result

For loop `i` (`0` the outer loop, `1 + h` for hole `h`), profile segment `j`
of that loop, and span `k`:

| Entity | Count | Surface | Role |
|---|---|---|---|
| start cap | 1 | `Plane` over `P_0`'s region | `capStart` |
| end cap | 1 | `Plane` over `P_N`'s region | `capEnd` |
| wall | one per `(k, i, j)` | `Plane`, a four-vertex loop `p, q, q', p'` | `side(k,i,j)` |
| longitudinal edge | one per span per profile vertex | — | through its two walls' origins |
| section edge | one per profile segment per section `P_0 … P_N` | — | through its incident faces' origins |

The roles are §8's own grammar, with `k` the span, unchanged: a mitred sweep
adds no index. With `S` the profile's total segment count over every loop,
the body has `2 + N·S` faces, `(N + 1)·S` vertices and `(2N + 1)·S` edges,
and `F = 2·N·S + capTriangles` held triangles, each wall split along the
fixed diagonal `tessellate.go` uses for a prism's lateral quad. `capStart` is
triangulated by `triangulate.go`, and `capEnd` carries the same index triples
on `P_N`: §16.3's projective map carries a triangulation of `P_0` to one of
`P_N`. One lump, one outer shell; a hole loop is a
void passage and never a second lump.

A wall is ONE face, not a loft's two triangles: `docs/loft-design.md` §5
splits a loft wall because its quad is not planar in general, and §16.3
proves every quad here planar exactly. The wall's `Plane` is the exact plane
through its rational quad; the `Frame` it publishes is that plane's float
rounding, within `delta`, which is the standing loft §5 already states for a
`Plane` whose vertices carry `delta`.

`Edge.IsConvex` keeps evaluator §3's meanings: a longitudinal or section edge
is a junction edge decided by `orient3d` over its two incident walls' apex
vertices, exactly as loft §5 decides a rung; a cap rim takes the rim rule —
outer convex, hole concave.

### 16.6 Measurements and bounds

All four readings are taken over the exact rational vertex set, never the
rounded one.

| Reading | Value | Exactness and bound |
|---|---|---|
| `Volume` | the exact tetrahedron sum over every held triangle, anchored at `V_0`, published as that rational's nearest float | `Exact` when that float is the rational; otherwise `Approximate` with that one rounding, read exactly, as its bound. No other term enters it |
| `Centroid` | the exact first-moment sum divided by the exact volume, each coordinate published as its nearest float | the published float's own rounding, read exactly; zero when every coordinate is already a float |
| `Area` | each triangle's area is the square root of an exact rational, enclosed by `ratSqrtUp` and its lower twin; the nearest float to the midpoint of the summed enclosure | `Approximate`, the larger exact gap to either end of the enclosure, rounded up; `Exact` only when the enclosure closes on a float |
| `Bounds` | per-axis minimum and maximum over the rational vertices, rounded outward | the largest outward rounding, read exactly: the box encloses the exact body and exceeds its extremes by at most one rounding; `Exact` when every extreme is a float |

The tolerance-gate diameter is §9's: the held vertex set's own diameter less
twice `delta`, rounded down.

### 16.7 Table DM — downstream

| DM | Consumer | Status |
|---|---|---|
| **DM1** | structural `Verify` + tolerance gate | lands with M1. The construction, SM6/SM7 per span and SM8's audit prove validity, and all four readings are judged |
| **DM2** | `Tessellate` / STL / OBJ / 3MF / faceted STEP | lands with M1 as an exact restatement: the held triangles of Table BM at any tolerance at or above `delta` (below it, `ErrUnsupported`, `docs/tessellation-design.md` §7's rule), each vertex's own rounding gap as its `vertexBound` and `sourceBound(face)` the largest of them over that face's triangles (`docs/faceted-vertex-bounds-design.md` §2.1), with `Bound` `delta`, `areaSlack` the per-triangle perturbation at `delta` (`perturbedTriangleAreaAllow`, the `stitchPayload` row's term), `volSymDiff = sweptVolumeAllow(delta, area upper)` with `symDiffOK == true`, and the boundary proof the build's own SM8 certificate. The implementing PR adds this payload's row to `docs/tessellation-design.md` §2's table |
| **DM3** | mesh booleans | lands with M2: an operand on `docs/tessellation-design.md` §11's terms. Two branches that cross carry `delta > 0` on both sides, so the hidden-tangency gate runs with slack `2·delta` and admits the pair only through a proven deep witness, which `boolean.go`'s witness search finds by sampling along each contact segment. Walls that are coplanar with the other operand's, as two branches sharing a span are, stay refused by that gate. A `Cut` whose tool is a mitred sweep wholly inside the target, such as a thinner bore through a trunk's corners that stops short of both caps, publishes the bore as a separate void shell (`docs/api-design.md` §6), and a later `Union` whose operand stays clear of the bore keeps that shell whole. A holed profile's passage cannot be capped into a void by a `Union` with end plugs, because a plug flush with the tube's cap is a coplanar contact this gate refuses |
| **DM4** | interference | lands with M2: the partition through the exact planar arm of `docs/interference-design.md` §3.2 against a prism, a stitched solid, a faceted result or another mitred sweep, the overlap volume through DM3's mesh path |
| **DM5** | clearance | box separation at once; `WithClearances` reads the exact planar arm's gap, the held gap widened by `delta`, against the partners DM4 names, and stays `Suspect` against every other payload |
| **DM6** | `Wall`, `Undercut`, `ConcaveRadius` | `Unavailable` with `DiagUnsupportedSurveyPayload`, as D6 |
| **DM7** | `Placed`, `Duplicate`, `PlacedCopy` | re-runs §16.3 from the record under the composed motion, applied to the rational vertices exactly (a transform's entries are floats, hence rationals), rounds once, recomputes `delta`, re-decides the shell orientation and re-runs SM8; `delta` never accumulates across placements |
| **DM8** | modify operations | `ErrUnsupported`, as D8 |

### 16.8 Determinism, cancellation, and budgets

Equal profile record, path record, factors and placement produce the same
rational vertices, roles, measurements and triangle order; the `λ_j`
enclosures are read at one fixed precision, so no step depends on a
transcendental library or on FMA contraction. The build polls `ctx` per span
and inside SM8's pair loop, and returns `ctx.Err()` unchanged. §11's span cap
bounds `N`, and SM10's preflight bounds `F` before any pair test runs; the
rational vertices' bit length grows with the span index, since each section
is a rational image of the one before it, and the span cap is what bounds
that growth.

### 16.9 Increments

| PR | Lands | Still staged |
|---|---|---|
| **M1** | `WithMitredJoins()` and `WithSectionScale` over `LineTo`-only paths and `LineSeg`-only profiles: §16.3's construction, Table SM, Table BM, §16.6's four readings, DM1, DM2, DM7, the executable example, and the `docs/tessellation-design.md` payload row | DM3 and DM4 until the witness fix lands; `WithSurfaceResult()`, arc spans, curved profile segments |
| **M2** | DM3 and DM4 after `boolean.go`'s segment-anchored witness search lands: the union-of-two-branches test row | `WithSurfaceResult()`, arc spans, curved profile segments |

### 16.10 Required tests

Every row asserts computed geometry or a proof bound, and every bound leg is
deleted once and watched fail before it is trusted.

- A square profile of side `a` centred on the path, swept along an L path of
  spans `L1` and `L2` meeting at a right angle, no scale: `Volume` is `Exact`
  and equals `a²·(L1 + L2)`; the join section's four rational vertices lie
  on the join plane and span a rectangle of sides `a` and `a·√2` within the
  published bounds; the body has `2 + 8` faces, `12` vertices and `20` edges,
  and every edge has two incident faces.
- A one-span square sweep with factor `f`: `Volume` is `Exact` and equals
  `L·a²·(1 + f + f²)/3`; every `capEnd` vertex sits at `f` times its start
  vertex's in-plane offset.
- A three-span zigzag with a regular 16-gon and factors `r_k / r_0`: every
  wall's four rational vertices are coplanar exactly (the exact `orient3d`
  over them is zero); `Tessellate` at `0.01 mm` with `VerifyBoundary` returns
  a verified mesh whose `Bound` equals `delta`; the STL round trip is
  watertight; `Placed` under a rotation reproduces `Volume` exactly.
- A hole loop sweeps to a void passage and subtracts from `Volume` exactly.
- Each Table SM row refuses with its sentinel before any commit: the
  document's live set, order and next producer identity are unchanged.
- A sharp bend with a strong taper and a wide section reaches SM6 and the
  error names the span and the vertex.
- A path that returns through an earlier span reaches SM8 with
  `ErrDegenerate`, and the audit names the two faces.
- M2: the union of two branches that cross is one lump whose `Volume` lies
  within its published bound of an independently computed
  `V1 + V2 − overlap`.
- M2: a leaning, tapered 16-gon trunk minus a thinner mitred bore that
  reaches neither cap is one lump of one outer and one void shell, every void
  face from the bore; its `Volume` encloses the exact trunk minus bore; the
  verified mesh, the STL and the 3MF each hold two components enclosing the
  trunk's volume and minus the bore's; and a branch whose root is buried in
  the wall unions onto it with the void shell unchanged.

### 16.11 Companion edits

- §1 names §16 as the owner of the corner mode and the scale, §2 lists the
  two options, Table S's row S6 points here, and §12 lists M1 and M2.
- `docs/layout/design-docs.md` row for this document names §16; the implementing PR
  adds rows for the files that build the mitred sweep.
- The implementing PR adds the payload's row to `docs/tessellation-design.md`
  §2's table and to `docs/payload-verification-design.md`, and changes
  `doc.go`'s support map.
- `docs/api-design.md` needs no edit: `Sweep`'s signature is unchanged and §8
  already names this document as the owner of its options and reach.
