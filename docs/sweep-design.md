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

Four tables are normative:

| Table | States | Section |
|---|---|---|
| **P** | spatial path representation and frame transport | §3 |
| **S** | refusals and their sentinels | §5 |
| **B** | payload, topology, faces, and roles | §8 |
| **D** | downstream coverage and staging | §10 |

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

The following remain outside this design:

- scale or taper along the path;
- guide surfaces, guide rails, and a second profile;
- a corner mode for non-tangent path joins;
- closed paths and their frame holonomy;
- a sweep that lets the section leave the path-normal plane.

`Loft` remains the operation for two different sections. `Revolve` remains the
direct operation for one angular span. `Sweep` does not infer either operation
from approximate input; its evaluator may reduce an admitted path segment to
their existing builders internally.

## 2. Public API

```go
func (d *Document) Sweep(
    s *sketch.Sketch,
    p *sketch.Profile,
    path *Path,
    opts ...SweepOption,
) (*Body, error)

func (d *Document) SweepContext(
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
body, err := doc.Sweep(s, profile, path)
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
| **S6** | an internal path join is not tangent | `ErrUnsupported` | no; a future corner-mode option may define it |
| **S7** | path is closed | `ErrUnsupported` | no; closed-frame holonomy is staged |
| **S8** | an arc span's rotation axis crosses the transported profile interior, or boundary contact fails Revolve's exact axis-incidence rule | `ErrDegenerate` | yes; the mapped boundary folds or pinches |
| **S9** | remote patches contact, or neighbours contact beyond their shared boundary | proved contact: `ErrDegenerate`; undecided budget: `ErrUnsupported` | contact is permanent; budget is not |
| **S10** | profile kind is unsupported by one of the path-span builders | `ErrUnsupported` | follows spline reach |
| **S11** | nonzero `WithSweepTwist` before the faceted-twist increment | `ErrUnsupported` | no |
| **S12** | option repeated, or a foreign type embeds the sealed marker | `ErrDegenerate` | yes |
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

The build therefore creates an ephemeral certified facet cover of every
lateral patch and both endpoint caps. It reuses the tessellation design's
shared boundary stations, source bounds, exact triangle predicates, fixed work
counters, and homotopy sign tests. This cover is an audit input, not the body
representation.

The audit classifies every facet pair:

- adjacent cells may meet only on their recorded shared edge or vertex;
- the two cells on opposite sides of an internal path section may meet on that
  section boundary only;
- all other pairs must have a separating certificate wider than the sum of
  their true-to-held displacement bounds;
- a proven extra contact is S9 `ErrDegenerate`;
- a pair inside the undecidable displacement band refines deterministically;
- exhausted refinement is S9 or S14 `ErrUnsupported`.

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
| **D1** | structural `Verify` + tolerance gate | lands with Sweep. The construction and global audit prove validity; all four readings are judged |
| **D2** | `Tessellate` / STL / OBJ | staged until the shared-span tessellator publishes complete source, area, and boundary proofs |
| **D3** | mesh booleans | staged until D2 also publishes `volSymDiff` with `symDiffOK == true` |
| **D4** | interference | bounds-disjoint pairs work immediately. Other pairs stay `Suspect` until D3 or a sweep analytic adapter lands |
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

`SweepContext` checks cancellation at every phase boundary and through path
derivation, station generation, facet-pair classification, exact predicates,
and measurement accumulation. It returns `ctx.Err()` unchanged. `Sweep` calls
it with `context.Background()`.

The evaluator uses the existing per-walk chord cap, mesh facet cap, cumulative
facet-work cap, and facet-pair-test cap. It adds one sweep-span cap so topology
and role counts are preflighted before allocation. The defining source constant,
not this design document, owns its numeric value and derivation.

No budget resets per span. Checked arithmetic computes the complete build's
candidate counts before any large allocation or audit.

## 12. Increments

The current evaluator implements the first three increments. Later rows remain
staged and return `ErrUnsupported` at the public boundary.

| PR | Lands | Still staged |
|---|---|---|
| **1** | `Path`, line/arc recording, exact join classification, public Sweep signatures and option validation | every build returns `ErrUnsupported` |
| **2** | zero-twist one-span line reduction, topology and all four measurements | arc spans, composite paths, downstream beyond D1 |
| **3** | zero-twist one-span arc reduction with Revolve's local gates | composite paths |
| **4** | composite tangent line/arc transport, internal-section topology, global contact audit, D1 and D7 | D2–D6, nonzero twist |
| **5** | shared-grid tessellation and complete proof record; D2 and D3 | analytic clearance and surveys |
| **6** | Tier A free-form profile reach supported by each span builder | Tier B/C profile kinds follow spline staging |
| **7** | nonzero distributed twist as a certified faceted sweep | closed paths, corner modes, scale/taper |
| **8** | sweep boundary adapter for clearance/interference | non-constant-section surveys |

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
- `docs/layout.md` lists this document;
- `docs/evaluator-design.md` §11 points to this document's staged delivery;
- the implementation increment that adds `sweepPayload` adds its row to
  tessellation and payload-verification tables in the same change;
- `doc.go` changes only when an implementation increment changes current
  evaluator support.

No current dependency exposes a 3D sketch. This design consumes none. A future
adapter is a separate design change after an upstream spatial-path snapshot and
its certification contract exist.
