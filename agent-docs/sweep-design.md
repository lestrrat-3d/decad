# Sweep Design

`Sweep` moves one recorded planar profile along an ordered spatial path. The
profile stays normal to the path and uses rotation-minimizing transport. This
document owns the spatial-path vocabulary, frame transport, refusals, topology,
measurements, downstream coverage, and staged implementation.

Companion contracts remain authoritative for their existing areas:

- `agent-docs/api-design.md` owns the public model and sketch-profile seam;
- `agent-docs/sketch-seam-design.md` owns profile authentication and recording;
- `agent-docs/evaluator-design.md` owns topology, commits, and payload dispatch;
- `agent-docs/tessellation-design.md` owns public mesh proofs and boolean admission;
- `agent-docs/verification-design.md` owns report meaning and tolerance gates;
- `agent-docs/spline-design.md` owns free-form profile-segment reach.

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

The current `Sweep` implementation admits:

- an open path made from `LineTo` and `ArcThrough` segments;
- §3.3's closed full circle made from two complementary `ArcThrough` segments
  when the result is a zero-twist solid;
- §3.4's closed planar cardinal loop, which includes the four-span racetrack, with any
  authenticated section the arc builders and the contact audit prove simple;
- §3.5's loops with split lines and any arc count on the same construction;
- line, circle, arc, and fitted-spline profile boundaries;
- zero total twist, or §17's bounded straight-path twist;
- tangent internal joins;
- a build whose local curvature gates and global contact audit prove a simple,
  closed solid.

The design also fixes these later increments:

- other Tier A free-form profile boundaries follow `agent-docs/spline-design.md` reach;
- nonzero distributed twist over curved and composite paths builds a certified
  faceted sweep in a later increment;
- exact-restatement tessellation admits the payload to export and booleans;
- a future constrained 3D sketch may be recorded into the same `Path` without
  changing `Sweep`.

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
- closed paths outside §§3.3–3.5 and their frame holonomy;
- a sweep that lets the section leave the path-normal plane;
- a helical path: `agent-docs/helix-design.md` builds it as `Coil`, a
  screw motion whose section stays in the axis plane, and its §12 states
  why it is not a `Path` segment.

`Loft` remains the operation for two different sections. `Revolve` remains the
direct operation for one angular span. `Sweep` does not infer either operation
from approximate input; its evaluator may reduce an admitted path segment to
their existing builders internally.

**`Sweep` takes a closed profile or an open sketch chain through one generic
entry point, and §15 owns the chain case** — its pairing rule, refusals,
result and staging. Everything §1
through §14 states about the path, the transport, the span builders and the
global audit is what that section consumes; what it replaces is the section
alone.

## 2. Public API

```go
// Section is the sketch geometry a section-fed operation consumes: a closed
// *sketch.Profile or an open *sketch.Chain.
type Section interface{ *sketch.Profile | *sketch.Chain }

func Sweep[S Section](
    ctx context.Context,
    d *Document,
    s *sketch.Sketch,
    sec S,
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

// WithSweepTwist rotates a profile about the transported tangent. Section 17
// states the current straight-polygon admission and its proof.
func WithSweepTwist(angle units.Value) SweepOption

// WithSurfaceResult omits the two section caps and publishes a sheet body
// instead of a solid (agent-docs/surface-design.md §4). Unlike every other sealed
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
body, err := decad.Sweep(ctx, doc, s, profile, path)
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
`ErrUnitKind`; a non-finite value is `ErrNotFinite`. Section 17 admits one
nonzero-twist case; other cases return `ErrUnsupported`.

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
| **P9** | A path ending at its start is closed. Sweep admits §§3.3–3.4 when exact path geometry proves the transported end frame equals the start frame. Every other closed path is staged, even when its internal joins are tangent. No distributed correction is inferred. |

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

### 3.3 Closed full-circle reduction

`Sweep` admits a zero-twist solid path of exactly two `ArcThrough` segments when
the second ends at the first start, both exact rational circumcenters agree,
and the two arc axes are exactly codirectional. The path's existing initial
tangent and internal-join checks run first. Each arc has a directed angle
strictly between zero and one turn. The shared endpoints and codirectional
circle carriers therefore make the arcs complementary: their total turn is
exactly one, and the terminal tangent is codirectional with the initial one.
Rotation-minimizing transport about the same fixed axis for one turn returns
the initial section frame without a fitted correction.

The common axis must lie in the source profile plane. The existing full
`Revolve` axis-side and axis-incidence checks decide whether the authenticated
profile sweeps a simple solid. `Sweep` evaluates that full revolution from its
recorded profile and exact circle carrier, then publishes a `sweepPayload` that
holds the original path and the full-revolution proof. A closed path with a
nonzero twist, `WithSurfaceResult()`, a corner option, or section scale is
staged. A near match never enters this reduction; §3.4 owns its separate
closed-path class.

### 3.4 Closed cardinal loop

`Sweep` admits a zero-twist solid on a closed path whose recorded spans are
lines and arcs, when exact path data proves all of these:

- the path has at least four spans and at least two arcs;
- every arc is an exact quarter or half turn (`Turn` of one quarter or one
  half, with no radian or interval angle) about one coordinate axis, and all
  arc axes are parallel to one coordinate axis (either sense; the arcs need not
  share one axis line);
- the start and end of every span and every arc centre have one common
  coordinate along that axis, so the loop lies in one plane perpendicular to
  it;
- the signed turns sum to exactly one whole turn in magnitude.

The gate reads recorded coordinates only; a close sampled path cannot pass.
Each refusal wraps `ErrUnsupported` and names the failed condition (S7). The
gate does not inspect the section. A section of any kind a composite sweep
builds, off-centre or not, reaches the construction below.

The transport then runs once over the closed path. Every quarter and half turn
about a coordinate axis takes the exact frame path, so the terminal frame is
compared with the initial frame at zero bound and equal components. A frame
that differs, or that carries any displacement, refuses with S7. The two
checks agree for every path the gate admits; the second guards the cyclic
construction against a gate that is ever loosened.

Each arc builder proves the section stays on one side of its carrier axis and
meets it per Revolve's exact axis-incidence rule (S8). The global audit
treats the last and first spans as neighbours. Neighbouring span interiors
must lie on opposite sides of their shared section plane, two neighbouring
exact half turns must keep their far sections apart by a strict bounded-box
gap, and every other span pair must have a strict bounded-box gap (S9, §7).
Short legs and off-centre sections therefore build exactly when the audit
proves them simple, and a section that touches an opposite leg refuses. The
audit, not a clearance inequality on the section, decides.

The span builders provide analytic walls and two temporary caps per span. All
rims pair by recorded profile station; the last rim pairs with the first. The
final body omits every temporary cap and has one closed shell. It keeps
`side(k,i,j)` provenance for every wall. `Transform` replays the same records
and the cyclic proof.

A circular section of radius `r` on §3.4's four-span racetrack (a positive-Z
line of length `L`, a positive-X semicircle of radius `R` in XZ, a
negative-Z line of length `L`, and a second semicircle returning to the start)
has volume `pi*r*r*(2*L + 2*pi*R)` and area `2*pi*r*(2*L + 2*pi*R)`. A
section whose centroid lies on the path has volume `area * pathLength` for any
loop (Pappus). An off-centre section is swept at the centroid radius on each
arc, so its volume is `area * (line length + sum of 2*pi*turn * centroid radius)`.
The body sums each span's certified volume and first moments, its live wall
areas, and its span bounds. Tessellation shares one profile grid across all
spans and identifies the last station with the first. It omits cap triangles,
charges the final rim's coordinate movement like every other join, and audits
cyclic contact, closed vertex links and positive orientation before
publishing the occupied-volume proof.

### 3.5 Loops with split lines and more arcs

The gate in §3.4 places no limit on the arc count or the line lengths, so an
eight-span rounded rectangle, a loop with consecutive collinear lines, and a
racetrack all take the same construction. Paths with non-cardinal arc turns,
arcs about different axes, spatial loops or uncertain frame closure remain
staged at S7.

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
two sampled tangents happen to be close. `Sweep` and the private sweep
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
| **S7** | path is closed but outside §§3.3–3.4's bounded solid reductions: fewer than four spans, an arc that is not an exact quarter or half turn, arcs about more than one coordinate axis, a span off the loop's plane, signed turns that do not sum to one whole turn, or a transported end frame that differs from the start | `ErrUnsupported`; the message names the failed gate | no; general closed-frame holonomy is staged |
| **S8** | an arc span's rotation axis crosses the transported profile interior, or boundary contact fails Revolve's exact axis-incidence rule | `ErrDegenerate` | yes; the mapped boundary folds or pinches |
| **S9** | remote patches contact, or neighbours contact beyond their shared boundary | proved contact: `ErrDegenerate`; undecided budget: `ErrUnsupported` | contact is permanent; budget is not |
| **S10** | profile kind is unsupported by one of the path-span builders | `ErrUnsupported` | follows spline reach |
| **S11** | nonzero `WithSweepTwist` outside §17's bounded profile and path cases | `ErrUnsupported` | no |
| **S12** | option repeated, or a foreign type embeds the sealed marker | `ErrDegenerate` | yes; `WithSurfaceResult()` is exempt — a repeat is idempotent (agent-docs/surface-design.md §4.1) |
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
whose spans touch only at their shared sections.

Opposite closed sides confine neighbour contact to the shared plane, but do
not confine it to the shared section. A prism, a drafted slab, or an
exact-turn arc of less than a half turn meets that plane only in the shared
section. An exact half turn keeps both caps in the plane through its axis.
Beside any other neighbour, its far cap can therefore meet that neighbour
only inside the shared section, which is its own other cap. Its two caps
share at most the axis points where S8 lets the profile touch the axis.
Those points lie in the shared section itself, so they add no contact
beyond it. When both neighbours are exact half turns, the first span's far
(start) section and the second's far (end) section both lie in the shared
plane. That pair must also have a strict separating gap between the two far
sections' bound-inflated axis-aligned boxes. Each box is the arc's own bounded extent
reading with its angular interval closed to that one end angle. Overlapping
or touching far sections refuse, and so does a gap the boxes cannot prove.
The closing join of a cyclic composite takes its last span as the first
neighbour. Every non-neighbour pair must have a strict separating gap
between its bound-inflated axis-aligned boxes. A pair without the
certificates its kind needs returns `ErrUnsupported`; the evaluator never
infers disjointness from samples.

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

For loop `i`, profile segment `j`, and path span `k` on an open path:

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
twist stores certified flat facets and reports `Faceted` wall surfaces; it
never labels a twisted patch with an analytic surface it is not.

Internal section edges remain even when the two incident patches are tangent.
They name the change of path carrier and preserve `side(k,i,j)` provenance.
Adjacent patches merge only when their complete analytic carriers and trims are
the same and merging preserves every origin role.

The §3.3 full circle has no section caps or terminal rim. Each full-turn wall
face carries both `side(0,i,j)` and `side(1,i,j)` origins for its two source
path arcs. The full `Revolve` builder owns its seam-free analytic topology and
its exact axis-incidence audit; `Sweep` changes only the face origins and
retains the source `Path` in its payload. A merged wall face keeps every origin
of its contributing profile segments.

A §3.4 cardinal loop also has no section caps. Its analytic wall faces meet at
one shared rim per span, including the final-to-initial section seam.

The body has one lump and one outer shell. Profile holes become void passages
through the sweep; they do not create extra lumps. S9 rejects any mapping that
would change those claims.

**This is the solid's own shape alone.** `WithSurfaceResult()`
(agent-docs/surface-design.md §2.2, §4) omits the two section caps that join a
holed profile's outer wall tube to its hole wall tube, so a surface-result
sweep's holed profile reports one `Lump` per tube instead — the identical
carve-out `agent-docs/surface-design.md` §2.2 already states for the surface-result
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

The §3.3 full circle takes all four readings and their bounds from the full
`Revolve` reduction. It does not sum two partial-turn readings across a seam.
A §3.4 cardinal loop sums one certified reading per span and omits all cap areas.

Every span carries the displacement of its derived path carrier, transported
frame, computed stations, and accumulated rigid motion. A reading composes only
the terms that can move the geometry it reads. Bounds round outward at every
sum, product, quotient, and min/max publication.

An `Exact` result requires every contributing closed form to be exactly
representable and every displacement term to be zero. Operation history grants
no exactness. A single positive span bound makes the combined result
`Approximate` with the outward-rounded sum or maximum appropriate to that
quantity.

The admitted nonzero twist uses §17's exact cross-sectional volume and centroid,
wall-area integral, boundary-displacement and occupied-volume proof. It never
sums untwisted analytic span formulas.

The body tolerance reference reads a proven lower bound on the true diameter.
It reads the single-cell Loft's vertices or the stationed build's authenticated
sewn mesh vertices, then subtracts twice the payload's maximum positional
displacement, rounded down. A non-positive result withholds the reference and
prevents a false `Sound` report.

## 10. Table D — downstream

| D | Consumer | Status |
|---|---|---|
| **D1** | structural `Verify` + tolerance gate | lands with Sweep. For a solid, the construction and global audit prove validity and all four readings are judged; §3.3 uses the full `Revolve` proof and §3.4 uses the cyclic span audit. For a `WithSurfaceResult()` sheet, only the one-span straight reduction carries an equivalent construction proof (it IS a prismPayload build, agent-docs/surface-design.md §9.1); the arc reduction and every composite sheet read `Suspect` — the arc build runs no crossing audit of its own, and the composite build's own boundary/vertex-link audit proves assembled topology, not geometric non-self-intersection |
| **D2** | `Tessellate` / STL / OBJ | One-span sweeps and §3.3's full circle use prism or Revolve. Composite sweeps mesh line, circle, arc and fitted-spline profiles with shared stations; §3.4 identifies the final station with the first. Meridian poles or unmatched stations refuse. Sheets have no occupied-volume proof |
| **D3** | mesh booleans | one-span solids and §3.3's full circle use their reduction's `volSymDiff`. An admitted composite solid sums span occupied-volume bounds and all join-vertex movement bounds, with `symDiffOK == true`; its mesh can enter `Union`, `Cut` and `Intersect` |
| **D4** | interference | bounds-disjoint pairs work immediately; an admitted solid sweep reaches the mesh intersection D3 admits. Other pairs stay `Suspect` until D3 or a sweep analytic adapter lands |
| **D5** | clearance | box separation may settle the partition, but `WithClearances` stays `Suspect` until a sweep boundary adapter lands |
| **D6** | `Wall`, `Undercut`, `ConcaveRadius` | `Unavailable` with `DiagUnsupportedSurveyPayload` until non-constant-section proofs land |
| **D7** | `Transform`, `Copy` | re-evaluates the payload under the composed rigid motion and reruns the applicable construction proof; every displacement and measurement is recomputed |
| **D8** | Fillet, Chamfer and Shell | Straight solid profile: lateral Fillet/Chamfer, one Route E cap edge for either blend, and Shell (modify R9). Twisted line: inward top-cap Shell (R10). Other targets refuse; sheets may Thicken (§16.8). |

The composite tessellator chooses the largest required profile chord count
from its prism and Revolve span plans for each recorded segment. Each span
meshes against that count. It maps join vertices by profile loop, segment and
station order, and refuses a station disagreement larger than the two meshes'
source bounds. It never searches for a nearby vertex. The first span's stored
coordinate becomes the one vertex at a join; the next span's exact dyadic
coordinate gap is charged to its source, area and occupied-volume proofs.
Internal cap triangles are omitted from the published mesh. The two span
cap triangulations tile the same shared section, so the composite mesh keeps
one closed shell with no internal face.

For a `FitSplineSeg`, the planner takes the smallest of the prism's chord
target and each arc span's proven meridian target. Every span independently
chords the same recorded Bézier chain at that target. The deterministic dyadic
walk therefore emits the same exact parameter stations in each span. The
planner records their common count, and each Revolve span checks that its
independent chain has that count before the shared-grid join audit runs.

`sourceBound(face)` contains the span reduction's profile chording, path
chording and coordinate displacement, plus any join-vertex movement on that
span. `Mesh.Bound` is their maximum. `areaSlack` sums the reductions'
non-cancelling terms and the moved vertices' per-triangle area allowance.
`volSymDiff` sums the per-span occupied-volume homotopies and the join
movement's swept-volume allowance. The combined mesh audits cross-span facet
pairs; each reduction already audits its own pairs. That audit charges only
coordinate movement, since each reduction's chording is already covered by
its own occupied-volume proof and the analytic composite audit separates the
true spans.

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

A composite `Sweep` carrying a `FitSplineSeg` gives its recorded-profile
exact-work counter an eightfold allowance, bounded at
`8 * freeform.FreeformWorkLimit`. The fitted-spline arc-line-arc case spends
2,101,728 units, above the default 1,048,576-unit ceiling. All analytic span
builds in one Sweep or placement replay share that counter. The composite mesh
planner and each independently certified span tessellator use the same bounded
allowance for their own exact-work replay. Other Sweep paths keep the default
limit. A profile that exceeds the composite build limit returns
`ErrUnsupported` before the document commits a body.

The analytic build's exact-work budget does not reset per span. Checked
arithmetic computes the complete build's candidate counts before any large
allocation or audit.

## 12. Increments

The current evaluator implements the first four increments and 5a, with composite
paths admitted when every transported frame is exact and the conservative §7
separation audit closes. Other composite paths remain staged as
`ErrUnsupported`.

| PR | Lands | Still staged |
|---|---|---|
| **1** | `Path`, line/arc recording, exact join classification, public Sweep signatures and option validation | every build returns `ErrUnsupported` |
| **2** | zero-twist one-span line reduction, topology and all four measurements | arc spans, composite paths, downstream beyond D1 |
| **3** | zero-twist one-span arc reduction with Revolve's local gates | composite paths |
| **4** | composite tangent line/arc transport, internal-section topology, global contact audit, D1 and D7 | D2–D6, nonzero twist |
| **5a** | one-span arc sweep tessellation through its revolve reduction, one-span sheet tessellation through its existing prism or revolve reduction, and mesh-boolean admission for arc solids; D2 and D3 for those reductions. **This row is landed.** | composite paths |
| **5b** | shared-station tessellation and complete proof records for composite paths with line and circular profile walks; D2 and D3 for that reach. **This row is landed.** | Meridian poles, analytic clearance and surveys |
| **6** | `FitSplineSeg` profile build and shared dyadic tessellation through composite arc-line-arc paths. **This reach is landed.** | Other Tier A profile kinds and Tier B/C kinds follow spline staging |
| **7a** | §17's one-span centred convex polygon twist and verified faceted mesh. **This reach is landed.** | curved/composite paths, holes, non-convex or free-form profiles, sheets, mitres and scale |
| **7b** | nonzero distributed twist over tangent composite paths | closed-path twist, a corner mode or scale other than §16's |
| **7c** | §17.4's centred symmetric Line/Arc profile on one straight path, with bounded Loft wall facets and verified mesh | other curved profiles, holes, sheets, mitres and scale |
| **8** | sweep boundary adapter for clearance/interference | non-constant-section surveys |
| **M1**, **M2** | §16's mitred polyline sweep; §16.9 owns the two rows | what §16.9 lists |
| **F1** | §3.3's exact two-arc full circle as a zero-twist solid, including D1–D3 and D7 through full `Revolve` | general closed paths and nonzero closed-path twist |
| **F2** | §3.4's closed cardinal loop with any proven-simple section as a zero-twist solid, including D1–D3 and D7 | nonzero closed-path twist |
| **F3** | §3.5's eight-span rounded rectangle and other split-line loops on the same construction | non-cardinal or spatial closed paths and nonzero closed-path twist |

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
  path segment, a non-tangent join, and a closed path outside §§3.3–3.5.
- Build §3.3's full circle from an authenticated profile. Compare its four
  readings with a full `Revolve`, assert both path-span wall roles, no caps,
  `VerifyAll` occupied-volume proof, and placement replay. Refuse closed-path
  twist and a closed surface result without committing a body.
- Build §3.4's racetrack from a solved Sketch circle. Check its volume and
  area against the tube formulas, four shared rims, no caps, cyclic
  `VerifyAll` at 0.1 mm, and placement replay. Build a short-leg racetrack
  and an off-centre circle and compare their volumes with Pappus. Refuse a
  circle that touches the arc axis without committing a body.
- Build the racetrack and the eight-span rounded rectangle from a solved
  square Sketch section, and the racetrack about the X axis. Check volume
  and area against `side*side*pathLength`, `Verify`, and `VerifyAll` at
  0.1 mm.
- Build §3.5's eight-span rounded rectangle from a solved Sketch circle.
  Check its tube volume and area, eight shared rims, no caps, `VerifyAll`
  at 0.1 mm, and placement replay. Refuse axis contact without committing.
- Refuse a closed loop of arcs that are not quarter or half turns, with the
  specific S7 message, and unit-test every other S7 condition of §3.4.
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

- `agent-docs/api-design.md` §8 adds `Sweep` and points here for its signature and
  path contract;
- `agent-docs/api-design.md` §13 removes Sweep from the non-goals list;
- `agent-docs/layout/design-docs.md` lists this document;
- `agent-docs/evaluator-design.md` §11 points to this document's staged delivery;
- the implementation increment that adds `sweepPayload` adds its row to
  tessellation and payload-verification tables in the same change;
- `doc.go` changes only when an implementation increment changes current
  evaluator support;
- `agent-docs/surface-design.md` §1.2, §13.5, Table R and Table D name §15 as the
  owner of a chain `Sweep`'s pairing rule, and its §15 carries §15.7's test rows.

No current dependency exposes a 3D sketch. This design consumes none. A future
adapter is a separate design change after an upstream spatial-path snapshot and
its certification contract exist.

## 15. `Sweep` over a chain — sweeping an open sketch chain

`agent-docs/surface-design.md` §13 owns the open chain itself: what `sketch.Chain`
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
(`agent-docs/surface-design.md` §13.1, §13.3). Two adjacent chain spans transport
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
lists. `buildSides` (`internal/chainrecord/build.go`) already mints that post set for one
span, and `agent-docs/surface-design.md` §13.3 states the same drop for a chain
revolve's own two free ends.

**What the loop supplies and the walk does not is a CAP FACE, and that is an
implementation seam rather than a proof.** The composite build reads each
span's two rim lists off that span's `capStart` and `capEnd` face loops and
refuses a span carrying neither. A chain-fed span mints no cap at all
(`agent-docs/surface-design.md` §13.4), so the chain composite build reads its
two rim lists off the span's WALL faces instead — wall `j`'s own start rim
edge and end rim edge, in recorded-segment order. That is the identical index,
reached through the face an open walk does have.

**The separation certificate carries over unchanged.** §7's adjacent-span rule
requires each span's material to be certified on its own side of the shared
section plane. It reads the span's analytic extent along that plane's normal
and the span's own endpoint-supporting-plane property, and neither reading asks
whether the section closes: a chain prism span supplies both through
`chainrecord.Payload.Prism`'s `prismPayload` view, and a chain arc span through
`chainRevolvePayload.Revolve`'s `revolvePayload` view. The non-neighbour rule
is a bounded-box separation and reads no section at all. The topology audit
that follows the sew already admits a free rim edge — it requires one or two
incident faces per edge and a vertex link that is one cycle or one path — so a
chain span's own free rims pass it for the reason a surface result's two
omitted outer caps already do.

**What the join does NOT inherit is a solid.** Every profile-fed span builds
closed and the two outer caps are dropped afterwards, which is why a
surface-result composite sweep still carries two cap roles through the pairing.
Every chain span publishes `BodySheet` from its own first face, so the
chain composite build pairs two sheets at every join and never a temporary
solid.

### 15.2 A one-span chain sweep states no join at all

A path of one span has no internal join, so §15.1's rule has nothing to
decide there. The whole build is the recorded walk transported over that one
span:

- a **`LineTo`** span is the chain prism a chain `Extrude` already
  builds, over the path's own signed height instead of a resolved `Extent`;
- an **`ArcThrough`** span is the chain shell `Revolve` over a chain already
  builds, over the arc's exact centre, axis and directed angle instead of a
  resolved `AngularExtent`.

Both constructions are landed (`agent-docs/surface-design.md` Table D row 6), so a
one-span chain `Sweep` was never waiting on a pairing rule. It was waiting on
this section to say so, and on Table SC to state which path shapes it admits.

**The two reductions are not the same call as their chain siblings, and the
difference is the height.** A chain `Extrude` takes its interval from an `Extent`
the document resolves; a straight chain sweep takes it from the path's two
recorded points, so the interval carries `sweepinput.ValidateStraightSweepPath`'s own
composed bound — the square root's committed error against the exact rational
squared length, plus the per-component departure of the held sweep vector from
the exact tangent. On an axis-aligned frame every one of those terms is zero
and the two calls publish bit-identical measurements; on any other frame the
chain sweep's `Area` is `Approximate` where the same walk's chain `Extrude`
reading may not be.

### 15.3 The entry point

A chain enters through the same function a profile does:

```go
func Sweep[S Section](ctx context.Context, d *Document, s *sketch.Sketch,
    sec S, path *Path, opts ...SweepOption) (*Body, error)
```

`Section` (`section.go`) is the constraint `*sketch.Profile |
*sketch.Chain`. `Sweep` type-switches on the section and calls the profile
build (§§2–14, §16, §17) or the chain build (this section). The two builds keep
distinct payloads, `sweepPayload` and `chainSweepPayload`, for §10's reason: a
consumer whose proof has not landed for a chain dispatches on the payload type
and refuses. Each build keeps its own gate order, so the chain build runs Table
SC below, not Table S.

**A chain section accepts the one `SweepOption` type, and refuses the options
it cannot honour at run time.** A chain sweep is always a sheet — an open walk
encloses no region, so there is no cap to omit and no solid to ask for — so
`WithSurfaceResult()` returns `ErrUnsupported`: accepting it as a no-op would
give one option two meanings. `WithSweepTwist` returns `ErrUnsupported` for any
angle, zero included, because §17 admits only a solid polygon profile for
nonzero twist (Table S row S11) and an open chain has no corresponding solid.
`WithMitredJoins` and `WithSectionScale` return `ErrUnsupported` as well,
because §16 builds solids from a profile only.

**The option refusal runs after the chain's seam gates (SC2) and before every
path-geometry gate (SC3 onward).** `Sweep` first checks for a nil argument and a
nil option (SC1), then records the chain, then decodes the options and refuses
the four above. A foreign, stale, invalid or unrecordable chain therefore still
reports its seam sentinel when it also carries one of those options, and a
chain that records but carries one of those options reports `ErrUnsupported`
even when its path would also fail SC3. A malformed option, such as
`WithSweepTwist` with a non-angle value, reports its codec sentinel at the same
point. A refused call commits nothing and leaves `Document.Bodies()` unchanged.

The call takes the sketch beside the chain because a chain's geometry is
plane-local and the plane is the sketch's (`agent-docs/api-design.md` §7).

**A chain-fed result is always a sheet**, which is `agent-docs/surface-design.md`
§13.2's answer for a chain `Extrude` and a chain `Revolve` restated with no change:
`Kind()` is `BodySheet` by construction, `IsSolid()` is `false`, and
`Volume()`/`Centroid()` answer `ErrNotSolid` by KIND rather than by soundness.

### 15.4 Table SC — what refuses a chain sweep

Every row lands on `agent-docs/api-design.md` §12's existing vocabulary, and no row
mints a sentinel of its own. Gate order is §5's, with its step 2 reading a
chain through `RecordChain` instead of a profile through `RecordProfile`.

| SC | Condition | Sentinel |
|---|---|---|
| **SC1** | a nil document, context, sketch, section (typed-nil included) or path; an empty path; a nil option | `ErrDegenerate` |
| **SC2** | the chain fails one of `agent-docs/surface-design.md` §13.3's gates | the seam's own: `ErrForeignProfile` / `ErrStaleProfile` / `ErrInvalidProfile` / `ErrUnrecordableProfile` |
| **SC3** | the path start is not in the sketch plane, or its initial tangent is not codirectional with that plane's positive normal | `ErrDegenerate`, S5's rule over a chain's own recorded plane |
| **SC4** | every path condition S3, S4, S7 and S13 through S15 already state — a non-finite point, a degenerate segment, a closed path, a non-finite derived bound, an exhausted budget, a collapse from computed-coordinate rounding | those rows' own sentinels, unchanged |
| **SC5** | an internal path join is not tangent | `ErrUnsupported`, S6 |
| **SC6** | a recorded chain segment outside the line, circle and arc set the span builders admit | `ErrUnsupported`, S10 |
| **SC7** | a composite path whose first span is an arc, or with an arc that is not an exact quarter or half turn about a coordinate axis, a transported frame with nonzero displacement, a span wall without four edges or line rims, two joined spans whose walls do not pair by recorded segment, or spans that §7's separation audit cannot certify (§15.8) | `ErrUnsupported` |
| **SC8** | an arc span the chain shell refuses — a walk that does not lie in one closed half-plane of the arc axis, both free ends on that axis, or an interior on-axis junction | exactly what a chain `Revolve` answers: `ErrDegenerate`, or `ErrUnsupported` under `agent-docs/surface-design.md` R22 and R33 |

**SC2 is decided before SC3, and that order is load-bearing.** A foreign,
stale, invalid or unrecordable chain names a repair the caller makes in the
sketch, and reporting a path refusal first would hide it behind geometry the
caller cannot act on. It is also what keeps a staged `ErrUnsupported`, the
option refusals of §15.3 included, from masking a seam refusal.

### 15.5 What a chain sweep publishes

| Reading | A chain sweep |
|---|---|
| `Kind()` | `BodySheet`, always and by construction — a decided enum carrying no bound |
| `IsSolid()` | `false`, always |
| `Volume()`, `Centroid()` | `ErrNotSolid`, by kind |
| faces | one wall per recorded segment per path span, after the rim coalescing `Extrude` already runs — `agent-docs/surface-design.md` Table G's own per-kind construction, never a cap and never a closing face |
| `Edges(Free())` | both rims of the walk over every span, plus the one sweep edge at each of the walk's two free ends; a one-span straight sweep of an `n`-segment walk resolves `2n + 2` |
| `Area` | the sum of each wall's own area through `boundedAdd`. A straight span's wall is `boundedMul(segment length, span height)`, the height carrying §15.2's composed path bound; an arc span's is `boundedMul(walkAxisMoment, sweep)` on `agent-docs/surface-design.md` §13.4's own terms. A circular wall carries its proven integral enclosure and a free-form wall `internal/freeform/spline_length.go`'s proven bracket |
| `Bounds` | the recorded walk's per-segment analytic extremes swept over the span's signed interval, from `internal/prismrecord/extent.go` and `internal/revolveplan/extent.go` verbatim, charging the frame and placement rounding those readings already charge |
| `Verify`'s validity | exactly what a chain `Extrude` ribbon or a chain `Revolve` shell over the same recorded walk and interval reads (`agent-docs/surface-design.md` §13.4, §9.1). A composite chain sweep reads `ValidityUndecided`, for D1's own reason: the assembled topology audit proves how the spans sew together, never that the swept walls do not fold back through one another |

No bound in that table is new, because no geometry is: every wall is built by
the per-segment construction its chain sibling already proves, and the one term
a chain sweep adds that neither sibling has is §15.2's path height bound.

### 15.6 Increments

| PR | Lands | Still staged |
|---|---|---|
| **C1** | `Sweep` over a chain along a one-span `LineTo` path: Table SC rows SC1 through SC4 and SC6, the chain prism reduction, and the four readings §15.5 states for it | a standalone arc and composite paths outside §15.8 |
| **C2** | a one-span `ArcThrough` reduction with the chain shell's own axis gates (SC8), plus §15.9's one-line sheet mesh | other chain-arc meshes |
| **C3a** | §15.8's radial line/half-circle composite with one source `LineSeg`: its certified sheet mesh | other composite sheet meshes and a placed composite sheet |
| **C3** | §15.8's composite join over `N` tangent line/arc spans that start with a line and turn only by exact coordinate-axis quarter and half turns: wall-face rim lists per recorded segment, indexed sew, separation and boundary audits, area and bounds, and `Thicken` of a source walk of axis-parallel lines with right-angle corners | an arc-first path, curved source segments, other arcs, other sheet meshes, `Thicken` of other line walks and placement |

The one-span straight chain sweep reuses the chain prism's exact-quad mesh.
§15.9 meshes one straight source line swept through a single arc. §15.8's
composite uses its own bounded sheet mesh. Other chain-sweep meshes return
`ErrUnsupported` at tessellation, and the modify operations have no receiver
row for the new one-span arc sheet (D8).

### 15.7 Required tests

Each row asserts on computed geometry, and each bound assertion is shown to
fail before it is trusted. `agent-docs/surface-design.md` §15 carries the rows
themselves, as T190 onward, so the obligations sit beside every other
chain-fed obligation rather than in two places.

### 15.8 Composite chain sheet over exact quarter and half turns

**Path gate.** A composite chain path has two or more tangent spans, and its
first span is a line; an arc-first path refuses SC7 with "the first span must
be a line". Every arc is an exact quarter or half turn about an axis parallel
to a coordinate axis: `sweepinput.CompositeChainSweepPath` reads the recorded exact axis and
turn fraction, never a rounded angle, so a near-quarter arc cannot pass. Those
are the turns whose sine and cosine are exact, and the transported section
frames must then carry zero displacement; any frame that does not refuses
SC7. The gate names the failing span. Without it, the zero-displacement check
still refuses such an arc, but names the frame rather than the arc.

**Spans.** Every span builds from the ONE authenticated `ChainRecord`: a
`LineTo` span through the chain prism builder over the path's own signed
height, an `ArcThrough` span through the chain revolve builder over the arc's
exact centre, axis and turn. Each span keeps its own gates. A line span must
follow its transported section normal; an arc span runs the chain shell's
axis-side gate (SC8), so a walk that crosses an arc axis refuses there. The
source walk itself is not pinned to a shape: a slanted line or a multi-segment
line walk is admitted when every span's own gates and the audits below pass.

**Rims and sew.** Each span publishes one wall per coalesced walk of the
recorded chain, in walk order. Every wall must have one loop of four edges,
and its two path rims must be `Line3` edges; a curved source segment refuses
at that check, as does a wall with a free end on an arc axis. Wall `j` of span
`k` and wall `j` of span `k+1` must carry the same recorded-segment roles,
which is the §15.1 index read off the wall faces; a span with a different wall
count refuses. At each join, wall `j`'s end rim on span `k` is replaced by
wall `j`'s start rim on span `k+1`, and both rims' posts are paired by source
endpoint order, including the arc-axis reversal flag. One old post may pair
with only one new post. The coordinate-bound comparison
(`compositesweep.CertifyRimPair`) can only reject a mismatch; it does not admit
one. Each shared edge is traversed in opposite directions by its two faces,
and the composite boundary audit checks each edge use and every vertex link
after all joins are sewn.

**Separation.** `compositesweep.Audit` runs over all spans. Adjacent spans
must lie on opposite sides of their shared section plane, read from each
span's chain prism or chain revolve extent along the plane normal. Every
non-adjacent pair must have a strict bounded-box gap. A pair whose boxes
touch or overlap refuses SC7, even where the spans themselves stay apart.

What this proves is that two spans meet only at their shared section. The
adjacent check alone leaves the shared plane itself open: span `k` and span
`k+1` each lie in a closed half-space of the plane `P` through their join
section, so they can meet only inside `P`. A line span meets `P` only in that
join section. A partial revolve of at most a half turn, of a section in one
closed half-plane of its axis, lies in the wedge between its start and end
half-planes, so it meets `P` only in its sections that lie in `P`: the join
section, plus its far section when the turn is a half turn. So beyond the join
section, span `k+1` can reach only span `k`'s start section, and span `k` only
span `k+1`'s end section. Span `k`'s start section is also the end section of
span `k−1`, and so lies in span `k−1`'s box. Contact there makes the boxes of
spans `k−1` and `k+1` meet, and the non-adjacent check refuses. Span
`k+1`'s end section is span `k+2`'s start section, and contact there makes the
boxes of spans `k` and `k+2` meet. If span `k+1` is the last span, that end
section can only reach span `k`'s start section, which the first case already
covers. The line-first gate makes sure every arc has a span before it, so the
first case always applies. For an arc-first path, a half turn's start section
has no earlier span to put it in a box, and two consecutive half turns could
touch there unseen.

Within one span, the per-span builder owns the walls' simplicity. A section
point on an arc axis does not reach this argument: a free walk end on the
axis leaves a three-edge wall that the rim check refuses, an interior on-axis
junction refuses SC8, and a segment lying on the axis mints no wall, so the
wall counts differ and the sew refuses.

**Readings.** The sheet has one wall per walk per span. Its area is the
`boundedAdd` of every wall's own area, and its box joins the span boxes.
`Verify` reports validity undecided: the topology and separation checks do not
certify every swept-wall contact.

**Mesh.** Only the radial line/half-circle class meshes: one authenticated
`LineSeg` whose complete forward walk is `(0,0)` to `(w,0)` on the world XY
sketch, with `0 < w < R`, swept from the origin on +Z by `L > 0`, then along
the XZ half-circle through `(R,0,L+R)` to `(2R,0,L)`. The comparisons `L+R`,
`2R` and `R−w` must hold as exact binary-rational equalities. Other composite
chain sheets refuse at tessellation with `ErrUnsupported`.

The mesh shares the two source-indexed join vertices. The straight face is
an exact quad; the arc face is a half-annulus divided into angular cells.
Exact-turn sine/cosine intervals enclose each circular station. The outer
radius `R` sets the chord sagitta bound; station and source-vertex bounds add
to each face's boundary bound. Exact triangle-area intervals and the live
faces' certified area intervals give `VerifyAll` an area slack. The sheet
publishes no occupied-volume proof. `VerifyNone` keeps identical triangles
and withholds area and contact results. A placed composite sheet refuses until
its transformed join-plane and station bounds have their own proof.

**Thicken.** `Thicken` admits an unplaced composite chain sheet on its three
thickness sides through `agent-docs/surface-design.md` §16.8's generated
section. Its one source walk must be in that document's §16.6 class:
axis-parallel lines with right-angle corners (R44). A slanted source line
refuses with "the open walk is not axis-parallel". Every arc span's own axis
feeds that document's §16.5 radial gate, so an offset that reaches or crosses an arc axis refuses. Other chain
sweeps still refuse at that consumer.

### 15.9 One-arc chain sheet and mesh

A one-span `ArcThrough` path passes the same `sweeparc.Derive` and chain
`Revolve` axis gates as an arc span of §15.8. The authenticated Sketch chain
enters that reduction unchanged, and the result has its chain Revolve wall
faces and free rims. A partial arc reports `ValidityUndecided`, as a partial
chain `Revolve` does. `Transform` and `Copy` replay the same reduction under
their composed placement.

`Thicken` sends this sheet through its chain `Revolve` reduction when
§16.7's exact-axis and generated-section gates admit it. The result carries
a closed profile in a solid one-arc `sweepPayload`; its wall and cap roles
follow the original path order. See `agent-docs/surface-design.md` §16.8.

The mesh admits one `LineSeg` whose complete re-expressed walk has both ends
strictly clear of the resolved axis. The recorded line and bounded path axis
give two certified endpoint intervals in axial coordinate `z` and radius `ρ`.
Both vary affinely along the line, so positive endpoint radius intervals keep the whole source
line on one side of the axis. Each angular cell's four exact corner points
are coplanar; the chosen diagonal adds no meridian chording error. The
denoted angle enclosure gives an outward sweep-angle upper bound, and the
larger true endpoint-radius upper bound sets `ChordSagitta` for every point
of the wall. `SweptPointGap` charges path-axis, source-endpoint, angle and
placement errors at every stored vertex. Its maximum plus the sagitta must
fit the requested tolerance.

The one source wall owns two triangles per angular cell and four free edges.
The sheet and positive-facet audits run at every verification level;
`RevolveContactAudit` runs at `VerifyBoundary` and `VerifyAll`. `VerifyNone`
keeps the same triangles but reports `BoundaryVerified() == false`.
`VerifyAll` sums the non-cancelling `CellSlack` over the angular cells and
adds a per-cell Jacobian difference for path-axis, source and denoted-angle
intervals. For one cell, true density is `Ltrue * stepTrue * ρtrue(t)`.
The added bound is `(LtrueUpper*δstep + stepHeldUpper*δL)*ρtrueUpper +
LheldUpper*stepHeldUpper*δρ`, where each `δ` is an outward interval difference
and `δρ` is the larger endpoint difference. The line makes that endpoint
maximum valid for every `t`. `CoordinateAreaAllow` charges held vertices.
The sheet publishes no occupied-volume proof. A source with a curved segment, several segments or
an endpoint on the axis may build analytically, but its mesh refuses until
it has a separate station and boundary proof.

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
the assembled triangle set before commit, exactly as `agent-docs/loft-design.md`
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
`agent-docs/loft-design.md` §5 does, then orient the whole shell once by the sign
of the exact tetrahedron sum over the rational vertices; a negative sum
reverses every triangle. Never orient a wall or a cap on its own.

### 16.4 Table SM — what refuses a mitred polyline sweep

Every row lands on `agent-docs/api-design.md` §12's vocabulary. Gate order is
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

A wall is ONE face, not a loft's two triangles: `agent-docs/loft-design.md` §5
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
| **DM2** | `Tessellate` / STL / OBJ / 3MF / faceted STEP | lands with M1 as an exact restatement: the held triangles of Table BM at any tolerance at or above `delta` (below it, `ErrUnsupported`, `agent-docs/tessellation-design.md` §7's rule), each vertex's own rounding gap as its `vertexBound` and `sourceBound(face)` the largest of them over that face's triangles (`agent-docs/faceted-vertex-bounds-design.md` §2.1), with `Bound` `delta`, `areaSlack` the per-triangle perturbation at `delta` (`perturbedTriangleAreaAllow`, the `stitchPayload` row's term), `volSymDiff = sweptVolumeAllow(delta, area upper)` with `symDiffOK == true`, and the boundary proof the build's own SM8 certificate. The implementing PR adds this payload's row to `agent-docs/tessellation-design.md` §2's table |
| **DM3** | mesh booleans | lands with M2: an operand on `agent-docs/tessellation-design.md` §11's terms. Two branches that cross carry `delta > 0` on both sides, so the hidden-tangency gate runs with slack `2·delta` and admits the pair only through a proven deep witness, which `boolean.go`'s witness search finds by sampling along each contact segment. Walls that are coplanar with the other operand's, as two branches sharing a span are, stay refused by that gate. A `Cut` whose tool is a mitred sweep wholly inside the target, such as a thinner bore through a trunk's corners that stops short of both caps, publishes the bore as a separate void shell (`agent-docs/api-design.md` §6), and a later `Union` whose operand stays clear of the bore keeps that shell whole. A holed profile's passage cannot be capped into a void by a `Union` with end plugs, because a plug flush with the tube's cap is a coplanar contact this gate refuses |
| **DM4** | interference | lands with M2: the partition through the exact planar arm of `agent-docs/interference-design.md` §3.2 against a prism, a stitched solid, a faceted result or another mitred sweep, the overlap volume through DM3's mesh path |
| **DM5** | clearance | box separation at once; `WithClearances` reads the exact planar arm's gap, the held gap widened by `delta`, against the partners DM4 names, and stays `Suspect` against every other payload |
| **DM6** | `Wall`, `Undercut`, `ConcaveRadius` | `Unavailable` with `DiagUnsupportedSurveyPayload`, as D6 |
| **DM7** | `Transform`, `Copy` | re-runs §16.3 from the record under the composed motion, applied to the rational vertices exactly (a transform's entries are floats, hence rationals), rounds once, recomputes `delta`, re-decides the shell orientation and re-runs SM8; `delta` never accumulates across placements |
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
| **M1** | `WithMitredJoins()` and `WithSectionScale` over `LineTo`-only paths and `LineSeg`-only profiles: §16.3's construction, Table SM, Table BM, §16.6's four readings, DM1, DM2, DM7, the executable example, and the `agent-docs/tessellation-design.md` payload row | DM3 and DM4 until the witness fix lands; `WithSurfaceResult()`, arc spans, curved profile segments |
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
  watertight; `Transform` under a rotation reproduces `Volume` exactly.
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
- `agent-docs/layout/design-docs.md` row for this document names §16; the implementing PR
  adds rows for the files that build the mitred sweep.
- The implementing PR adds the payload's row to `agent-docs/tessellation-design.md`
  §2's table and to `agent-docs/payload-verification-design.md`, and changes
  `doc.go`'s support map.
- `agent-docs/api-design.md` needs no edit: `Sweep`'s signature is unchanged and §8
  already names this document as the owner of its options and reach.

## 17. Bounded twist

### 17.1 One straight span

`WithSweepTwist` admits one whole, strictly convex polygon with 3–256 straight
edges and no holes, drawn on the origin XY sketch plane. Its exact shoelace
centroid must lie on the Z axis. The path is one positive-Z `LineTo` from the
origin, and the absolute denoted angle is at most one radian. Sheets, mitred
joins, section scale, other paths and other profiles remain S11 refusals.
The preflight uses exact rational coordinates and commits no body on refusal.
Section 17.3 extends this straight-path profile class to one full turn with
multiple certified Loft cells.

At height `h`, the true section is the input polygon rotated by `theta*z/h`.
Every section is a rigid copy, and the positive Z coordinate keeps distinct
sections disjoint. Thus the true volume is exactly profile area times `h`;
the centroid is `(0,0,h/2)` before placement. The wall area is the integral
over each edge `p(s)=p0+s*v` of
`sqrt(h²*|v|² + theta²*(p(s)·v)²)`. Exact rational intervals enclose the
angle, square roots and inverse hyperbolic sine in its antiderivative. Both
planar cap areas are added to this enclosure.

The held boundary is a certified Loft between the original polygon and its
rotated endpoint. Its two wall triangles per edge stay in the mesh but form
one live `Faceted` Face with `side(0,0,j)` provenance. Its internal triangle
diagonal is not a public Edge. The cap Faces retain `capStart` and `capEnd`.
The endpoint trigonometric interval encloses the frame Loft actually uses.
For maximum profile radius `R`, edge length `L`, angle `theta`, and endpoint
coordinate error `e`, the true wall's departure from the ideal Loft facets is
bounded by `R*theta²/8 + L*(|theta|+2e)/4 + 2R*e`.
The mesh `Bound` adds Loft's certified held-vertex displacement. A tolerance
below that bound returns `ErrUnsupported`.

Each horizontal section of the true twist and ideal Loft differs only within
a two-sided boundary tube. For polygon perimeter `P` and ideal displacement
`d`, its area is at most `2P*d + 2*pi*d²`; integrating over `h` bounds their
occupied-volume difference. Loft's own occupied-volume proof then covers the
ideal facets to the rounded, placed mesh, including axial rounding. The two
bounds add without cancellation. The area allowance adds the certified
true-to-Loft area interval gap to Loft's mesh area allowance. Consequently
`Tessellate(VerifyAll)` may publish boundary and volume verification for this
case; boolean consumers receive the same occupied-volume certificate.
Placement replays the recorded Sketch and path under the new transform and
charges the plane-local readings by the placed basis (§17.5).

The real public acceptance uses Sketch's solved centred rectangle, a 5 mm
positive-Z line and a 30° twist. It measures 20 mm³ of true volume while the
held 12-triangle shell encloses about 15.77 mm³, so a Loft measurement cannot
stand in for the twisted solid. It checks six live Faces, one wall Face per
profile edge, all live source Faces, placement replay, and `VerifyAll` at
0.5 mm. A thin placed case checks that axial coordinate rounding is included
in the occupied-volume allowance. A negative-angle case and each S11 refusal
complete the focused gates.

### 17.2 A line and a cardinal quarter arc

The same centred, strictly convex polygon may follow a positive-Z line of
height `h`, then a cardinal quarter arc in the XZ plane. The arc has centre
`(R,0,h)`, radius `R`, starts at `(0,0,h)` with positive-Z tangent, and ends
at `(R,0,h+R)` with positive-X tangent. The exact recorded circle and angle
must prove those facts. The admitted profile has `R > 8r`, where `r` is its
outward-rounded maximum vertex L1 radius. The absolute denoted twist remains
at most one radian. Other curved paths remain S11 refusals.

Twist is distributed by arclength. If `L = h + pi*R/2`, the join angle is
`theta*h/L`; at arc angle `phi`, the section angle is
`theta*(h+R*phi)/L`. The true arc section has centre
`(R*(1-cos(phi)),0,h+R*sin(phi))`. Its local radial axis is
`(cos(phi),0,-sin(phi))`, and its other axis is positive Y before the section
twist. Because every profile point has radial coordinate less than `r<R`,
the arc's distinct sections occupy distinct polar angles. Their Z coordinate
is strictly above `h` after the join, so they meet the straight span only
at the shared section.

The exact profile centroid is on the path axis. Integrating the arc's
Jacobian `1-a/R` over each section therefore gives true volume `A*L`, where
`A` is the exact profile area. The centroid's centreline moment is enclosed
with rational pi bounds. Its section correction is at most `r²/R`.
The straight wall uses §17.1's area integral at the join angle. The arc wall
lies between `P*(R-r)*pi/2` and
`P*((R+r)+r*abs(theta)*R/L)*pi/2`, with outward perimeter bounds.
Both endpoint cap areas are included once.

One Loft holds the line, and eight Lofts hold equal-angle arc cells. Each
cell's section frame is enclosed by rational sine and cosine intervals; a
stored station carries its measured origin and frame error. The Lofts use
those same station frames on both sides of every join. Their cap rims are
sewn by exact station coordinates, since Loft's start and end cap walks have
opposite coedge order. Each pair of wall triangles becomes one quad; the
eight arc quads for each profile edge merge into one live `Faceted` Face
with `side(1,0,j)` provenance. The line retains `side(0,0,j)`. Internal
section edges are absent within the arc, and the line-to-arc section edge
remains. The held mesh keeps all triangles and removes only internal caps.

For one arc cell of angle width `dphi` and twist increment `dbeta`, the
true wall's departure from the ideal Loft triangles is enclosed by a
centreline interpolation term `R*dphi²/8`, a section rotation term
`r*(dphi+abs(dbeta))²/8`, a triangle diagonal term
`e*(dphi+abs(dbeta))/4` for maximum profile edge length `e`, and the
endpoint station errors. Its stored facet bound adds Loft's own placement
and vertex rounding. A tolerance finer than the resulting maximum returns
`ErrUnsupported`.

The occupied-volume proof uses a pointwise homotopy from each true curved
wall patch to its corresponding ideal Loft triangle. The wall's swept
volume is at most its maximum departure times the product of proven bounds
on its profile-edge and path-parameter derivatives. Both cap homotopies
are charged by their perturbed triangle areas. The eight nonnegative cell
allowances add to §17.1's line allowance and each Loft's own
ideal-to-held allowance. The build also audits exact held-facet contact
across cells, watertight edges, vertex links and positive orientation.
It refuses a total allowance as large as the true body volume; the
`VerifyAll` certificate cannot permit the entire body to disappear.

The public Sketch acceptance uses a solved 0.2 mm square, a 1 mm line,
an R=5 mm quarter arc and a 5° twist. Its true volume is about
0.3541592654 mm³; the 76-triangle held mesh encloses about
0.3510007989 mm³. The published local occupied-volume allowance is
about 0.256 mm³ and covers the observed 0.00316 mm³ signed gap. It checks
ten live Faces, source roles, placement replay, and `VerifyAll`.

### 17.3 Stationed straight-path twist

The same centred, strictly convex 3–256-edge polygon and positive-Z line
admit a nonzero absolute angle above one radian and at most one full turn.
The denoted angle is compared with the upper rational bound on `2*pi`.
Other path, profile, sheet, mitre and scale inputs retain §17.1's refusals.
The build commits no body when a station or proof gate refuses.

The builder chooses 2–64 equal-height cells. Each cell's angle is at most
one radian, and §17.1's one-cell departure estimate must be at most four
percent of the profile's maximum L1 radius. Before building a cell, it
counts `2*edges*cells + 2*(edges-2)` held triangles and refuses a count
whose pair count exceeds the existing 8,000,000-test ceiling. An input
needing more than 64 cells also refuses. These are construction limits;
the completed boundary proof still decides the caller's mesh tolerance.

Station `k` uses exact height `h*k/cells` and the interval enclosure of
`theta*k/cells`. Rational sine/cosine intervals enclose the true frame;
the held `r3.Frame` and the height conversion contribute a bounded point
error. Adjacent Lofts receive the same held station and sew their cap rims
at exactly equal vertices. Each cell's two wall triangles per source edge
remain in the mesh. The cell walls merge into one live Face per source edge,
with `side(0,0,j)` provenance; the two outer caps retain their roles.
The sewn mesh checks edge uses, vertex links, orientation and facet contact.

For profile maximum L1 radius `R`, maximum edge L1 length `E`, cell angle
`a`, and maximum adjacent station point error `e`, the true wall's
departure from the cell's ideal two-triangle Loft is at most
`R*a*a/8 + E*a/4 + e`. The public boundary bound adds that cell's Loft
vertex and placement displacement, then takes the maximum over cells.
A horizontal true section and its ideal Loft section differ inside the
two-sided tube of this cell departure. For perimeter upper bound `P` and
cell height `h/cells`, its occupied-volume allowance is
`(h/cells)*(2*P*d + 2*pi*d*d)`, where `d` is the cell departure. The
comparison Loft uses the exact source heights. A held station may round
its height; moving the comparison Loft to the held heights moves each
cell vertex axially by at most the larger endpoint height error `ez`.
The build adds `ez*PerturbedAreaUpper(held triangles, ez + Loft delta)`
for the area every closed cell triangle can visit during that move.
The horizontal tubes, axial moves and each Loft's ideal-to-held
occupied-volume proof add across cells. The build refuses if the total reaches the exact
true volume `area*h`; a signed held-volume gap cannot replace this
symmetric-difference allowance.

The true volume, centroid and wall-area interval use §17.1's rotating
section formulas for the full angle. The mesh area interval is computed
from its held triangles and compared with the true area interval for
`VerifyAll`. A mesh tolerance below the maximum boundary bound refuses;
placement replays all stations and proofs under the placed transform and
charges them by the placed basis (§17.5).

The public acceptance sweeps Sketch's solved 2×2 mm square for 5 mm with
two radians. Fourteen cells give a 0.07653061224492375 mm boundary bound
and a 6.306450007046607 mm³ occupied-volume allowance, below the exact
20 mm³ volume. The held mesh encloses 18.98294353769568 mm³. It passes
`Tessellate(0.1 mm, VerifyAll)` before and after placement. Negative twist,
a full turn and the over-turn and work-ceiling refusals are focused cases.

### 17.4 Centred rounded profiles on one straight path

A straight positive-Z path also admits a solved, hole-free Sketch profile
with whole Line and Arc segments when its chord outline passes §17.1's
strict-convexity check and every segment has an exactly opposite partner
under a half-turn around the path axis. Each Arc has equal exact squared
endpoint radii, positive radius, and a certified turn no greater than π.
Other curve kinds, partial edges and mixed profiles without this symmetry
return `ErrUnsupported` before a body is committed. The paired source walks
make the true section's two first moments zero. Increasing Z keeps distinct
rotated sections disjoint.

The profile area reading supplies an interval for true volume `area*h` and
both cap areas. Circular-walk intervals supply each Arc length and radius.
For source perimeter `P` and maximum source radius `R`, the true wall area
lies between `h*P_low` and
`h*P_high*sqrt(1+(|theta|*R/h)^2)`. This follows from the rotating wall's
arc-length Jacobian `sqrt(h²+theta²*(p·p')²)` with `|p'|=1`.
The body area interval adds the two cap-area intervals. The mesh area
allowance keeps a positive source-plus-held charge for both caps. For each
wall cell, the authenticated Loft pairing records source arc length,
tangent-deviation energy and parameter-matched chord departure. The two
emitted held facets keep their cell and side IDs. These readings bound the
integral of the absolute true-versus-held area-density difference over the
cell's shared source parameters, with outward rounding for station and
vertex error. Summing the nonnegative cell bounds also covers any matched
parameter subset kept by a Boolean. The Boolean separately charges cut
boundary movement through its faceted-area geometry allowance.

The existing stationed build evaluates each cell with the authenticated
curved profile through Loft. It groups the Loft's curved wall triangles by
source segment and sews cells at the full station table. For a cell angle
`a`, maximum straight-edge L1 length `E` and station error `e`, the true
twist-to-ruled-Loft allowance is `R*a*a/8 + E*a/4 + e`. The public boundary
bound adds Loft's `FacetDeparture`, which covers each source Arc against
its chord facets. The horizontal symmetric-difference tube uses the true
perimeter upper bound. The volume allowance adds every cell's tube, axial
station move and Loft `VolSymDiff` without cancellation. The curved class
first finds the least count at a three-percent-of-radius planning target. It
optionally tries the least count at a 0.08 mm target when that is finer,
within the same 64-cell and facet-pair ceilings. If the finer count exceeds
a ceiling or its own Loft, contact or volume proof refuses, the evaluator
builds the original certified count. Each successful build publishes its
own computed boundary and occupied-volume bounds. A caller's finer mesh
tolerance still refuses.

The first refined Loft cell supplies a skip-only estimate of the full
arc-chorded mesh's facet-pair cost. The complete candidate still receives
its own audit, since later cells can have different chord counts. A late
refined refusal can cause at most two builds, each subject to its own
64-cell and facet-pair ceiling. It does not imply a single shared work
allowance across both attempts.

The public acceptance builds a 4×2 mm rounded rectangle with 0.5 mm
corners in Sketch, sweeps it 5 mm with a 30° twist, and asks for
`Tessellate(0.1 mm, VerifyAll)`. Its true volume is
`5*(7+π/4) mm³`; the mesh reports both boundary and volume verified.
The focused tests also cover all eight source wall roles, placement,
negative twist and an off-axis source refusal without a document commit.
An 8×6 mm rounded rectangle with 1.5 mm corners and a 30° twist uses the
optional finer count and passes `Tessellate(0.1 mm, VerifyAll)` with a bound
below 0.08 mm. The finer build took 0.522 s versus 0.209 s for the original
build in the measured local probe. A 1.4-radian twist of that profile keeps
the original certified mesh when the finer construction reaches the
facet-pair ceiling; the first-cell skip reduced that build from 1.57 s to
0.61 s in the measured local probe.
An overlapping Boolean raises its pair tolerance to this Sweep's certified
held boundary. The rounded-profile example unions with an overlapping
1.5×0.8×1 mm extruded box and passes `VerifyAll` at 0.2 mm. Its result's
refined mesh bound is about 0.111778 mm, so a 0.1 mm request refuses. The default
`Document.Verify` still reads `Suspect`. The caps retain coarse area charges,
and the source occupied-volume, centroid and bounds proofs do not yet meet
that separate oracle's thresholds. The rounded example's local wall charge
is about 6.78 mm²; its complete mixed-face allowance is about 37.92 mm²,
compared with about 145.71 mm² under the former positive wall charges.
The real overlapping Union carries that allowance through a cut Arc wall
and passes `VerifyAll(0.2 mm)` with a result area bound about 58.76 mm².

### 17.5 Placement charge

Every twisted sweep evaluates its true volume, wall-area integral, boundary
departure and occupied-volume tube in the coordinates of the origin XY sketch
plane, the only plane §17.1–§17.4 admit. The record denotes that plane-local
solid carried through the placement's held basis B, read as exact rationals
(`massmoment.PlacementRotation`). r3 keeps B orthonormal only to rounding, and
a placement composed through `r3.Transform.Then` drifts further with every
composition: 5000 composed float rotations hold a B with
`det B − 1 ≈ −5.6e-13`, which moves the 20000 mm³ volume of a 100×100 mm square
swept 2 mm by about 1.1e-8 mm³ while its closed form reads `Exact`.
Each of the three builders (`Eval`, `evalStationed`, `evalComposite`) reads
B's `massmoment.MapCharge` (`agent-docs/evaluator-design.md` §5.1) and widens
every plane-local reading before anything composes it:

- The exact volume interval widens by `||det B| − 1|·max(|lo|, |hi|)` on each
  side (`MapCharge.VolumeInterval`), since B scales every volume by exactly
  `|det B|`.
- The true area interval widens by `e·max(|lo|, |hi|)` on each side
  (`MapCharge.AreaInterval`), for B's orthonormality defect `e`. The body's
  `Area`, the true-to-held area gap in the mesh's `AreaSlack`, and every
  face's area bound, which is wider than the body's total area, all read the
  widened interval. The rounded-profile class's wall and cap allowance reads
  plane-local arc lengths, energies, radii and heights while
  `TwistMixedWallAreaL1` evaluates it on the placed held vertices, so the
  allowance itself takes the factor `(1 + e)` and then adds `e` times the
  true area's upper end (`MapCharge.AreaSlackOf`).
- The boundary displacement `Delta` and the occupied-volume allowance
  `VolSymDiff` take the length charge `(1 + e)` and the volume charge
  `(1 + ||det B| − 1|)` over their whole totals. Both mix plane-local terms
  with the held Lofts' own placed terms; the placed terms are over-covered by
  the same tiny factor.
- The line-and-arc centroid's plane-local enclosure, path moment and section
  correction take the length charge before `Apply`'s rounding allowance is
  added. The straight-path centroid `(0, 0, h/2)` is exact before placement,
  and an affine map carries it to the placed centroid exactly.

An exactly orthonormal B, such as the identity a translation holds, reads both
charges as zero, and every reading stays bit for bit. The focused fixture
places each builder, and the rounded-profile class, under 5000 composed
rotations. It checks that each
published volume meets `|det B|` times the unplaced enclosure. It checks that
each published area, body and face, meets the unplaced enclosure scaled by
the Gershgorin range of `cof(B)ᵀcof(B)`, which holds the squared area factor
of every plane.
