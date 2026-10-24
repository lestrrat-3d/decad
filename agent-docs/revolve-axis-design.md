# General-axis copied-tooth Union

This design extends the finite Revolve and point-Loft sector class
(`agent-docs/revolve-loft-sector-design.md`) and its point-cone trims
(`agent-docs/loft-point-design.md` "Coaxial finite cone trims") from the world
+X axis to any axis through the world origin that lies in a world coordinate
plane. The +X class is the special case in which every new charge is exactly
zero, and its arithmetic is unchanged. Nothing here is specific to gears: the
class is any Revolve blank, one cone-trimmed point-Loft sector and rigid copies
of that sector.

Until the PRs in §7 land, the general class is design-only and the code refuses
it (`agent-docs/missing-features.md`, Booleans).

## Question router

Navigation only; each section owns its content.

| Question | Section |
|---|---|
| Which code reads world +X today? | §2 |
| How is the axis represented, and which predicate selects +X? | §3 |
| What does the general class admit and refuse? | §4 |
| Why is each new charge sound? | §5 |
| What must a consumer model do? | §6 |
| Which PRs implement it, with which tests? | §7 |
| How is the +X path proven unchanged? | §8 |

## 1. Scope

The class covers these four steps, each about one axis line through the
world origin:

1. `Revolve` of the blank section.
2. `Cut` with a toe cone Revolve and `Intersect` with a heel cone Revolve of a
   point Loft whose apex is the world origin (`revolveplan.PointConeBoolean`).
3. `Copy(..., WithCopyPlacement(m))` of the trimmed tooth, where `m` is a
   rotation about the axis up to rounding (§4.5).
4. Serial and variadic `Union` (`revolveshell.SectorUnion`,
   `NextSectorUnion`, `SectorUnionMany`).

The following stay +X only. Each declines a general-axis record explicitly
(§4.6):

- The coaxial cylinder `Cut` of a joined body (`revolveshell.SectorCylinderCut`).
- `ConeFromFace`.
- The `LoftFaces` cone trims (`loftfacestrim`, `loft_faces.go`).

## 2. Inventory of +X assumptions

Each row names a site, what it reads as +X, and the PR in §7 that generalizes
it. "Canonical" means the +X body stays verbatim behind the canonical branch
(§3.3, §8.1).

### 2.1 Placement gates that stay

The repository holds 114 non-test `r3.Identity()` comparisons. All but the
eleven below sit on other routes (Loft cylinder, mirror, sweep, stitch, shell)
and none of them reads the axis. The eleven route gates require unplaced
records or a placed copy whose `Xform` equals its recorded motion. The general
class is unplaced too, so they stay unchanged:

- `sector_union.go` `SectorUnionMany` (`placed.Xform != ...Motion`).
- `NextSectorUnion` (`jp.Xform != r3.Identity()`).
- `sector_source.go` `RecordedSection` (`rp.Xform != r3.Identity()` only).
- `CertifySector` (`fp.Xform`, `PointCone.Base.Xform`, `PointSection.Apex`).
- `sector_join_build.go` `buildSectorsJoinAtSag` (`sources[0].Xform`).
- `sector_mesh.go` `BuildRevolveSectorMesh` (`motion != r3.Identity()`). This is
  a rounding branch, not a gate.
- `revolveplan.PointConeBoolean` (`fp.Xform`, `PointSection.Apex`).
- `revolveplan.PointConeSource` (`rp.Xform` only).
- `facetedrecord.PlacedWith` (`internal/facetedrecord/placed.go`). It records
  `RevolveLoftSector{Source, Motion}` for any motion and does not read the axis.
- `sector_bore_record.go` `RevolveSectorBoreCapRecord` (`joined.Xform`).
- `sector_bore_cut.go` `SectorCylinderCut` (`prism.Xform`).

### 2.2 Axis gates

| Site | Function | +X reading | PR |
|---|---|---|---|
| `internal/revolveshell/sector_source.go` | `RecordedSection` | Frame origin 0, `U = X`, `V = Y`; `Ax` `AU = AV = 0`, `DU = 1`, `DV = 0`, all bounds 0 | 3 |
| same | `RootCandidates` | `cone.Axis == {X:1}`, `Origin.Y = Origin.Z = 0`; apex read as `Origin.X` | 3 |
| same | `CertifySector` | reads `Lower`/`Upper` constants as +X; no axis comparison | 2 |
| `internal/revolveplan/point_cone_source.go` | `PointConeSource` | the same frame and `Ax` gate as `RecordedSection` | 2 |
| `internal/revolveplan/point_cone_boolean.go` | `PointConeBoolean` | combines `Lower` and `Upper` with no axis comparison | 2 |
| `internal/revolveproof/sector_fit.go` | `revolveFitMeridianDifference` | `cone.Axis == {X:1}`, `Apex.Y = Apex.Z = 0` | 3 |
| `internal/revolveproof/circle_side.go` | `CircleFragmentSide` | `cone.Axis == {X:1}` | 3 |
| `internal/massmoment/axis_placement.go` | `XAxisRotation`, `XAxisPlacementCharge` | block form about X | 1 (new), 4 (callers) |
| `internal/revolveshell/sector_union.go` | `NextSectorUnion` | `XAxisRotation(motion)` | 4 |
| `internal/revolveshell/sector_placement.go` | `RevolveSectorSourcesSeparate` | `XAxisPlacementCharge` | 4 |
| `internal/revolveshell/sector_join_build.go` | `buildSectorsJoinAtSag` | `XAxisPlacementCharge` | 4 |
| same | same | `ProjectedPatch.Axes = {1,2}` | 5 |
| `internal/revolveshell/sector_bore_cut.go` | `SectorCylinderCut` | prism frame `N = X`, `U = Y`, `V = Z` | declines (5) |
| `internal/revolveshell/sector_bore_record.go` | `RevolveSectorBoreCapRecord` | axis vertices `== {X: U}` | declines (5) |

### 2.3 World X as the axial coordinate, YZ as the radial plane

| Site | Function | +X reading | PR |
|---|---|---|---|
| `internal/pointcone/factor.go` | `FactorDyadic`, `FactorRat` | `g = x + k·hypot(y,z)`, `q = g/a` | 2 |
| `internal/pointcone/cone.go` | `ConstantsOf` | one constant set from the X apex `a` and `k`, used for samples and bounds alike | 2 |
| same | `AboveFarCap` | far plane `x = far`, `p.X − meshBound` | 2 |
| same | `Ordered` | X range, `rho` from `y²+z²`, YZ-box radial minimum | 2 |
| same | `ToeHeelOrdered` | X range | 2 |
| `internal/pointcone/cap.go` | `CapGeometry`, `capGeometry`, `CellGeometry`, `cellGeometryWith` | `xMin` from `p.X`, `rhoMin` from the YZ box | 2 |
| same | `ReachUpper` | the norm of `p` in millimetres; axis-free, kept in millimetres (§3.4) | 2 (checked) |
| same | `TriangleBound` | millimetre `a`, `G`, `h` (homogeneity, §3.4) | 2 |
| `internal/facetedrecord/point.go` | `PointConeLimit`, `Constants` | `Apex`, `Far` are X coordinates; no axis field; one constant set | 2 |
| `internal/facetedrecord/point_cone_helpers.go` | `PointConeFactor(s)`, `PointConeLimitLater/Earlier/Ordered` | through `pointcone` | 2 |
| `internal/revolveproof/sector_geometry.go` | `RevolveMeridianResidual` | meridian `(U,V)` against `Origin.X` | 3 |
| same | `RevolveImplicitDisplacement`, `revolveMeridianExtensionGap` | `world.X > 0`, X range of the fan | 3 |
| `internal/revolveproof/sector_seat.go` | `revolveSeatPoint` | form `y²+z² − s²(x − Origin.X)²`, `sourceForm`, X range | 3 |
| `internal/revolveproof/sector_fit.go` | `revolveFitMeridianDifference` | box `x = point[0]`, tangent `[0]` | 3 |
| `internal/revolveproof/circle_side.go` | `CircleFragmentSide` | `h = q[0]`, `q1²+q2² − s²h²` | 3 |
| `internal/revolveproof/sector_source.go` | `CertifyRevolve{Connector,Fit}Boundary` | `ImplicitMargin` against a cost in mm² (unit axis) | 3 |
| `internal/facetedrecord/revolve.go` | `ConeSection`, `MeridianEquation` | tag axis; `(U,V)` read as world `(X, r)` | 3 |
| `internal/revolveshell/section_tool_gap.go` | `RevolveSectionToolGap` | `U + k·V − Apex` with X-scalar cone limits | 3 |
| `internal/revolveproof/sector_path.go` | `RevolveExteriorPath` | `Angle = atan2(Z, Y)` | 3 |
| `internal/revolveshell/sector_join_plan.go` | `PrepareSectorJoinPlans` | `PlacedRound` charges `Stretch` times the node norm | 4 |
| same | same | `facetproof.AngleAt`; `MinRadius` from `V` | 5 |
| `internal/revolveshell/sector_cap.go` | `revolveFarRootPoint` | generator `{1, s·cos, s·sin}` | 5 |
| same | `RevolveSectorCapBoundary` | `atan2(Z, Y) + 2π` | 5 |
| `internal/revolveshell/section_ring.go` | `BuildRevolvedSectionRing` | vertex `{U, V·cos, V·sin}`; axis vertex `{U,0,0}`; X exact | 5 |
| `internal/revolveshell/section_angles.go` | `RevolveSectionAngles` | `maxRadius` from `V`; angles in the atan2(Z,Y) convention | 5 |
| `internal/revolveshell/sector_cap_refine.go`, `sector_mesh.go` | `SectorConeCapBounds`, `SectorRefineCap`, `BuildRevolveSectorMesh` | through `pointcone` | 5 |
| `internal/facetproof/sector_seam.go` | `AngleAt`, `SectorSeam` | `atan2(Z, Y)` | 5 |
| `internal/facetproof/sector.go` | `sectorPairProjectionSeparate(Float)` | YZ direction (sound for any axis, weak off X) | 5 |
| `internal/loftmesh/loft_projected_patch.go` | `ProjectedPatch` | coordinate planes only (sound, self-checking) | 5 (caller) |

### 2.4 Shared consumers of the +X records

These sites take `PointConeSource` limits or `RecordedSection` spans. They run
+X math on them and must decline a general record (§4.6):

- `cone_from_face.go` (`RecordedSection`, `revolveMeridianCone`).
- `loft_faces.go:128`.
- `internal/loftfacestrim/trim.go:73,143,329`.
- `internal/facetedrecord/trim_checks.go` `CapOutsideToeCone`.
- `CapOutsideHeelCone` and `loftface.ConePullback`.
- `internal/revolveshell/sector_bore_*.go`.
- `internal/revolvemesh/sector_bore.go` (`Axes {1,2}`).

## 3. Axis representation

### 3.1 Decision: the exact axis vector

The class carries the axis as the exact vector `A`: a world `r3.Vec` with float
components, a positive multiple of the axis direction that the Revolve's input
denotes, through the world origin. `A` need not be unit length. Code never
normalizes it.

- Exact predicates are written in A-scaled coordinates. These are the axial
  `s(p) = p·A` and the radial `σ(p)² = |p×A|²`. A cone with apex point `P` on
  the axis has the apex scalar `c = P·A` and the slope `k`, its tan(half-angle).
  Every value is an exact Dyadic or `big.Rat` of stored floats.
- The relation between a ray and a cone is homogeneous in `|A|`. The point-cone
  crossing `alpha(p) = c / (s(p) + k·σ(p))` therefore equals the millimetre
  form `a / (x + k·rho)` exactly, and so does every sign test of the form
  `|(p−O)×A|² − k²((p−O)·A)²`.
- `|A|` enters in two ways only:
  - `A·A`, which is exact, scales a squared millimetre quantity.
  - The bracket `[|A|↓, |A|↑]` = `RatSqrtDown/Up(A·A)` converts a linear one.
    Each conversion takes the side that widens the bound or refuses (§3.4).

Sketch already takes this form. `sketch.ConeSection` reads `Axis` as an exact
rational vector of any length, and `CertifyConeLineCrossing`,
`CertifyConeFitCrossing` and `CertifyConeCurveSide` decide
`F = |Q×Axis|² − Slope²(Q·Axis)²` exactly. With `Axis = A`, every Sketch
certificate is exact for the blank's own axis, and the axis adds no bound to
the certificate path.

### 3.2 Rejected alternatives

- **Bounded unit axis.** One option threads the Revolve's held `(DU, DV)` with
  `DUBound`/`DVBound` through every residual. That turns each exact sign test
  into an interval test and gives Sketch a cone that differs from the blank's.
  Every certificate would then need a cone-to-cone charge. The exact vector
  removes both, because Sketch and decad read the same rationals.
- **Frame-aligned reduction to +X.** This maps the inputs by a held rotation
  `G` with `G·Â ≈ X` and reuses the +X code. No exact rotation takes an
  irrational-length axis onto X. Every reduced fan vertex, meridian point and
  cone constant would therefore gain a rounding charge, and the point-cone
  trim's exact constants would become approximations. Sketch's certificates
  live in the tooth sketch's own frame and cannot be reduced either.

### 3.3 Predicates

Two predicates exist, and nothing else selects a branch.

- **Frame predicate**, on a Revolve record: `coneaxis.FrameCanonical(frame, ax)`
  is true exactly when the record's `r3.Frame` and `revolveaxis.Frame` pass
  today's gate. That gate is frame origin
  `0`, `U = X`, `V = Y`, `Ax` `AU = AV = 0`, `DU = 1`, `DV = 0`, and all four
  bounds `0`. It selects the record-reading branch in `RecordedSection`,
  `SpanFace`, `RootCandidates` and `PointConeSource`.
- **Axis predicate**, on the axis value: `Axis.Canonical()` is true exactly when
  `A` equals `r3.Vec{X: 1}`. It selects the arithmetic branch in `pointcone`,
  `revolveproof`, `revolveshell`, `facetproof` and `massmoment`.

`coneaxis.OfRevolve(frame, ax, axisVec)` builds the axis value of a record. It
takes the record's frame, resolved axis and `AxisVec` as plain values, because
`coneaxis` sits below `facetedrecord` in the import graph and cannot import
`revolveplan.Payload`. A record that passes the frame predicate gets the
constant `A = r3.Vec{X: 1}`; its `AxisVec` is never read. Any other record gets
`A = AxisVec`, and it is refused unless it meets the conditions of §3.6 and
`AxisVec` is a finite nonzero vector in the plane. A `PointConeLimit` stores the axis value of the tool record that made it.

Records that pass the frame predicate therefore always have `Canonical()` true,
and every +X fixture runs both canonical branches. A record outside the frame
predicate whose `AxisVec` is exactly `X`, such as an XZ-plane sketch revolved
about U, runs the general record branch and the canonical arithmetic branch. Its
arithmetic is the +X arithmetic, because that arithmetic reads world
coordinates only.

**Axis comparison.** A blank and a limit, or two limits, match exactly when
their `A` values are bitwise equal. A canonical axis (`X`) never equals a
general one, so a mixed canonical/general pair declines. Two general axes that
are parallel but of different lengths also decline, because the A-scaled
constants would mix units.

### 3.4 Units rule

- Crossing parameters, orderings and signs use exact A-scaled values.
- Bound formulas use millimetres:
  - A lower bound divides its A-scaled value by `|A|↑`. A negative value
    divides by `|A|↓`.
  - An upper bound divides by `|A|↓`.
  - An apex scalar that enters a bound numerator is
    `a↑ = RatUp(c / |A|↓)`.
- `ReachUpper` and the cell size `h` are distances `|p|` and `|p−q|`. They are
  already millimetres and take no conversion.

`ConstantsOf` therefore yields two sets for a general limit:

- sample constants from the scaled `(c, k)`, used by `FactorDyadic`;
- bound constants from `(a↑, k)`, used by `TriangleBound`.

For a canonical limit both are the current single set.

`TriangleBound` is not homogeneous, because `h` and `reach` are millimetres.
Its general inputs are:

- `xMin = s_min/|A|↑`, or `/|A|↓` when negative;
- `rhoMin = σ_min/|A|↑` (§3.5);
- the bound constants.

The canonical branch skips every conversion. The predicate (§3.3) decides
that, not the value of `A·A`.

### 3.5 Radial lower bound

A cap cell's lower bound on `σ` uses an exactly orthogonal pair. The admitted
axis lies in a world coordinate plane with normal `e_n`, so `A_n = 0`. Let
`N_A = A × e_n`, which is the in-plane normal of `A`. Then
`p×A = (p·N_A)·e_n + p_n·(e_n×A)`, and the two terms are orthogonal, so

`σ(p)² = (p·N_A)² + p_n²·(A·A)`.

Both functionals are linear, so over a triangle each range comes from the three
vertices. The bound is

`σ_min = RatSqrtDown(minAbs(range p·N_A)² + (A·A)·minAbs(range p_n)²)`.

It loses nothing against the true distance from the cell's coordinate box to
the axis. For `A = X` the canonical branch keeps the current
`max(minAbs(y), minAbs(z))` rule unchanged.

The identity `p·(A×e_k) ≤ |p×A|` holds for any axis, but taking the largest of
the three components can lose a factor of `√3`. That loss would inflate the
radial term of `TriangleBound` by up to `√3`, and an oblique axis would then
refuse more cap cells. The design does not use it. An axis outside a coordinate
plane is refused (§4.1), so the exact pair above always exists.

### 3.6 Recording `A`

`revolveplan.Payload` gains `AxisVec r3.Vec`. `Revolve` and the chain Revolve
set it when all of the following hold:

- The frame origin is the world origin.
- `U` and `V` are signed world basis vectors. The frame map `L` is then an
  exact signed permutation.
- The axis anchor is the plane origin: `AU = AV = 0` with zero bounds. Every
  axial coordinate the Revolve record holds is then measured from the world
  origin, so no anchor conversion exists anywhere in the class.
- The plane-local direction is a stored float pair, with no subtraction:
  - `SketchLine`: `Start` must be `(0,0)`, and the direction is `End`.
  - `ConstructionAxis`: `Origin` must be the world origin, and the direction is
    `(Dir·U, Dir·V)`. That pair is exact, because `U` and `V` are signed basis
    vectors.

`AxisVec = side·L·(du, dv, 0)`, where `side` is the sign `ResolveSide` applied
to `DU` and `DV`. Negation and a signed permutation are exact. A component that
would be `-0` is stored as `+0`, so two records of one axis compare bitwise
equal. Otherwise `AxisVec` is zero, and the general class refuses the record.
`coneaxis.RecordedVec(frame, input, ax, side)` derives it, and `Revolve` and the
chain Revolve call it.

`AxisVec` belongs to the frame and axis together, as `RadialProof` does. A
path that replaces either clears it. Placement keeps it, and the `Xform` gate
refuses a placed record.

### 3.7 The angular frame

Angles about the axis are construction quantities. The general class measures
them in the blank's own `revolvemesh.RevolveBasis`:

- `W = U·DU + V·DV`, the held axis direction;
- `E0 = −U·DV + V·DU`, the left normal of `W` in the sketch plane, which is
  angle 0;
- `E1 = W × E0`, which is angle π/2.

An angle is `atan2(p·E1, p·E0)`, so the branch cut lies along `−E0`. Ring
vertices come from `RevolveLift.Point`. All of this is `r3` arithmetic, and no
frame is written by hand. §5.6 charges the basis's departure from an exact
orthonormal frame about `A`.

## 4. Admission

### 4.1 Blank (`RecordedSection`, `RootCandidates`)

Unchanged gates: the record is full, solid, with no holes, `SectionDelta = 0`,
`Xform` identity, 4 to 128 whole lines, no duplicate points, a closed walk, and
four or more corners found by exact forward collinearity.

A record that passes the frame predicate takes the current branch. Any other
record needs `OfRevolve` to succeed (§3.3, §3.6) and then reads:

- Each recorded point has exact plane-scaled coordinates
  `Z = p·A_plane`, `R = cross2D(A_plane, p)`, measured from the origin, which
  is the axis anchor.
  - `Z > 0` replaces `U > 0`.
  - `R ≥ 0` replaces `V ≥ 0`.
  - The axis span is the one span with `R = 0` exactly at both ends.
  - A point the Revolve snapped onto the axis has `R ≠ 0`, so it is not an
    axis point and the section refuses.
- `SpanFace` rebuilds each wall's surface from the record's own axis walk
  (`rp.Ax.Walk` of the plane walk). The build used that same walk, so face
  matching stays an exact equality.

`RootCandidates` reads the cone from exact data, not from the rounded
`surfacegeom.Cone` tag:

- slope `k* = (R1−R0)/(Z1−Z0)` and apex scalar `Z_a = Z0 − R0/k*`;
- apex point `P* = (Z_a / A·A)·A`.

It requires all of the following:

- a finite slope with `k* > 0`;
- `Z_a` strictly below both span ends;
- the tag is a `Cone`, which binds the face.

### 4.2 Reported cone

`ConeSection()` returns `{Apex: float(P*), Axis: A, Slope: float(k*)}` and is
stored on the certificate. Sketch certifies against exactly this cone. §5.2
charges `ConeGap`, the distance between it and the true root cone, over the
fan's axial reach.

### 4.3 Cone tools (`PointConeSource`)

Unchanged gates: a full, unplaced, three-segment straight meridian, four
on-axis endpoints and two rim endpoints.

A record that passes the frame predicate takes the current branch and gets the
canonical axis. Any other record needs `OfRevolve` and then reads:

- On-axis means `R = 0` exactly, as in §4.1.
- `Apex = c = P_apex·A` and `Far = c_far` are A-scaled.
- `Slope = (c − c_far)/R_rim` is exact and homogeneous.
- `c > 0` and `c_far < 0` hold as today.

`PointConeLimit` gains the field `Axis coneaxis.Axis`. `PointConeSource` is
the one constructor that sets it.

### 4.4 Tooth and its trims (`PointConeBoolean`, `CertifySector`)

- The apex at the origin, the identity `Xform` and the current-source gates stay.
- `PointConeBoolean` admits a new limit only when its axis matches the axis of
  the limit the record already holds (§3.3). A toe about X and a heel about Y
  decline.
- `CertifySector` admits a tooth only when `Lower.Axis`, `Upper.Axis` and the
  blank's axis all match (§3.3). This check runs before any boundary
  certificate is requested. `SectorUnion`, `SectorUnionMany` and
  `NextSectorUnion` all reach the sector route through `CertifySector` or
  through a join certificate that passed it, so the check covers every entry.
  A +X blank with a Y-axis tooth trim declines.
- `RevolveImplicitDisplacement` requires `p·A > 0` for every fan vertex.

### 4.5 Copies (`AxisRotation`)

`massmoment.AxisRotation(m, axis)` returns `(MapCharge, ε, ok)`.

- For a canonical axis it is `XAxisRotation` and returns `ε = Stretch`.
- For a general axis it admits:
  - a valid, non-reflecting `m` with zero translation;
  - `MapChargeOf(m).Stretch < 1/2`;
  - `M·A·A > 0` exactly, where `M` is the exact rational linear part;
  - `s_φ ≤ DriftSinLimit`, where `s_φ` is the proven upper bound on
    `sin∠(MA, A)` (§5.5) and `DriftSinLimit = 2⁻⁴⁶`, which is 64 units in the
    last place of 1.

`DriftSinLimit` admits the drift that rounding leaves in `r3.Rotation` about a
float-normalized `A`. That drift is a few units in the last place. The limit
declines a deliberate tilt. A declined copy makes `AxisRotation` return
`ok = false`, and the Union route declines (matched false), as it does today
for a copy that `XAxisRotation` rejects. The drift that passes is charged
(§5.5), never assumed away.

### 4.6 Declines outside the class

- `ConeFromFace`, `loft_faces.go`, `loftfacestrim` and `CapOutside*` decline a
  `PointConeLimit` or section whose axis is not canonical. They keep +X math
  and never see a general record. PR 2 adds the limit declines, and PR 3 adds
  the section decline in `ConeFromFace`.
- `SectorCylinderCut` declines a joined record whose certificate axis is not
  canonical (PR 5). The pair then takes ordinary dispatch.
- From PR 3 until PR 5, `SectorUnion`, `NextSectorUnion` and
  `SectorUnionMany` decline a non-canonical certificate (matched false). A
  general set therefore keeps its current route and never receives an error
  that a later PR would remove.

## 5. Soundness

Each subsection states what the general class charges, why the charge covers
the displacement, and why the +X class charges exactly zero. No check below
admits by a small residual. Every admission is an exact predicate on recorded
data, a Sketch certificate, or a limit on a proven upper bound that only
declines. Every residual or measured gap is charged into a published bound or
refuses (CLAUDE.md, "A decad-side check may only FALSIFY").

### 5.1 Exact predicates

The following stay exact rational statements about stored floats, as `V = 0`,
`U > 0` and `x > far` are today:

- the axis span (`R = 0`), the side and axial signs (`R ≥ 0`, `Z > 0`);
- the cone-tool on-axis points;
- the axis comparison between limits and the blank (§3.3);
- the frame admission (signed basis vectors, origin and anchor at zero);
- `M·A·A > 0`.

Recorded geometry decides each one. None is a tolerance.

### 5.2 Root cone gap

The true root surface `K*` is the revolution of the exact root span about `A`.
It is the cone with apex point `P*`, axis `A` and slope `k*`. The reported cone
`K_r` has apex `O_r = float(P*)`, axis `A` and slope `s = float(k*)`.

1. Translating `K*` by `O_r − P*` moves every point by `δ_a = |O_r − P*|`,
   bounded by `RatSqrtUp` of the exact squared norm.
2. The translate and `K_r` share apex and axis. At axial distance `z` from that
   apex their radial gap is `|k* − s|·|z|`.
3. The translation also shifts each point's axial distance from the apex by at
   most `δ_a`. The fan's axial range, measured from `P*` as
   `[z_lo, z_hi]` with `z = (s(p) − Z_a)/|A|` divided per §3.4, therefore
   covers `|z| ≤ max(|z_lo|, |z_hi|) + δ_a` around the translated apex.

`ConeGap = δ_a + |k* − s|·(max(|z_lo|, |z_hi|) + δ_a)`.

The bound replaces both `RevolveMeridianResidual` over the span and
`revolveMeridianExtensionGap` over the fan reach. The span lies on `K*`
exactly, so no meridian residual remains.

For +X the canonical branch keeps the current tag-based residual.

### 5.3 Sketch margins and implicit costs

`ImplicitMargin` is an exact lower bound on `|F|` with `F` homogeneous of
degree 2 in `A`. `RevolveImplicitDisplacement` bounds the change of
`ρ² − s²z²` in mm². `CertifyRevolve*Boundary` and `CircleFragmentSide` must
therefore compare the margin with `cost·(A·A)`, and the factor is exact.

`revolveSeatPoint`'s brackets read `|(p−O_r)×A|² − s²((p−O_r)·A)²` (Sketch's
own `F`). They also read the source form `|p×A|² − (m·(p·A) + B)²`, where
`R = m·Z + B` is the meridian line in A-scaled coordinates. `B` is exact
because it comes from the scaled span ends. `fCost` scales by `A·A`.

For a canonical axis the branch is the current code.

### 5.4 Point-cone trims

The crossing `alpha = c/(s + k·σ)` equals the +X `a/(x + k·rho)` of the same
geometry expressed in axis coordinates. `FactorDyadic` keeps its single
`DySqrt` enclosure, now on `σ²`. Its point and `Q` errors are errors of
`p·alpha` and do not depend on the axis. `FactorRat` takes the same
substitution.

Each ordering and far-plane test compares A-scaled values. The mesh-bound
widening is a millimetre quantity, so it multiplies by `|A|↑`:

- `AboveFarCap`: `s(p) − meshBound·|A|↑ > c_far`.
- `ToeHeelOrdered`: the axial interval widens by `meshBound·|A|↑`.
- `Ordered`: the margin `dA·s + dR·σ` scales by `|A|²` and the move term by
  `|A|²` with one `|A|` from the mesh bound, so the move takes
  `(|dA|+|dR|)·meshBound·|A|↑`. Its radial range uses §3.5 for the minimum and
  `RatSqrtUp(σ²)` for the maximum.

`TriangleBound` follows §3.4, and `cellGeometryWith` follows §3.5.
`a↑ ≥ a`, `G↓ ≤ G` and `rhoMin↓ ≤ rhoMin`, and each `TriangleBound` term is
nondecreasing in `a` and nonincreasing in `G` and `rhoMin`. The general bound
is therefore no smaller than the exact one.

### 5.5 Copy drift

`M` is the exact rational linear part of the copy's motion, and the denoted copy
is `M(source)` (`facetedrecord.PlacedWith`).

**Bounds.**

- `e = MapChargeOf(M).Stretch`. Every eigenvalue of `MᵀM` lies in
  `[1−e, 1+e]`, so the polar factor `M_o` satisfies `‖M − M_o‖₂ ≤ e` for
  `e ≤ 1/2`.
- `s_φ ≥ sin∠(MA, A)`. It is
  `RatSqrtUp(|MA×A|²) / (RatSqrtDown(|MA|²)·RatSqrtDown(A·A))`, and
  `MA·A > 0` makes the angle acute. `s_φ ≤ DriftSinLimit` (§4.5).

**The ideal rotation.** The angle `φ_o = ∠(M_oÂ, Â)` satisfies
`φ_o ≤ asin(e) + asin(s_φ) ≤ (π/2)(e + s_φ)`. Let `R_min` be the rotation by
`φ_o` that carries `M_oÂ` to `Â`. Then `Q = R_min·M_o` is an exact rotation
with `QÂ = Â`, and `‖M − Q‖₂ ≤ e + ‖I − R_min‖₂ ≤ e + φ_o`. The charge is

`ε = e + (π/2)(e + s_φ)`.

`Q` maps the blank's root cone and the trim cones onto themselves exactly, as
the +X class's exact rotations do. The certified source contact therefore
transfers to `Q(source)`. `M(source)` differs from it by at most `ε·|p|`
at every point.

**Where the charge enters.** `PrepareSectorJoinPlans` charges placement
displacement in one term:
`PlacedRound = RigidRoundAllow + e·Radius3D(maxAbs)`. The general class puts
`ε` in place of `e` there and nowhere else. `PlacedRound` then reaches every
bound that the +X class gives it (`AccumulateSectorJoinBounds`):

- `plan.CrossingBound = PlacedBound(SourceCrossingBound) + PlacedRound`;
- `seamAllowance`, which adds `CrossingBound` and `PlacedRound`;
- `baseTerm = (PlacedBound(BaseMeshBound) + CrossingBound + PlacedRound)·lip`;
- `localRound`, which takes `(CrossingBound + PlacedRound)·lip`.

`lip = 1 + upperLip + lowerLip` is the Lipschitz factor that `CapBound` returns
for the toe and heel crossing maps. It carries the `1/sin` incidence of a fan
ray on a trim cone, which is how far a cap node moves per unit of displacement
of its far-plane station. The substitution changes the value fed into each
product and keeps every product, so the trim-cone transversality factor applies
to `ε` exactly as it applies to `e`.

**The root crossing.** The tooth crosses the blank's root cone transversally at
an angle `β`. Displacing the copy by at most `d = ε·|p|` moves the crossing
curve along the copy's wall by at most `d/sin β`. Over that stretch the wall
leaves the cone at slope `sin β`, so every wall point between the two crossing
curves lies within `d` of the cone surface. The cone surface it lies near is
either exposed in the other union or covered by that union's copy wall within
`d`. The union boundary therefore moves by at most `d`, and the `1/sin β`
cancels. The +X class relies on this same argument for `e`, so no new factor is
needed at the root.

**Separation.** `RevolveSectorSourcesSeparate` needs no drift term. It proves
that the denoted copies `M_i(source)` are disjoint, and those are the copies the
union holds.

**+X.** `XAxisRotation`'s block form gives `MX ∥ X` and `M_oX = X`. Hence
`s_φ = 0`, the `asin(e)` leg is zero and `ε = e`, the current charge.

### 5.6 Angular frame tilt

The held basis `B = (W, E0, E1)` read exactly has two bounds:

- `e_B = MapChargeOf(B).Stretch`;
- `s_W ≥ sin∠(W, A)`, computed as in §5.5.

The §5.5 argument gives an exact orthonormal frame `B*` with first axis `Â`
and `‖B − B*‖₂ ≤ δ_B = e_B + (π/2)(e_B + s_W)`.

`r_max` is `DvLenUpper` of the largest root-boundary node over every plan,
placed copies included. These are the nodes whose angles enter the angular-gap
proof.

For such a node `p`, `|p| ≤ r_max` and its true axial radius is at least
`ρ_min`. The projected point moves by at most `δ_B·r_max`. The held angle and
the angle about `A` therefore differ by at most
`asin(δ_B r_max/ρ_min) ≤ (π/2)δ_B r_max/ρ_min`. The class requires
`δ_B r_max < ρ_min`.

`PrepareSectorJoinPlans` converts spatial costs to angle by
`2·seamCost/minRadius`. Adding `τ = (π/2)·δ_B·r_max` to each pair's `seamCost`
covers both nodes' angle errors in the angular-gap proof. `minRadius` is
`min(R0,R1)/|A|↑`.

For +X, `B` is the exact identity permutation, so `τ = 0`. The canonical
branch does not fold it.

### 5.7 Ring vertices, root nodes and caps

General ring vertices are `RevolveLift.Point(basis, I, z, ρ, φ)`. Their gap to
the denoted swept point, which is the recorded plane point rotated about the
record's axis within its own `AxisBound`, is bounded exactly by the
`RevolveLift.SweptPointGap` construction. A per-station bound may compose
`BasisRound` once with the per-vertex product rounding, provided a unit test
shows it encloses `SweptPointGap`.

`sectionSag = maxRadius·maxStep²/8 + maxRound` keeps its form, with
`maxRadius = max R/|A|↓` and `maxRound` the largest such gap. The axis vertex is
`RevolveLift.Point(basis, I, z, 0, 0)`, and its gap is charged the same way.

`revolveFarRootPoint` builds its generator `W + s·(E0 cos θ + E1 sin θ)` from
the same basis. Each root node is then snapped to its ring vertex, and the
measured `Snap` is charged in `localBound` as it is today. A root node's
displacement is therefore covered by the ring vertex gap plus `Snap`, and needs
no separate charge.

### 5.8 Separation direction and audit patches

`sectorPairProjectionSeparate` stays a proof for any direction, because the
direction only selects a test. The general branch uses the exact box-centre
difference minus its `A` component. The canonical branch keeps YZ.

The cap patches project onto the coordinate plane that drops the index of the
largest `|A_k|`. For +X that is `{1,2}`. A failed patch proof keeps every pair in
the exact contact audit (`loftmesh.ProjectedPatch`), so the choice affects work
and never the result.

### 5.9 Adversarial review

Each question below was checked against the sections above.

1. **Can a general record reach +X-only math?** Only through `PointConeSource`,
   `RecordedSection`, a join record or a placement. §4.4 compares axes at
   `PointConeBoolean` and `CertifySector` from the PR that first makes a general
   limit, and §4.6 gates every consumer in §2.4. Each such PR carries a refusal
   test for each gate.
2. **Can `ImplicitMargin` units be mixed?** Margin is in `|A|²·mm²`. §5.3
   multiplies the cost by `A·A`. PR 3 uses `A·A = 5`, so a missing factor admits
   a fixture that must refuse.
3. **Can the reported cone differ from the one Sketch saw?** The certificate
   stores the `ConeSection` it passed. Every `cert.Cone() != cone` check
   compares that same value, `Axis` included.
4. **Can the apex miss the axis?** `O_r` may be off the axis line by a rounding.
   `δ_a` measures the full point distance, and §5.2 widens the reach by `δ_a`.
5. **Can an orientation sign flip?** `A` takes the resolved side sign (§3.6),
   and `R ≥ 0` is checked on every point. A profile on the other side refuses
   rather than revolving the wrong way.
6. **Can a parallel axis of another length, or a mixed pair, slip in?** §3.3
   requires bitwise equality, and a canonical axis never equals a general one.
7. **Can a reflected, translated, obtuse or tilted copy pass?** `IsReflection`,
   zero translation, `M·A·A > 0` and `DriftSinLimit` decline each one.
8. **Can drift go uncharged?** `ε` replaces `e` at the single site that charges
   placement displacement (§5.5). PR 4's test uses a copy whose axis is off by
   a few units in the last place. It shows that `ε·|p|` covers the exact
   distance of its placed root contacts from the root cone, and it goes red
   without the `s_φ` leg.
9. **Can the atan2 branch cut break a proof?** A source tooth whose root
   contacts straddle `−E0` has `|a−b| ≥ π` and refuses in
   `RevolveRootAngles`, as one straddling `−Y` does today. That is a refusal,
   not a wrong answer. §6 states the consumer requirement.
10. **Can the snap that hides an axis point go uncharged?** §4.1 and §4.3
    require exact on-axis points. A snapped point refuses, so no snap enters
    the class.
11. **Can the +X behaviour change?** The canonical branches are selected only
    by the predicates in §3.3, which every +X fixture passes, and run the moved
    bodies. §8 proves it.
12. **What do the construction checks guard?** The angle convention, the seam
    choice and the patch axes affect construction only. Orientation, vertex
    links, the exact triangle contact audit and the measured-volume gain still
    run on every result.
13. **Is any PR unsound on its own?** §7 states each PR's declines. PR 2 makes
    general limits and compares axes in the same PR. PR 3 makes general
    certificates and declines them at every Union entry. PR 4 changes placement
    charges while the Union still declines. PR 5 removes the declines only with
    the ring, angles and separation in place.

## 6. Consumer contract

A model in the general class does the following:

- Sketches the blank, toe cone and heel cone sections on a world coordinate
  plane through the origin. The gallery's shared XY sketch qualifies.
- Revolves all three about the same axis input with the anchor at the origin:
  a `SketchLine` from `(0,0)`, or a `ConstructionAxis` whose `Origin` is the
  world origin. `AxisVec` is then bitwise equal for all three.
- Places every on-axis meridian point exactly on the axis line.
  - With a float direction `A`, the point `t·A` is exact when both products
    round exactly.
  - One way: round the direction to 26 significant bits and the on-axis
    stations to 27.
  - Off-axis points need no rounding.
- Lofts the tooth from the world origin and trims it with the toe `Cut` and the
  heel `Intersect`.
- Places the source tooth so its two root contacts do not straddle `−E0`, the
  right normal of the axis in the sketch plane. Copies may lie anywhere.
- Copies it with `WithCopyPlacement(r3.Rotation(axis, θ))` about the
  float-normalized `A` and never uses `WithCopyRotation`. A copy whose axis
  drifts past `DriftSinLimit` declines.
- Does not `Transform` the blank or the joined body before the Union.

## 7. PR plan

Each PR keeps the general class declined wherever a later PR owns the work.
Each PR is green and sound on its own. Every new test is shown red first, either
by deleting the bound leg it guards or by running it against the base commit.

### 7.1 PR 1: Axis record, axis algebra and the bit-identity harness

The design document ships with this PR. No route consumes the new code yet.

Changes:

- `revolveplan.Payload.AxisVec` is set in `revolve.go` and in the chain Revolve
  (§3.6).
- A new leaf package `internal/coneaxis` holds `Axis` and `OfRevolve`, the two
  predicates (§3.3), and these methods:
  - `Axial`, `RadialSq` (Dyadic), `NormSq`, `NormBracket`;
  - `RadialLower` (§3.5);
  - `PlaneScaled(frame, u, v)`, which reads the axis in the frame's `U` and `V`;
  - `RecordedVec` (§3.6).
- `massmoment.AxisRotation(m, axis)` with `DriftSinLimit` (§4.5, §5.5).
  `XAxisRotation` becomes its canonical branch.
- The digest harness (§8.2).
- Layout rows for the package and the doc.

Tests:

- `coneaxis` against brute-force exact distances, using random dyadic points and
  `A ∈ {(1,2,0), (0,1,0), (3,4,0), (1,0,2)}`.
- `AxisRotation`:
  - a motion whose axis is off by a few units in the last place; `ε·|p|` covers
    the exact displacement, and the test goes red without the `s_φ` leg;
  - a tilt of `2⁻⁴⁰` declines.
- `AxisVec` for `SketchLine`, `ConstructionAxis`, a side flip, a non-zero
  `Start`, a non-origin `Origin` and a non-coordinate frame.
- The predicates: an XZ-plane record with `AxisVec = X` fails the frame
  predicate and passes the axis predicate.
- The harness digests are equal on base and head.

### 7.2 PR 2: Point-cone trims about `A`

Changes:

- `PointConeSource` admits a general record and sets `PointConeLimit.Axis`
  (§4.3).
- `ConstantsOf` yields the sample and bound sets, and `PointConeLimit`
  caches both (§3.4).
- `pointcone` takes an `Axis` in `FactorDyadic`, `FactorRat`, `AboveFarCap`,
  `Ordered`, `ToeHeelOrdered`, `CapGeometry`, `capGeometry`, `CellGeometry`,
  `cellGeometryWith` and `TriangleBound` (§3.4, §3.5, §5.4). `ReachUpper` stays
  in millimetres, and the PR shows that with a test.
- `facetedrecord` helpers pass `Axis` through.
- The axis comparison in `PointConeBoolean` and in `CertifySector` (§4.4).
- The limit declines in `loft_faces.go`, `loftfacestrim` and `CapOutside*`
  (§4.6).
- Update `agent-docs/loft-point-design.md` "Coaxial finite cone trims" and the
  missing-features row.

Soundness on its own: a general limit reaches only `PointConeBoolean`, its own
trim build, `CertifySector` and the declining consumers. Blanks are all
canonical in this PR, so `CertifySector` declines every general limit.

Tests:

- apitest trim about `A = (0,1,0)`. The fixture is the +X fixture with an exact
  coordinate permutation, so the volume intervals must overlap and the face
  classes must match.
- apitest trim about `A = (1,2,0)`. It runs `VerifyAll`, keeps the `PointCone`
  record, and checks the cone `NormalAt` against the analytic normal.
- `pointcone` Dyadic-versus-Rat differentials for a general `A`.
- An `AboveFarCap` fixture whose margin lies between `meshBound` and
  `meshBound·|A|`. It goes red without `|A|↑`.
- A `TriangleBound` fixture that goes red when the bound constants use `c`
  instead of `a↑`.
- Refusals:
  - a +X blank with a Y-axis tooth trim does not take the sector route;
  - a toe about X and a heel about Y decline in `PointConeBoolean`;
  - an on-axis point one unit in the last place off the axis;
  - a non-coordinate frame;
  - a `LoftFaces` cut with a general cone.

### 7.3 PR 3: Section and sector certificate about `A`

Changes:

- `RecordedSection`, `SpanFace` and `RootCandidates` (§4.1).
- `ConeSection` stored with `Axis = A` (§4.2), and `ConeGap` (§5.2).
- Connector, fit and circle boundaries, `revolveSeatPoint`,
  `revolveFitMeridianDifference`, `RevolveImplicitDisplacement` and
  `RevolveSectionToolGap` (§5.3).
- `RevolveExteriorPath` angles in the blank basis (§3.7).
- The Union declines of non-canonical certificates, and the `ConeFromFace`
  section decline (§4.6).

Soundness on its own: a general certificate reaches no join code, because every
Union entry declines it.

Tests:

- `CertifySector` on a hub and a lug about `(1,2,0)` certifies exactly one
  root face.
- `ConeGap` is at least the exact `K*`-to-`K_r` distance, computed
  independently in the test, and goes red without the `|k*−s|·δ_a` term on a
  fixture whose apex rounding is large.
- An `A·A = 5` margin fixture refuses, and goes red without the factor.
- Refusals: a snapped axis point, a tool axis `2·A`, a profile on the negative
  side, and `ConeFromFace` on a general blank.
- The Union of the general set still takes its current route.

### 7.4 PR 4: Placement and drift about `A`

Changes:

- `AxisRotation` at every placement site: `RevolveSectorSourcesSeparate`,
  `buildSectorsJoinAtSag` and `NextSectorUnion`. Each takes the certificate's
  axis.
- `ε` in `PlacedRound` (§5.5).

Soundness on its own: the Union still declines non-canonical certificates, so
the new code runs only on canonical axes there, where it is the current code.
General inputs reach it only through the internal tests below.

Tests:

- `RevolveSectorSourcesSeparate` with copies about `(1,2,0)`, separating and
  overlapping.
- `PrepareSectorJoinPlans` on a general plan: `PlacedRound` covers the exact
  distance of each placed root contact from the root cone, and goes red with
  `e` in place of `ε`.
- `NextSectorUnion` declines a copy past `DriftSinLimit`, a reflected copy, a
  translated copy and an obtuse copy.
- The canonical copy tests and the digest are unchanged.

### 7.5 PR 5: Ring, angles, separation and Union acceptance

Changes:

- `τ` in `seamCost` (§5.6).
- `SectorSeam` and `AngleAt`, `RevolveSectorCapBoundary` and
  `revolveFarRootPoint` in the blank basis.
- `BuildRevolvedSectionRing` through `RevolveLift` (§5.7).
- `RevolveSectionAngles` radii.
- The separation direction and the patch axes (§5.8).
- The join record carries the certificate axis.
- `SectorCylinderCut` declines a non-canonical join.
- The Union declines from PR 3 are removed.
- Update `agent-docs/revolve-loft-sector-design.md`, `agent-docs/api-design.md`
  §8 and missing-features.

Tests:

- apitest Union about `(0,1,0)`. It is congruent to the +X hub and lug, so the
  volume intervals overlap, Euler is 2 and `VerifyAll` passes.
- Hub and three lugs about `(1,2,0)`, serial and variadic. The gain is three
  times the single gain within the proven slack.
- A consumer-shaped bevel case: blank, back-cone tooth, toe and heel cones and
  three teeth on XY about a 26-bit direction. It checks `VerifyAll`, Euler 2
  and the gain.
- A basis-tilt fixture. It goes red without `τ`.
- A ring-vertex test whose composed bound encloses `SweptPointGap`.
- Refusals: a source tooth straddling `−E0`, and the bore `Cut` declining.

## 8. Bit-identical +X verification

### 8.1 Mechanism

Every function in §2.2 and §2.3 branches on one of the two predicates in §3.3:
record-reading code on the frame predicate and arithmetic on the axis
predicate. Every +X fixture passes both. The canonical branch is the current
body, moved without edits.

The new charges `ε − e`, `τ`, `δ_a` and the `|A|` conversions are skipped
rather than folded as zero. `proofbound.AbsSumUpper` rounds every term upward,
so folding a literal zero would still move a bound by one unit in the last
place. The canonical tests never read `AxisVec`. A +X record from a producer
that leaves `AxisVec` zero, such as a modify rewrite of the meridian, keeps its
current route.

### 8.2 Digest harness

No current test pins the route's bits. The route's tests check `VerifyAll`,
volume relations, Euler numbers and refusal text. Float bounds also differ
between amd64 and arm64, so a committed float literal would fail on the macOS
main job.

PR 1 adds two kinds of digest test:

- `apitest/sector_route_digest_test.go` holds the `TestSectorRouteDigest*` tests,
  one per group of fixtures so the CI shards place them independently. They
  build each fixture in §8.3 and hash, with SHA-256, the `Float64bits` of these public
  readings:
  - every tessellated vertex at the fixture's `VerifyAll` tolerance;
  - every `Body.Vertices()` position and bound;
  - `Volume`, `Area` and `Centroid`, each value and bound;
  - the `Bounds` box.

  These readings carry the payload's `MeshBound`, `VolSymDiff` and
  `AreaSlack`, so the test reads no unexported state.
- `internal/pointcone/digest_test.go` `TestPointConeDigest` hashes the samples
  of `FactorDyadic` and `FactorRat`, `CapBound`, `ReachUpper`, `Ordered`,
  `ToeHeelOrdered` and `AboveFarCap` on a fixed point set, one digest per cone.
  Its cones include a non-positive apex, which sends `PointConeFactor` to
  `FactorRat`. The test calls `FactorRat` directly for every cone and
  `FactorDyadic` for the positive ones. `PointConeSource` refuses a
  non-positive apex, so no public fixture reaches that branch. A coverage run
  of the `TestSectorRouteDigest*` tests over `internal/pointcone` shows `FactorRat` is
  never executed, so this test is the only digest that covers it.

Each apitest group builds its first fixture twice and asserts that the two
builds hash equally, and `TestPointConeDigest` hashes twice. When
`DECAD_SECTOR_DIGEST` names a directory, each apitest group writes its
per-fixture digests to `<group>.txt` there. When `DECAD_POINTCONE_DIGEST` names
a file, `TestPointConeDigest` writes its per-cone digests there.

Every PR runs both on its base commit and its head commit on the same machine.
The base commit lacks the test files, so the procedure copies the two files
from the head onto a scratch checkout of the base before running them. The two
digest outputs must be byte-equal, and the PR body records the digests.

### 8.3 Fixtures in the digest

The apitest digest builds these existing tests' fixtures:

- `apitest/union_revolve_loft_sector_test.go`:
  - `TestUnionTaperedHubAndRadialLug`;
  - `TestUnionTaperedHubAndFittedRadialLug`;
  - `TestUnionShoulderedHubAndRadialLug`;
  - `TestUnionSlopedHubCapAndRadialLug`;
  - `TestUnionShoulderedHubAndTwoRadialLugs` (serial `NextSectorUnion`);
  - `TestUnionSixSpanHubSerialRadialLugs`;
  - `TestUnionShoulderedHubAndTwoRadialLugsVariadic`;
  - `TestUnionShoulderedHubAndThreeRadialLugsVariadic`;
  - `TestUnionShoulderedHubWithNearAxisCopy` (an `EX.X` one step off 1).
- `apitest/union_revolve_loft_sector_many_test.go`:
  - `TestUnionHubAndManyRadialLugs` (40 copies).
- `apitest/union_embedded_root_teeth_test.go`:
  - `TestUnionBevelBlankAndEmbeddedRootTeeth` (circle fragment and partial
    fit route);
  - `TestUnionBevelTipTouchingHeelConeNeedsMatchingFaces`.
- `apitest/revolve_sector_cylinder_cut_test.go`:
  - the annular, repeated annular, straight band and repeated straight band
    `Cut` tests.
- `apitest/point_cone_trim_test.go`:
  - the six `TestPointConeTrimKeepsFittedSidesAndCone` cases;
  - `TestPointConeTrimConeNormalAndCopyPlacement`.

These existing tests still run unchanged on every PR:

- the `VerifyAll` and relation tests above;
- their refusal texts ("complete sector sources are not separate",
  "far source plane");
- the `pointcone` and `facetedrecord` differential `Float64bits` tests (fast
  path against the Rat form, parallel against serial);
- `revolve_sector_placement_test.go`;
- `internal/massmoment/axis_placement_test.go`;
- `internal/revolveshell/sector_source_test.go`, which includes
  `TestRootCandidates`'s exact `Axis == {X:1}`;
- `internal/facetproof/sector_seam_test.go`.
