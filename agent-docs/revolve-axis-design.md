# General-axis copied-tooth Union

This design extends the finite Revolve and point-Loft sector class
(`agent-docs/revolve-loft-sector-design.md`) and its point-cone trims
(`agent-docs/loft-point-design.md` "Coaxial finite cone trims") from the world
+X axis to any axis through the world origin that lies in a world coordinate
plane. The +X class is the special case in which every new charge is exactly
zero, and its arithmetic is unchanged. Nothing here is specific to gears: the
class is any Revolve blank, one cone-trimmed point-Loft sector and rigid copies
of that sector.

Every step of the class is implemented about a general axis: the point-cone
trims (§4.3, §4.4), the section and sector certificate (§4.1, §4.2, §5.2,
§5.3), the copy placement, drift, frame tilt and source separation (§4.5,
§5.5, §5.6, §5.8), and the ring, the angles, the patch axes and the serial and
variadic Union (§3.7, §5.7, §5.8). The coaxial cylinder `Cut` of a joined body
stays +X only (§4.6, `agent-docs/missing-features.md`, Booleans).

## Question router

Navigation only; each section owns its content.

| Question | Section |
|---|---|
| Which code reads world +X today? | §2 |
| How is the axis represented, and which predicate selects +X? | §3 |
| What does the general class admit and refuse? | §4 |
| Why is each new charge sound? | §5 |
| What must a consumer model do? | §6 |
| Which tests cover each part? | §7 |
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
  It also declines a join whose certificate came from the general record
  branch about `A = X`.
- `ConeFromFace`.
- The `LoftFaces` cone trims (`loftfacestrim`, `loft_faces.go`).

## 2. Inventory of +X assumptions

Each row names a site, what it reads as +X, and the part of §7 that tests its
general reading. "Canonical" means the +X body stays verbatim behind the canonical branch
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

| Site | Function | +X reading | Tests |
|---|---|---|---|
| `internal/revolveshell/sector_source.go` | `RecordedSection` | Frame origin 0, `U = X`, `V = Y`; `Ax` `AU = AV = 0`, `DU = 1`, `DV = 0`, all bounds 0 | §7.3 |
| same | `RootCandidates` | `cone.Axis == {X:1}`, `Origin.Y = Origin.Z = 0`; apex read as `Origin.X` | §7.3 |
| same | `CertifySector` | compares the blank's and both limits' axes (§4.4); the blank reading is +X | §7.2 |
| `internal/revolveplan/point_cone_source.go` | `PointConeSource` | the frame predicate selects the +X reading; any other record reads about its `AxisVec` (§4.3) | §7.2 |
| `internal/revolveplan/point_cone_boolean.go` | `PointConeBoolean` | admits a limit only about the held limits' axis (§4.4) | §7.2 |
| `internal/revolveproof/sector_fit.go` | `revolveFitMeridianDifference` | `cone.Axis == {X:1}`, `Apex.Y = Apex.Z = 0` | §7.3 |
| `internal/revolveproof/circle_side.go` | `CircleFragmentSide` | `cone.Axis == {X:1}` | §7.3 |
| `internal/massmoment/axis_placement.go` | `XAxisRotation`, `XAxisPlacementCharge` | block form about X | §7.1, §7.4 |
| `internal/revolveshell/sector_union.go` | `NextSectorUnion` | `XAxisRotation(motion)` | §7.4 |
| `internal/revolveshell/sector_placement.go` | `RevolveSectorSourcesSeparate` | `XAxisPlacementCharge` | §7.4 |
| `internal/revolveshell/sector_join_build.go` | `buildSectorsJoinAtSag` | `XAxisPlacementCharge` | §7.4 |
| same | same | `ProjectedPatch.Axes = {1,2}` | §7.5 |
| `internal/revolveshell/sector_bore_cut.go` | `SectorCylinderCut` | prism frame `N = X`, `U = Y`, `V = Z` | declines (§7.5) |
| `internal/revolveshell/sector_bore_record.go` | `RevolveSectorBoreCapRecord` | axis vertices `== {X: U}` | declines (§7.5) |

### 2.3 World X as the axial coordinate, YZ as the radial plane

| Site | Function | +X reading | Tests |
|---|---|---|---|
| `internal/pointcone/factor.go` | `FactorDyadic`, `FactorRat` | `g = x + k·hypot(y,z)`, `q = g/a` | §7.2 |
| `internal/pointcone/cone.go` | `ConstantsOf` | one constant set from the X apex `a` and `k`, used for samples and bounds alike | §7.2 |
| same | `AboveFarCap` | far plane `x = far`, `p.X − meshBound` | §7.2 |
| same | `Ordered` | X range, `rho` from `y²+z²`, YZ-box radial minimum | §7.2 |
| same | `ToeHeelOrdered` | X range | §7.2 |
| `internal/pointcone/cap.go` | `CapGeometry`, `capGeometry`, `CellGeometry`, `cellGeometryWith` | `xMin` from `p.X`, `rhoMin` from the YZ box | §7.2 |
| same | `ReachUpper` | the norm of `p` in millimetres; axis-free, kept in millimetres (§3.4) | §7.2 |
| same | `TriangleBound` | millimetre `a`, `G`, `h` (homogeneity, §3.4) | §7.2 |
| `internal/facetedrecord/point.go` | `PointConeLimit`, `Constants`, `BoundConstants` | `Apex`, `Far` are A-scaled about `Axis`, X coordinates for the canonical axis; one constant set for the canonical axis | §7.2 |
| `internal/facetedrecord/point_cone_helpers.go` | `PointConeFactor(s)`, `PointConeLimitLater/Earlier/Ordered` | through `pointcone` | §7.2 |
| `internal/revolveproof/sector_geometry.go` | `RevolveMeridianResidual` | meridian `(U,V)` against `Origin.X` | §7.3 |
| same | `RevolveImplicitDisplacement`, `revolveMeridianExtensionGap` | `world.X > 0`, X range of the fan | §7.3 |
| `internal/revolveproof/sector_seat.go` | `revolveSeatPoint` | form `y²+z² − s²(x − Origin.X)²`, `sourceForm`, X range | §7.3 |
| `internal/revolveproof/sector_fit.go` | `revolveFitMeridianDifference` | box `x = point[0]`, tangent `[0]` | §7.3 |
| `internal/revolveproof/circle_side.go` | `CircleFragmentSide` | `h = q[0]`, `q1²+q2² − s²h²` | §7.3 |
| `internal/revolveproof/sector_source.go` | `CertifyRevolve{Connector,Fit}Boundary` | `ImplicitMargin` against a cost in mm² (unit axis) | §7.3 |
| `internal/facetedrecord/revolve.go` | `ConeSection`, `MeridianEquation` | tag axis; `(U,V)` read as world `(X, r)` | §7.3 |
| `internal/revolveshell/section_tool_gap.go` | `RevolveSectionToolGap` | `U + k·V − Apex` with X-scalar cone limits | §7.3 |
| `internal/revolveproof/sector_path.go` | `RevolveExteriorPath` | `Angle = atan2(Z, Y)` | §7.3 |
| `internal/revolveshell/sector_join_plan.go` | `PrepareSectorJoinPlans` | `PlacedRound` charges `Stretch` times the node norm | §7.4 |
| same | same | `MinRadius` from `V` | §7.4 |
| same | same | `facetproof.AngleAt` | §7.5 |
| `internal/revolveshell/sector_cap.go` | `revolveFarRootPoint` | generator `{1, s·cos, s·sin}` | §7.5 |
| same | `RevolveSectorCapBoundary` | `atan2(Z, Y) + 2π` | §7.5 |
| `internal/revolveshell/section_ring.go` | `BuildRevolvedSectionRing` | vertex `{U, V·cos, V·sin}`; axis vertex `{U,0,0}`; X exact | §7.5 |
| `internal/revolveshell/section_angles.go` | `RevolveSectionAngles` | `maxRadius` from `V`; angles in the atan2(Z,Y) convention | §7.5 |
| `internal/revolveshell/sector_cap_refine.go`, `sector_mesh.go` | `SectorConeCapBounds`, `SectorRefineCap`, `BuildRevolveSectorMesh` | through `pointcone` | §7.5 |
| `internal/revolveshell/sector_mesh.go` | `BuildRevolveSectorMesh` | the root span's high end by `U`; a general certificate compares the exact `Z` | §7.5 |
| `internal/facetproof/sector_seam.go` | `AngleAt`, `SectorSeam` | `atan2(Z, Y)` | §7.5 |
| `internal/facetproof/sector.go` | `sectorPairProjectionSeparate(Float)` | YZ direction (sound for any axis, weak off X) | §7.4 |
| `internal/loftmesh/loft_projected_patch.go` | `ProjectedPatch` | coordinate planes only (sound, self-checking) | §7.5 |

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

The section certificate is the exception. Its +X arithmetic reads the
plane-local root ends as `(x, r)`, which holds only for a record that passes
the frame predicate. `RootCandidates` therefore records which branch built
the certificate: a certificate from the general record branch carries
`RevolveSectionCertificate.General`, the exact A-scaled readings of §4.1 and
§4.2, and every section consumer (`RevolveImplicitDisplacement`, the seat and
fit brackets, `RevolveSectionToolGap`, `RevolveExteriorPath`) selects its
branch on that field, not on `Canonical()`. Such a certificate takes the
general arithmetic even when `A = X`. `CircleFragmentSide` reads only world
coordinates and the cone, so it branches on the cone's axis.

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

`facetproof.AngleBasis` holds the convention. Its zero value is the +X
`atan2(Z, Y)`, and `RevolveSectionCertificate.AngleBasis()` returns the
general basis `(E0, E1)` exactly when `General` is set. Every angle of the
join reads it: the exterior path's root contacts, the cap boundary's root
stations, the seam (`AngleBasis.SectorSeam`) and each placed plan's root nodes
(`SectorJoinInput.Angles`, set by `GeneralPlanInput`).

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
  (`rp.Ax.Walk` of the plane walk) and classifies it with the build's own
  `revolveaxis.Classify`. The build used that same walk, so face matching
  stays an exact equality. A span the build split differently from the exact
  corners finds no face, and the section refuses.

`RootCandidates` reads the cone from exact data, not from the rounded
`surfacegeom.Cone` tag:

- slope `k* = (R1−R0)/(Z1−Z0)` and apex scalar `Z_a = Z0 − R0/k*`;
- apex point `P* = (Z_a / A·A)·A`.

It requires all of the following:

- a finite slope with `k* > 0`, which puts `Z_a` strictly below both span
  ends because `R0 > 0`;
- the tag is a `Cone`, which binds the face.

### 4.2 Reported cone

`ConeSection()` returns `{Apex: float(P*), Axis: A, Slope: float(k*)}` and is
stored on the certificate (`RevolveSectionAxis.Cone`). Each coordinate of
`P*` and the slope round to the nearest float. Sketch certifies against
exactly this cone. §5.2 charges `ConeGap`, the distance between it and the
true root cone: the certificate's `ConeGap` covers the root span's own axial
reach, and `RevolveImplicitDisplacement` recomputes it over the fan's reach.

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
`ok = false`, and the Union route declines (matched false), as it does for a
copy that `XAxisRotation` rejects. The drift that passes is charged (§5.5),
never assumed away.

`massmoment.AxisPlacementCharge(m, axis)` reads a recorded placement: zero
charge and `ε = 0` for the identity, `AxisRotation`'s charge and `ε` for an
admitted rotation, and an unsupported error otherwise. Every placement site
reads the certificate's axis `Section.Axis`:

- `NextSectorUnion` admits the new copy only through `AxisRotation`.
- `RevolveSectorSourcesSeparate` charges each copy's mesh bound with
  `AxisPlacementCharge`'s map charge.
- `buildSectorsJoinAtSag` passes each copy's charge and `ε` to
  `PrepareSectorJoinPlans` (`SectorJoinInput.Charges`, `Drifts`).

### 4.6 Declines outside the class

- `ConeFromFace`, `loft_faces.go`, `loftfacestrim` and `CapOutside*` decline a
  `PointConeLimit` or section whose axis is not canonical. They keep +X math
  and never see a general record. `ConeFromFace` also checks that the cone it
  built reads back about X.
- `SectorCylinderCut` declines a joined record whose certificate came from the
  general record branch, or whose tooth trim holds a limit about a
  non-canonical axis (`sectorJoinGeneralAxis`). The pair then takes ordinary
  dispatch (`agent-docs/missing-features.md`, Booleans). The certificate check covers a blank outside the frame predicate about `A = X`,
  such as an XZ-plane blank revolved about U: its limits are canonical, but its
  ring stands at angles in the blank's basis and the bore route reads
  `atan2(Z, Y)`.
- `ConeFromFace` reads a section side as `(x, r)` about world X, so it
  declines a blank outside the frame predicate before it reads a span.

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

The point distance between the two cones needs only
`δ_a + |k* − s|·max(|z_lo|, |z_hi|)`: a point of `K_r` moves by at most `δ_a`
under the translation, and the radial gap read at its own axial distance
covers the rest. The term `|k* − s|·δ_a` covers the implicit reading of §5.3,
which evaluates the slope gap at the translated point, whose axial distance
from `P*` may be `δ_a` larger. A test against the exact distance therefore
cannot show that term red, and the tests in §7.3 show the other two terms red.

For +X the canonical branch keeps the current tag-based residual.

### 5.3 Sketch margins and implicit costs

`ImplicitMargin` is an exact lower bound on `|F|` with `F` homogeneous of
degree 2 in `A`. `RevolveImplicitDisplacement` bounds the change of
`ρ² − s²z²` in mm². `CertifyRevolve*Boundary`, for its Sketch side
certificates and for `CircleFragmentSide`'s margin, must therefore compare
the margin with `cost·(A·A)` (`implicitMarginExceeds`), and the factor is
exact.

The general `RevolveImplicitDisplacement` requires `p·A > 0` for every fan
vertex. It reads the fan's axial range from `P*` per §3.4, widened by the
mesh bound, and requires its lower end above `δ_a`, so the fan lies past both
apexes. Its displacement is `d = MeshBound + ConeGap` over the fan's reach.
The cost `(1+s²)·d·(2r+d)` bounds the change of `ρ² − s²h²` only when `r`
bounds both `ρ` and the axial `h`. The apex may lie behind the origin, so `r`
is the larger of the fan's distance from the origin and its axial reach from
`P*`, plus `d`.

`revolveSeatPoint`'s brackets read `|(p−O_r)×A|² − s²((p−O_r)·A)²` (Sketch's
own `F`). They also read the source form `|p×A|² − (m·(p·A) + B)²`, where
`R = m·Z + B` is the meridian line in A-scaled coordinates. `B = −k*·Z_a` is
exact because it comes from the scaled span ends. Both forms are quadratic
along the exact connector `q + t·d`, so their difference `D(t)` is too. The
general bracket takes `fCost` as the exact maximum of `|D|` over `t ∈ [0, 1]`,
read at the two ends and the vertex. It needs neither `ConeGap` nor an `A·A`
factor, because `D` and Sketch's `F` share A-scaled units.

The general fitted-spline bracket (`revolveFitMeridianDifference`) bounds
`F_blank − F_cone` over the box of a spline range. With `u = (p−O_r)·A` and
`c_O = O_r·A`, the difference is `2·p·W − |O_r×A|²` plus `gap·sum`, where
`W = (A·A)·O_r − c_O·A`, `gap = (s−k*)·u − c` and `sum = (s+k*)·u + c` with
`c = k*·c_O + B`. The first part is zero for an apex on the axis line, and
the second is the +X factored form.

For a canonical certificate the branch is the current code.

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
`PlacedRound = RigidRoundAllow + ε·Radius3D(maxAbs)`, with `ε` read from
`SectorJoinInput.Drifts`. For the canonical axis `ε` is `e` exactly, the
Stretch itself. The general class puts `ε` in place of `e` there and nowhere
else. `PlacedRound` then reaches every
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
`min(R0,R1)/|A|↑`. The seam test already requires `seamCost < minRadius/2`,
and `τ ≥ δ_B·r_max`, so a passing pair meets `δ_B r_max < ρ_min` with no
separate check.

The code holds these pieces as follows:

- `RevolveSectionAxis` stores the blank basis's `W` with `E0` and `E1`.
- `massmoment.FrameTilt(W, E0, E1, A)` returns `δ_B`. It shares the sine
  bound and the `e + (π/2)(e + s)` rounding with `AxisRotation` and has no
  limit of its own: a large tilt only makes `τ` large.
- `revolveshell.GeneralPlanInput` sets `minRadius` from the exact `RootR`
  with `ScaledDown`, `FrameTilt` and `Tilted` on `SectorJoinInput`.
- `PrepareSectorJoinPlans` computes `r_max` over the placed nodes and folds
  `τ` only when `Tilted` is set.

For +X, `B` is the exact identity permutation, so `τ = 0`. The canonical
branch does not fold it.

### 5.7 Ring vertices, root nodes and caps

General ring vertices are `RevolveLift.Point(basis, I, z, ρ, φ)`, with `z` and
`ρ` the plane point's axis coordinates (`revolveaxis.Frame.ToAxis`) and the
blank Revolve's own lift and basis. Each vertex charges its exact
`RevolveLift.SweptPointGap`: the gap to the denoted swept point, which is the
recorded plane point rotated about the record's axis within its own
`AxisBound`, at the angle whose exact sine and cosine the station's
enclosures hold. The ring evaluates that construction at every vertex rather
than composing a per-station bound.

`sectionSag = maxRadius·maxStep²/8 + maxRound` keeps its form, with
`maxRadius = max R/|A|↓` and `maxRound` the largest such gap. A point is on the
axis exactly when its plane-scaled `R` is zero. Its axis vertex is
`RevolveLift.Point(basis, I, z, 0, 0)`, and its gap is charged the same way at
angle zero. `RevolveSectionAngles` reads its radius as `R/|A|↓` too, and
`BuildRevolveSectorMesh` orders the root span's ends by the exact `Z`.

`revolveFarRootPointAbout` builds its generator `W + s·(E0 cos θ + E1 sin θ)`
from the same basis, starting at the reported apex `O_r`. Each root node is then snapped to its ring vertex, and the
measured `Snap` is charged in `localBound` as it is today. A root node's
displacement is therefore covered by the ring vertex gap plus `Snap`, and needs
no separate charge.

### 5.8 Separation direction and audit patches

`sectorPairProjectionSeparate` stays a proof for any direction, because the
direction only selects a test. The general branch,
`facetproof.SectorSourcesSeparateAbout`, uses the exact box-centre
difference `d` minus its `A` component, `n = (A·A)·d − (d·A)·A`, and projects
every held vertex exactly onto it, widened by `|n|↑` times the placed bound.
It has no float fast path. The canonical branch keeps YZ and its fast path.
`RevolveSectorSourcesSeparate` selects the branch by the axis predicate.

The cap patches project onto the coordinate plane that drops the index of the
largest `|A_k|`. For +X that is `{1,2}`. A failed patch proof keeps every pair in
the exact contact audit (`loftmesh.ProjectedPatch`), so the choice affects work
and never the result.

### 5.9 Adversarial review

Each question below was checked against the sections above.

1. **Can a general record reach +X-only math?** Only through `PointConeSource`,
   `RecordedSection`, a join record or a placement. §4.4 compares axes at
   `PointConeBoolean` and `CertifySector`, and §4.6 gates every consumer in
   §2.4. Each gate has a refusal test (§7).
2. **Can `ImplicitMargin` units be mixed?** Margin is in `|A|²·mm²`. §5.3
   multiplies the cost by `A·A`. The margin test (§7.3) uses `A·A = 5`, so a
   missing factor admits a fixture that must refuse.
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
   placement displacement (§5.5). The drift test (§7.4) uses a copy whose axis
   is off by just under `DriftSinLimit`. It shows that `PlacedRound` covers the exact
   distance of its placed root contacts from the root cone, and it goes red
   with `e` in place of `ε`.
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
13. **Can a +X-only consumer read a general join?** The bore route reads the
    join record in +X arithmetic. `SectorCylinderCut` declines every general
    certificate before it reads the record, including one about `A = X` from a
    blank outside the frame predicate (§4.6).

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
- Places each cone tool's two rim endpoints exactly on its far plane:
  `PointConeSource` requires their `Z = p·A` to equal the far station's `Z`
  exactly, and a rim a rounding off that plane declines the trim.
  - One way: write each tool point as `s·A + q·N`, where `N` is `A` turned a
    quarter turn in the sketch plane, with `s` and `q` on a dyadic grid coarse
    enough that both plane coordinates are exact. Its readings are then
    exactly `Z = s·|A|²` and `R = q·|A|²`. A 26-bit direction and a `2⁻²²`
    grid keep coordinates below 16 exact.
  - The blank's other off-axis points need no rounding.
- Lofts the tooth from the world origin and trims it with the toe `Cut` and the
  heel `Intersect`.
- Places the source tooth so its two root contacts do not straddle `−E0`, the
  right normal of the axis in the sketch plane. Copies may lie anywhere.
- Copies it with `WithCopyPlacement(r3.Rotation(axis, θ))` about the
  float-normalized `A` and never uses `WithCopyRotation`. A copy whose axis
  drifts past `DriftSinLimit` declines.
- Does not `Transform` the blank or the joined body before the Union.
- Does not expect the coaxial cylinder `Cut` of the joined body: it takes
  ordinary dispatch (§4.6).

## 7. Tests

Each test below goes red when the bound leg or the code path it guards is
removed.

### 7.1 Axis record, axis algebra and the bit-identity harness

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
- The digest harness (§8.2).

### 7.2 Point-cone trims about `A`

- Root-package trim about `A = (0,1,0)`, which reads the `PointCone` record.
  The fixture is the +X fixture with an exact coordinate permutation, so the
  volume intervals must overlap and the face classes must match.
- Root-package trim about `A = (1,2,0)`. It runs `VerifyAll`, keeps the
  `PointCone` record, and checks the cone `NormalAt` against the analytic
  normal.
- `pointcone` Dyadic-versus-Rat differentials for a general `A`, and
  `ReachUpper` read in millimetres.
- An `AboveFarCap` fixture whose margin lies between `meshBound` and
  `meshBound·|A|`. It goes red without `|A|↑`.
- A `TriangleBound` fixture that goes red when the bound constants use `c`
  instead of `a↑`.
- Refusals:
  - a +X blank with a Y-axis tooth trim does not take the sector route;
  - a toe about X and a heel about Y decline in `PointConeBoolean`, and so
    does a heel about a parallel axis of another length;
  - an on-axis point one unit in the last place off the axis;
  - a non-coordinate frame;
  - a `LoftFaces` cut with a general cone, a heel `Intersect` with one,
    `LoftFacesToConeTangent` with one, and `CapOutside*`;
  - `ConeFromFace` registering a cone that reads back about another axis;
  - the bore `Cut` of a join whose tooth trim holds a general limit.

### 7.3 Section and sector certificate about `A`

- `CertifySector` on a hub and a lug about `(1,2,0)`, with line and fitted
  flanks, certifies exactly one root face, and the root contacts' angles in
  the blank basis match the +X fixture's.
- The congruent hub and lug about `(0,1,0)` certify the same root span and
  boundary roles as +X, with the same tool gap.
- `ConeGap` is at least the `K*`-to-`K_r` distance, sampled independently in
  512-bit arithmetic. It goes red without `δ_a` on a fixture whose apex
  rounds and whose slope is exact, and without the `|k*−s|` term on a fixture
  whose apex is the origin and whose slope rounds. §5.2 states why the
  `|k*−s|·δ_a` term has no distance test.
- An `A·A = 5` margin fixture refuses, and goes red without the factor.
- The seat shift covers the exact contact on a connector, and the fitted
  bracket is exact on point boxes, about an axis whose rounded apex lies off
  the axis line. `CircleFragmentSide` about `(0,1,0)` matches +X.
- Refusals: a snapped axis point, a tool axis `2·A`, a profile behind the
  origin, and `ConeFromFace` on a general blank.

### 7.4 Placement, drift, tilt and separation about `A`

- `RevolveSectorSourcesSeparate` with copies about `(1,2,0)` and `(0,1,0)`:
  a quarter turn separates only along the general direction, a five-degree
  turn overlaps, and a tilted, translated, reflected or obtuse copy refuses.
- `PrepareSectorJoinPlans` on a general plan about both axes: `PlacedRound`
  covers the exact distance of each placed root contact from the root cone,
  and goes red with `e` in place of `ε`.
- `PrepareSectorJoinPlans` with a frame tilt whose `τ` exceeds the angular
  gap refuses, and goes red without `τ`.
- `FrameTilt` bounds the change of a held angle difference against the exact
  angle about `A`, and goes red without the `s_W` leg.
- `GeneralPlanInput` reads `minRadius` about the axis, and `FrameTilt` is
  zero for the permuted fixture.
- `NextSectorUnion` with a certificate about `(1,2,0)` or `(0,1,0)` admits a
  rounded copy about the axis and declines a copy past `DriftSinLimit`, a
  reflected copy, a translated copy and an obtuse copy.
- The canonical copy tests and the digest are unchanged.

### 7.5 Ring, angles and Union

- `apitest/union_revolve_axis_test.go` builds the shouldered hub, the radial
  lug, its toe and heel cone tools and two copies at 120° and 240° through the
  public API about one axis, joins them with `Union`, and checks `VerifyAll`,
  Euler characteristic 2 and a closed mesh whose every edge meets its
  opposite once. Its volume, and the hub's, must agree with the same
  construction about +X within the sum of the proven bounds:
  - about `(0,1,0)`, and about X from an XZ-plane blank (a general
    certificate with `A = X`), both exact quarter-turn images of +X, with no
    slack;
  - about `(1,2,0)`, serial and variadic, and about `(1,2)/√5` rounded to 26
    significant bits, with every tool point on a dyadic grid (§6) and a slack
    of the surface area times the largest input displacement.

  Each goes red when any Union entry declines a general certificate, or when
  the ring, the cap boundary's root angles, the seam, the plans' angles, the
  root generator, the root span order or the ring radius reads the +X
  convention.
- The root-package Union of a tilted hub and lug in either operand order,
  serially with a copy and variadically, keeps the general certificate on
  every join record.
- `BuildRevolvedSectionRing` about `(1,2,0)` and `(1,−2,0)`: every vertex lies
  within its charged `SweptPointGap` of the circle its plane point sweeps
  about the exact axis, read in 512-bit arithmetic, and the ring's bound covers
  every vertex and the chord sag at the largest true radius. About `(1,−2,0)`
  every plane-local `V` is negative, so a ring that reads `V` as the radius
  goes red.
- Refusals:
  - a source tooth about `(0,1,0)` straddling `−E0 = −Z` refuses in
    `TraceSectorLoop` and in the public `Union`, while the same tooth along
    `+Z` traces; with the root angles read as `atan2(Z, Y)` it would not
    straddle;
  - the coaxial cylinder `Cut` of a join about `(0,1,0)`, and of the XZ-plane
    join about X, takes ordinary dispatch, while the +X join is cut by the
    bore route; `SectorCylinderCut` declines a certificate with `General` set
    even when its limits are about X.

The patch axes (§5.8) change only the audit's work, so no test can show them
red.

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

Two kinds of digest test exist:

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

A pull request that touches a site in §2 runs both on its base commit and its
head commit on the same machine, each tree with its own copy of the two test
files. One that changes the signature of a hashed function edits its copy to
pass the canonical axis and keeps every hashed reading and the output format.
The two digest outputs must be byte-equal, and the pull request body records
the digests.

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

These existing tests still run on every change. A signature change only adds the
canonical axis to a call in them, as `revolve_sector_placement_test.go` passes
`coneaxis.X()` to `RevolveSectorSourcesSeparate`:

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
