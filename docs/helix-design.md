# Helix Design

`Document.Coil` builds one solid by moving a closed planar profile along a
screw motion about an axis that lies in the profile's own plane: the section
turns about the axis and slides along it at a stated pitch, staying in the
axis plane throughout. A thread cutter, a spring and a coil are this one
build. This document owns the entry point, the screw-motion frame, the
refusals, the construction, the result's topology, the four readings and
their closed forms, downstream coverage, the thread use case and the staged
delivery.

Companion contracts stay authoritative for their own areas:

- `docs/api-design.md` owns the public model and the sketch-profile seam;
- `docs/evaluator-design.md` §6 owns `Revolve`, whose axis vocabulary,
  in-plane axis resolution and side gate this build reuses;
- `docs/sweep-design.md` owns `Path` and `Sweep`, which this build does NOT
  extend (§12 states why);
- `docs/loft-design.md` §5–§6 own the held-triangle shell model and the
  crossing audit this build reuses;
- `docs/faceted-vertex-bounds-design.md` owns the per-vertex bound model the
  held shell publishes;
- `docs/tessellation-design.md` owns mesh proofs and boolean admission;
- `docs/verification-design.md` owns report meaning and the tolerance gate.

Five tables are normative:

| Table | States | Section |
|---|---|---|
| **CP** | the screw motion and the section frame | §3 |
| **CS** | what refuses a coil, and its sentinel | §4 |
| **CB** | the result's topology, faces and roles | §6 |
| **CM** | the four readings, their closed forms and bounds | §7 |
| **CD** | downstream coverage and staging | §8 |

Question router (navigation, not authority):

| Your question | Section |
|---|---|
| What does `Coil` take, and what do pitch, turns and hand mean | §2 |
| Why the section stays in the axis plane, not normal to the helix | §3 |
| Which inputs refuse, with which sentinel, in which order | §4 |
| How the held shell is built, and what every vertex's bound is | §5 |
| What faces, edges and vertices the body has | §6 |
| What `Volume`, `Area`, `Centroid` and `Bounds` read, and how tight | §7 |
| What `Verify`, `Tessellate`, booleans, `Placed` and STEP do | §8 |
| How to cut an external or internal thread, and what it measures | §9 |
| Which PR lands what, and what each must prove first | §11 |
| Why not a `Helix` path segment, or a Frenet frame | §12 |

## 1. Scope

The admitted operation has one closed planar profile, one axis in the
profile's plane, a positive pitch and a positive number of turns. The build
admits:

- a profile whose every segment, on the outer loop and on every hole, is a
  whole `LineSeg`, a whole `ArcSeg` or a whole `CircleSeg`, the circle a
  loop on its own;
- an axis stated as `SketchLine`, `ConstructionAxis` or `EdgeAxis`, resolved
  into the sketch plane exactly as `Revolve` resolves it;
- any positive finite pitch and any positive finite turn count, whole or
  fractional;
- a right-hand coil by default, a left-hand coil through `WithLeftHand()`;
- a solid result only.

The build refuses, with Table CS, a profile that touches or crosses the
axis, a profile wider along the axis than one pitch when the coil makes one
turn or more, a free-form, elliptical or trimmed profile segment,
`WithSurfaceResult()`, and a station count past the fixed cap.

Outside this design, with no increment planned here:

- a `Helix` segment in `Path` (§12 states why `Coil` is a verb);
- a section carried normal to the helix (a Frenet or rotation-minimizing
  frame) rather than in the axis plane (§3 states why);
- a variable pitch, a conical (tapered) helix, a spring's closed or ground
  end, and a thread's lead-in chamfer: each is a later boolean or a later
  build, never a parameter of this one;
- `CoilChain` (an open sketch chain swept into a helicoidal sheet);
- modify operations on a coil.

`Revolve` remains the operation for a zero pitch. `Coil` refuses a zero
pitch (CS4) rather than reducing to it.

## 2. Public API

```go
// Coil moves the closed profile p of sketch s along the screw motion about
// axis: turns full rotations about the axis, right-handed about the axis
// direction, advancing pitch along that direction per rotation. The axis
// MUST lie in the sketch plane, and the profile MUST lie strictly on one
// side of it. The section stays in the axis plane throughout, so a thread
// profile drawn beside the axis cuts the thread it draws.
func (d *Document) Coil(
    ctx context.Context,
    s *sketch.Sketch,
    p *sketch.Profile,
    axis Axis,
    pitch units.Value,
    turns units.Value,
    opts ...CoilOption,
) (*Body, error)

// CoilOption configures Coil. Sealed.
type CoilOption interface { /* sealed */ }

// WithLeftHand turns the section left-handed about the axis direction while
// it advances along it. The default is right-handed.
func WithLeftHand() CoilOption
```

| Parameter | Kind | Meaning |
|---|---|---|
| `axis` | `Axis` | `SketchLine`, `ConstructionAxis` or `EdgeAxis` — `Revolve`'s own sealed vocabulary (`revolve.go`), resolved into the sketch plane by `axisInPlane` (`revolve_axis.go`). The axis direction fixes the advance direction |
| `pitch` | `units.Length`, finite, `> 0` | the axial advance per full turn |
| `turns` | `units.Dimensionless` (`units.Scalar`), finite, `> 0` | the number of full turns; the total angle is `Θ = 2π·turns` |
| `WithLeftHand()` | option, at most once | reverses the rotation sense; the advance direction is unchanged |

Stating the extent in turns rather than in height keeps the angular extent
a RATIONAL multiple of a turn: `turns` is a float, so `Θ/2π` is exact, and
the height `pitch·turns` is an exact rational product. A thread of length
`L` at pitch `p` passes `units.Scalar(L/p)`; the body then denotes
`turns` as the float the caller passed, and the height it denotes is
`pitch × that float`, never `L` itself. The executable example states this
in its comment.

A profile beside a `SketchLine` axis is the common thread case: the sketch
holds the V or trapezoid groove profile and a construction line for the
axis. Both pass through the unchanged sketch seam, and the seam's own
sentinel wins before any axis geometry is read.

## 3. Table CP — the screw motion and the section frame

Resolve the axis into the sketch plane as `Revolve` does: `axisInPlane`
gives a plane-local point `a = (aU, aV)` and a unit direction
`d = (dU, dV)`, each with the rounding bound `revolveaxis.AxisInPlane`
proves. Let `n` be the world unit vector along `d` (lifted through the
plane frame), `e_r` the world unit vector along the plane-local normal
`(−dV, dU)` or its negation — the sign that puts the profile on the
POSITIVE side, decided by `revolveaxis.ResolveSide` — and `e_t = n × e_r`.
`e_t` is the plane's own normal or its negation, decided by an exact sign.
Let `C` be the world point of `a`.

Every profile point has plane-local axis coordinates `(ρ, ζ)`: `ρ` its
signed distance from the axis along `e_r`, `ζ` its coordinate along `n`,
each a bounded reading charging the axis frame's own bound exactly as
`Revolve`'s radial and axial readings do.

Write `k = pitch / 2π`, `Θ = 2π·turns`, and `σ = +1` for a right-hand coil
and `−1` under `WithLeftHand()`. The screw motion is

```text
Φ(ρ, ζ, θ) = C + ρ·(cos θ · e_r + σ·sin θ · e_t) + (ζ + k·θ) · n,   θ ∈ [0, Θ]
```

| CP | Rule |
|---|---|
| **CP1** | The section at angle `θ` is the recorded profile under the rigid motion `Φ(·, ·, θ)`: a rotation by `σθ` about the axis followed by a slide `kθ` along it. It lies in the plane through `C + kθ·n` spanned by `n` and `cos θ·e_r + σ sin θ·e_t`, which CONTAINS the axis. |
| **CP2** | The section at `θ = 0` is the recorded profile in its own plane, exactly: `Φ(ρ, ζ, 0)` is the recorded lift. The start cap is the recorded region. |
| **CP3** | No Frenet and no rotation-minimizing frame is used. The section is never normal to the helix; the helix tangent makes the lead angle `atan(k / ρ)` with the section plane at radius `ρ`. |
| **CP4** | The map `Φ` has Jacobian determinant of magnitude `ρ`, the same as a revolve's. Every closed form in Table CM is a revolve's with `Θ` for the sweep angle plus the slide's own term. |
| **CP5** | `Φ` is injective on `Ω × [0, Θ]` whenever `ρ > 0` on `Ω` and either `turns < 1` or the profile's axial extent `H = max ζ − min ζ` is below `pitch`. Proof: `Φ(ρ,ζ,θ) = Φ(ρ',ζ',θ')` projects onto the plane normal to `n` as `ρ·e(θ) = ρ'·e(θ')` with both radii positive, so `ρ = ρ'` and `θ' = θ + 2πm`; the axial components then give `ζ' = ζ − pitch·m`. For `turns < 1` only `m = 0` fits in `[0, Θ]`. For `turns ≥ 1`, `m ≠ 0` needs two points of `Ω` at one radius whose axial coordinates differ by at least `pitch > H`, which `Ω` has none of. |

**Why the axis plane.** A thread is DEFINED by its axial section: ISO,
UN and trapezoidal thread forms state the groove in the plane through the
axis, and a cutter that holds that section while it screws along the axis
cuts exactly that thread. A section carried normal to the helix would cut a
different groove, tilted by the lead angle, and would have no closed-form
volume or centroid of its own. The axis-plane motion is a revolve with a
slide, so every reading a revolve has, this build has (CP4), and injectivity
is a one-paragraph proof (CP5) rather than a facet audit. A round-wire
spring modelled this way has a wire whose section normal to the helix is
not exactly circular; it is exactly the wire a lathe-wound coil of that
axial section has, and its volume is exact in the same closed form.

## 4. Table CS — what refuses a coil

The existence rule applies: a requested solid that does not exist is
`ErrDegenerate`; a solid that exists but this evaluator cannot build is
`ErrUnsupported`.

| CS | Condition | Sentinel |
|---|---|---|
| **CS1** | nil context, document, sketch, profile or axis; nil or foreign option; `WithLeftHand()` repeated | `ErrDegenerate` |
| **CS2** | profile seam rejection | the seam's own sentinel |
| **CS3** | an axis variant this evaluator cannot resolve, or an axis that does not lie in the sketch plane | what `axisInPlane` answers: `ErrUnsupported` for the variant, `revolveaxis.AxisInPlane`'s own sentinel for the placement |
| **CS4** | `pitch` not a length, or `turns` not dimensionless; either non-finite; either at or below zero | `ErrUnitKind`; `ErrNotFinite`; `ErrDegenerate` (a zero pitch names a revolve, a zero turn count names no solid) |
| **CS5** | the profile's near-axis radial extreme is not proven positive: proven at or below zero refuses outright; an interval straddling zero (a tilted axis whose own rounding leaves the sign undecided) refuses as undecided | `ErrDegenerate`; `ErrUnsupported`. Stricter than `Revolve`'s side gate: a point on the axis sweeps to a segment of the axis and pinches the wall, so no contact is admitted |
| **CS6** | `turns ≥ 1` and the upper bound of the profile's axial extent `H` is at or above `pitch` | `ErrUnsupported`: CP5 proves the solid simple only below one pitch; the refusal names `H`, `pitch` and the turn count |
| **CS7** | a profile segment, on any loop, that is not a `LineSeg`, `ArcSeg` or `CircleSeg`; a trimmed segment of any kind, a whole circle sharing its loop, or a loop whose recorded segment ends do not meet exactly, none of which has an exact recorded vertex for station 0 to hold at the junction (no increment lifts these) | `ErrUnsupported` |
| **CS8** | the station count `N = ⌈turns · coilStationsPerTurn⌉` exceeds `maxCoilStations`, or the wall and cap triangle count exceeds `maxCoilFacets = 1 << 20` | `ErrUnsupported` (a resource ceiling) |
| **CS9** | the held crossing audit proves two non-adjacent held triangles meet; or its pair budget runs out | `ErrUnsupported` in both arms: CP5 has already proven the TRUE solid simple, so a held crossing is a station artefact (two true turns closer than twice the chord departure), never a defect of the solid |
| **CS10** | a computed station, vertex, reading or bound is non-finite, a held triangle collapses from rounding, or the denoted map's orthonormality defect (§5.3) is at or above `1/2` | `ErrUnsupported` |
| **CS11** | `WithSurfaceResult()` — it is not a `CoilOption`, so this is a compile-time refusal, stated here so no later increment admits it silently | — |

Gate order is normative:

1. Validate context, nils, owned options and option arity (CS1).
2. Authenticate and record the planar profile (CS2).
3. Validate pitch and turns (CS4).
4. Resolve the axis into the plane (CS3), read the profile's radial and
   axial extremes, decide the side (CS5) and the pitch clearance (CS6).
5. Check every profile segment's kind (CS7) and preflight the station and
   facet counts (CS8).
6. Build the held shell, round once, orient once, run the crossing audit
   (CS9, CS10).
7. Compute the four readings and their bounds (Table CM).
8. Commit once.

Every failure leaves document membership and producer numbering unchanged.

## 5. Construction

### 5.1 Axis coordinates

`axisInPlane` and `revolveaxis.ResolveSide` give the axis line and the
profile's side exactly as `Revolve`'s step 4 does, over the same
`SideExtremes` the profile's walk resolves (`revolveaxis.ResolveLoop`).
CS5 reads the near-axis extreme's proven interval, which `ResolveSide`
returns as `SideResult.Near`: lower end above zero admits; upper end at or
below zero is `ErrDegenerate`; anything else is `ErrUnsupported`. CS6 reads the axial extremes' outer interval. Both are
reject-only readings off the profile's own record.

### 5.2 Stations

The sweep is cut at `N + 1` stations, `N = ⌈turns · coilStationsPerTurn⌉`,
at the exact turn fractions `t_j = turns · j / N`, `j = 0 … N`. Every `t_j`
is a rational, `t_0 = 0` and `t_N = turns` exactly. The angle at station
`j` is `θ_j = 2π t_j`, enclosed through `coil.TurnSinCos(t_j)`: a multiple
of a quarter turn reads its exact sine and cosine, and every other turn reads
`proofbound.TurnSinCosInterval(t_j)`, whose series margin keeps even an
octant boundary a few `2^-200` grid steps wide. A whole, half or quarter turn
is therefore exact. The slide at station `j` is `pitch · t_j`, an exact
rational.

`coilStationsPerTurn` is `256`, a power of two, and `maxCoilStations` is
`1 << 15`; the defining source constants own both values and their
derivation. At 256 stations per turn the helix sag (§5.4) is
`ρ_max · π²/(2·256²) ≈ 7.5e-5 · ρ_max`. The twist term is first order in
the station step, but §5.4's shifted correspondence moves its tangential
part along the true surface and leaves only the part normal to it: a segment
whose ends sit at radii `ρ_min < ρ_max` carries about
`|Δρ| · (π/256)/2 · (π/256 + k/ρ_min)` while `|Δρ| ≤ ρ_min`. The square
spring of §13 reads `δ ≈ 1.04e-3 mm`, and a profile `ρ ∈ [2, 4]` reads
`δ ≈ 1.95e-3 mm`, each inside the default `Verify` tolerance
(`docs/verification-design.md` §2). A profile whose radial run exceeds its
inner radius loses part of that gain (§5.4's `c < 1`) and can still read
`Bounds` past the tolerance and verify `Suspect`: the profile
`ρ ∈ [0.5, 3]` reads `δ ≈ 1.4e-2 mm`. Only `Bounds` and the faceted bounds
carry `δ`; the three other readings are closed forms. The cap admits 128
turns of any profile, and the facet-pair budget
(`proofbound.MaxFacetPairTestsPerCall`) bounds what the audit can run.

### 5.3 Held vertices

**The profile station chain.** The held profile is a chain of stations,
loft §5.1's chord chain read on one section (`coil.Loops`). A `LineSeg`
contributes its walk start, a recorded point. An `ArcSeg` or a whole
`CircleSeg` contributes its walk start and the `m − 1` interior stations of
its `m` chords, at the exact parameters `t_k = TStart + (k/m)·(TEnd − TStart)`,
so a wall cell is chorded along the profile as well as along the helix. An
arc of sweep `Δ` takes `m = ⌈coilArcChordsPerTurn·Δ/2π⌉` chords, at least
one, and a whole circle takes `coilArcChordsPerTurn`, both decided from the
held float sweep, which decides a count and nothing else.
`coilArcChordsPerTurn` is `16`: every chord cell enters §5.5's crossing
audit beside the helix stations, and at 16 a 5-turn round wire holds 40 960
wall triangles, inside the audit's ceiling of 65 528 triangles; at 24 it
holds 61 440, and a sixth turn passes the ceiling. A whole circle at 16
chords reaches the ceiling at 8 turns.

Every station carries a plane round `s_v`, the distance from its held point
to the point the record denotes there: zero at a line end and at an arc's
`Start`, which is the denoted start; the radial residual
(`circularbounds.ArcRadialResidualUpper`) at an arc's `End`, which the arc
denotes on `Start`'s radius at `End`'s angle and which the next segment
shares as its walk start; and at a generated station, the plane distance
from the float nearest the midpoint of `circularbounds.EndpointInterval`'s
enclosure to its far corner. A whole circle records no point at all, so even
its walk start is generated. The station's held coordinates `(ρ_v, ζ_v)` lift
the held point exactly; every bound and closed form reads them widened by
`s_v`, which encloses the denoted point.

For profile station `v` with axis coordinates `(ρ_v, ζ_v)` and helix station
`j`:

```text
X(v, j) = C + ρ_v·(cos θ_j·e_r + σ sin θ_j·e_t) + (ζ_v + pitch·t_j)·n
```

The build evaluates it in the plane frame's coordinates as
`p_v + ρ_v·(cos θ_j − 1)·e_r + pitch·t_j·d` in the plane and
`σ·Side·ρ_v·sin θ_j` along the plane normal, since `e_t = Side·N`; the
recorded point `p_v` is exact, so station 0 and every whole turn's station
carry no axis-frame width.

**The denoted map.** The frame's held `U`, `V`, `N` and the placement's held
basis and translation are read as exact rationals, the convention the loft
lift and the mitred placement already follow: the coil denotes the image of
the plane-coordinate screw sweep under the affine map
`x ↦ O + L·x`, `L = B·[U V N]`. r3 does not make `L` exactly orthonormal.
`r3.NewFrame` and `r3.FromBasis` normalize in float64, so each stored axis is
unit and orthogonal only to a few ulps, and `IsValid` admits a departure up
to `1e-9`; `N` is the float cross product `U × V`, the same leaf the prism
family's vertices and readings read (`docs/evaluator-design.md` §5.1). A
revolve reads the exact `U × V` instead, because its sweep's own `E1` is a
cross product (§6 there). Each payload's readings and its held vertices read
the same three columns, so every reading covers the solid its own vertices
denote, and the two conventions differ by `N`'s own rounding. The build
therefore reads
two numbers off `L`'s exact columns: `det L`, exactly, and the orthonormality
defect `e`, the entrywise absolute sum of `LᵀL − I`. Every eigenvalue of
`LᵀL` lies in `[1 − e, 1 + e]`, so `L` scales a length and an area by a
factor in `[1 − e, 1 + e]` and a volume by exactly `|det L|`. Table CM
charges both: `Volume` is `|det L|·Θ·Q`; every area and length closed form
is widened to `[lo·(1 − e), hi·(1 + e)]`; §5.4's analytic legs, derived in
plane coordinates, are multiplied by `1 + e`. The centroid of an affine image
is the image of the centroid, so `Centroid` maps the plane-coordinate closed
form through `L` exactly and needs no charge. An axis-aligned sketch plane
with the identity placement has `det L = 1` and `e = 0` exactly, and every
reading is then the plane-coordinate closed form unchanged. A defect at or
above `1/2` refuses `ErrUnsupported` (CS10); r3's own `1e-9` admission keeps
every real frame far below it.

**The held coordinates.** Each per-vertex term `P_v`, `B_v`, `S_v`, each
station's `cos θ_j − 1`, `sin θ_j` and slide `t_j·D` is an exact interval,
held once as the float nearest its midpoint with the outward distance to its
far end. The station point is then evaluated in float64,
`P_v + (cos θ_j − 1)·B_v + t_j·D + sin θ_j·S_v`, each product and sum rounded
once by an explicit conversion so no step fuses into another, and its bound
charges both operands' bounds and that operation's exact rounding
(`coil.Mul`, `coil.Add`; the exact residual comes from `math.FMA` or TwoSum).
`round(v, j)` is the largest coordinate's bound, turned into a 3D radius by
`proofbound.Radius3D`, plus `(1 + e)·s_v`: `Φ` is a rigid motion of the axis
plane at every `θ`, so the held profile station's image sits `s_v` from the
denoted point's in plane coordinates. Station 0 holds the recorded lift `P_v`
exactly where the lift is exact (CP2): every other term is an exact zero
there, and adding an exact zero rounds nothing.

The held vertex table is station-major: vertex `v` of station `j` is index
`j · stride + v`, with `stride` the profile's station count over every loop,
in `coil.Loops`'s loop-major order.

### 5.4 Chord departure and the per-vertex bound

A wall cell is profile chord `v → w` over stations `j → j + 1`. On a line,
the chord is the segment; because it is straight, `Φ` is linear along it at
every fixed `θ`, so the true cell is a ruled surface whose rulings are the segment's images and
whose two edges are helix arcs of radii `ρ_v` and `ρ_w` over `Δθ = 2π/N`
per turn fraction `Δt = turns / N`. The held cell is the two triangles the
quad splits into along the diagonal `tessellate.go` uses for a prism's
lateral quad.

Write `h = Δθ/2 = π·Δt`, `k = pitch/2π`, `Δρ = ρ_w − ρ_v`,
`ρ_min`/`ρ_max` the smaller/larger of the cell's two radii, and parametrize
the cell by `(λ, s) ∈ [0, 1]²` with `θ(s) = θ_j + 2h·s`. The held point at
`(λ, s)` is the barycentric point of the two held triangles (the diagonal
runs from `(0, 0)` to `(1, 1)`); `Tri(λ, s)` is the same point on the four
TRUE corners, `B(λ, s)` the bilinear patch through them and `S(λ, θ)` the
true surface. On the cell, `Tri − B = m·T` with
`m = min(s(1 − λ), λ(1 − s)) ≤ 1/4` and the true twist vector
`T = Δρ·(e(θ_{j+1}) − e(θ_j)) = 2Δρ·sin h·t̂(θ_mid)`, which is TANGENT to
the helix: the matched-parameter twist `|T|/4 = |Δρ|·sin h/2` is first order
in the step but lies almost entirely along the true surface.

**The shifted correspondence.** Map the held point at `(λ, s)` to
`H(λ, s) = S(λ, θ(s) + ε)` with `ε = c·m·2Δρ·sin h / ρ(λ)` and
`c = min(1, ρ_min/|Δρ|)`. `ε` vanishes on the cell's boundary, so `H` agrees
across cells and with the matched map on every edge; `c` keeps
`|ε| ≤ 2h·min(s, 1 − s)`, so `θ(s) + ε` stays in the cell's own angular
span and `H` lands on the true cell. The map `(λ, s) ↦ (λ, θ(s) + ε)` is the
identity on the square's boundary up to the affine `θ(s)`, so it has degree
one onto the parameter rectangle and `H` reaches EVERY true point of the
cell: the correspondence is two-sided without any monotonicity argument.
Expanding `S` in `θ` to second order (`∂θS = ρ·t̂ + k·n`, `|∂²θS| = ρ`):

```text
Tri − H = (B − S(λ, θ(s)))
        + m·2Δρ·sin h·[(1 − c)·t̂(θ_mid) + c·(t̂(θ_mid) − t̂(θ(s))) − c·(k/ρ(λ))·n]
        − R₂,          |R₂| ≤ ε²·ρ(λ)/2
```

with `|t̂(θ_mid) − t̂(θ(s))| ≤ h`. The `n` component is the part of the
twist normal to the true surface, scaled by `k/ρ` against the tangential
part the shift absorbs.

| Term | Bounds | Derivation | Rounding |
|---|---|---|---|
| `sag(cell)` | `\|B − S(λ, θ(s))\|` | each helix arc departs from the linear interpolation of its chord at the same `θ` by at most `ρ·(2h)²/8`, since its circular part has curvature vector of length `ρ` (the slide is linear in `θ` and the chord interpolates it exactly), and `S` and `B` are both linear in `λ` between the two arcs; take `ρ_max·h²/2`. The sagitta `ρ·(1 − cos h)` bounds the arc's midpoint only, not the matched departure at every `θ` | up |
| `twist(cell)` | the bracket above | `\|Δρ\|·h/2·((1 − c) + c·(h + k/ρ_min))`, from `m ≤ 1/4` and `sin h ≤ h` | up |
| `shift(cell)` | `\|R₂\|` | `c²·Δρ²·h²/(8·ρ_min)`, from `\|ε\| ≤ c·\|Δρ\|·sin h/(2ρ(λ))` | up |
| `arcSag(cell)` | an arc chord's own departure: `\|A(φ(λ)) − lerp(λ)\|` | the recorded arc departs from the linear interpolation of its chord between its two denoted ends, at the matched parameter, by at most `r·Δφ²/8`, its curvature vector being of length `r·Δφ²` in `λ`; read with the radius's and the sweep's upper ends over `m`. Zero on a line | up |
| `round(v, j)` | the held corner's distance from the point `X(v, j)` denotes | §5.3, with the station's plane round | up |
| `β(v, j)` | the per-vertex bound of `docs/faceted-vertex-bounds-design.md` §2 | the largest `(1 + e)·(sag + twist + shift + arcSag) + maxRound` over the cells that touch the vertex, where `maxRound` is the cell's largest corner `round` (the held triangles lie within it of `Tri`, by convexity) and `1 + e` carries the plane-coordinate legs through `L` (§5.3). It is never below `round(v, j)`; a cap vertex touches cells on one side only | `absSumUpper` |
| `δ` | the payload's displacement | the largest `β` over the table | max |

**An arc chord.** The true cell of an arc chord is the screw sweep of the
arc piece `A(φ(λ))`, `φ` affine in `λ` between the chord's two stations,
not of the chord through its two denoted ends. The shifted correspondence
carries over: map the held point at `(λ, s)` to
`H_arc(λ, s) = Φ(A(φ(λ)), θ(s) + ε)`, with `ε` the chord's own shift. `Φ`
is a rigid motion of the axis plane at each `θ`, so `|H_arc − H| ≤ arcSag`
pointwise, and `(λ, s) ↦ (φ(λ), θ(s) + ε)` keeps degree one onto the arc
cell's parameter rectangle, so the correspondence stays two-sided. The
chord's own legs read the radii of its two DENOTED ends, the held radii
widened by the stations' plane rounds; the triangles on those denoted
corners lie within `maxRound` of the held ones.

`coil.CellDepartureUpper` evaluates `sag + twist + shift` exactly over the
radii's interval ends, once per chord, and the build adds `arcSag`: the
legs depend on the chord and the station step alone. Every interval end is read on the side
that makes the sum larger, and `c` is computed from the same ends, so the
`c` the bound uses satisfies the range condition for the true radii too.

`β` folds each facet's own departure into every corner, which is what
`docs/faceted-vertex-bounds-design.md` §2.1 requires of a curved payload
that publishes a per-vertex record: every held point of a cell is within its
cell's departure of a true point under `H`, and every true point of the cell
is `H` of a held point, so the facet bound `max corner β` is two-sided.

§8.2's wall homotopy runs along the same shifted correspondence. §8.1's
area-density legs stay at matched parameters `(λ, s) ↦ S(λ, θ(s))`.

### 5.5 Triangles, orientation and the crossing audit

Walls are emitted ring by ring, `j = 0 … N − 1`, loop by loop, chord by
chord, two triangles per cell in the prism's lateral order; both carry the
role `side(i, j_seg)` of the recorded segment whose chord generated them.
`capStart` is `triangulate.go`'s triangulation of the station polygon,
reversed by swapping each triangle's second and third vertices; `capEnd` is
the same index triples on station `N`, retained — the identical seeding
loft §5 and sweep §16.3 use. Where a chord stands for an arc, the true cap
adds or removes the circular segment between them, every point of which
lies within `arcSag` of both; the chord's two end stations carry `β` at or
above `(1 + e)·arcSag`, so the cap facet holding the chord covers it. The whole shell is then oriented once by the sign of
the exact tetrahedron sum over the held floats (each a rational), anchored
at `C`; a negative sum reverses every triangle. A collapsed held triangle
is CS10.

`loftmesh.LoftCrossingAudit` runs over the complete held set with
`proofbound.MaxFacetPairTestsPerCall` as its pair ceiling (CS9). No
`F·(F−1)/2` preflight runs: the audit's sweep-and-prune charges candidate
pairs, and the facet cap of CS8 bounds the set. The true solid's simplicity
is CP5's; the audit proves the HELD shell embedded, which is what every
mesh consumer (`BoundaryVerified`, the boolean) reads.

### 5.6 Placement

A placement re-runs §5.1–§5.5 from the record under the composed map
`L = B·[U V N]` with the composed translation, both read exactly (§5.3):
the stations and trig are recomputed, and the held table, `β`, `δ`,
orientation and audit are rebuilt. `δ` never accumulates across placements.
The four readings are re-derived from the closed forms under the composed
map: `Volume` is the plane-coordinate closed form times the composed
`|det L|`, and `Area` the plane-coordinate closed forms widened by the
composed defect, so each stays within both bodies' bounds of the unplaced
reading and is bit-identical only when the placement's basis is exactly
orthonormal; `Centroid` is the composed image of the plane-coordinate
centroid, rounded once; `Bounds` reads the new held table.

## 6. Table CB — the result

For loop `i` (`0` the outer loop, `1 + h` for hole `h`), recorded profile
segment `j` of that loop, and junction `v`, the walk start of a segment:

| Entity | Count | Geometry | Role |
|---|---|---|---|
| start cap | 1 | `Plane` over the recorded region, frame the recorded plane's own (exact) | `capStart` |
| end cap | 1 | `Plane` over the section at `θ = Θ`, frame from the station-`N` trig, within `δ` | `capEnd` |
| wall | one per `(i, j)` | `Faceted{Bound}` — the whole helicoidal band of segment `j` over every turn, `Bound` the largest `β` over the vertices its triangles touch | `side(i, j)` |
| rim edge | one per profile segment per cap | `Line3` between the cap's two held vertices; `Arc3` for an `ArcSeg`, its centre and axis the arc's centre and the section plane's normal lifted through `Φ` at `θ = 0` or `θ = Θ`, signed so the rim runs counter-clockwise from its start vertex to its end vertex; `Circle3` for a whole circle or an arc whose `End` is its `Start` alone in its loop, which closes on one vertex. Every circular rim carries `Edge.curveBound`: twice the axial reach `|A·D| + r·(|A·u| + |A·v|)`, plus `|D|`, `r·e` and `|r − R|`, with `D` the denoted centre less the held one, `u`, `v` the denoted map's images of the turned in-plane axes, `e` their orthonormality defect, `A` the held axis and `R` the held radius (`coilRimCurveBound`) | through its two faces' origins |
| helix edge | one per junction | `FacetedCurve{Bound}`: the chain of held chords of junction `v` over every station, `Bound` the largest `β` along it | through its two walls' origins |
| vertex | one per junction per cap; a whole circle's walk start is its seam vertex | position the held station-`0` or station-`N` point, bound its `β` | — |

Interior stations, helix and chord alike, are mesh vertices, never
topology: a wall is ONE face whose loop is rim, helix edge, rim reversed,
helix edge reversed. For a hole-free profile of `m` segments the body has
`2 + m` faces, `3m` edges and `2m` vertices. A whole circle has no
junction: its wall is a band with two loops, its two rim circles, the
prism's closed-band rule, so a round wire has 3 faces, 2 edges and 2
vertices. An `ArcSeg` whose `End` is its `Start`, alone in its loop, sweeps
the same whole turn and builds the same band, so no wall loop holds one
edge twice. One lump, one outer shell; a hole loop is a void passage
through every turn, never a second lump.

`Edge.IsConvex` keeps evaluator §3's meanings. A helix edge is a junction
edge and takes the sign of the profile's own corner turn at `v`, exactly as
a prism's vertical edge does: left turn convex, read off the two exact
walk tangents there (a line's run, an arc's radius turned a quarter turn in
the walk's sense), so a tangent junction is not convex. A rim edge takes
the rim rule: a straight wall by the role of its loop, outer convex, hole
concave; a circular one by its walk, counter-clockwise convex.

The wall's `Faceted` variant is the honest surface kind: the true band is
a helicoidal surface no sealed analytic variant names, and `Faceted`'s
contract (`internal/surfacegeom/geometry.go`) is that the face IS its
polygons within `Bound` of the surface it stands for. `Face.Area()` on a
wall reads Table CM's closed form for its segment, not the held triangles'
sum.

## 7. Table CM — measurements and bounds

Let `Q = ∫_Ω ρ dA`, `I = ∫_Ω ρ² dA` and `M = ∫_Ω ρζ dA` be the profile's
axis-frame moments and `A_Ω` its area. For a `LineSeg`-only profile they
are the polygon's own integrals over every vertex's `(ζ, ρ)` interval
(`coil.RegionMoments`), exact rationals for an exact axis frame, and `A_Ω`
is the exact shoelace area of the recorded vertices. A profile with an arc
reads the recorded region's plane-origin integrals `∫dA`, `∫u dA`, `∫v dA`,
`∫u² dA`, `∫uv dA` and `∫v² dA`, the ones `Revolve` reads, each enclosed by
its own proven bound, and re-references them into the axis frame over
intervals (`coil.Axis.Moments`): with `x = u − a_U`, `y = v − a_V`,
`ρ = e_r·(x, y)` and `ζ = d·(x, y)`, so `Q`, `I` and `M` are linear in the
anchor-relative first and second moments.

| Reading | Closed form | Exactness and bound |
|---|---|---|
| `Volume` | `\|det L\| · Θ · Q = \|det L\| · 2π · turns · Q` (CP4, Pappus, under §5.3's map) | `Approximate` always: `2π` enters through `proofbound.TwoPiInterval` and `det L` is exact, so the enclosure is that interval times the exact `turns · Q · \|det L\|`, rounded once. Relative width is of order `1e-16` |
| `Centroid` | `C + (I/Q)·(sin Θ / Θ)·e_r + σ·(I/Q)·((1 − cos Θ)/Θ)·e_t + (M/Q + pitch·turns/2)·n`, in plane coordinates, mapped through §5.3's `L` exactly | `sin Θ`, `cos Θ` from `TurnSinCosInterval(turns)`; at a whole number of turns both are exact and the transverse terms vanish, leaving `C + (M/Q + pitch·turns/2)·n`, which is `Exact` when the frame is exact and the rationals round exactly |
| `Area` | `2·A_Ω` for the two caps, plus per `LineSeg` `v → w` with `Δρ = ρ_w − ρ_v`, `Δζ = ζ_w − ζ_v`, `L² = Δρ² + Δζ²`: for `Δρ = 0`, `Θ · ρ_v · \|Δζ\|`; otherwise `(1 / (L·\|Δρ\|)) · [ Θ·(F₁ − F₀) + (turns · pitch² · Δρ² / 4π) · (asinh(u₁/m) − asinh(u₀/m)) ]` with `u = L·ρ`, `u₀ ≤ u₁`, `m = k·\|Δρ\|`, `F(u) = (u/2)·sqrt(u² + m²)`. Where `Δρ`'s enclosure contains zero without being exactly zero (a segment parallel to a tilted axis), the integrand lies between `L·ρ` and `L·ρ + m²/(2·L·ρ_min)`, so the area lies between `Θ·L·(ρ_v + ρ_w)/2` and that plus `Θ·m²/(2·L·ρ_min)`; per `ArcSeg` or whole `CircleSeg`, §11.1's bracket | `Approximate`: the π enclosure, the certified square root (`proofbound.SqrtFixed`, new) and `proofbound.AsinhInterval` (new, §7.1) each contribute their width; a cylindrical band (`Δρ = 0`) carries the π enclosure alone; an arc's wall carries §11.1's remainder. Every area, and every rim and helix edge length, is then widened to `[lo·(1 − e), hi·(1 + e)]` by §5.3's defect, a no-op when `e = 0` |
| `Bounds` | per-axis extremes over the held vertex table, widened outward by `δ` | `Approximate` with bound `δ` plus the largest station rounding plus the widening's outward step: the box holds the true body, and every held vertex is within its own rounding of a true point; `Exact` only when `δ = 0`, which no coil reaches |

The area integrand is derived once: `∂Φ/∂λ = Δρ·e_r(θ) + Δζ·n` and
`∂Φ/∂θ = ρ·σ e_t(θ) + k·n` give `|∂λ × ∂θ|² = L²·ρ² + k²·Δρ²`, so a
segment's wall area is `Θ · ∫₀¹ sqrt(L²·ρ(λ)² + k²·Δρ²) dλ` with
`ρ(λ) = ρ_v + λ·Δρ`; the substitution `u = L·ρ` gives the stated
antiderivative. A hand-written integrator never replaces it: §13's tests
compare the published value against an independent quadrature, and the
published BOUND must enclose that quadrature's own error.

Every bound composes through `proofbound.BoundedAdd`/`BoundedMul` and
rounds outward at every publication. `Exact` requires every term exact;
operation history grants nothing.

The tolerance-gate diameter is the loft arm's (`verify_gate.go`): the held
table's own `pointSetDiameterContext` less `2·δ`, rounded down.

### 7.1 `proofbound.AsinhInterval`

PR 1 adds `LnInterval(x RatInterval) (RatInterval, bool)` (false for an
interval not above zero), `AsinhInterval(x RatInterval) RatInterval` and the
square-root enclosures `SqrtFixed`/`SqrtInterval` to `internal/proofbound`,
on the fixed-point grid `turn_trig.go` already uses:

- `ln y` for a rational `y > 0`: write `y = 2^e · m` with `m ∈ [1, 2)`
  (exact), then `ln y = e·ln 2 + 2·artanh((m − 1)/(m + 1))`; the artanh
  series `Σ w^(2n+1)/(2n+1)` has positive terms and the tail bound
  `w^(2K+1) / ((2K+1)(1 − w²))`, with `w ≤ 1/3`; `ln 2` is the same series
  at `w = 1/3`. Evaluate at both ends of `x` and take the outer interval;
  `ln` is increasing, so the ends decide the enclosure.
- `asinh x = ln(x + sqrt(x² + 1))`, the square root enclosed by
  `SqrtFixed`: with `N = floor(q·4^P)` and `s = isqrt(N)`,
  `s/2^P ≤ sqrt(q) < (s + 1)/2^P`, a point when `q·4^P` is itself `s²`; odd
  symmetry for `x < 0`; exactly zero at zero.

Tests: `ln 1 = 0` exactly, `ln 2` against the held constant, random
rationals against `math.Log`/`math.Asinh` (enclosure contains, width below
`2^-150`), and the §13 area fixtures.

## 8. Table CD — downstream

| CD | Consumer | Status |
|---|---|---|
| **CD1** | structural `Verify` + tolerance gate | lands with PR 1. Validity is proven by construction: CP5 over CS5/CS6 proves the true solid simple, and the held audit proves the shell embedded, so `payloadProvesSimple` answers true for a `coilPayload`. All four readings are judged. The gate diameter is §7's |
| **CD2** | `Tessellate` / STL / OBJ / 3MF / faceted STEP | landed (`tessellate_coil.go`) as an exact restatement of the held triangles at any tolerance at or above `δ` (below it `ErrUnsupported`, `docs/tessellation-design.md` §7's rule): each vertex's `β` as its `vertexBound`, `sourceBound(face)` the largest corner `β` over that face's triangles, `Bound` `δ`, `areaSlack` §8.1's term, `volSymDiff` §8.2's with `symDiffOK == true`, and the boundary proof the build's own audit. `docs/tessellation-design.md` §2's table carries the `coilPayload` row |
| **CD3** | mesh booleans | landed: an operand on `docs/tessellation-design.md` §11's terms, through CD2's proof. A coil raises the pair's chord tolerance to its `δ` (`heldPrimitiveFloorOf`, `docs/faceted-vertex-bounds-design.md` §5), so every facet it touches the partner with holds `β ≤ δ ≤ tol` and the chain-depth gate admits it; the partner is meshed at that tolerance, more coarsely than the pair's size alone would ask, with its own bounds proven. §9 is the use case |
| **CD4** | interference | landed through `Verify`'s read-only mesh intersection (`docs/interference-design.md` §5) for every partner the mesh boolean admits; a partner the chain-depth gate refuses reads the pair as unsupported staging |
| **CD5** | clearance | landed: `WithClearances` reads the exact planar arm (`clearance_planar.go`; `planarCoilSolid` in `contact_faceted_pair.go` lifts the held shell with `δ` the payload's) against a prism or a stitched solid, the held gap widened by `δ` outward; a held touch or a gap or overlap within `δ` stays undecided. `Suspect` against every other payload, a mitred sweep, a faceted result and another coil included |
| **CD6** | `Wall`, `Undercut`, `ConcaveRadius` | `Unavailable` with the unsupported-survey diagnostic |
| **CD7** | `Placed`, `Duplicate`, `PlacedCopy` | §5.6 |
| **CD8** | modify operations | `ErrUnsupported`; no receiver row |
| **CD9** | mass properties (`dynamics`) | landed through the verified-mesh ladder (`mass_properties_mesh.go`): the restated mesh settles at its first step, its occupied volume widened by §8.2's `volSymDiff` |
| **CD10** | STEP | the faceted AP214 writer: a `Faceted` wall has no analytic arm, so the whole body writes as held facets, which `docs/step-export-design.md` already states for any body with a non-planar, non-cylindrical face |

### 8.1 `areaSlack`

The area allowance follows every wall cell along a chain of three surfaces,
then adds the caps:

1. held triangles → bilinear patch on the held corners: the gap between the
   patch's area and the triangle pair's, per cell, the loft's own
   cancellation-preserving treatment (`docs/loft-design.md` §5.2). It reads
   `proofbound.CellTwistAreaProjectedAllow`, which charges only the part of
   the area element's variation normal to its mean: on a coil cell the twist
   runs along the surface, so that part is second order in the station step
   and the cell's gap is third order. Where the arm states no bound it falls
   back to `proofbound.CellTwistAreaAllow`;
2. that patch → bilinear patch on the TRUE corners: each corner moves at
   most `r`, the largest station rounding, so each derivative moves at most
   `2r` and the density at most `2r·|∂sB| + (|∂λB| + 2r)·2r`, with
   `|∂λB|` at most the world ruling and `|∂sB|` the world chord, each bounded
   by its plane-coordinate value times `1 + e` (§5.3);
3. that patch → the true cell at matched parameters, pointwise. Both
   densities have closed forms: in the frame at the cell's mid angle
   `B = (ρ(λ)·c(s), ζ(λ) + k·θ(s))` with `c(s)` the unit circle's chord, so
   `J_S² = 4h²(Δζ²ρ² + k²Δρ² + Δρ²ρ²)` and
   `J_B² = 4sin²h(Δρ(2s − 1)hk − Δζρ)² + 4h²k²Δρ²cos²h + 4ρ²Δρ²sin²h·cos²h`.
   `|J_S − J_B| = |J_S² − J_B²|/(J_S + J_B)` with `J_S ≥ 2h·L·ρ_min`, and
   the difference integrates over the cell to at most
   `(16/3)h⁴k²Δρ² + (16/3)ρ_max²Δρ²h⁴ + (4/3)Δζ²ρ_max²h⁴ + 4h³k|ΔζΔρ|ρ_max`
   (`coil.CellProof.Density`; its doc comment derives each term). It is
   derived in plane coordinates; `L` carries a density by a factor in
   `[1 − e, 1 + e]` that differs between the two surfaces' tangent planes,
   so the world gap adds `2e` times the patch's own area, at most
   `L·helix`, to `(1 + e)` times the plane gap. On an arc chord the true
   cell is the arc piece's sweep, not the chord's, and the leg adds the gap
   between the two: both densities are `2h·|(L·ρ, k·a)|`, `a` the radial
   part of `∂λ`, so they differ by at most
   `2h·[(Arc − L)·(ρ_max + arcSag) + L·arcSag + (8/3)·k·arcSag]`, with
   `Arc = r·Δφ` the arc piece's length (the derivation sits on
   `coil.CellProofUpper`).

Both caps add `proofbound.PerturbedTriangleAreaAllow` at `δ` per triangle:
the true cap is the planar section on the true corners, plus or minus the
circular segment between each arc chord and its arc, of area at most
`r²·Δφ³/12 ≤ Arc·arcSag`, which each cap adds once per arc chord. Every
term through `absSumUpper`. On §13's spring the allowance is near `0.064 mm²` of an area
near `128 mm²`, and the spring's union with a 60 mm core verifies `Sound`.

### 8.2 `volSymDiff`

Two homotopies of the whole closed boundary, each keeping it a closed
2-cycle, carry the held shell `M` to the true boundary `B`; any point whose
membership changes is swept, so their swept volumes bound `B △ M`
(`docs/tessellation-design.md` §11's argument):

1. `M` → the triangles on the TRUE corners, every vertex moving at most `r`
   along a straight line: `sweptVolumeAllow(r, perturbedAreaUpper(M, r))`,
   whose area argument covers every surface on the path. On a line-only
   profile the caps are then exact: the true cap is the planar section on
   the true corners.
2. Those triangles → the true walls under §5.4's shifted correspondence
   `H`, cell by cell: `G(λ, s, τ) = Tri + τ·(H − Tri)`. `H` agrees with the
   matched map on every cell edge (`ε` vanishes there), so adjacent cells
   share their edge motion. A line's rim edge is a ruling the triangles hold
   exactly, so its cap stays put; an arc chord's rim moves from the chord to
   the arc inside the cap's own plane, and the cap follows it there, which
   sweeps no volume. Per cell the swept volume is
   at most `∫|∂τG|·|∂λG|·|∂sG|`, with `|∂τG| ≤ sag + twist + shift + arcSag`
   (§5.4's plane-coordinate departure), `|∂sG| ≤ helix·(1 + q)` and
   `|∂λG| ≤ max(L, Arc) + helix·q·(1 + |Δρ|/(4ρ_min))`, `helix` read at
   `ρ_max + arcSag`, where `q = c·|Δρ|/ρ_min ≤ 1`
   bounds `|∂sε| ≤ 2h·q` and `|∂λε| ≤ 2h·q·(1 + |Δρ|/(4ρ_min))`
   (`coil.CellProof.Swept`). The homotopy runs in plane coordinates, and
   `L` maps its swept set to one of exactly `|det L|` times its volume.

```text
volSymDiff = sweptVolumeAllow(r, perturbedAreaUpper(M, r))
           + |det L| · N · Σ_chords CellProof.Swept
```

`symDiffOK` is true. No `Mesh.Bound × area` shortcut. On §13's spring the
proof reads near `0.14 mm³` against a volume of `10π mm³`.

## 9. The thread use case

Both threads are a `Cut` whose tool is a coil. The cylinder stays what it
was built as; the coil is the mesh-path operand CD3 admits; the result is a
`facetedPayload` (`docs/evaluator-design.md` §9).

**External thread.** A cylinder of radius `R` and height `L` from
`Revolve` or `Extrude`. In a sketch on a plane through the cylinder's
axis, draw the groove profile — a V or trapezoid whose root lies at radius
`R − depth` and whose mouth opens past `R` — beside a construction line on
the axis. `Coil` it with the thread's pitch over `turns` chosen so the coil
starts and ends INSIDE the cylinder's height: the coil's two caps then meet
the cylinder wall transversally and nothing is coplanar. `doc.Cut(ctx,
cylinder, coil)`.

```go
coil, err := doc.Coil(ctx, s, s.Profiles()[0], decad.SketchLine{Start: a, End: b},
    units.Millimeters(1.5), units.Scalar(8))
threaded, err := doc.Cut(ctx, cylinder, coil)
```

A thread that runs out of the cylinder's end is a coil that crosses the end
plane: start the groove a fraction of a pitch outside it, or build the coil
longer than the cylinder, and let the boolean trim it. The cut then removes
`∫_Ω ρ·(Θ − max(0, −ζ/k)) dA` over the groove's part inside `R`, measured
from the end plane: `Θ·Q + M⁻/k`, with `M⁻ = ∫ρζ dA` over that part below
the plane. A coil's cap never lies in the cylinder's end plane: `Φ(Ω, θ)`
lies in a plane through the axis, the axis plane at angle `θ`. What is
refused is a cap flush with another face in that same half-plane, such as
the start cap of a cylinder revolved through half a turn from the coil's own
sketch plane: that is the coplanar contact the mesh boolean's hidden-tangency
gate refuses (`docs/general-boolean-design.md` §2), and the repair is to
start the coil, or the half turn, at another angle.

**Internal thread.** A block with a bore (an `Extrude` of an annular
profile, or a block minus a cylinder). The groove profile sits at the bore
wall with its root at `R + depth` and its mouth opening into the bore past
`R`. `Coil` and `Cut` as above.

**What the result reads.** The cylinder's interior `ρ ≤ R` is invariant
under the screw motion, so the material the cut removes is the screw sweep
of the CLIPPED profile `Ω ∩ {ρ ≤ R}` (external) or `Ω ∩ {ρ ≥ R}`
(internal), and its volume is `Θ · Q(clipped)` by CP4. For a `LineSeg`
profile the clipped region is a polygon whose moment is an exact rational,
so the expected result volume `V_cylinder − Θ·Q(clipped)` is a closed form
the test computes independently.

| Reading on the threaded body | Value |
|---|---|
| `Volume` | `Approximate`: the mesh boolean composes the cylinder operand's own chord `volSymDiff` at the pair tolerance, the coil's §8.2 term, and the rim terms of `docs/faceted-vertex-bounds-design.md` §3. The published interval encloses the closed form above; that enclosure is §13's assertion |
| `Area`, `Centroid`, `Bounds` | `Approximate`, composed the same way |
| faces | `Faceted` throughout: a boolean result loses analytic identity (`docs/api-design.md` §6.1) |
| `Verify` | validity by the faceted verdict (`docs/payload-verification-design.md` §6.4): the held audit is clean and the feature scale — the crest-to-crest gap, a fraction of the pitch — clears `2·δ`, so validity is proven; `Status` is then the tolerance gate's answer over the composed bounds. On the §13 fixtures `Volume` sits inside the default tolerance and `Area` does not, nor, on the external thread, does `Centroid`, so both read `Suspect`: the rims run some 250 mm per helical edge and each carries its facet pair's trim amplification `(δ_coil + δ_cylinder)/sin θ ≈ 2e-3 mm`, and a face's area bound charges that times its own perimeter |

The §13 fixtures read (`examples/decad_thread_external_example_test.go`,
`examples/decad_thread_internal_example_test.go`):

| Fixture | Closed form | `Volume` | `Area` bound | `Verify` |
|---|---|---|---|---|
| external, `R = 5`, `L = 20` cylinder, groove root `4.1`, mouth `5.3` | `π·(500 − 16·Q(Ω ∩ {ρ ≤ 5}))` ≈ `1460.31 mm³` | `1460.0 ± 1.5 mm³` | `≈ 4.0 mm²` of `≈ 1000.8 mm²` | valid, `Suspect` on `Area` and `Centroid` |
| internal, annulus `5 ≤ ρ ≤ 10`, 20 mm, groove root `5.9`, mouth `4.7` | `π·(1500 − 16·Q(Ω ∩ {ρ ≥ 5}))` ≈ `4587.80 mm³` | `4587.3 ± 2.2 mm³` | `≈ 4.7 mm²` of `≈ 2665.7 mm²` | valid, `Suspect` on `Area` |

Each cut takes about 6 s, most of it the mesh boolean's exact contact
classification and stitch.

**What the station count buys.** Measured on the external fixture with the
facet-pair ceiling lifted for the measurement alone:

| Stations per turn | Tool triangles | `δ` | Tool build | Cut | `Volume` | `Area` bound | `Centroid` bound | Audit work, candidates |
|---|---|---|---|---|---|---|---|---|
| 256 | 12 290 | `9.2e-4 mm` | 0.6 s | 6.4 s | `1460.0 ± 1.45` | `4.00 mm²` | `0.025 mm` | `2.2 × 10⁶`, `1.8 × 10⁵` |
| 512 | 24 578 | `3.4e-4 mm` | 1.7 s | 11.6 s | `1460.1 ± 0.58` | `1.79 mm²` | `0.0099 mm` | `8.2 × 10⁶`, `5.8 × 10⁵` |
| 1024 | 49 154 | `1.4e-4 mm` | 5.5 s | 24.7 s | `1460.2 ± 0.36` | `1.35 mm²` | `0.0061 mm` | `3.2 × 10⁷`, `2.1 × 10⁶` |

The area tolerance is near `1.0 mm²`, so no count up to 1024 reaches
`Sound`: the rims' amplification `(δ_coil + δ_cylinder)/sin θ` keeps the
cylinder's own chord, the pair tolerance `2e-5 ×` its diameter, about
`5e-4 mm`, once the coil's `δ` falls below it. Past 256 stations per turn the
8-turn tool's crossing audit also exceeds `proofbound.MaxFacetPairTestsPerCall`
(`8 × 10⁶`) by its enumeration's work, though its candidates stay under it;
the triangle ceiling,
`maxLoftAuditTriangles = 65 528`, admits 1024. The count stays 256.

A coil built directly (no boolean) reads Table CM: `Volume` within
`1e-16` relative, `Area` and `Centroid` tight, `Bounds` within `δ`, and
`Verify` `Sound`.

## 10. Determinism, cancellation and budgets

Equal profile record, axis, pitch, turns, hand and placement produce the
same stations, held table, triangle order, roles and readings: every trig
value is a fixed-precision enclosure's midpoint, and every float product and
sum of §5.3's station evaluation is rounded by an explicit conversion, so no
step reads a transcendental library or depends on FMA contraction. The build polls `ctx`
per station ring, inside the audit's pair loop, per segment of the area
sum and per piece of §11.1's bracket, and returns `ctx.Err()` unchanged. CS8's caps bound `N` and the facet
count before any allocation; the trig enclosure runs once per station
(`N + 1` calls, each a fixed 200-bit series), every vertex costs a dozen
float operations, and the audit's work is charged against
`proofbound.MaxFacetPairTestsPerCall`. The crossing audit's exact pair
classifications take most of a build's time.

## 11. Increments

The design ships inside PR 1. Each PR's tests assert computed geometry
against closed forms, and every bound fixture is shown to fail first: delete
the leg it guards, watch the assertion go red, restore it, and record the
legs shown to fail in the test's own comment. PR 1 does not wait on any
open PR: `#1033` moves the contact-sweep motion planner into
`internal/sweeppath/`, which is the PAIR-sweep path of
`docs/contact-sweep-design.md`, not `Sweep`'s `Path`, and no coil file
touches it.

| PR | Model | Lands | Still staged |
|---|---|---|---|
| **1** | Opus (proof spec) | `Document.Coil`, `CoilOption`, `WithLeftHand`; Table CS; §5's construction over a `LineSeg`-only profile; Table CB; Table CM with `proofbound.LnInterval`/`AsinhInterval`; CD1 and CD7; the design doc, its layout row, `doc.go`'s support map, `docs/missing-features.md`; the executable example `examples/decad_coil_example_test.go` (a square-wire spring: `Volume`, `Centroid`, face count, `Verify` status). **This row is landed.** | CD2–CD5, CD9, arcs, threads |
| **2** | Opus (proof spec) | `tessellate_coil.go`: CD2 with §5.4's `β`, §8.1, §8.2; CD3, CD4, CD9 follow; the `coilPayload` row in `docs/tessellation-design.md` §2 and `docs/payload-verification-design.md` §1. The thread examples `examples/decad_thread_external_example_test.go` and `..._internal_...`. **This row is landed.** | arcs, CD5 |
| **3** | Opus (proof spec) | `ArcSeg`/`CircleSeg` profile segments: the profile station chain for an arc ruling (loft §5.1's chord chain, so a cell is chorded in both directions), `sag` folding the profile chord's own sagitta, the arc wall area by the Taylor-model bracket of §11.1, `Arc3` rim edges; the round-wire spring example `examples/decad_coil_round_wire_example_test.go`. **This row is landed**, with three choices the row left open: the chord count is a fixed `coilArcChordsPerTurn = 16` per turn of arc (§5.3), where loft §5.1 walks a count up to a sagitta target, since the crossing audit's triangle ceiling, not the sagitta, decides what a coil of several turns can hold; a whole circle's rim is a closed `Circle3`, the edge contract every builder follows for a closed circle, where §13 names `Arc3`; and a profile with an arc reads its region moments from the recorded region's bounded plane integrals (§7) | CD5 |
| **4** | Sonnet (file-by-file) | CD5: `coilPayload` in `planarPairAdmits` and the planar snapshot; the `Verify` clearance fixture against a prism. **This row is landed.** | `CoilChain`, `WithSurfaceResult()`, modify |

### 11.1 The arc wall area bracket (PR 3)

For an arc segment `ρ(φ) = c_ρ + r cos φ`, `ζ(φ) = c_ζ + r sin φ` the
integrand `sqrt(h(φ))`, `h = r²·ρ(φ)² + k²·r²·sin²φ`, has no elementary
antiderivative. Over each station sub-interval of the arc expand
`sqrt(h)` at the cell's `h_m` to second order: the linear and quadratic
terms integrate exactly (`h` and `h²` are trig polynomials, their
integrals certified sines and cosines at the ends), and the remainder
`|h − h_m|³ / (16·h_lo^{5/2})` is bounded by the cell's own `h` range.

`coil.ArcArea` reads it in the arc's own angle `α` about its centre, with
`A = e_r·(cos α, sin α)` and `B = A'`, so `ρ = ρ_c + r·A` and
`h = a₀ + a₁·A + a₂·A² + a₃·B²` with `a₀ = r²ρ_c²`, `a₁ = 2r³ρ_c`,
`a₂ = r⁴` and `a₃ = k²r²`. It cuts the arc into `coil.ArcAreaPieces = 1024`
equal pieces and reads `A` and `B` at every piece's two ends and midpoint:
a whole circle at exact turn fractions through `TurnSinCos`, an arc by
turning its `Start` direction step by step through one enclosed sine and
cosine of the step. Over a piece of width `Δ`, `h_m = g²` with `g` the
float nearest `√h` at the midpoint, so `√h_m` is exact; `∫A`, `∫A²`, `∫B²`,
`∫A³`, `∫A·B²`, `∫A⁴`, `∫B⁴` and `∫A²·B²` are closed forms in `Δ` and the
ends' `A` and `B` (`A = cos φ`, `B = −sin φ` up to a phase), so `∫h` and
`∫h²` are exact intervals. `|h'| = |a₁·B + (a₂ − a₃)·2A·B| ≤ |a₁| + |a₂ − a₃|`
bounds the drift from the midpoint, which gives both `|h − g²|` and
`h_lo` over the piece. Every per-piece enclosure is rounded outward onto a
`2⁻¹⁶⁰` grid. On §13's round wire the bracket's relative width is near
`1e-10`; the Taylor remainder is odd about each midpoint, so the true error
cancels far below the bound, and over four pieces the bound is what holds
the bracket around the integral.

### 11.2 Files

| File | Owns |
|---|---|
| `coil.go` | `Coil`, `CoilOption`, `WithLeftHand`, option validation, Table CS's gates in §4's order, `coilPayload` and its placement |
| `internal/coilshell/` | §5 and §8: stations, `β`, triangles, crossing audit and mesh proofs |
| `coil_body.go` | Table CB's topology and Table CM's four readings |
| `tessellate_coil.go` | CD2 (PR 2) |
| `internal/coil/` | station fractions and trig, the profile station chain (§5.3), the lift to axis coordinates, the segment area closed form, §11.1's arc bracket, the cell departure terms, §8.1 and §8.2's sums — every function pure over `big.Rat`/`RatInterval` inputs and unit-tested against hand values |
| `internal/proofbound/log.go` | `LnInterval`, `AsinhInterval` (§7.1) |

Every root file gets a `docs/layout.md` row in the PR that adds it; the
design row lands with this document.

## 12. Do not do this

- **Add a `Helix` segment to `Path`.** Every Table P rule of
  `docs/sweep-design.md` fails for it: its end is a derived transcendental
  point, so P4's "the next segment's start is a recorded coordinate" and
  §3.1's "`Path.End()` returns a recorded coordinate" cannot hold; its
  section is not normal to the path (CP3), so P6 and P7 do not describe it;
  and the path's start point is REDUNDANT — any point of the sketch plane
  off the axis names the same solid — which makes it a parameter that
  changes nothing and a refusal (`S5`) that guards nothing. A verb beside
  `Revolve`, taking `Revolve`'s own axis, states exactly the inputs the
  solid depends on.
- **Carry the section in a Frenet or rotation-minimizing frame.** It does
  not cut a thread (§3), and its volume has no closed form unless the
  section's centroid rides the helix.
- **Reuse `r3.Screw.At` or `internal/sweeppath`'s screw pose as the
  geometry.** Those are float poses for the pair-sweep motion checker; a
  held station is an interval-enclosed point with a published rounding
  gap, never a float pose.
- **Prove simplicity by the facet audit.** CP5 is the proof; the audit
  proves the HELD shell embedded. A held crossing on a solid CP5 admits is
  CS9's `ErrUnsupported`, never `ErrDegenerate`.
- **Read `Volume` or `Area` off the held triangles.** The tetrahedron sum
  carries `δ·A`; the closed forms carry `1e-16`. The held table serves
  `Bounds`, topology and the mesh alone.
- **Refine the station count on an audit failure.** One count per turn
  count, decided before allocation; a loop that doubles until the audit
  passes makes the body's facet count depend on a predicate's outcome and
  the build's cost unbounded.
- **Admit a wide profile over one turn by a per-fibre test.** CS6's extent
  rule is sufficient and reject-only; a fibre test that ADMITS is an
  admission gate on decad's own arithmetic, which `CLAUDE.md` forbids.
- **Let a zero pitch reduce to `Revolve`.** Two spellings of one solid; CS4
  refuses and names the other verb.
- **Publish the wall as a `NURBSSurface`.** A helix is not rational; the
  wall would be a surface the variant's own contract says it is not.

## 13. Required tests

Every row asserts computed geometry or a proof bound; each bound leg is
deleted once and watched fail. Fixtures assert against the production
path, never a local model.

PR 1:

- A `1 × 1` square section at `ρ ∈ [2, 3]`, `ζ ∈ [0, 1]`, about the
  sketch's `V` axis, pitch `1.5`, `2` turns: `Volume` encloses `10π` within
  its bound, and the bound is below `1e-12`; `Centroid` is `Exact` at
  `(0, 0, 2)` in the sketch frame's axis coordinates (`M/Q = 0.5`,
  `pitch·turns/2 = 1.5`); the two cylindrical bands read `8π` and `12π`
  within bound; the two flat annular walls read
  `4π·∫₂³ sqrt(ρ² + k²) dρ` within bound against an independent
  high-precision quadrature; `Bounds` encloses dense samples of `Φ` over
  the solid and exceeds them by at most `δ`; the body has `6` faces, `12`
  edges, `8` vertices; every edge has two incident faces. Legs shown to
  fail: the π enclosure, the asinh term, the `δ` widening.
- The same section with `2.5` turns: `Centroid` is `Approximate`, its
  transverse part equals `(I/Q)·(sin Θ/Θ, (1 − cos Θ)/Θ)` with
  `sin Θ = 0`, `cos Θ = −1` exactly; `Volume` is `12.5π`.
- `WithLeftHand()`: the same readings with the `e_t` term negated; every
  held vertex is the right-hand vertex mirrored in the sketch plane.
- A tilted `ConstructionAxis` in the plane: `Volume` and `Area` agree with
  the axis-aligned case within both bounds; `Centroid` is `Approximate`.
- A hole loop sweeps to a void passage: `Volume` subtracts the hole's
  `Θ·Q` exactly, and the body has one lump and one outer shell.
- Each Table CS row refuses with its sentinel before any commit: live set,
  order and next producer identity unchanged. CS5 at a profile touching
  the axis (`ErrDegenerate`) and at a straddling tilted axis
  (`ErrUnsupported`); CS6 at `H = pitch` with `turns = 1` (refuses) and
  with `turns = 0.75` (builds); CS8 past `maxCoilStations`.
- Repeated construction and `Placed` under a rotation reproduce
  bit-identical held tables, roles and readings; `Placed` keeps `Volume`
  and `Area` within both bodies' bounds and maps `Centroid` within its
  bound. Under that rotation, `Volume`'s exact enclosure holds
  `det L · 12.5π` and the start cap's `Area` holds `|L·U × L·V|`, each read
  off the rotation's own floats as exact rationals. Legs shown to fail: the
  `|det L|` factor, the cap area's defect widening.
- §5.4's `β`: dense samples of every cell's true wall lie within the cell's
  facet bound of its two held triangles, on the square spring, a profile
  `ρ ∈ [2, 4]` (which verifies `Sound`), a profile `ρ ∈ [0.5, 3]`
  (`c < 1`) and a profile whose middle band touches no annular wall; and
  `coil.CellDepartureUpper` encloses the shifted correspondence's own
  displacement, sampled over a cell of each segment kind. Legs shown to
  fail: twist, sag, and twist's `(1 − c)` part.
- `proofbound.LnInterval`/`AsinhInterval` as §7.1 states.
- `Verify` on the spring: `Validity` proven, every reading within the
  default tolerance, `Status` `Sound`; the gate diameter equals the held
  diameter less `2δ`.

PR 2:

- The spring's mesh: every vertex's `vertexBound` equals its `β`; `Bound`
  equals `δ`; a dense sample of the true surface lies within each facet's
  bound of that facet (falsifier only, never the proof); the STL and 3MF
  round trips are watertight; a tolerance below `δ` is `ErrUnsupported`.
- The held mesh's signed volume lies within `volSymDiff` of `Θ·Q`, and its
  area within `areaSlack` of `Area`, on the spring, the profile
  `ρ ∈ [2, 4]` and the near-axis profile `ρ ∈ [0.5, 3]` (falsifiers). Legs
  shown to fail: §8.2's wall homotopy.
- `Verify` measures the overlap of the spring with a 60 mm coaxial core of
  radius `2.5` as `4.5π`, and their `Union` encloses `380.5π`.
- The external thread of §9 on a `Revolve` cylinder (`R = 5`, `L = 20`,
  a `60°` V of depth `0.9` at pitch `1.5`, `8` turns inside the height):
  the result is one lump of one shell, `Volume` encloses
  `π R² L − Θ·Q(clipped)` with `Q(clipped)` the exact polygon moment, the
  box holds the true extremes within its bound; `Verify` proves validity and
  reads `Suspect` on `Area` and `Centroid` alone; the faceted mesh is
  watertight. Leg shown to fail: without the coil's arm in the pair
  tolerance, `Cut` refuses at the chain-depth gate.
- The internal thread on an annular `Extrude`: the same enclosure with
  `Ω ∩ {ρ ≥ R}`, `Suspect` on `Area` alone.
- The cylinder meshed at the raised pair tolerance is coarser than at the
  diameter-derived one, and its mesh volume and area still lie within its
  own `volSymDiff` and `areaSlack` of `500π` and `250π`.
- Starting the external groove `pitch/4` below the cylinder's end plane
  builds, and the cut's `Volume` encloses `π·(500 − 6Q − (4/3)·M⁻)` at
  three turns. A coil cap flush with the start cap of a cylinder revolved
  through half a turn, in either sense, refuses with the boolean's
  hidden-tangency code.
- The union of a coil with a coaxial cylinder through its core (a
  threaded rod built by union rather than cut) encloses
  `V_cylinder + Θ·Q(Ω ∩ {ρ ≥ R})`.
- `MassProperties` of the spring settles at the ladder's first step and
  its mass encloses `density·10π`.

PR 3:

- A round-wire spring (wire radius `0.5` at `ρ = 3`, pitch `1.5`, `5`
  turns): `Volume` encloses `2π·5·3·π·0.25`; `Area` encloses an
  independent quadrature of the wall within a bound below `1e-9` relative;
  `Bounds` within `δ`; the two rim edges are closed `Circle3` edges, and a
  slot's arc rims are `Arc3`, each running counter-clockwise from its start
  vertex to its end vertex.
- CS6 at a wire diameter equal to the pitch refuses.
- §11.1's bracket encloses independent quadratures of a whole circle, a
  semicircle and an arc walked in reverse, below `1e-9` relative; over four
  pieces only the remainder holds it. Leg shown to fail: the remainder.
- `β` over arc chords: dense samples of every chord cell's true wall lie
  within the cell's facet bound of its two held triangles. Leg shown to
  fail: `arcSag`.
- The round wire's mesh: signed volume within `volSymDiff` of `Θ·Q` and
  area within `areaSlack` of `Area`. Legs shown to fail: the arc legs of
  `CellProof.Swept` and `CellProof.Density`.

PR 4:

- `Verify` `WithClearances` on a coil beside a prism reads a gap within
  `δ` of the hand-computed value; beside a revolve it reads `Suspect`.

## 14. Companion edits

Landing this design makes these contract edits, each in the PR that lands
the behaviour:

- `docs/api-design.md` §8 adds `Coil` and points here for its signature,
  axis and extent contract; §13's non-goals need no change;
- `docs/layout.md` lists this document (with the design) and every file of
  §11.2 (with PR 1 and PR 2);
- `docs/missing-features.md`: PR 1 narrows "Helical or free-form sweep
  path" to the free-form path alone, narrows "Hole, thread, rib, web,
  emboss, coil features" to drop thread and coil, and adds rows for CD2's
  staging and CS7's arc refusal; PR 2 deletes the first, and PR 3 narrows
  the second to the free-form, elliptical and trimmed segments CS7 still
  refuses;
- `doc.go`'s support map adds `Coil` with CS5–CS9's refusals;
- `docs/evaluator-design.md` §11 points to this document's staged delivery;
- `docs/tessellation-design.md` §2 and `docs/payload-verification-design.md`
  §1 add the `coilPayload` row with PR 2;
- `docs/sweep-design.md` §1 names this document as the owner of the helical
  case it does not admit.

No 2D answer this design needs is one `sketch` cannot give today: the
profile's axis-frame extremes and moments are read off decad's own recorded
walk exactly as `Revolve` reads them, and no hand-off to `sketch`, `r3` or
`units` is required.
