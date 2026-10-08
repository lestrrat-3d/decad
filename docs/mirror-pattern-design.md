# Mirror and Pattern Design

Mirroring a body across a plane, joining a body with its own mirror image, and
repeating a body in a linear or circular pattern. Companion to
`docs/api-design.md` (core §N below), `docs/evaluator-design.md` §8 (placement
and copies), `docs/prism-boolean-design.md` (the analytic boolean a pattern
tool feeds), `docs/stacked-prism-design.md` (the payload a prism pattern
builds) and `docs/general-boolean-design.md` (the boolean classes a mirror or
pattern result needs next). Every number in §2 was measured with the probe
under `.tmp/probe/` at `8cc06f90`.

## 1. Problem

A reflection is already a placement. `r3.Transform` represents an improper
isometry (det −1, `Transform.IsReflection`), `Body.Placed` and
`Body.PlacedCopy` accept one, and every shipped payload re-evaluates under it:
`prismPayload`, `revolvePayload`, `stackedPrismPayload`, `capBlendPayload`,
`cupPayload` and the chain ribbon flip their outward normals, windings and
arc senses through `reflected()`; `loftPayload` re-decides its whole-shell
orientation from the placed triangles; `facetedPayload` reverses every
triangle and re-embeds the held mesh (`boolean_body.go`). §2 measures that
this is sound: volume, centroid, convexity, `Facing`, the mesh winding, the
analytic STEP writer and `Verify` all read a mirrored body correctly.

What is missing is everything a caller does WITH a mirror image:

- there is no vocabulary for the plane. A caller builds an `r3.Frame` by hand,
  where the natural statement is "this planar face of the part";
- the common symmetric-part workflow — model one half ending on the mirror
  plane, mirror, join — refuses at the join. The two halves are co-directional
  prisms sharing a wall, and the analytic boolean excludes a reflected operand
  (prism-boolean §3.1 G2), so the pair takes the mesh path and refuses on the
  coplanar contact. With G2 lifted and the shared wall resolved
  (general-boolean §3 A3) the same pair still takes the mesh path: the image
  carries the placement's rounding, and a displaced pair's wall inside the
  union is a sliver no recorded edge bounds;
- a pattern built from `PlacedCopy` and `Union` ends in one of three places:
  disjoint instances become a `Faceted` multi-lump body whose surveys read
  `Suspect` and which no later coplanar boolean accepts as a tool; instances
  placed by an in-plane translation combine analytically but carry a section
  displacement, so `Fillet` refuses the result and the wall survey is
  undecided; instances placed by `RotationAround` fall to the mesh path and
  refuse on the coplanar contact.

The design keeps the reflection where it is — in the payload's accumulated
placement — and adds three things: a plane vocabulary over the existing copy
path, an exact mirror-join for prism and stacked receivers that needs no
boolean at all, and a pattern operation whose instances share one frame and
one record set, so a pattern of N holes is one analytic arrangement.

## 2. Measured current behaviour

| Row | Scene | Result |
|---|---|---|
| M1 | L prism (1750 mm³) `PlacedCopy` across the plane x = 30 | analytic, `Exact` 1750 mm³, centroid x = 53.2143 (= 60 − 6.7857), bounds `Exact`, `Facing(+x)` one face, 17 of 18 edges convex on both bodies, mesh signed volume +1750, `Verify` `Sound`, wall measured, `Fillet` OK |
| M1 | STEP of the mirrored L | analytic writer path, 7617 bytes |
| M2 | `Union(L, mirror across the wall x = 20)` (touching along the wall) | `BooleanUnsupportedContact`: coplanar facets |
| M3 | `Union(L, mirror across x = 15)` (overlapping) | `BooleanUnsupportedContact`: coplanar facets |
| M4 | `Union(mirrored L, same-plane box)` | `BooleanUnsupportedContact`: coplanar facets |
| M5 | `Cut(mirrored L, same-plane cylinder)` | mesh path: 8 `Faceted` faces, bound 0.05 mm³ |
| M6 | `Placed(reflection)` | analytic, receiver retired, one live body |
| T1 | `Cut(mirrored L, mirrored cylinder)` (both reflected, proper relative map) | `BooleanUnsupportedContact`: G2 refuses either reflected operand |
| T2 | `Verify` on L beside an overlapping mirror | `Suspect`, `unsupported_pair_contact` |
| T3 | `Verify` on L beside a separated mirror | `Sound`, clearance 20 mm |
| M7 | partial revolve (270°) mirrored | same volume and bound, mesh winding outward, `Verify` `Sound` |
| M8 | loft mirrored | volume 1100 `Approximate` (bound 2.6e-10; the unmirrored loft is `Exact`), mesh outward |
| M9 | cross-drilled (faceted) body mirrored | same volume and bound, mesh outward |
| M10 | blind-cut (stacked) body mirrored | same volume, mesh outward |
| M11 | filleted prism mirrored | same volume, `Verify` `Sound`, wall measured |
| M12 | cup mirrored | `Exact` 1952 mm³, mesh outward |
| S5 | plate cut by six `PlacedCopy`-translated cylinders | six analytic cuts, bound 5e-10 mm³, `Fillet` refuses (section displacement), wall `undecided` |
| S5b | the same six holes each drawn in its own sketch | bound 2e-12 mm³, `Fillet` OK |
| S5c | six cylinders `PlacedCopy`-rotated about the plate normal (60° steps) | six analytic cuts (every rotation here keeps the normal exact), bound 5e-10 mm³ |
| P1 | hub ∪ six teeth placed by `RotationAround` (60° steps) | tooth 1 analytic; tooth 2 `BooleanUnsupportedContact` (§3.4 reroute, then coplanar facets) |
| P2 | four disjoint pegs by `PlacedCopy` + `Union` | `Faceted`, 4 lumps, `Exact` 500 mm³, wall `unavailable`; as a `Cut` tool: `BooleanUnsupportedContact` |
| W1–W4 | `Union` of two boxes sharing a wall (whole, partial, longer, 1 mm overlap with collinear walls) | `ErrUnsupported`: "the union scene's arrangement reports an invalid region" (RB1) |
| W5 | the same with no collinear walls | analytic, 800 mm³ |
| W6 | `Cut` by a box sharing the wall | `BooleanUnsupportedContact` |

## 3. How a reflection is carried

**Decision: the reflection stays in `xform`.** No payload normalises it into
its record or frame. An `r3.Frame` is right-handed by contract, so an improper
change of plane-local coordinates has no frame to land in; absorbing the
reflection as a world reflection composed onto `xform` would make the stored
placement a float composition the caller never asked for, orthonormalised by
`FromBasis`. The carried form is exact: every consumer reads one bit,
`reflected()`, and flips what it must.

| Consumer | Reading of a reflected body |
|---|---|
| Face outward normals, `Facing`, `NormalTo` | `prism_build.go` / `revolve_build.go` swap the sense; measured M1 |
| Loop winding, `Edge.IsConvex` | the walked-boundary convexity is a property of the record, which the reflection does not change; the built loops are re-wound (`prism_build.go`); measured M1 (17 of 18 convex on both) |
| Mass properties | magnitudes unchanged, centroid mapped through `xform`; measured M1, M7, M9–M12 |
| `Bounds` | extremes mapped and rounded as for any placement (evaluator §5) |
| `FeatureRef`, `Origins`, `CapStart`/`CapEnd` | the copy rule (core §8): roles re-minted under the copy's identity, faceted origins verbatim |
| Tessellation | winding flipped per payload; mesh outward (signed volume positive) on every payload measured |
| STEP | analytic writer follows the outward normal and reverses opposing loop walks (`docs/step-export-design.md`); measured M1 |
| STL/OBJ/3MF | read the mesh |
| `Verify` validity, surveys, tolerance gate | per payload, unchanged; measured `Sound` on prism, revolve, fillet result |
| Clearance kernel | `clearance_geom.go` reads `reflected()`; measured T3 |
| Interference | a reflected prism beside a coplanar prism enters the analytic Intersect and overlap readings (`docs/general-boolean-design.md` class A4); a pair whose outlines cross under a one-sided reflection is charged its crossings (general-boolean A6); T2's shared collinear walls are read as one carrier (general-boolean A3), and its overlap is measured: `Interfering`, 500 mm³ |
| `Union`/`Cut`/`Intersect` | a reflected operand enters the analytic path with its record re-wound (general-boolean class A4: M5, T1); M3's shared walls bound its union and build analytically (A3); M2's shared wall lies inside its union and the image is displaced, so `Union` takes the mesh path's coplanar refusal, and the mirror join builds it (§5) |
| `Trim`/`Extend`/`Split` | refuse a reflected operand (`surface_trim.go`). Unchanged by this design |
| `VerifyMotion`, contact sweeps | a reflection joined to a proper motion is `ErrDegenerate` (`motion.go`): no rigid path exists. `contact_sphere.go` and `contact_sweep_replay.go` refuse a reflected pose. Both unchanged |
| Loft | `loftPayload.placed` re-lifts every vertex under the full composed transform (M8's bound is the re-lift rounding, the same term any placed loft carries) |

**`r3` needs nothing new.** `r3.Reflection(mirror Frame)` and
`Transform.IsReflection` are the whole requirement. No hand-off is filed.

## 4. Public API

### 4.1 The mirror plane

```go
// MirrorPlane is what a body may be mirrored across. Sealed.
type MirrorPlane interface{ mirrorPlane() }

// MirrorFrame mirrors across the plane through Frame's origin spanned by its
// U and V axes — exactly what r3.Reflection takes.
type MirrorFrame struct{ Frame r3.Frame }

// MirrorFace mirrors across a planar face of Body, selected never pointed at
// (core §9). Body is what Face resolves against; it MUST be a live body of the
// same document, and the receiver itself is the ordinary choice.
type MirrorFace struct{ Body *Body; Face FaceSelector }
```

`MirrorFace.Face` resolves through `SelectFaces(Body)` under the implicit
exactly-one rule of core §9: zero or several faces is `ErrCardinality` with
`Expected "exactly 1"`, a non-planar face is `ErrDegenerate`, a flat face
with no analytic `Plane` surface (a mesh-boolean `Faceted` face) is
`ErrUnsupported`, another document's body `ErrForeignBody`, a retired one
`ErrRetiredBody`. With
`WithJoin` (§4.2) the selector may name several coplanar faces, and the
cardinality rule is `AtLeast(1)` with every face proven coplanar (§5.1).
Resolving the plane does not consume `Body`. The plane is the face's own
`Plane.Frame`, so a `MirrorFace` over a face of the receiver reflects
across a plane the body's own record states.

### 4.2 Mirror

```go
func (b *Body) Mirrored(ctx context.Context, plane MirrorPlane, opts ...MirrorOption) (*Body, error)
func (b *Body) MirroredCopy(ctx context.Context, plane MirrorPlane) (*Body, error)

type MirrorOption interface{ option.Interface; mirrorOption() }
func WithJoin() MirrorOption
```

`Mirrored` is `Placed` under `r3.Reflection(frame)`: it retires the receiver
and registers the image. `MirroredCopy` is `PlacedCopy` under the same
transform: the receiver stays live. Both take the gates `Placed` takes — a
nil context, a body no evaluator built, a cancelled context before commit —
and add only the plane's own resolution. A `MirrorFrame` whose frame is the
zero value is `ErrDegenerate`.

`WithJoin` makes `Mirrored` return the UNION of the receiver and its image as
one body. It is the symmetric-part operation, and §5 owns it: the join is an
exact rewrite of the receiver's own record, never a boolean, so it is admitted
on a narrower class than `Mirrored` alone and refuses the rest with
`ErrUnsupported` rather than falling back to `Union`. `MirroredCopy` takes no
options: a join consumes its receiver by definition, and a caller wanting the
half kept duplicates it first.

### 4.3 Pattern

```go
// PatternSpec is how instances are laid out. Sealed.
type PatternSpec interface{ patternSpec() }

// LinearPattern places instance i at i·Step along Dir, i = 0..Count-1.
// Instance 0 is the receiver itself. Dir is a direction (dimensionless,
// non-zero, not normalised by the caller); Step is a length magnitude.
type LinearPattern struct{ Dir r3.Vec; Step units.Value; Count int }

// CircularPattern places instance i rotated by i·(one turn / Count) about the
// axis through Center along Axis, i = 0..Count-1. The step angle is denoted by
// Count, never by a float: a quarter turn is exactly a quarter turn.
type CircularPattern struct{ Center, Axis r3.Vec; Count int }

func (b *Body) PatternCopies(ctx context.Context, spec PatternSpec) ([]*Body, error)
func (b *Body) Patterned(ctx context.Context, spec PatternSpec) (*Body, error)
```

`PatternCopies` leaves the receiver live and returns `Count − 1` new live
bodies, instances `1..Count−1` in order. `Patterned` retires the receiver and
returns ONE body holding every instance (§6). `Count < 2` is `ErrDegenerate`
(a pattern of one is the receiver), a non-finite or zero `Dir`/`Axis` is
`ErrNotFinite`/`ErrDegenerate` (a non-finite `Center` too is `ErrNotFinite`),
a wrong-kind `Step` is `ErrUnitKind`, a negative one `ErrNegativeMagnitude`
(core §12: sense is `Dir`'s) and a zero one `ErrDegenerate` (every instance
would sit on the receiver). Every instance is built before any is registered,
so a refusal or a cancelled context leaves the document unchanged. A
`CircularPattern` whose `Center` does not lie on the receiver's own sweep axis
is admitted: the instance is rotated about the stated axis, wherever it is.

**An instance of a co-directional prism keeps the receiver's frame.** Where
the receiver is a `prismPayload` or `stackedPrismPayload` whose every
segment is a `LineSeg`, `ArcSeg` or `CircleSeg` with no trimmed arc or circle,
and the motion keeps it co-directional — the exact rational `Dir · N` zero
for a linear pattern, or `Axis` bit-identical to `±N` for a circular one,
both read on the composed world normal `xform.ApplyDir(frame.N())` as
prism-boolean G3 reads it — an
instance is NOT a `PlacedCopy`. It is the same frame and placement over a
TRANSLATED or ROTATED copy of the record (§6.2), with the rounding of that
record motion charged into the instance's `sectionDelta`. That is what keeps
a later analytic boolean's re-expression the identity (prism-boolean §3.4),
and what makes a pattern of integer-millimetre holes `Exact`. Every other
receiver, and every non-co-directional motion, instances through
`PlacedCopy` under the composed rigid motion, which is what a caller's own
loop would do.

## 5. The mirror join

### 5.1 Admission

Checked in order; a miss is a refusal with the stated sentinel, never a
fallback to `Union`:

| # | Condition | Sentinel |
|---|---|---|
| J1 | The receiver's payload is `prismPayload` or `stackedPrismPayload` whose slabs share one outer loop (a union-built stack does not) and hold one region each (a prism group does not), and the receiver is a solid. | `ErrUnsupported` (a revolve, loft, sweep, cup, cap blend or faceted receiver joins through `MirroredCopy` + `Union`, with that boolean's own reach; a sheet has no union to build) |
| J2 | The plane is a `MirrorFace` of the receiver itself, its selector resolves to at least one face, and every selected face is a planar WALL of the receiver — a face whose role is `side(i, j)` (or `slab(k).region(0).side(i, j)`) over a `LineSeg`. | `ErrUnsupported` for a `MirrorFrame` or another body's face (the join needs the mirror line as a recorded carrier, §5.2); `ErrCardinality` with `Expected "at least 1"` for no face; `ErrDegenerate` for a cap or a curved wall |
| J3 | Every selected wall's segment lies on ONE line: the exact rational cross product of each segment's recorded endpoints against the first's is zero (`internal/proof`). | `ErrDegenerate` |
| J4 | Every selected segment is WHOLE (`TStart`/`TEnd` the natural domain). | `ErrUnsupported` (a fragment of a longer carrier would mirror a wall the record does not state whole) |
| J5 | Every selected wall walks the first one's way, every region's outer loop holds a selected wall, and every other segment of every loop is a `LineSeg`, `ArcSeg` or `CircleSeg` in the closed half-plane on the material side of the line: both recorded endpoints (and a narrowed line's walked endpoints) have a non-negative exact signed distance, and an `ArcSeg`/`CircleSeg` additionally has its circle on that side — exact rational comparison of the centre's signed distance against the radius squared, an arc's radius the larger of its two recorded ones. A segment with an endpoint ON the line that is not a selected wall is admitted only as the neighbour of a selected wall at that junction. | `ErrUnsupported` (a loop crossing the line would need an arrangement, which §5.2 never asks for) |
| J6 | No `ArcSeg`/`CircleSeg` of the receiver is recorded over a narrowed range (`prismProfileHasTrimmedCircularSource`, prism-boolean §4.1). | `ErrUnsupported` |
| J7 | `sectionDelta == 0` on the receiver. | `ErrUnsupported` (a reflection amplifies nothing, but the §5.3 audit proves closure on recorded coordinates and a displaced record states none) |

J5 is exact arithmetic over the record's own floats and decides, so it is a
decision rather than a residual: a loop is on one side or it is not.

### 5.2 Assembly

The join is a rewrite of the receiver's `ProfileRecord` in the receiver's own
frame, with the receiver's own `xform`, interval and axial displacements
unchanged. The image needs no transform: reflecting the record across the
line IS the mirrored geometry, and the mirror plane is perpendicular to the
sketch plane by J2, so the sweep is unchanged.

**This is a decad-side 2D answer, and CLAUDE.md requires the reason.** The
join computes no crossing, no containment, no cut parameter and no region
membership: it reflects decad's own recorded coordinates in exact rational
arithmetic and removes recorded segments by exact identity. `sketch` can
arrange the two halves (general-boolean §3 A3), but only after the image is
placed, and the placement's rounding displaces it: the shared wall then lies
inside the union as a displaced span, which A3 sends to the mesh path. The
join wins on exactness, since its result carries no displacement at all.
The assembly is then audited exactly as every modify op's rewritten section
is (modify §5), which is the same carve-out `docs/prism-boolean-design.md`
§5 claim 2 already uses.

Let `m` be the exact reflection across the line through the selected
segment's recorded `Start` and `End`. For a point `X`, `m(X) = X − 2·d·n`
with `d` the signed distance along the line's normal `n`, a rational function
of the three points: no square root, since `n` need not be unit length when
`d·n` is written as `((X − P)·n⊥)/(n⊥·n⊥) · n⊥`. Each reflected coordinate
is computed over `big.Rat` and rounded to the nearest float ONCE. A point's
charge is its two coordinates' `rationalFloatError` summed (the one that
rounded, when only one did), which bounds the held image's distance from the
exact one. A segment's charge is its largest point charge, tripled for an arc
(its centre and radius both move with its three points' rounding, the
argument `offsetSectionDelta` states for a recorded arc), plus a narrowed
line's `δ_walk`. `δ_mirror` is the largest segment charge. For a wall on an
axis-aligned line through integer millimetre coordinates every reflected
coordinate is a float, and `δ_mirror == 0`.

A loop's segments are walked in record order. Runs of consecutive segments
between selected (on-line) segments are the material's own boundary arcs; each
run `R` closes with its own reflection: `R` followed by `m(R)` walked
backwards, each segment mirrored. That is the per-loop re-winding rule a
reflected boolean operand takes (`docs/general-boolean-design.md` §3 A4,
`rewindLoop`), run under the exact reflection:

| Segment | Mirrored, reversed |
|---|---|
| `LineSeg{Start, End}` | `LineSeg{m(End), m(Start)}`, whole |
| `ArcSeg{Center, Start, End}` (CCW from Start to End) | `ArcSeg{m(Center), m(End), m(Start)}`: the reflection turns the arc clockwise, and walking it backwards turns it counter-clockwise again from the new Start |
| `CircleSeg` | `CircleSeg{m(Center), Radius, CCW}` over the same range: the reflection reverses the circle's winding and the reversed walk reverses it back, so the flag and the range order still agree. A hole's own loop, never on the line (J5) |
| a `LineSeg` recorded over a narrowed range | its walked endpoints (`walkOf`, prism-boolean §7 `δ_walk`) stand in as a whole segment before mirroring; the charge joins `δ_mirror` |

Every closed run walks with the material on its left by construction, since
the receiver's walk did and reflection-plus-reversal preserves that side. A
loop with one selected wall yields one closed loop. A loop with `k ≥ 2`
selected walls (a U whose two tips both end on the line) yields `k` closed
loops, and the one whose signed area is the largest positive is the outer;
the rest are holes (the notch and its image close into one). The pick reads
the float signed area; §5.3's S8 then refuses a hole that winds as an outer
and S9 a hole outside the outer, so a wrong pick cannot be built.
A loop with no selected wall (every hole, by J5) is mirrored whole and
reversed, and appended as one more hole beside its original. The result's
`Outer` is the one outer; `Holes` is every other loop.

For a `stackedPrismPayload` receiver, a selected wall marks its segment in
every slab whose region holds an equal loop (its face spans that column),
J5–J6 run over every slab's region (an exposed record is a hole's own loop
reversed, so it is covered), and the rewrite runs per region. The interfaces'
exposed records are re-derived from the rewritten regions, and the stacked
audit (`stackedrecord.Falsify`, stacked §2.2) re-checks them.

### 5.3 Audit and exactness

The assembled record passes modify §5's audit verbatim with an empty blend
map, in prism-boolean §6's order: the junction falsifier (seam §3) on every
junction — two recorded endpoints compare exactly, and a narrowed line's
walked endpoint with the range falsifier's tolerance — S8 (the outer's signed
area positive, every hole's negative), S7 (no non-adjacent
pair crosses or contacts within the diameter-anchored floor), S9 (every hole
provably inside the outer). The sentinels are prism-boolean §9's RB3–RB6 and
RB9. Every check is reject-only.

The result carries `sectionDelta = δ_mirror` and the receiver's own axial
displacements. `Exactness` follows prism-boolean §7: `Exact` only when every
surviving segment is a `LineSeg` and `δ_mirror == 0`. Roles are fresh under
the join's own producer identity (prism-boolean §11): `capStart`/`capEnd`
stand, and a wall's `side(i, j)` indexes the assembled record.

Downstream, the result is an ordinary `prismPayload` or `stackedPrismPayload`:
`Fillet`/`Chamfer`/`Shell`, the three surveys, clearance, tessellation and
export dispatch on payload class and need no new code where
`δ_mirror == 0`; where `δ_mirror > 0`, prism-boolean §12's rows apply.

## 6. The pattern body

### 6.1 Combination rule

`Patterned` returns one body. Its instances are combined by the first rule
that applies:

1. **Proven pairwise disjoint, co-directional prism receiver** (§4.3's
   frame-keeping arm): the result is a one-slab `stackedPrismPayload` whose
   slab holds `Count` regions, one per instance, on the receiver's frame and
   interval — the prism group of §6.3. A prism group receiver patterns every
   region at once. No boolean runs, and the result takes one producer
   identity.
2. **Proven pairwise disjoint, any other receiver**: `ErrUnsupported` in this
   design. A multi-lump body of revolves, lofts, faceted lumps or multi-slab
   stacks needs a group payload no consumer reads today; the caller uses
   `PatternCopies` and keeps the instances as separate bodies, which `Verify`
   reports on individually. The proof is the §6.3 scene, so the rule reaches
   a cut-built stack on the frame-keeping arm, over its one outer loop; a
   union-built stack, whose outer changes between slabs, and every receiver
   off that arm, are not proven and take rule 3.
3. **Not proven disjoint**: the instances are combined by `Union` in index
   order — `Union(Union(b0, b1), b2)` … — each with that boolean's own gates
   and refusals, unchanged. A refusal propagates as that `Union`'s error, and
   the document is unchanged: every instance is built and combined before the
   receiver is retired and the result registered. The result's producer
   identity is the last `Union`'s, and the document reserves one identity for
   every instance and every intermediate result, so no later body repeats
   one a faceted result's provenance can carry. Disjoint co-directional
   instances that reach this rule still combine into a group, through
   general-boolean class A5's disjoint `Union`.

Rule 3 is where touching instances land (a pattern of pegs sharing walls),
which `Union` joins through general-boolean class A3. The rule is stated here
so a caller knows the result is `Union`'s, never a silently merged one.

### 6.2 Instance records

For the frame-keeping arm, instance `i`'s record is the receiver's record
under an in-plane motion of the section:

- **Linear.** The offset is `t_i = (i · step / |Dir|) · Dir` in world
  millimetres, read on the placed frame axes as the exact rationals
  `(t_i · U_w, t_i · V_w)`, with `U_w`, `V_w` the frame's axes under the
  placement's basis, taken over the held floats. `step` is the exact rational
  the caller's `Step` denotes in millimetres (`exactConversion`, the value
  `conversionRound` compares against), and `1/|Dir|` is a certified rational
  enclosure (`RatSqrtDown`/`RatSqrtUp` over the exact `Dir · Dir`),
  zero-width when `Dir · Dir` is a float's square — every stored axis is.
  Each moved coordinate `u + du_i`, `v + dv_i` is therefore an exact
  interval; the float nearest its midpoint is held, and its
  `IntervalFloatError` covers the enclosure's width, the conversion and the
  sum's rounding at once. A `Dir` along a frame axis, an integer-millimetre
  `step` and integer-millimetre coordinates make every term exactly zero.
- **Circular about `±N`.** `Center` is read on the placed frame axes as the
  exact rationals `(c_u, c_v)`, and the rotation is applied to exact
  rationals, so the centre enters no rounding of its own. The rotation of
  instance `i` is `i/Count` of a turn, and `cos`/`sin` of it are certified
  enclosures from `proofbound.TurnSinCosInterval` (sin and cos of `2πt` for a
  rational turn `t`), so each rotated coordinate `c + R(θ_i)(p − c)` is an
  interval whose midpoint is held and whose `IntervalFloatError` is charged.
  The half turn and the quarter turns (`t` reduced to `1/2`, `1/4`, `3/4`)
  are the exact maps `(u, v) ↦ (−u, −v)`, `(−v, u)` and `(v, −u)` about `c`,
  since the angle denotes exactly those and no trig is needed; a bolt circle
  of four holes on integer coordinates is `Exact`. The plane's rotation sense
  is the world rotation's, reversed when `Axis` is `−N` and reversed again
  when `xform` is a reflection, which conjugates a rotation into its inverse.
- A point's charge is its two coordinates' errors summed (the one that
  rounded, when only one did). A segment's charge is its largest point
  charge, tripled for an arc (its centre and radius both move with its three
  points' rounding, the argument `offsetSectionDelta` states), and
  `δ_pattern` is the largest segment charge, added to the receiver's own
  `sectionDelta`.
- Every `CircleSeg`'s `CCW`, every `ArcSeg`'s sense and every `TStart`/`TEnd`
  carry over unchanged: a translation or a proper rotation changes no sense,
  and the reflection of a reflected receiver stays in `xform`. A stacked
  receiver's interfaces are re-derived from its moved regions
  (`stackedInterfaces`), each changed-outer interface of a union-built stack
  on the side its receiver records. A union-built stack keeps its frame only
  under a motion with `δ_pattern == 0`: its narrower outer lies inside the
  wider one by construction, which no audit re-proves, so a rounding motion
  copies through `PlacedCopy` instead.

`PatternCopies` builds exactly these records too, one `prismPayload` per
instance under the receiver's frame and placement, so a copy and a group
instance are the same geometry with the same bound.

### 6.3 The prism group

A prism group is a `stackedPrismPayload` with one slab and `Count` regions.
Stacked §2.2's I1 and I2 read differently for it, and the implementation PR
changes that document on the same branch: I1 — a one-slab stack with one
region is a prism (`ErrUnsupported` as today); with two or more regions it is
a prism group. I2 — a slab holds one region, except the single slab of a
prism group, whose regions must be proven pairwise disjoint (below). I3–I7
are unchanged (no interfaces exist). Each region carries the receiver's
holes, translated or rotated with its outer.

**Disjointness is `sketch`'s answer, read structurally, and the check may only
refuse.** A private scene holds every instance's OUTER loop (holes stay
inside their outers by the receiver's own validity). `s.Profiles()` must
return exactly `Count` cells, each `Valid`, each reproducing one instance's
outer with every edge `Whole`, and none carrying a hole. Any `Partial` edge
(two outers cross), any cell with a hole (one outer inside another), any
extra or missing cell, or any invalid cell means "not proven disjoint", and
`Patterned` takes §6.1 rule 3. Two instances sharing a wall arrive as
coincident carriers whose shared span bounds two cells, never one instance's
whole outer, so they take rule 3 as well, where `Union` joins them
(general-boolean class A3). The scene is capped by `prismMaxArrangementSegments`
(prism-boolean §10); exhaustion is `ErrUnsupported`.

Topology, roles and measurements are stacked §3–§4 with the region index
alive: a wall is `slab(0).region(r).side(i, j)`, the caps are `capStart` /
`capEnd` on every region's cap face (so `FaceCreatedBy(CapStart(group))`
selects `Count` faces, which is the selector semantics core §9 already
states), each lump is one region's prism, and `Volume`, `Area`, `Centroid`,
`Bounds` are the bounded sums stacked §4 states, with one `sectionDelta`
(`δ_pattern`) for the payload. Tessellation is stacked §5 per column with
loop clearance proven per slab, which is the one slab. `Lumps()` reports
`Count` lumps.

### 6.4 Consumers of a prism group

| Consumer | Behaviour |
|---|---|
| `Cut(target, group)` | the N-hole case. `docs/general-boolean-design.md` class A5 admits a prism-group tool: its regions enter one private scene as `Count` hole candidates, and the clean-nesting match requires the target's outer whole plus `Count` new holes each reproducing one region whole. One arrangement, one result, `Exact` where the target and `δ_pattern` are |
| `Union(group, other)` / `Intersect` | general-boolean class A5; until it lands, the mesh path over the group's own tessellation (stacked §6) |
| `Patterned` of a group (a grid) | the frame-keeping arm applies to every region at once; the disjointness scene holds `Count_1 · Count_2` outers |
| `Mirrored` / `MirroredCopy` | reflection in `xform` (§3) |
| `WithJoin` | `ErrUnsupported` (J1): the join does not splice a group's lumps one by one |
| `Fillet` / `Chamfer` / `Shell` | `ErrUnsupported` (modify-reach RX3 / SX10 for a stacked receiver), as today |
| `Verify` surveys, clearance, tolerance gate | stacked §6 |
| `Tessellate`, STL/OBJ/3MF/STEP | stacked §5; STEP refuses several shells (`docs/step-export-design.md`) and a group has `Count`, so STEP of a group is `ErrUnsupported` until that writer takes several solids |

## 7. Soundness of each computed quantity

| Quantity | Computation | Bound and charge | What a check may only reject |
|---|---|---|---|
| Mirror image under `Mirrored`/`MirroredCopy` | `r3.Reflection` composed onto `xform` | the placement rounding every placed payload already charges (evaluator §8) | nothing new: no check runs |
| Mirror line (J2–J4) | the selected wall's recorded `Start`/`End` | exact: the record's own floats | J3 collinearity and J4 wholeness are exact decisions over the record |
| Side test (J5) | exact rational signed distances and radius-squared comparison | exact | refuses a loop on the wrong side or crossing; admits nothing the arithmetic does not decide |
| Reflected coordinate | `big.Rat` reflection, one rounding | `δ_mirror = max rationalFloatError`, into `sectionDelta` | — |
| Closure of each spliced loop | the seam's junction falsifier on the assembled record | — | rejects a junction the two recorded coordinates contradict (RB9); admits nothing |
| Simplicity, orientation, nesting of the assembled record | modify §5 audit, empty blend map | the diameter-anchored floor is verification §4's | S7 refuses near contact; S8/S9 refuse a decided break; none admits |
| Instance translation | exact rational offset on the placed axes, per-coordinate interval sums | `δ_pattern`: `IntervalFloatError` per coordinate, covering offset, enclosure, conversion and sum, into `sectionDelta` | — |
| Instance rotation | `TurnSinCosInterval` enclosures over an exact centre, exact quarter and half turns | `IntervalFloatError` per coordinate into `δ_pattern` | — |
| Disjointness of a group | `sketch` arrangement of the outers, read structurally | — | any `Partial` edge, hole or invalid cell refuses the group; nothing admits one but `Count` whole valid cells |
| Group measurements | stacked §4 bounded sums | each term's own bound; `exactSumRound` on the sum | — |

No step evaluates a residual against a curve to decide anything. The two
places a tolerance appears — S7's contact floor and `sketch`'s own carrier
band inside the disjointness scene — can only refuse a pair.

## 8. Test plan

Every test asserts computed geometry; bounds are asserted as relations
(FMA rounding differs between amd64 and arm64).

- **Mirror of every payload** (prism, revolve, loft, sweep, stacked, cup, cap
  blend, faceted): `MirroredCopy` across `MirrorFrame` at x = 30 reports the
  source's volume and area within their bounds, a centroid whose x is
  `60 − x_source` within the centroid bound, `Verify` `Sound` for the
  payloads whose source is, and a `Tessellate` whose signed volume is
  positive. The prism and cup cases assert `Exact` volume.
- **`MirrorFace`**: a 20×10×10 box mirrored across its wall x = 20 by
  `Faces(Planar(), Facing(+x))` lands the image at x ∈ [20, 40]; across its
  top cap, at z ∈ [10, 20] (only the join's J2 refuses a cap); across a
  cylinder's curved face: `ErrDegenerate`; across a `Faceted` flat face:
  `ErrUnsupported`; the L prism of §2, whose `Facing(+x)` resolves two
  walls, without `WithJoin`: `ErrCardinality` with `Expected "exactly 1"`.
- **Join, one wall**: the L (area 175 mm², height 10) joined across its wall
  x = 0 (`Faces(Planar(), Facing(−x))`, y ∈ [0, 20]) yields one lump of
  `Exact` volume 3500 mm³ (a T), an outer loop of 8 line segments, centroid
  x = 0 exactly, 10 faces, every edge on two faces, `Fillet` of the six
  convex vertical edges succeeding and `Concave()` selecting the two others.
- **Join, two walls (U → ring)**: a U of outer 30×20 with a 10×15 notch
  open at x = 30, joined across the two tip walls at x = 30 (selected by
  `FaceCreatedBy` over the two faces' own `Origins()`, `AtLeast(1)`): one
  outer loop, one hole of 20×15, `Exact` volume `(60·20 − 20·15)·h`,
  `Lumps()` one, `Shells()` one non-void; the same selection plus the
  notch's inner wall at x = 20 is J3's `ErrDegenerate`.
- **Join on a stacked receiver**: a 10×10×10 plate with a 4×4×4 pocket from
  the top (936 mm³) joined across its wall x = 10 becomes a 20×10×10 plate
  with two pockets, `Exact` 1872 mm³, two slabs, one interface with two
  exposed floor records, and the stacked audit passing on the re-derived
  exposed records.
- **Join refusals**: `MirrorFrame` → `ErrUnsupported`; a cap → J2's
  `ErrDegenerate`; a loop crossing the line (the wall a sub-segment of a
  longer carrier, J4) → `ErrUnsupported`;
  an arc bulging past the line (J5) → `ErrUnsupported`, with the test
  asserting the arc's endpoints are both on the material side so the bulge
  is the only cause; a receiver with `sectionDelta > 0` → `ErrUnsupported`.
- **`δ_mirror`**: a wall on the line u = 0.1 (not a float multiple of a
  power of two) reports `sectionDelta > 0` equal to the largest
  `rationalFloatError` of `2·0.1 − u` over the record, and the volume bound
  contains the exact rational residual; the same shape on u = 8 reports
  exactly `0.0`.
- **Linear pattern, exact**: a 5×5×5 peg patterned 4× at 10 mm along +x:
  `Patterned` returns 4 lumps, 24 faces, `Exact` 500 mm³, `sectionDelta`
  exactly `0.0`; `PatternCopies` returns 3 live bodies and leaves the
  receiver live, each instance `Exact` with centroid x = 2.5 + 10i.
- **Linear pattern, charged**: step `units.Inches(0.3)` reports
  `sectionDelta > 0`, at least the conversion's `rationalFloatError`, and an
  instance centroid enclosing the exact `2.5 + i · 0.3 · 25.4` over the held
  floats (`units.Inches(1)` converts to the float 25.4 with no rounding).
- **Circular pattern**: six Ø4 holes on a 20 mm radius about the plate normal:
  `Patterned` on the cylinder tool gives 6 regions, each centre within the
  published `sectionDelta` of `(20 cos θ_i, 20 sin θ_i)` taken over
  `math/big` certified trig; `Count = 4` on integer coordinates reports
  `sectionDelta` exactly `0.0`.
- **N-hole cut** (with general-boolean class A5): a 60×40×5 plate cut by the
  six-hole linear group reports no `Faceted` face, volume within its bound of
  `12000 − 6·π·4·5`, a bound below 1e-9 mm³, `sectionDelta` exactly `0.0`,
  `Fillet` succeeding and the wall survey measured. The fixture is shown to
  fail with the group tool replaced by six `PlacedCopy` cuts (which publish a
  positive `sectionDelta`).
- **Overlapping instances**: a 15 mm peg at 10 mm steps: the disjointness
  scene does not prove the pair disjoint, the test asserts that premise, and
  `Patterned` returns exactly what `Union` returns for the receiver and the
  instance `PatternCopies` builds: one analytic prism of 625 mm³, the two
  pegs' long walls joined as coincident carriers (general-boolean class A3).
  Two Ø6 discs crossing about the origin join into one lump the same way.
- **Touching instances**: a 10 mm peg at 10 mm steps: `Patterned` returns
  `Union`'s one analytic prism of 500 mm³, pinned to `Union`'s own outcome.
- **Non-co-directional**: a 5 mm peg patterned along +z at 5 mm: a
  `PlacedCopy` instance and, for `Patterned`, `Union`'s own result: the
  mesh path's coplanar refusal, since the instance's own placement keeps the
  pair off the analytic stacked union, pinned.
- **Cancellation**: a context cancelled during the disjointness arrangement
  returns `ctx.Err()` with the receiver live and nothing registered.

## 9. Do not do this

- **Normalise the reflection into the record.** A right-handed frame cannot
  absorb an improper map, and composing a world reflection onto `xform` to
  make it proper rounds the stored placement and re-orthonormalises it. The
  carried bit is exact and already read by every consumer.
- **Decide "lies on the mirror plane" by a residual.** A wall is on the line
  by exact identity with a recorded carrier (J2–J3) or the join is refused.
- **Build a pattern as N sequential mesh unions.** The result is `Faceted`
  forever: no `Fillet`, no survey, no later coplanar boolean (§2 P2).
- **Let `Patterned` merge overlapping instances silently.** It runs `Union`
  and returns that boolean's own verdict (§6.1 rule 3).
- **Place prism instances by `PlacedCopy` inside the pattern.** A per-instance
  placement makes every later analytic boolean re-express the section and
  charge `δ_reexpress` (§2 S5); the group keeps one frame.
- **Wait on an exact planar rotation from `r3`.** The circular pattern rotates
  the RECORD with certified trig; `RotationAround`'s known inexactness
  (prism-boolean §3.3) never enters.
- **Treat `sketch.CreatePatternRect` as the 3D pattern.** It answers only
  holes drawn in one sketch before the extrude, which is the right tool for
  that case and no other.

## 10. Open questions

- **Should `WithJoin` accept a `MirrorFrame` whose plane exactly contains a
  recorded wall?** Recommendation: no. The exact identity between a frame
  the caller built and a recorded carrier is a bit comparison that rarely
  holds; `MirrorFace` names the wall directly and is the natural statement.
- **Should a prism group of non-prism receivers (revolves, faceted lumps)
  exist?** Recommendation: not now. It needs a group payload every consumer
  reads; `PatternCopies` covers the use, and the pattern's efficiency case
  (N holes) is a prism tool.
- **Should `Patterned` return `[]*Body` for rule 3 instead of running
  `Union`?** Recommendation: run `Union`. A pattern that returns several
  bodies is `PatternCopies`; two shapes of one call would make a caller
  branch on a hidden decision.
- **Count semantics (receiver included).** Recommendation: included, as the
  definition states; Fusion's quantity counts the original too, and the
  Fusion add-in a caller writes next mirrors the number.
- **Order of lifting prism-boolean G2 against this design's join.**
  Recommendation: the join first (PR 2), since it needs no boolean and no
  upstream change; G2's lift (general-boolean A4) then covers the general
  reflected pair.

## 11. PR split

Each PR ships code and tests; this document ships with PR 1.

1. **`Mirrored`, `MirroredCopy`, `MirrorPlane`.** The sealed plane
   vocabulary, `MirrorFace` resolution through the implicit exactly-one
   rule, both methods over `Placed`/`PlacedCopy` with `r3.Reflection`, the
   §3 consumer table verified by §8's every-payload test, an executable
   example, Layout rows.
2. **`WithJoin`.** J1–J7, the exact reflection and splice (§5.2) over prism
   and stacked receivers, `δ_mirror`, the modify §5 audit reuse, §8's join
   tests, and the stacked §2.2 wording for a joined stacked payload.
3. **`PatternCopies`, `LinearPattern`, `CircularPattern`.** The frame-keeping
   instance records (§6.2) with `δ_pattern`, certified trig and the exact
   quarter/half turns, `PlacedCopy` for every other case, §8's pattern tests.
4. **`Patterned` and the prism group.** Stacked §2.2's I1/I2 reading, the
   disjointness scene, multi-region slab topology, measurements and
   tessellation, rule 3's `Union` fallback, and the N-hole `Cut` admission
   shared with `docs/general-boolean-design.md` class A5 (that PR lands the
   boolean side; this one lands the tool and the test that pins the two
   together).
