package decad

import (
	"context"
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/r3"
)

// This file is the single owner of every proven error bound a faceted
// (boolean-built) measurement reports. NO measurement site computes a bound
// inline: each error mechanism the mesh boolean is subject to has exactly one
// helper here, and every site routes through it.
//
// The mechanisms, and the helper that owns each:
//
//   - the chord displacement δ of a point on an OPERAND's surface → carried in
//     the payload, composed by rimDelta;
//   - the TRIM AMPLIFICATION of a point on the boolean's OWN rim: a rim vertex
//     is the crossing of two chord PLANES, so it is displaced not by δ but by
//     (δA + δB)/sin θ, θ the crossing angle — unbounded as the surfaces
//     approach tangency → rimDelta, which refuses when the inflated bound
//     stops meaning anything;
//   - float SUMMATION of the reported value itself → sumSlop, a proven bound
//     for a NAIVE loop (never zero for a float-computed value, which is what
//     keeps exactnessOf honest);
//   - ACCUMULATION over the N elements of a chain → chainLengthBound;
//   - RIGID-MOTION rounding → rigidRoundAllow, charged at the INPUT and
//     translation magnitudes, which is where the rounding is actually
//     committed — never at the output's;
//   - the VOLUME a vertex displacement sweeps out → sweptVolumeAllow, charged
//     against perturbedAreaUpper — the area of the surface the displacement
//     acted ON, which is NOT the area of the mesh that survived it;
//   - the VOLUME between a loft's HELD CHORDED polyhedron and the curved
//     solid its paired curved sections denote → chordedBoundaryVolumeAllow,
//     the SAME closed form sweptVolumeAllow states but for a DIFFERENT
//     mechanism — a boundary REPLACED by a nearby non-mesh surface, rather
//     than a mesh whose vertices moved — so it is its own helper with its
//     own derivation rather than an extension of that one;
//   - the AREA one chorded wall cell's intermediate RULED surface can exceed
//     its own held chord facet's area by, on that same chord-to-curve
//     homotopy → cellRuledExcessUpper, the term chordedBoundaryVolumeAllow's
//     own areaUpper obligation composes beside perturbedAreaUpper;
//   - the AREA a 2D boundary displacement sweeps out → sectionDisplacementArea,
//     the same identity one dimension down: the region a recorded section can
//     move is a tube about its own recorded boundary, with
//     sectionDisplacementLength reading the same displacement as a perimeter;
//   - the DISPLACEMENT a recorded CUT PARAMETER puts on the point it names —
//     the fragment endpoint slides along its own exact carrier by the parameter
//     rounding times the carrier's speed → cutDisplacementAllow;
//   - the DISPLACEMENT a consumed source segment's own WALKED endpoint puts on
//     the point the private scene builds from it, when the segment's recorded
//     range is narrower than its own natural domain →
//     walkEndpointAllow, charged at the SOURCE operand magnitudes the walk's
//     arithmetic touches — never at the endpoint that arithmetic produced,
//     which a cancelling difference can leave arbitrarily small;
//   - a free-form WALK ENDPOINT's own per-component bound (walkEndBound,
//     segmentWalk's startBound/endBound and the identical reading over an
//     interior span joint), read as a 3D world-space DISTANCE the point can
//     sit from the curve it denotes → walkEndBoundAllow, named for the
//     endpoint rather than for its one caller today (verify.go's free-form
//     tolerance-gate diameter arm) so a later reading over the same bound can
//     share it;
//   - the AREA of ONE RULED QUAD whose cap-level chord alone is displaced (the
//     cap-loop chamfer's own band patches, docs/modify-reach-design.md §8.4) →
//     bandPatchAreaAllow, the same two-factor product one patch at a time
//     rather than one whole section;
//   - the AREA of that SAME patch under a displacement of its OTHER directrix
//     — the side level, one rounded float sum, so the whole directrix
//     translates rigidly rather than moving point by point →
//     bandLevelAreaAllow;
//   - the VOLUME gap between a cap-loop chamfer's straight-ruled Cone patch
//     and the curved miter locus it denotes at a non-tangential corner →
//     chordLocusVolumeAllow, an erosion-monotonicity sandwich against the
//     ordinary shared-window cone-sector flux plus a swept-volume term over
//     the built patch's own closed-form displacement from it;
//   - the FIRST MOMENT a cap-loop chamfer's contour displacement can move →
//     sweptMomentAllow, sweptVolumeAllow's own one-dimension-higher sibling;
//   - the FIRST MOMENT a loft's chorded boundary can move under the same
//     chord-to-curve homotopy → chordedBoundaryMomentAllow,
//     chordedBoundaryVolumeAllow's own one-dimension-higher sibling, NOT
//     sweptMomentAllow reused, since that helper calls sweptVolumeAllow
//     internally and so speaks for the wrong mechanism here;
//   - the LENGTH gap between a cap-loop chamfer's straight-ruled corner
//     miter ruling and the curved locus it denotes at a non-tangential
//     corner adjacent to a circular wall → chordLocusLengthAllow, the
//     range's own width times a proven upper bound on the locus's own speed,
//     minus the held chord;
//   - a per-coordinate maximum read as a 3D DISTANCE → radius3D;
//   - propagating a proven interval through a SQUARE ROOT (a candidate disk's
//     own centre distance, an Apollonius radius) → boundedSqrt, which reads
//     the operand's own interval ends through the same rational sqrt brackets
//     (ratSqrtDown/ratSqrtUp) a free-form arc's radius already does, rather
//     than trusting math.Sqrt's accuracy on either end;
//   - a linear functional's own extreme over a bounded region moving when its
//     DIRECTION is perturbed (a revolved solid's directional extent, whose swept direction
//     carries the sweep angle's own trig enclosure) →
//     directionalPerturbationAllow, charged against the envelope of the very
//     coordinate that direction multiplies — which is the caller's to name,
//     since a revolve's swept radial coefficient multiplies a distance from
//     the axis and not from the profile's own frame origin;
//   - a HELD float measured against a bounded scalar's own proven enclosure of
//     the quantity that float stands for → boundedFloatError, the bridge every
//     candidate producer crosses when it evaluates a value one way and proves
//     it another (extrude.go's circular boundary-extreme candidate, whose held
//     position runs through math.Cos/math.Sin while its enclosure comes from
//     the angle-free apex identity).
//   - a HELD scalar coefficient a directional reading lifts a boundary or
//     sweep extreme through — a prism or cap-blend box's base/gu/gv/gz, a
//     revolve box's base/wg/c0/c1 — measured against the EXACT rational image
//     of the same frame-and-placement chain the coefficient's own float
//     evaluation ran → exactIsometryDotRound, rigidRoundAllow's tight
//     companion: zero exactly where that chain's arithmetic commits no
//     rounding for the input at hand (an identity placement, an axis-aligned
//     frame direction), never a worst-case ulp estimate where an exact
//     rational comparison is available instead.
//   - a HELD PLANE-LOCAL COORDINATE a world point projects to under a frame —
//     a revolve axis's own anchor (revolve.go's axisInPlane) — measured against
//     the EXACT rational image of the same (p − origin)·axis chain the
//     coordinate's own float evaluation ran → exactFrameLocalRound,
//     exactIsometryDotRound's sibling one transform earlier. It charges the
//     ROUNDING that projection commits and nothing else, so a magnitude
//     envelope over the point's own distance from the frame origin — a term
//     that grows with that distance while the projection's true error stays
//     zero — never stands in for it.
//   - a HELD PLANE-LOCAL DOT PRODUCT of a direction against a frame's own axis
//     anchor — the anchor shift a revolve's extreme reading subtracts to move a
//     plane-local extreme into axis coordinates (revolve.go's
//     axisExtremeContext) — measured against the EXACT rational value of the
//     same two products and their sum → exactPlaneDotRound. Its two products
//     round at the ANCHOR's own magnitude, so the term grows with the anchor
//     while staying the rounding it is.
//   - the same plane-local dot product gu·u + gv·v evaluated at every CANDIDATE
//     the boundary-extreme scan folds (extrude.go's
//     boundaryExtremesBoundedContext), where no single u and v is available to
//     measure against → planeDotDecompositionRoundAllow, exactPlaneDotRound's
//     envelope-scaled sibling: the anchor helper answers for ONE stated point,
//     this one for a whole scan, at the SECTION's own coordinate envelope
//     rather than the anchor's magnitude. A reading that charges only the
//     anchor's dot publishes a section extreme whose own multiply-and-sum
//     rounded at a magnitude nothing else in the reading scales with.
//   - the FINAL SUMMATION a directional extent reading commits when it
//     recombines its own already-held terms into ONE published endpoint — a
//     prism's base + boundary extreme + sweep level, a revolve's or a cap-loop
//     chamfer's base + boundary extreme → exactSumRound, exactIsometryDotRound's
//     own companion one step later: that helper proves each COEFFICIENT right,
//     this one charges what ADDING the terms commits, and a placement can leave
//     every coefficient exactly right while the sum still rounds.
//   - the DISPLACEMENT a deliberate SNAP-TO-ZERO puts on the coordinate it
//     overwrites — a revolve walk endpoint's radial coordinate assigned exactly
//     0 within the axis's own contact tolerance (revolve.go's axisFrame.walk) →
//     snapToZeroAllow, which charges the discarded magnitude itself, because
//     that magnitude IS the error the assignment introduces. A reading that
//     charges only the pre-snap arithmetic's own rounding publishes the assigned
//     zero as Exact and excludes the positive radius it stands for.

const (
	// unitRoundoff is float64's u = 2⁻⁵³: the relative error a single
	// round-to-nearest operation can commit.
	unitRoundoff = 1.1102230246251565e-16
	// sqrt3Up is √3 rounded UP, so a per-coordinate bound scaled by it stays
	// an upper bound on the 3D corner distance.
	sqrt3Up = 1.7320508075688774
)

// upRound nudges a positive bound to the next representable float64, so the
// bound's own rounding can never land it below the quantity it bounds.
func upRound(x float64) float64 {
	if x > 0 {
		return math.Nextafter(x, math.Inf(1))
	}
	return x
}

// radius3D turns a per-coordinate bound into the 3D distance bound its
// consumers read: all three coordinates can be off at once, so the corner sits
// up to √3 times the per-coordinate bound away (core §5.2 — a coordinate's
// error bound is a radius, not an axis extent).
func radius3D(perCoord float64) float64 {
	if perCoord <= 0 {
		return 0
	}
	return upRound(perCoord * sqrt3Up)
}

// sumSlop is a PROVEN bound on the rounding a NAIVE float64 summation of n
// terms commits, given absSum = Σ|term|. The classic bound is
// (n−1)·u/(1 − (n−1)·u) · Σ|term|, which 2·(n−1)·u·Σ|term| dominates for
// (n−1)·u ≤ ½ — true for any mesh a machine can hold. On top of it each term
// is itself a float evaluation (a cross product, a norm, a square root): a
// handful of ulps, charged as 4·u per term.
//
// It is NEVER zero for a positive FINITE absSum. That is the point: a
// float-computed value is not exactly representable, so it may never reach
// Exact.
//
// A non-finite absSum is the one case it answers 0 for, because a saturated
// scale is no scale and this helper may not invent one. That answer is a
// PRECONDITION on the caller, not a bound: a caller whose absSum can saturate
// independently of the value it speaks for must state its own bound for that
// case, or the term silently vanishes (loft_moments.go's wallBound does, with
// +Inf). Every other caller here passes the held value itself as absSum and
// adds this term to it, so a saturation carries +Inf into the published bound
// on its own.
func sumSlop(n int, absSum float64) float64 {
	if n <= 0 || absSum <= 0 || isNonFinite(absSum) {
		return 0
	}
	loop := 2 * float64(n-1) * unitRoundoff * absSum
	terms := 4 * unitRoundoff * absSum
	return upRound(loop + terms)
}

// chainLengthBound is the proven bound on a boolean rim's length: the chain
// holds nSegs chords whose two endpoints EACH move by up to delta, so the
// held length can be off by 2·nSegs·delta — plus the float slop of summing
// nSegs square roots. Never zero for a float-computed length, even when
// delta is (an all-planar boolean's rim is a float sum of sqrts, and the last
// ulp is not free).
func chainLengthBound(nSegs int, delta, heldLen float64) float64 {
	if nSegs <= 0 {
		return 0
	}
	return upRound(2*float64(nSegs)*delta + sumSlop(nSegs, heldLen))
}

// maxFiniteUlp is 2⁹⁷¹, the spacing between the two largest finite float64s
// and therefore the LARGEST spacing any pair of adjacent finite float64s has.
// Half of it bounds the rounding a single operation whose result is finite can
// commit, whatever that result's magnitude.
const maxFiniteUlp = 0x1p971

// rigidRoundAllow bounds the rounding a rigid motion commits on one point.
// The rounding happens INSIDE the products and sums — at the magnitude of the
// INPUT coordinate and of the translation — not at the magnitude of the
// result: a body built far from the origin and moved back rounds at the far
// magnitude and would be charged at the near one. Every intermediate stays
// under 2·maxInputAbs + maxTransAbs (an orthonormal row's dot product is at
// most √3·|input|), 16 ulps there cover the products, the two sums and the
// translation, and the consumers read a 3D distance, so ×√3.
//
// That scale is a MAGNITUDE, not a value the motion produces, so it saturates
// on inputs the motion itself handles perfectly well: 2·maxInputAbs overflows
// for any coordinate above MaxFloat64/2, and ulpOf answers +Inf at MaxFloat64
// and NaN past it, since its own math.Nextafter step leaves the finite range.
// The answer must never be NaN. NaN is not a large bound but the ABSENCE of
// one, and it is silent: NaN > 0 is false, so every consumer's own `delta > 0`
// widening is skipped and the term vanishes from the measurements it was
// supposed to widen. So a saturated scale falls back to maxFiniteUlp, which
// is a PROVEN charge rather than a substitute for one: this helper answers for
// a point whose placed coordinates the caller has already proven finite (a
// non-finite coordinate is that caller's own refusal — docs/loft-design.md
// Table S row S13 for a loft), an operation with a finite result rounds by at
// most half the spacing at that result, and 16·maxFiniteUlp dominates the six
// products and sums one placed coordinate commits (three products, two sums
// joining them, and the translation's own).
//
// A second call site is the plane-frame lift of a COMPUTED chord station (a
// loft's own curved-pairing reach, docs/loft-design.md §5.2): the mechanism
// is identical, an orthonormal map (Plane.ToWorldUV, in effect r3's own
// Frame) composed with a translation, so the rounding it commits is bounded
// the same way — at the pre-lift plane-local coordinate's own magnitude and
// the frame origin's, never at the lifted world point's.
func rigidRoundAllow(maxInputAbs, maxTransAbs float64) float64 {
	m := 2*math.Abs(maxInputAbs) + math.Abs(maxTransAbs)
	ulp := ulpOf(m)
	if isNonFinite(ulp) {
		ulp = maxFiniteUlp
	}
	return radius3D(16 * ulp)
}

// perturbedAreaUpper bounds the total facet area of a mesh whose vertices may
// each sit up to delta from the HELD ones — and of every mesh on the straight
// path between the two, which is what the swept-volume bound integrates over.
//
// Per facet, with held edge vectors u', v' and true ones u = u' + du,
// v = v' + dv (|du|, |dv| ≤ 2·delta, each endpoint moving by up to delta):
// |u × v| ≤ |u' × v'| + 2·delta·(|u'| + |v'|) + 4·delta², so the true area is
// at most the held area plus delta·(|u'| + |v'|) + 2·delta². A facet the weld
// COLLAPSED holds zero area and the correction is the whole of its bound —
// which is the point: it is the only term that speaks for it.
func perturbedAreaUpper(verts []r3.Vec, tris [][3]int, delta float64) float64 {
	area, _ := perturbedAreaUpperWithBudget(nil, verts, tris, delta)
	return area
}

// perturbedTriangleAreaAllow bounds how far ONE triangle's area can move when
// each of its three vertices sits within delta of the held ones: delta·(|u| +
// |v|) + 2·delta², u and v the triangle's own edge vectors b-a and c-a — the
// per-facet correction term perturbedAreaUpper's own doc comment derives,
// pulled out as its single owner (docs/loft-design.md PR 2a) so a caller that
// needs one triangle's own allowance, rather than a whole mesh's held area
// plus the same correction summed over every facet, does not recompute it
// inline.
func perturbedTriangleAreaAllow(a, b, c r3.Vec, delta float64) float64 {
	u, v := b.Sub(a), c.Sub(a)
	return delta*(u.Len()+v.Len()) + 2*delta*delta
}

func perturbedAreaUpperContext(
	ctx context.Context,
	verts []r3.Vec,
	tris [][3]int,
	delta float64,
) (float64, error) {
	return perturbedAreaUpperWithBudget(newWorkBudget(ctx), verts, tris, delta)
}

func perturbedAreaUpperWithBudget(
	budget *workBudget,
	verts []r3.Vec,
	tris [][3]int,
	delta float64,
) (float64, error) {
	total := 0.0
	for _, t := range tris {
		if budget != nil {
			if err := budget.step(); err != nil {
				return 0, err
			}
		}
		a, b, c := verts[t[0]], verts[t[1]], verts[t[2]]
		u, v := b.Sub(a), c.Sub(a)
		total += u.Cross(v).Len()/2 + perturbedTriangleAreaAllow(a, b, c, delta)
	}
	if budget != nil {
		if err := budget.err(); err != nil {
			return 0, err
		}
	}
	return upRound(total + sumSlop(len(tris), total)), nil
}

// sweptVolumeAllow bounds the volume between two closed meshes whose vertices
// correspond and differ by at most delta. The signed volume is a polynomial in
// the vertices, so along the straight path from one mesh to the other
// |dV/dt| ≤ delta · A(t) — every boundary point moves at speed at most delta,
// and it can only displace volume at the rate the area it sweeps allows. So
// |V' − V| ≤ delta · sup A(t), and areaUpper must bound the area along the WHOLE
// path (perturbedAreaUpper does).
//
// The identity holds facet by facet, so it holds whatever the rounding does to
// the mesh's shape — a facet flattened to zero area still answers for the volume
// it swept getting there. What it needs is the area of the surface the motion
// acted ON: charge it against what survived the motion and the collapsed facets'
// own swept volume drops silently out of the bound.
func sweptVolumeAllow(delta, areaUpper float64) float64 {
	if delta <= 0 || areaUpper <= 0 {
		return 0
	}
	return upRound(delta * areaUpper)
}

// cellRuledExcessUpper bounds the AREA one loft wall cell's intermediate
// ruled surface can exceed its own held chord facet's area by, on the
// chord-to-curve homotopy chordedBoundaryVolumeAllow integrates over
// (docs/loft-design.md §5.2, the A10 plan's Part 4 R1 fallback).
//
// One intermediate surface is itself a RULED surface X(s, r) = (1−r)·a(s) +
// r·b(s) between an interpolated curve a on section 0 and b on section 1, so
// X_r = b(s) − a(s) and X_s = (1−r)·a'(s) + r·b'(s), giving |X_s × X_r| <=
// (|a'(s)| + |b'(s)|) · |b(s) − a(s)| — the rule's own length times the
// interpolated boundary's own speed — and therefore Area <= ruleLengthUpper ·
// (length(a) + length(b)). Curve length is a convex functional
// (|(1−t)·P' + t·Q'| <= (1−t)·|P'| + t·|Q'|), so the interpolated curve a (or
// b) never runs longer than the LARGER of its own chord chain and the true
// curve it interpolates toward — which is the true curve. The excess over
// the chord facet's own held area is therefore at most ruleLengthUpper ·
// ((arcLen0 − chordLen0) + (arcLen1 − chordLen1)), an unconditional and
// closed form.
//
// ruleLengthUpper must be a PROVEN upper bound on the cell's own rule length
// (the diagonal or rung spanning the two sections). excess0 and excess1 must
// each be a PROVEN upper bound on that section's own arc-minus-chord length
// excess over the cell — never negative, since a chord never exceeds the
// length of the curve it subtends; a negative reading is the caller's own
// error and this helper answers 0 for it rather than let a negative excess
// shrink the term it is meant to widen.
func cellRuledExcessUpper(ruleLengthUpper, excess0, excess1 float64) float64 {
	if ruleLengthUpper <= 0 || excess0 < 0 || excess1 < 0 {
		return 0
	}
	return productUpper(ruleLengthUpper, absSumUpper(excess0, excess1))
}

// chordedBoundaryVolumeAllow bounds the VOLUME between a loft's HELD chorded
// polyhedron and the TRUE solid its paired curved sections denote
// (docs/loft-design.md §5.2, §8; the A10 plan's Part 1/Part 2 Q4). It carries
// the SAME closed form sweptVolumeAllow states — sectionDelta · sup A(t) —
// but for a DIFFERENT mechanism, so it is its OWN helper with its OWN
// derivation rather than an extension of that one:
//
// (a) The homotopy. Each chord point moves along the STRAIGHT path to the
// curve point at its own parameter, a motion of at most sectionDelta — the
// in-section-plane displacement of a built chord from the recorded curve it
// chords (docs/loft-design.md §5.2). The signed volume is a polynomial in the
// boundary, so along that path |dV/dt| <= sectionDelta · A(t) — the flux of
// that velocity through the moving boundary — and |V_true − V_chord| <=
// sectionDelta · sup_t A(t).
//
// (b) What areaUpper must bound: the area of EVERY intermediate surface on
// that path, not merely the held chord mesh's own area. An intermediate
// surface is NOT a mesh whose vertices moved — it is a family of RULED
// surfaces, one per wall cell (cellRuledExcessUpper's own derivation) —
// so perturbedAreaUpper's per-facet argument (three held vertices, each
// individually displaced by at most delta) does not reach it on its own:
// mere containment in a sectionDelta-thick neighbourhood of the chord mesh
// does NOT bound a surface's area, since a surface can carry unbounded area
// inside an arbitrarily thin slab, and a containment argument alone is
// therefore not a proof. The caller must supply areaUpper as
// perturbedAreaUpper(verts, tris, sectionDelta) PLUS, summed over every wall
// cell, cellRuledExcessUpper(ruleLengthUpper, excess0, excess1) for that
// cell's own two sides — the sum this file's own chord-versus-curve fallback
// takes, which proves the bound unconditionally rather than resting on
// containment.
func chordedBoundaryVolumeAllow(sectionDelta, areaUpper float64) float64 {
	if sectionDelta <= 0 || areaUpper <= 0 {
		return 0
	}
	return upRound(sectionDelta * areaUpper)
}

// sectionDisplacementArea bounds the AREA a recorded 2D section can differ from
// the section its construction denotes, given that every recorded boundary
// coordinate sits within delta of that denoted boundary
// (docs/prism-boolean-design.md §7).
//
// The two regions' symmetric difference lies inside the delta-neighbourhood of
// the recorded boundary: a point in one region and not the other has the two
// boundaries between it and either interior, so it is within delta of the
// recorded one. That neighbourhood is covered by a rectangle 2·delta wide along
// each of the walks — perimeterUpper must be a PROVEN upper bound on their total
// length — plus a disk of radius delta at each of the walks joints, so
// 2·delta·p + n·π·delta² encloses it. The bound is therefore a bound on the
// whole SET displacement, not on the arithmetic that produced the coordinates:
// it stands even where moving the boundary by delta changes which regions the
// construction merged, which coordinate-rounding terms alone cannot cover.
func sectionDisplacementArea(delta float64, walks int, perimeterUpper float64) float64 {
	if delta <= 0 || walks <= 0 {
		return 0
	}
	tube := productUpper(productUpper(2, delta), perimeterUpper)
	joints := productUpper(
		productUpper(float64(walks), math.Nextafter(math.Pi, math.Inf(1))),
		productUpper(delta, delta),
	)
	return upRound(tube + joints)
}

// sectionDisplacementLength bounds how far the total LENGTH of a recorded
// section's boundary — walks lines and circular arcs, the class
// docs/prism-boolean-design.md §3.1's G4 admits — can differ from the length of
// the boundary it denotes, given that every recorded coordinate sits within
// delta of that denoted boundary. It is the perimeter's own reading of the same
// displacement sectionDisplacementArea reads as an area.
//
// The per-walk factor is 12·π, which covers both walk kinds. A straight walk's
// two ends each move by at most delta, so its length moves by at most 2·delta
// (chainLengthBound's own reasoning). A circular walk moves more, because its
// radius and its swept angle both move: its centre and both endpoints sit within
// delta, so the radius moves by at most 2·delta and — while the radius is at
// least 4·delta — each endpoint's angle by at most π·delta/R, giving
// |R'θ' − Rθ| ≤ 2·delta·2π + R'·2π·delta/R' = 6·π·delta; a radius under 4·delta
// leaves both arcs shorter than 12·π·delta outright, so the same figure stands.
// The held sum's own float slop is NOT included: the perimeter this composes
// into already carries it.
func sectionDisplacementLength(delta float64, walks int) float64 {
	if delta <= 0 || walks <= 0 {
		return 0
	}
	perWalk := productUpper(12, math.Nextafter(math.Pi, math.Inf(1)))
	return productUpper(productUpper(float64(walks), perWalk), delta)
}

// cutParamUlps is the allowance, in ulps of the parameter domain [0, 1], that
// a sketch-certified cut parameter is charged against the true parameter of the
// crossing it names (docs/prism-boolean-design.md §7).
//
// It is the ONE number the analytic path's displacement rests on beyond what
// sketch itself certifies, so it is stated here once rather than derived at a
// call site. TExact's own claim (docs/sketch-seam-design.md §1) is that the
// range reproduces the fragment's endpoints "to machine precision", and the
// range comes from one closed-form crossing solve over the two entities' own
// defining data — a short expression whose accumulated relative rounding is a
// handful of ulps. 8 ulps of the WHOLE domain dominates a handful of ulps of any
// t ≤ 1, and it is not a proof that sketch rounds well: it is the quantitative
// reading decad gives the seam's precision claim, and a cut that misses it by
// more is a sketch bug the seam's own falsifier is there to catch.
const cutParamUlps = 8

// cutDisplacementAllow bounds how far the point a recorded cut parameter names
// sits from the true crossing point it denotes, given the carrier's own speed
// |dP/dt| bounded above by tangentUpper.
//
// A cut fragment records the entity's UNCHANGED defining data and a narrowed
// t range (docs/sketch-seam-design.md §1), so the carrier is exactly the
// carrier the construction denotes and the whole displacement sits in the
// parameter: the endpoint slides ALONG that carrier by at most |t − t*| ·
// sup|dP/dt|, which cutParamUlps · ulp(1) · tangentUpper covers. Sliding along
// a carrier is still a boundary coordinate moving, so the answer feeds the
// section displacement every other consumer reads, not a private term.
//
// A non-finite carrier speed answers +Inf rather than a number: an absent bound
// must never read as a small one, and a zero would let the displacement vanish
// silently from every measurement that composes it.
func cutDisplacementAllow(tangentUpper float64) float64 {
	if isNonFinite(tangentUpper) {
		return math.Inf(1)
	}
	if tangentUpper <= 0 {
		// A carrier that does not move cannot put the point anywhere else,
		// whatever the parameter reads.
		return 0
	}
	return productUpper(cutParamUlps*ulpOf(1), tangentUpper)
}

// walkEndpointAllow bounds the rounding a source segment's own WALKED
// endpoint commits when the record's parameterisation is evaluated at a
// narrowed t rather than read off the segment's own natural bound
// (docs/prism-boolean-design.md §7's δ_walk — the analytic boolean's private
// scene is built from walkOf's own walked geometry, prism_boolean.go's
// buildPrismScene).
//
// operandUpper must be a PROVEN upper bound on the magnitude of every operand
// the walk's OWN arithmetic touches — not on the answer that arithmetic
// produces. The two are different quantities and the difference is the whole
// point: a line's walked endpoint is fl(a + fl(t·fl(b−a))) for the carrier's
// own recorded a and b, and the difference b−a CANCELS, so a fragment sitting
// near the plane origin on a far-reaching carrier rounds by ulps of the
// CARRIER while its own endpoint magnitude is tiny. Charging the endpoint's
// magnitude would under-charge that walk without limit, so callers pass the
// SOURCE envelope: prism_boolean.go's lineWalkOperandUpper for a line (the
// recorded Start/End coordinates, folded together with the walked endpoint so
// the envelope stands whatever the recorded parameter is).
//
// A trimmed line is the only carrier the analytic boolean charges here. A
// trimmed circular one is refused before its scene is built
// (prismProfileHasTrimmedCircularSource), since its rebuilt radius and sweep
// move as well as its endpoints, so walkChargeOf's circular arm — which
// passes segmentWalk.coordUpper, whose |c|+|c|+r+r L1 form bounds the centre
// and radius a cos/sin walk works on — is unreachable through that path. It
// stands so a widening of that refusal meets a charge rather than a silent
// zero.
//
// The proof, per coordinate, with E = operandUpper and u = 2⁻⁵³ the unit
// roundoff. fl(b−a) is off by at most u·|b−a| ≤ 2uE; multiplying by t carries
// that through and rounds once more, at the magnitude of t·(b−a), which is the
// walked endpoint less a and so at most 2E, for another 2uE; the outer sum
// rounds at |a + t(b−a)| ≤ E, for uE. That totals 5uE + O(u²) per coordinate.
// A target that FUSES the multiply and the add — the gc arm64 backend
// compiles lerp2's general arm to a single FMADDD — drops the middle rounding
// and keeps the other two, so it stays under the same total: fusion only ever
// removes a rounding from that sequence, and fl(b−a), the term this
// mechanism's own cancellation makes large, is committed before any fusion
// and survives it unchanged.
// The answer charges 16·ulp(2E) ≥ 16·ulp(1)·E = 32uE per coordinate, better
// than six times that, read as a 3D radius (radius3D) — the SAME shape
// rigidRoundAllow states for a rigid map's own rounding, which keeps every
// displacement mechanism in this file stated the same way.
//
// A non-finite envelope answers +Inf, never 0: an absent bound must never read
// as a small one (cutDisplacementAllow's own rule, restated here because this
// helper sits right beside it).
func walkEndpointAllow(operandUpper float64) float64 {
	if isNonFinite(operandUpper) {
		return math.Inf(1)
	}
	if operandUpper <= 0 {
		return 0
	}
	ulp := ulpOf(2 * operandUpper)
	if isNonFinite(ulp) {
		return math.Inf(1)
	}
	return radius3D(16 * ulp)
}

// walkEndBoundAllow turns a free-form walk endpoint's own PROVEN
// per-component bound (walkEndBound — segmentWalk's startBound/endBound, or
// the identical reading a caller takes over one interior span joint) into a
// 3D world-space displacement bound. The wider of the two plane-local
// components is read as a per-coordinate bound and carried through the
// payload's orthonormal frame by radius3D, exactly as any other coordinate
// error is (core §5.2 — a coordinate's error bound is a radius, not an axis
// extent): a frame's U/V/N are pairwise orthogonal unit directions, so a
// bound on each in-plane component is itself a bound on the corresponding
// world-space one, and folding the wider of the two through radius3D's own
// three-axis shape can only widen the true two-axis figure, never narrow it.
//
// An underivable component answers +Inf, never a small number silently spent
// in its place (cutDisplacementAllow's own rule).
func walkEndBoundAllow(bound walkEndBound) float64 {
	if !bound.derivable() {
		return math.Inf(1)
	}
	return radius3D(math.Max(math.Abs(bound.u), math.Abs(bound.v)))
}

// bandPatchAreaAllow bounds how far ONE chamfer band patch's own area
// (docs/modify-reach-design.md §8.4) can differ from the area of the ruled
// quad the construction DENOTES, given that its cap-level directrix sits
// within delta of the point it denotes (docs/prism-boolean-design.md §7's
// identity, one ruled patch at a time rather than one whole section). The
// side-level directrix is NOT exact either, but its displacement is a
// different mechanism and has its own helper: bandLevelAreaAllow below, which
// patchAreaOf composes beside this one.
//
// A ruled quad's area is, to first order, its chord length times its slant
// distance, so moving only the cap-level chord changes area two ways at
// once: the chord's own length can change by at most
// sectionDisplacementLength(delta, 1) — the SAME per-walk bound a recorded
// boundary segment's length carries under this displacement, since a single
// chord is exactly what that helper already bounds for one walk — which
// moves area at the rate of the patch's own slant distance; and the slant
// distance can itself change by at most delta, because only its cap-level
// endpoint moves, which moves area at the rate of the chord length it rules
// along. chordUpper and slantUpper must each be a PROVEN upper bound on the
// patch's own held chord length and held slant distance.
func bandPatchAreaAllow(delta, chordUpper, slantUpper float64) float64 {
	if delta <= 0 {
		return 0
	}
	return upRound(productUpper(sectionDisplacementLength(delta, 1), slantUpper) + productUpper(chordUpper, delta))
}

// bandLevelAreaAllow is bandPatchAreaAllow's companion on the band's OTHER
// directrix: how far ONE chamfer band patch's own area can differ from the
// area of the patch the construction DENOTES, given that its SIDE-level
// directrix sits within levelDelta of the level it denotes.
//
// The two directrices are displaced by different mechanisms and so are bounded
// by different helpers. The cap-level contour is moved point by point by a
// float offset solve, which is what bandPatchAreaAllow charges. The side level
// is the single float sum capZ + matSign*d (capblend_geom.go's levelDelta), so
// the whole side directrix is translated AXIALLY and RIGIDLY by at most that
// much, its own in-plane shape untouched.
//
// Under such a translation a patch's area moves at the rate of its own two
// directrix lengths, and both patch shapes the band builds give the same
// figure:
//
//   - a Plane patch is two triangles over the quad (sideA, sideB, capB, capA).
//     Moving sideA and sideB together by t leaves (sideB−sideA) alone, so the
//     first triangle's cross product moves by at most |sideB−sideA|·|t|; the
//     second triangle's only moved vertex is sideA, so its cross product moves
//     by at most |capB−capA|·|t| + |t|². Halving each for the triangle areas,
//     the quad's own area moves by at most
//     ½·(|sideB−sideA| + |capB−capA|)·|t| + ½·|t|².
//   - a Cone patch is the frustum sector A = (Δθ/2)·(R0+R1)·√(ΔR²+H²), whose
//     derivative in H is (Δθ/2)·(R0+R1)·H/√(ΔR²+H²) and is therefore bounded
//     by (Δθ/2)·(R0+R1) — half the sum of that patch's own two directrix arc
//     lengths, Δθ·R0 and Δθ·R1.
//
// directrixSumUpper must be a PROVEN upper bound on the sum of the patch's two
// directrix lengths and levelDelta a proven bound on the axial displacement.
// The ½ is dropped and one levelDelta folded in, which dominates both readings
// above.
func bandLevelAreaAllow(levelDelta, directrixSumUpper float64) float64 {
	if levelDelta <= 0 {
		return 0
	}
	return productUpper(absSumUpper(directrixSumUpper, levelDelta), levelDelta)
}

// chordLocusVolumeAllow bounds capblend_moments.go's chord-versus-locus
// residual: the gap between a cap-loop chamfer's regular-wall Cone patch — the
// STRAIGHT-RULED surface the topology actually builds between its two
// directrices — and the TRUE denoted miter locus (docs/modify-reach-design.md
// §8.3: "at axial fraction s, the denoted miter locus is the parallel section
// offset by s*dc" — a contract the ruled Cone patch meets only at s=0 and
// s=1, chording the true curve strictly between them), at a non-tangential
// corner where the cap-level directrix (capTh0, capTh1) sweeps a window
// narrower than the side-level one (th0, th1) by windowSkewMax at its widest
// corner.
//
// The proof is a two-term decomposition, |Vol(true)-Vol(built)| <=
// |Vol(true)-Vol(wide)| + |Vol(wide)-Vol(built)|, where "wide"/"narrow" are
// the ordinary ROTATIONALLY-SYMMETRIC cone-sector flux this file's own
// pre-existing formula gives for the SIDE window (th0, th1) shared by both
// directrices, and for the CAP window (capTh0, capTh1) shared by both:
//
//   - Vol(true) in [Vol(narrow), Vol(wide)]. Erosion is monotonic in its
//     offset amount (a point surviving the deepest cut d always survived
//     every shallower one, and every survivor of any cut is a point of the
//     wall's own un-eroded extent), so the true swept solid's own
//     cross-sectional window at every axial fraction s is a SET sandwiched
//     between the cap window and the side window — its flux is therefore
//     sandwiched the same way, and envelopeSlack = |Vol(wide)-Vol(narrow)|
//     bounds |Vol(true)-Vol(wide)|.
//   - |Vol(wide)-Vol(built)| <= sweptVolumeAllow(patchDeviation, areaUpper):
//     the built patch is a convex combination, in Cartesian coordinates, of a
//     side-level point at radius sideRadius and angle in [th0, th1] and a
//     cap-level point at radius capRadius and angle in [capTh0, capTh1], so
//     (triangle inequality on the two component vectors) its own radius never
//     exceeds the radius the wide cone's own linear taper gives at that axial
//     level, and (a convex combination of two rays stays angularly between
//     them) its own angle never leaves [th0, th1] either — patchDeviation is
//     the proven per-point cap on how far the built surface can then sit from
//     the wide cone's own surface (a radial term from the two directrices'
//     radii and the corner skew's own cosine, an angular term the same skew
//     scaled by the larger radius), and sweptVolumeAllow turns a per-point
//     displacement bound into a volume one exactly as it already does for a
//     mesh vertex.
//
// windowSkewMax <= 0 (a tangent join, or either degenerate patch, where the
// two windows coincide) leaves the term at zero, unchanged from the
// already-shipped tangent-junction/apex/whole-turn readings.
func chordLocusVolumeAllow(fluxWide, fluxWideBound, fluxNarrow, fluxNarrowBound, sideRadius, capRadius, windowSkewMax, areaUpper float64) float64 {
	if windowSkewMax <= 0 || isNonFinite(windowSkewMax) {
		return 0
	}
	envelopeSlack := absSumUpper(fluxWide-fluxNarrow, fluxWideBound, fluxNarrowBound)
	// 1 - cos(x) = 2*sin(x/2)^2, the numerically stable form: the naive
	// subtraction cancels catastrophically for a small skew, while sin(x/2)
	// itself is computed directly and squaring a small accurate value stays
	// accurate.
	radialDeficit := math.Sqrt(math.Max(0, sideRadius*capRadius)) * math.Abs(math.Sin(windowSkewMax/2))
	maxRadius := math.Max(math.Abs(sideRadius), math.Abs(capRadius))
	patchDeviation := absSumUpper(radialDeficit, productUpper(maxRadius, windowSkewMax))
	// sweptVolumeAllow returns a VOLUME, but this function's return value is a
	// FLUX term (envelopeSlack is a difference of two patchRawFlux results,
	// three times a volume) that patchRawFlux's own bound folds into, to be
	// divided by 3 exactly once at capBandVolume's single division
	// (capblend_moments.go:418). Scaling this volume term up by 3 here is what
	// makes that later division land it back at its true size instead of a
	// third of it.
	return absSumUpper(envelopeSlack, productUpper(3, sweptVolumeAllow(patchDeviation, areaUpper)))
}

// chordLocusLengthAllow bounds a cap-blend miter ruling's own chord-versus-
// locus excess (docs/modify-reach-design.md §8.3's boundary bullet): how far
// the denoted corner-foot locus's true length can exceed the built chord it
// is tagged `Line3` as, given a proven upper bound speedUpper on the locus's
// own in-plane speed |dP/dt| over the offset range [0, dc] and the locus's
// own EXACT axial speed |axialSpan|/dc — z is affine in the offset amount by
// construction (a fixed side level and a fixed cap level, ruled linearly),
// so that ratio needs no enclosure, only a division rounded up.
//
// The locus length is at most dc·sqrt(speedUpper² + (axialSpan/dc)²) — the
// range's own width times an upper bound on the 3D speed, radius2D's own
// √2-scaled bound on the two independently-bounded components — and a chord
// never exceeds the curve it subtends, so chordUpper (a PROVEN upper bound on
// the patch's own held chord) is itself a lower bound on the true locus
// length. The excess is that product minus chordUpper, clamped at zero (a
// negative reading proves nothing — the bound is loose there, not the locus
// short) and rounded up.
func chordLocusLengthAllow(speedUpper, dc, axialSpan, chordUpper float64) float64 {
	if speedUpper < 0 || dc <= 0 || isNonFinite(speedUpper) || isNonFinite(chordUpper) {
		return math.Inf(1)
	}
	rdc, raxial := floatRat(dc), floatRat(axialSpan)
	if rdc == nil || raxial == nil {
		return math.Inf(1)
	}
	zSpeed, exact := new(big.Rat).Quo(new(big.Rat).Abs(raxial), rdc).Float64()
	if !exact {
		zSpeed = math.Nextafter(zSpeed, math.Inf(1))
	}
	if isNonFinite(zSpeed) {
		return math.Inf(1)
	}
	locusUpper := productUpper(dc, radius2D(speedUpper, zSpeed))
	excess := locusUpper - chordUpper
	if excess <= 0 {
		return 0
	}
	return upRound(excess)
}

// sweptMomentAllow bounds the FIRST MOMENT a cap contour's own displacement
// can move — sweptVolumeAllow's own sibling one power higher
// (docs/modify-reach-design.md §8.4's fourth reading, beside the band
// volume, the chamfered cap face area, and each band patch's own area). The
// symmetric difference between the band the build holds and the one the
// offset denotes has volume at most sweptVolumeAllow(delta, areaUpper) — that
// identity is unchanged here — and every point of that difference lies within
// coordUpper of the plane-local origin (the fixed point every first-moment
// integral in this file is taken about), so the moment the difference can
// carry is at most that volume times coordUpper: a region of proven volume V
// all of whose points lie within coordUpper of the origin has |∫p dV| <=
// V·coordUpper for any single coordinate p, since |p| <= coordUpper
// pointwise. coordUpper must be a PROVEN upper bound on |u|, |v| and |z| over
// the band's own material — the band lies BETWEEN the original loop and the
// offset cap boundary, so both loops' own envelopes (extrude.go's
// profileCoordinateUpper) are needed, together with max(|z0|, |z1|) — the
// same envelope prismCentroidGeometryBound already forms for the axial
// levels.
func sweptMomentAllow(delta, areaUpper, coordUpper float64) float64 {
	vol := sweptVolumeAllow(delta, areaUpper)
	if vol <= 0 || coordUpper <= 0 {
		return 0
	}
	return productUpper(vol, coordUpper)
}

// chordedBoundaryMomentAllow is chordedBoundaryVolumeAllow's own one-
// dimension-higher sibling, the SAME relation sweptMomentAllow states for the
// vertex-displacement mechanism: a region of proven volume V all of whose
// points lie within coordUpper of the plane-local origin has |∫p dV| <=
// V·coordUpper for any single coordinate p, since |p| <= coordUpper
// pointwise. It is NOT sweptMomentAllow reused unchanged — that helper calls
// sweptVolumeAllow internally, which speaks for the vertex-displacement
// mechanism rather than this one — so this is its own three-line twin over
// the chorded volume term. coordUpper carries sweptMomentAllow's own
// obligation: a PROVEN upper bound on |u|, |v| and |z| over the boundary's
// own material.
func chordedBoundaryMomentAllow(sectionDelta, areaUpper, coordUpper float64) float64 {
	vol := chordedBoundaryVolumeAllow(sectionDelta, areaUpper)
	if vol <= 0 || coordUpper <= 0 {
		return 0
	}
	return productUpper(vol, coordUpper)
}

// boundedSqrt propagates a proven bound through a square root: x.bound must
// already prove the true operand lies in [x.value−x.bound, x.value+x.bound]
// (clamped at zero, since every caller's operand is a sum of squares or a
// disk radius). The result brackets the square root of that whole interval
// through ratSqrtDown/ratSqrtUp — the same rational sqrt brackets
// circularLengthInterval reads an ArcSeg's own radius through — evaluated at
// the interval's two ends, never through math.Sqrt's own accuracy on either.
// A non-finite operand bound answers +Inf: an absent bound must never read as
// a small one.
//
// The outward math.Nextafter step on each end is charged only where there is
// rounding for it to cover: x.value ± x.bound is itself a rounded float
// operation, so a genuinely bounded operand's computed ends can each sit an
// ulp inside the interval they stand for and MUST be stepped out. Adding or
// subtracting exactly zero rounds nothing, so a zero-bound operand's ends are
// already the exact interval — the held value twice — and stepping them out
// would invent a rounding error that provably did not occur. The rational
// brackets then decide the answer by exact comparison: a zero bound precisely
// when the held value is a perfect square of a float64, and a genuine
// directed-rounding bound whenever it is not.
func boundedSqrt(x boundedScalar) boundedScalar {
	value := math.Sqrt(math.Max(x.value, 0))
	if isNonFinite(x.bound) {
		return measuredScalar(value, math.Inf(1))
	}
	lo := math.Max(0, x.value-x.bound)
	hi := x.value + x.bound
	if x.bound != 0 {
		lo = math.Nextafter(lo, math.Inf(-1))
		if lo < 0 {
			lo = 0
		}
		hi = math.Nextafter(hi, math.Inf(1))
	}
	loR, hiR := floatRat(lo), floatRat(hi)
	if loR == nil || hiR == nil {
		return measuredScalar(value, math.Inf(1))
	}
	sqrtLo := ratSqrtDown(loR)
	sqrtHi := ratSqrtUp(hiR)
	if isNonFinite(sqrtLo) || isNonFinite(sqrtHi) {
		return measuredScalar(value, math.Inf(1))
	}
	bound := upRound(math.Max(value-sqrtLo, sqrtHi-value))
	return measuredScalar(value, bound)
}

// boundedNorm2 is boundedSqrt's own reduction of a 2D length, over two
// components that already carry their own proven bounds. It never routes
// through math.Hypot's undocumented accuracy: the sum of squares composes
// through the bounded arithmetic, and the square root's own rounding comes
// from boundedSqrt.
func boundedNorm2(x, y boundedScalar) boundedScalar {
	return boundedSqrt(boundedAdd(boundedMul(x, x), boundedMul(y, y)))
}

// boundedHypot is boundedNorm2 over two EXACT leaves — the convention a
// recorded coordinate takes throughout this package (line endpoints and
// normals, arc centres, junction vertices). A radius this evaluator COMPUTED
// is not one of them: it carries its own bound and must be passed through
// boundedNorm2 instead.
func boundedHypot(dx, dy float64) boundedScalar {
	return boundedNorm2(exactScalar(dx), exactScalar(dy))
}

// directionalPerturbationAllow bounds how far a linear functional's own
// extreme over a bounded region can move when the functional's direction is
// perturbed by dirBound: an extreme of gu·u + gv·v over a boundary is
// 1-Lipschitz in (gu, gv) against the boundary's own coordinate envelope,
// since |Δgu·u + Δgv·v| <= |(Δgu, Δgv)| · envelopeUpper by Cauchy-Schwarz.
//
// envelopeUpper must be a PROVEN upper bound on the norm of the very
// coordinate the perturbed direction multiplies, measured about the SAME
// origin the functional is written about. Which envelope that is belongs to
// the caller's own geometry, and the two are NOT interchangeable: a section
// read about its plane frame charges the profile's plane-local envelope
// (profileCoordinateUpper), while a revolve's swept radial coefficient
// multiplies the distance from the RESOLVED AXIS and so charges the axis's
// own radial envelope (axisFrame.radialUpper, which folds in the axis
// anchor). Handing this the frame-origin envelope for an axis-referred
// coordinate understates the result without limit as the two origins
// separate.
func directionalPerturbationAllow(dirBound, envelopeUpper float64) float64 {
	return productUpper(dirBound, envelopeUpper)
}

// pointPerturbationAllow is directionalPerturbationAllow's transpose: how far
// the value of the linear functional gu·u + gv·v can move when the POINT it
// reads carries a proven bound on each of its own components, rather than the
// direction carrying one. |gu·Δu + gv·Δv| ≤ |gu|·boundU + |gv|·boundV, summed
// outward.
//
// The two components enter SEPARATELY, each against the direction coefficient
// that actually multiplies it: a direction reading one axis alone charges
// nothing for the other axis's error, which is what keeps a whole circle's
// reading along u exact even though its own held endpoint misses in v
// (walkEndBound's own comment). boundU and boundV must be PROVEN bounds on the
// point's displacement from the one the record denotes — segmentWalk's
// startBound/endBound, the only producer of those numbers for a walked
// endpoint.
//
// A charged component answers zero exactly where its bound is zero or its
// coefficient is: a coordinate the record states verbatim, read through a
// direction this evaluator reads as an exact leaf, has no width for a directed
// rounding to invent (boundaryExtremesBoundedContext's own convention). A
// non-finite bound answers +Inf, never 0, so an absent bound never reads as a
// small one.
func pointPerturbationAllow(bound walkEndBound, gu, gv float64) float64 {
	if isNonFinite(bound.u) || isNonFinite(bound.v) {
		return math.Inf(1)
	}
	return absSumUpper(
		productUpper(math.Abs(gu), bound.u),
		productUpper(math.Abs(gv), bound.v),
	)
}

// boundedFloatError is the proven error bound of a HELD float64 against a
// bounded scalar that already encloses the quantity the float stands for:
// |held − true| ≤ |held − value| + bound, the first term measured exactly over
// the rationals (rationalFloatError) and the two summed outward.
//
// It exists because a producer may evaluate a quantity one way and PROVE it
// another, and the two spellings are then different floats. The circular
// boundary-extreme candidate is that shape: its held position runs through
// math.Cos/math.Sin at the candidate's own angle, while its enclosure comes
// from the angle-free apex identity circularExtremeInterval states. Composing
// the gap this way keeps the held reading exactly where it was — no consumer's
// value moves — while the published bound speaks for the truth.
//
// A non-finite operand answers +Inf, never 0: an absent bound must never read
// as a small one (cutDisplacementAllow's own rule).
func boundedFloatError(bs boundedScalar, held float64) float64 {
	if isNonFinite(bs.value) || isNonFinite(bs.bound) || isNonFinite(held) {
		return math.Inf(1)
	}
	return absSumUpper(rationalFloatError(floatRat(bs.value), held), bs.bound)
}

// rimDelta is the trim-amplified displacement bound of a vertex the boolean
// itself creates. A rim vertex is not a point of either operand's surface: it
// is the exact crossing of operand A's chord PLANE with operand B's, and the
// true intersection curve lies anywhere within deltaA of the one and deltaB
// of the other. That region is a tube of half-width (deltaA + deltaB)/sin θ
// about the crossing line — so the displacement grows without limit as the
// two surfaces approach tangency, and δ itself is NOT a bound on it.
//
// sinMin is the smallest sine of a crossing angle any contact of this pair
// takes, computed exactly from the facet normals. When the inflated bound
// reaches the pair's own diameter it has stopped bounding anything, and the
// operation is refused (ErrUnsupported) rather than reported with a bound
// nobody can use — decad never understates a bound, and never fakes one.
func rimDelta(deltaA, deltaB, sinMin, dPair float64) (float64, error) {
	d := deltaA + deltaB
	if d <= 0 {
		// Both operands are held exactly (all-planar analytic faces, or a
		// faceted body whose polygons ARE its boundary): there is no chord
		// error to amplify, at any crossing angle.
		return 0, nil
	}
	if sinMin <= 0 || isNonFinite(sinMin) {
		return 0, fmt.Errorf(`%w: the operands' facets meet at an angle this evaluator cannot bound`, ErrUnsupported)
	}
	rim := upRound(d / sinMin)
	if isNonFinite(rim) || rim >= dPair {
		return 0, fmt.Errorf(`%w: the operands cross too shallowly — the rim's proven displacement bound reaches the pair's own diameter, so no measurement of the result would be trustworthy`, ErrUnsupported)
	}
	return rim, nil
}

// exactIsometryDotRound is the tight companion to rigidRoundAllow: instead of
// a worst-case ulp estimate from an input MAGNITUDE, it proves |held − true|
// exactly, over the rationals, for one scalar coefficient of the form
// xform.Apply(pt).Dot(g) (translate true) or xform.ApplyDir(pt).Dot(g)
// (translate false) — the shape extrude.go's prism box and revolve.go's
// revolve box each lift a boundary or sweep extreme through, reading the
// frame and placement as an exact leaf. It is exactPrismPointRound's own
// rational chain (extrude.go), reduced to the single coefficient a
// directional reading needs rather than a whole placed point, and read
// through a caller-supplied g rather than fixed to a Vec's three components.
//
// held must be the SAME float the caller's own arithmetic produced —
// xform.Apply(pt).Dot(g) or xform.ApplyDir(pt).Dot(g), computed the ordinary
// way — so the comparison measures the rounding that arithmetic actually
// committed, never a different evaluation order's own. Every basis vector,
// pt and g component must itself be an exactly representable float64 (they
// always are here: a Transform's basis and translation, a payload's own
// frame vectors, and g one of the three world unit axes), or the answer is
// +Inf — an absent bound must never read as a small one
// (cutDisplacementAllow's own rule).
func exactIsometryDotRound(xform r3.Transform, pt, g r3.Vec, translate bool, held float64) float64 {
	ratOfVec := func(v r3.Vec) [3]*big.Rat {
		return [3]*big.Rat{floatRat(v.X), floatRat(v.Y), floatRat(v.Z)}
	}
	anyNil := func(r [3]*big.Rat) bool { return r[0] == nil || r[1] == nil || r[2] == nil }
	ptR, gR := ratOfVec(pt), ratOfVec(g)
	basis := xform.Basis()
	exR, eyR, ezR := ratOfVec(basis.EX), ratOfVec(basis.EY), ratOfVec(basis.EZ)
	if anyNil(ptR) || anyNil(gR) || anyNil(exR) || anyNil(eyR) || anyNil(ezR) {
		return math.Inf(1)
	}
	placed := [3]*big.Rat{}
	for i := range placed {
		placed[i] = ratAdd(ratMul(exR[i], ptR[0]), ratMul(eyR[i], ptR[1]), ratMul(ezR[i], ptR[2]))
	}
	if translate {
		trR := ratOfVec(xform.Translation())
		if anyNil(trR) {
			return math.Inf(1)
		}
		for i := range placed {
			placed[i] = ratAdd(placed[i], trR[i])
		}
	}
	exact := ratAdd(ratMul(placed[0], gR[0]), ratMul(placed[1], gR[1]), ratMul(placed[2], gR[2]))
	return rationalFloatError(exact, held)
}

// exactFrameLocalRound proves, over the rationals, the rounding ONE plane-local
// coordinate of frame.ToLocal(p) commits: |held − (p − frame.Origin())·axis|
// measured exactly, with axis the frame's OWN u or v. It is
// exactIsometryDotRound one transform earlier — that helper reads a placement's
// basis, this one a frame's — and it is what a plane-local anchor's proven
// bound is: the frame and the world point are exact leaves, so the projection's
// only error is what its own float arithmetic rounded away.
//
// held must be the SAME float the caller's own arithmetic produced —
// frame.ToLocal(p).X against frame.U(), .Y against frame.V() — so the
// comparison measures the rounding that call actually committed. The answer is
// zero exactly where that arithmetic is exact for the input at hand (an
// axis-aligned frame and an exactly representable anchor), and a component no
// rational holds answers +Inf, never 0 (cutDisplacementAllow's own rule).
func exactFrameLocalRound(frame r3.Frame, p, axis r3.Vec, held float64) float64 {
	ratOfVec := func(v r3.Vec) [3]*big.Rat {
		return [3]*big.Rat{floatRat(v.X), floatRat(v.Y), floatRat(v.Z)}
	}
	anyNil := func(r [3]*big.Rat) bool { return r[0] == nil || r[1] == nil || r[2] == nil }
	pR, axR, oR := ratOfVec(p), ratOfVec(axis), ratOfVec(frame.Origin())
	if anyNil(pR) || anyNil(axR) || anyNil(oR) {
		return math.Inf(1)
	}
	terms := [3]*big.Rat{}
	for i := range terms {
		terms[i] = ratMul(new(big.Rat).Sub(pR[i], oR[i]), axR[i])
	}
	return rationalFloatError(ratAdd(terms[0], terms[1], terms[2]), held)
}

// exactPlaneDotRound proves, over the rationals, the rounding the float64
// evaluation of a plane-local dot product gu·u + gv·v commits: the anchor shift
// a revolve's extreme reading subtracts to carry a plane-local extreme into
// axis coordinates (revolve.go's axisExtremeContext). Both products round at
// the ANCHOR's own magnitude, which the boundary scan's own bound — proven over
// the SECTION's coordinates — says nothing about.
//
// held must be the SAME float the caller's own arithmetic produced, so the
// answer covers however that arithmetic grouped its two products and their sum.
// It speaks for the ROUNDING alone: the u/v operands' own proven uncertainty is
// a different mechanism the caller composes beside it. A component no rational
// holds answers +Inf, never 0 (cutDisplacementAllow's own rule).
func exactPlaneDotRound(gu, gv, u, v, held float64) float64 {
	guR, gvR, uR, vR := floatRat(gu), floatRat(gv), floatRat(u), floatRat(v)
	if guR == nil || gvR == nil || uR == nil || vR == nil {
		return math.Inf(1)
	}
	return rationalFloatError(ratAdd(ratMul(guR, uR), ratMul(gvR, vR)), held)
}

// planeDotDecompositionRoundAllow bounds the rounding the boundary-extreme
// scan's OWN evaluation of gu·u + gv·v commits over every candidate it folds
// (extrude.go's boundaryExtremesBoundedContext), given gu and gv themselves
// already proven against the frame, axis and placement chain that produced them
// — the caller's own coefficient terms. It is exactPlaneDotRound scaled to a
// whole scan: that helper measures ONE stated (u, v) exactly, and the scan
// states none, because the candidate that wins the extremization is not a value
// the scan reports.
//
// The rounding it charges is the SECTION's, and no other term in a revolve's
// extent reading scales with that magnitude: the scan's own published bound
// speaks for each candidate's POSITIONAL displacement (pointPerturbationAllow,
// a circular candidate's enclosure, a free-form span's), which is zero for a
// coordinate the record states verbatim, while the anchor shift's dot rounds at
// the ANCHOR's magnitude alone. A section a million millimetres long under a
// placement-derived direction therefore rounds here and nowhere else.
//
// IEEE 754 multiplies exactly by 0, 1 and -1 for any operand, so a coefficient
// pair drawn from those three values WITH one of the two zero evaluates one
// exact product and adds it to zero: nothing rounds, whatever the coordinates,
// and the term is zero — which is what keeps an unplaced revolve about an
// axis-aligned frame reading its box exactly, however far its section reaches.
// Every other pair can round in each product and again in their sum, and the
// term is analyticRoundBound's own budget (two multiplies and one addition, far
// under its 128-operation contract) at the envelope of the very coordinates
// those products multiply.
//
// coordUpper must be a PROVEN upper bound on |u| and |v| over every candidate
// the scan folds — profileCoordinateEnvelope, which reads each walk's own
// coordUpper — measured about the SAME frame origin the scan's candidates are
// written about. extrude.go's prismDecompositionRoundAllow is the prism's own
// spelling of this mechanism, one sweep coordinate wider, and the two are never
// composed: each reading charges its own decomposition exactly once.
func planeDotDecompositionRoundAllow(gu, gv, coordUpper float64) float64 {
	trivial := func(c float64) bool { return c == 0 || c == 1 || c == -1 }
	if (gu == 0 || gv == 0) && trivial(gu) && trivial(gv) {
		return 0
	}
	return analyticRoundBound(absSumUpper(
		productUpper(math.Abs(gu), coordUpper),
		productUpper(math.Abs(gv), coordUpper),
	))
}

// exactSumRound proves, over the rationals, the rounding the FINAL float64
// summation of already-held terms commits: the recombination every directional
// extent reading publishes each of its two endpoints through — a prism's
// base + boundary extreme + sweep level, a revolve's or a cap-loop chamfer's
// base + boundary extreme (extrude.go, revolve.go and capblend.go's
// extentBoundedAlong).
//
// It is exactIsometryDotRound's companion one step later. That helper proves
// each COEFFICIENT the reading lifts an extreme through exactly right; this one
// charges what ADDING the resulting terms commits, and the two are independent:
// IEEE 754 multiplies exactly by 0, 1 and -1, so an axis-permuting frame under
// an axis-permuting placement leaves every coefficient exactly right and still
// rounds here, because a translation component added to a section coordinate is
// an ordinary float64 addition (10 + 0 is exact, 10 + 0.1 is not). A reading
// that charges only the coefficients therefore publishes a translated body's
// box as Exact with a zero bound while its own endpoint misses the truth.
//
// held must be the SAME float the caller's own arithmetic produced and terms
// the SAME summands it produced it from, in any order: the answer is
// |Σterms − held| measured exactly, so it covers every rounding that summation
// committed however the caller grouped it. It speaks for the summation ALONE —
// each term's own displacement is a different mechanism with its own helper
// above, and the caller composes the two outward, per endpoint.
//
// The answer is zero exactly where the summation is exactly representable, so a
// proven-exact reading — an unplaced body, or a placement whose translation
// lands on a float64 sum — keeps its zero bound and stays Exact. A term no
// rational holds answers +Inf, never 0 (cutDisplacementAllow's own rule).
func exactSumRound(held float64, terms ...float64) float64 {
	sum := new(big.Rat)
	for _, term := range terms {
		r := floatRat(term)
		if r == nil {
			return math.Inf(1)
		}
		sum.Add(sum, r)
	}
	return rationalFloatError(sum, held)
}

// snapToZeroAllow composes the bound a coordinate carries once a deliberate
// snap has overwritten it with exactly 0. bound is what the caller had already
// proven about the pre-snap coordinate and discarded is that coordinate's own
// magnitude, so bound + discarded encloses the same truth about the assigned
// zero: the truth sits within bound of the pre-snap value, which sits exactly
// discarded from 0.
//
// The composition is the whole point of routing through here rather than
// dropping the term. A snap is a decision the caller makes for its own reasons
// — revolve.go's axisFrame.walk snaps so contact classification and vertex
// placement agree exactly — and the assignment is no less an error for being
// deliberate. Charging it leaves the snap's own behaviour untouched while every
// reading that folds the snapped coordinate into a published measurement
// (survey.go's revolveMinRadius) stops claiming an exactness the assignment
// took away.
//
// It answers the caller's own bound unchanged for a discarded magnitude of
// zero, so a coordinate the arithmetic already put exactly on zero keeps
// whatever exactness it arrived with. A discarded magnitude no float can state
// answers +Inf rather than that unchanged bound: a NaN would be the ABSENCE of
// a charge and would silently vanish from every reading it was meant to widen
// (rigidRoundAllow's own rule).
func snapToZeroAllow(bound, discarded float64) float64 {
	if isNonFinite(discarded) {
		return math.Inf(1)
	}
	if discarded <= 0 {
		return bound
	}
	return upRound(bound + discarded)
}
