package decad

import (
	"context"
	"fmt"
	"math"

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
//   - the AREA a 2D boundary displacement sweeps out → sectionDisplacementArea,
//     the same identity one dimension down: the region a recorded section can
//     move is a tube about its own recorded boundary, with
//     sectionDisplacementLength reading the same displacement as a perimeter;
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
//   - a per-coordinate maximum read as a 3D DISTANCE → radius3D.

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
