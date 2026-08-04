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

// rigidRoundAllow bounds the rounding a rigid motion commits on one point.
// The rounding happens INSIDE the products and sums — at the magnitude of the
// INPUT coordinate and of the translation — not at the magnitude of the
// result: a body built far from the origin and moved back rounds at the far
// magnitude and would be charged at the near one. Every intermediate stays
// under 2·maxInputAbs + maxTransAbs (an orthonormal row's dot product is at
// most √3·|input|), 16 ulps there cover the products, the two sums and the
// translation, and the consumers read a 3D distance, so ×√3.
func rigidRoundAllow(maxInputAbs, maxTransAbs float64) float64 {
	m := 2*math.Abs(maxInputAbs) + math.Abs(maxTransAbs)
	return radius3D(16 * ulpOf(m))
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
		total += u.Cross(v).Len()/2 + delta*(u.Len()+v.Len()) + 2*delta*delta
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
// within delta of the point it denotes while its side-level directrix is
// exact (docs/prism-boolean-design.md §7's identity, one ruled patch at a
// time rather than one whole section).
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
