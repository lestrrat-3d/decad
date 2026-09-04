package decad

import (
	"fmt"
	"math"
	"math/big"
)

// This file is docs/tessellation-design.md §13's increment T4
// (docs/tessellation-reach-design.md §6, R5): the OCCUPIED-VOLUME proof a
// revolve mesh must carry before any boolean may consume it. §§8-10 prove the
// mesh itself — where its facets sit and how much area they hold — and this
// file proves the one further thing a boolean needs, which is how much VOLUME
// the mesh and the body it stands for can differ by.
//
// §11 forbids the obvious shortcut outright: Mesh.Bound × held area is NOT that
// proof. A two-sided Hausdorff bound does not bound occupied volume, because a
// torus's inner and outer walls move in OPPOSITE material senses and a
// doubly-curved cell can gain material where another loses it — the signed
// error cancels while the symmetric difference does not. So §11 instead walks
// the four explicit stages between the analytic body and the returned
// polyhedron and charges each of them separately:
//
//	B0 → BM   the meridian arcs replaced by their chords     Mmeridian
//	BM → BH   the angular direction chorded                  Σ_cells Icell
//	BH → BC   coordinate-construction rounding               Mconstruct
//	BC → BR   final placement rounding                       Mround
//
// The set triangle inequality composes them, and every term is an ABSOLUTE
// swept volume, so nothing cancels between stages, between loops, or between a
// hole and the outline it sits in.
//
// Two structural facts make the middle term cheap:
//
//   - BM's meridian is a POLYLINE. Every circular generator has already been
//     replaced by its chords at this stage, so §11's straight-generator
//     homotopy H answers for EVERY cell of a sphere or a torus too; the
//     curvature it dropped is charged in full by Mmeridian, one stage earlier.
//   - The angular factor of Icell does not depend on the cell. Rotating a cell
//     about the axis is an isometry, so the same reading answers for every
//     angular interval, and separating the meridian direction out of the triple
//     integral (below) leaves a factor that depends on dφ ALONE. It is
//     therefore proven once per mesh rather than once per cell.

// revolveAngularIntegralSteps is the fixed certified-subdivision budget §11's
// angular homotopy integral spends in the u direction.
//
// It is a separate constant from revolveArcIntegralSteps because it is spent on
// a different scale: an Ecell reading is taken per CELL, while this one is taken
// ONCE for the whole mesh (revolveAngularHomotopyFactor's own doc comment says
// why), so a deeper cut costs nothing per facet and the budget is spent where it
// buys the most. Like that one it is fixed rather than adaptive, and for the
// same reason: every piece's contribution is already an upper bound at any
// depth, so depth buys tightness and never soundness.
const revolveAngularIntegralSteps = 64

// revolveAngularHomotopyFactor proves the ANGULAR factor of
// docs/tessellation-design.md §11's per-cell triple integral: the part of
//
//	Icell ≥ ∫_[0,1]³ |∂H/∂λ · (∂H/∂t × ∂H/∂u)| d(λ, t, u)
//
// that survives once the meridian direction has been integrated out. It depends
// on the angular step dφ alone, so one reading answers for every cell of the
// mesh and for every angular interval of each of them.
//
// The separation is exact, not an approximation. Write the homotopy §11 states
// in the axis basis (w, e0, e1), with e(φ) = cos φ·e0 + sin φ·e1, A(u) =
// e(φ0 + u·dφ) the rotated point, B0 = e(φ0), B1 = e(φ1) and C(u) =
// (1−u)·B0 + u·B1 the chord point:
//
//	H(λ, t, u) = a3 + z(t)·w + ρ(t)·((1−λ)·A(u) + λ·C(u))
//
// with z and ρ AFFINE in t along the meridian chord. Then ∂H/∂λ = ρ·(C − A) and
// ∂H/∂u = ρ·∂E/∂u are both in the (e0, e1) plane while ∂H/∂t = z'·w + ρ'·E, so
// the ρ' term of the cross product points along w and the ∂H/∂λ dot kills it.
// What is left is
//
//	∂H/∂λ · (∂H/∂t × ∂H/∂u) = z' · ρ(t)² · (Eu ∧ (C − A))
//
// where ∧ is the in-plane scalar cross product and Eu = ∂E/∂u. The t direction
// is therefore a closed-form rational factor the caller owns
// (revolveCellSweptVolume), and everything else is this function's:
//
//	Eu ∧ (C − A) = (1−λ)·P(u) + λ·Q(u)
//	P(u) = dφ·((1−u)·(1 − cos(u·dφ)) + u·(1 − cos((1−u)·dφ)))
//	Q(u) = sin(u·dφ)·(1 − cos dφ) − sin(dφ)·(1 − cos(u·dφ))
//
// P is written in that grouped form deliberately: the raw expression
// 1 − (1−u)cos(u·dφ) − u·cos((1−u)·dφ) is a difference of order-one terms whose
// value is of order dφ², and the grouping makes every term non-negative and the
// cancellation vanish. Both are ODD in dφ, so |·| is unchanged by the sweep
// sense and the reading is taken at |dφ|.
//
// The λ direction then integrates in closed form and exactly, by the triangle
// inequality on a CONVEX COMBINATION:
//
//	∫₀¹ |(1−λ)·P + λ·Q| dλ ≤ ∫₀¹ ((1−λ)·|P| + λ·|Q|) dλ = (|P| + |Q|) / 2
//
// so the whole reading is (∫|P| + ∫|Q|)/2. Neither of those two has a closed
// form — the zeros are arcsines — so each takes
// docs/tessellation-design.md §15's other admissible path, CERTIFIED INTERVAL
// SUBDIVISION: [0, 1] is cut into revolveAngularIntegralSteps equal pieces, both
// functions are enclosed at every NODE of that cut, and one piece is charged the
// larger of its two nodes' magnitudes plus the proven second-order allowance
// angularHomotopyBulges states. Because the absolute value is taken inside every
// piece, nothing cancels between pieces and the sum is an upper bound at any
// depth.
//
// The nodes are read, rather than a whole piece being enclosed at once, for the
// reason revArcCell.rhoNodes gives for its own subdivision, and the reason
// binds harder here: P is of order dφ³ while a piece-wide radSinCosSpan widens
// by the piece's own width, of order dφ/N. A whole-piece enclosure is therefore
// not merely loose, it stops converging — the node reading plus a second-order
// term is what keeps a fixed budget worth spending.
//
// Nothing here calls math.Sin or math.Cos, and nothing here compares against π:
// radSinCosSpan reduces through moments_trig.go's own certified series.
func revolveAngularHomotopyFactor(step ratInterval) (*big.Rat, error) {
	d := intervalAbsSpan(step)
	if d.lo.Sign() < 0 || d.hi.Cmp(d.lo) < 0 {
		return nil, errRevolveAngularHomotopy
	}
	one := pointInterval(big.NewRat(1, 1))
	sinD, cosD, ok := radSinCosSpan(d)
	if !ok {
		return nil, errRevolveAngularHomotopy
	}
	versD := intervalSub(one, cosD)

	n := int64(revolveAngularIntegralSteps)
	pAt := make([]*big.Rat, n+1)
	qAt := make([]*big.Rat, n+1)
	for i := int64(0); i <= n; i++ {
		u := big.NewRat(i, n)
		co := new(big.Rat).Sub(big.NewRat(1, 1), u)
		sinA, cosA, okA := radSinCosSpan(intervalScale(d, u))
		_, cosB, okB := radSinCosSpan(intervalScale(d, co))
		if !okA || !okB {
			return nil, errRevolveAngularHomotopy
		}
		versA := intervalSub(one, cosA)
		p := intervalMul(d, intervalAdd(
			intervalScale(versA, co),
			intervalScale(intervalSub(one, cosB), u),
		))
		q := intervalSub(intervalMul(sinA, versD), intervalMul(sinD, versA))
		pAt[i], qAt[i] = intervalAbsUpper(p), intervalAbsUpper(q)
	}

	bulgeP, bulgeQ := angularHomotopyBulges(d, n)
	total := new(big.Rat)
	width := big.NewRat(1, n)
	for i := range n {
		piece := new(big.Rat).Add(ratMax(pAt[i], pAt[i+1]), bulgeP)
		piece.Add(piece, new(big.Rat).Add(ratMax(qAt[i], qAt[i+1]), bulgeQ))
		total.Add(total, new(big.Rat).Mul(piece, width))
	}
	return total.Mul(total, big.NewRat(1, 2)), nil
}

// angularHomotopyBulges states the per-piece second-order allowance
// revolveAngularHomotopyFactor charges beside each piece's two node readings,
// one for P and one for Q.
//
// The argument is elementary and needs no monotonicity: a twice-differentiable f
// departs from the chord through its own two endpoints by at most
// max|f”|·h²/8, so |f| over a piece of width h = 1/N is at most the larger of
// its endpoint magnitudes plus that. What is left is a bound on each second
// derivative, and both differentiate in closed form. Writing
// g(u) = (1−u)·(1 − cos(u·dφ)), so that P = dφ·(g(u) + g(1−u)):
//
//	g”(u) = −2·dφ·sin(u·dφ) + (1−u)·dφ²·cos(u·dφ)
//	Q”(u) = −dφ²·sin(u·dφ)·(1 − cos dφ) − dφ²·sin(dφ)·cos(u·dφ)
//
// With |cos| ≤ 1, |sin x| ≤ min(1, |x|) and |1 − cos x| ≤ min(2, x²/2) — the
// elementary bounds, not derived ones — and |u·dφ| ≤ dφ throughout:
//
//	|P”| ≤ 2·dφ·(2·dφ·S + dφ²)      |Q”| ≤ dφ²·S·(V + 1)
//
// for S = min(1, dφ) and V = min(2, dφ²/2). Both S bounds matter: taking S = 1
// alone would charge dφ² where the truth is dφ³, and the allowance would stop
// falling with the step while everything it sits beside kept falling.
func angularHomotopyBulges(d ratInterval, n int64) (*big.Rat, *big.Rat) {
	dh := new(big.Rat).Set(d.hi)
	sq := new(big.Rat).Mul(dh, dh)
	sinB := ratMin(big.NewRat(1, 1), dh)
	versB := ratMin(big.NewRat(2, 1), new(big.Rat).Mul(sq, big.NewRat(1, 2)))

	p := new(big.Rat).Mul(new(big.Rat).Mul(big.NewRat(2, 1), dh), new(big.Rat).Add(
		new(big.Rat).Mul(new(big.Rat).Mul(big.NewRat(2, 1), dh), sinB),
		sq,
	))
	q := new(big.Rat).Mul(new(big.Rat).Mul(sq, sinB), new(big.Rat).Add(versB, big.NewRat(1, 1)))
	denom := new(big.Rat).SetInt64(8 * n * n)
	return new(big.Rat).Quo(p, denom), new(big.Rat).Quo(q, denom)
}

var errRevolveAngularHomotopy = fmt.Errorf(`%w: a revolve cell's angular homotopy states no enclosure of the volume it sweeps, so this mesh can prove no occupied-volume bound`, ErrUnsupported)

// revolveCellSweptVolume is docs/tessellation-design.md §11's Icell for ONE
// meridian cell at ONE angular interval: the meridian factor of the triple
// integral revolveAngularHomotopyFactor separated out, times that shared
// angular factor.
//
// With z and ρ affine along the meridian chord, the integrand's t dependence is
// |z'|·ρ(t)² and both pieces are exact rationals in the cell's own enclosed
// endpoints:
//
//	∫₀¹ ρ(t)² dt = (ρ0² + ρ0·ρ1 + ρ1²) / 3
//
// A cell with one ring ON the axis needs no separate statement: ρ0 = 0 makes
// H's λ = 1 surface the fan triangle the mesh actually holds, exactly as the
// λ = 1 surface of an off-axis cell is its planar quad — the four corners of
// that quad are coplanar (their triple product carries a D ∧ D factor), so the
// bilinear patch H(1, ·, ·) IS the two held facets and no fourth stage exists
// between them.
//
// The enclosures are read rather than the stored floats because §11's homotopy
// runs between IDEAL-coordinate surfaces; what the stored floats cost is
// Mconstruct's and Mround's business, one stage later.
func revolveCellSweptVolume(lo, hi revMeridian, angular *big.Rat) *big.Rat {
	third := big.NewRat(1, 3)
	quad := intervalScale(intervalAdd(
		intervalAdd(intervalSquare(lo.rhoIv), intervalSquare(hi.rhoIv)),
		intervalMul(lo.rhoIv, hi.rhoIv),
	), third)
	axial := intervalAbsUpper(intervalSub(hi.zIv, lo.zIv))
	return new(big.Rat).Mul(new(big.Rat).Mul(axial, intervalAbsUpper(quad)), angular)
}

// revolveMeridianMoment is docs/tessellation-design.md §11's Mmeridian: the
// volume between the analytic body B0 and the body BM whose every circular
// meridian subarc has been replaced by its chord.
//
//	Mmeridian = sweepAngle · Σ_c |∫_{S_c} ρ dA|
//
// The two bodies differ, in the (z, ρ) half plane, by exactly the circular
// segments S_c between each arc and its chords, and revolving a region of that
// half plane through an angle Φ occupies Φ·∫ ρ dA of volume — Pappus, valid
// because ρ ≥ 0 keeps the region off the far side of the axis. The absolute
// value is taken per sliver, so a hole that GAINS material is charged the same
// sign as an outline that loses it and nothing cancels across loops.
//
// Each walk's slivers are bounded by their own total area times an upper bound
// on ρ over them: chordSegmentArea already proves the first (with no trig call
// and no library ulp assumption), and the second is the largest ρ the walk's own
// endpoints and enclosed cardinal points reach — a sliver lies between its arc
// and the chord joining two points of that arc, so it reaches no farther from
// the axis than the arc does.
func revolveMeridianMoment(p *revolvePlan) float64 {
	total := 0.0
	for li, r := range p.resolved {
		for k, w := range r.walks {
			if !w.isCircular() {
				continue
			}
			area := chordSegmentArea(w.radius, math.Abs(w.th1-w.th0), p.counts[li][k])
			rho := 0.0
			for _, pt := range revolveWalkExtremes(w.segmentWalk) {
				rho = math.Max(rho, pt[1])
			}
			total = absSumUpper(total, productUpper(area, rho))
		}
	}
	if total == 0 {
		return 0
	}
	return productUpper(revolveSweepUpper(p), total)
}

// revolveSweepUpper is an upward-rounded bound on the swept angle Mmeridian
// multiplies by. A full turn reads the in-tree π bracket rather than
// math.Pi — the constant is the nearest float to π and may sit below it, which
// is the wrong side for a bound — and a partial sweep rounds its own float
// subtraction up by one ulp, which covers that subtraction's whole rounding.
func revolveSweepUpper(p *revolvePlan) float64 {
	if p.rp.full {
		return ratFloatUp(new(big.Rat).Mul(big.NewRat(2, 1), piUpper))
	}
	return upRound(p.sweep)
}

// revolveSymDiff composes docs/tessellation-design.md §11's four stages into the
// mesh's volSymDiff.
//
//	volSymDiff = upRound(Mmeridian + Σ_cells Icell + Mconstruct + Mround)
//
// angular is the already-summed Σ_cells Icell, in exact rationals, so the one
// float rounding it takes is the conversion here. The two coordinate stages are
// swept-volume allowances in the shape §11 names: a boundary point moving at
// speed at most delta can displace volume no faster than the area it sweeps
// allows, and perturbedAreaUpper covers every surface on the stage's path, not
// only its two ends. Both stages read the composed displacement as their area
// argument because BH sits within deltaC + deltaR of the returned mesh and BC
// within deltaR of it, so one area bound at the composed figure covers every
// intermediate surface of both paths.
//
// Every leg is an absolute occupied-volume charge; none of them may cancel
// another, which is why they compose through absSumUpper rather than a signed
// sum.
func revolveSymDiff(m *Mesh, p *revolvePlan, angular *big.Rat, deltaC, deltaR float64) (float64, error) {
	if angular == nil || angular.Sign() < 0 {
		return 0, errRevolveAngularHomotopy
	}
	coord := absSumUpper(deltaC, deltaR)
	area := perturbedAreaUpper(m.vertices, m.triangles, coord)
	sym := absSumUpper(
		revolveMeridianMoment(p),
		ratFloatUp(angular),
		sweptVolumeAllow(deltaC, area),
		sweptVolumeAllow(deltaR, area),
	)
	if isNonFinite(sym) || sym < 0 {
		return 0, fmt.Errorf(`%w: this revolve mesh states no finite bound on the volume it and the body it stands for differ by`, ErrUnsupported)
	}
	return upRound(sym), nil
}
