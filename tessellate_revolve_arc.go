package decad

import (
	"fmt"
	"math"
	"math/big"
)

// This file is docs/tessellation-design.md §13's increment T3
// (docs/tessellation-reach-design.md §6, R4): the CIRCULAR meridian generator —
// a sphere or a torus wall — as an extension of the straight-generator cells
// tessellate_revolve.go assembles and tessellate_revolve_proof.go proves.
//
// It owns three things a straight generator never needs:
//
//   - The meridian STATIONS. A circular walk is chorded along the meridian, so
//     the mesh carries samples between its two recorded junctions. Each one is
//     enclosed at the recorded parameter it denotes and STORED as the float
//     nearest that enclosure's midpoint — never a math.Cos/math.Sin evaluation,
//     which Go states no ulp contract for — so the station's construction gap
//     is a fact about this build rather than an assumption about a library.
//   - The cell's own §10.2 Ecell. A straight generator's true area density is
//     affine in the meridian parameter, which is what let R3 decompose the sign
//     in closed form (tess §15). A circular generator's is not: with the arc
//     matched to its chord by the shared parameter, ρ(t) = cV + r·sin(θ0 + t·Δθ)
//     and the difference against the flat facet's constant density has no
//     rational root. So this file takes tess §15's OTHER admissible path,
//     CERTIFIED INTERVAL SUBDIVISION under a fixed budget: the unit interval is
//     cut into a fixed number of pieces, every node encloses the integrand
//     through a certified radian sine enclosure, and each piece contributes an
//     outward-rounded ABSOLUTE integral bound built from its own two nodes plus
//     a proven second-order allowance. The absolute value is inside the sum, so
//     nothing cancels between pieces and a later boolean retaining one sign
//     lobe is still bounded by the whole.
//   - The proven circular-segment area a partial cap's curved trim omits.
//
// Nothing here samples anything to decide anything, and nothing here calls a
// library trig function to publish a bound.

// revArcCell is the meridian model of ONE circular chord: the axis-coordinate
// circle the payload's own walk states, restricted to that chord's own angular
// sub-range and matched to the chord by the shared parameter t ∈ [0, 1].
//
//	z(t)   = cU + r·cos(θ0 + t·Δθ)
//	ρ(t)   = cV + r·sin(θ0 + t·Δθ)
//
// Every field is an exact rational read from a float the payload holds, so the
// model itself commits no rounding. It is the payload's own AXIS-coordinate
// circle rather than the exact axis image of the recorded curve, which is the
// same reading a straight cell takes when it measures its meridian length from
// the held samples (revolveCellAreaSlack). The gap between the two is a
// coordinate-construction displacement bounded by deltaC, and it enters twice:
// the ρ enclosure below is WIDENED by it, and §10.2's per-triangle
// coordinate-stage allowance charges it again over the whole mesh.
type revArcCell struct {
	cV, radius, th0, dth *big.Rat
}

// revolveArcChordCell builds the model of chord k of a circular axis walk
// divided into n chords. th0/th1 on an axis walk are the plane walk's own
// angles shifted by the axis rotation, so their difference is the recorded
// sweep and chord k spans [θ0 + k·Δθ, θ0 + (k+1)·Δθ] exactly.
func revolveArcChordCell(w segmentWalk, k, n int) (*revArcCell, bool) {
	cV, radius := floatRat(w.cV), floatRat(w.radius)
	th0, th1 := floatRat(w.th0), floatRat(w.th1)
	if cV == nil || radius == nil || th0 == nil || th1 == nil || n <= 0 || radius.Sign() < 0 {
		return nil, false
	}
	dth := new(big.Rat).Quo(new(big.Rat).Sub(th1, th0), new(big.Rat).SetInt64(int64(n)))
	start := new(big.Rat).Add(th0, new(big.Rat).Mul(dth, new(big.Rat).SetInt64(int64(k))))
	return &revArcCell{cV: cV, radius: radius, th0: start, dth: dth}, true
}

// speed is |dγ/dt| for the matched parameterisation, r·|Δθ|, which is CONSTANT
// on a circular arc — the one simplification a curved generator does give.
func (c revArcCell) speed() *big.Rat {
	return new(big.Rat).Mul(c.radius, new(big.Rat).Abs(new(big.Rat).Set(c.dth)))
}

// rhoNodes encloses ρ at each node of the fixed subdivision, and states the
// per-piece second-order allowance the integral below charges beside them.
//
// Nothing here compares against π: radSinCosInterval encloses each node's
// radian angle through moments_trig.go's own series.
//
// The allowance is elementary and needs no monotonicity argument. Over one
// piece of width h in t, the integrand's own second derivative is
// scale·r·Δθ²·(−sin), so its magnitude is at most scale·r·Δθ²; a
// twice-differentiable function departs from the chord through its own two
// endpoints by at most max|f”|·h²/8. This returns the factor r·Δθ²/(8·N²)
// with N the step count, and the caller multiplies by its own scale. Reading
// the NODES and charging that term is what makes the fixed budget worth
// spending: a whole-span enclosure would instead widen by the first-order h,
// which is larger by N·8/Δθ at every depth.
func (c revArcCell) rhoNodes() ([]ratInterval, *big.Rat, bool) {
	nodes := make([]ratInterval, revolveArcIntegralSteps+1)
	for i := range revolveArcIntegralSteps + 1 {
		t := big.NewRat(int64(i), revolveArcIntegralSteps)
		angle := new(big.Rat).Add(c.th0, new(big.Rat).Mul(c.dth, t))
		sin, _, ok := radSinCosInterval(angle)
		if !ok {
			return nil, nil, false
		}
		nodes[i] = intervalAdd(pointInterval(c.cV), intervalScale(sin, c.radius))
	}
	steps := big.NewRat(revolveArcIntegralSteps, 1)
	bulge := new(big.Rat).Quo(
		new(big.Rat).Mul(c.radius, new(big.Rat).Mul(c.dth, c.dth)),
		new(big.Rat).Mul(big.NewRat(8, 1), new(big.Rat).Mul(steps, steps)),
	)
	return nodes, bulge, true
}

// revolveArcIntegralSteps is the fixed certified-subdivision budget one
// circular cell's Ecell spends: the common parameter domain's meridian
// direction is cut into this many equal pieces, and every piece is enclosed and
// charged. It is fixed rather than adaptive because tess §10.2 asks for a
// bound under a SHARED FIXED budget, and because the bound each piece
// contributes is already valid at any depth — depth buys tightness, never
// soundness. Thirty-two pieces leave the reading within a few percent of the
// integral it bounds on an ordinary cell, and the second-order allowance
// rhoNodes charges beside them falls as 1/N², so the budget is spent where it
// buys the most.
const revolveArcIntegralSteps = 32

// revolveArcCellSlack is docs/tessellation-design.md §10.2's Ecell for one wall
// cell of a CIRCULAR generator, by certified interval subdivision — tess §15's
// second admissible path, and the one T3 takes because the first does not
// exist here.
//
// Over the common domain (t, u) ∈ [0,1]², with t along the meridian chord and u
// across one angular interval, the true patch is
//
//	Ftrue(t, u) = a3 + z(t)·w + ρ(t)·e(φ0 + u·dφ)
//
// and the arc matched to its chord by t has CONSTANT speed r·|Δθ|, so
//
//	Jtrue(t, u) = r·|Δθ|·dφ·ρ(t)
//
// which does not depend on u — the one thing the straight case and this one
// share. The held facet is flat, so Jheld is the constant twice-area on each
// half of the domain the fixed diagonal cuts, exactly as in the straight case.
// What is NOT shared is the difference's shape: ρ(t) is a sinusoid in t, so
// Jtrue − Jheld has no rational root and no closed-form sign decomposition.
//
// So each of the two half-domains is cut into revolveArcIntegralSteps pieces.
// Each NODE of that subdivision encloses ρ through a certified radian sine
// enclosure, which never compares against π, and one piece's |Jtrue − Jheld| is
// at most the larger of its two nodes' magnitudes plus the proven second-order
// chord allowance (rhoNodes). Multiplying by the exact ∫_a^b w(t) dt of the
// half's own weight and summing gives an upper bound on ∫|Jtrue − Jheld| that
// never cancels: the absolute value is taken inside every piece, so a cell
// whose Jacobian error changes sign — the inner wall of a torus does — is
// charged the sum of both lobes rather than their difference, which is what a
// later boolean retaining one lobe needs.
//
// step encloses dφ, twoArea encloses twice each ideal half-triangle's area in
// the order (diagonal-low half, diagonal-high half), and slack is the proven
// departure of this model's ρ from the meridian the record denotes.
func revolveArcCellSlack(cell revArcCell, step ratInterval, twoArea [2]ratInterval, slack float64) (float64, error) {
	scale, rho, extra, ok := revolveArcScale(cell, step, slack)
	if !ok {
		return 0, errRevolveArcCellSlack
	}
	total := new(big.Rat)
	zero := pointInterval(new(big.Rat))
	for half, weight := range [2]int{revolveWeightT, revolveWeightOneMinusT} {
		total.Add(total, revolveArcAbsIntegral(rho, scale, twoArea[half], zero, extra, weight))
	}
	return ratFloatUp(total), nil
}

// revolveArcFanSlack is revolveArcCellSlack for a circular cell with ONE ring
// on the axis — a sphere's polar cell. The held facets are a fan of single
// triangles over the whole unit square with the pole edge collapsed, so
// Jheld = 2A·t for a pole at t = 0 and 2A·(1 − t) for a pole at t = 1, while
// Jtrue keeps the same sinusoidal ρ(t) the quad case has. The subdivision is
// therefore identical with a LINEAR held density in place of a constant one.
func revolveArcFanSlack(cell revArcCell, poleFirst bool, step, twoArea ratInterval, slack float64) (float64, error) {
	scale, rho, extra, ok := revolveArcScale(cell, step, slack)
	if !ok {
		return 0, errRevolveArcCellSlack
	}
	held, slope := pointInterval(new(big.Rat)), twoArea
	if !poleFirst {
		held, slope = twoArea, intervalNeg(twoArea)
	}
	return ratFloatUp(revolveArcAbsIntegral(rho, scale, held, slope, extra, revolveWeightOne)), nil
}

// revolveArcScale composes the non-negative factor r·|Δθ|·|dφ| every circular
// cell's true density carries, beside the cell's own node ρ enclosures and the
// per-piece allowance charged on top of them. The sweep interval's own sign
// never enters: the density is a MAGNITUDE, and a sweep run in the opposed
// direction would otherwise be charged its own magnitude twice over.
//
// The allowance composes two independent terms, both scaled by the density's
// own factor: rhoNodes' second-order chord term, which is about the model's own
// curvature between two nodes, and the model SLACK, which is how far the true
// meridian may sit from the model at all. The slack is kept out of the
// curvature argument because a displacement of the true meridian is not
// required to be smooth.
func revolveArcScale(cell revArcCell, step ratInterval, slack float64) (ratInterval, []ratInterval, *big.Rat, bool) {
	s := floatRat(slack)
	if s == nil || s.Sign() < 0 || cell.radius.Sign() < 0 {
		return ratInterval{}, nil, nil, false
	}
	rho, bulge, ok := cell.rhoNodes()
	if !ok {
		return ratInterval{}, nil, nil, false
	}
	scale := intervalScale(intervalAbsSpan(step), cell.speed())
	extra := new(big.Rat).Mul(scale.hi, new(big.Rat).Add(bulge, s))
	return scale, rho, extra, true
}

var errRevolveArcCellSlack = fmt.Errorf(`%w: a circular revolve cell states no enclosure of the area its held facets and the patch they stand for differ by`, ErrUnsupported)

// revolveArcAbsIntegral bounds ∫₀¹ |scale·ρ(t) − (held + slope·t)|·w(t) dt
// upward over the fixed subdivision whose NODE ρ enclosures the caller holds.
// One piece is charged the larger of its two nodes' magnitudes plus the
// caller's own allowance, times the exact integral of the weight over that
// piece, so the answer is an upper bound at any depth and nothing cancels
// between pieces.
func revolveArcAbsIntegral(rho []ratInterval, scale, held, slope ratInterval, extra *big.Rat, weight int) *big.Rat {
	at := func(i int) *big.Rat {
		t := big.NewRat(int64(i), revolveArcIntegralSteps)
		f := intervalSub(intervalMul(scale, rho[i]), intervalAdd(held, intervalScale(slope, t)))
		return intervalAbsUpper(f)
	}
	total := new(big.Rat)
	prev := at(0)
	for i := range revolveArcIntegralSteps {
		next := at(i + 1)
		a := big.NewRat(int64(i), revolveArcIntegralSteps)
		b := big.NewRat(int64(i+1), revolveArcIntegralSteps)
		piece := new(big.Rat).Add(ratMax(prev, next), extra)
		total.Add(total, new(big.Rat).Mul(piece, revolveWeightIntegral(a, b, weight)))
		prev = next
	}
	return total
}

// revolveWeightIntegral is the exact ∫_a^b w(t) dt of the three weights
// absLinearIntegral integrates against, so the subdivision charges each piece
// the measure its own half-domain gives it.
func revolveWeightIntegral(a, b *big.Rat, weight int) *big.Rat {
	width := new(big.Rat).Sub(b, a)
	half := new(big.Rat).Mul(
		new(big.Rat).Sub(new(big.Rat).Mul(b, b), new(big.Rat).Mul(a, a)),
		big.NewRat(1, 2),
	)
	switch weight {
	case revolveWeightT:
		return half
	case revolveWeightOneMinusT:
		return new(big.Rat).Sub(width, half)
	default:
		return width
	}
}

// intervalAbsSpan is the enclosure of |x| for x in the given enclosure.
func intervalAbsSpan(a ratInterval) ratInterval {
	if a.lo.Sign() >= 0 {
		return a
	}
	if a.hi.Sign() <= 0 {
		return intervalNeg(a)
	}
	return interval(new(big.Rat), intervalAbsUpper(a))
}

// revolveArcStation is one interior meridian sample of a circular walk: the
// axis coordinates the RECORD denotes at station k of n, enclosed exactly, and
// the float pair this mesh stores for it.
//
// The station's recorded parameter is TStart + (k/n)·(TEnd − TStart), taken as
// an EXACT rational — rounding it to a float first would enclose the recorded
// curve at a neighbouring parameter and prove a bound about a point this
// chording never named (chordStationBound's own rule). The plane-local
// enclosure is circularEndpointInterval's, and axisCoordInterval carries it
// into (z, ρ) through the payload's own axis frame with no rounding.
//
// The stored pair is the float NEAREST the enclosure's midpoint. That is what
// makes the station's construction gap a fact about this build: it is at most
// the enclosure's own width plus half an ulp, both of which the tolerance split
// reserved a-priori (revolveStationGapPrior) and the caller measures and holds
// to account. Nothing here calls math.Cos or math.Sin.
//
// A station the record cannot enclose refuses. A station the arithmetic puts
// ON or BEYOND the axis is ErrDegenerate rather than a pole: a generator that
// meets the axis at an interior point sweeps no manifold solid, and §12 forbids
// rounding a near-axis ring onto the axis to make one.
func revolveArcStation(ax axisFrame, seg CurveSegment, k, n int) (revMeridian, float64, error) {
	seg, err := normalizeSegment(seg)
	if err != nil {
		return revMeridian{}, 0, err
	}
	start, span, ok := circularSegmentRange(seg)
	if !ok || n <= 0 || k <= 0 || k >= n {
		return revMeridian{}, 0, errRevolveStationEnclosure
	}
	frac := new(big.Rat).SetFrac64(int64(k), int64(n))
	rt := new(big.Rat).Add(start, new(big.Rat).Mul(frac, span))
	uIv, vIv, ok := circularEndpointInterval(seg, rt)
	if !ok {
		return revMeridian{}, 0, errRevolveStationEnclosure
	}
	zIv, rhoIv, ok := axisCoordInterval(ax, uIv, vIv)
	if !ok {
		return revMeridian{}, 0, errRevolveStationEnclosure
	}
	z, _ := intervalMid(zIv).Float64()
	rho, _ := intervalMid(rhoIv).Float64()
	gap := math.Max(intervalFloatError(zIv, z), intervalFloatError(rhoIv, rho))
	if isNonFinite(z) || isNonFinite(rho) || isNonFinite(gap) {
		return revMeridian{}, 0, errRevolveStationEnclosure
	}
	if rho <= 0 {
		return revMeridian{}, 0, fmt.Errorf(`%w: a revolve meridian chord station lands on the axis or across it, so the recorded generator sweeps no manifold solid there`, ErrDegenerate)
	}
	return revMeridian{z: z, rho: rho, zIv: zIv, rhoIv: rhoIv}, gap, nil
}

var errRevolveStationEnclosure = fmt.Errorf(`%w: a revolve meridian chord station states no enclosure of the axis coordinates its record denotes`, ErrUnsupported)

// chordSegmentArea is a PROVEN upper bound on the total area of the n circular
// segments between one circular walk's arc and its chords — the area a partial
// cap's curved trim omits (docs/tessellation-design.md §10.2), and the circular
// twin of chordSagitta.
//
// One subarc of angle φ = θ/n cuts off (r²/2)(φ − sin φ), and sin φ ≥ φ − φ³/6
// on the whole non-negative axis — the elementary alternating-series bound, not
// a derived one — so that segment is at most r²φ³/12 and the n of them sum to
// r²θ³/(12 n²). Like chordSagitta it therefore needs no trig call, carries none
// of Sin's missing ulp contract, and does not move with FMA contraction between
// architectures. Every product is outward-rounded and the single division is
// rounded outward once; the denominator 12 n² is left exact, since every
// admitted n keeps it far inside float64's exact-integer range and rounding a
// DIVISOR outward would tighten the quotient, the wrong direction.
func chordSegmentArea(radius, sweep float64, n int) float64 {
	if n <= 0 || sweep < 0 || isNonFinite(radius) || isNonFinite(sweep) {
		return math.Inf(1)
	}
	if radius <= 0 || sweep == 0 {
		return 0
	}
	denom := 12 * float64(n) * float64(n)
	if denom <= 0 || isNonFinite(denom) {
		return 0
	}
	cube := productUpper(sweep, productUpper(sweep, sweep))
	return upRound(productUpper(productUpper(radius, radius), cube) / denom)
}
