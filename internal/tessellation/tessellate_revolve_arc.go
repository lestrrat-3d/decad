package tessellation

import (
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/survey2d"

	"github.com/lestrrat-3d/decad/internal/proofbound"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
)

// RevArcCell is the meridian model of ONE circular chord: the axis-coordinate
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
// the held samples (RevolveCellAreaSlack). The gap between the two is a
// coordinate-construction displacement bounded by deltaC, and it enters twice:
// the ρ enclosure below is WIDENED by it, and §10.2's per-triangle
// coordinate-stage allowance charges it again over the whole mesh.
type RevArcCell struct {
	CV, Radius, Th0, Dth *big.Rat
}

// RevolveArcChordCell builds the model of chord k of a circular axis walk
// divided into n chords. th0/th1 on an axis walk are the plane walk's own
// angles shifted by the axis rotation, so their difference is the recorded
// sweep and chord k spans [θ0 + k·Δθ, θ0 + (k+1)·Δθ] exactly.
func RevolveArcChordCell(w survey2d.SegmentWalk, k, n int) (*RevArcCell, bool) {
	cV, radius := proofarith.FloatRat(w.CV), proofarith.FloatRat(w.Radius)
	th0, th1 := proofarith.FloatRat(w.Th0), proofarith.FloatRat(w.Th1)
	if cV == nil || radius == nil || th0 == nil || th1 == nil || n <= 0 || radius.Sign() < 0 {
		return nil, false
	}
	dth := new(big.Rat).Quo(new(big.Rat).Sub(th1, th0), new(big.Rat).SetInt64(int64(n)))
	start := new(big.Rat).Add(th0, new(big.Rat).Mul(dth, new(big.Rat).SetInt64(int64(k))))
	return &RevArcCell{CV: cV, Radius: radius, Th0: start, Dth: dth}, true
}

// speed is |dγ/dt| for the matched parameterisation, r·|Δθ|, which is CONSTANT
// on a circular arc — the one simplification a curved generator does give.
func (c RevArcCell) Speed() *big.Rat {
	return new(big.Rat).Mul(c.Radius, new(big.Rat).Abs(new(big.Rat).Set(c.Dth)))
}

// rhoNodes encloses ρ at each node of the fixed subdivision, and states the
// per-piece second-order allowance the integral below charges beside them.
//
// Nothing here compares against π. Two survey2d.RadSinCosInterval calls enclose the
// starting angle and one step. The addition identities then carry certified
// intervals from one node to the next. This avoids running the trig series at
// every node while preserving an enclosure at each exact rational angle.
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
func (c RevArcCell) RhoNodes() ([]proofbound.RatInterval, *big.Rat, bool) {
	nodes := make([]proofbound.RatInterval, RevolveArcIntegralSteps+1)
	step := new(big.Rat).Quo(c.Dth, big.NewRat(RevolveArcIntegralSteps, 1))
	sinIv, cosIv, ok := survey2d.RadSinCosInterval(c.Th0)
	if !ok {
		return nil, nil, false
	}
	stepSinIv, stepCosIv, ok := survey2d.RadSinCosInterval(step)
	if !ok {
		return nil, nil, false
	}
	sin, cos := ArcFixedFromRat(sinIv), ArcFixedFromRat(cosIv)
	stepSin, stepCos := ArcFixedFromRat(stepSinIv), ArcFixedFromRat(stepCosIv)
	for i := range nodes {
		nodes[i] = proofbound.IntervalAdd(proofbound.PointInterval(c.CV), proofbound.IntervalScale(sin.Rat(), c.Radius))
		if i+1 < len(nodes) {
			nextSin := ArcFixedAdd(ArcFixedMul(sin, stepCos), ArcFixedMul(cos, stepSin))
			cos = ArcFixedSub(ArcFixedMul(cos, stepCos), ArcFixedMul(sin, stepSin))
			sin = nextSin
		}
	}
	steps := big.NewRat(RevolveArcIntegralSteps, 1)
	bulge := new(big.Rat).Quo(
		new(big.Rat).Mul(c.Radius, new(big.Rat).Mul(c.Dth, c.Dth)),
		new(big.Rat).Mul(big.NewRat(8, 1), new(big.Rat).Mul(steps, steps)),
	)
	return nodes, bulge, true
}

// ArcFixedInterval holds a certified interval as integer multiples of the
// package's 2^-proofbound.TrigFixedBits grid. The recurrence rounds each product outward,
// keeping numerator and denominator sizes fixed across all 32 nodes.
type ArcFixedInterval struct{ Lo, Hi *big.Int }

func ArcFixedFromRat(a proofbound.RatInterval) ArcFixedInterval {
	return ArcFixedInterval{proofbound.FixedFloor(a.Lo), proofbound.FixedCeil(a.Hi)}
}

func (a ArcFixedInterval) Rat() proofbound.RatInterval {
	return proofbound.IntervalOwned(proofbound.FixedToRat(a.Lo), proofbound.FixedToRat(a.Hi))
}

func ArcFixedAdd(a, b ArcFixedInterval) ArcFixedInterval {
	return ArcFixedInterval{new(big.Int).Add(a.Lo, b.Lo), new(big.Int).Add(a.Hi, b.Hi)}
}

func ArcFixedSub(a, b ArcFixedInterval) ArcFixedInterval {
	return ArcFixedInterval{new(big.Int).Sub(a.Lo, b.Hi), new(big.Int).Sub(a.Hi, b.Lo)}
}

func ArcFixedMul(a, b ArcFixedInterval) ArcFixedInterval {
	products := [4]*big.Int{
		new(big.Int).Mul(a.Lo, b.Lo), new(big.Int).Mul(a.Lo, b.Hi),
		new(big.Int).Mul(a.Hi, b.Lo), new(big.Int).Mul(a.Hi, b.Hi),
	}
	lo, hi := products[0], products[0]
	for _, product := range products[1:] {
		if product.Cmp(lo) < 0 {
			lo = product
		}
		if product.Cmp(hi) > 0 {
			hi = product
		}
	}
	// Rsh rounds a negative integer toward minus infinity. Negating that
	// floor gives the outward ceiling for the upper endpoint.
	lo = new(big.Int).Rsh(lo, proofbound.TrigFixedBits)
	hi = new(big.Int).Neg(new(big.Int).Rsh(new(big.Int).Neg(hi), proofbound.TrigFixedBits))
	return ArcFixedInterval{lo, hi}
}

// RevolveArcIntegralSteps is the fixed certified-subdivision budget one
// circular cell's Ecell spends: the common parameter domain's meridian
// direction is cut into this many equal pieces, and every piece is enclosed and
// charged. It is fixed rather than adaptive because tess §10.2 asks for a
// bound under a SHARED FIXED budget, and because the bound each piece
// contributes is already valid at any depth — depth buys tightness, never
// soundness. Thirty-two pieces leave the reading within a few percent of the
// integral it bounds on an ordinary cell, and the second-order allowance
// rhoNodes charges beside them falls as 1/N², so the budget is spent where it
// buys the most.
const RevolveArcIntegralSteps = 32

// RevolveArcCellSlack is docs/tessellation-design.md §10.2's Ecell for one wall
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
// So each of the two half-domains is cut into RevolveArcIntegralSteps pieces.
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
func RevolveArcCellSlack(cell RevArcCell, step proofbound.RatInterval, twoArea [2]proofbound.RatInterval, slack float64) (float64, error) {
	_, rho, extra, ok := RevolveArcScale(cell, step, slack)
	if !ok {
		return 0, ErrRevolveArcCellSlack
	}
	total := new(big.Rat)
	zero := proofbound.PointInterval(new(big.Rat))
	for half, weight := range [2]int{RevolveWeightT, RevolveWeightOneMinusT} {
		total.Add(total, RevolveArcAbsIntegral(rho, twoArea[half], zero, extra, weight))
	}
	return proofbound.RatFloatUp(total), nil
}

// RevolveArcFanSlack is RevolveArcCellSlack for a circular cell with ONE ring
// on the axis — a sphere's polar cell. The held facets are a fan of single
// triangles over the whole unit square with the pole edge collapsed, so
// Jheld = 2A·t for a pole at t = 0 and 2A·(1 − t) for a pole at t = 1, while
// Jtrue keeps the same sinusoidal ρ(t) the quad case has. The subdivision is
// therefore identical with a LINEAR held density in place of a constant one.
func RevolveArcFanSlack(cell RevArcCell, poleFirst bool, step, twoArea proofbound.RatInterval, slack float64) (float64, error) {
	_, rho, extra, ok := RevolveArcScale(cell, step, slack)
	if !ok {
		return 0, ErrRevolveArcCellSlack
	}
	held, slope := proofbound.PointInterval(new(big.Rat)), twoArea
	if !poleFirst {
		held, slope = twoArea, proofbound.IntervalNeg(twoArea)
	}
	return proofbound.RatFloatUp(RevolveArcAbsIntegral(rho, held, slope, extra, RevolveWeightOne)), nil
}

// RevolveArcScale composes the non-negative factor r·|Δθ|·|dφ| every circular
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
func RevolveArcScale(cell RevArcCell, step proofbound.RatInterval, slack float64) (proofbound.RatInterval, []proofbound.RatInterval, *big.Rat, bool) {
	s := proofarith.FloatRat(slack)
	if s == nil || s.Sign() < 0 || cell.Radius.Sign() < 0 {
		return proofbound.RatInterval{}, nil, nil, false
	}
	rho, bulge, ok := cell.RhoNodes()
	if !ok {
		return proofbound.RatInterval{}, nil, nil, false
	}
	scale := proofbound.IntervalScale(IntervalAbsSpan(step), cell.Speed())
	extra := new(big.Rat).Mul(scale.Hi, new(big.Rat).Add(bulge, s))
	// The subdivision's two halves read the same scale·ρ at every node, so
	// the product is taken here once and the nodes are returned pre-scaled.
	for i := range rho {
		rho[i] = proofbound.IntervalMul(scale, rho[i])
	}
	return scale, rho, extra, true
}

var ErrRevolveArcCellSlack = fmt.Errorf(`%w: a circular revolve cell states no enclosure of the area its held facets and the patch they stand for differ by`, decaderr.ErrUnsupported)

// RevolveArcAbsIntegral bounds ∫₀¹ |scale·ρ(t) − (held + slope·t)|·w(t) dt
// upward over the fixed subdivision whose NODE ρ enclosures the caller holds,
// pre-multiplied by scale (RevolveArcScale's own doc comment). One piece is
// charged the larger of its two nodes' magnitudes plus the caller's own
// allowance, times the exact integral of the weight over that piece, so the
// answer is an upper bound at any depth and nothing cancels between pieces.
func RevolveArcAbsIntegral(scaledRho []proofbound.RatInterval, held, slope proofbound.RatInterval, extra *big.Rat, weight int) *big.Rat {
	// Every node uses t=i/N, and each weight integrates to an integer over
	// 2N². Put all endpoint and allowance rationals over one denominator,
	// then sum integer numerators. The final SetFrac is the only reduction;
	// the resulting rational is identical to the per-piece Rat sum.
	const n = RevolveArcIntegralSteps
	den := RevolveArcIntegralDenominator(scaledRho, held, slope, extra)
	heldLo := RevolveArcScaledNumerator(held.Lo, den, 1)
	heldHi := RevolveArcScaledNumerator(held.Hi, den, 1)
	slopeLo := RevolveArcScaledNumerator(slope.Lo, den, n)
	slopeHi := RevolveArcScaledNumerator(slope.Hi, den, n)
	extraNum := RevolveArcScaledNumerator(extra, den, 1)
	at := func(i int) *big.Int {
		lo, hi := RevolveArcNodeNumerators(scaledRho[i], den, heldLo, heldHi, slopeLo, slopeHi, i)
		lo.Abs(lo)
		hi.Abs(hi)
		if lo.Cmp(hi) > 0 {
			return lo
		}
		return hi
	}
	total := new(big.Int)
	prev := at(0)
	for i := range n {
		next := at(i + 1)
		maximum := prev
		if next.Cmp(maximum) > 0 {
			maximum = next
		}
		piece := new(big.Int).Add(maximum, extraNum)
		weightNum := int64(2 * n)
		switch weight {
		case RevolveWeightT:
			weightNum = int64(2*i + 1)
		case RevolveWeightOneMinusT:
			weightNum = int64(2*n - 2*i - 1)
		}
		total.Add(total, piece.Mul(piece, big.NewInt(weightNum)))
		prev = next
	}
	return new(big.Rat).SetFrac(total, new(big.Int).Mul(den, big.NewInt(2*n*n)))
}

// RevolveArcIntegralDenominator is divisible by every endpoint denominator,
// the allowance's denominator, and each slope denominator times the grid size.
// This lets every node and piece be evaluated over the same exact unit.
func RevolveArcIntegralDenominator(rho []proofbound.RatInterval, held, slope proofbound.RatInterval, extra *big.Rat) *big.Int {
	den := big.NewInt(1)
	include := func(r *big.Rat, factor int64) {
		d := new(big.Int).Mul(r.Denom(), big.NewInt(factor))
		g := new(big.Int).GCD(nil, nil, den, d)
		den.Mul(new(big.Int).Quo(den, g), d)
	}
	for _, node := range rho {
		include(node.Lo, 1)
		include(node.Hi, 1)
	}
	include(held.Lo, 1)
	include(held.Hi, 1)
	include(slope.Lo, RevolveArcIntegralSteps)
	include(slope.Hi, RevolveArcIntegralSteps)
	include(extra, 1)
	return den
}

func RevolveArcScaledNumerator(r *big.Rat, den *big.Int, factor int64) *big.Int {
	d := new(big.Int).Mul(r.Denom(), big.NewInt(factor))
	return new(big.Int).Mul(r.Num(), new(big.Int).Quo(den, d))
}

// RevolveArcNodeNumerators returns the old interval subtraction's exact lower
// and upper endpoints as integer numerators over den. Inputs remain owned by
// the caller; the returned integers are fresh and may be mutated.
func RevolveArcNodeNumerators(rho proofbound.RatInterval, den, heldLo, heldHi, slopeLo, slopeHi *big.Int, i int) (*big.Int, *big.Int) {
	idx := big.NewInt(int64(i))
	lo := new(big.Int).Sub(RevolveArcScaledNumerator(rho.Lo, den, 1), heldHi)
	lo.Sub(lo, new(big.Int).Mul(idx, slopeHi))
	hi := new(big.Int).Sub(RevolveArcScaledNumerator(rho.Hi, den, 1), heldLo)
	hi.Sub(hi, new(big.Int).Mul(idx, slopeLo))
	return lo, hi
}

// IntervalAbsSpan is the enclosure of |x| for x in the given enclosure.
func IntervalAbsSpan(a proofbound.RatInterval) proofbound.RatInterval {
	if a.Lo.Sign() >= 0 {
		return a
	}
	if a.Hi.Sign() <= 0 {
		return proofbound.IntervalNeg(a)
	}
	return proofbound.Interval(new(big.Rat), IntervalAbsUpper(a))
}

var ErrRevolveStationEnclosure = fmt.Errorf(`%w: a revolve meridian chord station states no enclosure of the axis coordinates its record denotes`, decaderr.ErrUnsupported)

// ChordSegmentArea is a PROVEN upper bound on the total area of the n circular
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
func ChordSegmentArea(radius, sweep float64, n int) float64 {
	if n <= 0 || sweep < 0 || proofbound.IsNonFinite(radius) || proofbound.IsNonFinite(sweep) {
		return math.Inf(1)
	}
	if radius <= 0 || sweep == 0 {
		return 0
	}
	denom := 12 * float64(n) * float64(n)
	if denom <= 0 || proofbound.IsNonFinite(denom) {
		return 0
	}
	cube := proofbound.ProductUpper(sweep, proofbound.ProductUpper(sweep, sweep))
	return proofbound.UpRound(proofbound.ProductUpper(proofbound.ProductUpper(radius, radius), cube) / denom)
}
