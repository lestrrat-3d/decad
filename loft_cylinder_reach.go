package decad

import (
	"math/big"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// The first piece of each radial-monotone connector must meet the circle
// before its shared diagonal. Its cap vector vanishes at one end. Factoring
// that axial factor keeps the crossing distance correlated with height.
// The bound applies to both the denoted source wall and the held Loft wall.
func loftCylinderConnectorReach(lp loftPayload, root int, radius, collar float64) (float64, float64, float64, bool) {
	if collar < 0 || proofbound.IsNonFinite(collar) || radius <= 0 ||
		proofbound.IsNonFinite(radius) || lp.proof.FacetDeparture < 0 ||
		proofbound.IsNonFinite(lp.proof.FacetDeparture) {
		return 0, 0, 0, false
	}
	n := len(lp.verts) / 2
	if len(lp.verts)%2 != 0 || root < 3 || root >= n-1 ||
		len(lp.tris) != 4*n-4 {
		return 0, 0, 0, false
	}
	z0, z1 := proofarith.FloatRat(lp.verts[0].Z), proofarith.FloatRat(lp.verts[n].Z)
	height := new(big.Rat).Sub(z1, z0)
	if height.Sign() <= 0 {
		return 0, 0, 0, false
	}
	lo := new(big.Rat).Quo(proofarith.FloatRat(collar), height)
	hi := new(big.Rat).Sub(big.NewRat(1, 1), lo)
	if lo.Cmp(hi) >= 0 {
		return 0, 0, 0, false
	}
	r := proofarith.FloatRat(radius)
	r2 := new(big.Rat).Mul(r, r)
	delta := proofarith.FloatRat(lp.proof.FacetDeparture)
	if delta.Sign() < 0 || delta.Cmp(r) >= 0 {
		return 0, 0, 0, false
	}
	outer := new(big.Rat).Add(r, delta)
	inner := new(big.Rat).Sub(r, delta)
	deviation := new(big.Rat).Sub(new(big.Rat).Mul(outer, outer), r2)
	innerDeviation := new(big.Rat).Sub(r2, new(big.Rat).Mul(inner, inner))
	if innerDeviation.Cmp(deviation) > 0 {
		deviation = innerDeviation
	}
	maximumShift, maximumAngular := 0.0, 0.0
	for _, cell := range []int{0, root - 1} {
		source, ok := loftCylinderSourceConnectorOf(lp, cell, root)
		if !ok {
			return 0, 0, 0, false
		}
		held, ok := loftCylinderHeldConnectorOf(lp, cell, root, source)
		if !ok {
			return 0, 0, 0, false
		}
		angular, ok := loftCylinderConnectorRootAngle(source, held, lo, hi, r)
		if !ok {
			return 0, 0, 0, false
		}
		maximumAngular = max(maximumAngular, proofbound.RatFloatUp(angular))
		for _, connector := range []loftCylinderSourceConnector{source, held} {
			if !loftCylinderConnectorReachPiece(connector, r2, lo, hi, proofarith.FloatRat(0.08)) {
				return 0, 0, 0, false
			}
			// At one circle crossing, its parameter-matched point on the other
			// Loft wall is within δ. Both crossings lie on the first piece.
			// Squared-radius monotonicity bounds their parameter gap, and the
			// vanishing cap factor cancels against that piece's length.
			shift, ok := loftCylinderConnectorCrossShift(connector, lo, hi, deviation)
			if !ok || new(big.Rat).Add(delta, shift).Cmp(proofarith.FloatRat(0.08)) > 0 {
				return 0, 0, 0, false
			}
			maximumShift = max(maximumShift, proofbound.AbsSumUpper(
				lp.proof.FacetDeparture, proofbound.RatFloatUp(shift)))
		}
	}
	quarterRadius := new(big.Rat).Quo(r, big.NewRat(4, 1))
	if proofarith.FloatRat(maximumShift).Cmp(quarterRadius) >= 0 ||
		proofarith.FloatRat(maximumAngular).Cmp(quarterRadius) >= 0 {
		return 0, 0, 0, false
	}
	return 0.08, maximumShift, maximumAngular, true
}

// Bound the chord on radius R between the held root ray, where the gap ring
// starts, and the true connector crossing ray. The first term pairs the
// parameter-matched root rulings. The second charges only the true
// connector's tangential motion before it reaches R.
func loftCylinderConnectorRootAngle(source, held loftCylinderSourceConnector,
	lo, hi, radius *big.Rat) (*big.Rat, bool) {
	aLo := loftCylinderSourceLerp(source.a[0], source.a[1], lo)
	aHi := loftCylinderSourceLerp(source.a[0], source.a[1], hi)
	hLo := loftCylinderSourceLerp(held.a[0], held.a[1], lo)
	hHi := loftCylinderSourceLerp(held.a[0], held.a[1], hi)
	minimum2 := loftCylinderSourceRadiusMin2(aLo, aHi)
	heldMinimum2 := loftCylinderSourceRadiusMin2(hLo, hHi)
	if heldMinimum2.Cmp(minimum2) < 0 {
		minimum2 = heldMinimum2
	}
	minimum := proofarith.FloatRat(proofbound.RatSqrtDown(minimum2))
	if minimum.Sign() <= 0 {
		return nil, false
	}
	v0 := loftCylinderSourceSub(source.d[0], source.a[0])
	v1 := loftCylinderSourceSub(source.d[1], source.a[1])
	if (loftCylinderSourceDot(v0, v0).Sign() == 0) ==
		(loftCylinderSourceDot(v1, v1).Sign() == 0) {
		return nil, false
	}
	v := v0
	if loftCylinderSourceDot(v0, v0).Sign() == 0 {
		v = v1
	}
	length := proofarith.FloatRat(proofbound.RatSqrtDown(loftCylinderSourceDot(v, v)))
	if length.Sign() <= 0 {
		return nil, false
	}
	angleReach := proofarith.FloatRat(0.02)
	for _, sign := range []int64{-1, 1} {
		gate := func(t *big.Rat) *big.Rat {
			a := loftCylinderSourceLerp(source.a[0], source.a[1], t)
			cross := new(big.Rat).Sub(new(big.Rat).Mul(a.x, v.y),
				new(big.Rat).Mul(a.y, v.x))
			cross.Mul(cross, big.NewRat(sign, 1))
			derivative := new(big.Rat).Mul(big.NewRat(2, 1), loftCylinderSourceDot(a, v))
			deficit := new(big.Rat).Sub(new(big.Rat).Mul(radius, radius), loftCylinderSourceDot(a, a))
			left := new(big.Rat).Mul(angleReach, new(big.Rat).Mul(minimum, derivative))
			right := new(big.Rat).Mul(big.NewRat(2, 1), new(big.Rat).Mul(deficit, cross))
			return left.Sub(left, right)
		}
		if !loftCylinderCubicPositive(gate, lo, hi, 6) {
			return nil, false
		}
	}
	rootDifference2 := loftCylinderSourceDot(loftCylinderSourceSub(source.a[0], held.a[0]),
		loftCylinderSourceSub(source.a[0], held.a[0]))
	otherDifference := loftCylinderSourceDot(loftCylinderSourceSub(source.a[1], held.a[1]),
		loftCylinderSourceSub(source.a[1], held.a[1]))
	if otherDifference.Cmp(rootDifference2) > 0 {
		rootDifference2 = otherDifference
	}
	rootDifference := proofarith.FloatRat(proofbound.RatSqrtUp(rootDifference2))
	rootRay := new(big.Rat).Quo(new(big.Rat).Mul(big.NewRat(2, 1),
		new(big.Rat).Mul(radius, rootDifference)), minimum)
	return rootRay.Add(rootRay, angleReach), true
}

func loftCylinderCubicPositive(f func(*big.Rat) *big.Rat, lo, hi *big.Rat, depth int) bool {
	third := new(big.Rat).Quo(new(big.Rat).Sub(hi, lo), big.NewRat(3, 1))
	t1 := new(big.Rat).Add(lo, third)
	t2 := new(big.Rat).Add(t1, third)
	b0, f1, f2, b3 := f(lo), f(t1), f(t2), f(hi)
	a := new(big.Rat).Sub(new(big.Rat).Mul(big.NewRat(27, 1), f1),
		new(big.Rat).Add(new(big.Rat).Mul(big.NewRat(8, 1), b0), b3))
	b := new(big.Rat).Sub(new(big.Rat).Mul(big.NewRat(27, 1), f2),
		new(big.Rat).Add(b0, new(big.Rat).Mul(big.NewRat(8, 1), b3)))
	b1 := new(big.Rat).Quo(new(big.Rat).Sub(new(big.Rat).Mul(big.NewRat(2, 1), a), b),
		big.NewRat(18, 1))
	b2 := new(big.Rat).Quo(new(big.Rat).Sub(new(big.Rat).Mul(big.NewRat(2, 1), b), a),
		big.NewRat(18, 1))
	if b0.Sign() > 0 && b1.Sign() > 0 && b2.Sign() > 0 && b3.Sign() > 0 {
		return true
	}
	if depth == 0 {
		return false
	}
	mid := new(big.Rat).Quo(new(big.Rat).Add(lo, hi), big.NewRat(2, 1))
	return loftCylinderCubicPositive(f, lo, mid, depth-1) &&
		loftCylinderCubicPositive(f, mid, hi, depth-1)
}

// Source root and connector rulings move linearly between the cap sections.
// A cap-collar point is at most collar/height of its whole XY travel from
// the matching cap point. Source wall positions lie within the held wall's
// matched Loft bound, so the paired held vertices give a radius ceiling.
func loftCylinderCollarSweep(parts []loftCylinderTooth, z0, z1, collar float64) (float64, bool) {
	if collar == 0 {
		return 0, true
	}
	height := new(big.Rat).Sub(proofarith.FloatRat(z1), proofarith.FloatRat(z0))
	if height.Sign() <= 0 || collar < 0 || proofbound.IsNonFinite(collar) {
		return 0, false
	}
	maximum := 0.0
	for _, part := range parts {
		toothMaximum := 0.0
		for _, p := range part.loft.verts {
			x, y := proofarith.FloatRat(p.X), proofarith.FloatRat(p.Y)
			radius := proofbound.RatSqrtUp(new(big.Rat).Add(
				new(big.Rat).Mul(x, x), new(big.Rat).Mul(y, y)))
			toothMaximum = max(toothMaximum, radius)
		}
		maximum = max(maximum, proofbound.AbsSumUpper(toothMaximum, part.loft.proof.FacetDeparture))
	}
	ratio := proofbound.RatFloatUp(new(big.Rat).Quo(proofarith.FloatRat(collar), height))
	sweep := proofbound.ProductUpper(proofbound.ProductUpper(2, maximum), ratio)
	return sweep, !proofbound.IsNonFinite(sweep)
}

func loftCylinderConnectorCrossShift(c loftCylinderSourceConnector, lo, hi, deviation *big.Rat) (*big.Rat, bool) {
	v0 := loftCylinderSourceSub(c.d[0], c.a[0])
	v1 := loftCylinderSourceSub(c.d[1], c.a[1])
	zero0 := loftCylinderSourceDot(v0, v0).Sign() == 0
	zero1 := loftCylinderSourceDot(v1, v1).Sign() == 0
	if zero0 == zero1 {
		return nil, false
	}
	v := v0
	if zero0 {
		v = v1
	}
	length := proofarith.FloatRat(proofbound.RatSqrtUp(loftCylinderSourceDot(v, v)))
	aLo := loftCylinderSourceLerp(c.a[0], c.a[1], lo)
	aHi := loftCylinderSourceLerp(c.a[0], c.a[1], hi)
	gLo := new(big.Rat).Mul(big.NewRat(2, 1), loftCylinderSourceDot(aLo, v))
	gHi := new(big.Rat).Mul(big.NewRat(2, 1), loftCylinderSourceDot(aHi, v))
	if gHi.Cmp(gLo) < 0 {
		gLo = gHi
	}
	if gLo.Sign() <= 0 {
		return nil, false
	}
	return new(big.Rat).Quo(new(big.Rat).Mul(deviation, length), gLo), true
}

func loftCylinderHeldConnectorOf(lp loftPayload, cell, root int,
	source loftCylinderSourceConnector) (loftCylinderSourceConnector, bool) {
	var held loftCylinderSourceConnector
	n := len(lp.verts) / 2
	near, far := 0, 1
	if cell != 0 {
		near, far = root, root-1
	}
	for cap := range 2 {
		held.a[cap] = loftCylinderExactXY(lp.verts[cap*n+near])
		held.f[cap] = loftCylinderExactXY(lp.verts[cap*n+far])
		switch {
		case loftCylinderSourceSameXY(source.d[cap], source.a[cap]):
			held.d[cap] = held.a[cap]
		case loftCylinderSourceSameXY(source.d[cap], source.f[cap]):
			held.d[cap] = held.f[cap]
		default:
			return loftCylinderSourceConnector{}, false
		}
	}
	return held, true
}

func loftCylinderSourceSameXY(a, b loftCylinderXY) bool {
	return a.x.Cmp(b.x) == 0 && a.y.Cmp(b.y) == 0
}

func loftCylinderConnectorReachPiece(c loftCylinderSourceConnector, radius2, lo, hi, reach *big.Rat) bool {
	aLo := loftCylinderSourceLerp(c.a[0], c.a[1], lo)
	aHi := loftCylinderSourceLerp(c.a[0], c.a[1], hi)
	dLo := loftCylinderSourceLerp(c.d[0], c.d[1], lo)
	dHi := loftCylinderSourceLerp(c.d[0], c.d[1], hi)
	if loftCylinderSourceDot(aLo, aLo).Cmp(radius2) >= 0 ||
		loftCylinderSourceDot(aHi, aHi).Cmp(radius2) >= 0 ||
		loftCylinderSourceRadiusMin2(dLo, dHi).Cmp(radius2) <= 0 {
		return false
	}
	v0 := loftCylinderSourceSub(c.d[0], c.a[0])
	v1 := loftCylinderSourceSub(c.d[1], c.a[1])
	zero0 := loftCylinderSourceDot(v0, v0).Sign() == 0
	zero1 := loftCylinderSourceDot(v1, v1).Sign() == 0
	if zero0 == zero1 {
		return false
	}
	v := v0
	if zero0 {
		v = v1
	}
	length := proofarith.FloatRat(proofbound.RatSqrtUp(loftCylinderSourceDot(v, v)))
	if length.Sign() <= 0 {
		return false
	}
	gate := func(t *big.Rat) *big.Rat {
		a := loftCylinderSourceLerp(c.a[0], c.a[1], t)
		derivative := new(big.Rat).Mul(big.NewRat(2, 1), loftCylinderSourceDot(a, v))
		deficit := new(big.Rat).Sub(radius2, loftCylinderSourceDot(a, a))
		return new(big.Rat).Sub(new(big.Rat).Mul(reach, derivative),
			new(big.Rat).Mul(length, deficit))
	}
	// The radial derivative must remain strictly positive after removing the
	// cap's vanishing axial factor. This also proves a transverse crossing.
	if loftCylinderSourceDot(aLo, v).Sign() <= 0 || loftCylinderSourceDot(aHi, v).Sign() <= 0 {
		return false
	}
	return loftCylinderQuadraticPositive(gate, lo, hi, 3)
}

// Exact quadratic Bernstein coefficients are sufficient on each interval.
// A fixed depth bounds work and lets a positive quadratic with a negative
// whole-interval middle coefficient pass after subdivision.
func loftCylinderQuadraticPositive(f func(*big.Rat) *big.Rat, lo, hi *big.Rat, depth int) bool {
	mid := new(big.Rat).Quo(new(big.Rat).Add(lo, hi), big.NewRat(2, 1))
	b0, bm, b2 := f(lo), f(mid), f(hi)
	b1 := new(big.Rat).Sub(new(big.Rat).Mul(big.NewRat(2, 1), bm),
		new(big.Rat).Quo(new(big.Rat).Add(b0, b2), big.NewRat(2, 1)))
	if b0.Sign() > 0 && b1.Sign() > 0 && b2.Sign() > 0 {
		return true
	}
	return depth > 0 && loftCylinderQuadraticPositive(f, lo, mid, depth-1) &&
		loftCylinderQuadraticPositive(f, mid, hi, depth-1)
}
