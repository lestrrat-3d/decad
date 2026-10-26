package loftcylinder

import (
	"math/big"

	"github.com/lestrrat-3d/decad/internal/momentinput"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// A LineSeg Loft wall is the held triangle pair, so its section at height t
// is a two-segment chain through the triangles' common diagonal. Prove that
// both segments increase in projected radius for every t. The inside root
// ruling and outside flank ruling then give exactly one circle crossing.
func ConnectorGate(verts []r3.Vec, tris [][3]int, segments [][]int,
	root int, radius, height, sourceBound float64) (float64, bool) {
	n := len(verts) / 2
	if root < 3 || segments[0][0] != 0 || segments[0][1] != 1 ||
		segments[0][root-2] != 3 || segments[0][root-1] != 4 {
		return 0, false
	}
	collar := 0.0
	for _, spec := range [][2]int{{0, 1}, {root, root - 1}} {
		near, far := spec[0], spec[1]
		j := min(near, far)
		a := [2]r3.Vec{verts[near], verts[n+near]}
		f := [2]r3.Vec{verts[far], verts[n+far]}
		minimum := TriangleMinRadius(f[:], [3]int{0, 1, 1})
		threshold := proofarith.AddRat(new(big.Rat), proofarith.FloatRat(radius), proofarith.FloatRat(sourceBound))
		if proofarith.FloatRat(minimum).Cmp(threshold) <= 0 {
			return 0, false
		}
		var shared []int
		for _, v := range tris[2*j] {
			for _, w := range tris[2*j+1] {
				if v == w {
					shared = append(shared, v)
				}
			}
		}
		if len(shared) != 2 {
			return 0, false
		}
		diagonal := [2]int{-1, -1}
		for _, v := range shared {
			switch v {
			case near, far:
				diagonal[0] = v
			case n + near, n + far:
				diagonal[1] = v
			default:
				return 0, false
			}
		}
		if diagonal[0] < 0 || diagonal[1] < 0 {
			return 0, false
		}
		d := [2]r3.Vec{verts[diagonal[0]], verts[diagonal[1]]}
		if !heldRadialMonotone(a, d) || !heldRadialMonotone(d, f) {
			return 0, false
		}
		nearCollar, ok := heldRootCollar(a, radius, height)
		if !ok {
			return 0, false
		}
		collar = max(collar, nearCollar)
	}
	return collar, true
}

func heldRadialMonotone(u, v [2]r3.Vec) bool {
	dot := func(a, b XY) *big.Rat {
		return proofarith.AddRat(new(big.Rat),
			proofarith.MulRat(new(big.Rat), a.X, b.X), proofarith.MulRat(new(big.Rat), a.Y, b.Y))
	}
	delta := func(a, b XY) XY {
		return XY{proofarith.SubRat(new(big.Rat), b.X, a.X), proofarith.SubRat(new(big.Rat), b.Y, a.Y)}
	}
	u0, u1 := ExactXY(u[0]), ExactXY(u[1])
	d0, d1 := delta(u0, ExactXY(v[0])), delta(u1, ExactXY(v[1]))
	g0, g2 := dot(u0, d0), dot(u1, d1)
	g1 := proofarith.AddRat(new(big.Rat), dot(u0, d1), dot(u1, d0))
	return g0.Sign() >= 0 && g1.Sign() >= 0 && g2.Sign() >= 0
}

func heldRootCollar(near [2]r3.Vec, radius, height float64) (float64, bool) {
	q := func(p XY) *big.Rat {
		return proofarith.AddRat(new(big.Rat), proofarith.MulRat(new(big.Rat), p.X, p.X), proofarith.MulRat(new(big.Rat), p.Y, p.Y))
	}
	a, b := ExactXY(near[0]), ExactXY(near[1])
	r := proofarith.FloatRat(radius)
	r2 := proofarith.MulRat(new(big.Rat), r, r)
	if q(a).Cmp(r2) <= 0 && q(b).Cmp(r2) <= 0 {
		return 0, true
	}
	for shift := 52; shift >= 8; shift-- {
		epsilon := new(big.Rat).SetFrac(big.NewInt(1), new(big.Int).Lsh(big.NewInt(1), uint(shift)))
		oneMinus := proofarith.SubRat(new(big.Rat), big.NewRat(1, 1), epsilon)
		bottom := XY{
			X: proofarith.AddRat(new(big.Rat), proofarith.MulRat(new(big.Rat), oneMinus, a.X), proofarith.MulRat(new(big.Rat), epsilon, b.X)),
			Y: proofarith.AddRat(new(big.Rat), proofarith.MulRat(new(big.Rat), oneMinus, a.Y), proofarith.MulRat(new(big.Rat), epsilon, b.Y)),
		}
		top := XY{
			X: proofarith.AddRat(new(big.Rat), proofarith.MulRat(new(big.Rat), epsilon, a.X), proofarith.MulRat(new(big.Rat), oneMinus, b.X)),
			Y: proofarith.AddRat(new(big.Rat), proofarith.MulRat(new(big.Rat), epsilon, a.Y), proofarith.MulRat(new(big.Rat), oneMinus, b.Y)),
		}
		if q(bottom).Cmp(r2) < 0 && q(top).Cmp(r2) < 0 {
			collar := proofbound.ProductUpper(proofbound.RatFloatUp(epsilon), height)
			return collar, !proofbound.IsNonFinite(collar)
		}
	}
	return 0, false
}

// The first piece of each radial-monotone connector must meet the circle
// before its shared diagonal. Its cap vector vanishes at one end. Factoring
// that axial factor keeps the crossing distance correlated with height.
// The bound applies to both the denoted source wall and the held Loft wall.
func ConnectorReach(verts []r3.Vec, tris [][3]int, segments [][]int,
	profiles [2]momentinput.Profile, ex r3.Vec, facetDeparture float64,
	root int, radius, collar float64) (float64, float64, float64, bool) {
	if collar < 0 || proofbound.IsNonFinite(collar) || radius <= 0 ||
		proofbound.IsNonFinite(radius) || facetDeparture < 0 ||
		proofbound.IsNonFinite(facetDeparture) {
		return 0, 0, 0, false
	}
	n := len(verts) / 2
	if len(verts)%2 != 0 || root < 3 || root >= n-1 ||
		len(tris) != 4*n-4 {
		return 0, 0, 0, false
	}
	z0, z1 := proofarith.FloatRat(verts[0].Z), proofarith.FloatRat(verts[n].Z)
	height := proofarith.SubRat(new(big.Rat), z1, z0)
	if height.Sign() <= 0 {
		return 0, 0, 0, false
	}
	lo := proofarith.QuoRat(new(big.Rat), proofarith.FloatRat(collar), height)
	hi := proofarith.SubRat(new(big.Rat), big.NewRat(1, 1), lo)
	if lo.Cmp(hi) >= 0 {
		return 0, 0, 0, false
	}
	r := proofarith.FloatRat(radius)
	r2 := proofarith.MulRat(new(big.Rat), r, r)
	delta := proofarith.FloatRat(facetDeparture)
	if delta.Sign() < 0 || delta.Cmp(r) >= 0 {
		return 0, 0, 0, false
	}
	outer := proofarith.AddRat(new(big.Rat), r, delta)
	inner := proofarith.SubRat(new(big.Rat), r, delta)
	deviation := proofarith.SubRat(new(big.Rat), proofarith.MulRat(new(big.Rat), outer, outer), r2)
	innerDeviation := proofarith.SubRat(new(big.Rat), r2, proofarith.MulRat(new(big.Rat), inner, inner))
	if innerDeviation.Cmp(deviation) > 0 {
		deviation = innerDeviation
	}
	maximumShift, maximumAngular := 0.0, 0.0
	for _, cell := range []int{0, root - 1} {
		source, ok := SourceConnectorOf(verts, tris, segments, profiles, ex, cell, root)
		if !ok {
			return 0, 0, 0, false
		}
		held, ok := heldConnectorOf(verts, cell, root, source)
		if !ok {
			return 0, 0, 0, false
		}
		angular, ok := connectorRootAngle(source, held, lo, hi, r)
		if !ok {
			return 0, 0, 0, false
		}
		maximumAngular = max(maximumAngular, proofbound.RatFloatUp(angular))
		for _, connector := range []Connector{source, held} {
			if !connectorReachPiece(connector, r2, lo, hi, proofarith.FloatRat(0.08)) {
				return 0, 0, 0, false
			}
			// At one circle crossing, its parameter-matched point on the other
			// Loft wall is within δ. Both crossings lie on the first piece.
			// Squared-radius monotonicity bounds their parameter gap, and the
			// vanishing cap factor cancels against that piece's length.
			shift, ok := connectorCrossShift(connector, lo, hi, deviation)
			if !ok || proofarith.AddRat(new(big.Rat), delta, shift).Cmp(proofarith.FloatRat(0.08)) > 0 {
				return 0, 0, 0, false
			}
			maximumShift = max(maximumShift, proofbound.AbsSumUpper(
				facetDeparture, proofbound.RatFloatUp(shift)))
		}
	}
	quarterRadius := proofarith.QuoRat(new(big.Rat), r, big.NewRat(4, 1))
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
func connectorRootAngle(source, held Connector,
	lo, hi, radius *big.Rat) (*big.Rat, bool) {
	aLo := sourceLerp(source.a[0], source.a[1], lo)
	aHi := sourceLerp(source.a[0], source.a[1], hi)
	hLo := sourceLerp(held.a[0], held.a[1], lo)
	hHi := sourceLerp(held.a[0], held.a[1], hi)
	minimum2 := sourceRadiusMin2(aLo, aHi)
	heldMinimum2 := sourceRadiusMin2(hLo, hHi)
	if heldMinimum2.Cmp(minimum2) < 0 {
		minimum2 = heldMinimum2
	}
	minimum := proofarith.FloatRat(proofbound.RatSqrtDown(minimum2))
	if minimum.Sign() <= 0 {
		return nil, false
	}
	v0 := sourceSub(source.d[0], source.a[0])
	v1 := sourceSub(source.d[1], source.a[1])
	if (SourceDot(v0, v0).Sign() == 0) ==
		(SourceDot(v1, v1).Sign() == 0) {
		return nil, false
	}
	v := v0
	if SourceDot(v0, v0).Sign() == 0 {
		v = v1
	}
	length := proofarith.FloatRat(proofbound.RatSqrtDown(SourceDot(v, v)))
	if length.Sign() <= 0 {
		return nil, false
	}
	angleReach := proofarith.FloatRat(0.02)
	for _, sign := range []int64{-1, 1} {
		gate := func(t *big.Rat) *big.Rat {
			a := sourceLerp(source.a[0], source.a[1], t)
			cross := proofarith.SubRat(new(big.Rat), proofarith.MulRat(new(big.Rat), a.X, v.Y),
				proofarith.MulRat(new(big.Rat), a.Y, v.X))
			proofarith.MulRat(cross, cross, big.NewRat(sign, 1))
			derivative := proofarith.MulRat(new(big.Rat), big.NewRat(2, 1), SourceDot(a, v))
			deficit := proofarith.SubRat(new(big.Rat), proofarith.MulRat(new(big.Rat), radius, radius), SourceDot(a, a))
			left := proofarith.MulRat(new(big.Rat), angleReach, proofarith.MulRat(new(big.Rat), minimum, derivative))
			right := proofarith.MulRat(new(big.Rat), big.NewRat(2, 1), proofarith.MulRat(new(big.Rat), deficit, cross))
			return proofarith.SubRat(left, left, right)
		}
		if !cubicPositive(gate, lo, hi, 6) {
			return nil, false
		}
	}
	rootDifference2 := SourceDot(sourceSub(source.a[0], held.a[0]),
		sourceSub(source.a[0], held.a[0]))
	otherDifference := SourceDot(sourceSub(source.a[1], held.a[1]),
		sourceSub(source.a[1], held.a[1]))
	if otherDifference.Cmp(rootDifference2) > 0 {
		rootDifference2 = otherDifference
	}
	rootDifference := proofarith.FloatRat(proofbound.RatSqrtUp(rootDifference2))
	rootRay := proofarith.QuoRat(new(big.Rat), proofarith.MulRat(new(big.Rat), big.NewRat(2, 1),
		proofarith.MulRat(new(big.Rat), radius, rootDifference)), minimum)
	return proofarith.AddRat(rootRay, rootRay, angleReach), true
}

func cubicPositive(f func(*big.Rat) *big.Rat, lo, hi *big.Rat, depth int) bool {
	third := new(big.Rat).Quo(proofarith.SubRat(new(big.Rat), hi, lo), big.NewRat(3, 1))
	t1 := proofarith.AddRat(new(big.Rat), lo, third)
	t2 := proofarith.AddRat(new(big.Rat), t1, third)
	b0, f1, f2, b3 := f(lo), f(t1), f(t2), f(hi)
	a := proofarith.SubRat(new(big.Rat), proofarith.MulRat(new(big.Rat), big.NewRat(27, 1), f1),
		proofarith.AddRat(new(big.Rat), proofarith.MulRat(new(big.Rat), big.NewRat(8, 1), b0), b3))
	b := proofarith.SubRat(new(big.Rat), proofarith.MulRat(new(big.Rat), big.NewRat(27, 1), f2),
		proofarith.AddRat(new(big.Rat), b0, proofarith.MulRat(new(big.Rat), big.NewRat(8, 1), b3)))
	b1 := new(big.Rat).Quo(proofarith.SubRat(new(big.Rat), proofarith.MulRat(new(big.Rat), big.NewRat(2, 1), a), b),
		big.NewRat(18, 1))
	b2 := new(big.Rat).Quo(proofarith.SubRat(new(big.Rat), proofarith.MulRat(new(big.Rat), big.NewRat(2, 1), b), a),
		big.NewRat(18, 1))
	if b0.Sign() > 0 && b1.Sign() > 0 && b2.Sign() > 0 && b3.Sign() > 0 {
		return true
	}
	if depth == 0 {
		return false
	}
	mid := new(big.Rat).Quo(proofarith.AddRat(new(big.Rat), lo, hi), big.NewRat(2, 1))
	return cubicPositive(f, lo, mid, depth-1) &&
		cubicPositive(f, mid, hi, depth-1)
}

func connectorCrossShift(c Connector, lo, hi, deviation *big.Rat) (*big.Rat, bool) {
	v0 := sourceSub(c.d[0], c.a[0])
	v1 := sourceSub(c.d[1], c.a[1])
	zero0 := SourceDot(v0, v0).Sign() == 0
	zero1 := SourceDot(v1, v1).Sign() == 0
	if zero0 == zero1 {
		return nil, false
	}
	v := v0
	if zero0 {
		v = v1
	}
	length := proofarith.FloatRat(proofbound.RatSqrtUp(SourceDot(v, v)))
	aLo := sourceLerp(c.a[0], c.a[1], lo)
	aHi := sourceLerp(c.a[0], c.a[1], hi)
	gLo := proofarith.MulRat(new(big.Rat), big.NewRat(2, 1), SourceDot(aLo, v))
	gHi := proofarith.MulRat(new(big.Rat), big.NewRat(2, 1), SourceDot(aHi, v))
	if gHi.Cmp(gLo) < 0 {
		gLo = gHi
	}
	if gLo.Sign() <= 0 {
		return nil, false
	}
	return new(big.Rat).Quo(proofarith.MulRat(new(big.Rat), deviation, length), gLo), true
}

func heldConnectorOf(verts []r3.Vec, cell, root int,
	source Connector) (Connector, bool) {
	var held Connector
	n := len(verts) / 2
	near, far := 0, 1
	if cell != 0 {
		near, far = root, root-1
	}
	for capIndex := range 2 {
		held.a[capIndex] = ExactXY(verts[capIndex*n+near])
		held.f[capIndex] = ExactXY(verts[capIndex*n+far])
		switch {
		case sameXY(source.d[capIndex], source.a[capIndex]):
			held.d[capIndex] = held.a[capIndex]
		case sameXY(source.d[capIndex], source.f[capIndex]):
			held.d[capIndex] = held.f[capIndex]
		default:
			return Connector{}, false
		}
	}
	return held, true
}

func sameXY(a, b XY) bool {
	return a.X.Cmp(b.X) == 0 && a.Y.Cmp(b.Y) == 0
}

func connectorReachPiece(c Connector, radius2, lo, hi, reach *big.Rat) bool {
	aLo := sourceLerp(c.a[0], c.a[1], lo)
	aHi := sourceLerp(c.a[0], c.a[1], hi)
	dLo := sourceLerp(c.d[0], c.d[1], lo)
	dHi := sourceLerp(c.d[0], c.d[1], hi)
	if SourceDot(aLo, aLo).Cmp(radius2) >= 0 ||
		SourceDot(aHi, aHi).Cmp(radius2) >= 0 ||
		sourceRadiusMin2(dLo, dHi).Cmp(radius2) <= 0 {
		return false
	}
	v0 := sourceSub(c.d[0], c.a[0])
	v1 := sourceSub(c.d[1], c.a[1])
	zero0 := SourceDot(v0, v0).Sign() == 0
	zero1 := SourceDot(v1, v1).Sign() == 0
	if zero0 == zero1 {
		return false
	}
	v := v0
	if zero0 {
		v = v1
	}
	length := proofarith.FloatRat(proofbound.RatSqrtUp(SourceDot(v, v)))
	if length.Sign() <= 0 {
		return false
	}
	gate := func(t *big.Rat) *big.Rat {
		a := sourceLerp(c.a[0], c.a[1], t)
		derivative := proofarith.MulRat(new(big.Rat), big.NewRat(2, 1), SourceDot(a, v))
		deficit := proofarith.SubRat(new(big.Rat), radius2, SourceDot(a, a))
		return proofarith.SubRat(new(big.Rat), proofarith.MulRat(new(big.Rat), reach, derivative),
			proofarith.MulRat(new(big.Rat), length, deficit))
	}
	// The radial derivative must remain strictly positive after removing the
	// cap's vanishing axial factor. This also proves a transverse crossing.
	if SourceDot(aLo, v).Sign() <= 0 || SourceDot(aHi, v).Sign() <= 0 {
		return false
	}
	return quadraticPositive(gate, lo, hi, 3)
}

// Exact quadratic Bernstein coefficients are sufficient on each interval.
// A fixed depth bounds work and lets a positive quadratic with a negative
// whole-interval middle coefficient pass after subdivision.
func quadraticPositive(f func(*big.Rat) *big.Rat, lo, hi *big.Rat, depth int) bool {
	mid := new(big.Rat).Quo(proofarith.AddRat(new(big.Rat), lo, hi), big.NewRat(2, 1))
	b0, bm, b2 := f(lo), f(mid), f(hi)
	b1 := proofarith.SubRat(new(big.Rat), proofarith.MulRat(new(big.Rat), big.NewRat(2, 1), bm),
		new(big.Rat).Quo(proofarith.AddRat(new(big.Rat), b0, b2), big.NewRat(2, 1)))
	if b0.Sign() > 0 && b1.Sign() > 0 && b2.Sign() > 0 {
		return true
	}
	return depth > 0 && quadraticPositive(f, lo, mid, depth-1) &&
		quadraticPositive(f, mid, hi, depth-1)
}
