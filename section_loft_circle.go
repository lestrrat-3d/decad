package decad

import (
	"math"
	"math/big"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// loftCircleSectionRecord reads a parallel section directly from an
// authenticated circle-pair Loft. Its identical outer records denote the
// same region boundary at every level. Equal-angle correspondence makes the
// moving hole a circle with linearly interpolated center and radius.
func loftCircleSectionRecord(pl loftPayload, plane r3.Frame) (profileRecord, bool) {
	// An exact-axis Cut is refused by analyticBore. The direct-circle gate
	// refuses non-analytic Cut and Fillet rewrites; their markers remain
	// reject-only backstops if those gates change.
	if pl.constructionDelta != 0 || len(pl.blendSegs) != 0 ||
		pl.throughBoreCut || pl.analyticBore || pl.xform != r3.Identity() ||
		directLoftCircularHole(pl) == loftCircleWallNone {
		return profileRecord{}, false
	}
	x, y, z := r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0), r3.NewVec(0, 0, 1)
	if pl.frame0.U() != x || pl.frame0.V() != y || pl.frame0.N() != z ||
		pl.frame1.U() != x || pl.frame1.V() != y || pl.frame1.N() != z ||
		plane.U() != x || plane.V() != y || plane.N() != z {
		return profileRecord{}, false
	}
	if pl.frame0.Origin().X != pl.frame1.Origin().X ||
		pl.frame0.Origin().Y != pl.frame1.Origin().Y ||
		pl.frame0.Origin().X != plane.Origin().X ||
		pl.frame0.Origin().Y != plane.Origin().Y ||
		pl.plane0.Origin.Z != pl.frame0.Origin().Z ||
		pl.plane1.Origin.Z != pl.frame1.Origin().Z {
		return profileRecord{}, false
	}
	z0, z1 := proofarith.FloatRat(pl.frame0.Origin().Z), proofarith.FloatRat(pl.frame1.Origin().Z)
	cut := proofarith.FloatRat(plane.Origin().Z)
	if z0 == nil || z1 == nil || cut == nil || cut.Cmp(z0) <= 0 || cut.Cmp(z1) >= 0 {
		return profileRecord{}, false
	}
	fraction := new(big.Rat).Quo(new(big.Rat).Sub(cut, z0), new(big.Rat).Sub(z1, z0))
	bottom, okBottom := pl.profile0.Holes[0].Segments[0].(circleSeg)
	top, okTop := pl.profile1.Holes[0].Segments[0].(circleSeg)
	if !okBottom || !okTop {
		return profileRecord{}, false
	}
	u, okU := loftSectionLerpExact(bottom.Center.U, top.Center.U, fraction)
	v, okV := loftSectionLerpExact(bottom.Center.V, top.Center.V, fraction)
	radius, okRadius := loftSectionLerpExact(bottom.Radius.Base(), top.Radius.Base(), fraction)
	if !okU || !okV || !okRadius || radius <= 0 {
		return profileRecord{}, false
	}
	hole := bottom
	hole.Center.U, hole.Center.V = u, v
	hole.Radius = units.Millimeters(radius)
	return profileRecord{
		Outer: pl.profile0.Outer,
		Holes: []loopRecord{{Segments: []curveSegment{hole}}},
	}, true
}

// loftVaryingCircularOuterSectionRecord intersects equal-angle circle pairs
// from two authenticated source profiles. At a fixed height their ruled
// boundary is a circle with linearly interpolated center and radius.
func loftVaryingCircularOuterSectionRecord(pl loftPayload, plane r3.Frame) (profileRecord, bool) {
	// Cut/Fillet markers, invalid alignment shapes and plane/frame disagreement
	// are reject-only for current direct Loft producers.
	if !pl.authenticatedReconstruction || pl.surfaceResult ||
		pl.constructionDelta != 0 || len(pl.blendSegs) != 0 ||
		pl.throughBoreCut || pl.analyticBore || pl.xform != r3.Identity() ||
		len(pl.profile0.Holes) != len(pl.profile1.Holes) ||
		(len(pl.alignment) != 0 && len(pl.alignment) != 1+len(pl.profile0.Holes)) {
		return profileRecord{}, false
	}
	for _, offset := range pl.alignment {
		if offset != 0 {
			return profileRecord{}, false
		}
	}
	x, y, z := r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0), r3.NewVec(0, 0, 1)
	if pl.frame0.U() != x || pl.frame0.V() != y || pl.frame0.N() != z ||
		pl.frame1.U() != x || pl.frame1.V() != y || pl.frame1.N() != z ||
		plane.U() != x || plane.V() != y || plane.N() != z ||
		pl.plane0.U != x || pl.plane0.V != y ||
		pl.plane1.U != x || pl.plane1.V != y ||
		pl.plane0.Origin != pl.frame0.Origin() || pl.plane1.Origin != pl.frame1.Origin() ||
		pl.frame0.Origin().X != pl.frame1.Origin().X ||
		pl.frame0.Origin().Y != pl.frame1.Origin().Y ||
		pl.frame0.Origin().X != plane.Origin().X ||
		pl.frame0.Origin().Y != plane.Origin().Y {
		return profileRecord{}, false
	}
	z0, z1 := proofarith.FloatRat(pl.frame0.Origin().Z), proofarith.FloatRat(pl.frame1.Origin().Z)
	cut := proofarith.FloatRat(plane.Origin().Z)
	if z0 == nil || z1 == nil || cut == nil || cut.Cmp(z0) <= 0 || cut.Cmp(z1) >= 0 {
		return profileRecord{}, false
	}
	fraction := new(big.Rat).Quo(new(big.Rat).Sub(cut, z0), new(big.Rat).Sub(z1, z0))
	outer, outerPair, ok := loftSectionCirclePair(pl.profile0.Outer, pl.profile1.Outer, true, fraction)
	if !ok {
		return profileRecord{}, false
	}
	profile := profileRecord{Outer: outer, Holes: make([]loopRecord, len(pl.profile0.Holes))}
	holePairs := make([][2]circleSeg, len(profile.Holes))
	for i := range profile.Holes {
		hole, pair, ok := loftSectionCirclePair(pl.profile0.Holes[i], pl.profile1.Holes[i], false, fraction)
		if !ok || !loftSectionCircleGapAllLevels(outerPair, pair, true) {
			return profileRecord{}, false
		}
		for j := range i {
			if !loftSectionCircleGapAllLevels(pair, holePairs[j], false) {
				return profileRecord{}, false
			}
		}
		profile.Holes[i] = hole
		holePairs[i] = pair
	}
	return profile, true
}

func loftSectionCirclePair(lower, upper loopRecord, ccw bool, fraction *big.Rat) (loopRecord, [2]circleSeg, bool) {
	if len(lower.Segments) != 1 || len(upper.Segments) != 1 {
		return loopRecord{}, [2]circleSeg{}, false
	}
	bottom, okBottom := lower.Segments[0].(circleSeg)
	top, okTop := upper.Segments[0].(circleSeg)
	start, end := 0.0, 1.0
	if !ccw {
		start, end = 1, 0
	}
	if !okBottom || !okTop || bottom.CCW != ccw || top.CCW != ccw ||
		bottom.TStart != start || bottom.TEnd != end ||
		top.TStart != start || top.TEnd != end ||
		!loftSectionFiniteCircle(bottom) || !loftSectionFiniteCircle(top) {
		return loopRecord{}, [2]circleSeg{}, false
	}
	u, okU := loftSectionLerpExact(bottom.Center.U, top.Center.U, fraction)
	v, okV := loftSectionLerpExact(bottom.Center.V, top.Center.V, fraction)
	radius, okRadius := loftSectionLerpExact(bottom.Radius.Base(), top.Radius.Base(), fraction)
	if !okU || !okV || !okRadius || radius <= 0 {
		return loopRecord{}, [2]circleSeg{}, false
	}
	circle := bottom
	circle.Center.U, circle.Center.V = u, v
	circle.Radius = units.Millimeters(radius)
	return loopRecord{Segments: []curveSegment{circle}}, [2]circleSeg{bottom, top}, true
}

// loftSectionCircleGapAllLevels proves two paired circles remain nested or
// separated for the whole Loft. Both center differences and the radius gap
// or sum are linear in the sweep fraction, so their squared gap is quadratic.
func loftSectionCircleGapAllLevels(a, b [2]circleSeg, contained bool) bool {
	x := [2]*big.Rat{}
	y := [2]*big.Rat{}
	r := [2]*big.Rat{}
	for i := range a {
		x[i] = new(big.Rat).Sub(proofarith.FloatRat(a[i].Center.U), proofarith.FloatRat(b[i].Center.U))
		y[i] = new(big.Rat).Sub(proofarith.FloatRat(a[i].Center.V), proofarith.FloatRat(b[i].Center.V))
		r[i] = proofarith.FloatRat(a[i].Radius.Base())
		if contained {
			r[i].Sub(r[i], proofarith.FloatRat(b[i].Radius.Base()))
		} else {
			r[i].Add(r[i], proofarith.FloatRat(b[i].Radius.Base()))
		}
		if r[i].Sign() <= 0 {
			return false
		}
	}
	dx := new(big.Rat).Sub(x[1], x[0])
	dy := new(big.Rat).Sub(y[1], y[0])
	dr := new(big.Rat).Sub(r[1], r[0])
	sq := func(v *big.Rat) *big.Rat { return new(big.Rat).Mul(v, v) }
	quadratic := new(big.Rat).Sub(new(big.Rat).Add(sq(dx), sq(dy)), sq(dr))
	linear := new(big.Rat).Sub(new(big.Rat).Add(
		new(big.Rat).Mul(x[0], dx), new(big.Rat).Mul(y[0], dy)), new(big.Rat).Mul(r[0], dr))
	linear.Mul(linear, big.NewRat(2, 1))
	constant := new(big.Rat).Sub(new(big.Rat).Add(sq(x[0]), sq(y[0])), sq(r[0]))
	if contained {
		quadratic.Neg(quadratic)
		linear.Neg(linear)
		constant.Neg(constant)
	}
	return loftSectionQuadraticPositive(quadratic, linear, constant)
}

// loftSectionQuadraticPositive requires a·t²+b·t+c > 0 on closed [0,1].
func loftSectionQuadraticPositive(a, b, c *big.Rat) bool {
	atOne := new(big.Rat).Add(new(big.Rat).Add(a, b), c)
	if c.Sign() <= 0 || atOne.Sign() <= 0 {
		return false
	}
	if a.Sign() <= 0 || b.Sign() >= 0 || b.Cmp(new(big.Rat).Neg(new(big.Rat).Mul(a, big.NewRat(2, 1)))) <= 0 {
		return true
	}
	// The vertex lies inside (0,1), and 4a times its value is 4ac-b².
	minimum := new(big.Rat).Sub(new(big.Rat).Mul(big.NewRat(4, 1), new(big.Rat).Mul(a, c)),
		new(big.Rat).Mul(b, b))
	return minimum.Sign() > 0
}

func loftSectionFiniteCircle(circle circleSeg) bool {
	return !math.IsNaN(circle.Center.U) && !math.IsInf(circle.Center.U, 0) &&
		!math.IsNaN(circle.Center.V) && !math.IsInf(circle.Center.V, 0) &&
		!math.IsNaN(circle.Radius.Base()) && !math.IsInf(circle.Radius.Base(), 0) &&
		circle.Radius.Base() > 0
}

// loftSectionLerpExact returns an interpolation only when binary64 holds its
// exact rational value.
func loftSectionLerpExact(a, b float64, fraction *big.Rat) (float64, bool) {
	value := new(big.Rat).Add(proofarith.FloatRat(a),
		new(big.Rat).Mul(fraction, new(big.Rat).Sub(proofarith.FloatRat(b), proofarith.FloatRat(a))))
	held, exact := value.Float64()
	return held, exact && !math.IsInf(held, 0)
}
