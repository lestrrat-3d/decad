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
	if pl.constructionDelta != 0 || pl.xform != r3.Identity() ||
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
	u, okU := loftCircleSectionLerp(bottom.Center.U, top.Center.U, fraction)
	v, okV := loftCircleSectionLerp(bottom.Center.V, top.Center.V, fraction)
	radius, okRadius := loftCircleSectionLerp(bottom.Radius.Base(), top.Radius.Base(), fraction)
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

func loftCircleSectionLerp(a, b float64, fraction *big.Rat) (float64, bool) {
	value := new(big.Rat).Add(proofarith.FloatRat(a),
		new(big.Rat).Mul(fraction, new(big.Rat).Sub(proofarith.FloatRat(b), proofarith.FloatRat(a))))
	held, exact := value.Float64()
	return held, exact && !math.IsInf(held, 0)
}
