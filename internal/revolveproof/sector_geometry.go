package revolveproof

import (
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/r3"
)

// RevolveForwardCollinear checks the exact forward turn of three recorded points.
func RevolveForwardCollinear(a, b, c sectionrecord.Point2) bool {
	ax := new(big.Rat).Sub(proofarith.FloatRat(b.U), proofarith.FloatRat(a.U))
	ay := new(big.Rat).Sub(proofarith.FloatRat(b.V), proofarith.FloatRat(a.V))
	bx := new(big.Rat).Sub(proofarith.FloatRat(c.U), proofarith.FloatRat(b.U))
	by := new(big.Rat).Sub(proofarith.FloatRat(c.V), proofarith.FloatRat(b.V))
	cross := new(big.Rat).Sub(new(big.Rat).Mul(ax, by), new(big.Rat).Mul(ay, bx))
	dot := new(big.Rat).Add(new(big.Rat).Mul(ax, bx), new(big.Rat).Mul(ay, by))
	return cross.Sign() == 0 && dot.Sign() > 0
}

// RevolveMeridianResidual bounds one recorded point against a cone meridian.
func RevolveMeridianResidual(point sectionrecord.Point2, slope, originX float64) float64 {
	r := new(big.Rat).Sub(proofarith.FloatRat(point.V),
		new(big.Rat).Mul(proofarith.FloatRat(slope),
			new(big.Rat).Sub(proofarith.FloatRat(point.U), proofarith.FloatRat(originX))))
	if r.Sign() < 0 {
		r.Neg(r)
	}
	return proofbound.RatFloatUp(r)
}

// RevolveImplicitDisplacement bounds the source fan's two-sided held-mesh
// departure against the reported cone, including the finite meridian gap.
func RevolveImplicitDisplacement(section facetedrecord.RevolveSectionCertificate,
	base facetedrecord.Payload) (float64, bool) {
	if !proofbound.FiniteVec(r3.Vec{X: base.MeshBound}) || base.MeshBound < 0 ||
		section.RootStart.U <= 0 || len(base.Verts) < 4 {
		return 0, false
	}
	minX, maxX, reach := math.Inf(1), math.Inf(-1), 0.0
	for _, world := range base.Verts[1:] {
		if !proofbound.FiniteVec(world) || world.X <= 0 {
			return 0, false
		}
		minX, maxX = min(minX, world.X), max(maxX, world.X)
		reach = max(reach, proofbound.DvLenUpper(proofbound.HeldDelta(world, r3.Vec{})))
	}
	minX = math.Nextafter(minX-base.MeshBound, math.Inf(-1))
	maxX = proofbound.AbsSumUpper(maxX, base.MeshBound)
	if !(minX > section.RootCone.Origin.X) {
		return 0, false
	}
	slope := section.ConeSection().Slope
	d := proofbound.AbsSumUpper(base.MeshBound,
		revolveMeridianExtensionGap(section, minX, maxX))
	r := proofbound.AbsSumUpper(reach, d)
	cost := proofbound.ProductUpper(proofbound.AbsSumUpper(1,
		proofbound.ProductUpper(slope, slope)),
		proofbound.ProductUpper(d, proofbound.AbsSumUpper(proofbound.ProductUpper(2, r), d)))
	return cost, !proofbound.IsNonFinite(cost)
}

func revolveMeridianExtensionGap(section facetedrecord.RevolveSectionCertificate, xLo, xHi float64) float64 {
	x0, x1 := proofarith.FloatRat(section.RootStart.U), proofarith.FloatRat(section.RootEnd.U)
	r0, r1 := proofarith.FloatRat(section.RootStart.V), proofarith.FloatRat(section.RootEnd.V)
	m := new(big.Rat).Quo(new(big.Rat).Sub(r1, r0), new(big.Rat).Sub(x1, x0))
	b := new(big.Rat).Sub(r0, new(big.Rat).Mul(m, x0))
	slope := proofarith.FloatRat(section.ConeSection().Slope)
	originX := proofarith.FloatRat(section.RootCone.Origin.X)
	gap := func(x float64) *big.Rat {
		value := new(big.Rat).Add(new(big.Rat).Mul(new(big.Rat).Sub(m, slope), proofarith.FloatRat(x)),
			new(big.Rat).Add(b, new(big.Rat).Mul(slope, originX)))
		return value.Abs(value)
	}
	a, z := gap(xLo), gap(xHi)
	if a.Cmp(z) < 0 {
		a = z
	}
	return proofbound.RatFloatUp(a)
}
