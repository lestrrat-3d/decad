package spiralsector

import (
	"math/big"

	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/loftface"
	"github.com/lestrrat-3d/decad/internal/revolveloftfaces"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
)

// SourcePhysicalSector proves the whole ruled source stays in positive Y
// and above zero signed radius on the blank's root meridian. The opposite
// root-circle arc passes negative Y; a squared-implicit root with negative
// signed radius is not a physical cone crossing.
func SourcePhysicalSector(trim *loftFacesTrimRecord, m, b *big.Rat) bool {
	box, err := revolveloftfaces.LfSourceBox(trim.Source)
	if err != nil {
		return false
	}
	corners := loftface.BoxCorners(box)
	y := loftFaceRatVec{seamRatZero(), seamRatOne(), seamRatZero()}
	x := loftFaceRatVec{seamRatOne(), seamRatZero(), seamRatZero()}
	for _, mapped := range trim.Maps {
		if loftface.ScalarRange(mapped, y, corners).Lo.Sign() <= 0 {
			return false
		}
		xRange := loftface.ScalarRange(mapped, x, corners)
		r0, r1 := seamRatAdd(seamRatMul(m, xRange.Lo), b), seamRatAdd(seamRatMul(m, xRange.Hi), b)
		if r0.Sign() <= 0 || r1.Sign() <= 0 {
			return false
		}
	}
	return true
}

// RootToolX returns the exact axial coordinate where the blank root
// r=m*x+b intersects the tool cone generator x+k*r=apex.
func RootToolX(m, b *big.Rat, cone *facetedrecord.PointConeLimit) (*big.Rat, bool) {
	if cone == nil || cone.Apex == nil || cone.Slope == nil || cone.Slope.Sign() <= 0 {
		return nil, false
	}
	denominator := seamRatAdd(seamRatOne(), seamRatMul(cone.Slope, m))
	if denominator.Sign() <= 0 {
		return nil, false
	}
	return seamRatDiv(seamRatSub(cone.Apex, seamRatMul(cone.Slope, b)), denominator), true
}

// FiniteConeSide proves a straight cone meridian stays on one side over
// its complete finite generator. The far-plane and apex checks keep that
// sign on the operative cone wall.
func FiniteConeSide(cone *facetedrecord.PointConeLimit, first, last sectionrecord.Point2, sign int) bool {
	if cone == nil || cone.Apex == nil || cone.Far == nil || cone.Slope == nil ||
		cone.Slope.Sign() <= 0 || cone.Far.Sign() >= 0 ||
		first.U <= 0 || last.U <= first.U || first.V <= 0 || last.V <= 0 ||
		seamRatF(first.U).Cmp(cone.Far) <= 0 || seamRatF(last.U).Cmp(cone.Apex) >= 0 {
		return false
	}
	residual := func(p sectionrecord.Point2) *big.Rat {
		return seamRatSub(seamRatAdd(seamRatF(p.U), seamRatMul(cone.Slope, seamRatF(p.V))), cone.Apex)
	}
	return residual(first).Sign() == sign && residual(last).Sign() == sign
}

// NearTrimSides proves a LoftFaces wall meets a blank cap only outside
// the toe Cut and inside the heel Intersect. Exact meridian signs cover
// both finite blank outer caps. They also separate the matching tool-cap
// faces, the toe cap from the heel tool, and the whole finite blank root
// from both tool caps.
func NearTrimSides(proof *spiralFirstJoinSource) bool {
	if proof == nil || proof.Blank == nil || proof.Tooth == nil ||
		proof.Tooth.Toe == nil || proof.Tooth.Heel == nil ||
		proof.ToeCapGap == nil || proof.HeelCapGap == nil ||
		proof.ToeCapGap.Sign() <= 0 || proof.HeelCapGap.Sign() <= 0 ||
		proof.ToeOffset == nil || proof.HeelOffset == nil ||
		proof.ToeOffset.Sign() >= 0 || proof.HeelOffset.Sign() <= 0 {
		return false
	}
	blank := proof.Blank
	return FiniteConeSide(proof.Tooth.Toe, blank.ToeRoot, blank.ToeOuter, 1) &&
		FiniteConeSide(proof.Tooth.Heel, blank.ToeRoot, blank.ToeOuter, -1) &&
		FiniteConeSide(proof.Tooth.Toe, blank.Ded, blank.HeelOuter, 1) &&
		FiniteConeSide(proof.Tooth.Heel, blank.Ded, blank.HeelOuter, -1) &&
		FiniteConeSide(proof.Tooth.Toe, blank.ToeRoot, blank.Ded, 1) &&
		FiniteConeSide(proof.Tooth.Heel, blank.ToeRoot, blank.Ded, -1)
}
