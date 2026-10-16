package prismpattern

import (
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/r3"
)

// A sector encloses the complete source change at one corner: the connector
// cutback, circular-wall cutback, and analytic fillet arc. It is periodic.
type filletSector struct{ lo, hi *big.Rat }

func filletSectorOf(rec *facetedrecord.PrismPatternJoinRecord,
	contact FilletContact, radius float64) (filletSector, error) {
	var empty filletSector
	part := rec.Parts[contact.part]
	rootPoint := rec.Source.RootArc.Start
	if contact.station == part.RootStart {
		rootPoint = rec.Source.RootArc.End
	}
	root := exactPlacedPoint(part, rootPoint)
	center := contact.centerIV
	line := contact.lineIV
	// Q is the nearest point of the fillet circle to the blank center.
	// P lies on its near half. Therefore the short P-to-Q arc remains on
	// one side of the center ray and inside that circle's tangent cone.
	if center.Dot(line.Sub(center)).Hi.Sign() >= 0 {
		return empty, fmt.Errorf("%w: joined-prism fillet has no near-side arc", decaderr.ErrUnsupported)
	}
	cross := proofbound.IntervalSub(proofbound.IntervalMul(center.X, line.Y),
		proofbound.IntervalMul(center.Y, line.X))
	if contact.station == 0 && cross.Lo.Sign() <= 0 ||
		contact.station == part.RootStart && cross.Hi.Sign() >= 0 {
		return empty, fmt.Errorf("%w: joined-prism fillet arc has no sided polar sweep", decaderr.ErrUnsupported)
	}
	centerNorm, err := IVSqrt(center.Dot(center))
	r := proofarith.FloatRat(radius)
	if err != nil || centerNorm.Lo.Cmp(r) <= 0 {
		return empty, fmt.Errorf("%w: joined-prism fillet circle reaches the axis", decaderr.ErrUnsupported)
	}
	tangentSq := new(big.Rat).Sub(new(big.Rat).Mul(centerNorm.Lo, centerNorm.Lo),
		new(big.Rat).Mul(r, r))
	tangentLo := proofbound.RatSqrtDown(tangentSq)
	if tangentLo <= 0 {
		return empty, fmt.Errorf("%w: joined-prism fillet has no polar tangent bound", decaderr.ErrUnsupported)
	}
	delta := proofbound.Atan2Interval(r, proofarith.FloatRat(tangentLo), false).Hi
	cAngle, err := HeldAngle(contact.center, contact.pointError)
	if err != nil {
		return empty, err
	}
	sector := filletSector{lo: cAngle.Lo, hi: cAngle.Hi}
	if contact.station == 0 {
		sector.hi = new(big.Rat).Add(sector.hi, delta)
	} else {
		sector.lo = new(big.Rat).Sub(sector.lo, delta)
	}
	for _, angleOf := range []struct {
		point r3.Vec
		iv    IVPoint
		held  bool
	}{{iv: root}, {point: contact.line, held: true}, {point: contact.circle, held: true}} {
		var angle proofbound.RatInterval
		if angleOf.held {
			angle, err = HeldAngle(angleOf.point, contact.pointError)
		} else {
			angle, err = ExactAngle(angleOf.iv)
		}
		if err != nil {
			return empty, err
		}
		angle, ok := AngleNear(angle, cAngle)
		if !ok {
			return empty, fmt.Errorf("%w: joined-prism fillet contact leaves its polar sector", decaderr.ErrUnsupported)
		}
		if angle.Lo.Cmp(sector.lo) < 0 {
			sector.lo = angle.Lo
		}
		if angle.Hi.Cmp(sector.hi) > 0 {
			sector.hi = angle.Hi
		}
	}
	width := new(big.Rat).Sub(sector.hi, sector.lo)
	if width.Sign() <= 0 || width.Cmp(proofbound.HalfPiInterval().Lo) >= 0 {
		return empty, fmt.Errorf("%w: joined-prism fillet has no narrow polar sector", decaderr.ErrUnsupported)
	}
	return sector, nil
}

func filletConnectorSector(part facetedrecord.PrismPatternPart,
	line sectionrecord.LineSeg) (filletSector, error) {
	var empty filletSector
	a, err := ExactAngle(exactPlacedPoint(part, line.Start))
	if err != nil {
		return empty, err
	}
	b, err := ExactAngle(exactPlacedPoint(part, line.End))
	if err != nil {
		return empty, err
	}
	b, ok := AngleNear(b, a)
	if !ok {
		return empty, fmt.Errorf("%w: joined-prism connector has no narrow polar sector", decaderr.ErrUnsupported)
	}
	lo, hi := a.Lo, a.Hi
	if b.Lo.Cmp(lo) < 0 {
		lo = b.Lo
	}
	if b.Hi.Cmp(hi) > 0 {
		hi = b.Hi
	}
	return filletSector{lo: lo, hi: hi}, nil
}

func filletSectorsSeparate(a, b filletSector) bool {
	for k := int64(-2); k <= 2; k++ {
		shift := proofbound.IntervalScale(proofbound.TwoPiInterval(), big.NewRat(k, 1))
		lo := new(big.Rat).Add(b.lo, shift.Lo)
		hi := new(big.Rat).Add(b.hi, shift.Hi)
		if a.hi.Cmp(lo) >= 0 && hi.Cmp(a.lo) >= 0 {
			return false
		}
	}
	return true
}
