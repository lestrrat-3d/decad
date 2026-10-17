package revolveloftfaces

import (
	"context"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/loftface"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/sketch"
)

// mappedArcRadialBand proves that the complete live Sketch arc maps to one
// axial coordinate and one radial implicit value at each LoftFaces band time.
// q is quadratic Bernstein data in exact rational band time. A root of q is
// therefore a contact of the whole mapped arc, not a unique angular root.
type mappedArcRadialBand struct {
	Arc     *sketch.Arc
	Segment int
	Band    int
	Q       [3]*big.Rat
	X       [2]*big.Rat
}

func MappedArcRadialTimeBand(ctx context.Context, trim *loftFacesTrimRecord,
	segmentIndex, band int, meridian revolveSectionSpan) (mappedArcRadialBand, bool) {
	var empty mappedArcRadialBand
	if ctx == nil || ctx.Err() != nil || trim == nil || trim.Source == nil ||
		trim.Source.Source == nil ||
		segmentIndex < 0 || segmentIndex >= len(trim.Source.Profile.Outer.Segments) ||
		segmentIndex >= len(trim.Source.Source.Entities) ||
		band < 0 || band+1 >= len(trim.Maps) ||
		meridian.Start.U == meridian.End.U ||
		meridian.Start.V <= 0 || meridian.End.V <= 0 {
		return empty, false
	}
	segment, ok := trim.Source.Profile.Outer.Segments[segmentIndex].(arcSeg)
	if !ok || !((segment.TStart == 0 && segment.TEnd == 1) ||
		(segment.TStart == 1 && segment.TEnd == 0)) {
		return empty, false
	}
	arc, ok := trim.Source.Source.Entities[segmentIndex].(*sketch.Arc)
	if !ok || !MappedArcRecordMatches(segment, arc) {
		return empty, false
	}
	uBox, vBox, err := arc.CertifiedSpanBounds(0, 1)
	if err != nil {
		return empty, false
	}
	sourceBox, err := LfSourceBox(trim.Source)
	if err != nil || proofarith.FloatRat(uBox.Lo).Cmp(sourceBox[0]) < 0 ||
		proofarith.FloatRat(uBox.Hi).Cmp(sourceBox[1]) > 0 ||
		proofarith.FloatRat(vBox.Lo).Cmp(sourceBox[2]) < 0 ||
		proofarith.FloatRat(vBox.Hi).Cmp(sourceBox[3]) > 0 {
		return empty, false
	}
	centerU, centerV := proofarith.FloatRat(arc.Center.X()), proofarith.FloatRat(arc.Center.Y())
	startU, startV := proofarith.FloatRat(arc.Start.X()), proofarith.FloatRat(arc.Start.Y())
	endU, endV := proofarith.FloatRat(arc.End.X()), proofarith.FloatRat(arc.End.Y())
	radius2 := loftface.RadiusSquare(startU, startV, centerU, centerV)
	if radius2.Sign() <= 0 ||
		loftface.RadiusSquare(endU, endV, centerU, centerV).Cmp(radius2) != 0 {
		return empty, false
	}
	var mappedCaps [2]loftface.RadialCap
	for index, mapped := range [...]loftFaceMap{trim.Maps[band], trim.Maps[band+1]} {
		if mapped.G0.Sign() <= 0 || mapped.GU.Sign() != 0 || mapped.GV.Sign() != 0 ||
			mapped.QU[0].Sign() != 0 || mapped.QV[0].Sign() != 0 {
			return empty, false
		}
		atCenter := mapped.At(centerU, centerV)
		if atCenter[1].Sign() != 0 || atCenter[2].Sign() != 0 {
			return empty, false
		}
		mappedCaps[index] = loftface.RadialCap{X: atCenter[0],
			U: [2]*big.Rat{proofarith.QuoRat(new(big.Rat), mapped.QU[1], mapped.G0), proofarith.QuoRat(new(big.Rat), mapped.QU[2], mapped.G0)},
			V: [2]*big.Rat{proofarith.QuoRat(new(big.Rat), mapped.QV[1], mapped.G0), proofarith.QuoRat(new(big.Rat), mapped.QV[2], mapped.G0)}}
	}
	lambda0, ok0 := loftface.RadialScale(mappedCaps, new(big.Rat))
	lambdaMid, okMid := loftface.RadialScale(mappedCaps, big.NewRat(1, 2))
	lambda2, ok2 := loftface.RadialScale(mappedCaps, big.NewRat(1, 1))
	lambda1 := proofarith.SubRat(new(big.Rat), proofarith.MulRat(new(big.Rat), big.NewRat(2, 1), lambdaMid),
		proofarith.MulRat(new(big.Rat), big.NewRat(1, 2), proofarith.AddRat(new(big.Rat), lambda0, lambda2)))
	if !ok0 || !okMid || !ok2 ||
		lambda0.Sign() <= 0 || lambda1.Sign() < 0 || lambda2.Sign() <= 0 {
		return empty, false
	}
	x0, x2 := mappedCaps[0].X, mappedCaps[1].X
	xMid := proofarith.MulRat(new(big.Rat), big.NewRat(1, 2), proofarith.AddRat(new(big.Rat), x0, x2))
	sectionX0, sectionX1 := proofarith.FloatRat(meridian.Start.U), proofarith.FloatRat(meridian.End.U)
	sectionR0, sectionR1 := proofarith.FloatRat(meridian.Start.V), proofarith.FloatRat(meridian.End.V)
	m := proofarith.QuoRat(new(big.Rat), proofarith.SubRat(new(big.Rat), sectionR1, sectionR0), proofarith.SubRat(new(big.Rat), sectionX1, sectionX0))
	b := proofarith.SubRat(new(big.Rat), sectionR0, proofarith.MulRat(new(big.Rat), m, sectionX0))
	implicit := func(lambda, x *big.Rat) *big.Rat {
		radius := proofarith.AddRat(new(big.Rat), proofarith.MulRat(new(big.Rat), m, x), b)
		return proofarith.SubRat(new(big.Rat), proofarith.MulRat(new(big.Rat), lambda, radius2), proofarith.MulRat(new(big.Rat), radius, radius))
	}
	q0, qMid, q2 := implicit(lambda0, x0), implicit(lambdaMid, xMid), implicit(lambda2, x2)
	q1 := proofarith.SubRat(new(big.Rat), proofarith.MulRat(new(big.Rat), big.NewRat(2, 1), qMid),
		proofarith.MulRat(new(big.Rat), big.NewRat(1, 2), proofarith.AddRat(new(big.Rat), q0, q2)))
	return mappedArcRadialBand{Arc: arc, Segment: segmentIndex, Band: band,
		Q: [3]*big.Rat{q0, q1, q2}, X: [2]*big.Rat{x0, x2}}, true
}
