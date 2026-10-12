package decad

import (
	"context"
	"math/big"

	"github.com/lestrrat-3d/sketch"
)

// mappedArcQDerivativeSignCell proves the sign of the mapped meridian
// implicit's derivative along a live Sketch arc's CCW angle. It covers one
// source slice and one rational LoftFaces band-time slab. A strict sign proves
// that the implicit has at most one root along that source slice at each time;
// endpoint signs, axial clipping, and adjoining-curve continuity are separate.
func mappedArcQDerivativeSignCell(ctx context.Context, trim *loftFacesTrimRecord,
	segmentIndex int, meridian revolveSectionSpan, band int,
	sLo, sHi float64, tLo, tHi *big.Rat) (int, bool) {
	if ctx == nil || ctx.Err() != nil || trim == nil || trim.source == nil ||
		trim.source.source == nil ||
		segmentIndex < 0 || segmentIndex >= len(trim.source.profile.Outer.Segments) ||
		segmentIndex >= len(trim.source.source.entities) ||
		band < 0 || band+1 >= len(trim.maps) ||
		sLo < 0 || sHi > 1 || sLo > sHi ||
		tLo == nil || tHi == nil || tLo.Sign() < 0 ||
		tLo.Cmp(tHi) > 0 || tHi.Cmp(lfOne()) > 0 ||
		meridian.start.U == meridian.end.U ||
		meridian.start.V <= 0 || meridian.end.V <= 0 {
		return 0, false
	}
	segment, ok := trim.source.profile.Outer.Segments[segmentIndex].(arcSeg)
	if !ok || !((segment.TStart == 0 && segment.TEnd == 1) ||
		(segment.TStart == 1 && segment.TEnd == 0)) {
		return 0, false
	}
	arc, ok := trim.source.source.entities[segmentIndex].(*sketch.Arc)
	if !ok || !mappedArcRecordMatches(segment, arc) {
		return 0, false
	}
	u, v, err := arc.CertifiedSpanBounds(sLo, sHi)
	if err != nil {
		return 0, false
	}
	box := [4]*big.Rat{lfF(u.Lo), lfF(u.Hi), lfF(v.Lo), lfF(v.Hi)}
	if sLo == 0 {
		point := segment.Start
		box[0], box[1] = ratMin(box[0], lfF(point.U)), ratMax(box[1], lfF(point.U))
		box[2], box[3] = ratMin(box[2], lfF(point.V)), ratMax(box[3], lfF(point.V))
	}
	if sHi == 1 {
		point := segment.End
		box[0], box[1] = ratMin(box[0], lfF(point.U)), ratMax(box[1], lfF(point.U))
		box[2], box[3] = ratMin(box[2], lfF(point.V)), ratMax(box[3], lfF(point.V))
	}
	sourceBox, err := lfSourceBox(trim.source)
	if err != nil || box[0].Cmp(sourceBox[0]) < 0 || box[1].Cmp(sourceBox[1]) > 0 ||
		box[2].Cmp(sourceBox[2]) < 0 || box[3].Cmp(sourceBox[3]) > 0 {
		return 0, false
	}
	corners := lfBoxCorners(box)
	// For a CCW circular carrier, d(u,v)/d(theta) = (centerV-v, u-centerU).
	du := loftFaceInterval{lfSub(lfF(arc.Center.Y()), box[3]),
		lfSub(lfF(arc.Center.Y()), box[2])}
	dv := loftFaceInterval{lfSub(box[0], lfF(arc.Center.X())),
		lfSub(box[1], lfF(arc.Center.X()))}
	var cap [2][3]loftFaceInterval
	var capDerivative [2][3]loftFaceInterval
	for capIndex, mapped := range [...]loftFaceMap{trim.maps[band], trim.maps[band+1]} {
		cap[capIndex] = mappedArcCapRanges(mapped, corners)
		for axis := range cap[capIndex] {
			var row loftFaceRatVec
			row[0], row[1], row[2] = lfZero(), lfZero(), lfZero()
			row[axis] = lfOne()
			partialU, errU := lfDerivRange(mapped, row, 0, corners)
			partialV, errV := lfDerivRange(mapped, row, 1, corners)
			if errU != nil || errV != nil {
				return 0, false
			}
			capDerivative[capIndex][axis] = mappedArcIntervalAdd(
				mappedArcIntervalMul(partialU, du), mappedArcIntervalMul(partialV, dv))
		}
	}
	var value, derivative [3]loftFaceInterval
	for axis := range value {
		value[axis] = lfIntervalHull(mappedArcBlend(cap[0][axis], cap[1][axis], tLo),
			mappedArcBlend(cap[0][axis], cap[1][axis], tHi))
		derivative[axis] = lfIntervalHull(
			mappedArcBlend(capDerivative[0][axis], capDerivative[1][axis], tLo),
			mappedArcBlend(capDerivative[0][axis], capDerivative[1][axis], tHi))
	}
	x0, x1 := lfF(meridian.start.U), lfF(meridian.end.U)
	r0, r1 := lfF(meridian.start.V), lfF(meridian.end.V)
	m := lfDiv(lfSub(r1, r0), lfSub(x1, x0))
	b := lfSub(r0, lfMul(m, x0))
	radius := mappedArcIntervalAdd(mappedArcIntervalScale(value[0], m),
		loftFaceInterval{b, b})
	radiusDerivative := mappedArcIntervalScale(derivative[0], m)
	qDerivative := mappedArcIntervalSub(mappedArcIntervalAdd(
		mappedArcIntervalMul(value[1], derivative[1]),
		mappedArcIntervalMul(value[2], derivative[2])),
		mappedArcIntervalMul(radius, radiusDerivative))
	if qDerivative.lo.Sign() > 0 {
		return 1, true
	}
	if qDerivative.hi.Sign() < 0 {
		return -1, true
	}
	return 0, false
}

func mappedArcIntervalAdd(a, b loftFaceInterval) loftFaceInterval {
	return loftFaceInterval{lfAdd(a.lo, b.lo), lfAdd(a.hi, b.hi)}
}

func mappedArcIntervalSub(a, b loftFaceInterval) loftFaceInterval {
	return loftFaceInterval{lfSub(a.lo, b.hi), lfSub(a.hi, b.lo)}
}

func mappedArcIntervalMul(a, b loftFaceInterval) loftFaceInterval {
	products := [4]*big.Rat{lfMul(a.lo, b.lo), lfMul(a.lo, b.hi),
		lfMul(a.hi, b.lo), lfMul(a.hi, b.hi)}
	out := loftFaceInterval{products[0], products[0]}
	for _, value := range products[1:] {
		out.lo, out.hi = ratMin(out.lo, value), ratMax(out.hi, value)
	}
	return out
}

func mappedArcIntervalScale(a loftFaceInterval, scale *big.Rat) loftFaceInterval {
	return mappedArcIntervalMul(a, loftFaceInterval{scale, scale})
}
