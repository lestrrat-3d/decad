package decad

import (
	"context"
	"math/big"

	"github.com/lestrrat-3d/sketch"
)

// mappedArcRadialBand proves that the complete live Sketch arc maps to one
// axial coordinate and one radial implicit value at each LoftFaces band time.
// q is quadratic Bernstein data in exact rational band time. A root of q is
// therefore a contact of the whole mapped arc, not a unique angular root.
type mappedArcRadialBand struct {
	arc     *sketch.Arc
	segment int
	band    int
	q       [3]*big.Rat
	x       [2]*big.Rat
}

type mappedArcRadialCap struct {
	X    *big.Rat
	U, V [2]*big.Rat
}

func mappedArcRadialTimeBand(ctx context.Context, trim *loftFacesTrimRecord,
	segmentIndex, band int, meridian revolveSectionSpan) (mappedArcRadialBand, bool) {
	var empty mappedArcRadialBand
	if ctx == nil || ctx.Err() != nil || trim == nil || trim.source == nil ||
		trim.source.source == nil ||
		segmentIndex < 0 || segmentIndex >= len(trim.source.profile.Outer.Segments) ||
		segmentIndex >= len(trim.source.source.entities) ||
		band < 0 || band+1 >= len(trim.maps) ||
		meridian.start.U == meridian.end.U ||
		meridian.start.V <= 0 || meridian.end.V <= 0 {
		return empty, false
	}
	segment, ok := trim.source.profile.Outer.Segments[segmentIndex].(arcSeg)
	if !ok || !((segment.TStart == 0 && segment.TEnd == 1) ||
		(segment.TStart == 1 && segment.TEnd == 0)) {
		return empty, false
	}
	arc, ok := trim.source.source.entities[segmentIndex].(*sketch.Arc)
	if !ok || !mappedArcRecordMatches(segment, arc) {
		return empty, false
	}
	uBox, vBox, err := arc.CertifiedSpanBounds(0, 1)
	if err != nil {
		return empty, false
	}
	sourceBox, err := lfSourceBox(trim.source)
	if err != nil || lfF(uBox.Lo).Cmp(sourceBox[0]) < 0 ||
		lfF(uBox.Hi).Cmp(sourceBox[1]) > 0 ||
		lfF(vBox.Lo).Cmp(sourceBox[2]) < 0 ||
		lfF(vBox.Hi).Cmp(sourceBox[3]) > 0 {
		return empty, false
	}
	centerU, centerV := lfF(arc.Center.X()), lfF(arc.Center.Y())
	startU, startV := lfF(arc.Start.X()), lfF(arc.Start.Y())
	endU, endV := lfF(arc.End.X()), lfF(arc.End.Y())
	radius2 := mappedArcRadiusSquare(startU, startV, centerU, centerV)
	if radius2.Sign() <= 0 ||
		mappedArcRadiusSquare(endU, endV, centerU, centerV).Cmp(radius2) != 0 {
		return empty, false
	}
	var mappedCaps [2]mappedArcRadialCap
	for index, mapped := range [...]loftFaceMap{trim.maps[band], trim.maps[band+1]} {
		if mapped.G0.Sign() <= 0 || mapped.GU.Sign() != 0 || mapped.GV.Sign() != 0 ||
			mapped.QU[0].Sign() != 0 || mapped.QV[0].Sign() != 0 {
			return empty, false
		}
		atCenter := mapped.At(centerU, centerV)
		if atCenter[1].Sign() != 0 || atCenter[2].Sign() != 0 {
			return empty, false
		}
		mappedCaps[index] = mappedArcRadialCap{X: atCenter[0],
			U: [2]*big.Rat{lfDiv(mapped.QU[1], mapped.G0), lfDiv(mapped.QU[2], mapped.G0)},
			V: [2]*big.Rat{lfDiv(mapped.QV[1], mapped.G0), lfDiv(mapped.QV[2], mapped.G0)}}
	}
	lambda0, ok0 := mappedArcRadialScale(mappedCaps, lfZero())
	lambdaMid, okMid := mappedArcRadialScale(mappedCaps, big.NewRat(1, 2))
	lambda2, ok2 := mappedArcRadialScale(mappedCaps, lfOne())
	lambda1 := lfSub(lfMul(big.NewRat(2, 1), lambdaMid),
		lfMul(big.NewRat(1, 2), lfAdd(lambda0, lambda2)))
	if !ok0 || !okMid || !ok2 ||
		lambda0.Sign() <= 0 || lambda1.Sign() < 0 || lambda2.Sign() <= 0 {
		return empty, false
	}
	x0, x2 := mappedCaps[0].X, mappedCaps[1].X
	xMid := lfMul(big.NewRat(1, 2), lfAdd(x0, x2))
	sectionX0, sectionX1 := lfF(meridian.start.U), lfF(meridian.end.U)
	sectionR0, sectionR1 := lfF(meridian.start.V), lfF(meridian.end.V)
	m := lfDiv(lfSub(sectionR1, sectionR0), lfSub(sectionX1, sectionX0))
	b := lfSub(sectionR0, lfMul(m, sectionX0))
	implicit := func(lambda, x *big.Rat) *big.Rat {
		radius := lfAdd(lfMul(m, x), b)
		return lfSub(lfMul(lambda, radius2), lfMul(radius, radius))
	}
	q0, qMid, q2 := implicit(lambda0, x0), implicit(lambdaMid, xMid), implicit(lambda2, x2)
	q1 := lfSub(lfMul(big.NewRat(2, 1), qMid),
		lfMul(big.NewRat(1, 2), lfAdd(q0, q2)))
	return mappedArcRadialBand{arc: arc, segment: segmentIndex, band: band,
		q: [3]*big.Rat{q0, q1, q2}, x: [2]*big.Rat{x0, x2}}, true
}

func mappedArcRadiusSquare(u, v, centerU, centerV *big.Rat) *big.Rat {
	du, dv := lfSub(u, centerU), lfSub(v, centerV)
	return lfAdd(lfMul(du, du), lfMul(dv, dv))
}

// A quadratic polynomial in band time vanishes identically when its exact
// values at 0, 1/2, and 1 vanish. These checks prove angular independence
// for every intermediate map, not merely for the two cap endpoints.
func mappedArcRadialScale(mappedCaps [2]mappedArcRadialCap, t *big.Rat) (*big.Rat, bool) {
	var u, v [2]*big.Rat
	for axis := range u {
		u[axis] = lfAdd(lfMul(lfSub(lfOne(), t), mappedCaps[0].U[axis]),
			lfMul(t, mappedCaps[1].U[axis]))
		v[axis] = lfAdd(lfMul(lfSub(lfOne(), t), mappedCaps[0].V[axis]),
			lfMul(t, mappedCaps[1].V[axis]))
	}
	u2 := lfAdd(lfMul(u[0], u[0]), lfMul(u[1], u[1]))
	v2 := lfAdd(lfMul(v[0], v[0]), lfMul(v[1], v[1]))
	dot := lfAdd(lfMul(u[0], v[0]), lfMul(u[1], v[1]))
	return u2, u2.Cmp(v2) == 0 && dot.Sign() == 0
}
