package decad

import (
	"context"
	"math/big"

	"github.com/lestrrat-3d/sketch"
)

const (
	mappedArcMiss    = 0
	mappedArcInside  = -1
	mappedArcOutside = 1
)

// mappedArcFiniteSide proves that the whole mapped Sketch arc misses a finite
// straight Revolve generator, or stays on one strict side wherever its axial
// range overlaps that generator. An undecidable box, including a possible
// crossing, refuses. No faceted station grants admission.
func mappedArcFiniteSide(ctx context.Context, trim *loftFacesTrimRecord,
	segmentIndex int, meridian revolveSectionSpan) (int, bool) {
	if trim == nil || len(trim.maps) < 2 {
		return 0, false
	}
	var side int
	for band := 0; band+1 < len(trim.maps); band++ {
		bandSide, ok := mappedArcFiniteSideCell(ctx, trim, segmentIndex, meridian,
			band, 0, 1, lfZero(), lfOne())
		if !ok || side != 0 && bandSide != 0 && side != bandSide {
			return 0, false
		}
		if bandSide != 0 {
			side = bandSide
		}
	}
	return side, true
}

// mappedArcFiniteSideCell classifies one source-parameter slice over one
// rational LoftFaces band-time slab. Its source parameter follows the live
// Sketch arc, not the possibly reversed profile walk. A possible crossing
// refuses, so a joint fit/arc proof must use this only for strict cells.
func mappedArcFiniteSideCell(ctx context.Context, trim *loftFacesTrimRecord,
	segmentIndex int, meridian revolveSectionSpan, band int,
	sLo, sHi float64, tLo, tHi *big.Rat) (int, bool) {
	if ctx == nil || trim == nil || trim.source == nil || trim.source.source == nil ||
		segmentIndex < 0 || segmentIndex >= len(trim.source.profile.Outer.Segments) ||
		segmentIndex >= len(trim.source.source.entities) ||
		band < 0 || band+1 >= len(trim.maps) ||
		sLo < 0 || sHi > 1 || sLo > sHi ||
		tLo == nil || tHi == nil || tLo.Sign() < 0 ||
		tLo.Cmp(tHi) > 0 || tHi.Cmp(lfOne()) > 0 {
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
	sourceBox, err := lfSourceBox(trim.source)
	if err != nil {
		return 0, false
	}
	x0, x1 := lfF(meridian.start.U), lfF(meridian.end.U)
	r0, r1 := lfF(meridian.start.V), lfF(meridian.end.V)
	if x0.Cmp(x1) == 0 || r0.Sign() < 0 || r1.Sign() < 0 {
		return 0, false
	}
	m := lfDiv(lfSub(r1, r0), lfSub(x1, x0))
	b := lfSub(r0, lfMul(m, x0))
	if x0.Cmp(x1) > 0 {
		x0, x1 = x1, x0
	}
	checks := 0
	var visit func(float64, float64, int) (int, bool)
	visit = func(lo, hi float64, depth int) (int, bool) {
		if ctx.Err() != nil || checks >= 4096 {
			return 0, false
		}
		checks++
		u, v, err := arc.CertifiedSpanBounds(lo, hi)
		if err != nil {
			return 0, false
		}
		box := [4]*big.Rat{lfF(u.Lo), lfF(u.Hi), lfF(v.Lo), lfF(v.Hi)}
		if lo == 0 {
			point := segment.Start
			box[0] = ratMin(box[0], lfF(point.U))
			box[1] = ratMax(box[1], lfF(point.U))
			box[2] = ratMin(box[2], lfF(point.V))
			box[3] = ratMax(box[3], lfF(point.V))
		}
		// A pinned end can miss the ideal circular carrier by the Sketch solve
		// residual. Include it whenever this slice reaches that endpoint.
		if hi == 1 {
			point := segment.End
			box[0] = ratMin(box[0], lfF(point.U))
			box[1] = ratMax(box[1], lfF(point.U))
			box[2] = ratMin(box[2], lfF(point.V))
			box[3] = ratMax(box[3], lfF(point.V))
		}
		if box[0].Cmp(sourceBox[0]) < 0 || box[1].Cmp(sourceBox[1]) > 0 ||
			box[2].Cmp(sourceBox[2]) < 0 || box[3].Cmp(sourceBox[3]) > 0 {
			return 0, false
		}
		corners := lfBoxCorners(box)
		var side int
		undecided := false
		first, second := mappedArcCapRanges(trim.maps[band], corners),
			mappedArcCapRanges(trim.maps[band+1], corners)
		bandSide, ok := mappedArcBandSide(first, second, x0, x1, m, b, tLo, tHi, 6)
		if !ok {
			undecided = true
		} else {
			side = bandSide
		}
		if !undecided {
			return side, true
		}
		if depth == 0 {
			return 0, false
		}
		mid := lo + (hi-lo)/2
		if mid <= lo || mid >= hi {
			return 0, false
		}
		left, ok := visit(lo, mid, depth-1)
		if !ok {
			return 0, false
		}
		right, ok := visit(mid, hi, depth-1)
		if !ok || left != 0 && right != 0 && left != right {
			return 0, false
		}
		if left != 0 {
			return left, true
		}
		return right, true
	}
	return visit(sLo, sHi, 10)
}

func ratMin(a, b *big.Rat) *big.Rat {
	if a.Cmp(b) < 0 {
		return a
	}
	return b
}

func mappedArcRecordMatches(segment arcSeg, arc *sketch.Arc) bool {
	return arc != nil && arc.Center != nil && arc.Start != nil && arc.End != nil &&
		segment.Center.U == arc.Center.X() && segment.Center.V == arc.Center.Y() &&
		segment.Start.U == arc.Start.X() && segment.Start.V == arc.Start.Y() &&
		segment.End.U == arc.End.X() && segment.End.V == arc.End.Y()
}

func ratMax(a, b *big.Rat) *big.Rat {
	if a.Cmp(b) > 0 {
		return a
	}
	return b
}

func mappedArcCapRanges(mapped loftFaceMap, corners [4][2]*big.Rat) [3]loftFaceInterval {
	var out [3]loftFaceInterval
	for axis := range out {
		var row loftFaceRatVec
		row[0], row[1], row[2] = lfZero(), lfZero(), lfZero()
		row[axis] = lfOne()
		out[axis] = lfScalarRange(mapped, row, corners)
	}
	return out
}

func mappedArcBlend(first, second loftFaceInterval, t *big.Rat) loftFaceInterval {
	a, z := lfSub(lfOne(), t), t
	return loftFaceInterval{
		Lo: lfAdd(lfMul(a, first.Lo), lfMul(z, second.Lo)),
		Hi: lfAdd(lfMul(a, first.Hi), lfMul(z, second.Hi)),
	}
}

func mappedArcBandSide(first, second [3]loftFaceInterval, x0, x1, m, b *big.Rat,
	tLo, tHi *big.Rat, depth int) (int, bool) {
	var box [3]loftFaceInterval
	for axis := range box {
		box[axis] = lfIntervalHull(mappedArcBlend(first[axis], second[axis], tLo),
			mappedArcBlend(first[axis], second[axis], tHi))
	}
	if box[0].Hi.Cmp(x0) < 0 || box[0].Lo.Cmp(x1) > 0 {
		return mappedArcMiss, true
	}
	x := loftFaceInterval{Lo: ratMax(box[0].Lo, x0), Hi: ratMin(box[0].Hi, x1)}
	radius := loftFaceInterval{Lo: lfAdd(lfMul(m, x.Lo), b), Hi: lfAdd(lfMul(m, x.Hi), b)}
	if radius.Lo.Cmp(radius.Hi) > 0 {
		radius.Lo, radius.Hi = radius.Hi, radius.Lo
	}
	low := lfSub(lfAdd(loftIntervalSquareMin(box[1]), loftIntervalSquareMin(box[2])),
		loftIntervalSquareMax(radius))
	high := lfSub(lfAdd(loftIntervalSquareMax(box[1]), loftIntervalSquareMax(box[2])),
		loftIntervalSquareMin(radius))
	if low.Sign() > 0 {
		return mappedArcOutside, true
	}
	if high.Sign() < 0 {
		return mappedArcInside, true
	}
	if depth == 0 {
		return 0, false
	}
	mid := lfMul(lfAdd(tLo, tHi), big.NewRat(1, 2))
	left, ok := mappedArcBandSide(first, second, x0, x1, m, b, tLo, mid, depth-1)
	if !ok {
		return 0, false
	}
	right, ok := mappedArcBandSide(first, second, x0, x1, m, b, mid, tHi, depth-1)
	if !ok || left != 0 && right != 0 && left != right {
		return 0, false
	}
	if left != 0 {
		return left, true
	}
	return right, true
}
