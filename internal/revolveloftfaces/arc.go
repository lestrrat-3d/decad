package revolveloftfaces

import (
	"context"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/loftface"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/sketch"
)

// VerticalTraceMapPoint maps a source point into the exact vertical side plane.
func VerticalTraceMapPoint(affine *loftface.VerticalAffine,
	point sectionrecord.Point2) [2]*big.Rat {
	u, v := lfF(point.U), lfF(point.V)
	return [2]*big.Rat{
		lfAdd(affine.Offset[0], lfAdd(lfMul(affine.Column[0][0], u), lfMul(affine.Column[1][0], v))),
		lfAdd(affine.Offset[1], lfAdd(lfMul(affine.Column[0][1], u), lfMul(affine.Column[1][1], v))),
	}
}

// VerticalTraceArcRadius accepts a centered circular source only under an exact
// positive similarity. This is the class whose ray hit and arc length can be
// bounded without treating a sampled arc point as exact.
func VerticalTraceArcRadius(affine *loftface.VerticalAffine,
	arc arcSeg) (*big.Rat, bool) {
	if affine == nil || affine.Det == nil || affine.Det.Sign() <= 0 {
		return nil, false
	}
	center := VerticalTraceMapPoint(affine, arc.Center)
	if center[0].Sign() != 0 || center[1].Sign() != 0 {
		return nil, false
	}
	u, v := affine.Column[0], affine.Column[1]
	u2 := lfAdd(lfMul(u[0], u[0]), lfMul(u[1], u[1]))
	v2 := lfAdd(lfMul(v[0], v[0]), lfMul(v[1], v[1]))
	if u2.Sign() <= 0 || u2.Cmp(v2) != 0 ||
		lfAdd(lfMul(u[0], v[0]), lfMul(u[1], v[1])).Sign() != 0 {
		return nil, false
	}
	start := VerticalTraceMapPoint(affine, arc.Start)
	end := VerticalTraceMapPoint(affine, arc.End)
	r2 := lfAdd(lfMul(start[0], start[0]), lfMul(start[1], start[1]))
	end2 := lfAdd(lfMul(end[0], end[0]), lfMul(end[1], end[1]))
	return r2, r2.Sign() > 0 && r2.Cmp(end2) == 0
}

// mappedArcFiniteSide proves that the whole mapped Sketch arc misses a finite
// straight Revolve generator, or stays on one strict side wherever its axial
// range overlaps that generator. An undecidable box, including a possible
// crossing, refuses. No faceted station grants admission.
func MappedArcFiniteSide(ctx context.Context, trim *loftFacesTrimRecord,
	segmentIndex int, meridian revolveSectionSpan) (int, bool) {
	if trim == nil || len(trim.Maps) < 2 {
		return 0, false
	}
	var side int
	for band := 0; band+1 < len(trim.Maps); band++ {
		bandSide, ok := mappedArcFiniteSideCell(ctx, trim, segmentIndex, meridian,
			band, 0, 1, new(big.Rat), big.NewRat(1, 1))
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
	if ctx == nil || trim == nil || trim.Source == nil || trim.Source.Source == nil ||
		segmentIndex < 0 || segmentIndex >= len(trim.Source.Profile.Outer.Segments) ||
		segmentIndex >= len(trim.Source.Source.Entities) ||
		band < 0 || band+1 >= len(trim.Maps) ||
		sLo < 0 || sHi > 1 || sLo > sHi ||
		tLo == nil || tHi == nil || tLo.Sign() < 0 ||
		tLo.Cmp(tHi) > 0 || tHi.Cmp(big.NewRat(1, 1)) > 0 {
		return 0, false
	}
	segment, ok := trim.Source.Profile.Outer.Segments[segmentIndex].(arcSeg)
	if !ok || !((segment.TStart == 0 && segment.TEnd == 1) ||
		(segment.TStart == 1 && segment.TEnd == 0)) {
		return 0, false
	}
	arc, ok := trim.Source.Source.Entities[segmentIndex].(*sketch.Arc)
	if !ok || !MappedArcRecordMatches(segment, arc) {
		return 0, false
	}
	sourceBox, err := LfSourceBox(trim.Source)
	if err != nil {
		return 0, false
	}
	x0, x1 := proofarith.FloatRat(meridian.Start.U), proofarith.FloatRat(meridian.End.U)
	r0, r1 := proofarith.FloatRat(meridian.Start.V), proofarith.FloatRat(meridian.End.V)
	if x0.Cmp(x1) == 0 || r0.Sign() < 0 || r1.Sign() < 0 {
		return 0, false
	}
	m := proofarith.QuoRat(new(big.Rat), proofarith.SubRat(new(big.Rat), r1, r0), proofarith.SubRat(new(big.Rat), x1, x0))
	b := proofarith.SubRat(new(big.Rat), r0, proofarith.MulRat(new(big.Rat), m, x0))
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
		box := [4]*big.Rat{proofarith.FloatRat(u.Lo), proofarith.FloatRat(u.Hi), proofarith.FloatRat(v.Lo), proofarith.FloatRat(v.Hi)}
		if lo == 0 {
			point := segment.Start
			box[0] = loftface.RatMin(box[0], proofarith.FloatRat(point.U))
			box[1] = loftface.RatMax(box[1], proofarith.FloatRat(point.U))
			box[2] = loftface.RatMin(box[2], proofarith.FloatRat(point.V))
			box[3] = loftface.RatMax(box[3], proofarith.FloatRat(point.V))
		}
		// A pinned end can miss the ideal circular carrier by the Sketch solve
		// residual. Include it whenever this slice reaches that endpoint.
		if hi == 1 {
			point := segment.End
			box[0] = loftface.RatMin(box[0], proofarith.FloatRat(point.U))
			box[1] = loftface.RatMax(box[1], proofarith.FloatRat(point.U))
			box[2] = loftface.RatMin(box[2], proofarith.FloatRat(point.V))
			box[3] = loftface.RatMax(box[3], proofarith.FloatRat(point.V))
		}
		if box[0].Cmp(sourceBox[0]) < 0 || box[1].Cmp(sourceBox[1]) > 0 ||
			box[2].Cmp(sourceBox[2]) < 0 || box[3].Cmp(sourceBox[3]) > 0 {
			return 0, false
		}
		corners := loftface.BoxCorners(box)
		var side int
		undecided := false
		first, second := loftface.ArcCapRanges(trim.Maps[band], corners),
			loftface.ArcCapRanges(trim.Maps[band+1], corners)
		bandSide, ok := loftface.ArcBandSide(first, second, x0, x1, m, b, tLo, tHi, 6)
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

func MappedArcRecordMatches(segment arcSeg, arc *sketch.Arc) bool {
	return arc != nil && arc.Center != nil && arc.Start != nil && arc.End != nil &&
		segment.Center.U == arc.Center.X() && segment.Center.V == arc.Center.Y() &&
		segment.Start.U == arc.Start.X() && segment.Start.V == arc.Start.Y() &&
		segment.End.U == arc.End.X() && segment.End.V == arc.End.Y()
}
