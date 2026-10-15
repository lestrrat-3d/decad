package decad

import (
	"context"
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/loftface"
	"github.com/lestrrat-3d/decad/internal/polynomial"
	"github.com/lestrrat-3d/decad/internal/splinebezier"
	"github.com/lestrrat-3d/sketch"
)

// revolveLoftFacesMappedCell is a strict side or one transverse crossing of
// a complete source Bezier span over one LoftFaces band. A crossing's two
// cap brackets are exact rational enclosures, not rounded held vertices.
type revolveLoftFacesMappedCell struct {
	segment, span, band int
	side                int
	jointKind           revolveLoftFacesJointKind
	jointPoint          *sketch.Point
	jointArc            *sketch.Arc
	first, last         loftface.RootCrossing
	xFirst, xLast       loftFaceInterval
	xDirection          int
	sLo, sHi            *big.Rat
	tLo, tHi            *big.Rat
	tLoOpen, tHiOpen    bool
}

// revolveLoftFacesFiniteSpanCells classifies every fitted and straight
// source span against a finite, nonvertical Revolve section side. A finite
// contact is only possible where the crossing's axial interval overlaps the
// section side. This is a source certificate; it does not admit a held seam.
func revolveLoftFacesFiniteSpanCells(ctx context.Context, trim *loftFacesTrimRecord,
	section revolveSectionSpan) ([]revolveLoftFacesMappedCell, bool, error) {
	if ctx == nil || trim == nil || trim.Source == nil ||
		len(trim.Maps) < 2 || section.start.U == section.end.U ||
		section.start.V <= 0 || section.end.V <= 0 {
		return nil, false, nil
	}
	m, b := revolveLoftFacesMeridian(section)
	joints, ok, err := revolveLoftFacesEnumerateNonverticalJoints(ctx, trim, section)
	if err != nil || !ok {
		return nil, false, err
	}
	var cells []revolveLoftFacesMappedCell
	for segment, source := range trim.Source.profile.Outer.Segments {
		if err := ctx.Err(); err != nil {
			return nil, false, err
		}
		spans, ok := revolveLoftFacesStraightOrFitSpans(trim, segment, source)
		if !ok {
			if _, isArc := source.(arcSeg); isArc {
				arcCells, certified, arcErr := revolveLoftFacesArcSpanCells(ctx,
					trim, section, segment)
				if arcErr != nil || !certified {
					return nil, false, arcErr
				}
				cells = append(cells, arcCells...)
				continue
			}
			return nil, false, nil
		}
		for index, span := range spans {
			for band := 0; band+1 < len(trim.Maps); band++ {
				if err := ctx.Err(); err != nil {
					return nil, false, err
				}
				var knot *revolveLoftFacesFitKnotJoint
				knotLeft := false
				var radial *revolveLoftFacesRadialJoint
				var fitLines []*revolveLoftFacesFitLineJoint
				for i := range joints.fitLines {
					joint := &joints.fitLines[i]
					if joint.band != band {
						continue
					}
					if joint.vertex.fitSegment == segment &&
						index == 0 && !joint.vertex.fitAtEnd ||
						joint.vertex.fitSegment == segment &&
							index == len(spans)-1 && joint.vertex.fitAtEnd ||
						joint.vertex.lineSegment == segment && index == 0 {
						fitLines = append(fitLines, joint)
					}
				}
				for i := range joints.fitKnots {
					joint := &joints.fitKnots[i]
					if joint.vertex.segment != segment || joint.band != band ||
						index != joint.vertex.leftIndex && index != joint.vertex.leftIndex+1 {
						continue
					}
					if knot != nil {
						return nil, false, fmt.Errorf("mapped source segment %d span %d band %d has overlapping joints",
							segment, index, band)
					}
					knot, knotLeft = joint, index == joint.vertex.leftIndex
				}
				for i := range joints.radial {
					joint := &joints.radial[i]
					if joint.vertex.fitSegment != segment || joint.radial.band != band ||
						joint.vertex.fitAtEnd && index != len(spans)-1 ||
						!joint.vertex.fitAtEnd && index != 0 {
						continue
					}
					if radial != nil {
						return nil, false, fmt.Errorf("mapped source segment %d span %d band %d has overlapping joints",
							segment, index, band)
					}
					radial = joint
				}
				if len(fitLines) != 0 {
					refined, certified, err := revolveLoftFacesFitLinePartitionCells(ctx,
						trim, section, segment, index, band, span,
						knot, knotLeft, radial, fitLines)
					if err != nil || !certified {
						return nil, false, err
					}
					cells = append(cells, refined...)
					continue
				}
				if knot != nil || radial != nil {
					var refined []revolveLoftFacesMappedCell
					var certified bool
					var err error
					if knot != nil && radial != nil {
						refined, certified, err = revolveLoftFacesMergeEndpointCells(ctx,
							trim, section, segment, index, band, span, knot, radial, knotLeft)
					} else if knot != nil {
						refined, certified, err = revolveLoftFacesKnotPartitionCells(ctx,
							trim, section, segment, index, band, span, *knot, knotLeft)
					} else {
						refined, certified, err = revolveLoftFacesRadialPartitionCells(ctx,
							trim, section, segment, index, band, span, *radial)
					}
					if err != nil || !certified {
						return nil, false, err
					}
					cells = append(cells, refined...)
					continue
				}
				if revolveLoftFacesSpanAxialMiss(trim.Maps[band], trim.Maps[band+1],
					span, section, lfZero(), lfOne(), lfZero(), lfOne()) {
					cells = append(cells, revolveLoftFacesMappedCell{segment: segment,
						span: index, band: band, side: revolveLoftFacesAxialMiss,
						sLo: lfZero(), sHi: lfOne(), tLo: lfZero(), tHi: lfOne()})
					continue
				}
				cell, ok := revolveLoftFacesMappedBezierCell(revolveLoftFacesMeridianMemoOf(ctx), trim, m, b,
					segment, index, band, span)
				if ok {
					cells = append(cells, cell)
					continue
				}
				refined, ok, err := revolveLoftFacesAdaptiveBezierCells(ctx, trim, m, b,
					segment, index, band, span, section)
				if err != nil {
					return nil, false, fmt.Errorf("mapped source segment %d span %d band %d: %w",
						segment, index, band, err)
				}
				if !ok {
					return nil, false, fmt.Errorf("mapped source segment %d span %d band %d is undecidable",
						segment, index, band)
				}
				cells = append(cells, refined...)
			}
		}
	}
	return cells, true, nil
}

func revolveLoftFacesMeridian(section revolveSectionSpan) (*big.Rat, *big.Rat) {
	x0, x1 := lfF(section.start.U), lfF(section.end.U)
	r0, r1 := lfF(section.start.V), lfF(section.end.V)
	m := lfDiv(lfSub(r1, r0), lfSub(x1, x0))
	return m, lfSub(r0, lfMul(m, x0))
}

func revolveLoftFacesStraightOrFitSpans(trim *loftFacesTrimRecord,
	segment int, source curveSegment) ([]freeform.BezierSpan, bool) {
	if segment >= len(trim.Source.source.entities) {
		return nil, false
	}
	switch typed := source.(type) {
	case lineSeg:
		if _, ok := trim.Source.source.entities[segment].(*sketch.Line); !ok ||
			!((typed.TStart == 0 && typed.TEnd == 1) ||
				(typed.TStart == 1 && typed.TEnd == 0)) {
			return nil, false
		}
		return []freeform.BezierSpan{{
			{U: lfF(typed.Start.U), V: lfF(typed.Start.V)},
			{U: lfF(typed.End.U), V: lfF(typed.End.V)},
		}}, true
	case fitSplineSeg:
		if _, ok := trim.Source.source.entities[segment].(*sketch.FitSpline); !ok ||
			!((typed.TStart == 0 && typed.TEnd == 1) ||
				(typed.TStart == 1 && typed.TEnd == 0)) {
			return nil, false
		}
		work := freeform.NewFreeformWork()
		work.RaiseReconstructionLimit(freeform.LoftReconstructionWorkLimit)
		spans, err := splinebezier.FitSplineBezierSpans(typed, work)
		return spans, err == nil && len(spans) > 0 && len(spans) <= 128
	default:
		return nil, false
	}
}

func revolveLoftFacesMappedBezierCell(memo *loftface.MeridianMemo, trim *loftFacesTrimRecord, m, b *big.Rat,
	segment, spanIndex, band int, span freeform.BezierSpan) (revolveLoftFacesMappedCell, bool) {
	first, last := trim.Maps[band], trim.Maps[band+1]
	q := memo.QRows(first, last, span, m, b)
	kind, signed := loftface.ClassifyRows(q)
	cell := revolveLoftFacesMappedCell{segment: segment, span: spanIndex,
		band: band, side: kind, sLo: lfZero(), sHi: lfOne(), tLo: lfZero(), tHi: lfOne()}
	if kind == 2 {
		return cell, false
	}
	if kind != loftface.SideCrossed {
		return cell, true
	}
	order := loftface.MeridianJRows(first, last, span, signed)
	switch {
	case polynomial.BernsteinRowsPositive(order):
		cell.xDirection = 1
	case polynomial.BernsteinRowsPositive(loftface.NegateRows(order)):
		cell.xDirection = -1
	default:
		return cell, false
	}
	for capIndex, mapped := range [...]loftFaceMap{first, last} {
		capQ := memo.QRows(mapped, mapped, span, m, b)
		capKind, capSigned := loftface.ClassifyRows(capQ)
		if capKind != loftface.SideCrossed {
			return cell, false
		}
		root := loftface.IsolateRoot(capSigned[0])
		root.Span = spanIndex
		direction := loftface.CapXDirection(mapped, span)
		if direction == 2 {
			return cell, false
		}
		x := loftface.DirectedCapXRange(mapped, span, root, direction)
		if capIndex == 0 {
			cell.first, cell.xFirst = root, x
		} else {
			cell.last, cell.xLast = root, x
		}
	}
	if cell.xDirection > 0 && cell.xFirst.Hi.Cmp(cell.xLast.Lo) >= 0 ||
		cell.xDirection < 0 && cell.xLast.Hi.Cmp(cell.xFirst.Lo) >= 0 {
		return cell, false
	}
	return cell, true
}

// The mapped endpoint has constant rational source coordinates, so its X
// coordinate interpolates linearly between the two authenticated cap maps.
// A finite section admits a root only when the complete exact X bracket lies
// strictly within that section. A boundary overlap is undecidable.
func revolveLoftFacesEndpointAxialClass(root loftface.EndpointRoot,
	x0, x1 *big.Rat, section revolveSectionSpan) (loftface.EndpointRoot, bool) {
	if x0 == nil || x1 == nil || root.Time.Lo == nil || root.Time.Hi == nil {
		return root, false
	}
	xAt := func(t *big.Rat) *big.Rat { return lfAdd(x0, lfMul(t, lfSub(x1, x0))) }
	xLo, xHi := xAt(root.Time.Lo), xAt(root.Time.Hi)
	if xLo.Cmp(xHi) > 0 {
		xLo, xHi = xHi, xLo
	}
	root.X = loftFaceInterval{Lo: xLo, Hi: xHi}
	start, end := lfF(section.start.U), lfF(section.end.U)
	if start.Cmp(end) > 0 {
		start, end = end, start
	}
	if xHi.Cmp(start) < 0 || xLo.Cmp(end) > 0 {
		return root, true
	}
	if xLo.Cmp(start) > 0 && xHi.Cmp(end) < 0 {
		root.Finite = true
		return root, true
	}
	return root, false
}

func revolveLoftFacesEndpointFiniteRoots(trim *loftFacesTrimRecord,
	section revolveSectionSpan, span freeform.BezierSpan, band int, atEnd bool) (
	[]loftface.EndpointRoot, bool,
) {
	return revolveLoftFacesEndpointFiniteRootsWith(nil, trim, section, span, band, atEnd)
}

func revolveLoftFacesEndpointFiniteRootsWith(memo *loftface.MeridianMemo, trim *loftFacesTrimRecord,
	section revolveSectionSpan, span freeform.BezierSpan, band int, atEnd bool) (
	[]loftface.EndpointRoot, bool,
) {
	if trim == nil || band < 0 || band+1 >= len(trim.Maps) || len(span) == 0 ||
		section.start.U == section.end.U {
		return nil, false
	}
	m, b := revolveLoftFacesMeridian(section)
	q := memo.QRows(trim.Maps[band], trim.Maps[band+1], span, m, b)
	roots, ok := loftface.EndpointTimeRoots(q, atEnd)
	if !ok {
		return nil, false
	}
	s := lfZero()
	if atEnd {
		s = lfOne()
	}
	point := loftface.BezierPoint(span, s)
	x0 := trim.Maps[band].At(point[0], point[1])[0]
	x1 := trim.Maps[band+1].At(point[0], point[1])[0]
	for i, root := range roots {
		roots[i], ok = revolveLoftFacesEndpointAxialClass(root, x0, x1, section)
		if !ok {
			return nil, false
		}
	}
	return roots, true
}

func revolveLoftFacesSpanAxialMiss(first, last loftFaceMap, span freeform.BezierSpan,
	section revolveSectionSpan, sLo, sHi, tLo, tHi *big.Rat) bool {
	x, ok := loftface.SpanXRange(first, last, span, sLo, sHi, tLo, tHi)
	if !ok {
		return false
	}
	start, end := lfF(section.start.U), lfF(section.end.U)
	if start.Cmp(end) > 0 {
		start, end = end, start
	}
	return x.Hi.Cmp(start) < 0 || x.Lo.Cmp(end) > 0
}
