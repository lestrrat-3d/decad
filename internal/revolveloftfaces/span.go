package revolveloftfaces

import (
	"context"
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/loftface"
	"github.com/lestrrat-3d/decad/internal/polynomial"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/splinebezier"
	"github.com/lestrrat-3d/sketch"
)

type arcSeg = sectionrecord.ArcSeg

type curveSegment = sectionrecord.CurveSegment

type fitSplineSeg = sectionrecord.FitSplineSeg

type lineSeg = sectionrecord.LineSeg

type loftFaceInterval = loftface.Interval

type loftFaceMap = loftface.Map

type loftFacesTrimRecord = facetedrecord.LoftFacesTrimRecord

type pointSectionRecord = facetedrecord.PointSectionRecord

type revolveSectionSpan = facetedrecord.RevolveSectionSpan

// revolveLoftFacesMappedCell is a strict side or one transverse crossing of
// a complete source Bezier span over one LoftFaces band. A crossing's two
// cap brackets are exact rational enclosures, not rounded held vertices.
type RevolveLoftFacesMappedCell struct {
	Segment, Span, Band int
	Side                int
	JointKind           RevolveLoftFacesJointKind
	JointPoint          *sketch.Point
	JointArc            *sketch.Arc
	First, Last         loftface.RootCrossing
	XFirst, XLast       loftFaceInterval
	XDirection          int
	SLo, SHi            *big.Rat
	TLo, THi            *big.Rat
	TLoOpen, THiOpen    bool
}

// TimeInCell reports whether time belongs to the cell's exact time interval.
func TimeInCell(time *big.Rat, cell RevolveLoftFacesMappedCell) bool {
	if time == nil || cell.TLo == nil || cell.THi == nil {
		return false
	}
	lo, hi := time.Cmp(cell.TLo), time.Cmp(cell.THi)
	return (lo > 0 || lo == 0 && !cell.TLoOpen) &&
		(hi < 0 || hi == 0 && !cell.THiOpen)
}

// CapBoundaryBoxEqual compares the three exact intervals of two cap boundary boxes.
func CapBoundaryBoxEqual(a, b [3]loftface.Interval) bool {
	for axis := range a {
		if !loftface.IntervalEqual(a[axis], b[axis]) {
			return false
		}
	}
	return true
}

// revolveLoftFacesFiniteSpanCells classifies every fitted and straight
// source span against a finite, nonvertical Revolve section side. A finite
// contact is only possible where the crossing's axial interval overlaps the
// section side. This is a source certificate; it does not admit a held seam.
func RevolveLoftFacesFiniteSpanCells(ctx context.Context, trim *loftFacesTrimRecord,
	section revolveSectionSpan) ([]RevolveLoftFacesMappedCell, bool, error) {
	if ctx == nil || trim == nil || trim.Source == nil ||
		len(trim.Maps) < 2 || section.Start.U == section.End.U ||
		section.Start.V <= 0 || section.End.V <= 0 {
		return nil, false, nil
	}
	m, b := RevolveLoftFacesMeridian(section)
	joints, ok, err := RevolveLoftFacesEnumerateNonverticalJoints(ctx, trim, section)
	if err != nil || !ok {
		return nil, false, err
	}
	var cells []RevolveLoftFacesMappedCell
	for segment, source := range trim.Source.Profile.Outer.Segments {
		if err := ctx.Err(); err != nil {
			return nil, false, err
		}
		spans, ok := RevolveLoftFacesStraightOrFitSpans(trim, segment, source)
		if !ok {
			if _, isArc := source.(arcSeg); isArc {
				arcCells, certified, arcErr := RevolveLoftFacesArcSpanCells(ctx,
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
				var fitLines []*RevolveLoftFacesFitLineJoint
				for i := range joints.FitLines {
					joint := &joints.FitLines[i]
					if joint.Band != band {
						continue
					}
					if joint.Vertex.FitSegment == segment &&
						index == 0 && !joint.Vertex.fitAtEnd ||
						joint.Vertex.FitSegment == segment &&
							index == len(spans)-1 && joint.Vertex.fitAtEnd ||
						joint.Vertex.LineSegment == segment && index == 0 {
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
					if joint.vertex.fitSegment != segment || joint.radial.Band != band ||
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
					refined, certified, err := RevolveLoftFacesFitLinePartitionCells(ctx,
						trim, section, segment, index, band, span,
						knot, knotLeft, radial, fitLines)
					if err != nil || !certified {
						return nil, false, err
					}
					cells = append(cells, refined...)
					continue
				}
				if knot != nil || radial != nil {
					var refined []RevolveLoftFacesMappedCell
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
				if RevolveLoftFacesSpanAxialMiss(trim.Maps[band], trim.Maps[band+1],
					span, section, new(big.Rat), big.NewRat(1, 1), new(big.Rat), big.NewRat(1, 1)) {
					cells = append(cells, RevolveLoftFacesMappedCell{Segment: segment,
						Span: index, Band: band, Side: RevolveLoftFacesAxialMiss,
						SLo: new(big.Rat), SHi: big.NewRat(1, 1), TLo: new(big.Rat), THi: big.NewRat(1, 1)})
					continue
				}
				cell, ok := revolveLoftFacesMappedBezierCell(RevolveLoftFacesMeridianMemoOf(ctx), trim, m, b,
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

func RevolveLoftFacesMeridian(section revolveSectionSpan) (*big.Rat, *big.Rat) {
	x0, x1 := proofarith.FloatRat(section.Start.U), proofarith.FloatRat(section.End.U)
	r0, r1 := proofarith.FloatRat(section.Start.V), proofarith.FloatRat(section.End.V)
	m := proofarith.QuoRat(new(big.Rat), proofarith.SubRat(new(big.Rat), r1, r0), proofarith.SubRat(new(big.Rat), x1, x0))
	return m, proofarith.SubRat(new(big.Rat), r0, proofarith.MulRat(new(big.Rat), m, x0))
}

func RevolveLoftFacesStraightOrFitSpans(trim *loftFacesTrimRecord,
	segment int, source curveSegment) ([]freeform.BezierSpan, bool) {
	if segment >= len(trim.Source.Source.Entities) {
		return nil, false
	}
	switch typed := source.(type) {
	case lineSeg:
		if _, ok := trim.Source.Source.Entities[segment].(*sketch.Line); !ok ||
			!((typed.TStart == 0 && typed.TEnd == 1) ||
				(typed.TStart == 1 && typed.TEnd == 0)) {
			return nil, false
		}
		return []freeform.BezierSpan{{
			{U: proofarith.FloatRat(typed.Start.U), V: proofarith.FloatRat(typed.Start.V)},
			{U: proofarith.FloatRat(typed.End.U), V: proofarith.FloatRat(typed.End.V)},
		}}, true
	case fitSplineSeg:
		if _, ok := trim.Source.Source.Entities[segment].(*sketch.FitSpline); !ok ||
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
	segment, spanIndex, band int, span freeform.BezierSpan) (RevolveLoftFacesMappedCell, bool) {
	first, last := trim.Maps[band], trim.Maps[band+1]
	q := memo.QRows(first, last, span, m, b)
	kind, signed := loftface.ClassifyRows(q)
	cell := RevolveLoftFacesMappedCell{Segment: segment, Span: spanIndex,
		Band: band, Side: kind, SLo: new(big.Rat), SHi: big.NewRat(1, 1), TLo: new(big.Rat), THi: big.NewRat(1, 1)}
	if kind == 2 {
		return cell, false
	}
	if kind != loftface.SideCrossed {
		return cell, true
	}
	order := loftface.MeridianJRows(first, last, span, signed)
	switch {
	case polynomial.BernsteinRowsPositive(order):
		cell.XDirection = 1
	case polynomial.BernsteinRowsPositive(loftface.NegateRows(order)):
		cell.XDirection = -1
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
			cell.First, cell.XFirst = root, x
		} else {
			cell.Last, cell.XLast = root, x
		}
	}
	if cell.XDirection > 0 && cell.XFirst.Hi.Cmp(cell.XLast.Lo) >= 0 ||
		cell.XDirection < 0 && cell.XLast.Hi.Cmp(cell.XFirst.Lo) >= 0 {
		return cell, false
	}
	return cell, true
}

// The mapped endpoint has constant rational source coordinates, so its X
// coordinate interpolates linearly between the two authenticated cap maps.
// A finite section admits a root only when the complete exact X bracket lies
// strictly within that section. A boundary overlap is undecidable.
func RevolveLoftFacesEndpointAxialClass(root loftface.EndpointRoot,
	x0, x1 *big.Rat, section revolveSectionSpan) (loftface.EndpointRoot, bool) {
	if x0 == nil || x1 == nil || root.Time.Lo == nil || root.Time.Hi == nil {
		return root, false
	}
	xAt := func(t *big.Rat) *big.Rat {
		return proofarith.AddRat(new(big.Rat), x0, proofarith.MulRat(new(big.Rat), t, proofarith.SubRat(new(big.Rat), x1, x0)))
	}
	xLo, xHi := xAt(root.Time.Lo), xAt(root.Time.Hi)
	if xLo.Cmp(xHi) > 0 {
		xLo, xHi = xHi, xLo
	}
	root.X = loftFaceInterval{Lo: xLo, Hi: xHi}
	start, end := proofarith.FloatRat(section.Start.U), proofarith.FloatRat(section.End.U)
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

func RevolveLoftFacesEndpointFiniteRoots(trim *loftFacesTrimRecord,
	section revolveSectionSpan, span freeform.BezierSpan, band int, atEnd bool) (
	[]loftface.EndpointRoot, bool,
) {
	return RevolveLoftFacesEndpointFiniteRootsWith(nil, trim, section, span, band, atEnd)
}

func RevolveLoftFacesEndpointFiniteRootsWith(memo *loftface.MeridianMemo, trim *loftFacesTrimRecord,
	section revolveSectionSpan, span freeform.BezierSpan, band int, atEnd bool) (
	[]loftface.EndpointRoot, bool,
) {
	if trim == nil || band < 0 || band+1 >= len(trim.Maps) || len(span) == 0 ||
		section.Start.U == section.End.U {
		return nil, false
	}
	m, b := RevolveLoftFacesMeridian(section)
	q := memo.QRows(trim.Maps[band], trim.Maps[band+1], span, m, b)
	roots, ok := loftface.EndpointTimeRoots(q, atEnd)
	if !ok {
		return nil, false
	}
	s := new(big.Rat)
	if atEnd {
		s = big.NewRat(1, 1)
	}
	point := loftface.BezierPoint(span, s)
	x0 := trim.Maps[band].At(point[0], point[1])[0]
	x1 := trim.Maps[band+1].At(point[0], point[1])[0]
	for i, root := range roots {
		roots[i], ok = RevolveLoftFacesEndpointAxialClass(root, x0, x1, section)
		if !ok {
			return nil, false
		}
	}
	return roots, true
}

func RevolveLoftFacesSpanAxialMiss(first, last loftFaceMap, span freeform.BezierSpan,
	section revolveSectionSpan, sLo, sHi, tLo, tHi *big.Rat) bool {
	x, ok := loftface.SpanXRange(first, last, span, sLo, sHi, tLo, tHi)
	if !ok {
		return false
	}
	start, end := proofarith.FloatRat(section.Start.U), proofarith.FloatRat(section.End.U)
	if start.Cmp(end) > 0 {
		start, end = end, start
	}
	return x.Hi.Cmp(start) < 0 || x.Lo.Cmp(end) > 0
}
