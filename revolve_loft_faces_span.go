package decad

import (
	"context"
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/freeform"
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
	first, last         spiralRootCrossing
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
	if ctx == nil || trim == nil || trim.source == nil ||
		len(trim.maps) < 2 || section.start.U == section.end.U ||
		section.start.V <= 0 || section.end.V <= 0 {
		return nil, false, nil
	}
	m, b := revolveLoftFacesMeridian(section)
	joints, ok, err := revolveLoftFacesEnumerateNonverticalJoints(ctx, trim, section)
	if err != nil || !ok {
		return nil, false, err
	}
	var cells []revolveLoftFacesMappedCell
	for segment, source := range trim.source.profile.Outer.Segments {
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
			for band := 0; band+1 < len(trim.maps); band++ {
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
				if revolveLoftFacesSpanAxialMiss(trim.maps[band], trim.maps[band+1],
					span, section, lfZero(), lfOne(), lfZero(), lfOne()) {
					cells = append(cells, revolveLoftFacesMappedCell{segment: segment,
						span: index, band: band, side: revolveLoftFacesAxialMiss,
						sLo: lfZero(), sHi: lfOne(), tLo: lfZero(), tHi: lfOne()})
					continue
				}
				cell, ok := revolveLoftFacesMappedBezierCell(trim, m, b,
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
	if segment >= len(trim.source.source.entities) {
		return nil, false
	}
	switch typed := source.(type) {
	case lineSeg:
		if _, ok := trim.source.source.entities[segment].(*sketch.Line); !ok ||
			!((typed.TStart == 0 && typed.TEnd == 1) ||
				(typed.TStart == 1 && typed.TEnd == 0)) {
			return nil, false
		}
		return []freeform.BezierSpan{{
			{U: lfF(typed.Start.U), V: lfF(typed.Start.V)},
			{U: lfF(typed.End.U), V: lfF(typed.End.V)},
		}}, true
	case fitSplineSeg:
		if _, ok := trim.source.source.entities[segment].(*sketch.FitSpline); !ok ||
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

func revolveLoftFacesMappedBezierCell(trim *loftFacesTrimRecord, m, b *big.Rat,
	segment, spanIndex, band int, span freeform.BezierSpan) (revolveLoftFacesMappedCell, bool) {
	first, last := trim.maps[band], trim.maps[band+1]
	q := loftFacesMeridianQRows(first, last, span, m, b)
	kind, signed := revolveLoftFacesClassifyRows(q)
	cell := revolveLoftFacesMappedCell{segment: segment, span: spanIndex,
		band: band, side: kind, sLo: lfZero(), sHi: lfOne(), tLo: lfZero(), tHi: lfOne()}
	if kind == 2 {
		return cell, false
	}
	if kind != revolveLoftFacesCrossed {
		return cell, true
	}
	order := loftFacesMeridianJRows(first, last, span, signed)
	switch {
	case polynomial.BernsteinRowsPositive(order):
		cell.xDirection = 1
	case polynomial.BernsteinRowsPositive(revolveLoftFacesNegateRows(order)):
		cell.xDirection = -1
	default:
		return cell, false
	}
	for capIndex, mapped := range [...]loftFaceMap{first, last} {
		capQ := loftFacesMeridianQRows(mapped, mapped, span, m, b)
		capKind, capSigned := revolveLoftFacesClassifyRows(capQ)
		if capKind != revolveLoftFacesCrossed {
			return cell, false
		}
		root := spiralIsolateRoot(capSigned[0])
		root.span = spanIndex
		direction := revolveLoftFacesCapXDirection(mapped, span)
		if direction == 2 {
			return cell, false
		}
		x := revolveLoftFacesCapXRange(mapped, span, root, direction)
		if capIndex == 0 {
			cell.first, cell.xFirst = root, x
		} else {
			cell.last, cell.xLast = root, x
		}
	}
	if cell.xDirection > 0 && cell.xFirst.hi.Cmp(cell.xLast.lo) >= 0 ||
		cell.xDirection < 0 && cell.xLast.hi.Cmp(cell.xFirst.lo) >= 0 {
		return cell, false
	}
	return cell, true
}

// A crossing can enter another whole Sketch entity through their shared
// endpoint twice in one LoftFaces band. These exact rational brackets cover
// every zero of the quadratic endpoint polynomial in band time. Each simple
// root changes side. The caller must bind the same live source endpoint on
// both adjacent entities before interpreting either change as a joint seam.
type revolveLoftFacesEndpointRoot struct {
	time      loftFaceInterval
	x         loftFaceInterval
	direction int
	finite    bool
}

// The mapped endpoint has constant rational source coordinates, so its X
// coordinate interpolates linearly between the two authenticated cap maps.
// A finite section admits a root only when the complete exact X bracket lies
// strictly within that section. A boundary overlap is undecidable.
func revolveLoftFacesEndpointAxialClass(root revolveLoftFacesEndpointRoot,
	x0, x1 *big.Rat, section revolveSectionSpan) (revolveLoftFacesEndpointRoot, bool) {
	if x0 == nil || x1 == nil || root.time.lo == nil || root.time.hi == nil {
		return root, false
	}
	xAt := func(t *big.Rat) *big.Rat { return lfAdd(x0, lfMul(t, lfSub(x1, x0))) }
	xLo, xHi := xAt(root.time.lo), xAt(root.time.hi)
	if xLo.Cmp(xHi) > 0 {
		xLo, xHi = xHi, xLo
	}
	root.x = loftFaceInterval{lo: xLo, hi: xHi}
	start, end := lfF(section.start.U), lfF(section.end.U)
	if start.Cmp(end) > 0 {
		start, end = end, start
	}
	if xHi.Cmp(start) < 0 || xLo.Cmp(end) > 0 {
		return root, true
	}
	if xLo.Cmp(start) > 0 && xHi.Cmp(end) < 0 {
		root.finite = true
		return root, true
	}
	return root, false
}

func revolveLoftFacesEndpointFiniteRoots(trim *loftFacesTrimRecord,
	section revolveSectionSpan, span freeform.BezierSpan, band int, atEnd bool) (
	[]revolveLoftFacesEndpointRoot, bool,
) {
	if trim == nil || band < 0 || band+1 >= len(trim.maps) || len(span) == 0 ||
		section.start.U == section.end.U {
		return nil, false
	}
	m, b := revolveLoftFacesMeridian(section)
	q := spiralRootPolynomial(trim.maps[band], trim.maps[band+1], span, m, b)
	roots, ok := revolveLoftFacesEndpointTimeRoots(q, atEnd)
	if !ok {
		return nil, false
	}
	s := lfZero()
	if atEnd {
		s = lfOne()
	}
	point := spiralBezierPoint(span, s)
	x0 := trim.maps[band].at(point[0], point[1])[0]
	x1 := trim.maps[band+1].at(point[0], point[1])[0]
	for i, root := range roots {
		roots[i], ok = revolveLoftFacesEndpointAxialClass(root, x0, x1, section)
		if !ok {
			return nil, false
		}
	}
	return roots, true
}

// Bezier hulls enclose both cap numerators and their positive projective
// denominator over a source subspan. X is affine in band time at each source
// point, so the hull of its two time-end ranges encloses the full time slab.
func revolveLoftFacesSpanXRange(first, last loftFaceMap, span freeform.BezierSpan,
	sLo, sHi, tLo, tHi *big.Rat) (loftFaceInterval, bool) {
	if len(span) == 0 || sLo == nil || sHi == nil || tLo == nil || tHi == nil ||
		sLo.Sign() < 0 || sLo.Cmp(sHi) > 0 || sHi.Cmp(lfOne()) > 0 ||
		tLo.Sign() < 0 || tLo.Cmp(tHi) > 0 || tHi.Cmp(lfOne()) > 0 {
		return loftFaceInterval{}, false
	}
	g0, h0 := spiralCapBern(first, span)
	g1, h1 := spiralCapBern(last, span)
	denominator := polynomial.BernsteinRestrict(polynomial.BernsteinMul(g0, g1), sLo, sHi)
	denRange := revolveLoftFacesCoeffRange(denominator)
	if denRange.lo.Sign() <= 0 {
		return loftFaceInterval{}, false
	}
	firstX, lastX := polynomial.BernsteinMul(h0[0], g1), polynomial.BernsteinMul(h1[0], g0)
	at := func(t *big.Rat) loftFaceInterval {
		numerator := polynomial.BernsteinAdd(polynomial.BernsteinScale(firstX, lfSub(lfOne(), t)),
			polynomial.BernsteinScale(lastX, t))

		numRange := revolveLoftFacesCoeffRange(polynomial.BernsteinRestrict(numerator, sLo, sHi))
		var out loftFaceInterval
		for _, valueNum := range [...]*big.Rat{numRange.lo, numRange.hi} {
			for _, valueDen := range [...]*big.Rat{denRange.lo, denRange.hi} {
				value := lfDiv(valueNum, valueDen)
				if out.lo == nil || value.Cmp(out.lo) < 0 {
					out.lo = value
				}
				if out.hi == nil || value.Cmp(out.hi) > 0 {
					out.hi = value
				}
			}
		}
		return out
	}
	return lfIntervalHull(at(tLo), at(tHi)), true
}

func revolveLoftFacesCoeffRange(coeff []*big.Rat) loftFaceInterval {
	var out loftFaceInterval
	for _, value := range coeff {
		if out.lo == nil || value.Cmp(out.lo) < 0 {
			out.lo = value
		}
		if out.hi == nil || value.Cmp(out.hi) > 0 {
			out.hi = value
		}
	}
	return out
}

func revolveLoftFacesSpanAxialMiss(first, last loftFaceMap, span freeform.BezierSpan,
	section revolveSectionSpan, sLo, sHi, tLo, tHi *big.Rat) bool {
	x, ok := revolveLoftFacesSpanXRange(first, last, span, sLo, sHi, tLo, tHi)
	if !ok {
		return false
	}
	start, end := lfF(section.start.U), lfF(section.end.U)
	if start.Cmp(end) > 0 {
		start, end = end, start
	}
	return x.hi.Cmp(start) < 0 || x.lo.Cmp(end) > 0
}

func revolveLoftFacesEndpointTimeRoots(q [3][]*big.Rat,
	atEnd bool) ([]revolveLoftFacesEndpointRoot, bool) {
	if len(q[0]) == 0 || len(q[0]) != len(q[1]) || len(q[0]) != len(q[2]) {
		return nil, false
	}
	index := 0
	if atEnd {
		index = len(q[0]) - 1
	}
	coeff := []*big.Rat{q[0][index], q[1][index], q[2][index]}
	first, last := coeff[0].Sign(), coeff[2].Sign()
	if first == 0 || last == 0 {
		return nil, false
	}
	if first != last {
		root, ok := revolveLoftFacesIsolateTimeRoot(coeff, lfZero(), lfOne())
		if !ok {
			return nil, false
		}
		return []revolveLoftFacesEndpointRoot{root}, true
	}
	if coeff[1].Sign() == first {
		return nil, true
	}
	// A quadratic with equal endpoint signs has either zero or two simple
	// interior roots. A bounded dyadic search finds an opposite-sign point;
	// two disjoint sign-changing intervals then prove both roots and exclude
	// further roots by the polynomial degree. If the search misses a narrow
	// opposite-side pocket, refuse instead of guessing from samples.
	for depth := 1; depth <= 12; depth++ {
		denominator := int64(1 << depth)
		for numerator := int64(1); numerator < denominator; numerator += 2 {
			middle := big.NewRat(numerator, denominator)
			sign := polynomial.BernsteinSign(coeff, middle)
			if sign == 0 {
				return nil, false
			}
			if sign != first {
				left, leftOK := revolveLoftFacesIsolateTimeRoot(coeff, lfZero(), middle)
				right, rightOK := revolveLoftFacesIsolateTimeRoot(coeff, middle, lfOne())
				if !leftOK || !rightOK || left.time.hi.Cmp(right.time.lo) >= 0 {
					return nil, false
				}
				return []revolveLoftFacesEndpointRoot{left, right}, true
			}
		}
	}
	return nil, false
}

func revolveLoftFacesIsolateTimeRoot(coeff []*big.Rat,
	lo, hi *big.Rat) (revolveLoftFacesEndpointRoot, bool) {
	loSign, hiSign := polynomial.BernsteinSign(coeff, lo), polynomial.BernsteinSign(coeff, hi)
	if loSign == 0 || hiSign == 0 || loSign == hiSign {
		return revolveLoftFacesEndpointRoot{}, false
	}
	direction := hiSign
	for range 64 {
		mid := lfMul(lfAdd(lo, hi), big.NewRat(1, 2))
		sign := polynomial.BernsteinSign(coeff, mid)
		if sign == 0 {
			return revolveLoftFacesEndpointRoot{
				time: loftFaceInterval{lo: mid, hi: mid}, direction: direction}, true
		}
		if sign == loSign {
			lo = mid
		} else {
			hi = mid
		}
	}
	return revolveLoftFacesEndpointRoot{
		time: loftFaceInterval{lo: lo, hi: hi}, direction: direction}, true
}
