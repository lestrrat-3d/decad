package smoothloft

import (
	"context"
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/polynomial"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/sketch/geom"
)

func circularFitIndex(profile momentinput.Profile) (int, error) {
	if len(profile.Holes) != 0 || len(profile.Outer.Segments) != 4 {
		return 0, twistUnsupported("circular fitted loft needs one spline and three lines")
	}
	fit := -1
	for i, raw := range profile.Outer.Segments {
		var start, end float64
		switch segment := raw.(type) {
		case sectionrecord.LineSeg:
			start, end = segment.TStart, segment.TEnd
		case sectionrecord.FitSplineSeg:
			start, end = segment.TStart, segment.TEnd
			if fit >= 0 {
				return 0, twistUnsupported("circular fitted loft needs one spline")
			}
			fit = i
		case sectionrecord.SplineSeg:
			start, end = segment.TStart, segment.TEnd
			if fit >= 0 {
				return 0, twistUnsupported("circular fitted loft needs one spline")
			}
			fit = i
		default:
			return 0, twistUnsupported("circular fitted loft needs only whole lines and one spline")
		}
		if !((start == 0 && end == 1) || (start == 1 && end == 0)) {
			return 0, twistUnsupported("circular fitted loft cannot use trimmed edges")
		}
	}
	if fit < 0 {
		return 0, twistUnsupported("circular fitted loft needs one spline")
	}
	return fit, nil
}

// circularFitSource identifies a complete cardinal circle in its recorded
// cyclic order. The returned quarter is the first source arc's chart index.
func circularFitSource(profile momentinput.Profile) (*big.Rat, freeform.RatPoint, int, error) {
	if len(profile.Holes) != 0 || len(profile.Outer.Segments) != 4 {
		return nil, freeform.RatPoint{}, 0, twistUnsupported("circular fitted loft needs four whole quarter arcs")
	}
	first, ok := profile.Outer.Segments[0].(sectionrecord.ArcSeg)
	if !ok || math.IsNaN(first.Start.U) || math.IsNaN(first.Start.V) ||
		math.IsInf(first.Start.U, 0) || math.IsInf(first.Start.V, 0) ||
		math.IsNaN(first.Center.U) || math.IsNaN(first.Center.V) ||
		math.IsInf(first.Center.U, 0) || math.IsInf(first.Center.V, 0) {
		return nil, freeform.RatPoint{}, 0, twistUnsupported("circular fitted loft needs finite cardinal arcs")
	}
	center := twistRat(first.Center)
	dx, dy := sub(rf(first.Start.U), center.U), sub(rf(first.Start.V), center.V)
	r := maxRat(abs(dx), abs(dy))
	if r.Sign() <= 0 || r.Cmp(rint(32)) > 0 {
		return nil, freeform.RatPoint{}, 0, twistUnsupported("circular fitted loft radius must be positive and at most 32 mm")
	}
	if add(mul(center.U, center.U), mul(center.V, center.V)).Cmp(mul(r, r)) >= 0 {
		return nil, freeform.RatPoint{}, 0, twistUnsupported("circular fitted loft origin must lie strictly inside the circle")
	}
	cardinal := [4]freeform.RatPoint{
		{U: add(center.U, r), V: center.V}, {U: center.U, V: add(center.V, r)},
		{U: sub(center.U, r), V: center.V}, {U: center.U, V: sub(center.V, r)},
	}
	start := -1
	for i, p := range cardinal {
		if twistSame(p, twistRat(first.Start)) {
			start = i
			break
		}
	}
	if start < 0 {
		return nil, freeform.RatPoint{}, 0, twistUnsupported("circular fitted loft needs cardinal arc endpoints")
	}
	for i, raw := range profile.Outer.Segments {
		arc, ok := raw.(sectionrecord.ArcSeg)
		if !ok || math.IsNaN(arc.Start.U) || math.IsNaN(arc.Start.V) || math.IsNaN(arc.End.U) || math.IsNaN(arc.End.V) ||
			math.IsInf(arc.Start.U, 0) || math.IsInf(arc.Start.V, 0) || math.IsInf(arc.End.U, 0) || math.IsInf(arc.End.V, 0) ||
			arc.Center != first.Center || arc.TStart != 0 || arc.TEnd != 1 ||
			!twistSame(twistRat(arc.Start), cardinal[(start+i)%4]) || !twistSame(twistRat(arc.End), cardinal[(start+i+1)%4]) {
			return nil, freeform.RatPoint{}, 0, twistUnsupported("circular fitted loft arc %d is not a whole counterclockwise cardinal quarter", i)
		}
	}
	return r, center, start, nil
}

// circularChart restricts one cardinal quarter to a fitted span's interval.
func circularChart(radius *big.Rat, center freeform.RatPoint, quarter int, lo, hi *big.Rat) rationalCurve {
	x := []*big.Rat{radius, radius, rint(0)}
	y := []*big.Rat{rint(0), radius, mul(rint(2), radius)}
	w := []*big.Rat{rint(1), rint(1), rint(2)}
	x, y, w = polynomial.BernsteinRestrict(x, lo, hi), polynomial.BernsteinRestrict(y, lo, hi),
		polynomial.BernsteinRestrict(w, lo, hi)
	px, py := freeform.RpFromBernstein(x), freeform.RpFromBernstein(y)
	switch quarter {
	case 1:
		px, py = polynomial.RpNeg(py), px
	case 2:
		px, py = polynomial.RpNeg(px), polynomial.RpNeg(py)
	case 3:
		px, py = py, polynomial.RpNeg(px)
	}
	pw := freeform.RpFromBernstein(w)
	return rationalCurve{x: polynomial.RpAdd(px, polynomial.RpScale(pw, center.U)),
		y: polynomial.RpAdd(py, polynomial.RpScale(pw, center.V)), w: pw}
}

func circularMappedChart(radius *big.Rat, center freeform.RatPoint, quarter int, lo, hi *big.Rat,
	controls []*big.Rat) rationalCurve {
	full := circularChart(radius, center, quarter, rint(0), rint(1))
	mapPoly := freeform.RpFromBernstein(controls)
	affine := polynomial.RatPoly{lo, sub(hi, lo)}
	q := rationalCompose(mapPoly, affine)
	return rationalCurve{x: rationalCompose(full.x, q), y: rationalCompose(full.y, q), w: rationalCompose(full.w, q)}
}

func fittedOnDenominator(span twistSpan, w polynomial.RatPoly) rationalCurve {
	x, y := make([]*big.Rat, len(span)), make([]*big.Rat, len(span))
	for i, p := range span {
		x[i], y[i] = p.U, p.V
	}
	return rationalCurve{x: polynomial.RpMul(freeform.RpFromBernstein(x), w),
		y: polynomial.RpMul(freeform.RpFromBernstein(y), w), w: w}
}

// BuildCircularFits constructs §14's exact four-arc to fitted transition.
func BuildCircularFits(ctx context.Context, profiles []momentinput.Profile,
	planes []sectionrecord.PlaneRecord, work []*freeform.FreeformWork) (Build, error) {
	return BuildCircularFitsWithMaps(ctx, profiles, planes, work, nil, false)
}

// BuildCircularFitsWithMaps applies four explicit cardinal-quarter maps. A nil
// map set retains BuildCircularFits' default correspondence.
func BuildCircularFitsWithMaps(ctx context.Context, profiles []momentinput.Profile,
	planes []sectionrecord.PlaneRecord, work []*freeform.FreeformWork, maps [][]*big.Rat, smoothCircularBands bool) (Build, error) {
	if maps != nil {
		if len(maps) != 4 {
			return Build{}, twistUnsupported("circular fitted loft needs four correspondence maps")
		}
		for _, controls := range maps {
			if len(controls) < 2 || len(controls) > 6 || controls[0] == nil || controls[len(controls)-1] == nil ||
				controls[0].Sign() != 0 || controls[len(controls)-1].Cmp(rint(1)) != 0 {
				return Build{}, twistUnsupported("circular fitted loft map needs two to six controls from zero to one")
			}
			for i := 1; i < len(controls); i++ {
				if controls[i] == nil || controls[i].Cmp(controls[i-1]) <= 0 {
					return Build{}, twistUnsupported("circular fitted loft map controls must strictly increase")
				}
			}
		}
	}
	if len(profiles) < 3 || len(profiles) > MaxTwistedSections || len(planes) != len(profiles) || len(work) != len(profiles) {
		return Build{}, twistUnsupported("circular fitted loft needs 3 to %d matching sections", MaxTwistedSections)
	}
	if err := ctx.Err(); err != nil {
		return Build{}, err
	}
	zs, err := twistPlanes(planes)
	if err != nil {
		return Build{}, err
	}
	radius, center, firstQuarter, err := circularFitSource(profiles[0])
	if err != nil {
		return Build{}, err
	}
	firstFit := 1
	radii := []*big.Rat{radius}
	for firstFit < len(profiles) {
		if _, err := circularFitIndex(profiles[firstFit]); err == nil {
			break
		}
		r, c, quarter, err := circularFitSource(profiles[firstFit])
		if err != nil {
			return Build{}, fmt.Errorf("circular prefix section %d: %w", firstFit, err)
		}
		if !twistSame(c, center) || quarter != firstQuarter {
			return Build{}, twistUnsupported("circular fitted loft prefix circles must share center and seam")
		}
		radii = append(radii, r)
		firstFit++
	}
	if firstFit == len(profiles) {
		return Build{}, twistUnsupported("circular fitted loft needs a fitted section")
	}
	sections := make([]twistSection, len(profiles)-firstFit)
	fitAt0, err := circularFitIndex(profiles[firstFit])
	if err != nil {
		return Build{}, err
	}
	firstSpline, isSpline := profiles[firstFit].Outer.Segments[fitAt0].(sectionrecord.SplineSeg)
	var knots []*big.Rat
	if isSpline {
		n := len(firstSpline.Control) - 3
		if n <= 0 {
			return Build{}, twistUnsupported("circular fitted loft spline needs at least four controls")
		}
		rawKnots := geom.ClampedKnots(len(firstSpline.Control))
		knots = make([]*big.Rat, n+1)
		for i := range knots {
			if firstSpline.TStart == 1 {
				knots[i] = sub(rint(1), rf(rawKnots[3+n-i]))
			} else {
				knots[i] = rf(rawKnots[3+i])
			}
		}
	}
	for i := range sections {
		if err := ctx.Err(); err != nil {
			return Build{}, err
		}
		if work[i+firstFit] == nil {
			return Build{}, twistUnsupported("circular fitted loft needs free-form work counters")
		}
		index, err := circularFitIndex(profiles[i+firstFit])
		if err != nil {
			return Build{}, err
		}
		currentSpline, currentIsSpline := profiles[i+firstFit].Outer.Segments[index].(sectionrecord.SplineSeg)
		if isSpline != currentIsSpline || (isSpline && len(currentSpline.Control) != len(firstSpline.Control)) {
			return Build{}, twistUnsupported("circular fitted loft spline kinds or control counts differ")
		}
		if isSpline && (currentSpline.TStart != firstSpline.TStart || currentSpline.TEnd != firstSpline.TEnd) {
			return Build{}, twistUnsupported("circular fitted loft spline record directions differ between sections")
		}
		sections[i].edges, err = gatherSplineOne(profiles[i+firstFit], work[i+firstFit], fitAt0, 4, circularFitIndex)
		if err != nil {
			return Build{}, err
		}
		spans := sections[i].edges[fitAt0]
		for j := 1; j < len(spans); j++ {
			a := fittedOnDenominator(spans[j-1], polynomial.RatPoly{rint(1)})
			b := fittedOnDenominator(spans[j], polynomial.RatPoly{rint(1)})
			if _, _, ok := rationalJoinSlopes(a, b); !ok {
				return Build{}, twistUnsupported("circular fitted loft spline section %d span %d has no exact G1 join", i, j)
			}
		}
	}
	var chartIntervals [][]circularInterval
	halfGroups, halfIntervals, useHalves := circularFlatQuarter(sections[0].edges, fitAt0, knots)
	if maps != nil && !useHalves {
		return Build{}, twistUnsupported("explicit circular correspondence needs a vertical flat and an even-span spline")
	}
	if useHalves {
		sections[0].edges, chartIntervals = halfGroups, halfIntervals
		for i := 1; i < len(sections); i++ {
			groups, intervals, ok := circularFlatQuarter(sections[i].edges, fitAt0, knots)
			if !ok {
				return Build{}, twistUnsupported("circular fitted loft flat correspondence differs between sections")
			}
			for edge := range intervals {
				if len(intervals[edge]) != len(chartIntervals[edge]) {
					return Build{}, twistUnsupported("circular fitted loft flat join counts differ")
				}
				for j, interval := range intervals[edge] {
					want := chartIntervals[edge][j]
					if interval.lo.Cmp(want.lo) != 0 || interval.hi.Cmp(want.hi) != 0 {
						return Build{}, twistUnsupported("circular fitted loft flat join positions differ in section %d, edge %d, span %d: [%s,%s] versus [%s,%s]",
							i, edge, j, interval.lo, interval.hi, want.lo, want.hi)
					}
				}
			}
			sections[i].edges = groups
		}
	}
	var patches [][]rationalTransition
	var edgeOf []int
	for edge, spans := range sections[0].edges {
		for j := range spans {
			quarter := (firstQuarter + edge) % 4
			interval := circularInterval{big.NewRat(int64(j), int64(len(spans))), big.NewRat(int64(j+1), int64(len(spans)))}
			face := edge
			if useHalves {
				quarter, interval = edge, chartIntervals[edge][j]
				face = (quarter - firstQuarter + 4) % 4
			}
			chart := circularChart(radius, center, quarter, interval.lo, interval.hi)
			if maps != nil {
				chart = circularMappedChart(radius, center, quarter, interval.lo, interval.hi, maps[quarter])
			}
			row := make([]rationalTransition, len(profiles)-1)
			prior := chart
			for i := range firstFit - 1 {
				next := circularChart(radii[i+1], center, quarter, interval.lo, interval.hi)
				if maps != nil {
					next = circularMappedChart(radii[i+1], center, quarter, interval.lo, interval.hi, maps[quarter])
				}
				row[i] = rationalTransition{a: prior, b: next, linear: !smoothCircularBands || i == 0}
				prior = next
			}
			for i, section := range sections {
				if len(section.edges[edge]) != len(spans) {
					return Build{}, twistUnsupported("circular fitted loft span counts differ")
				}
				next := fittedOnDenominator(section.edges[edge][j], chart.w)
				row[i+firstFit-1] = rationalTransition{a: prior, b: next}
				prior = next
			}
			patches = append(patches, row)
			edgeOf = append(edgeOf, face)
		}
	}
	patches, err = rationalSmoothPatches(ctx, patches, edgeOf, firstFit, maps != nil)
	if err != nil {
		return Build{}, err
	}
	built, err := buildSectionMesh(ctx, nil, patches, edgeOf, 4, planes, zs)
	if err != nil {
		return Build{}, err
	}
	built.CircularFitPrimitive = true
	return built, nil
}
