package decad

import (
	"context"
	"math"
	"math/big"
	"math/rand/v2"
	"slices"
	"testing"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/loftface"
	"github.com/lestrrat-3d/decad/internal/polynomial"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/revolvecontact"
	"github.com/lestrrat-3d/decad/internal/revolveloftfaces"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/stretchr/testify/require"
)

func TestMappedArcRadialTimeBandTwoExactTimeCrossings(t *testing.T) {
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	center := s.CreatePoint(0, 0)
	start := s.CreatePoint(2, 0)
	end := s.CreatePoint(0, 2)
	arc := s.CreateArc(center, start, end)
	segment := arcSeg{Center: planePoint{}, Start: planePoint{U: 2},
		End: planePoint{V: 2}, TStart: 0, TEnd: 1}
	zero, one := lfZero(), lfOne()
	first := loftFaceMap{Constant: loftFaceRatVec{zero, zero, zero},
		Q0: loftFaceRatVec{zero, zero, zero},
		QU: loftFaceRatVec{zero, one, zero},
		QV: loftFaceRatVec{zero, zero, one},
		G0: one, GU: zero, GV: zero}
	second := loftFaceMap{Constant: loftFaceRatVec{one, zero, zero},
		Q0: loftFaceRatVec{zero, zero, zero},
		QU: loftFaceRatVec{zero, zero, one},
		QV: loftFaceRatVec{zero, big.NewRat(-1, 1), zero},
		G0: one, GU: zero, GV: zero}
	trim := &loftFacesTrimRecord{
		Source: &pointSectionRecord{
			Profile:   profileRecord{Outer: loopRecord{Segments: []curveSegment{segment}}},
			Source:    &pointSectionSource{Entities: []sketch.Entity{arc}},
			SourceBox: [4]float64{-3, 3, -3, 3},
		},
		Maps: []loftFaceMap{first, second},
	}
	meridian := revolveSectionSpan{Start: planePoint{V: 1.5}, End: planePoint{U: 1, V: 1.5}}
	_, certified := revolveloftfaces.MappedArcFiniteSide(context.Background(), &loftFacesTrimRecord{}, 0, meridian)
	require.False(t, certified)
	got, certified := revolveloftfaces.MappedArcRadialTimeBand(context.Background(), trim, 0, 0, meridian)
	require.True(t, certified)
	require.Same(t, arc, got.Arc)
	require.Equal(t, 0, got.Segment)
	require.Equal(t, 0, got.Band)
	for i, want := range [...]string{"7/4", "-9/4", "7/4"} {
		require.Equal(t, want, got.Q[i].RatString())
	}
	require.Zero(t, got.X[0].Sign())
	require.Equal(t, 0, got.X[1].Cmp(lfOne()))
	segment.TStart, segment.TEnd = 1, 0
	trim.Source.Profile.Outer.Segments[0] = segment
	_, certified = revolveloftfaces.MappedArcRadialTimeBand(context.Background(), trim, 0, 0, meridian)
	require.True(t, certified)
	segment.Start.U = 2.1
	trim.Source.Profile.Outer.Segments[0] = segment
	_, certified = revolveloftfaces.MappedArcRadialTimeBand(context.Background(), trim, 0, 0, meridian)
	require.False(t, certified)
	segment.Start.U = 2
	trim.Source.Profile.Outer.Segments[0] = segment

	// A shear breaks the all-time radial identity even though the cap still
	// maps every sampled endpoint to a finite point.
	trim.Maps[1].QV[2] = big.NewRat(1, 100)
	_, certified = revolveloftfaces.MappedArcRadialTimeBand(context.Background(), trim, 0, 0, meridian)
	require.False(t, certified)
	trim.Maps[1].QV[2] = zero
	require.True(t, s.RemoveEntity(arc))
	_, certified = revolveloftfaces.MappedArcRadialTimeBand(context.Background(), trim, 0, 0, meridian)
	require.False(t, certified)
}

func TestMappedEndpointTimeRoots(t *testing.T) {
	two := []*big.Rat{big.NewRat(-1, 1), big.NewRat(2, 1), big.NewRat(-1, 1)}
	q := [3][]*big.Rat{{two[0]}, {two[1]}, {two[2]}}
	roots, ok := loftface.EndpointTimeRoots(q, true)
	if !ok || len(roots) != 2 || roots[0].Direction != 1 || roots[1].Direction != -1 {
		t.Fatalf("expected two ordered transverse roots, got %+v, %t", roots, ok)
	}
	if roots[0].Time.Lo.Cmp(big.NewRat(1, 8)) <= 0 ||
		roots[0].Time.Hi.Cmp(big.NewRat(1, 4)) >= 0 ||
		roots[1].Time.Lo.Cmp(big.NewRat(3, 4)) <= 0 ||
		roots[1].Time.Hi.Cmp(big.NewRat(7, 8)) >= 0 {
		t.Fatalf("roots escaped certified separate intervals: %+v", roots)
	}
	for _, root := range roots {
		if polynomial.BernsteinValue(two, root.Time.Lo).Sign()*polynomial.BernsteinValue(two, root.Time.Hi).Sign() >= 0 {
			t.Fatalf("root bracket does not change Side: %+v", root)
		}
	}

	q = [3][]*big.Rat{{big.NewRat(-1, 1)}, {big.NewRat(0, 1)}, {big.NewRat(1, 1)}}
	roots, ok = loftface.EndpointTimeRoots(q, false)
	if !ok || len(roots) != 1 || roots[0].Direction != 1 {
		t.Fatalf("expected one transverse root, got %+v, %t", roots, ok)
	}
	q = [3][]*big.Rat{{big.NewRat(1, 1)}, {big.NewRat(1, 1)}, {big.NewRat(1, 1)}}
	roots, ok = loftface.EndpointTimeRoots(q, false)
	if !ok || len(roots) != 0 {
		t.Fatalf("expected certified strict side, got %+v, %t", roots, ok)
	}
	q = [3][]*big.Rat{{big.NewRat(0, 1)}, {big.NewRat(1, 1)}, {big.NewRat(-1, 1)}}
	if _, ok = loftface.EndpointTimeRoots(q, false); ok {
		t.Fatal("boundary contact must refuse")
	}
}

func TestMappedEndpointFiniteSection(t *testing.T) {
	section := revolveSectionSpan{Start: planePoint{U: 5.7, V: 2.28},
		End: planePoint{U: 5.8, V: 3.3}}
	x0, x1 := big.NewRat(5, 1), big.NewRat(6, 1)
	outside := loftface.EndpointRoot{Time: loftFaceInterval{
		Lo: big.NewRat(1, 8), Hi: big.NewRat(1, 8)}}
	outside, ok := revolveloftfaces.RevolveLoftFacesEndpointAxialClass(outside, x0, x1, section)
	if !ok || outside.Finite || outside.X.Hi.Cmp(big.NewRat(41, 8)) != 0 {
		t.Fatalf("expected a certified axial miss, got %+v, %t", outside, ok)
	}
	inside := loftface.EndpointRoot{Time: loftFaceInterval{
		Lo: big.NewRat(3, 4), Hi: big.NewRat(3, 4)}}
	inside, ok = revolveloftfaces.RevolveLoftFacesEndpointAxialClass(inside, x0, x1, section)
	if !ok || !inside.Finite || inside.X.Lo.Cmp(big.NewRat(23, 4)) != 0 {
		t.Fatalf("expected a certified finite crossing, got %+v, %t", inside, ok)
	}
	boundaryTime := lfSub(lfF(section.Start.U), x0)
	boundary := loftface.EndpointRoot{Time: loftFaceInterval{
		Lo: boundaryTime, Hi: boundaryTime}}
	if _, ok := revolveloftfaces.RevolveLoftFacesEndpointAxialClass(boundary, x0, x1, section); ok {
		t.Fatal("a contact on the section end must refuse")
	}
}

func TestMappedSpanAxialHull(t *testing.T) {
	makeMap := func(offset int64) loftFaceMap {
		zero, one := big.NewRat(0, 1), big.NewRat(1, 1)
		return loftFaceMap{
			Constant: loftFaceRatVec{zero, zero, zero},
			Q0:       loftFaceRatVec{big.NewRat(offset, 1), zero, zero},
			QU:       loftFaceRatVec{one, zero, zero},
			QV:       loftFaceRatVec{zero, zero, zero},
			G0:       one, GU: zero, GV: zero,
		}
	}
	span := freeform.BezierSpan{
		{U: big.NewRat(0, 1), V: big.NewRat(0, 1)},
		{U: big.NewRat(1, 3), V: big.NewRat(0, 1)},
		{U: big.NewRat(2, 3), V: big.NewRat(0, 1)},
		{U: big.NewRat(1, 1), V: big.NewRat(0, 1)},
	}
	first, last := makeMap(5), makeMap(6)
	x, ok := loftface.SpanXRange(first, last, span,
		big.NewRat(1, 2), big.NewRat(1, 1), big.NewRat(1, 8), big.NewRat(1, 4))
	if !ok || x.Lo.Cmp(big.NewRat(45, 8)) != 0 || x.Hi.Cmp(big.NewRat(25, 4)) != 0 {
		t.Fatalf("expected exact affine mapped X hull, got %+v, %t", x, ok)
	}
	section := revolveSectionSpan{Start: planePoint{U: 6.5, V: 1},
		End: planePoint{U: 7, V: 2}}
	if !revolveloftfaces.RevolveLoftFacesSpanAxialMiss(first, last, span, section,
		big.NewRat(1, 2), big.NewRat(1, 1), big.NewRat(1, 8), big.NewRat(1, 4)) {
		t.Fatal("mapped span should miss the finite section")
	}
	section.Start.U = 6
	if revolveloftfaces.RevolveLoftFacesSpanAxialMiss(first, last, span, section,
		big.NewRat(1, 2), big.NewRat(1, 1), big.NewRat(1, 8), big.NewRat(1, 4)) {
		t.Fatal("overlapping mapped span cannot be clipped")
	}
}

func TestMappedRadialJointTimePartition(t *testing.T) {
	q := [3]*big.Rat{big.NewRat(-1, 1), big.NewRat(2, 1), big.NewRat(-1, 1)}
	rows := [3][]*big.Rat{{q[0]}, {q[1]}, {q[2]}}
	roots, ok := loftface.EndpointTimeRoots(rows, true)
	if !ok {
		t.Fatal("exact quadratic roots were not isolated")
	}
	pieces, ok := loftface.PartitionTime(q, roots)
	if !ok || len(pieces) != 5 {
		t.Fatalf("expected five time pieces, got %+v, %t", pieces, ok)
	}
	for i, want := range []struct {
		side, root     int
		loOpen, hiOpen bool
	}{
		{-1, -1, false, true}, {0, 0, false, false},
		{1, -1, true, true}, {0, 1, false, false},
		{-1, -1, true, false},
	} {
		if pieces[i].Side != want.side || pieces[i].RootIndex != want.root ||
			pieces[i].LoOpen != want.loOpen || pieces[i].HiOpen != want.hiOpen ||
			pieces[i].Lo.Cmp(pieces[i].Hi) > 0 {
			t.Fatalf("piece %d lost its exact side or root: %+v", i, pieces[i])
		}
		if i != 0 && pieces[i-1].Hi.Cmp(pieces[i].Lo) != 0 {
			t.Fatalf("time partition has a gap at piece %d", i)
		}
	}
	roots[0].Direction = -1
	if _, ok := loftface.PartitionTime(q, roots); ok {
		t.Fatal("inconsistent root direction must refuse")
	}
}

func TestMappedRadialJointExactRootBoundary(t *testing.T) {
	q := [3]*big.Rat{big.NewRat(-1, 1), big.NewRat(0, 1), big.NewRat(1, 1)}
	rows := [3][]*big.Rat{{q[0]}, {q[1]}, {q[2]}}
	roots, ok := loftface.EndpointTimeRoots(rows, false)
	if !ok || len(roots) != 1 || roots[0].Time.Lo.Cmp(big.NewRat(1, 2)) != 0 ||
		roots[0].Time.Hi.Cmp(big.NewRat(1, 2)) != 0 {
		t.Fatalf("expected an exact dyadic root, got %+v, %t", roots, ok)
	}
	pieces, ok := loftface.PartitionTime(q, roots)
	if !ok || len(pieces) != 3 || !pieces[0].HiOpen || !pieces[2].LoOpen ||
		pieces[1].LoOpen || pieces[1].HiOpen ||
		pieces[0].Hi.Cmp(pieces[1].Lo) != 0 || pieces[1].Hi.Cmp(pieces[2].Lo) != 0 {
		t.Fatalf("exact contact must belong only to the root piece: %+v, %t", pieces, ok)
	}
}

func TestMappedFitKnotRootCellRequiresFiniteSourceSliver(t *testing.T) {
	mapAt := func(x, y int64) loftFaceMap {
		zero, one := lfZero(), lfOne()
		return loftFaceMap{
			Constant: loftFaceRatVec{zero, zero, zero},
			Q0:       loftFaceRatVec{big.NewRat(x, 10), big.NewRat(y, 1), zero},
			QU:       loftFaceRatVec{zero, one, zero},
			QV:       loftFaceRatVec{zero, zero, zero},
			G0:       one, GU: zero, GV: zero,
		}
	}
	trim := &loftFacesTrimRecord{Maps: []loftFaceMap{mapAt(4, 0), mapAt(6, 1)}}
	span := freeform.BezierSpan{
		{U: big.NewRat(0, 1), V: lfZero()},
		{U: big.NewRat(1, 3), V: lfZero()},
		{U: big.NewRat(2, 3), V: lfZero()},
		{U: big.NewRat(1, 1), V: lfZero()},
	}
	finite := revolveSectionSpan{Start: planePoint{U: 0.45, V: 1.5},
		End: planePoint{U: 0.55, V: 1.5}}
	roots, ok := revolveloftfaces.RevolveLoftFacesEndpointFiniteRoots(trim, finite, span, 0, true)
	if !ok || len(roots) != 1 || !roots[0].Finite ||
		roots[0].Time.Lo.Cmp(big.NewRat(1, 2)) != 0 {
		t.Fatalf("expected one exact finite knot time, got %+v, %t", roots, ok)
	}
	cell, ok := revolveloftfaces.RevolveLoftFacesKnotRootCell(trim, finite, span, 0, true, roots[0])
	if !ok || !cell.Finite || cell.OutsideSide != -1 || cell.SDerivative != 1 ||
		cell.X.Lo.Cmp(big.NewRat(1, 2)) != 0 || cell.X.Hi.Cmp(big.NewRat(1, 2)) != 0 {
		t.Fatalf("exact contact must have one bounded source sliver: %+v, %t", cell, ok)
	}
	outside := revolveSectionSpan{Start: planePoint{U: 0.7, V: 1.5},
		End: planePoint{U: 0.8, V: 1.5}}
	roots, ok = revolveloftfaces.RevolveLoftFacesEndpointFiniteRoots(trim, outside, span, 0, true)
	if !ok || len(roots) != 1 || roots[0].Finite {
		t.Fatalf("expected one exact axial miss, got %+v, %t", roots, ok)
	}
	cell, ok = revolveloftfaces.RevolveLoftFacesKnotRootCell(trim, outside, span, 0, true, roots[0])
	if !ok || cell.Finite || cell.X.Hi.Cmp(lfF(outside.Start.U)) >= 0 {
		t.Fatalf("nonfinite contact must be clipped by exact X: %+v, %t", cell, ok)
	}
	roots[0].Direction = -roots[0].Direction
	if _, ok := revolveloftfaces.RevolveLoftFacesKnotRootCell(trim, outside, span, 0, true, roots[0]); ok {
		t.Fatal("a root with the wrong source-side direction must refuse")
	}
}

func TestRevolveLoftFacesOpenTimeExcludesExactPointRoot(t *testing.T) {
	// Q(s,t)=s+t-1/2 vanishes at (s=0,t=1/2). The strict side starts
	// immediately after that exact time, but includes no root point.
	q := [3][]*big.Rat{
		{big.NewRat(-1, 2), big.NewRat(1, 2)},
		{big.NewRat(0, 1), big.NewRat(1, 1)},
		{big.NewRat(1, 2), big.NewRat(3, 2)},
	}
	require.Equal(t, loftface.SideOutside,
		polynomial.BernsteinRectOpenSide(q, big.NewRat(1, 2), lfOne(), true, false))
	require.Zero(t,
		polynomial.BernsteinRectOpenSide(q, big.NewRat(1, 2), lfOne(), false, false))
	// On the left, Q has a moving source root that reaches s=0 at the
	// excluded time endpoint. The strict-side shortcut must refuse it.
	require.Zero(t,
		polynomial.BernsteinRectOpenSide(q, lfZero(), big.NewRat(1, 2), false, true))
}

func TestRevolveLoftFacesOpenMovingCrossingRefuses(t *testing.T) {
	mapAt := func(x, y *big.Rat) loftFaceMap {
		zero, one := lfZero(), lfOne()
		return loftFaceMap{Constant: loftFaceRatVec{zero, zero, zero},
			Q0: loftFaceRatVec{x, y, zero},
			QU: loftFaceRatVec{one, one, zero},
			QV: loftFaceRatVec{zero, zero, zero},
			G0: one, GU: zero, GV: zero}
	}
	first := mapAt(lfZero(), big.NewRat(1, 2))
	last := mapAt(lfOne(), big.NewRat(3, 2))
	span := freeform.BezierSpan{
		{U: lfZero(), V: lfZero()}, {U: lfOne(), V: lfZero()},
	}
	section := revolveSectionSpan{Start: planePoint{U: 0, V: 1},
		End: planePoint{U: 2, V: 1}}
	m, b := revolveloftfaces.RevolveLoftFacesMeridian(section)
	q := loftface.MeridianQRows(first, last, span, m, b)
	j := loftface.MeridianJRows(first, last, span, q)
	remaining := revolveloftfaces.RevolveLoftFacesMaxBandCells
	_, ok, _ := revolveloftfaces.RevolveLoftFacesOpenTimeSlab(t.Context(), q, j,
		first, last, span, section, 0, 0, 0,
		lfZero(), big.NewRat(1, 2), false, true, &remaining)
	require.False(t, ok)
}

// legacyRevolveLoftFacesBracketTimeRoot is revolveLoftFacesBracketTimeRoot as
// it was when every restriction repeated the band-time stage on q.
func legacyRevolveLoftFacesBracketTimeRoot(q, j [3][]*big.Rat,
	segment, span, band int, tLo, tHi *big.Rat) ([]revolveloftfaces.RevolveLoftFacesMappedCell, bool) {
	box := polynomial.BernsteinRectRestrict(q, lfZero(), lfOne(), tLo, tHi)
	for _, orientation := range []int{1, -1} {
		oriented := box
		if orientation < 0 {
			oriented = loftface.NegateRows(box)
		}
		if polynomial.BernsteinRowsPointSign(oriented, lfZero()) != -1 ||
			polynomial.BernsteinRowsPointSign(oriented, lfOne()) != 1 {
			continue
		}
		sLo, sHi := lfZero(), lfOne()
		for range 10 {
			mid := lfMul(lfAdd(sLo, sHi), big.NewRat(1, 2))
			switch polynomial.BernsteinRowsPointSign(oriented, mid) {
			case -1:
				sLo = mid
			case 1:
				sHi = mid
			default:
				goto check
			}
		}
	check:
		middle := polynomial.BernsteinRectRestrict(q, sLo, sHi, tLo, tHi)
		if orientation < 0 {
			middle = loftface.NegateRows(middle)
		}
		if !polynomial.BernsteinRowsCross(middle) {
			continue
		}
		if sLo.Sign() > 0 {
			left := polynomial.BernsteinRectRestrict(q, lfZero(), sLo, tLo, tHi)
			if orientation > 0 {
				left = loftface.NegateRows(left)
			}
			if !polynomial.BernsteinRowsPositive(left) {
				continue
			}
		}
		if sHi.Cmp(lfOne()) < 0 {
			right := polynomial.BernsteinRectRestrict(q, sHi, lfOne(), tLo, tHi)
			if orientation < 0 {
				right = loftface.NegateRows(right)
			}
			if !polynomial.BernsteinRowsPositive(right) {
				continue
			}
		}
		order := polynomial.BernsteinRectRestrict(j, sLo, sHi, tLo, tHi)
		if orientation < 0 {
			order = loftface.NegateRows(order)
		}
		xDirection := 0
		if polynomial.BernsteinRowsPositive(order) {
			xDirection = 1
		} else if polynomial.BernsteinRowsPositive(loftface.NegateRows(order)) {
			xDirection = -1
		}
		if xDirection == 0 {
			continue
		}
		makeCell := func(side int, a, z *big.Rat) revolveloftfaces.RevolveLoftFacesMappedCell {
			return revolveloftfaces.RevolveLoftFacesMappedCell{Segment: segment, Span: span,
				Band: band, Side: side, SLo: a, SHi: z, TLo: tLo, THi: tHi}
		}
		var cells []revolveloftfaces.RevolveLoftFacesMappedCell
		if sLo.Sign() > 0 {
			cells = append(cells, makeCell(-orientation, lfZero(), sLo))
		}
		crossing := makeCell(loftface.SideCrossed, sLo, sHi)
		crossing.XDirection = xDirection
		cells = append(cells, crossing)
		if sHi.Cmp(lfOne()) < 0 {
			cells = append(cells, makeCell(orientation, sHi, lfOne()))
		}
		return cells, true
	}
	return nil, false
}

// legacyRevolveLoftFacesKnotRootCell is revolveLoftFacesKnotRootCellWith as it
// was when every restriction repeated the band-time stage on q.
func legacyRevolveLoftFacesKnotRootCell(trim *loftFacesTrimRecord,
	section revolveSectionSpan, span freeform.BezierSpan, band int,
	atEnd bool, root loftface.EndpointRoot) (revolveloftfaces.RevolveLoftFacesJointCell, bool) {
	var empty revolveloftfaces.RevolveLoftFacesJointCell
	if trim == nil || band < 0 || band+1 >= len(trim.Maps) ||
		root.Time.Lo == nil || root.Time.Hi == nil {
		return empty, false
	}
	m, b := revolveloftfaces.RevolveLoftFacesMeridian(section)
	q := loftface.MeridianQRows(trim.Maps[band], trim.Maps[band+1], span, m, b)
	endpoint := lfZero()
	if atEnd {
		endpoint = lfOne()
	}
	endQ := []*big.Rat{
		polynomial.BernsteinValue(q[0], endpoint),
		polynomial.BernsteinValue(q[1], endpoint),
		polynomial.BernsteinValue(q[2], endpoint),
	}
	loSign, hiSign := polynomial.BernsteinSign(endQ, root.Time.Lo),
		polynomial.BernsteinSign(endQ, root.Time.Hi)
	if root.Time.Lo.Cmp(root.Time.Hi) == 0 {
		if loSign != 0 ||
			polynomial.BernsteinSign(polynomial.BernsteinDeriv(endQ), root.Time.Lo) != root.Direction {
			return empty, false
		}
	} else if loSign != -root.Direction || hiSign != root.Direction {
		return empty, false
	}
	startX, endX := lfF(section.Start.U), lfF(section.End.U)
	if startX.Cmp(endX) > 0 {
		startX, endX = endX, startX
	}
	for depth := range 13 {
		width := big.NewRat(1, int64(1<<depth))
		sLo, sHi := lfZero(), width
		if atEnd {
			sLo, sHi = lfSub(lfOne(), width), lfOne()
		}
		box := polynomial.BernsteinRectRestrict(q, sLo, sHi,
			root.Time.Lo, root.Time.Hi)

		var derivative [3][]*big.Rat
		for row := range box {
			derivative[row] = polynomial.BernsteinDeriv(box[row])
		}
		direction := 0
		if polynomial.BernsteinRowsPositive(derivative) {
			direction = 1
		} else if polynomial.BernsteinRowsPositive(loftface.NegateRows(derivative)) {
			direction = -1
		}
		if direction == 0 {
			continue
		}
		fixed := lfZero()
		if !atEnd {
			fixed = lfOne()
		}
		fixedSide := polynomial.BernsteinRowsPointSign(box, fixed)
		if fixedSide == 0 {
			continue
		}
		if depth > 0 {
			outerLo, outerHi := lfZero(), sLo
			if !atEnd {
				outerLo, outerHi = sHi, lfOne()
			}
			outer := polynomial.BernsteinRectRestrict(q, outerLo, outerHi,
				root.Time.Lo, root.Time.Hi)

			if fixedSide > 0 && !polynomial.BernsteinRowsPositive(outer) ||
				fixedSide < 0 && !polynomial.BernsteinRowsPositive(loftface.NegateRows(outer)) {
				continue
			}
		}
		x, ok := loftface.SpanXRange(trim.Maps[band], trim.Maps[band+1],
			span, sLo, sHi, root.Time.Lo, root.Time.Hi)
		if !ok || root.Finite && (x.Lo.Cmp(startX) <= 0 || x.Hi.Cmp(endX) >= 0) ||
			!root.Finite && x.Hi.Cmp(startX) >= 0 && x.Lo.Cmp(endX) <= 0 {
			continue
		}
		return revolveloftfaces.RevolveLoftFacesJointCell{SLo: sLo, SHi: sHi,
			Time: root.Time, OutsideSide: fixedSide,
			SDerivative: direction, X: x, Finite: root.Finite}, true
	}
	return empty, false
}

// timeStageInterval draws lo < hi in [0, 1]; each end is exactly 0 or 1 in a
// quarter of the draws, dyadic in 40 %, and otherwise not dyadic.
func timeStageInterval(rng *rand.Rand) (*big.Rat, *big.Rat) {
	for {
		end := func(exact *big.Rat) *big.Rat {
			switch d := rng.IntN(20); {
			case d < 5:
				return exact
			case d < 13:
				return big.NewRat(rng.Int64N(1<<8), 1<<8)
			default:
				q := rng.Int64N(97) + 3
				return big.NewRat(rng.Int64N(q+1), q)
			}
		}
		lo, hi := end(new(big.Rat)), end(big.NewRat(1, 1))
		if lo.Cmp(hi) < 0 {
			return lo, hi
		}
	}
}

func timeStageSmallRat(rng *rand.Rand, scale int64) *big.Rat {
	return big.NewRat(rng.Int64N(2*scale+1)-scale, rng.Int64N(7)+1)
}

func requireMappedCellsSame(t *testing.T, want, got []revolveloftfaces.RevolveLoftFacesMappedCell) {
	t.Helper()
	require.Len(t, got, len(want))
	for i := range want {
		w, g := want[i], got[i]
		require.Equal(t, w.Segment, g.Segment)
		require.Equal(t, w.Span, g.Span)
		require.Equal(t, w.Band, g.Band)
		require.Equal(t, w.Side, g.Side)
		require.Equal(t, w.XDirection, g.XDirection)
		require.Zero(t, w.SLo.Cmp(g.SLo), "cell %d sLo", i)
		require.Zero(t, w.SHi.Cmp(g.SHi), "cell %d sHi", i)
		require.Zero(t, w.TLo.Cmp(g.TLo), "cell %d tLo", i)
		require.Zero(t, w.THi.Cmp(g.THi), "cell %d tHi", i)
	}
}

func requireTimeStageBracketRootsMatch(t *testing.T) {
	t.Helper()
	rng := rand.New(rand.NewPCG(97, 101))
	var found, missed int
	for trial := range 400 {
		var q, j [3][]*big.Rat
		length, jLength := 3+rng.IntN(5), 2+rng.IntN(4)
		for row := range q {
			amplitude := big.NewRat(rng.Int64N(9)+1, rng.Int64N(4)+1)
			step := big.NewRat(rng.Int64N(5)+1, rng.Int64N(4)+1)
			q[row] = make([]*big.Rat, length)
			for k := range q[row] {
				c := new(big.Rat).Sub(new(big.Rat).Mul(big.NewRat(int64(k), 1), step), amplitude)
				q[row][k] = c.Add(c, big.NewRat(rng.Int64N(7)-3, rng.Int64N(9)+4))
			}
			j[row] = make([]*big.Rat, jLength)
			for k := range j[row] {
				switch trial % 3 {
				case 0:
					j[row][k] = big.NewRat(rng.Int64N(5)+1, rng.Int64N(4)+1)
				case 1:
					j[row][k] = big.NewRat(-rng.Int64N(5)-1, rng.Int64N(4)+1)
				default:
					j[row][k] = timeStageSmallRat(rng, 5)
				}
			}
		}
		tLo, tHi := timeStageInterval(rng)
		wantCells, wantOK := legacyRevolveLoftFacesBracketTimeRoot(q, j, 3, 4, 5, tLo, tHi)
		gotCells, gotOK := revolveloftfaces.RevolveLoftFacesBracketTimeRoot(
			polynomial.BernsteinRectTimeRestrict(q, tLo, tHi), j, 3, 4, 5, tLo, tHi)
		require.Equal(t, wantOK, gotOK, "trial %d", trial)
		requireMappedCellsSame(t, wantCells, gotCells)
		if gotOK {
			found++
		} else {
			missed++
		}
	}
	require.GreaterOrEqual(t, found, 100)
	require.GreaterOrEqual(t, missed, 20)
}

func requireTimeStageKnotRootCellsMatch(t *testing.T) {
	t.Helper()
	rng := rand.New(rand.NewPCG(131, 137))
	mapAt := func(x, y int64, qv loftFaceRatVec) loftFaceMap {
		zero, one := lfZero(), lfOne()
		return loftFaceMap{
			Constant: loftFaceRatVec{zero, zero, zero},
			Q0:       loftFaceRatVec{big.NewRat(x, 10), big.NewRat(y, 1), zero},
			QU:       loftFaceRatVec{zero, one, zero},
			QV:       qv,
			G0:       one, GU: zero, GV: zero,
		}
	}
	var withCell int
	for trial := range 35 {
		qv := loftFaceRatVec{lfZero(), lfZero(), lfZero()}
		if rng.IntN(2) == 0 {
			qv = loftFaceRatVec{big.NewRat(rng.Int64N(41)-20, 10), big.NewRat(rng.Int64N(41)-20, 10), lfZero()}
		}
		trim := &loftFacesTrimRecord{Maps: []loftFaceMap{
			mapAt(rng.Int64N(9)+1, rng.Int64N(7)-3, qv),
			mapAt(rng.Int64N(9)+1, rng.Int64N(7)-3, qv),
			mapAt(rng.Int64N(9)+1, rng.Int64N(7)-3, qv),
		}}
		band := rng.IntN(2)
		var us [4]int64
		for i := range us {
			us[i] = rng.Int64N(31)
		}
		// Sorting keeps the span monotone in U.
		for i := range us {
			for k := i + 1; k < len(us); k++ {
				if us[k] < us[i] {
					us[i], us[k] = us[k], us[i]
				}
			}
		}
		var span freeform.BezierSpan
		for _, u := range us {
			span = append(span, freeform.RatPoint{U: big.NewRat(u, 30), V: lfZero()})
		}
		start := 0.05 + float64(rng.IntN(45))/100
		end := start + 0.02 + float64(rng.IntN(45))/100
		section := revolveSectionSpan{Start: planePoint{U: start, V: 1.5}, End: planePoint{U: end, V: 1.5}}
		for _, atEnd := range []bool{false, true} {
			roots, ok := revolveloftfaces.RevolveLoftFacesEndpointFiniteRoots(trim, section, span, band, atEnd)
			if !ok {
				continue
			}
			// The isolated brackets are narrow. Widened copies keep the
			// root inside and make the time stage matter.
			var cases []loftface.EndpointRoot
			for _, root := range roots {
				cases = append(cases, root)
				for _, w := range []*big.Rat{big.NewRat(1, 64), big.NewRat(1, 8)} {
					wide := root
					wide.Time.Lo = loftface.RatMax(lfZero(), lfSub(root.Time.Lo, w))
					wide.Time.Hi = ratMinOf(lfOne(), lfAdd(root.Time.Hi, w))
					cases = append(cases, wide)
				}
			}
			for _, root := range cases {
				want, wantOK := legacyRevolveLoftFacesKnotRootCell(trim, section, span, band, atEnd, root)
				got, gotOK := revolveloftfaces.RevolveLoftFacesKnotRootCell(trim, section, span, band, atEnd, root)
				require.Equal(t, wantOK, gotOK, "trial %d atEnd %t", trial, atEnd)
				if !gotOK {
					continue
				}
				withCell++
				require.Equal(t, want.Finite, got.Finite)
				require.Equal(t, want.OutsideSide, got.OutsideSide)
				require.Equal(t, want.SDerivative, got.SDerivative)
				require.Zero(t, want.SLo.Cmp(got.SLo))
				require.Zero(t, want.SHi.Cmp(got.SHi))
				require.Zero(t, want.Time.Lo.Cmp(got.Time.Lo))
				require.Zero(t, want.Time.Hi.Cmp(got.Time.Hi))
				require.Zero(t, want.X.Lo.Cmp(got.X.Lo))
				require.Zero(t, want.X.Hi.Cmp(got.X.Hi))
			}
		}
	}
	require.GreaterOrEqual(t, withCell, 30)
}

// TestCurvedTimeStageCellsMatchLegacy compares the two routines that now share
// one band-time restriction across their source restrictions with verbatim
// copies of the code that repeated it. The curved fixture tests do not notice
// a wrong source range in the bracket-root search.
func TestCurvedTimeStageCellsMatchLegacy(t *testing.T) {
	t.Run("bracket root", requireTimeStageBracketRootsMatch)
	t.Run("knot root cell", requireTimeStageKnotRootCellsMatch)
}

func ratMinOf(a, b *big.Rat) *big.Rat {
	if a.Cmp(b) < 0 {
		return a
	}
	return b
}

func TestRevolveLoftFacesOwnedTimeRootsRemainDisjoint(t *testing.T) {
	knotPoint, arcPoint := new(sketch.Point), new(sketch.Point)
	makeRoot := func(lo, hi *big.Rat, kind revolveloftfaces.RevolveLoftFacesJointKind,
		point *sketch.Point, atEnd bool) revolveloftfaces.RevolveLoftFacesOwnedRoot {
		sLo, sHi := big.NewRat(0, 1), big.NewRat(1, 8)
		if atEnd {
			sLo, sHi = big.NewRat(7, 8), big.NewRat(1, 1)
		}
		return revolveloftfaces.RevolveLoftFacesOwnedRoot{
			Root: loftface.EndpointRoot{
				Time: loftFaceInterval{Lo: lo, Hi: hi}, Finite: true},
			Cell: revolveloftfaces.RevolveLoftFacesJointCell{
				Time: loftFaceInterval{Lo: lo, Hi: hi},
				SLo:  sLo, SHi: sHi, Finite: true},
			Kind: kind, Point: point, AtEnd: atEnd,
		}
	}
	knot := makeRoot(big.NewRat(1, 4), big.NewRat(1, 3),
		revolveloftfaces.RevolveLoftFacesFitKnotContact, knotPoint, false)
	arc := makeRoot(big.NewRat(2, 3), big.NewRat(3, 4),
		revolveloftfaces.RevolveLoftFacesRadialArcContact, arcPoint, true)
	roots := []revolveloftfaces.RevolveLoftFacesOwnedRoot{arc, knot}
	require.True(t, revolveloftfaces.RevolveLoftFacesOrderOwnedRoots(roots))
	require.Equal(t, revolveloftfaces.RevolveLoftFacesFitKnotContact, roots[0].Kind)
	require.Same(t, knotPoint, roots[0].Point)
	require.Equal(t, revolveloftfaces.RevolveLoftFacesRadialArcContact, roots[1].Kind)
	require.Same(t, arcPoint, roots[1].Point)

	overlap := makeRoot(big.NewRat(1, 3), big.NewRat(1, 2),
		revolveloftfaces.RevolveLoftFacesRadialArcContact, arcPoint, true)
	require.False(t, revolveloftfaces.RevolveLoftFacesOrderOwnedRoots([]revolveloftfaces.RevolveLoftFacesOwnedRoot{knot, overlap}))
	wrongEnd := arc
	wrongEnd.Cell.SHi = big.NewRat(7, 8)
	require.False(t, revolveloftfaces.RevolveLoftFacesOrderOwnedRoots([]revolveloftfaces.RevolveLoftFacesOwnedRoot{knot, wrongEnd}))
}

func TestCurvedMappedSourceConsumesLiveProducer(t *testing.T) {
	blank, tooth, trim := neutralArcSeamSource(t)
	proof, ok, err := revolvecontact.CertifySource(t.Context(), bodyNode(blank), bodyNode(tooth))
	if err != nil || !ok {
		t.Fatalf("live source was not admitted: %v, %t", err, ok)
	}
	section := proof.Section.Spans[proof.Section.RootSpan]
	cells, ok, err := revolveloftfaces.RevolveLoftFacesFiniteSpanCells(t.Context(), trim, section)
	if err != nil || !ok {
		t.Fatalf("live source did not reach mapped finite-span consumer: %v, %t", err, ok)
	}
	if len(cells) == 0 {
		t.Fatal("mapped finite-span consumer returned no cells")
	}
	t.Run("exact rows of live mapped span", func(t *testing.T) {
		crossing := proof.Crossings[0]
		first, last := trim.Maps[0], trim.Maps[1]
		wantQ := spiralRootPolynomialRef(first, last, crossing.Span, proof.RootM, proof.RootB)
		gotQ := loftface.MeridianQRows(first, last, crossing.Span, proof.RootM, proof.RootB)
		for i := range gotQ {
			requireSameRows(t, wantQ[i], gotQ[i])
		}
		wantJ := spiralRootXOrderRef(first, last, crossing.Span, wantQ)
		gotJ := loftface.MeridianJRows(first, last, crossing.Span, gotQ)
		for i := range gotJ {
			requireSameRows(t, wantJ[i], gotJ[i])
		}
		for _, row := range gotQ {
			for _, parameter := range []*big.Rat{
				big.NewRat(0, 1), big.NewRat(1, 3), big.NewRat(2, 3), big.NewRat(1, 1),
			} {
				reference := spiralBernValueRef(row, parameter)
				require.Equal(t, reference.Sign(), polynomial.BernsteinSign(row, parameter))
			}
		}
	})
	t.Logf("root section %d mapped %d source cells", proof.Section.RootSpan, len(cells))
	radial := 0
	for segment, entity := range trim.Source.Source.Entities {
		if _, isArc := entity.(*sketch.Arc); !isArc {
			continue
		}
		for band := 0; band+1 < len(trim.Maps); band++ {
			if _, certified := revolveloftfaces.MappedArcRadialTimeBand(t.Context(), trim, segment, band, section); certified {
				radial++
			}
		}
	}
	if radial == 0 {
		t.Fatal("no live mapped arc reached the radial source consumer")
	}
	t.Logf("mapped %d whole-arc bands", radial)
	manifest, complete, manifestErr := revolvecontact.NonverticalManifest(t.Context(), proof)
	if manifestErr != nil || !complete || len(manifest) == 0 {
		t.Fatalf("live nonvertical source was not certified: %v, %t", manifestErr, complete)
	}
	var ref provenance.Ref
	var owner *Face
	for _, side := range manifest {
		if len(side.Cells) == 0 {
			t.Fatalf("nonvertical section %d has no retained source cells", side.SectionIndex)
		}
		for _, cell := range side.Cells {
			if cell.Mapped.Segment != proof.Crossings[0].Segment &&
				cell.Mapped.Segment != proof.Crossings[1].Segment && owner == nil {
				ref, owner = cell.Ref, faceHandle(cell.Face)
			}
		}
	}
	if owner == nil {
		t.Fatal("the real source has no retained noncrossing wall")
	}
	t.Run("missing noncrossing owner", func(t *testing.T) {
		original := owner.n.Origins
		defer func() { owner.n.Origins = original }()
		owner.n.Origins = slices.DeleteFunc(slices.Clone(original),
			func(got provenance.Ref) bool { return got == ref })
		_, certified, err := revolvecontact.NonverticalManifest(t.Context(), proof)
		if err != nil || certified {
			t.Fatalf("missing live origin was admitted: %v, %t", err, certified)
		}
	})
	t.Run("wrong noncrossing triangle owner", func(t *testing.T) {
		original := tooth.n.Payload
		defer func() { tooth.n.Payload = original }()
		payload := original.(facetedPayload)
		payload.FaceOf = slices.Clone(payload.FaceOf)
		other := -1
		for i, face := range tooth.Faces() {
			if face != owner {
				other = i
				break
			}
		}
		if other < 0 {
			t.Fatal("the real source has no second face")
		}
		changed := false
		for i, group := range payload.Src {
			if slices.Contains(payload.Groups[group].Origins, ref) {
				payload.FaceOf[i] = other
				changed = true
				break
			}
		}
		if !changed {
			t.Fatal("the retained wall has no source triangle")
		}
		tooth.n.Payload = payload
		_, certified, err := revolvecontact.NonverticalManifest(t.Context(), proof)
		if err != nil || certified {
			t.Fatalf("wrong live face was admitted: %v, %t", err, certified)
		}
	})
	t.Run("changed tooth transform", func(t *testing.T) {
		original := tooth.n.Payload
		defer func() { tooth.n.Payload = original }()
		payload := original.(facetedPayload)
		shift, err := r3.Translation(r3.Vec{X: 1})
		if err != nil {
			t.Fatal(err)
		}
		payload.Xform = shift
		tooth.n.Payload = payload
		if revolvecontact.TrimCurrent(bodyNode(tooth), proof.Trim) {
			t.Fatal("changed tooth transform retained a current source record")
		}
		_, certified, err := revolvecontact.NonverticalManifest(t.Context(), proof)
		if err != nil || certified {
			t.Fatalf("changed tooth transform was admitted: %v, %t", err, certified)
		}
	})
}

// The census installs one meridian memo; the manifest path installs none.
// Both must certify the same shell manifest, and a shared memo must return
// the same endpoint roots as a direct call.
func TestCurvedCensusMeridianMemoMatchesDirect(t *testing.T) {
	blank, trimmed, _ := neutralArcSeamSource(t)
	proof, ok, err := revolvecontact.CertifySource(t.Context(), bodyNode(blank), bodyNode(trimmed))
	require.NoError(t, err)
	require.True(t, ok)
	census, ok, err := revolvecontact.Census(t.Context(), proof)
	require.NoError(t, err)
	require.True(t, ok)
	manifest, ok, err := revolvecontact.AllSideManifest(t.Context(), proof)
	require.NoError(t, err)
	require.True(t, ok)
	require.True(t, revolvecontact.SameShellManifest(census.Manifest, manifest))

	trim := proof.Trim
	memo := loftface.NewMeridianMemo()
	compared := 0
	for _, section := range proof.Section.Spans {
		if section.Face == nil || section.Start.U == section.End.U {
			continue
		}
		for segment, source := range trim.Source.Profile.Outer.Segments {
			spans, ok := revolveloftfaces.RevolveLoftFacesStraightOrFitSpans(trim, segment, source)
			if !ok {
				continue
			}
			for _, span := range spans {
				for band := 0; band+1 < len(trim.Maps); band++ {
					for _, atEnd := range [...]bool{false, true} {
						want, wantOK := revolveloftfaces.RevolveLoftFacesEndpointFiniteRootsWith(nil,
							trim, section, span, band, atEnd)
						for range 2 {
							got, gotOK := revolveloftfaces.RevolveLoftFacesEndpointFiniteRootsWith(memo,
								trim, section, span, band, atEnd)
							require.Equal(t, wantOK, gotOK)
							require.Len(t, got, len(want))
							for i := range want {
								require.Equal(t, want[i].Direction, got[i].Direction)
								require.Equal(t, want[i].Finite, got[i].Finite)
								require.Zero(t, want[i].Time.Lo.Cmp(got[i].Time.Lo))
								require.Zero(t, want[i].Time.Hi.Cmp(got[i].Time.Hi))
								require.Zero(t, want[i].X.Lo.Cmp(got[i].X.Lo))
								require.Zero(t, want[i].X.Hi.Cmp(got[i].X.Hi))
							}
						}
						compared++
					}
				}
			}
		}
	}
	require.Positive(t, compared, "the fixture must have nonvertical fitted or straight spans")
}

func TestFiniteRevolveFitLineJointKeepsBothLiveSourceSides(t *testing.T) {
	_, _, trim := neutralArcSeamSource(t)
	vertex, ok := revolveloftfaces.RevolveLoftFacesFitLineVertexAt(trim, 0, 5, false)
	require.True(t, ok)
	require.Same(t, trim.Source.Source.Entities[0].(*sketch.FitSpline).Fit[0], vertex.Point)
	other, ok := revolveloftfaces.RevolveLoftFacesFitLineVertexAt(trim, 2, 3, true)
	require.True(t, ok)
	require.False(t, other.LineAtEnd)
	band := 3
	first := trim.Maps[band].At(vertex.Coordinate[0], vertex.Coordinate[1])
	last := trim.Maps[band+1].At(vertex.Coordinate[0], vertex.Coordinate[1])
	x0, _ := first[0].Float64()
	x1, _ := last[0].Float64()
	y0, _ := first[1].Float64()
	y1, _ := last[1].Float64()
	z0, _ := first[2].Float64()
	z1, _ := last[2].Float64()
	r0, r1 := math.Hypot(y0, z0), math.Hypot(y1, z1)
	require.NotEqual(t, r0, r1)
	section := revolveSectionSpan{Start: planePoint{U: math.Min(x0, x1) - 0.05, V: (r0 + r1) / 2},
		End: planePoint{U: math.Max(x0, x1) + 0.05, V: (r0 + r1) / 2}}
	joint, ok := revolveloftfaces.CertifyRevolveLoftFacesFitLineJoint(t.Context(), trim, section, 0, 5, band, false)
	require.True(t, ok)
	require.Len(t, joint.Roots, 1)
	require.True(t, joint.Roots[0].Finite)
	for _, part := range []struct {
		segment int
		span    freeform.BezierSpan
	}{
		{segment: 0, span: vertex.FitSpan},
		{segment: 5, span: vertex.LineSpan},
	} {
		cells, certified, err := revolveloftfaces.RevolveLoftFacesFitLinePartitionCells(t.Context(),
			trim, section, part.segment, 0, band, part.span, nil, false, nil,
			[]*revolveloftfaces.RevolveLoftFacesFitLineJoint{&joint})
		require.NoError(t, err)
		require.True(t, certified)
		require.NotEmpty(t, cells)
		contacts := 0
		for _, cell := range cells {
			if cell.Side == revolveloftfaces.RevolveLoftFacesJointContact {
				contacts++
				require.Equal(t, revolveloftfaces.RevolveLoftFacesFitLineContact, cell.JointKind)
				require.Same(t, vertex.Point, cell.JointPoint)
			}
		}
		require.Equal(t, 1, contacts)
	}
	joints, certified, err := revolveloftfaces.RevolveLoftFacesEnumerateNonverticalJoints(t.Context(), trim, section)
	require.NoError(t, err)
	require.True(t, certified)
	found := false
	for _, candidate := range joints.FitLines {
		if candidate.Band == band && candidate.Vertex.Point == vertex.Point &&
			candidate.Vertex.FitSegment == 0 && candidate.Vertex.LineSegment == 5 {
			found = true
		}
	}
	require.True(t, found)
	mapped, certified, err := revolveloftfaces.RevolveLoftFacesFiniteSpanCells(t.Context(), trim, section)
	require.NoError(t, err)
	require.True(t, certified)
	for _, segment := range []int{0, 5} {
		matched := false
		for _, cell := range mapped {
			if cell.Segment == segment && cell.Band == band &&
				cell.Side == revolveloftfaces.RevolveLoftFacesJointContact &&
				cell.JointKind == revolveloftfaces.RevolveLoftFacesFitLineContact &&
				cell.JointPoint == vertex.Point {
				matched = true
			}
		}
		require.True(t, matched)
	}
	t.Run("changed recorded line endpoint refuses", func(t *testing.T) {
		original := trim.Source.Profile.Outer.Segments[5]
		defer func() { trim.Source.Profile.Outer.Segments[5] = original }()
		changed := original.(lineSeg)
		changed.End.U += 0.001
		trim.Source.Profile.Outer.Segments[5] = changed
		_, valid := revolveloftfaces.RevolveLoftFacesFitLineVertexAt(trim, 0, 5, false)
		require.False(t, valid)
	})
}
