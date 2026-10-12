package decad

import (
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/freeform"
)

func TestMappedEndpointTimeRoots(t *testing.T) {
	two := []*big.Rat{big.NewRat(-1, 1), big.NewRat(2, 1), big.NewRat(-1, 1)}
	q := [3][]*big.Rat{{two[0]}, {two[1]}, {two[2]}}
	roots, ok := revolveLoftFacesEndpointTimeRoots(q, true)
	if !ok || len(roots) != 2 || roots[0].direction != 1 || roots[1].direction != -1 {
		t.Fatalf("expected two ordered transverse roots, got %+v, %t", roots, ok)
	}
	if roots[0].time.lo.Cmp(big.NewRat(1, 8)) <= 0 ||
		roots[0].time.hi.Cmp(big.NewRat(1, 4)) >= 0 ||
		roots[1].time.lo.Cmp(big.NewRat(3, 4)) <= 0 ||
		roots[1].time.hi.Cmp(big.NewRat(7, 8)) >= 0 {
		t.Fatalf("roots escaped certified separate intervals: %+v", roots)
	}
	for _, root := range roots {
		if spiralBernValue(two, root.time.lo).Sign()*spiralBernValue(two, root.time.hi).Sign() >= 0 {
			t.Fatalf("root bracket does not change side: %+v", root)
		}
	}

	q = [3][]*big.Rat{{big.NewRat(-1, 1)}, {big.NewRat(0, 1)}, {big.NewRat(1, 1)}}
	roots, ok = revolveLoftFacesEndpointTimeRoots(q, false)
	if !ok || len(roots) != 1 || roots[0].direction != 1 {
		t.Fatalf("expected one transverse root, got %+v, %t", roots, ok)
	}
	q = [3][]*big.Rat{{big.NewRat(1, 1)}, {big.NewRat(1, 1)}, {big.NewRat(1, 1)}}
	roots, ok = revolveLoftFacesEndpointTimeRoots(q, false)
	if !ok || len(roots) != 0 {
		t.Fatalf("expected certified strict side, got %+v, %t", roots, ok)
	}
	q = [3][]*big.Rat{{big.NewRat(0, 1)}, {big.NewRat(1, 1)}, {big.NewRat(-1, 1)}}
	if _, ok = revolveLoftFacesEndpointTimeRoots(q, false); ok {
		t.Fatal("boundary contact must refuse")
	}
}

func TestMappedEndpointFiniteSection(t *testing.T) {
	section := revolveSectionSpan{start: Point2{U: 5.7, V: 2.28},
		end: Point2{U: 5.8, V: 3.3}}
	x0, x1 := big.NewRat(5, 1), big.NewRat(6, 1)
	outside := revolveLoftFacesEndpointRoot{time: loftFaceInterval{
		lo: big.NewRat(1, 8), hi: big.NewRat(1, 8)}}
	outside, ok := revolveLoftFacesEndpointAxialClass(outside, x0, x1, section)
	if !ok || outside.finite || outside.x.hi.Cmp(big.NewRat(41, 8)) != 0 {
		t.Fatalf("expected a certified axial miss, got %+v, %t", outside, ok)
	}
	inside := revolveLoftFacesEndpointRoot{time: loftFaceInterval{
		lo: big.NewRat(3, 4), hi: big.NewRat(3, 4)}}
	inside, ok = revolveLoftFacesEndpointAxialClass(inside, x0, x1, section)
	if !ok || !inside.finite || inside.x.lo.Cmp(big.NewRat(23, 4)) != 0 {
		t.Fatalf("expected a certified finite crossing, got %+v, %t", inside, ok)
	}
	boundaryTime := lfSub(lfF(section.start.U), x0)
	boundary := revolveLoftFacesEndpointRoot{time: loftFaceInterval{
		lo: boundaryTime, hi: boundaryTime}}
	if _, ok := revolveLoftFacesEndpointAxialClass(boundary, x0, x1, section); ok {
		t.Fatal("a contact on the section end must refuse")
	}
}

func TestMappedSpanAxialHull(t *testing.T) {
	makeMap := func(offset int64) loftFaceMap {
		zero, one := big.NewRat(0, 1), big.NewRat(1, 1)
		return loftFaceMap{
			constant: loftFaceRatVec{zero, zero, zero},
			q0:       loftFaceRatVec{big.NewRat(offset, 1), zero, zero},
			qu:       loftFaceRatVec{one, zero, zero},
			qv:       loftFaceRatVec{zero, zero, zero},
			g0:       one, gu: zero, gv: zero,
		}
	}
	span := freeform.BezierSpan{
		{U: big.NewRat(0, 1), V: big.NewRat(0, 1)},
		{U: big.NewRat(1, 3), V: big.NewRat(0, 1)},
		{U: big.NewRat(2, 3), V: big.NewRat(0, 1)},
		{U: big.NewRat(1, 1), V: big.NewRat(0, 1)},
	}
	first, last := makeMap(5), makeMap(6)
	x, ok := revolveLoftFacesSpanXRange(first, last, span,
		big.NewRat(1, 2), big.NewRat(1, 1), big.NewRat(1, 8), big.NewRat(1, 4))
	if !ok || x.lo.Cmp(big.NewRat(45, 8)) != 0 || x.hi.Cmp(big.NewRat(25, 4)) != 0 {
		t.Fatalf("expected exact affine mapped X hull, got %+v, %t", x, ok)
	}
	section := revolveSectionSpan{start: Point2{U: 6.5, V: 1},
		end: Point2{U: 7, V: 2}}
	if !revolveLoftFacesSpanAxialMiss(first, last, span, section,
		big.NewRat(1, 2), big.NewRat(1, 1), big.NewRat(1, 8), big.NewRat(1, 4)) {
		t.Fatal("mapped span should miss the finite section")
	}
	section.start.U = 6
	if revolveLoftFacesSpanAxialMiss(first, last, span, section,
		big.NewRat(1, 2), big.NewRat(1, 1), big.NewRat(1, 8), big.NewRat(1, 4)) {
		t.Fatal("overlapping mapped span cannot be clipped")
	}
}
