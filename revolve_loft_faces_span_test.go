package decad

import (
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/loftface"
	"github.com/lestrrat-3d/decad/internal/polynomial"
	"github.com/lestrrat-3d/decad/internal/revolveloftfaces"
)

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
			t.Fatalf("root bracket does not change side: %+v", root)
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
