package decad

import (
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/revolveloftfaces"
)

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
