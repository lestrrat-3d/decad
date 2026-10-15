package decad

import (
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/freeform"
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
	finite := revolveSectionSpan{start: planePoint{U: 0.45, V: 1.5},
		end: planePoint{U: 0.55, V: 1.5}}
	roots, ok := revolveLoftFacesEndpointFiniteRoots(trim, finite, span, 0, true)
	if !ok || len(roots) != 1 || !roots[0].Finite ||
		roots[0].Time.Lo.Cmp(big.NewRat(1, 2)) != 0 {
		t.Fatalf("expected one exact finite knot time, got %+v, %t", roots, ok)
	}
	cell, ok := revolveLoftFacesKnotRootCell(trim, finite, span, 0, true, roots[0])
	if !ok || !cell.finite || cell.outsideSide != -1 || cell.sDerivative != 1 ||
		cell.x.Lo.Cmp(big.NewRat(1, 2)) != 0 || cell.x.Hi.Cmp(big.NewRat(1, 2)) != 0 {
		t.Fatalf("exact contact must have one bounded source sliver: %+v, %t", cell, ok)
	}
	outside := revolveSectionSpan{start: planePoint{U: 0.7, V: 1.5},
		end: planePoint{U: 0.8, V: 1.5}}
	roots, ok = revolveLoftFacesEndpointFiniteRoots(trim, outside, span, 0, true)
	if !ok || len(roots) != 1 || roots[0].Finite {
		t.Fatalf("expected one exact axial miss, got %+v, %t", roots, ok)
	}
	cell, ok = revolveLoftFacesKnotRootCell(trim, outside, span, 0, true, roots[0])
	if !ok || cell.finite || cell.x.Hi.Cmp(lfF(outside.start.U)) >= 0 {
		t.Fatalf("nonfinite contact must be clipped by exact X: %+v, %t", cell, ok)
	}
	roots[0].Direction = -roots[0].Direction
	if _, ok := revolveLoftFacesKnotRootCell(trim, outside, span, 0, true, roots[0]); ok {
		t.Fatal("a root with the wrong source-side direction must refuse")
	}
}
