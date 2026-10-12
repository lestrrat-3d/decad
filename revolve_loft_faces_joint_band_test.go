package decad

import (
	"math/big"
	"testing"
)

func TestMappedRadialJointTimePartition(t *testing.T) {
	q := [3]*big.Rat{big.NewRat(-1, 1), big.NewRat(2, 1), big.NewRat(-1, 1)}
	rows := [3][]*big.Rat{{q[0]}, {q[1]}, {q[2]}}
	roots, ok := revolveLoftFacesEndpointTimeRoots(rows, true)
	if !ok {
		t.Fatal("exact quadratic roots were not isolated")
	}
	pieces, ok := revolveLoftFacesPartitionTime(q, roots)
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
		if pieces[i].side != want.side || pieces[i].rootIndex != want.root ||
			pieces[i].loOpen != want.loOpen || pieces[i].hiOpen != want.hiOpen ||
			pieces[i].lo.Cmp(pieces[i].hi) > 0 {
			t.Fatalf("piece %d lost its exact side or root: %+v", i, pieces[i])
		}
		if i != 0 && pieces[i-1].hi.Cmp(pieces[i].lo) != 0 {
			t.Fatalf("time partition has a gap at piece %d", i)
		}
	}
	roots[0].direction = -1
	if _, ok := revolveLoftFacesPartitionTime(q, roots); ok {
		t.Fatal("inconsistent root direction must refuse")
	}
}

func TestMappedRadialJointExactRootBoundary(t *testing.T) {
	q := [3]*big.Rat{big.NewRat(-1, 1), big.NewRat(0, 1), big.NewRat(1, 1)}
	rows := [3][]*big.Rat{{q[0]}, {q[1]}, {q[2]}}
	roots, ok := revolveLoftFacesEndpointTimeRoots(rows, false)
	if !ok || len(roots) != 1 || roots[0].time.lo.Cmp(big.NewRat(1, 2)) != 0 ||
		roots[0].time.hi.Cmp(big.NewRat(1, 2)) != 0 {
		t.Fatalf("expected an exact dyadic root, got %+v, %t", roots, ok)
	}
	pieces, ok := revolveLoftFacesPartitionTime(q, roots)
	if !ok || len(pieces) != 3 || !pieces[0].hiOpen || !pieces[2].loOpen ||
		pieces[1].loOpen || pieces[1].hiOpen ||
		pieces[0].hi.Cmp(pieces[1].lo) != 0 || pieces[1].hi.Cmp(pieces[2].lo) != 0 {
		t.Fatalf("exact contact must belong only to the root piece: %+v, %t", pieces, ok)
	}
}
