package decad

import (
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/loftface"
)

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
