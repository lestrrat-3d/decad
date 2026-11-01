package topo

import (
	"context"
	"errors"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
)

type faceBoundTestBudget struct {
	records, bytes, work uint64
	refuse               bool
}

func (b *faceBoundTestBudget) ReserveRecords(_ context.Context, n uint64) error {
	if b.refuse {
		return decaderr.ErrUnsupported
	}
	b.records += n
	return nil
}
func (b *faceBoundTestBudget) ReserveArray(_ context.Context, n, s uint64) error {
	if b.refuse {
		return decaderr.ErrUnsupported
	}
	b.bytes += n * s
	return nil
}
func (b *faceBoundTestBudget) SpendWork(_ context.Context, n uint64) error {
	if b.refuse {
		return decaderr.ErrUnsupported
	}
	b.work += n
	return nil
}

func TestFaceBoundTableFixedCapacityAndClone(t *testing.T) {
	ctx := t.Context()
	budget := new(faceBoundTestBudget)
	table, err := NewFaceBoundTable(ctx, 3, budget)
	if err != nil {
		t.Fatal(err)
	}
	faces := [4]*Face{{}, {}, {}, {}}
	for i := range 3 {
		if err := table.Set(ctx, faces[i], float64(i), budget); err != nil {
			t.Fatal(err)
		}
	}
	if err := table.Set(ctx, faces[3], 4, budget); !errors.Is(err, decaderr.ErrUnsupported) {
		t.Fatalf("capacity: %v", err)
	}
	if table.Len() != 3 {
		t.Fatalf("count=%d", table.Len())
	}
	if err := table.Set(ctx, faces[1], 9, budget); err != nil {
		t.Fatal(err)
	}
	clone, err := table.Clone(ctx, budget)
	if err != nil {
		t.Fatal(err)
	}
	for i := range 3 {
		got, ok, err := clone.Lookup(ctx, faces[i], budget)
		want := float64(i)
		if i == 1 {
			want = 9
		}
		if err != nil || !ok || got != want {
			t.Fatalf("face %d: got=%v present=%v err=%v", i, got, ok, err)
		}
	}
	seen := 0
	if err := clone.Each(ctx, budget, func(_ *Face, _ float64) error { seen++; return nil }); err != nil || seen != 3 {
		t.Fatalf("iteration count=%d err=%v", seen, err)
	}
	budget.refuse = true
	if _, _, err := clone.Lookup(ctx, faces[0], budget); !errors.Is(err, decaderr.ErrUnsupported) {
		t.Fatalf("lookup refusal: %v", err)
	}
	if _, err := NewFaceBoundTable(ctx, 1, budget); !errors.Is(err, decaderr.ErrUnsupported) {
		t.Fatalf("allocation refusal: %v", err)
	}
}
