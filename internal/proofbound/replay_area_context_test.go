package proofbound

import (
	"errors"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/r3"
)

func TestReplayAreaContextParityAndLimits(t *testing.T) {
	ctx := t.Context()
	verts := []r3.Vec{{}, {X: 3}, {Y: 4}, {X: 3, Y: 4}}
	tris := [][3]int{{0, 1, 2}, {1, 3, 2}}
	want, allows, err := PerturbedAreaUpperAllowsContext(ctx, verts, tris, .01)
	if err != nil {
		t.Fatal(err)
	}
	run := func(m *ReplayBudget) (float64, []float64, error) {
		return PerturbedAreaUpperAllowsReplay(ctx, m.WorkBudget(ctx), verts, tris, .01)
	}
	m := NewBrakeReplayBudget()
	got, gotAllows, err := run(m)
	if err != nil || math.Float64bits(got) != math.Float64bits(want) || len(gotAllows) != len(allows) {
		t.Fatalf("area parity: got=%v want=%v error=%v", got, want, err)
	}
	for i := range allows {
		if math.Float64bits(gotAllows[i]) != math.Float64bits(allows[i]) {
			t.Fatalf("facet %d allowance changed", i)
		}
	}
	work, records := m.Usage()
	bytes := m.Bytes()
	for _, short := range []string{replayBudgetTestExact, "work", "records", "bytes"} {
		t.Run(short, func(t *testing.T) {
			m := NewBrakeReplayBudget()
			w, r, z := BrakeReplayWork-work, BrakeReplayRecords-records, BrakeReplayBytes-bytes
			switch short {
			case "work":
				w++
			case "records":
				r++
			case "bytes":
				z++
			}
			if err := m.SpendWork(ctx, w); err != nil {
				t.Fatal(err)
			}
			if err := m.ReserveRecords(ctx, r); err != nil {
				t.Fatal(err)
			}
			if err := m.ReserveBytes(ctx, z); err != nil {
				t.Fatal(err)
			}
			_, _, err := run(m)
			if short == replayBudgetTestExact && err != nil {
				t.Fatalf("exact: %v", err)
			}
			if short != replayBudgetTestExact && !errors.Is(err, decaderr.ErrUnsupported) {
				t.Fatalf("one-short %s: %v", short, err)
			}
		})
	}
}
