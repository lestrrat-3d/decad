package diameter

import (
	"errors"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

func TestReplayPointsContextParityAndLimits(t *testing.T) {
	ctx := t.Context()
	points := []r3.Vec{{}, {X: 3}, {Y: 4}, {X: 3, Y: 4, Z: 2}}
	want, ok, err := PointsContext(ctx, points)
	if err != nil || !ok {
		t.Fatalf("ordinary diameter: %v", err)
	}
	run := func(m *proofbound.ReplayBudget) (float64, bool, error) {
		return PointsReplay(ctx, m.WorkBudget(ctx), points)
	}
	m := proofbound.NewBrakeReplayBudget()
	got, ok, err := run(m)
	if err != nil || !ok || math.Float64bits(got) != math.Float64bits(want) {
		t.Fatalf("diameter parity: got=%v want=%v error=%v", got, want, err)
	}
	work, records := m.Usage()
	bytes := m.Bytes()
	for _, short := range []string{"exact", "work", "records", "bytes"} {
		t.Run(short, func(t *testing.T) {
			m := proofbound.NewBrakeReplayBudget()
			w, r, z := proofbound.BrakeReplayWork-work, proofbound.BrakeReplayRecords-records, proofbound.BrakeReplayBytes-bytes
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
			if short == "exact" && err != nil {
				t.Fatalf("exact: %v", err)
			}
			if short != "exact" && !errors.Is(err, decaderr.ErrUnsupported) {
				t.Fatalf("one-short %s: %v", short, err)
			}
		})
	}
}
