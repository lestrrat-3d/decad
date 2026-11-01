package stackedrecord

import (
	"context"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/units"
)

func TestReplayBlindColumns(t *testing.T) {
	circle := func(x float64) sectionrecord.LoopRecord {
		return sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{sectionrecord.CircleSeg{Center: sectionrecord.Point2{U: x}, Radius: units.Millimeters(1), CCW: true, TEnd: 1}}}
	}
	outer, h0, h1 := circle(0), circle(3), circle(6)
	sp := Payload{Slabs: []Slab{{Regions: []momentinput.Profile{{Outer: outer, Holes: []sectionrecord.LoopRecord{h0}}}}, {Regions: []momentinput.Profile{{Outer: outer, Holes: []sectionrecord.LoopRecord{h0, h1}}}}}}
	want, wantSlabs, err := sp.Columns()
	if err != nil {
		t.Fatal(err)
	}
	run := func(ctx context.Context, m *proofbound.ReplayBudget) ([]WallColumn, [][]SlabLoop, bool, error) {
		return replayBlindColumns(ctx, sp, m)
	}
	baseline := proofbound.NewReplayTrialBudget()
	got, rows, ok, err := run(context.Background(), baseline)
	if err != nil || !ok || len(got) != len(want) || len(rows) != len(wantSlabs) {
		t.Fatalf("result ok=%t columns=%d rows=%d error=%v", ok, len(got), len(rows), err)
	}
	for i := range got {
		a, b := got[i].Column, want[i].Column
		if a.Start != b.Start || a.End != b.End || a.Region != b.Region || a.LoopIndex != b.LoopIndex || !equalLoop(a.Loop, b.Loop) {
			t.Fatalf("column %d differs", i)
		}
	}
	for k := range rows {
		if len(rows[k]) != len(wantSlabs[k]) {
			t.Fatalf("row %d length differs", k)
		}
		for i := range rows[k] {
			if rows[k][i] != wantSlabs[k][i] {
				t.Fatalf("row %d item %d differs", k, i)
			}
		}
	}
	w, r := baseline.Usage()
	b := baseline.Bytes()
	for _, tc := range []struct {
		name    string
		w, r, b uint64
		pass    bool
	}{{replayTestCaseExact, w, r, b, true}, {replayTestCaseWorkShort, w - 1, r, b, false}, {replayTestCaseRecordsShort, w, r - 1, b, false}, {replayTestCaseBytesShort, w, r, b - 1, false}} {
		t.Run(tc.name, func(t *testing.T) {
			m, err := proofbound.NewReplayBudgetLimits(tc.w, tc.r, tc.b)
			if err != nil {
				t.Fatal(err)
			}
			c, rows, ok, err := run(context.Background(), m)
			if tc.pass && (err != nil || !ok) {
				t.Fatalf("ok=%t error=%v", ok, err)
			}
			if !tc.pass && (err == nil || ok || c != nil || rows != nil) {
				t.Fatalf("rollback ok=%t error=%v", ok, err)
			}
		})
	}
	// A returned column owns its segment slice, even when the source changes.
	saved := sp.Slabs[0].Regions[0].Outer.Segments[0]
	sp.Slabs[0].Regions[0].Outer.Segments[0] = sectionrecord.CircleSeg{Radius: units.Millimeters(5), CCW: true, TEnd: 1}
	if got[0].Loop.Segments[0] != saved {
		t.Fatal("column aliases the source segment slice")
	}
	sp.Slabs[0].Regions[0].Outer.Segments[0] = saved
	bad := saved.(sectionrecord.CircleSeg)
	bad.Center.U = math.NaN()
	sp.Slabs[0].Regions[0].Outer.Segments[0] = bad
	if c, rows, ok, err := run(context.Background(), proofbound.NewReplayTrialBudget()); err != nil || ok || c != nil || rows != nil {
		t.Fatalf("nonfinite ok=%t error=%v", ok, err)
	}
	bad = saved.(sectionrecord.CircleSeg)
	bad.TEnd = math.Inf(1)
	sp.Slabs[0].Regions[0].Outer.Segments[0] = bad
	if c, rows, ok, err := run(context.Background(), proofbound.NewReplayTrialBudget()); err != nil || ok || c != nil || rows != nil {
		t.Fatalf("infinite parameter ok=%t error=%v", ok, err)
	}
	sp.Slabs[0].Regions[0].Outer.Segments[0] = saved
	sp.Slabs[0].Regions[0].Holes = append(sp.Slabs[0].Regions[0].Holes, h0)
	if c, rows, ok, err := run(context.Background(), proofbound.NewReplayTrialBudget()); err != nil || ok || c != nil || rows != nil {
		t.Fatalf("duplicate ok=%t error=%v", ok, err)
	}
	ctx, cancel := context.WithCancel(context.Background())
	cancel()
	if c, rows, ok, err := run(ctx, proofbound.NewReplayTrialBudget()); err == nil || ok || c != nil || rows != nil {
		t.Fatalf("cancel ok=%t error=%v", ok, err)
	}
}
