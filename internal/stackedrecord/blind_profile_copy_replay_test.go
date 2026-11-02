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

func TestReplayCopyBlindProfile(t *testing.T) {
	circle := func(x float64) sectionrecord.LoopRecord {
		return sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{sectionrecord.CircleSeg{Center: sectionrecord.Point2{U: x}, Radius: units.Millimeters(1), CCW: true, TEnd: 1}}}
	}
	source := momentinput.Profile{Outer: circle(0), Holes: []sectionrecord.LoopRecord{circle(3)}}
	run := func(ctx context.Context, b *proofbound.ReplayBudget) (momentinput.Profile, bool, error) {
		return replayCopyBlindProfile(ctx, source, b)
	}
	base := proofbound.NewReplayTrialBudget()
	out, ok, err := run(context.Background(), base)
	if err != nil || !ok || len(out.Holes) != 1 {
		t.Fatalf("ok=%t error=%v", ok, err)
	}
	w, r := base.Usage()
	n := base.Bytes()
	for _, tc := range []struct {
		name    string
		w, r, n uint64
		pass    bool
	}{{replayTestCaseExact, w, r, n, true}, {replayTestCaseWorkShort, w - 1, r, n, false}, {replayTestCaseRecordsShort, w, r - 1, n, false}, {replayTestCaseBytesShort, w, r, n - 1, false}} {
		t.Run(tc.name, func(t *testing.T) {
			m, e := proofbound.NewReplayBudgetLimits(tc.w, tc.r, tc.n)
			if e != nil {
				t.Fatal(e)
			}
			out, ok, e := run(context.Background(), m)
			if tc.pass && (e != nil || !ok) {
				t.Fatalf("ok=%t error=%v", ok, e)
			}
			if !tc.pass && (e == nil || ok || out.Outer.Segments != nil || out.Holes != nil) {
				t.Fatalf("rollback ok=%t error=%v", ok, e)
			}
		})
	}
	old := out.Holes[0].Segments[0]
	source.Holes[0].Segments[0] = sectionrecord.CircleSeg{Radius: units.Millimeters(9), CCW: true, TEnd: 1}
	if out.Holes[0].Segments[0] != old {
		t.Fatal("copy retained source segment slice")
	}
	source.Holes[0].Segments[0] = old
	oldOuter := source.Outer.Segments[0]
	bad := oldOuter.(sectionrecord.CircleSeg)
	bad.Center.U = math.Inf(1)
	source.Outer.Segments[0] = bad
	if out, ok, err := run(context.Background(), proofbound.NewReplayTrialBudget()); err != nil || ok || out.Outer.Segments != nil {
		t.Fatalf("nonfinite ok=%t error=%v", ok, err)
	}
	ctx, cancel := context.WithCancel(context.Background())
	cancel()
	if out, ok, err := run(ctx, proofbound.NewReplayTrialBudget()); err == nil || ok || out.Outer.Segments != nil {
		t.Fatalf("cancel ok=%t error=%v", ok, err)
	}
}
