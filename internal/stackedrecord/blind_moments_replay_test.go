package stackedrecord

import (
	"context"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/units"
)

func TestReplayBlindRegionMomentsAndPatchArea(t *testing.T) {
	points := []sectionrecord.Point2{{}, {U: 10}, {U: 10, V: 10}, {V: 10}}
	outer := sectionrecord.LoopRecord{Segments: make([]sectionrecord.CurveSegment, len(points))}
	for i := range points {
		outer.Segments[i] = sectionrecord.LineSeg{Start: points[i], End: points[(i+1)%len(points)], TEnd: 1}
	}
	source := momentinput.Profile{Outer: outer}
	computed, err := source.EvaluatorIntegralsContext(t.Context(), freeform.MomentFirstOrder,
		freeform.NewFreeformWork())
	if err != nil {
		t.Fatal(err)
	}
	walks, err := momentinput.ResolveProfileWalks(source, freeform.NewFreeformWork())
	if err != nil {
		t.Fatal(err)
	}
	target := prismrecord.Payload{Profile: source, Walks: walks.WithIntegrals(computed, freeform.MomentFirstOrder)}
	circle := sectionrecord.CircleSeg{Center: sectionrecord.Point2{U: 2, V: 3},
		Radius: units.Millimeters(0.5), TStart: 1, TEnd: 0}
	cut := momentinput.Profile{Outer: outer, Holes: []sectionrecord.LoopRecord{{Segments: []sectionrecord.CurveSegment{circle}}}}
	baseline := proofbound.NewReplayTrialBudget()
	got, ok, err := replayBlindRegionMoments(t.Context(), cut, target, baseline)
	if err != nil || !ok {
		t.Fatalf("region ok=%t err=%v", ok, err)
	}
	want, err := cut.EvaluatorIntegralsContext(t.Context(), freeform.MomentFirstOrder,
		freeform.NewFreeformWork())
	if err != nil {
		t.Fatal(err)
	}
	for _, pair := range [][4]float64{{got.Area, got.AreaBound, want.Area, want.AreaBound},
		{got.Mu, got.MuBound, want.Mu, want.MuBound}, {got.Mv, got.MvBound, want.Mv, want.MvBound}} {
		if math.Abs(pair[0]-pair[2]) > proofbound.AbsSumUpper(pair[1], pair[3]) {
			t.Fatalf("replay and ordinary moment enclosures do not intersect: %.17g, %.17g", pair[0], pair[2])
		}
	}
	work, records := baseline.Usage()
	bytes := baseline.Bytes()
	if work == 0 || records == 0 || bytes == 0 {
		t.Fatalf("missing charges work=%d records=%d bytes=%d", work, records, bytes)
	}
	for _, tc := range []struct {
		name    string
		w, r, b uint64
		pass    bool
	}{
		{replayTestCaseExact, work, records, bytes, true},
		{replayTestCaseWorkShort, work - 1, records, bytes, false},
		{replayTestCaseRecordsShort, work, records - 1, bytes, false},
		{replayTestCaseBytesShort, work, records, bytes - 1, false},
	} {
		t.Run(tc.name, func(t *testing.T) {
			meter, e := proofbound.NewReplayBudgetLimits(tc.w, tc.r, tc.b)
			if e != nil {
				t.Fatal(e)
			}
			_, admitted, e := replayBlindRegionMoments(t.Context(), cut, target, meter)
			if tc.pass && (e != nil || !admitted) || !tc.pass && (e == nil || admitted) {
				t.Fatalf("admitted=%t err=%v", admitted, e)
			}
		})
	}
	changed := cut
	changed.Holes = []sectionrecord.LoopRecord{{Segments: []sectionrecord.CurveSegment{circle}}}
	bad := circle
	bad.TEnd = 0.5
	changed.Holes[0].Segments[0] = bad
	if _, ok, err := replayBlindRegionMoments(t.Context(), changed, target,
		proofbound.NewReplayTrialBudget()); err != nil || ok {
		t.Fatalf("trimmed circle admitted=%t err=%v", ok, err)
	}
	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	if _, ok, err := replayBlindRegionMoments(ctx, cut, target,
		proofbound.NewReplayTrialBudget()); err == nil || ok {
		t.Fatalf("cancellation admitted=%t err=%v", ok, err)
	}

	positive := circle
	positive.CCW, positive.TStart, positive.TEnd = true, 0, 1
	patch := Patch{Record: momentinput.Profile{Outer: sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{positive}}},
		Loops: []PatchLoop{{Column: 0, Outer: true}}}
	columns := []WallColumn{{Loop: patch.Record.Outer, Perimeter: proofbound.MeasuredScalar(math.Pi, 0)}}
	patchMeter := proofbound.NewReplayTrialBudget()
	area, err := patchAreaReplay(t.Context(), 1e-12, []float64{0}, columns, patch, patchMeter)
	if err != nil {
		t.Fatal(err)
	}
	if math.Abs(area.Value-math.Pi*0.25) > area.Bound {
		t.Fatalf("patch area %.17g bound %.17g", area.Value, area.Bound)
	}
	if patchMeter.Bytes() == 0 {
		t.Fatal("patch area did not charge exact arithmetic")
	}
	pw, pr := patchMeter.Usage()
	pb := patchMeter.Bytes()
	for _, tc := range []struct {
		name    string
		w, r, b uint64
		pass    bool
	}{
		{"patch exact", pw, pr, pb, true},
		{"patch work short", pw - 1, pr, pb, false},
		{"patch records short", pw, pr - 1, pb, false},
		{"patch bytes short", pw, pr, pb - 1, false},
	} {
		t.Run(tc.name, func(t *testing.T) {
			meter, e := proofbound.NewReplayBudgetLimits(tc.w, tc.r, tc.b)
			if e != nil {
				t.Fatal(e)
			}
			_, e = patchAreaReplay(t.Context(), 1e-12, []float64{0}, columns, patch, meter)
			if tc.pass && e != nil || !tc.pass && e == nil {
				t.Fatalf("patch err=%v", e)
			}
		})
	}
}
