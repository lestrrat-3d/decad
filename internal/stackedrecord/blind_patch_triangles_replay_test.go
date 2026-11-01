package stackedrecord

import (
	"context"
	"errors"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/triangulation"
)

func TestReplayBlindPatchTrianglesParityAndLimits(t *testing.T) {
	point := func(u, v float64) sectionrecord.Point2 { return sectionrecord.Point2{U: u, V: v} }
	profiles := [][]sectionrecord.Point2{
		{point(0, 0), point(12, 0), point(12, 8), point(0, 8)},
		{point(2, 2), point(2, 3), point(3, 3), point(3, 2)},
		{point(5, 2), point(5, 3), point(6, 3), point(6, 2)},
		{point(8, 2), point(8, 3), point(9, 3), point(9, 2)},
	}
	rings := make([]tessellation.StackedRing[*topo.Face], len(profiles))
	loops := make([]tessellation.StackedPatchLoop, len(profiles))
	index := 0
	for i, profile := range profiles {
		rings[i].Samples = profile
		rings[i].Top = make([]int, len(profile))
		rings[i].Bottom = make([]int, len(profile))
		for j := range profile {
			rings[i].Top[j], rings[i].Bottom[j] = index, index
			index++
		}
		loops[i] = tessellation.StackedPatchLoop{Column: i, Top: true, Outer: i == 0, ColumnOuter: i == 0}
	}
	ordinary, err := tessellation.StackedPatchTriangles(t.Context(), rings, loops, false,
		tessellation.RequireLoopClearance, triangulation.Triangulate)
	if err != nil {
		t.Fatal(err)
	}
	run := func(ctx context.Context, budget *proofbound.ReplayBudget) (tessellation.StackedPatchMesh, error) {
		return ReplayBlindPatchTriangles(ctx, rings, loops, false, budget)
	}
	baseline := proofbound.NewReplayTrialBudget()
	got, err := run(t.Context(), baseline)
	if err != nil || len(got.Triangles) != len(ordinary.Triangles) || got.Sag != ordinary.Sag {
		t.Fatalf("triangles=%d want=%d sag=%g error=%v", len(got.Triangles), len(ordinary.Triangles), got.Sag, err)
	}
	for i := range got.Triangles {
		if got.Triangles[i] != ordinary.Triangles[i] {
			t.Fatalf("triangle %d differs", i)
		}
	}
	work, records := baseline.Usage()
	bytes := baseline.Bytes()
	for _, tc := range []struct {
		name              string
		work, recs, bytes uint64
		pass              bool
	}{
		{replayTestCaseExact, work, records, bytes, true},
		{replayTestCaseWorkShort, work - 1, records, bytes, false},
		{replayTestCaseRecordsShort, work, records - 1, bytes, false},
		{replayTestCaseBytesShort, work, records, bytes - 1, false},
	} {
		t.Run(tc.name, func(t *testing.T) {
			budget, err := proofbound.NewReplayBudgetLimits(tc.work, tc.recs, tc.bytes)
			if err != nil {
				t.Fatal(err)
			}
			out, err := run(t.Context(), budget)
			if tc.pass && (err != nil || len(out.Triangles) != len(ordinary.Triangles)) {
				t.Fatalf("exact triangles=%d error=%v", len(out.Triangles), err)
			}
			if !tc.pass && (!errors.Is(err, decaderr.ErrUnsupported) || len(out.Triangles) != 0) {
				t.Fatalf("short triangles=%d error=%v", len(out.Triangles), err)
			}
		})
	}
	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	if out, err := run(ctx, proofbound.NewReplayTrialBudget()); !errors.Is(err, context.Canceled) || len(out.Triangles) != 0 {
		t.Fatalf("cancel triangles=%d error=%v", len(out.Triangles), err)
	}
	lateBudget := proofbound.NewReplayTrialBudget()
	// The first 105 polls finish the bridge and ear preflight. Poll 108
	// refuses during the reached ear candidate/blocker scan.
	late := &cancelPatchAtPoll{Context: context.Background(), budget: lateBudget, stop: 108}
	if out, err := run(late, lateBudget); !errors.Is(err, context.Canceled) ||
		len(out.Triangles) != 0 || late.polls != late.stop {
		t.Fatalf("late cancel polls=%d triangles=%d error=%v", late.polls, len(out.Triangles), err)
	}
	t.Logf("late cancellation polls=%d work=%d", late.polls, late.workAtCancel)
	bad := append([]tessellation.StackedRing[*topo.Face](nil), rings...)
	bad[1].Top = bad[1].Top[:3]
	if out, err := ReplayBlindPatchTriangles(t.Context(), bad, loops, false, proofbound.NewReplayTrialBudget()); !errors.Is(err, decaderr.ErrUnsupported) || len(out.Triangles) != 0 {
		t.Fatalf("bad selected ring triangles=%d error=%v", len(out.Triangles), err)
	}
}

type cancelPatchAtPoll struct {
	context.Context //nolint:containedctx // The context is confined to one synchronous replay or test.
	budget          *proofbound.ReplayBudget
	stop, polls     int
	workAtCancel    uint64
}

func (c *cancelPatchAtPoll) Err() error {
	c.polls++
	if c.polls >= c.stop {
		c.workAtCancel, _ = c.budget.Usage()
		return context.Canceled
	}
	return nil
}
