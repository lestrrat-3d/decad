package stackedrecord

import (
	"context"
	"sync/atomic"
	"testing"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismcells"
	"github.com/lestrrat-3d/decad/internal/prismplacement"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

type bindingPollContext struct {
	context.Context //nolint:containedctx // The context is confined to one synchronous replay or test.
	polls           atomic.Int64
	limit           int64
	cancel          context.CancelFunc
}

func (c *bindingPollContext) Err() error {
	count := c.polls.Add(1)
	if c.limit > 0 && count >= c.limit && c.cancel != nil {
		c.cancel()
	}
	return c.Context.Err()
}

// This small synthetic test checks certificate binding only. Its scene setup
// uses the ordinary arranger and is not evidence of fully metered admission.
func TestPrepareBlindBuildReplaySceneBinding(t *testing.T) {
	frame, err := r3.NewFrame(r3.Vec{}, r3.Vec{X: 1}, r3.Vec{Y: 1})
	if err != nil {
		t.Fatal(err)
	}
	p := func(u, v float64) sectionrecord.Point2 { return sectionrecord.Point2{U: u, V: v} }
	corners := []sectionrecord.Point2{p(-10, -10), p(10, -10), p(10, 10), p(-10, 10)}
	host := prismrecord.Payload{Frame: frame, Xform: r3.Identity(), Z1: 10}
	for i := range corners {
		host.Profile.Outer.Segments = append(host.Profile.Outer.Segments, sectionrecord.LineSeg{Start: corners[i], End: corners[(i+1)%len(corners)], TEnd: 1})
	}
	host.Walks, err = momentinput.ResolveProfileWalks(host.Profile, freeform.NewFreeformWork())
	if err != nil {
		t.Fatal(err)
	}
	circle := func(x float64) momentinput.Profile {
		return momentinput.Profile{Outer: sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{sectionrecord.CircleSeg{Center: p(x, 0), Radius: units.Millimeters(1), CCW: true, TEnd: 1}}}}
	}
	tool := Payload{Frame: frame, Xform: r3.Identity(), Slabs: []Slab{{Z0: 6, Z1: 10, Regions: []momentinput.Profile{circle(-3), circle(3)}}}}
	mutation := 0
	run := func(ctx context.Context, m *proofbound.ReplayBudget) (*replayBlindBuildPlan, bool, error) {
		slab := tool.Slabs[0]
		place := prismplacement.Operand{Frame: tool.Frame, Xform: tool.Xform, Z0: slab.Z0, Z1: slab.Z1}
		cert, ok, err := prismcells.ReplayBlindGroupScene(ctx, host.Profile, slab.Regions, host.PlacementOperand(), place, host.SectionDelta, tool.SectionDelta, m)
		if err != nil || !ok {
			return nil, false, err
		}
		section, ok, err := prismcells.ReplayBlindGroupSection(ctx, cert, m)
		if err != nil || !ok {
			return nil, false, err
		}
		payload, _, ok, err := ReplayBlindGroupPayload(ctx, host, tool, cert, m)
		if err != nil || !ok {
			return nil, false, err
		}
		hostCopy := host
		if mutation == 1 {
			section.HoleRegions[0] = 99
		}
		if mutation == 2 {
			hostCopy.Profile.Outer.Segments = append([]sectionrecord.CurveSegment(nil), host.Profile.Outer.Segments...)
			line := hostCopy.Profile.Outer.Segments[0].(sectionrecord.LineSeg)
			line.Start.U++
			hostCopy.Profile.Outer.Segments[0] = line
		}
		return prepareBlindBuildReplay(ctx, payload, hostCopy, tool, section, 6, true, cert, m)
	}
	baseline := proofbound.NewReplayTrialBudget()
	plan, ok, err := run(context.Background(), baseline)
	if err != nil || !ok || plan == nil || len(plan.patches) != 2 {
		t.Fatalf("plan ok=%t error=%v", ok, err)
	}
	w, r := baseline.Usage()
	bytes := baseline.Bytes()
	for _, tc := range []struct {
		name    string
		w, r, b uint64
		pass    bool
	}{{replayTestCaseExact, w, r, bytes, true}, {replayTestCaseWorkShort, w - 1, r, bytes, false}, {replayTestCaseRecordsShort, w, r - 1, bytes, false}, {replayTestCaseBytesShort, w, r, bytes - 1, false}} {
		t.Run(tc.name, func(t *testing.T) {
			m, e := proofbound.NewReplayBudgetLimits(tc.w, tc.r, tc.b)
			if e != nil {
				t.Fatal(e)
			}
			p, ok, e := run(context.Background(), m)
			if tc.pass && (e != nil || !ok || p == nil) {
				t.Fatalf("ok=%t error=%v", ok, e)
			}
			if !tc.pass && (e == nil || ok || p != nil) {
				t.Fatalf("rollback ok=%t error=%v", ok, e)
			}
		})
	}
	for _, mode := range []int{1, 2} {
		mutation = mode
		if p, ok, err := run(context.Background(), proofbound.NewReplayTrialBudget()); err != nil || ok || p != nil {
			t.Fatalf("mutation %d ok=%t error=%v", mode, ok, err)
		}
	}
	mutation = 0
	measured := &bindingPollContext{Context: context.Background()}
	if p, ok, err := run(measured, proofbound.NewReplayTrialBudget()); err != nil || !ok || p == nil {
		t.Fatalf("poll baseline ok=%t error=%v", ok, err)
	}
	lateBase, lateCancel := context.WithCancel(context.Background())
	defer lateCancel()
	late := &bindingPollContext{Context: lateBase, cancel: lateCancel, limit: measured.polls.Load() * 3 / 4}
	if p, ok, err := run(late, proofbound.NewReplayTrialBudget()); err == nil || ok || p != nil {
		t.Fatalf("late cancellation ok=%t error=%v", ok, err)
	}
	ctx, cancel := context.WithCancel(context.Background())
	cancel()
	if p, ok, err := run(ctx, proofbound.NewReplayTrialBudget()); err == nil || ok || p != nil {
		t.Fatalf("cancel ok=%t error=%v", ok, err)
	}
}
