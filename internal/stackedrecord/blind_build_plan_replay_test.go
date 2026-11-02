package stackedrecord

import (
	"context"
	"testing"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismcells"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/units"
)

const replayTestCaseBytesShort = "bytes short"

func TestPrepareBlindBuildReplay(t *testing.T) {
	circle := func(x float64, ccw bool) sectionrecord.LoopRecord {
		c := sectionrecord.CircleSeg{Center: sectionrecord.Point2{U: x}, Radius: units.Millimeters(1), CCW: ccw, TEnd: 1}
		if !ccw {
			c.TStart, c.TEnd = 1, 0
		}
		return sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{c}}
	}
	target := prismrecord.Payload{Z1: 10, Profile: momentinput.Profile{Outer: circle(0, true), Holes: []sectionrecord.LoopRecord{circle(1, false)}}}
	var err error
	target.Walks, err = momentinput.ResolveProfileWalks(target.Profile, freeform.NewFreeformWork())
	if err != nil {
		t.Fatal(err)
	}
	section := prismcells.ReplayBlindSection{Profile: momentinput.Profile{Outer: target.Profile.Outer, Holes: []sectionrecord.LoopRecord{circle(1, false), circle(3, false), circle(6, false)}}, HoleRegions: []int{0, 1}, SectionDelta: 1e-12}
	sp := Payload{SectionDelta: section.SectionDelta, Slabs: []Slab{{Z1: 6, Regions: []momentinput.Profile{target.Profile}}, {Z0: 6, Z1: 10, Regions: []momentinput.Profile{section.Profile}}}, Interfaces: []Interface{{LowerExposed: []momentinput.Profile{{Outer: circle(3, true)}, {Outer: circle(6, true)}}}}}
	run := func(ctx context.Context, b *proofbound.ReplayBudget) (*replayBlindBuildPlan, bool, error) {
		return prepareBlindBuildPlanParts(ctx, sp, target, section, 6, true, b)
	}
	m := proofbound.NewReplayTrialBudget()
	plan, ok, err := run(context.Background(), m)
	if err != nil || !ok || plan == nil || len(plan.columns) != 4 || len(plan.rows) != 2 || len(plan.patches) != 2 {
		t.Fatalf("ok=%t error=%v", ok, err)
	}
	w, r := m.Usage()
	bytes := m.Bytes()
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
	old := plan.patches[0].Record.Outer.Segments[0]
	sp.Interfaces[0].LowerExposed[0].Outer.Segments[0] = sectionrecord.CircleSeg{Radius: units.Millimeters(3), CCW: true, TEnd: 1}
	if plan.patches[0].Record.Outer.Segments[0] != old {
		t.Fatal("patch retains mutable source slice")
	}
	if p, ok, e := run(context.Background(), proofbound.NewReplayTrialBudget()); e != nil || ok || p != nil {
		t.Fatalf("mutated floor ok=%t error=%v", ok, e)
	}
	ctx, cancel := context.WithCancel(context.Background())
	cancel()
	if p, ok, e := run(ctx, proofbound.NewReplayTrialBudget()); e == nil || ok || p != nil {
		t.Fatalf("cancellation ok=%t error=%v", ok, e)
	}
}

func TestPrepareBlindBuildReplayRefusesMissingCertificate(t *testing.T) {
	tool := Payload{Slabs: []Slab{{Regions: []momentinput.Profile{{}, {}}}}}
	if out, ok, err := prepareBlindBuildReplay(context.Background(), Payload{}, prismrecord.Payload{}, tool, prismcells.ReplayBlindSection{}, 0, true, nil, proofbound.NewReplayTrialBudget()); err != nil || ok || out != nil {
		t.Fatalf("missing certificate ok=%t error=%v", ok, err)
	}
	ctx, cancel := context.WithCancel(context.Background())
	cancel()
	if out, ok, err := prepareBlindBuildReplay(ctx, Payload{}, prismrecord.Payload{}, tool, prismcells.ReplayBlindSection{}, 0, true, nil, proofbound.NewReplayTrialBudget()); err == nil || ok || out != nil {
		t.Fatalf("cancelled ok=%t error=%v", ok, err)
	}
}
