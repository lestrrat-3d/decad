package prismcells

import (
	"context"
	"testing"

	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismplacement"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

func TestReplayGroupSourceClearanceBudget(t *testing.T) {
	frame, err := r3.NewFrame(r3.Vec{}, r3.Vec{X: 1}, r3.Vec{Y: 1})
	if err != nil {
		t.Fatal(err)
	}
	place := prismplacement.Operand{Frame: frame, Xform: r3.Identity()}
	point := func(u, v float64) sectionrecord.Point2 { return sectionrecord.Point2{U: u, V: v} }
	corners := []sectionrecord.Point2{point(-20, -20), point(20, -20), point(20, 20), point(-20, 20)}
	target := momentinput.Profile{}
	for i := range corners {
		target.Outer.Segments = append(target.Outer.Segments, sectionrecord.LineSeg{Start: corners[i], End: corners[(i+1)%len(corners)], TEnd: 1})
	}
	circle := func(x float64) momentinput.Profile {
		return momentinput.Profile{Outer: sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{sectionrecord.CircleSeg{Center: point(x, 0), Radius: units.Millimeters(1), CCW: true, TEnd: 1}}}}
	}
	group := []momentinput.Profile{circle(-4), circle(4)}
	run := func(ctx context.Context, budget *proofbound.ReplayBudget) (bool, error) {
		cert := &ReplayBlindScene{budget: budget, target: target, group: group, targetPlacement: place, groupPlacement: place, groupDelta: 1e-12}
		return ReplayGroupSourceClearance(ctx, cert, budget)
	}
	baseline := proofbound.NewReplayTrialBudget()
	ok, err := run(context.Background(), baseline)
	if err != nil || !ok {
		t.Fatalf("clear=%t error=%v", ok, err)
	}
	w, r := baseline.Usage()
	b := baseline.Bytes()
	for _, tc := range []struct {
		name    string
		w, r, b uint64
		pass    bool
	}{
		{"exact", w, r, b, true}, {"work short", w - 1, r, b, false}, {"records short", w, r - 1, b, false}, {"bytes short", w, r, b - 1, false},
	} {
		t.Run(tc.name, func(t *testing.T) {
			meter, err := proofbound.NewReplayBudgetLimits(tc.w, tc.r, tc.b)
			if err != nil {
				t.Fatal(err)
			}
			ok, err := run(context.Background(), meter)
			if tc.pass && (err != nil || !ok) {
				t.Fatalf("clear=%t error=%v", ok, err)
			}
			if !tc.pass && (err == nil || ok) {
				t.Fatalf("clear=%t error=%v", ok, err)
			}
		})
	}
	group[1] = circle(-3)
	if ok, err := run(context.Background(), proofbound.NewReplayTrialBudget()); err != nil || ok {
		t.Fatalf("overlap clear=%t error=%v", ok, err)
	}
	group[1] = circle(-2)
	if ok, err := run(context.Background(), proofbound.NewReplayTrialBudget()); err != nil || ok {
		t.Fatalf("contact clear=%t error=%v", ok, err)
	}
	group[1] = circle(-2 + 1e-9)
	if ok, err := run(context.Background(), proofbound.NewReplayTrialBudget()); err != nil || !ok {
		t.Fatalf("separated clear=%t error=%v", ok, err)
	}
	other := proofbound.NewReplayTrialBudget()
	cert := &ReplayBlindScene{budget: baseline, target: target, group: group, targetPlacement: place, groupPlacement: place}
	if ok, err := ReplayGroupSourceClearance(context.Background(), cert, other); err != nil || ok {
		t.Fatalf("different meter clear=%t error=%v", ok, err)
	}
	ctx, cancel := context.WithCancel(context.Background())
	cancel()
	if ok, err := run(ctx, proofbound.NewReplayTrialBudget()); err == nil || ok {
		t.Fatalf("cancel clear=%t error=%v", ok, err)
	}
}
