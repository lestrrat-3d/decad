package rimextension

// CI: isolated-tests

import (
	"context"
	"testing"

	"github.com/lestrrat-3d/decad/internal/loftrecord"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

func TestWindowRuledSupport(t *testing.T) {
	frame, err := r3.NewFrame(r3.Vec{X: 32}, r3.Vec{X: 1}, r3.Vec{Z: 1})
	if err != nil {
		t.Fatal(err)
	}
	p := loftrecord.Payload{SourceIdentity: new(loftrecord.SourceIdentity), AuthenticatedReconstruction: true, Frame0: frame, Frame1: frame, Xform: r3.Identity(), ConstructionDelta: 0.125}
	p.Profile0.Outer.Segments = append(p.Profile0.Outer.Segments, momentinput.LineSeg{End: momentinput.Point2{U: 3.8}, TEnd: 1})
	p.Profile1.Outer.Segments = append(p.Profile1.Outer.Segments, momentinput.ArcSeg{Start: momentinput.Point2{V: 4}, End: momentinput.Point2{U: -4}, TEnd: 1})
	test := func(m *proofbound.ReplayBudget) (bool, error) {
		x, ok, e := LoftWindowXMax(context.Background(), &p, 0.5, m)
		if ok && x.Cmp(scalar(36.625)) != 0 {
			t.Fatalf("upper %s", x)
		}
		return ok, e
	}
	if ok, e := test(proofbound.NewReplayTrialBudget()); !ok || e != nil {
		t.Fatalf("support %t %v", ok, e)
	}
	m, e := proofbound.NewReplayBudgetLimits(1, 1, 1)
	if e != nil {
		t.Fatal(e)
	}
	if ok, e := test(m); ok || e == nil {
		t.Fatalf("budget %t %v", ok, e)
	}
	p.SourceIdentity = nil
	if ok, e := test(proofbound.NewReplayTrialBudget()); ok || e != nil {
		t.Fatalf("source %t %v", ok, e)
	}
}

func TestWindowCrossingRefusesSecondExteriorStrip(t *testing.T) {
	n := windowNative{x0: scalar(0), z0: scalar(.99), m: scalar(0), cx: scalar(0), cz: scalar(0), r: scalar(1), out: 1, last: scalar(.5)}
	run := func(want bool) {
		t.Helper()
		at, _, ok, e := windowCrossing(context.Background(), n, scalar(-.5), proofbound.NewStrictReplayBudget())
		if ok != want || e != nil {
			t.Fatalf("crossing %t error %v", ok, e)
		}
		if ok && (at.Cmp(scalar(-.15)) < 0 || at.Cmp(scalar(-.14)) > 0) {
			t.Fatalf("crossing at %s", at)
		}
	}
	run(false)
	n.last = scalar(0)
	run(true)
	n.z0 = scalar(1.01)
	run(false)
}
