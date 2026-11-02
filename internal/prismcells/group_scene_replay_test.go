package prismcells

import (
	"context"
	"testing"

	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
)

func TestGroupSceneProfilesReplayLimits(t *testing.T) {
	profiles := []momentinput.Profile{{Outer: sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{sectionrecord.LineSeg{TEnd: 1}}}}, {Outer: sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{sectionrecord.LineSeg{TEnd: 1}}}}}
	run := func(ctx context.Context, m *proofbound.ReplayBudget) ([]SceneProfile, error) {
		return groupSceneProfiles(m.WorkBudget(ctx), profiles)
	}
	baseline := proofbound.NewReplayTrialBudget()
	out, err := run(context.Background(), baseline)
	if err != nil || len(out) != len(profiles) {
		t.Fatalf("count=%d error=%v", len(out), err)
	}
	w, r := baseline.Usage()
	b := baseline.Bytes()
	for _, tc := range []struct {
		name    string
		w, r, b uint64
		pass    bool
	}{{"exact", w, r, b, true}, {"work short", w - 1, r, b, false}, {"records short", w, r - 1, b, false}, {"bytes short", w, r, b - 1, false}} {
		t.Run(tc.name, func(t *testing.T) {
			m, e := proofbound.NewReplayBudgetLimits(tc.w, tc.r, tc.b)
			if e != nil {
				t.Fatal(e)
			}
			out, e := run(context.Background(), m)
			if tc.pass && (e != nil || len(out) != len(profiles)) {
				t.Fatalf("count=%d error=%v", len(out), e)
			}
			if !tc.pass && (e == nil || out != nil) {
				t.Fatalf("rollback count=%d error=%v", len(out), e)
			}
		})
	}
	ctx, cancel := context.WithCancel(context.Background())
	cancel()
	if out, e := run(ctx, proofbound.NewReplayTrialBudget()); e == nil || out != nil {
		t.Fatalf("cancel count=%d error=%v", len(out), e)
	}
}
