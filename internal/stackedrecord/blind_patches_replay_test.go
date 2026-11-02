package stackedrecord

import (
	"context"
	"testing"

	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismcells"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/units"
)

func TestReplayBlindPatches(t *testing.T) {
	circle := func(x float64, ccw bool) sectionrecord.LoopRecord {
		c := sectionrecord.CircleSeg{Center: sectionrecord.Point2{U: x}, Radius: units.Millimeters(1), CCW: ccw, TEnd: 1}
		if !ccw {
			c.TStart, c.TEnd = 1, 0
		}
		return sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{c}}
	}
	target := prismrecord.Payload{Z1: 10, Profile: momentinput.Profile{Outer: circle(0, true), Holes: []sectionrecord.LoopRecord{circle(1, false)}}}
	section := prismcells.ReplayBlindSection{Profile: momentinput.Profile{Outer: target.Profile.Outer, Holes: []sectionrecord.LoopRecord{circle(1, false), circle(3, false), circle(6, false)}}, HoleRegions: []int{0, 1}, SectionDelta: 1e-12}
	sp := Payload{SectionDelta: section.SectionDelta, Slabs: []Slab{{Z1: 6, Regions: []momentinput.Profile{target.Profile}}, {Z0: 6, Z1: 10, Regions: []momentinput.Profile{section.Profile}}}, Interfaces: []Interface{{LowerExposed: []momentinput.Profile{{Outer: circle(3, true)}, {Outer: circle(6, true)}}}}}
	columns, rows, err := sp.Columns()
	if err != nil {
		t.Fatal(err)
	}
	want, err := sp.InterfacePatches(columns, rows, 0)
	if err != nil {
		t.Fatal(err)
	}
	run := func(ctx context.Context, b *proofbound.ReplayBudget) ([]Patch, bool, error) {
		return replayBlindPatches(ctx, sp, target, section, 6, true, columns, rows, b)
	}
	baseline := proofbound.NewReplayTrialBudget()
	got, ok, err := run(context.Background(), baseline)
	if err != nil || !ok || len(got) != len(want) {
		t.Fatalf("ok=%t patches=%d error=%v", ok, len(got), err)
	}
	for i := range got {
		if got[i].Role != want[i].Role || got[i].Floor != want[i].Floor || len(got[i].Loops) != 1 || len(want[i].Loops) != 1 || got[i].Loops[0] != want[i].Loops[0] || !equalLoop(got[i].Record.Outer, want[i].Record.Outer) {
			t.Fatalf("patch %d differs", i)
		}
	}
	bottom := sp
	bottom.Slabs = []Slab{{Z1: 6, Regions: []momentinput.Profile{section.Profile}}, {Z0: 6, Z1: 10, Regions: []momentinput.Profile{target.Profile}}}
	bottom.Interfaces = []Interface{{UpperExposed: sp.Interfaces[0].LowerExposed}}
	bottomColumns, bottomRows, e := bottom.Columns()
	if e != nil {
		t.Fatal(e)
	}
	bottomWant, e := bottom.InterfacePatches(bottomColumns, bottomRows, 0)
	if e != nil {
		t.Fatal(e)
	}
	bottomGot, bottomOK, e := replayBlindPatches(context.Background(), bottom, target, section, 6, false, bottomColumns, bottomRows, proofbound.NewReplayTrialBudget())
	if e != nil || !bottomOK || len(bottomGot) != len(bottomWant) {
		t.Fatalf("ceiling ok=%t patches=%d error=%v", bottomOK, len(bottomGot), e)
	}
	for i := range bottomGot {
		if bottomGot[i].Role != bottomWant[i].Role || bottomGot[i].Floor != bottomWant[i].Floor || bottomGot[i].Loops[0] != bottomWant[i].Loops[0] {
			t.Fatalf("ceiling patch %d differs", i)
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
			m, e := proofbound.NewReplayBudgetLimits(tc.w, tc.r, tc.b)
			if e != nil {
				t.Fatal(e)
			}
			got, ok, e := run(context.Background(), m)
			if tc.pass && (e != nil || !ok) {
				t.Fatalf("ok=%t error=%v", ok, e)
			}
			if !tc.pass && (e == nil || ok || got != nil) {
				t.Fatalf("rollback ok=%t error=%v", ok, e)
			}
		})
	}
	rows[1][len(rows[1])-1].Column = -1
	if got, ok, err := run(context.Background(), proofbound.NewReplayTrialBudget()); err != nil || ok || got != nil {
		t.Fatalf("invalid column ok=%t error=%v", ok, err)
	}
	ctx, cancel := context.WithCancel(context.Background())
	cancel()
	if got, ok, err := run(ctx, proofbound.NewReplayTrialBudget()); err == nil || ok || got != nil {
		t.Fatalf("cancel ok=%t error=%v", ok, err)
	}
}

func TestReplayBlindPatchesThreeDigitRole(t *testing.T) {
	circle := func(x float64, ccw bool) sectionrecord.LoopRecord {
		c := sectionrecord.CircleSeg{Center: sectionrecord.Point2{U: x}, Radius: units.Millimeters(1), CCW: ccw, TEnd: 1}
		if !ccw {
			c.TStart, c.TEnd = 1, 0
		}
		return sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{c}}
	}
	target := prismrecord.Payload{Z1: 10, Profile: momentinput.Profile{Outer: circle(0, true)}}
	section := prismcells.ReplayBlindSection{Profile: momentinput.Profile{Outer: target.Profile.Outer}}
	floors := make([]momentinput.Profile, 101)
	for i := range floors {
		section.Profile.Holes = append(section.Profile.Holes, circle(float64(i+1)*4, false))
		section.HoleRegions = append(section.HoleRegions, i)
		floors[i] = momentinput.Profile{Outer: circle(float64(i+1)*4, true)}
	}
	sp := Payload{Slabs: []Slab{{Z1: 6, Regions: []momentinput.Profile{target.Profile}}, {Z0: 6, Z1: 10, Regions: []momentinput.Profile{section.Profile}}}, Interfaces: []Interface{{LowerExposed: floors}}}
	columns, rows, err := sp.Columns()
	if err != nil {
		t.Fatal(err)
	}
	run := func(b *proofbound.ReplayBudget) ([]Patch, bool, error) {
		return replayBlindPatches(context.Background(), sp, target, section, 6, true, columns, rows, b)
	}
	baseline := proofbound.NewReplayTrialBudget()
	patches, ok, err := run(baseline)
	if err != nil || !ok || len(patches) != 101 || patches[100].Role != "floor(0,100)" {
		t.Fatalf("ok=%t count=%d error=%v", ok, len(patches), err)
	}
	w, r := baseline.Usage()
	b := baseline.Bytes()
	exact, err := proofbound.NewReplayBudgetLimits(w, r, b)
	if err != nil {
		t.Fatal(err)
	}
	if out, ok, err := run(exact); err != nil || !ok || len(out) != 101 {
		t.Fatalf("exact ok=%t count=%d error=%v", ok, len(out), err)
	}
	short, err := proofbound.NewReplayBudgetLimits(w, r-1, b)
	if err != nil {
		t.Fatal(err)
	}
	if out, ok, err := run(short); err == nil || ok || out != nil {
		t.Fatalf("short ok=%t error=%v", ok, err)
	}
}
