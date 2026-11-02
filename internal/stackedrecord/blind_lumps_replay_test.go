package stackedrecord

import (
	"context"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topo/topotest"
	"github.com/lestrrat-3d/decad/internal/topobuild"
)

const replayTestCaseRecordsShort = "records short"
const replayTestCaseWorkShort = "work short"

func TestLumpsReplayMatchesConnectedShellsAndLimits(t *testing.T) {
	makeInput := func() ([]*topo.Face, map[*topo.Face]struct{}) {
		shared, lone := &topo.Edge{}, &topo.Edge{}
		faces := []*topo.Face{
			{Loops: []*topo.Loop{{CoEdges: []topo.CoEdge{{Edge: shared}}}}},
			{Loops: []*topo.Loop{{CoEdges: []topo.CoEdge{{Edge: shared}}}}},
			{Loops: []*topo.Loop{{CoEdges: []topo.CoEdge{{Edge: lone}}}}},
		}
		if err := topobuild.AttachFaceLoops(t.Context(), faces); err != nil {
			t.Fatal(err)
		}
		return faces, map[*topo.Face]struct{}{faces[0]: {}}
	}
	wantFaces, wantAnchors := makeInput()
	want, err := lumps(topotest.Alloc{}, wantFaces, wantAnchors)
	if err != nil {
		t.Fatal(err)
	}
	run := func(ctx context.Context, budget *proofbound.ReplayBudget) ([]*topo.Lump, error) {
		faces, anchors := makeInput()
		return lumpsReplay(ctx, topotest.Alloc{}, faces, anchors, budget)
	}
	meter := proofbound.NewReplayTrialBudget()
	got, err := run(t.Context(), meter)
	if err != nil {
		t.Fatal(err)
	}
	if len(got) != len(want) || len(got[0].Shells) != len(want[0].Shells) {
		t.Fatalf("lump/shell count differs: %d/%d vs %d/%d", len(got), len(got[0].Shells), len(want), len(want[0].Shells))
	}
	for i, shell := range got[0].Shells {
		if shell.Void != want[0].Shells[i].Void || shell.Open != want[0].Shells[i].Open ||
			len(shell.Faces) != len(want[0].Shells[i].Faces) {
			t.Fatalf("shell %d differs", i)
		}
	}
	work, records := meter.Usage()
	bytes := meter.Bytes()
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
			budget, e := proofbound.NewReplayBudgetLimits(tc.w, tc.r, tc.b)
			if e != nil {
				t.Fatal(e)
			}
			out, e := run(t.Context(), budget)
			if tc.pass && (e != nil || out == nil) || !tc.pass && (e == nil || out != nil) {
				t.Fatalf("out=%t err=%v", out != nil, e)
			}
		})
	}
	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	if out, err := run(ctx, proofbound.NewReplayTrialBudget()); err == nil || out != nil {
		t.Fatalf("cancelled out=%t err=%v", out != nil, err)
	}
}
