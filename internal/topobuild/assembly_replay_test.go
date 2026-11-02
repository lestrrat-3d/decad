package topobuild_test

import (
	"context"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
)

const replayTestCaseExact = "exact"

func TestAttachFaceLoopsReplayParityAndLimits(t *testing.T) {
	makeFaces := func() ([]*topo.Face, *topo.Edge) {
		edge := &topo.Edge{}
		return []*topo.Face{
			{Loops: []*topo.Loop{{CoEdges: []topo.CoEdge{{Edge: edge}}}}},
			{Loops: []*topo.Loop{{CoEdges: []topo.CoEdge{{Edge: edge}}}}},
		}, edge
	}
	wantFaces, wantEdge := makeFaces()
	if err := topobuild.AttachFaceLoops(t.Context(), wantFaces); err != nil {
		t.Fatal(err)
	}
	run := func(ctx context.Context, budget *proofbound.ReplayBudget) ([]*topo.Face, *topo.Edge, error) {
		faces, edge := makeFaces()
		if err := topobuild.AttachFaceLoopsReplay(ctx, faces, budget); err != nil {
			return nil, nil, err
		}
		return faces, edge, nil
	}
	meter := proofbound.NewReplayTrialBudget()
	gotFaces, gotEdge, err := run(t.Context(), meter)
	if err != nil {
		t.Fatal(err)
	}
	if len(gotEdge.Faces) != len(wantEdge.Faces) {
		t.Fatalf("face count %d vs %d", len(gotEdge.Faces), len(wantEdge.Faces))
	}
	for i := range gotEdge.Faces {
		if gotEdge.Faces[i] != gotFaces[i] || wantEdge.Faces[i] != wantFaces[i] {
			t.Fatalf("face order differs at %d", i)
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
		{"work short", work - 1, records, bytes, false},
		{"records short", work, records - 1, bytes, false},
		{"bytes short", work, records, bytes - 1, false},
	} {
		t.Run(tc.name, func(t *testing.T) {
			budget, e := proofbound.NewReplayBudgetLimits(tc.w, tc.r, tc.b)
			if e != nil {
				t.Fatal(e)
			}
			faces, _, e := run(t.Context(), budget)
			if tc.pass && (e != nil || faces == nil) || !tc.pass && (e == nil || faces != nil) {
				t.Fatalf("faces=%t err=%v", faces != nil, e)
			}
		})
	}
	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	if faces, _, err := run(ctx, proofbound.NewReplayTrialBudget()); err == nil || faces != nil {
		t.Fatalf("cancelled faces=%t err=%v", faces != nil, err)
	}
}
