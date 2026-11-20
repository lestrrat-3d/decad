package loftmesh

import (
	"errors"
	"testing"

	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

func TestBoundedCrossingCandidateStorage(t *testing.T) {
	verts := []r3.Vec{{}, {X: 1}, {Y: 1}, {Z: 1}}
	tris := [][3]int{{0, 2, 1}, {0, 1, 3}, {0, 3, 2}, {1, 2, 3}}
	for _, tt := range []struct {
		name           string
		records, bytes uint64
		resource       string
	}{
		{"fits", 1024, 4096, ""}, {"bytes", 1024, 1, "replay_bytes"}, {"records", 1, 4096, "multiway_records"},
	} {
		t.Run(tt.name, func(t *testing.T) {
			meter, err := proofbound.NewReplayBudgetLimits(100000, tt.records, tt.bytes)
			if err != nil {
				t.Fatal(err)
			}
			err = LoftCrossingAuditBounded(meter.WorkBudget(t.Context()), verts, tris, 4, 1000)
			if tt.resource == "" {
				if err != nil {
					t.Fatal(err)
				}
				return
			}
			var limit *proofbound.WorkLimitError
			if !errors.As(err, &limit) || limit.Resource != tt.resource {
				t.Fatalf("error %v", err)
			}
		})
	}
}

func TestBoundedCrossingVerdictStorage(t *testing.T) {
	meter, err := proofbound.NewReplayBudgetLimits(1000, 128, 1)
	if err != nil {
		t.Fatal(err)
	}
	cands := loftCandidates{starts: []int{0, 1, 1}, js: []int32{1}}
	i, j, err := parallelLoftPairsWorkers(meter.WorkBudget(t.Context()), 1, cands, nil, &LoftAuditWork{})
	var limit *proofbound.WorkLimitError
	if i != -1 || j != -1 || !errors.As(err, &limit) || limit.Resource != "replay_bytes" {
		t.Fatalf("pair %d, %d; error %v", i, j, err)
	}
}

func TestBoundedCrossingStreamStorage(t *testing.T) {
	verts := []r3.Vec{{}, {X: 1}, {Y: 1}, {Z: 1}}
	tris := [][3]int{{0, 2, 1}, {0, 1, 3}, {0, 3, 2}, {1, 2, 3}}
	members := []int{0, 1, 2, 3}
	starts := []int{0, 3, 5, 6, 6}
	boxes := make([][2]r3.Vec, len(tris))
	for i, tri := range tris {
		boxes[i] = meshbool.TriBox(verts, tri)
	}
	tree := newTreeScan(boxes, members)
	for _, available := range []uint64{1, 4096} {
		b, e := proofbound.NewReplayBudgetLimits(10000, 1024, available)
		if e != nil {
			t.Fatal(e)
		}
		calls := 0
		_, _, e = streamLoftPairsWorkers(b.WorkBudget(t.Context()), tree, members, starts, 6, 1024, 2, tris, boxes, nil, LoftAuditShortcuts{}, &LoftAuditWork{}, func(i, j int) error { calls++; return nil }, 2)
		if available == 1 {
			var limit *proofbound.WorkLimitError
			if !errors.As(e, &limit) || limit.Resource != "replay_bytes" || calls != 0 {
				t.Fatalf("calls %d, error %v", calls, e)
			}
		} else if e != nil || calls != 6 {
			t.Fatalf("calls %d, error %v", calls, e)
		}
	}
}
