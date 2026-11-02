package prismcells

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/proofbound"
)

func TestGroupUnionLoopsWithOriginsRetainsDisjointRectangleWalls(t *testing.T) {
	profiles, tags := curvedScene(t, sceneRect(-5, -1, -3, 1), sceneRect(3, -1, 5, 1))
	budget := proofbound.NewReplayTrialBudget()
	loops, origins, _, ok, err := GroupUnionLoopsWithOrigins(budget.WorkBudget(t.Context()),
		GroupScene{Profiles: profiles, Tags: tags})
	if err != nil || !ok || len(loops) != 2 || len(origins) != 2 {
		t.Fatalf("admitted=%t loops=%d origin rows=%d error=%v", ok, len(loops), len(origins), err)
	}
	for i, loop := range loops {
		if len(loop.Segments) != 4 || len(origins[i]) != 4 {
			t.Fatalf("row %d segments=%d origins=%d", i, len(loop.Segments), len(origins[i]))
		}
	}
}
