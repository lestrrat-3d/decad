package topobuild_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/r3"
)

func TestRequireShellWindingReplayBudget(t *testing.T) {
	vertices := []r3.Vec{{}, {X: 1}, {Y: 1}, {Z: 1}}
	outer := [][3]int{{0, 2, 1}, {0, 1, 3}, {0, 3, 2}, {1, 2, 3}}
	face0, face1 := &topo.Face{}, &topo.Face{}
	triangles := append(append([][3]int(nil), outer...), outer...)
	for i := len(outer); i < len(triangles); i++ {
		triangles[i][1], triangles[i][2] = triangles[i][2], triangles[i][1]
	}
	mesh := &topo.Mesh{Vertices: vertices, Triangles: triangles,
		Source: []*topo.Face{face0, face0, face0, face0, face1, face1, face1, face1}}
	lumps := []*topo.Lump{{Shells: []*topo.FaceShell{{Faces: []*topo.Face{face0}},
		{Faces: []*topo.Face{face1}, Void: true}}}}
	if err := topobuild.RequireShellWinding(t.Context(), mesh, lumps); err != nil {
		t.Fatal(err)
	}
	trial := proofbound.NewReplayTrialBudget()
	if err := topobuild.RequireShellWindingReplay(t.Context(), mesh, lumps, trial); err != nil {
		t.Fatal(err)
	}
	work, records := trial.Usage()
	bytes := trial.Bytes()
	for _, tc := range []struct {
		name                 string
		work, records, bytes uint64
		want                 bool
	}{
		{replayTestCaseExact, work, records, bytes, true},
		{"work short", work - 1, records, bytes, false},
		{"records short", work, records - 1, bytes, false},
		{"bytes short", work, records, bytes - 1, false},
	} {
		t.Run(tc.name, func(t *testing.T) {
			meter, err := proofbound.NewReplayBudgetLimits(tc.work, tc.records, tc.bytes)
			if err != nil {
				t.Fatal(err)
			}
			err = topobuild.RequireShellWindingReplay(t.Context(), mesh, lumps, meter)
			if (err == nil) != tc.want {
				t.Fatalf("shell winding refusal=%v want admission=%t", err, tc.want)
			}
		})
	}
}
