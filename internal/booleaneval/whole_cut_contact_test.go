package booleaneval

import (
	"context"
	"testing"

	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/topo"
)

func TestWholeCutBindsPreparedOperands(t *testing.T) {
	a, b := &topo.Body{}, &topo.Body{}
	mesh := func(owner *topo.Body) *topo.Mesh {
		return &topo.Mesh{Triangles: [][3]int{{0, 1, 2}}, Source: []*topo.Face{{Body: owner}}}
	}
	ma, mb := mesh(a), mesh(b)
	meter := proofbound.NewStrictReplayBudget()
	for _, tc := range []struct {
		name         string
		target, tool *topo.Body
		ma, mb       *topo.Mesh
		reported     float64
		called, ok   bool
	}{
		{"overlap", a, b, ma, mb, .125, true, true},
		{"prepared pair", a, b, ma, mb, .125, true, true},
		{"wrong operand order", b, a, mb, ma, .125, false, false},
		{"foreign tool mesh", a, b, ma, ma, .125, false, false},
		{"above original full bound", a, b, ma, mb, 1.125, true, false},
		{"above original slab", a, b, ma, mb, .75, true, true},
		{"invalid bound", a, b, ma, mb, -1, true, false},
	} {
		t.Run(tc.name, func(t *testing.T) {
			called := false
			ctx := WithWholeCutBounds(t.Context(), a, b, meter, func(_ context.Context, stock, tool *topo.Mesh, m *proofbound.ReplayBudget) (map[int]meshbool.CutFacetBound, bool, error) {
				called = true
				if stock != ma || tool != mb || m != meter {
					t.Fatal("prepared operands or budget changed")
				}
				return map[int]meshbool.CutFacetBound{0: {Distance: tc.reported, Preserve: tc.name == "overlap"}}, true, nil
			})
			bm := meshbool.BoolMesh{FacetBound: []float64{1}, SlabBound: []float64{.5}}
			ok, err := bindWholeCut(ctx, tc.target, tc.tool, tc.ma, tc.mb, &bm)
			if err != nil || ok != tc.ok || called != tc.called {
				t.Fatalf("admitted %t, called %t, error %v", ok, called, err)
			}
			want := tc.reported
			if tc.name == "overlap" {
				want = .5
			}
			if val, present := bm.CutGateBounds[0]; present != tc.ok || present && val != want {
				t.Fatal("incorrect gate bound")
			}
		})
	}
}
