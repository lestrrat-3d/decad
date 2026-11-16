package brepmodify

import (
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/topo"
)

// cubeMesh is the closed, outward-wound mesh of the box between lo and hi.
func cubeMesh(lo, hi r3.Vec) ([]r3.Vec, [][3]int) {
	v := []r3.Vec{
		{X: lo.X, Y: lo.Y, Z: lo.Z}, {X: hi.X, Y: lo.Y, Z: lo.Z}, {X: hi.X, Y: hi.Y, Z: lo.Z}, {X: lo.X, Y: hi.Y, Z: lo.Z},
		{X: lo.X, Y: lo.Y, Z: hi.Z}, {X: hi.X, Y: lo.Y, Z: hi.Z}, {X: hi.X, Y: hi.Y, Z: hi.Z}, {X: lo.X, Y: hi.Y, Z: hi.Z},
	}
	t := [][3]int{
		{0, 2, 1}, {0, 3, 2}, {4, 5, 6}, {4, 6, 7}, {0, 1, 5}, {0, 5, 4}, {1, 2, 6}, {1, 6, 5}, {2, 3, 7}, {2, 7, 6},
		{3, 0, 4}, {3, 4, 7},
	}
	return v, t
}

// clearFixture is a receiver whose three faces hold one facet each: the loop's own face, its wall, and another face. The
// run's cutter is a unit cube about the origin's octant.
type clearFixture struct {
	body, cutter *topo.Body
	run          chamferRun
}

func newClearFixture(other [3]r3.Vec, ownInside bool) clearFixture {
	own, wall, far := &topo.Face{}, &topo.Face{}, &topo.Face{}
	// The facets of the own faces lie inside the cutter when ownInside; the other face's facet is `other`.
	in := [3]r3.Vec{{X: 0.2, Y: 0.2, Z: 0.5}, {X: 0.4, Y: 0.2, Z: 0.5}, {X: 0.2, Y: 0.4, Z: 0.5}}
	if !ownInside {
		in = [3]r3.Vec{{X: 5, Y: 5, Z: 5}, {X: 6, Y: 5, Z: 5}, {X: 5, Y: 6, Z: 5}}
	}
	verts := append(append([]r3.Vec{}, in[:]...), other[:]...)
	fp := facetedrecord.Payload{
		Verts: verts, Tris: [][3]int{{0, 1, 2}, {3, 4, 5}}, FaceOf: []int{0, 2},
		VertexBound: make([]float64, len(verts)),
	}
	body := &topo.Body{Payload: fp, Lumps: []*topo.Lump{{Shells: []*topo.FaceShell{{Faces: []*topo.Face{own, wall, far}}}}}}
	cv, ct := cubeMesh(r3.Vec{}, r3.Vec{X: 1, Y: 1, Z: 1})
	cutter := &topo.Body{Payload: facetedrecord.Payload{Verts: cv, Tris: ct, VertexBound: make([]float64, len(cv)),
		FaceOf: make([]int, len(ct))}}
	// Three edges around the run: before it, the run's own, and after it, each between the own face and the wall.
	edge := &topo.Edge{Faces: []*topo.Face{own, wall}}
	loop := &chamferLoop{face: own, edges: []*topo.Edge{edge, edge, edge}}
	return clearFixture{body: body, cutter: cutter, run: chamferRun{loop: loop, first: 1, count: 1}}
}

func TestCutterClearRefusesAnotherFaceOfTheBody(t *testing.T) {
	t.Parallel()
	crossing := [3]r3.Vec{{X: 0.5, Y: 0.5, Z: -1}, {X: 0.6, Y: 0.5, Z: 2}, {X: 0.5, Y: 0.6, Z: 2}}
	inside := [3]r3.Vec{{X: 0.3, Y: 0.3, Z: 0.3}, {X: 0.4, Y: 0.3, Z: 0.3}, {X: 0.3, Y: 0.4, Z: 0.3}}
	near := [3]r3.Vec{{X: 1.004, Y: 0.3, Z: 0.3}, {X: 1.004, Y: 0.4, Z: 0.3}, {X: 1.004, Y: 0.3, Z: 0.4}}
	away := [3]r3.Vec{{X: 3, Y: 0.3, Z: 0.3}, {X: 3, Y: 0.4, Z: 0.3}, {X: 3, Y: 0.3, Z: 0.4}}
	for _, c := range []struct {
		name   string
		facet  [3]r3.Vec
		bound  float64
		refuse bool
	}{
		{"a facet crossing the cutter", crossing, 0, true},
		{"a facet wholly inside the cutter", inside, 0, true},
		{"a facet within the cutter's bound", near, 0.01, true},
		{"a facet beyond the cutter's bound", near, 0.001, false},
		{"a facet far from the cutter", away, 0.01, false},
	} {
		t.Run(c.name, func(t *testing.T) {
			t.Parallel()
			// The own faces' facet lies inside the cutter and is not tested.
			fx := newClearFixture(c.facet, true)
			err := fx.run.checkCutterClear(t.Context(), fx.body, fx.cutter, c.bound)
			if c.refuse {
				require.ErrorIs(t, err, decaderr.ErrUnsupported)
				return
			}
			require.NoError(t, err)
		})
	}
}

// A receiver's own faces may lie anywhere in the cutter, and a facet's own bound widens its clearance.
func TestCutterClearReadsFacetBounds(t *testing.T) {
	t.Parallel()
	near := [3]r3.Vec{{X: 1.004, Y: 0.3, Z: 0.3}, {X: 1.004, Y: 0.4, Z: 0.3}, {X: 1.004, Y: 0.3, Z: 0.4}}
	fx := newClearFixture(near, false)
	require.NoError(t, fx.run.checkCutterClear(t.Context(), fx.body, fx.cutter, 0.001))
	fp := fx.body.Payload.(facetedrecord.Payload)
	fp.VertexBound[3] = 0.01
	fx.body.Payload = fp
	require.ErrorIs(t, fx.run.checkCutterClear(t.Context(), fx.body, fx.cutter, 0.001), decaderr.ErrUnsupported)
}
