package meshbool_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// gateBox is one closed held operand: the axis-aligned box [lo, hi] with
// every facet attributed to one face, which the mesh charges delta.
type gateBox struct {
	face *topo.Face
	mesh *topo.Mesh
	bool *meshbool.BoolMesh
}

func newGateBox(t *testing.T, lo, hi r3.Vec, delta float64) gateBox {
	t.Helper()
	verts := make([]r3.Vec, 8)
	for k := range verts {
		v := lo
		if k&1 != 0 {
			v.X = hi.X
		}
		if k&2 != 0 {
			v.Y = hi.Y
		}
		if k&4 != 0 {
			v.Z = hi.Z
		}
		verts[k] = v
	}
	// Outward-wound quads on the six sides, two triangles each.
	quads := [][4]int{{0, 2, 3, 1}, {4, 5, 7, 6}, {0, 1, 5, 4}, {2, 6, 7, 3}, {0, 4, 6, 2}, {1, 3, 7, 5}}
	tris := make([][3]int, 0, 12)
	for _, q := range quads {
		tris = append(tris, [3]int{q[0], q[1], q[2]}, [3]int{q[0], q[2], q[3]})
	}
	face := &topo.Face{}
	source := make([]*topo.Face, len(tris))
	for i := range source {
		source[i] = face
	}
	bm, err := meshbool.PrepContext(t.Context(), verts, tris, nil)
	require.NoError(t, err)
	beta := make([]float64, len(verts))
	for i := range beta {
		beta[i] = delta
	}
	bm.SetVertexBounds(beta)
	return gateBox{
		face: face,
		mesh: &topo.Mesh{Vertices: verts, Triangles: tris, Source: source,
			FaceBound: map[*topo.Face]float64{face: delta}},
		bool: bm,
	}
}

func gate(t *testing.T, a, b gateBox, capFace, coneFace *topo.Face) error {
	t.Helper()
	return meshbool.RefuseUndecidableProximity(t.Context(), a.mesh, b.mesh, a.bool, b.bool,
		meshbool.NewContactMemo(a.bool, b.bool), capFace, coneFace)
}

func TestRefuseUndecidableProximity(t *testing.T) {
	// b sits 0.1 above a's top: within a combined chord slack of 0.2, beyond
	// one of 0.02. c overlaps a.
	pairOf := func(deltaA, deltaB float64) (gateBox, gateBox) {
		return newGateBox(t, r3.NewVec(0, 0, 0), r3.NewVec(1, 1, 1), deltaA),
			newGateBox(t, r3.NewVec(0.25, 0.25, 1.1), r3.NewVec(0.75, 0.75, 2), deltaB)
	}
	pair := func(delta float64) (gateBox, gateBox) { return pairOf(delta, delta) }
	overlapping := func() (gateBox, gateBox) {
		return newGateBox(t, r3.NewVec(0, 0, 0), r3.NewVec(1, 1, 1), 0.1),
			newGateBox(t, r3.NewVec(0.25, 0.25, 0.5), r3.NewVec(0.75, 0.75, 2), 0.1)
	}
	t.Run("exact faces hide no tangency", func(t *testing.T) {
		a, b := pair(0)
		require.NoError(t, gate(t, a, b, nil, nil))
	})
	t.Run("a pair farther apart than its slack is decided", func(t *testing.T) {
		a, b := pair(0.01)
		require.NoError(t, gate(t, a, b, nil, nil))
	})
	t.Run("a near miss within the slack is refused", func(t *testing.T) {
		a, b := pair(0.1)
		err := gate(t, a, b, nil, nil)
		require.ErrorIs(t, err, decaderr.ErrUnsupported)
		require.ErrorContains(t, err, "held facets come within the chord tolerance")
	})
	t.Run("the slack sums both faces' bounds", func(t *testing.T) {
		a, b := pairOf(0.06, 0.06)
		require.ErrorIs(t, gate(t, a, b, nil, nil), decaderr.ErrUnsupported)
	})
	t.Run("a deep crossing is decided", func(t *testing.T) {
		a, b := overlapping()
		require.NoError(t, gate(t, a, b, nil, nil))
	})
	t.Run("a cone-cap pair admits held facets that do not touch", func(t *testing.T) {
		a, b := pair(0.1)
		require.NoError(t, gate(t, a, b, a.face, b.face))
	})
	t.Run("a cone-cap pair refuses a held contact", func(t *testing.T) {
		a, b := overlapping()
		err := gate(t, a, b, a.face, b.face)
		require.ErrorIs(t, err, decaderr.ErrUnsupported)
		require.ErrorContains(t, err, "held facet contact")
	})
	t.Run("the cone-cap exemption covers only its own pair", func(t *testing.T) {
		a, b := pair(0.1)
		err := gate(t, a, b, a.face, &topo.Face{})
		require.ErrorIs(t, err, decaderr.ErrUnsupported)
		require.ErrorContains(t, err, "held facets come within the chord tolerance")
	})
	t.Run("a source face with no stated bound is an evaluator failure", func(t *testing.T) {
		a, b := pair(0.1)
		delete(b.mesh.FaceBound, b.face)
		require.ErrorIs(t, gate(t, a, b, nil, nil), decaderr.ErrBooleanFailed)
	})
}

func TestRefuseUndecidableProximityLocalSlack(t *testing.T) {
	t.Parallel()
	// b sits 0.1 above a's top. The faces' bounds sum to 0.2, but the facets that
	// meet there hold their true pieces within 0.01 each.
	a := newGateBox(t, r3.NewVec(0, 0, 0), r3.NewVec(1, 1, 1), 0.1)
	b := newGateBox(t, r3.NewVec(0.25, 0.25, 1.1), r3.NewVec(0.75, 0.75, 2), 0.1)
	tight := make([]float64, len(a.mesh.Vertices))
	for i := range tight {
		tight[i] = 0.01
	}
	a.bool.SetVertexBounds(tight)
	tightB := make([]float64, len(b.mesh.Vertices))
	for i := range tightB {
		tightB[i] = 0.01
	}
	b.bool.SetVertexBounds(tightB)
	t.Run("close facets with small bounds decide a pair the face bounds cannot", func(t *testing.T) {
		require.NoError(t, gate(t, a, b, nil, nil))
	})
	t.Run("a refusal names the contacts and their slack", func(t *testing.T) {
		c := newGateBox(t, r3.NewVec(0.25, 0.25, 1.01), r3.NewVec(0.75, 0.75, 2), 0.1)
		cb := make([]float64, len(c.mesh.Vertices))
		for i := range cb {
			cb[i] = 0.01
		}
		c.bool.SetVertexBounds(cb)
		err := gate(t, a, c, nil, nil)
		var touch *meshbool.TouchContactError
		require.ErrorAs(t, err, &touch)
		require.NotEmpty(t, touch.Contacts)
		for _, ct := range touch.Contacts {
			require.InDelta(t, 0.02, ct.Slack, 1e-8)
			require.InDelta(t, 1, ct.NormalA.Len(), 1e-9)
		}
	})
}

func TestRefuseUndecidableProximityFaceBoxes(t *testing.T) {
	t.Parallel()
	const delta = 0.0625
	slack := 2 * delta * (1 + 1e-9)
	for _, axis := range []r3.Vec{{X: 1}, {Y: 1}, {Z: 1}, {X: -1}, {Y: -1}, {Z: -1}} {
		for _, gap := range []float64{0, delta, slack, math.Nextafter(slack, math.Inf(1)), 0.25} {
			for _, split := range []bool{false, true} {
				lo, hi := r3.NewVec(100, 200, 300), r3.NewVec(101, 201, 301)
				a := newGateBox(t, lo, hi, delta)
				shift := axis.Scale(1 + gap)
				b := newGateBox(t, lo.Add(shift), hi.Add(shift), delta)
				size := len(a.mesh.Source)
				if split {
					size = 2
					for _, box := range []gateBox{a, b} {
						for i := 0; i < len(box.mesh.Source); i += 2 {
							face := &topo.Face{}
							box.mesh.Source[i], box.mesh.Source[i+1] = face, face
							box.mesh.FaceBound[face] = delta
						}
					}
				}
				// Scan every face pair without the face-box filter, including
				// the exact same padding and rounding at the slack boundary.
				allA, allB := meshbool.AllFacets(a.bool), meshbool.AllFacets(b.bool)
				memo := meshbool.NewContactMemo(a.bool, b.bool)
				near := false
				for i := 0; i < len(allA); i += size {
					for j := 0; j < len(allB); j += size {
						got, err := meshbool.FacesNearMiss(t.Context(), a.bool, allA[i:i+size],
							b.bool, allB[j:j+size], slack, memo)
						require.NoError(t, err)
						near = near || got
					}
				}
				err := gate(t, a, b, nil, nil)
				if near {
					require.ErrorIs(t, err, decaderr.ErrUnsupported, "axis %v, gap %g, split %t", axis, gap, split)
				} else {
					require.NoError(t, err, "axis %v, gap %g, split %t", axis, gap, split)
				}
			}
		}
	}
}
