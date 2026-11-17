package retainedblend

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// fakeTangent answers every corner with ok and records the faces it was asked about.
type fakeTangent struct {
	ok    bool
	asked [][3]*topo.Face
	proof TangentProof
}

func (f *fakeTangent) Prove(b, g, k *topo.Face) (TangentProof, bool) {
	f.asked = append(f.asked, [3]*topo.Face{b, g, k})
	return f.proof, f.ok
}

// fourFaceCorner is chargedJunction's top-front line and corner arc with a
// fourth face K on the arc's wall at the shared vertex: A is the top, B the
// charged corner wall, G the front and K the new face.
func fourFaceCorner(t *testing.T) (all []*topo.Edge, arc *topo.Edge, v *topo.Vertex, a, b, g, k *topo.Face) {
	t.Helper()
	line, arc, v := chargedJunction(t, 1e-14)
	a, b, g = arc.Faces[0], arc.Faces[1], line.Faces[1]
	k = &topo.Face{Surface: surfacegeom.Cylinder{Origin: r3.NewVec(0, 0, 0), Axis: r3.NewVec(0, 0, 1)}}
	seam := &topo.Edge{Curve: surfacegeom.Line3{}, Start: v, End: &topo.Vertex{Position: r3.NewVec(3, 0, 0)},
		Faces: []*topo.Face{b, k}}
	return []*topo.Edge{line, arc, seam}, arc, v, a, b, g, k
}

func TestProvenEnd(t *testing.T) {
	t.Parallel()

	t.Run("the prover reads the faces by their place in the corner", func(t *testing.T) {
		t.Parallel()
		all, arc, v, _, b, g, k := fourFaceCorner(t)
		prover := &fakeTangent{ok: true}
		require.True(t, provenEnd(prover, all, arc, v))
		require.Equal(t, [][3]*topo.Face{{b, g, k}}, prover.asked)
	})

	t.Run("no prover proves nothing", func(t *testing.T) {
		t.Parallel()
		all, arc, v, _, _, _, _ := fourFaceCorner(t)
		require.False(t, provenEnd(nil, all, arc, v))
		require.False(t, provenArc(nil, all, arc))
	})

	t.Run("a refusing prover proves nothing", func(t *testing.T) {
		t.Parallel()
		all, arc, v, _, _, _, _ := fourFaceCorner(t)
		require.False(t, provenEnd(&fakeTangent{}, all, arc, v))
	})

	t.Run("a corner of three faces is not asked", func(t *testing.T) {
		t.Parallel()
		all, arc, v, _, _, _, _ := fourFaceCorner(t)
		prover := &fakeTangent{ok: true}
		require.False(t, provenEnd(prover, all[:2], arc, v))
		require.Empty(t, prover.asked)
	})

	t.Run("a fifth face is not asked", func(t *testing.T) {
		t.Parallel()
		all, arc, v, _, b, _, _ := fourFaceCorner(t)
		extra := &topo.Edge{Curve: surfacegeom.Line3{}, Start: v, End: &topo.Vertex{}, Faces: []*topo.Face{b, {}}}
		prover := &fakeTangent{ok: true}
		require.False(t, provenEnd(prover, append(all, extra), arc, v))
		require.Empty(t, prover.asked)
	})

	t.Run("a second face beside the plane is not asked", func(t *testing.T) {
		t.Parallel()
		all, arc, v, a, _, _, k := fourFaceCorner(t)
		extra := &topo.Edge{Curve: surfacegeom.Line3{}, Start: v, End: &topo.Vertex{}, Faces: []*topo.Face{a, k}}
		prover := &fakeTangent{ok: true}
		require.False(t, provenEnd(prover, append(all, extra), arc, v))
		require.Empty(t, prover.asked)
	})

	t.Run("an arc passes the charged gate only when both ends are proven", func(t *testing.T) {
		t.Parallel()
		all, arc, _, _, _, _, _ := fourFaceCorner(t)
		prover := &fakeTangent{ok: true}
		// The far end of the arc is a corner of two faces only.
		require.False(t, provenArc(prover, all, arc))
		err := refuseChargedArcProven(arc, all, prover)
		require.ErrorIs(t, err, decaderr.ErrUnsupported)
		require.ErrorContains(t, err, "an arc on a wall whose tag states a displacement is not built")
		require.ErrorIs(t, refuseChargedArcProven(arc, all, nil), decaderr.ErrUnsupported)

		// Give the far end its own four-face corner.
		side := arc.Start
		a, b := arc.Faces[0], arc.Faces[1]
		g, k := &topo.Face{}, &topo.Face{}
		all = append(all,
			&topo.Edge{Curve: surfacegeom.Line3{}, Start: side, End: &topo.Vertex{}, Faces: []*topo.Face{a, g}},
			&topo.Edge{Curve: surfacegeom.Line3{}, Start: side, End: &topo.Vertex{}, Faces: []*topo.Face{b, k}},
			&topo.Edge{Curve: surfacegeom.Line3{}, Start: side, End: &topo.Vertex{}, Faces: []*topo.Face{g, k}})
		require.NoError(t, refuseChargedArcProven(arc, all, prover))
		require.ErrorIs(t, refuseChargedArcProven(arc, all, &fakeTangent{}), decaderr.ErrUnsupported)
	})

	t.Run("a proven end is no junction", func(t *testing.T) {
		t.Parallel()
		all, arc, _, _, _, _, _ := fourFaceCorner(t)
		body := &topo.Body{Lumps: []*topo.Lump{{Shells: []*topo.FaceShell{{Faces: []*topo.Face{{
			Loops: []*topo.Loop{{CoEdges: []topo.CoEdge{{Edge: all[0]}, {Edge: all[1]}, {Edge: all[2]}}}}}}}}}}}
		edges := []*topo.Edge{arc}

		// Red leg: with no prover the shared end is a junction whose continuation is not selected.
		_, _, _, err := planChains(body, edges, nil, false)
		require.ErrorIs(t, err, decaderr.ErrUnsupported)
		require.ErrorContains(t, err, "whose continuation is not selected")

		// Only the shared end is proven; the arc's other end has no continuation, so no link forms.
		chains, rest, handled, err := planChains(body, edges, &fakeTangent{ok: true}, false)
		require.NoError(t, err)
		require.False(t, handled)
		require.Empty(t, chains)
		require.Empty(t, rest)

		_, _, _, err = planChains(body, edges, &fakeTangent{}, false)
		require.ErrorIs(t, err, decaderr.ErrUnsupported)
	})
}
