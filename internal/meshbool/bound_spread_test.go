package meshbool

import (
	"errors"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func spreadIdentity(v r3.Vec) r3.Vec { return v }

// spreadDelta is the bound of the operand facet every test piece lies on.
const spreadDelta = 0.001

func spreadFacet(beta [3]float64, pts ...r3.Vec) KeptFacet {
	f := KeptFacet{Src: 7, Beta: beta, Delta: spreadDelta}
	for k := range 3 {
		f.V[k] = proof.XptOf(pts[k])
	}
	return f
}

// spreadArea is the area of the exact corners read as floats.
func spreadArea(f KeptFacet) float64 {
	a, b, c := f.V[0].Vec(), f.V[1].Vec(), f.V[2].Vec()
	return b.Sub(a).Cross(c.Sub(a)).Len() / 2
}

func spreadNormal(f KeptFacet) r3.Vec {
	a, b, c := f.V[0].Vec(), f.V[1].Vec(), f.V[2].Vec()
	n, _ := b.Sub(a).Cross(c.Sub(a)).Normalize()
	return n
}

// curvedMesh is an operand whose four side quads are one source face, so every
// facet of it, facet 0 included, lies on a face that is not planar.
func curvedMesh(t *testing.T) *BoolMesh {
	t.Helper()
	box := boxMesh(t, r3.Vec{X: 1, Y: 1, Z: 1}, spreadIdentity, r3.Vec{})
	src := []int{0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 2, 2}
	m, err := PrepContext(t.Context(), box.Verts, box.Tris, src)
	require.NoError(t, err)
	m.SetVertexBounds(box.VertexBound)
	return m
}

// farMesh is an operand 100 mm from the pieces the tests split.
func farMesh(t *testing.T) *BoolMesh {
	t.Helper()
	return boxMesh(t, r3.Vec{X: 1, Y: 1, Z: 1}, spreadIdentity, r3.Vec{X: 100})
}

// A long piece with one rim corner splits into a rim piece and two pieces whose
// corners all carry the operand facet's own bound. The pieces keep the area and
// the orientation of the piece, and the rim piece keeps the rim's bound.
func TestSplitSpreadOneRimCorner(t *testing.T) {
	t.Parallel()
	const delta = spreadDelta
	piece := spreadFacet([3]float64{0.05, delta, delta},
		r3.Vec{}, r3.Vec{Z: 20}, r3.Vec{X: 1, Z: 20})
	out, err := SplitSpread(t.Context(), []KeptFacet{piece}, curvedMesh(t), farMesh(t), 0, nil)
	require.NoError(t, err)
	require.Len(t, out, 3)

	area := 0.0
	rimPieces := 0
	for _, f := range out {
		area += spreadArea(f)
		require.InDelta(t, 1.0, spreadNormal(f).Dot(spreadNormal(piece)), 1e-12)
		require.Equal(t, piece.Src, f.Src)
		require.Equal(t, delta, f.Delta)
		if max(f.Beta[0], f.Beta[1], f.Beta[2]) > delta {
			rimPieces++
			require.Equal(t, 0.05, max(f.Beta[0], f.Beta[1], f.Beta[2]))
			// The rim piece reaches no farther than a few times its bound.
			for _, v := range f.V {
				require.LessOrEqual(t, v.Vec().Len(), 8*0.05+1e-9)
			}
		}
	}
	require.Equal(t, 1, rimPieces)
	require.InDelta(t, spreadArea(piece), area, 1e-9)
}

// A piece with two rim corners splits into a rim piece of two facets and a far
// piece that holds only the operand facet's bound.
func TestSplitSpreadTwoRimCorners(t *testing.T) {
	t.Parallel()
	const delta = spreadDelta
	piece := spreadFacet([3]float64{0.05, 0.04, delta},
		r3.Vec{}, r3.Vec{X: 1}, r3.Vec{X: 0.5, Z: 30})
	out, err := SplitSpread(t.Context(), []KeptFacet{piece}, curvedMesh(t), farMesh(t), 0, nil)
	require.NoError(t, err)
	require.Len(t, out, 3)
	far := 0
	area := 0.0
	for _, f := range out {
		area += spreadArea(f)
		require.InDelta(t, 1.0, spreadNormal(f).Dot(spreadNormal(piece)), 1e-12)
		if max(f.Beta[0], f.Beta[1], f.Beta[2]) <= delta {
			far++
		}
	}
	require.Equal(t, 1, far)
	require.InDelta(t, spreadArea(piece), area, 1e-9)
}

// A piece on a planar face is never split, since a split point is a vertex off
// every boundary loop of its face.
func TestSplitSpreadLeavesPlanarFaces(t *testing.T) {
	t.Parallel()
	const delta = spreadDelta
	piece := spreadFacet([3]float64{0.05, delta, delta},
		r3.Vec{}, r3.Vec{Z: 20}, r3.Vec{X: 1, Z: 20})
	out, err := SplitSpread(t.Context(), []KeptFacet{piece}, farMesh(t), farMesh(t), 0, nil)
	require.NoError(t, err)
	require.Len(t, out, 1)
}

// The pieces SplitSpread must leave alone come back as the same slice.
func TestSplitSpreadLeavesPiecesWithoutAFarPart(t *testing.T) {
	t.Parallel()
	const delta = spreadDelta
	pts := []r3.Vec{{}, {Z: 20}, {X: 1, Z: 20}}
	for name, piece := range map[string]KeptFacet{
		"no rim corner":   spreadFacet([3]float64{delta, delta, delta}, pts...),
		"all rim corners": spreadFacet([3]float64{0.05, 0.05, 0.05}, pts...),
		// 0.4 mm is under twice the reach of a 0.05 bound.
		"edge shorter than the reach": spreadFacet([3]float64{0.05, delta, delta},
			r3.Vec{}, r3.Vec{Z: 0.4}, r3.Vec{X: 0.2, Z: 0.4}),
	} {
		in := []KeptFacet{piece}
		out, err := SplitSpread(t.Context(), in, curvedMesh(t), farMesh(t), 0, nil)
		require.NoError(t, err, name)
		require.Len(t, out, 1, name)
		require.Equal(t, in[0].Beta, out[0].Beta, name)
		require.Equal(t, in[0].V, out[0].V, name)
	}
}

// A split point is taken only where the other operand's held boundary is
// farther than the operand facet's bound plus the other operand's: a piece that
// runs alongside it within that distance stays whole.
func TestSplitSpreadRefusesPointsNearTheOtherOperand(t *testing.T) {
	t.Parallel()
	const delta = spreadDelta
	piece := spreadFacet([3]float64{0.05, delta, delta},
		r3.Vec{}, r3.Vec{Z: 20}, r3.Vec{X: 1, Z: 20})
	// The box's face at x = -0.0005 runs the whole length of the edge, and the
	// box is 1 mm thick in x: every point of the piece is within 0.001 of it.
	near := boxMesh(t, r3.Vec{X: 0.5, Y: 5, Z: 30}, spreadIdentity, r3.Vec{X: -0.5005, Z: 10})
	out, err := SplitSpread(t.Context(), []KeptFacet{piece}, curvedMesh(t), near, 0, nil)
	require.NoError(t, err)
	require.Len(t, out, 1)

	// With the face 0.01 mm away the clearance of delta plus the box's own bound
	// holds, and the piece splits.
	roomy := boxMesh(t, r3.Vec{X: 0.5, Y: 5, Z: 30}, spreadIdentity, r3.Vec{X: -0.51, Z: 10})
	out, err = SplitSpread(t.Context(), []KeptFacet{piece}, curvedMesh(t), roomy, 0, nil)
	require.NoError(t, err)
	require.Len(t, out, 3)
}

// A piece of the second operand is read against the first.
func TestSplitSpreadReadsEachPieceAgainstTheOtherOperand(t *testing.T) {
	t.Parallel()
	const delta = spreadDelta
	piece := spreadFacet([3]float64{0.05, delta, delta},
		r3.Vec{}, r3.Vec{Z: 20}, r3.Vec{X: 1, Z: 20})
	piece.Second = true
	near := boxMesh(t, r3.Vec{X: 0.5, Y: 5, Z: 30}, spreadIdentity, r3.Vec{X: -0.5005, Z: 10})
	out, err := SplitSpread(t.Context(), []KeptFacet{piece}, near, curvedMesh(t), 0, nil)
	require.NoError(t, err)
	require.Len(t, out, 1, "the second operand's piece is read against the first operand")
	out, err = SplitSpread(t.Context(), []KeptFacet{piece}, farMesh(t), curvedMesh(t), 0, nil)
	require.NoError(t, err)
	require.Len(t, out, 3)
}

// The charge function sees the work the clearance scans cost, and its error
// stops the split.
func TestSplitSpreadChargesItsWork(t *testing.T) {
	t.Parallel()
	const delta = spreadDelta
	piece := spreadFacet([3]float64{0.05, delta, delta},
		r3.Vec{}, r3.Vec{Z: 20}, r3.Vec{X: 1, Z: 20})
	work := uint64(0)
	_, err := SplitSpread(t.Context(), []KeptFacet{piece}, curvedMesh(t), farMesh(t),
		0, func(w, _ uint64) error { work += w; return nil })
	require.NoError(t, err)
	require.Positive(t, work)

	stop := errors.New("budget spent")
	_, err = SplitSpread(t.Context(), []KeptFacet{piece}, curvedMesh(t), farMesh(t),
		0, func(uint64, uint64) error { return stop })
	require.ErrorIs(t, err, stop)
}

// A nearer point that fails the clearance does not stop a farther one that
// passes: the other operand runs alongside only the near part of the edge.
func TestSplitSpreadMovesPastTheNearPartOfTheEdge(t *testing.T) {
	t.Parallel()
	const delta = spreadDelta
	piece := spreadFacet([3]float64{0.05, delta, delta},
		r3.Vec{}, r3.Vec{Z: 20}, r3.Vec{X: 1, Z: 20})
	alongside := boxMesh(t, r3.Vec{X: 0.5, Y: 5, Z: 3.75}, spreadIdentity, r3.Vec{X: -0.5005, Z: 3.75})
	out, err := SplitSpread(t.Context(), []KeptFacet{piece}, curvedMesh(t), alongside, 0, nil)
	require.NoError(t, err)
	require.Len(t, out, 3)
	// The split point stands on the edge, past the box's 7.5 mm.
	split := out[0].V[1].Vec()
	require.Equal(t, 0.0, split.X)
	require.Greater(t, split.Z, 7.5)
}
