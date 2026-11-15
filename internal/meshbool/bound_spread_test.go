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
		0, func(w, _, _ uint64) error { work += w; return nil })
	require.NoError(t, err)
	require.Positive(t, work)

	stop := errors.New("budget spent")
	_, err = SplitSpread(t.Context(), []KeptFacet{piece}, curvedMesh(t), farMesh(t),
		0, func(uint64, uint64, uint64) error { return stop })
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

// spreadWedge is a convex wedge below the plane z = 0 whose top is
// z = -a*Y - b*|X|, a ridge along X = 0. It crosses z = 0 along the V
// Y = -(b/a)|X| with its apex at the origin, so the region Y > 0 of the plane
// lies outside it. The distance from a plane point (X, Y) with Y > 0 to the
// wedge grows like a*Y + b*|X|, so it is not affine along a segment that
// crosses X = 0: it dips between two points that both clear the operand.
func spreadWedge(t *testing.T, a, b, beta float64) *BoolMesh {
	t.Helper()
	top := func(x, y float64) r3.Vec { return r3.Vec{X: x, Y: y, Z: -a*y - b*max(x, -x)} }
	verts := []r3.Vec{
		top(-1, -1), top(0, -1), top(1, -1),
		top(-1, 1), top(0, 1), top(1, 1),
		{X: -1, Y: -1, Z: -1}, {X: 1, Y: -1, Z: -1}, {X: 1, Y: 1, Z: -1}, {X: -1, Y: 1, Z: -1},
	}
	tris := [][3]int{
		{0, 1, 4}, {0, 4, 3},
		{1, 2, 5}, {1, 5, 4},
		{6, 9, 8}, {6, 8, 7},
		{6, 7, 2}, {6, 2, 1}, {6, 1, 0},
		{9, 3, 4}, {9, 4, 5}, {9, 5, 8},
		{6, 0, 3}, {6, 3, 9},
		{7, 8, 5}, {7, 5, 2},
	}
	src := []int{0, 0, 1, 1, 2, 2, 3, 3, 3, 4, 4, 4, 5, 5, 6, 6}
	m, err := PrepContext(t.Context(), verts, tris, src)
	require.NoError(t, err)
	bounds := make([]float64, len(verts))
	for i := range bounds {
		bounds[i] = beta
	}
	m.SetVertexBounds(bounds)
	return m
}

// spreadWedgePiece is a piece in the plane z = 0 with its rim corner at the
// wedge's apex and the far corners 20 mm out.
func spreadWedgePiece() KeptFacet {
	return spreadFacet([3]float64{0.074, spreadDelta, spreadDelta},
		r3.Vec{}, r3.Vec{X: 20, Y: 1}, r3.Vec{X: -20, Y: 1})
}

// Both split points of the piece clear the other operand, but the far piece
// between them reaches within the clearance of it: a point on the segment
// between the split points is 0.0004 mm from the wedge's held boundary, and the
// clearance is 0.002 mm. The far piece's facet bound would be unproven there,
// so the nearest split is not emitted. Any split SplitSpread does emit stands
// farther out, and every sampled point of every far piece it makes clears.
func TestSplitSpreadProvesTheFarPieceNotItsSplitPoints(t *testing.T) {
	t.Parallel()
	const clearance = 2 * spreadDelta
	other := spreadWedge(t, 0.025, 0.01, spreadDelta)
	piece := spreadWedgePiece()

	// Both nearest split points pass the point test alone.
	nearest := []r3.Vec{{X: 0.3125, Y: 0.015625}, {X: -0.3125, Y: 0.015625}}
	hint := -1
	for _, split := range nearest {
		require.True(t, depthBeyond(proof.XptOf(split), other, clearance, &hint), "split point %v", split)
	}
	// The point between them does not.
	dip := proof.XptOf(r3.Vec{Y: 0.015625})
	require.Less(t, CertifiedInteriorDepth(dip, other), clearance)

	out, err := SplitSpread(t.Context(), []KeptFacet{piece}, curvedMesh(t), other, 0, nil)
	require.NoError(t, err)
	for _, f := range out {
		for _, v := range f.V {
			require.NotContains(t, nearest, v.Vec(), "the nearest split point is not used")
		}
		if max(f.Beta[0], f.Beta[1], f.Beta[2]) > spreadDelta {
			continue
		}
		const n = 60
		c := [3]r3.Vec{f.V[0].Vec(), f.V[1].Vec(), f.V[2].Vec()}
		for i := 0; i <= n; i++ {
			for j := 0; i+j <= n; j++ {
				l0, l1 := float64(i)/n, float64(j)/n
				p := c[0].Scale(l0).Add(c[1].Scale(l1)).Add(c[2].Scale(1 - l0 - l1))
				require.Greater(t, CertifiedInteriorDepth(proof.XptOf(p), other), clearance,
					"a far piece reaches within the clearance at %v", p)
			}
		}
	}
}

// A wedge no split can clear leaves the piece whole: the wedge's ridge runs
// through every far piece the piece can make.
func TestSplitSpreadKeepsAPieceWhenNoFarPieceClears(t *testing.T) {
	t.Parallel()
	// The ridge z = -0.025 Y - 0.01|X| is shallow, and the piece's far corners
	// lie on the wedge's top face, 0.0005 mm above its held boundary.
	other := spreadWedge(t, 0.025, 0.01, spreadDelta)
	piece := spreadFacet([3]float64{0.074, spreadDelta, spreadDelta},
		r3.Vec{}, r3.Vec{X: 0.5, Y: 0.5, Z: -0.0005 - 0.025*0.5 - 0.01*0.5},
		r3.Vec{X: -0.5, Y: 0.5, Z: -0.0005 - 0.025*0.5 - 0.01*0.5})
	out, err := SplitSpread(t.Context(), []KeptFacet{piece}, curvedMesh(t), other, 0, nil)
	require.NoError(t, err)
	require.Len(t, out, 1)
	require.Equal(t, piece.V, out[0].V)
	require.Equal(t, piece.Beta, out[0].Beta)
}

// A piece whose far corner is a chain corner, a vertex on the other operand's
// boundary that carries only the operand facet's bound, never splits: the far
// piece holding it touches the boundary and no distance above the clearance
// can be proven for it, although every split point stands deep inside the
// other operand.
func TestSplitSpreadKeepsAPieceHoldingALowBoundChainCorner(t *testing.T) {
	t.Parallel()
	// The box spans z in [0, 20] and is 100 mm across: the piece's rim corner is
	// 5 mm inside it, and its other two corners lie on its top face.
	other := boxMesh(t, r3.Vec{X: 50, Y: 50, Z: 10}, spreadIdentity, r3.Vec{Z: 10})
	piece := spreadFacet([3]float64{0.05, spreadDelta, spreadDelta},
		r3.Vec{Z: 5}, r3.Vec{Z: 20}, r3.Vec{X: 1, Z: 20})
	hint := -1
	for _, split := range []r3.Vec{{Z: 5 + 15.0/64}, {X: 1.0 / 64, Z: 5 + 15.0/64}} {
		require.True(t, depthBeyond(proof.XptOf(split), other, 2*spreadDelta, &hint), "split point %v", split)
	}

	out, err := SplitSpread(t.Context(), []KeptFacet{piece}, curvedMesh(t), other, 0, nil)
	require.NoError(t, err)
	require.Len(t, out, 1)
	require.Equal(t, piece.V, out[0].V)
	require.Equal(t, piece.Beta, out[0].Beta)
}

// The replay charge covers the output storage: the facets the output slice
// holds and the one it grows to are reserved before they are allocated, and a
// refusal at either stops the split.
func TestSplitSpreadReservesItsOutputStorage(t *testing.T) {
	t.Parallel()
	one := spreadFacet([3]float64{0.05, spreadDelta, spreadDelta},
		r3.Vec{}, r3.Vec{Z: 20}, r3.Vec{X: 1, Z: 20})
	two := spreadFacet([3]float64{0.05, spreadDelta, spreadDelta},
		r3.Vec{X: 5}, r3.Vec{X: 5, Z: 20}, r3.Vec{X: 6, Z: 20})
	var slots []uint64
	out, err := SplitSpread(t.Context(), []KeptFacet{one, two}, curvedMesh(t), farMesh(t), 0,
		func(_, _, facets uint64) error {
			if facets != 0 {
				slots = append(slots, facets)
			}
			return nil
		})
	require.NoError(t, err)
	require.Len(t, out, 6)
	// 2 + 2/4 slots fit the first split's three pieces only, and the second split grows.
	require.Equal(t, []uint64{3, 6}, slots)

	stop := errors.New("storage spent")
	for refuse := range 2 {
		seen := 0
		_, err = SplitSpread(t.Context(), []KeptFacet{one, two}, curvedMesh(t), farMesh(t), 0,
			func(_, _, facets uint64) error {
				if facets == 0 {
					return nil
				}
				seen++
				if seen > refuse {
					return stop
				}
				return nil
			})
		require.ErrorIs(t, err, stop)
	}
}
