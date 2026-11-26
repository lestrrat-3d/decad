package decad

import (
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file checks agent-docs/exact-chamfer-design.md: an R1 chamfer of a straight edge between two retained
// planes of a mesh Boolean result holds its feet exactly, so a chamfer whose feet are floats reads vertex bound
// 0 and adds nothing to the volume bound. It reads the mesh node's VertexBound, which no exported API
// publishes, so it lives in the root package.

// lipSocket is fixture S (design §7): a block x ∈ [−10, 10], y ∈ [0, 16], z ∈ [0, 10] cut from below by the
// prism through (−5, −6), (x4, −6), (x6, 6), (−5, 6) sketched on XZ and spanning y ∈ [−1, 17]. The different
// extrusion axes keep the Cut on the mesh path. Its walls are x = −5 (square) and the line from (5, 0). It also
// returns the block's start cap, the face the lips lie on.
func (sc *stripScene) lipSocket(t testing.TB, x4, x6 float64) (*Body, FeatureRef) {
	t.Helper()
	block := sc.rect(t, -10, 0, 10, 16, 10)
	lips := CapStart(block)
	s, err := sc.w.CreateSketch(sc.w.XZ())
	require.NoError(t, err)
	corners := [][2]float64{{-5, -6}, {x4, -6}, {x6, 6}, {-5, 6}}
	first := s.CreatePoint(corners[0][0], corners[0][1])
	s.Fix(first)
	prev := first
	for _, c := range corners[1:] {
		p := s.CreatePoint(c[0], c[1])
		s.CreateLine(prev, p)
		prev = p
	}
	s.CreateLine(prev, first)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	// Against the sketch normal the prism spans y ∈ [0, 18]; shifted by −1 it passes both ends of the block.
	tool, err := Extrude(sc.doc, s, s.Profiles()[0], Distance{D: units.Millimeters(18), Dir: Against}).Do(t.Context())
	require.NoError(t, err)
	move, err := r3.Translation(r3.Vec{Y: -1})
	require.NoError(t, err)
	tool, err = Transform(tool, move).Do(t.Context())
	require.NoError(t, err)
	socket, err := Cut(block, tool).Do(t.Context())
	require.NoError(t, err)
	return socket, lips
}

// lipEdges selects the edges along Y whose bottom ends are at (x, 0, 0) for each x.
func lipEdges(xs ...float64) EdgeSelector {
	var q *EdgeQuery
	for _, x := range xs {
		preds := []EdgePredicate{EndpointAt(r3.NewVec(x, 0, 0)), ParallelTo(r3.NewVec(0, 1, 0))}
		if q == nil {
			q = Edges(preds...)
			continue
		}
		q = q.Or(preds...)
	}
	return q.Exactly(len(xs))
}

// vertexBounds is the mesh node's per-vertex bounds and positions of b.
func vertexBounds(t *testing.T, b *Body) ([]r3.Vec, []float64) {
	t.Helper()
	mesh, err := b.Tessellate(t.Context(), units.Millimeters(0.1), WithVerification(VerifyNone))
	require.NoError(t, err)
	require.Len(t, mesh.n.VertexBound, len(mesh.n.Vertices))
	return mesh.n.Vertices, mesh.n.VertexBound
}

// requireZeroBounds requires every vertex of b to state bound 0.
func requireZeroBounds(t *testing.T, b *Body) {
	t.Helper()
	verts, bounds := vertexBounds(t, b)
	for i, bound := range bounds {
		require.Zero(t, bound, "vertex %d at %v", i, verts[i])
	}
}

// requireFootBound requires the bound of the vertex at p to be exactly zero (zero) or in (0, max].
func requireFootBound(t *testing.T, b *Body, p r3.Vec, zero bool, maxBound float64) {
	t.Helper()
	verts, bounds := vertexBounds(t, b)
	found := 0
	for i, v := range verts {
		if v.Sub(p).Len() > 1e-3 {
			continue
		}
		found++
		if zero {
			require.Zero(t, bounds[i], "vertex at %v", v)
			require.Equal(t, p, v, "a foot with bound 0 is the exact float")
			continue
		}
		require.Positive(t, bounds[i], "vertex at %v", v)
		require.LessOrEqual(t, bounds[i], maxBound, "vertex at %v", v)
	}
	require.Positive(t, found, "no vertex near %v", p)
}

// requireOnChamferPlane requires every vertex of the tilted lip's chamfer face to lie within its own bound of the
// exact chamfer plane, through the feet (5.5, y, 0) and (5 + 0.5/√37, y, 3/√37) computed at 512 bits.
func requireOnChamferPlane(t *testing.T, b *Body) {
	t.Helper()
	f := func(x float64) *big.Float { return new(big.Float).SetPrec(512).SetFloat64(x) }
	root := new(big.Float).SetPrec(512).Sqrt(f(37))
	a := new(big.Float).SetPrec(512).Quo(f(0.5), root)
	h := new(big.Float).SetPrec(512).Quo(f(3), root)
	// In (x, z) the plane runs from (5.5, 0) to (5 + a, h): normal (h, 0.5 − a).
	nz := new(big.Float).SetPrec(512).Sub(f(0.5), a)
	norm := new(big.Float).SetPrec(512).Sqrt(new(big.Float).SetPrec(512).Add(
		new(big.Float).SetPrec(512).Mul(h, h), new(big.Float).SetPrec(512).Mul(nz, nz)))
	verts, bounds := vertexBounds(t, b)
	checked := 0
	for i, v := range verts {
		onFace := false
		for _, y := range []float64{0, 16} {
			onFace = onFace || v.Sub(r3.NewVec(5.5, y, 0)).Len() < 1e-3 ||
				v.Sub(r3.NewVec(5+0.5/math.Sqrt(37), y, 3/math.Sqrt(37))).Len() < 1e-3
		}
		if !onFace {
			continue
		}
		checked++
		d := new(big.Float).SetPrec(512).Add(
			new(big.Float).SetPrec(512).Mul(h, new(big.Float).SetPrec(512).Sub(f(v.X), f(5.5))),
			new(big.Float).SetPrec(512).Mul(nz, f(v.Z)))
		d.Quo(d, norm).Abs(d)
		require.LessOrEqual(t, d.Cmp(f(bounds[i])), 0, "vertex %v is %v from the chamfer plane, bound %v", v, d, bounds[i])
	}
	require.Equal(t, 4, checked)
}

func TestRetainedLineChamferExact(t *testing.T) {
	t.Parallel()

	// chamfered cuts the socket, chamfers sel by d and returns the result with the volumes before and after.
	// options builds the chamfer's options from the lips' face feature.
	chamfered := func(t *testing.T, x4, x6 float64, sel EdgeSelector, d float64,
		options func(lips FeatureRef, c *ChamferCommand) *ChamferCommand) (out *Body, before, after Measurement) {
		t.Helper()
		sc := newStripScene()
		cut, lips := sc.lipSocket(t, x4, x6)
		requireZeroBounds(t, cut)
		cmd := Chamfer(cut, sel, units.Millimeters(d))
		if options != nil {
			cmd = options(lips, cmd)
		}
		var err error
		before, err = cut.Volume()
		require.NoError(t, err)
		out, err = cmd.Do(t.Context())
		require.NoError(t, err)
		after, err = out.Volume()
		require.NoError(t, err)
		return out, before, after
	}

	t.Run("square lip", func(t *testing.T) {
		t.Parallel()
		out, before, after := chamfered(t, 4, 6, lipEdges(-5), 0.5, nil)
		requireZeroBounds(t, out)
		require.Equal(t, before.Bound.Mag(), after.Bound.Mag())
		require.Equal(t, 2.0, before.Value.Mag()-after.Value.Mag())
	})

	t.Run("asymmetric square lips", func(t *testing.T) {
		t.Parallel()
		// Both walls square, so both lips are exact; the bottom face is the reference of each.
		out, before, after := chamfered(t, 5, 5, lipEdges(-5, 5), 0.5,
			func(lips FeatureRef, c *ChamferCommand) *ChamferCommand {
				return c.Asymmetric(Faces(FaceCreatedBy(lips)), units.Millimeters(0.25))
			})
		requireZeroBounds(t, out)
		require.Equal(t, before.Bound.Mag(), after.Bound.Mag())
		require.Equal(t, 2.0, before.Value.Mag()-after.Value.Mag())
	})

	t.Run("tilted lip", func(t *testing.T) {
		t.Parallel()
		out, before, after := chamfered(t, 4, 6, lipEdges(5), 0.5, nil)
		// The wall rises 6 over 1, so its foot lies 0.5/√37 in and 3/√37 up from the corner.
		root := math.Sqrt(37)
		for _, y := range []float64{0, 16} {
			// The feet on the lip's bottom face are floats; the ones on the wall are irrational.
			requireFootBound(t, out, r3.NewVec(5.5, y, 0), true, 0)
			requireFootBound(t, out, r3.NewVec(5+0.5/root, y, 3/root), false, 1e-14)
		}
		require.LessOrEqual(t, after.Bound.Mag()-before.Bound.Mag(), 1e-11)
		requireOnChamferPlane(t, out)
	})

	t.Run("steeper wall and a larger setback", func(t *testing.T) {
		t.Parallel()
		out, before, after := chamfered(t, 3.5, 6.5, lipEdges(-5), 1.0, nil)
		requireZeroBounds(t, out)
		require.Equal(t, before.Bound.Mag(), after.Bound.Mag())
		require.Equal(t, 8.0, before.Value.Mag()-after.Value.Mag())
	})

	t.Run("tilted exact planes", func(t *testing.T) {
		t.Parallel()
		// A block x ∈ [−20, 20], y ∈ [−10, 25], z ∈ [0, 10] with a square hole through (0, 0), (6, 8), (−2, 14),
		// (−8, 6): its walls run along (0.6, 0.8) and (−0.8, 0.6), whose float tags need not contain the corners.
		// The hole's corner at (0, 0) is concave, so its chamfer fills.
		sc := newStripScene()
		xz, err := sc.w.CreateSketch(sc.w.XZ())
		require.NoError(t, err)
		r := xz.CreateRectangle(-20, 0, 20, 10)
		xz.Fix(r.A)
		_, err = xz.Solve(t.Context())
		require.NoError(t, err)
		// Along the sketch normal the block spans y ∈ [−35, 0]; shifted by 25 it spans y ∈ [−10, 25].
		block, err := Extrude(sc.doc, xz, xz.Profiles()[0], Distance{D: units.Millimeters(35), Dir: Along}).Do(t.Context())
		require.NoError(t, err)
		move, err := r3.Translation(r3.Vec{Y: 25})
		require.NoError(t, err)
		block, err = Transform(block, move).Do(t.Context())
		require.NoError(t, err)
		xy, err := sc.w.CreateSketch(sc.w.XY())
		require.NoError(t, err)
		corners := [][2]float64{{0, 0}, {6, 8}, {-2, 14}, {-8, 6}}
		first := xy.CreatePoint(corners[0][0], corners[0][1])
		xy.Fix(first)
		prev := first
		for _, c := range corners[1:] {
			p := xy.CreatePoint(c[0], c[1])
			xy.CreateLine(prev, p)
			prev = p
		}
		xy.CreateLine(prev, first)
		_, err = xy.Solve(t.Context())
		require.NoError(t, err)
		// Along +Z the prism spans z ∈ [0, 12]; shifted down by 1 it passes both faces of the block.
		tool, err := Extrude(sc.doc, xy, xy.Profiles()[0], Distance{D: units.Millimeters(12), Dir: Along}).Do(t.Context())
		require.NoError(t, err)
		down, err := r3.Translation(r3.Vec{Z: -1})
		require.NoError(t, err)
		tool, err = Transform(tool, down).Do(t.Context())
		require.NoError(t, err)
		cut, err := Cut(block, tool).Do(t.Context())
		require.NoError(t, err)
		before, err := cut.Volume()
		require.NoError(t, err)
		sel := Edges(EndpointAt(r3.NewVec(0, 0, 0)), ParallelTo(r3.NewVec(0, 0, 1))).Exactly(1)
		out, err := Chamfer(cut, sel, units.Millimeters(1.25)).Do(t.Context())
		require.NoError(t, err)
		after, err := out.Volume()
		require.NoError(t, err)
		for _, z := range []float64{0, 10} {
			requireFootBound(t, out, r3.NewVec(0.75, 1, z), true, 0)
			requireFootBound(t, out, r3.NewVec(-1, 0.75, z), true, 0)
		}
		// The Cut's rational wall corners are not floats and keep bounds near 1e-15, so the tilted walls' facets
		// charge their tag distance (about 1.6e-14 and 2.2e-14 on the two walls) and the bound grows by 3.6e-14,
		// not by 0. Only a Cut with all-float corners reads the slab term's corner minimum end to end; the unit
		// cases of TestPlaneFacetVolumeCornerBounds cover it.
		require.LessOrEqual(t, after.Bound.Mag()-before.Bound.Mag(), 1e-13)
		require.Equal(t, 7.8125, after.Value.Mag()-before.Value.Mag())
	})
}
