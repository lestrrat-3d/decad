package decad

import (
	"context"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/booleaneval"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

type blendScene struct {
	w   *sketch.World
	doc *Document
}

func (sc *blendScene) plate(ctx context.Context, t *testing.T) *Body {
	t.Helper()
	s, err := sc.w.CreateSketch(sc.w.XY())
	require.NoError(t, err)
	r := s.CreateRectangle(0, 0, 40, 20)
	s.Fix(r.A)
	_, err = s.Solve(ctx)
	require.NoError(t, err)
	plate, err := Extrude(ctx, sc.doc, s, s.Profiles()[0],
		Distance{D: units.Millimeters(20), Dir: Along})
	require.NoError(t, err)
	return plate
}

// zTool is a through cylinder along z at (cx, cy), placed by a translation so
// a Boolean takes the mesh path.
func (sc *blendScene) zTool(ctx context.Context, t *testing.T, cx, cy, r float64) *Body {
	t.Helper()
	s, err := sc.w.CreateSketch(sc.w.XY())
	require.NoError(t, err)
	c := s.CreatePoint(cx, cy)
	s.Fix(c)
	s.CreateCircle(c, r)
	_, err = s.Solve(ctx)
	require.NoError(t, err)
	pin, err := Extrude(ctx, sc.doc, s, s.Profiles()[0],
		Distance{D: units.Millimeters(60), Dir: Along})
	require.NoError(t, err)
	move, err := r3.Translation(r3.Vec{Z: -29.999})
	require.NoError(t, err)
	placed, err := Transform(ctx, pin, move)
	require.NoError(t, err)
	return placed
}

func zRimEdge(t *testing.T, b *Body, z float64) EdgeSelector {
	t.Helper()
	for _, e := range b.Edges() {
		c, ok := e.Curve().(Circle3)
		if !ok || math.Abs(c.Center.Z-z) > 1e-6 {
			continue
		}
		return Edges(EndpointAt(e.Start().Position().Value)).Exactly(1)
	}
	require.Fail(t, "no rim at that height")
	return nil
}

// blendRun bores the plate, applies blend to the bored plate, then cuts a z
// tool at (cx, cy, r) in a fresh document under ctx. It returns the bored
// plate, the blended body and the cut result.
func blendRun(ctx context.Context, t *testing.T, blend func(*Body) (*Body, error),
	cx, cy, r float64) (bored, blended, out *Body, err error) {
	t.Helper()
	sc := &blendScene{w: sketch.NewWorld(), doc: New()}
	bored, err = Cut(ctx, sc.plate(ctx, t), sc.zTool(ctx, t, 20, 10, 4))
	require.NoError(t, err)
	blended, err = blend(bored)
	require.NoError(t, err)
	out, err = Cut(ctx, blended, sc.zTool(ctx, t, cx, cy, r))
	return bored, blended, out, err
}

// planeFace returns the one Plane face of b whose frame origin satisfies at.
func planeFace(t *testing.T, b *Body, at func(r3.Vec) bool) *topo.Face {
	t.Helper()
	var found *topo.Face
	for _, f := range b.Faces() {
		n := faceNode(f)
		pl, ok := n.Surface.(surfacegeom.Plane)
		if !ok || !at(pl.Frame.Origin()) {
			continue
		}
		require.Nil(t, found, "one plane face matches")
		found = n
	}
	require.NotNil(t, found, "a plane face matches")
	return found
}

// requireSamePositions checks that two runs placed every vertex at the same
// coordinates: the slab changes bounds and gates only (retention §11). It
// reports the first differing index only.
func requireSamePositions(t *testing.T, a, b *Body) {
	t.Helper()
	va, vb := a.Vertices(), b.Vertices()
	require.Len(t, vb, len(va))
	for i := range va {
		if va[i].Position().Value != vb[i].Position().Value {
			require.Failf(t, "vertex positions differ", "first at index %d", i)
		}
	}
}

// rimBounds returns, per vertex index, the bound of each vertex that lies on
// the plane through p with unit normal n and within 0.05 mm of the circle of
// radius r about the axis through (cx, cy), and requires each to lie within its
// bound of that circle. Other indexes hold a negative value.
func rimBounds(t *testing.T, b *Body, p, n r3.Vec, cx, cy, r float64) []float64 {
	t.Helper()
	vs := b.Vertices()
	out := make([]float64, len(vs))
	checked := 0
	for i, v := range vs {
		out[i] = -1
		pos := v.Position()
		if math.Abs(pos.Value.Sub(p).Dot(n)) > 1e-9 {
			continue
		}
		off := math.Abs(math.Hypot(pos.Value.X-cx, pos.Value.Y-cy) - r)
		if off > 0.05 {
			continue
		}
		require.LessOrEqual(t, off, pos.Bound.Mag()+1e-9, "vertex %d", i)
		out[i] = pos.Bound.Mag()
		checked++
	}
	require.Positive(t, checked, "the new wall leaves rim vertices on the plane")
	return out
}

// requireTighter checks that every rim vertex the real run bounds is bounded
// strictly below the control's bound for the same vertex.
func requireTighter(t *testing.T, got, control []float64) {
	t.Helper()
	require.Len(t, control, len(got))
	for i := range got {
		if got[i] < 0 {
			continue
		}
		if !(got[i] < control[i]) {
			require.Failf(t, "rim bound not tighter", "first at index %d", i)
		}
	}
}

func requireRemoved(t *testing.T, before, after *Body, want float64) {
	t.Helper()
	vb, err := before.Volume()
	require.NoError(t, err)
	va, err := after.Volume()
	require.NoError(t, err)
	require.InDelta(t, want, vb.Value.Mag()-va.Value.Mag(), vb.Bound.Mag()+va.Bound.Mag()+1e-6)
}

// TestFacetedSourcePatchBlendThenCut shows that a retained plane keeps its slab
// through a retained blend, then reaches a later Cut. Unlike the Cut-then-Cut
// scene, a one-level chain does not trip the control's chain-depth gate, so
// both runs build; the slab shows as strictly tighter rim bounds at identical
// positions.
func TestFacetedSourcePatchBlendThenCut(t *testing.T) {
	t.Parallel()

	control := func() context.Context { return booleaneval.WithoutRetainedPlaneSlab(t.Context()) }
	topAt := func(p r3.Vec) bool { return p.Z == 20 && p.X == 0 && p.Y == 0 }

	t.Run("mouth fillet then cut beside the mouth", func(t *testing.T) {
		t.Parallel()
		fillet := func(b *Body) (*Body, error) {
			return Fillet(t.Context(), b, zRimEdge(t, b, 20), units.Millimeters(1))
		}
		const cx, cy, r = 32.0, 10.0, 2.0
		bored, blended, out, err := blendRun(t.Context(), t, fillet, cx, cy, r)
		require.NoError(t, err)

		boredTop := planeFace(t, bored, topAt)
		top := planeFace(t, blended, topAt)
		require.True(t, top.RetainedTag)
		require.Zero(t, top.NormalBound)
		require.True(t, boredTop.Surface.(surfacegeom.Plane).Frame == top.Surface.(surfacegeom.Plane).Frame,
			"the blend leaves the top frame bit-equal")

		_, _, ctl, err := blendRun(control(), t, fillet, cx, cy, r)
		require.NoError(t, err, "the one-level chain does not reach the chain-depth gate")
		requireSamePositions(t, out, ctl)

		n := r3.NewVec(0, 0, 1)
		p := r3.NewVec(0, 0, 20)
		requireTighter(t, rimBounds(t, out, p, n, cx, cy, r), rimBounds(t, ctl, p, n, cx, cy, r))
		requireRemoved(t, blended, out, math.Pi*r*r*20)
	})

	t.Run("cut through the torus rim takes no slab there", func(t *testing.T) {
		t.Parallel()
		fillet := func(b *Body) (*Body, error) {
			return Fillet(t.Context(), b, zRimEdge(t, b, 20), units.Millimeters(1))
		}
		// The tool lies within 5 mm of the bore axis, so it crosses the torus
		// (radius 4 to 5 at the top) and never the top plane. It runs through
		// the bottom plane too, which keeps its slab.
		const cx, cy, r = 24.6, 10.0, 0.3
		_, _, out, err := blendRun(t.Context(), t, fillet, cx, cy, r)
		require.NoError(t, err)
		_, _, ctl, err := blendRun(control(), t, fillet, cx, cy, r)
		require.NoError(t, err)
		requireSamePositions(t, out, ctl)
		va, vb := out.Vertices(), ctl.Vertices()
		torus, bottom := 0, 0
		for i := range va {
			z := va[i].Position().Value.Z
			ba, bc := va[i].Position().Bound, vb[i].Position().Bound
			switch {
			case z > 19+1e-9:
				torus++
				if ba != bc {
					require.Failf(t, "a curved group took a slab", "first at index %d", i)
				}
			case math.Abs(z) < 1e-9 && ba != bc:
				bottom++
				require.Less(t, ba.Mag(), bc.Mag(), "vertex %d", i)
			}
		}
		require.Positive(t, torus, "the tool leaves vertices on the torus")
		require.Positive(t, bottom, "the bottom plane keeps its slab")
	})

	t.Run("chamfer then cut across the chamfer face", func(t *testing.T) {
		t.Parallel()
		chamfer := func(b *Body) (*Body, error) {
			sel := Edges(ParallelTo(r3.NewVec(0, 1, 0)), EndpointAt(r3.NewVec(0, 0, 20))).Exactly(1)
			return Chamfer(t.Context(), b, sel, units.Millimeters(1))
		}
		const cx, cy, r = 1.5, 8.0, 1.0
		_, blended, out, err := blendRun(t.Context(), t, chamfer, cx, cy, r)
		require.NoError(t, err)

		// The chamfer plane runs through (0, y, 19) and (1, y, 20).
		cham := planeFace(t, blended, func(p r3.Vec) bool {
			return math.Abs(p.Z-p.X-19) < 1e-9
		})
		require.True(t, cham.RetainedTag)
		require.Zero(t, cham.NormalBound)

		_, _, ctl, err := blendRun(control(), t, chamfer, cx, cy, r)
		require.NoError(t, err)
		requireSamePositions(t, out, ctl)

		s := math.Sqrt2 / 2
		n := r3.NewVec(-s, 0, s)
		p := r3.NewVec(0, 0, 19)
		requireTighter(t, rimBounds(t, out, p, n, cx, cy, r), rimBounds(t, ctl, p, n, cx, cy, r))

		// Material inside the tool: 19 mm of column plus the part of the 1 mm
		// wedge under z = 19 + x, the integral of min(x, 1) over the disc.
		// With u = x - cx, that is cx*pi*r^2 minus the integral of (u+0.5)
		// over u > -0.5, whose chord is 2*sqrt(1-u^2).
		F := func(u float64) float64 { // antiderivative of (u+0.5)*2*sqrt(1-u^2)
			return -2.0/3*math.Pow(1-u*u, 1.5) + 0.5*(u*math.Sqrt(1-u*u)+math.Asin(u))
		}
		tail := F(1) - F(-0.5)
		want := 19*math.Pi + cx*math.Pi - tail
		requireRemoved(t, blended, out, want)
	})
}
