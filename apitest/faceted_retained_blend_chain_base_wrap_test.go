package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// The base-wrap chain-end fixtures of agent-docs/retained-agent-docs/modify-design.md Part R
// §5.5 step 5 and §12: a concave open tangent chain on a plate's top whose
// ends lie on the sharp vertical corners of a block joined on top. There the
// plate's top wraps the corner (it is reflex), the block's wall is convex, and
// the end face G is the block's other side face, convex at the corner. The
// base keeps the corner and the blend's end section lies in G's plane outside
// G's held face, so G takes it.

// wrapR is the radius of the block's rounded vertical corners.
const wrapR = 2.0

// wrapBlock is a 30x26x10 plate with a block over x = 10 … 20, y = 10 … 16 and
// height 6 joined on top (z = 10 … 16; it starts 1 inside the plate). corners names which of the block's
// x = 20 vertical edges are rounded at wrapR: 0 none, 1 the y = 10 one, 2 both.
func wrapBlock(t *testing.T, corners int) *decad.Body {
	t.Helper()
	return wrapBlockIn(t, sketch.NewWorld(), decad.New(), corners)
}

// wrapBlockIn is wrapBlock built in w and doc.
func wrapBlockIn(t *testing.T, w *sketch.World, doc *decad.Document, corners int) *decad.Body {
	t.Helper()
	plate := rectBox(t, w, doc, 0, 0, 30, 26, 10)
	block := rectBox(t, w, doc, 10, 10, 20, 16, 7)
	if corners > 0 {
		up := decad.ParallelTo(r3.NewVec(0, 0, 1))
		sel := decad.Edges(up, decad.EndpointAt(r3.NewVec(20, 10, 0)))
		if corners == 2 {
			sel = sel.Or(up, decad.EndpointAt(r3.NewVec(20, 16, 0)))
		}
		var err error
		block, err = decad.Fillet(t.Context(), block, sel.Exactly(corners), units.Millimeters(wrapR))
		require.NoError(t, err)
	}
	out, err := decad.Union(t.Context(), plate, placed(t, block, 9))
	require.NoError(t, err)
	return out
}

// wrapBlockWithCube is wrapBlock(t, corners) joined with a 0.1 thick box over
// x = x0 … x1, y = y0 … y0 + 0.1, z = 10.05 … 10.15, floating above the plate
// beside the block's y = 10 face.
func wrapBlockWithCube(t *testing.T, corners int, x0, x1, y0 float64) *decad.Body {
	t.Helper()
	w, doc := sketch.NewWorld(), decad.New()
	out, err := decad.Union(t.Context(), wrapBlockIn(t, w, doc, corners), placed(t, rectBox(t, w, doc, x0, y0, x1, y0+0.1, 0.1), 10.05))
	require.NoError(t, err)
	return out
}

// wrapRoot selects the block's root edge along x at y = 10 that starts at the
// sharp corner (10,7,10).
func wrapRoot() decad.EdgeSelector {
	return decad.Edges(decad.Concave(), decad.ParallelTo(r3.NewVec(1, 0, 0)), decad.EndpointAt(r3.NewVec(10, 10, 10))).Exactly(1)
}

// wrapFill is the volume a fillet of radius r adds along a chain of lines
// summing to lines and arcs, each a quarter turn of a vertical rim of radius
// wrapR, whose section's centroid lies r·x̄ outside the rim's circle.
func wrapFill(r, lines float64, quarterArcs int) float64 {
	xbar := r * (10 - 3*math.Pi) / (3 * (4 - math.Pi))
	af := r * r * (1 - math.Pi/4)
	return af * (lines + float64(quarterArcs)*math.Pi/2*(wrapR+xbar))
}

func TestFacetedRetainedBlendChainBaseWrap(t *testing.T) {
	t.Parallel()

	t.Run("a block's root through two rounded corners fillets", func(t *testing.T) {
		t.Parallel()
		joined := wrapBlock(t, 2)
		before, err := joined.Volume()
		require.NoError(t, err)
		const r = 1.0
		out, err := decad.Fillet(t.Context(), joined, wrapRoot(), units.Millimeters(r), decad.WithTangentChain())
		require.NoError(t, err)
		after, err := out.Volume()
		require.NoError(t, err)
		// Lines 8 + 2 + 8, two quarter arcs.
		requireVolumeChange(t, before, after, -wrapFill(r, 18, 2))
		requireFaceAreaChange(t, joined, out, 10, 2*r*r*(1-math.Pi/4))
		requireCapSide(t, out, 10, true, r3.NewVec(10, 9.95, 10.05), r3.NewVec(10, 16.05, 10.05))
		cylinders, tori, majors := blendKinds(out, r)
		require.Equal(t, 3, cylinders)
		require.Equal(t, 2, tori)
		for _, m := range majors {
			require.InDelta(t, wrapR+r, m, 1e-12)
		}
		requireBlendsFaceVoid(t, out, 5)
		requireVerifiedMesh(t, out)
	})

	t.Run("a chamfer of the same root", func(t *testing.T) {
		t.Parallel()
		joined := wrapBlock(t, 2)
		before, err := joined.Volume()
		require.NoError(t, err)
		out, err := decad.Chamfer(t.Context(), joined, wrapRoot(), units.Millimeters(1), decad.WithTangentChain())
		require.NoError(t, err)
		after, err := out.Volume()
		require.NoError(t, err)
		// The chamfer's section is a right triangle of area 1/2 whose centroid
		// lies 1/3 outside the rim's circle.
		requireVolumeChange(t, before, after, -(18+math.Pi*(wrapR+1.0/3))/2)
		requireFaceAreaChange(t, joined, out, 10, 2*0.5)
		requireCapSide(t, out, 10, true, r3.NewVec(10, 9.95, 10.05), r3.NewVec(10, 16.05, 10.05))
		requireVerifiedMesh(t, out)
	})

	t.Run("a root through one rounded corner ends on two different faces", func(t *testing.T) {
		t.Parallel()
		joined := wrapBlock(t, 1)
		before, err := joined.Volume()
		require.NoError(t, err)
		const r = 1.0
		out, err := decad.Fillet(t.Context(), joined, wrapRoot(), units.Millimeters(r), decad.WithTangentChain())
		require.NoError(t, err)
		after, err := out.Volume()
		require.NoError(t, err)
		// Lines 8 and 4, one quarter arc.
		requireVolumeChange(t, before, after, -wrapFill(r, 12, 1))
		requireFaceAreaChange(t, joined, out, 10, r*r*(1-math.Pi/4))
		requireVerifiedMesh(t, out)
	})

	t.Run("a solid crossing the cap the end face takes is refused", func(t *testing.T) {
		t.Parallel()
		// The box crosses x = 10 inside the cap at the first corner and misses
		// the strip's other parts.
		_, err := decad.Fillet(t.Context(), wrapBlockWithCube(t, 2, 9, 10.05, 9.85), wrapRoot(), units.Millimeters(1),
			decad.WithTangentChain())
		require.ErrorIs(t, err, decad.ErrUnsupported)
		require.ErrorContains(t, err, "the cap an end face takes meets another facet of the body at")
	})

	t.Run("a second block standing in the base's setback band is refused", func(t *testing.T) {
		t.Parallel()
		w, doc := sketch.NewWorld(), decad.New()
		joined, err := decad.Union(t.Context(), wrapBlockIn(t, w, doc, 2),
			placed(t, rectBox(t, w, doc, 9.5, 8.5, 10.5, 9.5, 5), 9))
		require.NoError(t, err)
		_, err = decad.Fillet(t.Context(), joined, wrapRoot(), units.Millimeters(1), decad.WithTangentChain())
		require.ErrorIs(t, err, decad.ErrDegenerate)
		require.ErrorContains(t, err, "the blend consumes the base face")
	})

	t.Run("a solid clear of the cap the end face takes is kept", func(t *testing.T) {
		t.Parallel()
		joined := wrapBlockWithCube(t, 2, 8, 9, 8.5)
		before, err := joined.Volume()
		require.NoError(t, err)
		const r = 1.0
		out, err := decad.Fillet(t.Context(), joined, wrapRoot(), units.Millimeters(r), decad.WithTangentChain())
		require.NoError(t, err)
		after, err := out.Volume()
		require.NoError(t, err)
		requireVolumeChange(t, before, after, -wrapFill(r, 18, 2))
		requireVerifiedMesh(t, out)
	})

	// With sharp corners the single-edge route ends on the same corners and
	// gains the same caps; the strip test alone does not cover them.
	t.Run("a solid crossing a cap the single-edge fillet gains is refused", func(t *testing.T) {
		t.Parallel()
		for _, c := range []struct {
			name   string
			x0, x1 float64
			at     string
		}{
			{"first end", 9, 10.05, `\{10 `},
			{"far end", 19.95, 21, `\{20 `},
		} {
			t.Run(c.name, func(t *testing.T) {
				t.Parallel()
				_, err := decad.Fillet(t.Context(), wrapBlockWithCube(t, 0, c.x0, c.x1, 9.85), wrapRoot(),
					units.Millimeters(1))
				require.ErrorIs(t, err, decad.ErrUnsupported)
				require.Regexp(t, `the cap an end face takes meets another facet of the body at `+c.at, err.Error())
			})
		}
	})

	t.Run("a solid clear of a cap the single-edge fillet gains is kept", func(t *testing.T) {
		t.Parallel()
		joined := wrapBlockWithCube(t, 0, 8, 9, 8.5)
		before, err := joined.Volume()
		require.NoError(t, err)
		const r = 1.0
		out, err := decad.Fillet(t.Context(), joined, wrapRoot(), units.Millimeters(r))
		require.NoError(t, err)
		after, err := out.Volume()
		require.NoError(t, err)
		// The fill's section area times the root edge's length.
		requireVolumeChange(t, before, after, -10*r*r*(1-math.Pi/4))
		requireVerifiedMesh(t, out)
		requireNoCrossingTriangles(t, out)
	})
}
