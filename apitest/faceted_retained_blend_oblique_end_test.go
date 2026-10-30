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

// requireEndCurveCovered samples the true end curve of a quarter-turn fillet,
// at(α) for α in [0, π/2], where the blend's rows end on a plane oblique to
// the edge, and requires every sample within the mesh's published Bound of
// the held facets (agent-docs/retained-edge-blend-design.md §6.3). The curve
// is the ellipse the blend's cylinder cuts in that plane, and the facets reach
// it only through the chords between the rows' vertices there. A sample
// farther than Bound disproves the bound; samples within it prove nothing.
func requireEndCurveCovered(t *testing.T, out *decad.Body, at func(alpha float64) r3.Vec) {
	t.Helper()
	mesh, err := out.Tessellate(t.Context(), units.Millimeters(0.1), decad.WithVerification(decad.VerifyNone))
	require.NoError(t, err)
	const n = 1000
	samples := make([]r3.Vec, n+1)
	lo, hi := r3.NewVec(math.Inf(1), math.Inf(1), math.Inf(1)), r3.NewVec(math.Inf(-1), math.Inf(-1), math.Inf(-1))
	for k := range samples {
		p := at(math.Pi / 2 * float64(k) / n)
		samples[k] = p
		lo = r3.NewVec(math.Min(lo.X, p.X), math.Min(lo.Y, p.Y), math.Min(lo.Z, p.Z))
		hi = r3.NewVec(math.Max(hi.X, p.X), math.Max(hi.Y, p.Y), math.Max(hi.Z, p.Z))
	}
	// Only a facet whose box comes near the curve's box can be nearest to a
	// sample within the bound.
	reach := 10 * mesh.Bound().Base()
	verts := mesh.Vertices()
	var near [][3]r3.Vec
	for _, tri := range mesh.Triangles() {
		a, b, c := verts[tri[0]], verts[tri[1]], verts[tri[2]]
		if math.Max(a.X, math.Max(b.X, c.X)) < lo.X-reach || math.Min(a.X, math.Min(b.X, c.X)) > hi.X+reach ||
			math.Max(a.Y, math.Max(b.Y, c.Y)) < lo.Y-reach || math.Min(a.Y, math.Min(b.Y, c.Y)) > hi.Y+reach ||
			math.Max(a.Z, math.Max(b.Z, c.Z)) < lo.Z-reach || math.Min(a.Z, math.Min(b.Z, c.Z)) > hi.Z+reach {
			continue
		}
		near = append(near, [3]r3.Vec{a, b, c})
	}
	worst := 0.0
	for _, p := range samples {
		best := math.Inf(1)
		for _, f := range near {
			best = math.Min(best, pointTriangleDistance(p, f[0], f[1], f[2]))
		}
		worst = math.Max(worst, best)
	}
	t.Logf("end curve within %.6g mm of the mesh, Bound %.6g mm, %d facets", worst, mesh.Bound().Base(),
		len(mesh.Triangles()))
	require.LessOrEqual(t, worst, mesh.Bound().Base())
}

// TestFacetedRetainedBlendObliqueEnd fillets straight edges whose rows end on
// planes oblique to them: an end face, the mitre between two edges of a run
// (§4.7), and the mitre of the corner route (§4.8). In each plane a row
// chord's sagitta is the cross-section's over |d·n|; the rows are spaced for
// that plane, so the true end curve stays within the published bound.
func TestFacetedRetainedBlendObliqueEnd(t *testing.T) {
	t.Parallel()
	x, y, z := r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0), r3.NewVec(0, 0, 1)

	// The plate's top front edge along x, at y = 0 and z = 20, ends on the
	// tool's face x − y = 5, at 45° to it. The ball's centre is (y, z) =
	// (r, 20 − r).
	t.Run("an end face at 45 degrees", func(t *testing.T) {
		t.Parallel()
		const r = 2.0
		cut := wedgedPlate(t, func(sc *retentionScene) *sketch.Plane { return sc.w.XY() },
			[][2]float64{{-60, -30}, {-25, -30}, {35, 30}, {-60, 30}}, 80, r3.Vec{Z: 0.5})
		out, err := decad.Fillet(t.Context(), cut, straightEdge(t, cut, r3.NewVec(40, 0, 20), x),
			units.Millimeters(r))
		require.NoError(t, err)
		requireEndCurveCovered(t, out, func(alpha float64) r3.Vec {
			py := r - r*math.Cos(alpha)
			return r3.NewVec(5+py, py, 20-r+r*math.Sin(alpha))
		})
	})

	// Two top edges of a block meeting at (0, 0, 10): the cylinders meet in
	// the mitre plane x = y.
	t.Run("the mitre of a run", func(t *testing.T) {
		t.Parallel()
		const r = 3.0
		pair := topEdge(r3.NewVec(0, 0, 10), x).Or(decad.EndpointAt(r3.NewVec(0, 0, 10)), decad.ParallelTo(y))
		box := blockIn(t, decad.New(), r3.NewVec(0, 0, 0), r3.NewVec(20, 20, 10))
		out, err := decad.Fillet(t.Context(), box, pair, units.Millimeters(r))
		require.NoError(t, err)
		requireEndCurveCovered(t, out, func(alpha float64) r3.Vec {
			py := r - r*math.Cos(alpha)
			return r3.NewVec(py, py, 10-r+r*math.Sin(alpha))
		})
	})

	// Three edges at (0, 0, 10) take the corner route; the edges along x at
	// y = 0 and along y at x = 20 meet at (20, 0, 10) in the mitre plane
	// x + y = 20.
	t.Run("the mitre of the corner route", func(t *testing.T) {
		t.Parallel()
		const r = 2.0
		corner := r3.NewVec(0, 0, 10)
		sel := decad.Edges(decad.EndpointAt(corner), decad.ParallelTo(x)).
			Or(decad.EndpointAt(corner), decad.ParallelTo(y)).Or(decad.EndpointAt(corner), decad.ParallelTo(z)).
			Or(decad.EndpointAt(r3.NewVec(20, 0, 10)), decad.ParallelTo(y))
		box := blockIn(t, decad.New(), r3.NewVec(0, 0, 0), r3.NewVec(20, 20, 10))
		out, err := decad.Fillet(t.Context(), box, sel, units.Millimeters(r))
		require.NoError(t, err)
		requireEndCurveCovered(t, out, func(alpha float64) r3.Vec {
			py := r - r*math.Cos(alpha)
			return r3.NewVec(20-py, py, 10-r+r*math.Sin(alpha))
		})
	})
}
