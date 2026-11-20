package meshbool_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"

	"github.com/lestrrat-3d/decad/internal/meshbool"
)

// cylinderPoint is the point of the cylinder of radius r about the z axis at
// angle a and height z.
func cylinderPoint(r, a, z float64) r3.Vec {
	sin, cos := math.Sincos(a)
	return r3.NewVec(r*cos, r*sin, z)
}

// pointTriangleDistance is the distance from p to the closed triangle t,
// read in floats: the test's independent oracle.
func pointTriangleDistance(p r3.Vec, t [3]r3.Vec) float64 {
	n, ok := t[1].Sub(t[0]).Cross(t[2].Sub(t[0])).Normalize()
	if !ok {
		return math.Inf(1)
	}
	foot := p.Sub(n.Scale(p.Sub(t[0]).Dot(n)))
	inside := true
	for i := range 3 {
		e := t[(i+1)%3].Sub(t[i])
		if e.Cross(foot.Sub(t[i])).Dot(n) < 0 {
			inside = false
		}
	}
	if inside {
		return p.Sub(foot).Len()
	}
	best := math.Inf(1)
	for i := range 3 {
		a, b := t[i], t[(i+1)%3]
		e := b.Sub(a)
		s := math.Max(0, math.Min(1, p.Sub(a).Dot(e)/e.Dot(e)))
		best = math.Min(best, p.Sub(a.Add(e.Scale(s))).Len())
	}
	return best
}

// sampledSlab is the largest distance from the facet's plane of a point of
// the cylinder of radius r about the z axis that lies within beta of the
// facet, over a fine grid of angles and heights.
func sampledSlab(t [3]r3.Vec, r, beta, zLo, zHi float64) (float64, int) {
	n, _ := t[1].Sub(t[0]).Cross(t[2].Sub(t[0])).Normalize()
	const na, nz = 2000, 400
	worst, hits := 0.0, 0
	for i := range na {
		a := -math.Pi/2 + math.Pi*(float64(i)+0.5)/na
		for j := range nz {
			p := cylinderPoint(r, a, zLo+(zHi-zLo)*(float64(j)+0.5)/nz)
			if pointTriangleDistance(p, t) > beta {
				continue
			}
			hits++
			worst = math.Max(worst, math.Abs(p.Sub(t[0]).Dot(n)))
		}
	}
	return worst, hits
}

func TestCylinderSlab(t *testing.T) {
	t.Parallel()
	const r = 3.0
	axis := meshbool.SlabCylinder{Axis: r3.NewVec(0, 0, 1), Radius: r}
	t.Run("a chord facet reads its sagitta, not its corner bound", func(t *testing.T) {
		t.Parallel()
		// One triangle of a chorded wall strip, 100 mm long and 0.22 mm wide,
		// with a corner bound far above its chord sagitta of 0.002 mm.
		const half = 0.0367
		facet := [3]r3.Vec{cylinderPoint(r, -half, 0), cylinderPoint(r, half, 0), cylinderPoint(r, half, 100)}
		sagitta := r * (1 - math.Cos(half))
		for _, beta := range []float64{0.01, 0.05, 0.5} {
			slab, ok := meshbool.CylinderSlab(facet, beta, axis)
			require.True(t, ok, "beta %v", beta)
			worst, hits := sampledSlab(facet, r, beta, -1, 101)
			require.Positive(t, hits)
			require.GreaterOrEqual(t, slab, worst, "beta %v", beta)
			require.GreaterOrEqual(t, slab, sagitta, "beta %v", beta)
			require.Less(t, slab, beta, "beta %v: the slab must beat the corner bound", beta)
		}
	})
	t.Run("a facet across the axis direction reads its tilt", func(t *testing.T) {
		t.Parallel()
		// A small triangle whose plane holds no generator of the cylinder.
		facet := [3]r3.Vec{cylinderPoint(r, -0.05, 10), cylinderPoint(r, 0.08, 10.3), cylinderPoint(r, 0.01, 10.9)}
		for _, beta := range []float64{0.02, 0.2} {
			slab, ok := meshbool.CylinderSlab(facet, beta, axis)
			require.True(t, ok, "beta %v", beta)
			worst, hits := sampledSlab(facet, r, beta, 9, 12)
			require.Positive(t, hits)
			require.GreaterOrEqual(t, slab, worst, "beta %v", beta)
		}
	})
	t.Run("a facet tilted along the axis reads its height reach", func(t *testing.T) {
		t.Parallel()
		// A small facet at 45 degrees to the axis, touching the wall: a point
		// of the wall above or below it within beta lies off its plane by
		// about its height over the facet's.
		const beta = 0.2
		at := func(y, z float64) r3.Vec { return r3.NewVec(r+(z-10), y, z) }
		facet := [3]r3.Vec{at(-0.05, 9.95), at(0.05, 9.95), at(0, 10.05)}
		slab, ok := meshbool.CylinderSlab(facet, beta, axis)
		require.True(t, ok)
		worst, hits := sampledSlab(facet, r, beta, 9, 11)
		require.Positive(t, hits)
		require.GreaterOrEqual(t, slab, worst)
	})
	t.Run("a displaced wall adds its charge", func(t *testing.T) {
		t.Parallel()
		const half, delta, beta = 0.0367, 0.01, 0.05
		facet := [3]r3.Vec{cylinderPoint(r, -half, 0), cylinderPoint(r, half, 0), cylinderPoint(r, half, 100)}
		charged := axis
		charged.Delta = delta
		slab, ok := meshbool.CylinderSlab(facet, beta, charged)
		require.True(t, ok)
		// The true wall may be the cylinder of radius r + delta.
		worst, hits := sampledSlab(facet, r+delta, beta, -1, 101)
		require.Positive(t, hits)
		require.GreaterOrEqual(t, slab, worst)
	})
	t.Run("a plane through the axis proves nothing", func(t *testing.T) {
		t.Parallel()
		facet := [3]r3.Vec{r3.NewVec(-3, 0, 0), r3.NewVec(3, 0, 0), r3.NewVec(3, 0, 10)}
		_, ok := meshbool.CylinderSlab(facet, 0.01, axis)
		require.False(t, ok)
	})
}
