package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// The fixtures of agent-docs/retained-modify-reach-design.md §8 (R5): the 40x20x20
// plate with two overlapping Ø8 bores along z at (14, 10) and (21, 10), each
// cut by a placed tool so the Booleans take the mesh path. The bores meet in
// two straight generators at x = 17.5; the one nearer y = 0 is selected.

const (
	pairRad = 4.0
	pairLen = 20.0
)

// pairCorner is the generator's cross-section point.
var pairCorner = [2]float64{17.5, 10 - math.Sqrt(pairRad*pairRad-3.5*3.5)}

var pairAxes = [2][2]float64{{14, 10}, {21, 10}}

func borePair(t *testing.T) *decad.Body {
	t.Helper()
	sc := newRetentionScene()
	b := blendPlate(t, sc)
	for _, a := range pairAxes {
		var err error
		b, err = decad.Cut(t.Context(), b, zCylinderAt(t, sc, a[0], a[1], pairRad, -10, 30))
		require.NoError(t, err)
	}
	return b
}

func pairEdge(t *testing.T, b *decad.Body) decad.EdgeSelector {
	t.Helper()
	return straightEdge(t, b, r3.NewVec(pairCorner[0], pairCorner[1], 20), r3.NewVec(0, 0, 1))
}

// circlePoints samples the circle about o of radius rad from angle a0 to a1.
func circlePoints(o [2]float64, rad, a0, a1 float64) [][2]float64 {
	const n = 2000
	out := make([][2]float64, 0, n+1)
	for i := 0; i <= n; i++ {
		a := a0 + (a1-a0)*float64(i)/float64(n)
		out = append(out, [2]float64{o[0] + rad*math.Cos(a), o[1] + rad*math.Sin(a)})
	}
	return out
}

// shortSweep is the signed angle from a0 to a1 within (-pi, pi].
func shortSweep(a0, a1 float64) float64 {
	d := a1 - a0
	for d > math.Pi {
		d -= 2 * math.Pi
	}
	for d <= -math.Pi {
		d += 2 * math.Pi
	}
	return d
}

// pairCentre is the centre, in (x, y), of the fillet ball of radius r outside
// both bores (or both bosses): r + 4 mm from each axis, nearest the ridge.
func pairCentre(r float64) [2]float64 {
	return [2]float64{17.5, 10 - math.Sqrt((pairRad+r)*(pairRad+r)-3.5*3.5)}
}

// cornerRegionArea is the cross-section area between the ridge, the two walls'
// tangent points and the fillet arc of radius r.
func cornerRegionArea(r float64) float64 {
	centre := pairCentre(r)
	tangent := func(o [2]float64) [2]float64 {
		return [2]float64{o[0] + (centre[0]-o[0])*pairRad/(pairRad+r), o[1] + (centre[1]-o[1])*pairRad/(pairRad+r)}
	}
	t0, t1 := tangent(pairAxes[0]), tangent(pairAxes[1])
	ang := func(o, p [2]float64) float64 { return math.Atan2(p[1]-o[1], p[0]-o[0]) }
	region := circlePoints(pairAxes[0], pairRad, ang(pairAxes[0], pairCorner), ang(pairAxes[0], t0))
	a0, a1 := ang(centre, t0), ang(centre, t1)
	region = append(region, circlePoints(centre, r, a0, a0+shortSweep(a0, a1))...)
	region = append(region, circlePoints(pairAxes[1], pairRad, ang(pairAxes[1], t1), ang(pairAxes[1], pairCorner))...)
	return shoelaceArea(region)
}

func TestFacetedRetainedBlendParallelCylinders(t *testing.T) {
	t.Parallel()

	t.Run("the ridge between two bores is a published line", func(t *testing.T) {
		t.Parallel()
		b := borePair(t)
		lines := 0
		for _, e := range b.Edges() {
			if _, ok := e.Curve().(decad.Line3); !ok {
				continue
			}
			p, q := e.Start().Position().Value, e.End().Position().Value
			if math.Abs(p.X-17.5) < 1e-2 && math.Abs(q.X-17.5) < 1e-2 && math.Abs(p.Sub(q).Z) > 19 {
				lines++
			}
		}
		require.Equal(t, 2, lines)
	})

	t.Run("fillet of the ridge", func(t *testing.T) {
		t.Parallel()
		b := borePair(t)
		before, err := b.Volume()
		require.NoError(t, err)
		const r = 1.0
		filleted, err := decad.Fillet(t.Context(), b, pairEdge(t, b), units.Millimeters(r))
		require.NoError(t, err)
		after, err := filleted.Volume()
		require.NoError(t, err)
		centre := pairCentre(r)
		requireVolumeChange(t, before, after, pairLen*cornerRegionArea(r))
		cylinders := 0
		for _, f := range filleted.Faces() {
			if c, ok := f.Surface().(decad.Cylinder); ok && math.Abs(c.Radius.Base()-r) < 1e-12 {
				cylinders++
				require.InDelta(t, centre[0], c.Origin.X, 1e-9)
				require.InDelta(t, centre[1], c.Origin.Y, 1e-9)
			}
		}
		require.Equal(t, 1, cylinders)
		mesh, err := filleted.Tessellate(t.Context(), units.Millimeters(0.1), decad.WithVerification(decad.VerifyAll))
		require.NoError(t, err)
		require.True(t, mesh.BoundaryVerified())
		require.True(t, mesh.VolumeVerified())
	})

	t.Run("chamfer of the ridge", func(t *testing.T) {
		t.Parallel()
		b := borePair(t)
		before, err := b.Volume()
		require.NoError(t, err)
		const d = 0.7
		chamfered, err := decad.Chamfer(t.Context(), b, pairEdge(t, b), units.Millimeters(d))
		require.NoError(t, err)
		after, err := chamfered.Volume()
		require.NoError(t, err)
		ang := func(o, p [2]float64) float64 { return math.Atan2(p[1]-o[1], p[0]-o[0]) }
		// 0.5 mm of arc along each bore wall from the ridge, away from it.
		a0 := ang(pairAxes[0], pairCorner)
		a1 := ang(pairAxes[1], pairCorner)
		e0 := circlePoints(pairAxes[0], pairRad, a0, a0-d/pairRad)
		e1 := circlePoints(pairAxes[1], pairRad, a1+d/pairRad, a1)
		region := append(e0, e1...)
		requireVolumeChange(t, before, after, pairLen*shoelaceArea(region))
		// The bevel is the plane through the two setback generators.
		wantY := e0[len(e0)-1][1]
		bevels := 0
		for _, f := range chamfered.Faces() {
			if p, ok := f.Surface().(decad.Plane); ok && math.Abs(p.Frame.Origin().Y-wantY) < 1e-9 &&
				math.Abs(math.Abs(p.Frame.N().Y)-1) < 1e-9 {
				bevels++
			}
		}
		require.Equal(t, 1, bevels)
		mesh, err := chamfered.Tessellate(t.Context(), units.Millimeters(0.1), decad.WithVerification(decad.VerifyAll))
		require.NoError(t, err)
		require.True(t, mesh.BoundaryVerified())
		require.True(t, mesh.VolumeVerified())
	})

	t.Run("a valley that ends on the plate and on the boss top refuses", func(t *testing.T) {
		t.Parallel()
		sc := newRetentionScene()
		tools, err := decad.Union(t.Context(), zCylinderAt(t, sc, pairAxes[0][0], pairAxes[0][1], pairRad, 10, 30),
			zCylinderAt(t, sc, pairAxes[1][0], pairAxes[1][1], pairRad, 10, 30))
		require.NoError(t, err)
		b, err := decad.Union(t.Context(), blendPlate(t, sc), tools)
		require.NoError(t, err)
		// The valley is concave, so it must end on a face that is reflex at the
		// corner. The boss top is, but the plate's top face is convex there.
		_, err = decad.Fillet(t.Context(), b,
			straightEdge(t, b, r3.NewVec(pairCorner[0], pairCorner[1], 20), r3.NewVec(0, 0, 1)), units.Millimeters(1))
		require.ErrorIs(t, err, decad.ErrUnsupported)
	})

	t.Run("crossing cylinders stay refused", func(t *testing.T) {
		t.Parallel()
		sc := newRetentionScene()
		b := blendPlate(t, sc)
		b, err := decad.Cut(t.Context(), b, zCylinderAt(t, sc, 20, 10, 4.5, -10, 30))
		require.NoError(t, err)
		s, err := sc.w.CreateSketch(sc.w.YZ())
		require.NoError(t, err)
		c := s.CreatePoint(10, 10)
		s.Fix(c)
		s.CreateCircle(c, 3)
		_, err = s.Solve(t.Context())
		require.NoError(t, err)
		cross, err := decad.Extrude(t.Context(), sc.doc, s, s.Profiles()[0], decad.Symmetric{D: units.Millimeters(30)})
		require.NoError(t, err)
		shift, err := r3.Translation(r3.Vec{X: 20.001})
		require.NoError(t, err)
		cross, err = decad.Transform(t.Context(), cross, shift)
		require.NoError(t, err)
		b, err = decad.Cut(t.Context(), b, cross)
		require.NoError(t, err)
		// The two bores meet along a curve that is not a straight generator, so
		// none of its edges is a Line3 between two cylinders.
		found := false
		for _, e := range b.Edges() {
			cyl := 0
			for _, f := range e.Faces() {
				if _, ok := f.Surface().(decad.Cylinder); ok {
					cyl++
				}
			}
			if cyl != 2 {
				continue
			}
			found = true
			_, isLine := e.Curve().(decad.Line3)
			require.False(t, isLine)
			_, err = decad.Fillet(t.Context(), b, decad.Edges(decad.EndpointAt(e.Start().Position().Value),
				decad.EndpointAt(e.End().Position().Value)).AtLeast(1), units.Millimeters(0.5))
			require.ErrorIs(t, err, decad.ErrUnsupported)
			break
		}
		require.True(t, found, "the bores meet in an edge between two cylinders")
	})
}
