package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// The R5 cases of agent-docs/retained-modify-reach-design.md §8 that the bore
// pair of faceted_retained_blend_cylinders_test.go does not reach: a concave
// valley whose both ends are reflex corners, a two-distance chamfer, and
// walls that meet at a grazing angle.

// slabCutter is a placed 60x40 slab from z0 to z1 that covers the figure
// eight, so a Cut by it takes the mesh path and leaves a retained plane
// normal to the axes.
func slabCutter(t *testing.T, sc *retentionScene, z0, z1 float64) *decad.Body {
	t.Helper()
	s, err := sc.w.CreateSketch(sc.w.XY())
	require.NoError(t, err)
	r := s.CreateRectangle(-20, -20, 60, 40)
	s.Fix(r.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	slab, err := decad.Extrude(t.Context(), sc.doc, s, s.Profiles()[0],
		decad.Distance{D: units.Millimeters(z1 - z0), Dir: decad.Along})
	require.NoError(t, err)
	move, err := r3.Translation(r3.Vec{Z: z0 + 0.001})
	require.NoError(t, err)
	placed, err := decad.Transform(t.Context(), slab, move)
	require.NoError(t, err)
	return placed
}

// figureEightHeight is the height of the figure eight below, and its ends sit
// on retained planes at z = 5.001 and z = 15.001.
const figureEightHeight = 10.0

// figureEight is the union of two Ø8 posts along z at (14, 10) and (21, 10),
// cut to z in [5.001, 15.001]. The valley between them is a straight generator
// at x = 17.5 whose ends are reflex corners of the end faces, so the edge and
// both ends are concave.
func figureEight(t *testing.T) *decad.Body {
	t.Helper()
	sc := newRetentionScene()
	b, err := decad.Union(t.Context(), zCylinderAt(t, sc, pairAxes[0][0], pairAxes[0][1], pairRad, 0, 20),
		zCylinderAt(t, sc, pairAxes[1][0], pairAxes[1][1], pairRad, 0, 20))
	require.NoError(t, err)
	b, err = decad.Cut(t.Context(), b, slabCutter(t, sc, 15, 40))
	require.NoError(t, err)
	b, err = decad.Cut(t.Context(), b, slabCutter(t, sc, -40, 5))
	require.NoError(t, err)
	return b
}

func valleyEdge(t *testing.T, b *decad.Body) decad.EdgeSelector {
	t.Helper()
	return straightEdge(t, b, r3.NewVec(pairCorner[0], pairCorner[1], 15.001), r3.NewVec(0, 0, 1))
}

// setbackRegion is the cross-section polygon between the ridge and the two
// setback generators: dA of arc along the wall about pairAxes[0] and dB along
// the wall about pairAxes[1], both away from the ridge. It also returns the
// two setback points.
func setbackRegion(dA, dB float64) ([][2]float64, [2]float64, [2]float64) {
	ang := func(o, p [2]float64) float64 { return math.Atan2(p[1]-o[1], p[0]-o[0]) }
	a0 := ang(pairAxes[0], pairCorner)
	a1 := ang(pairAxes[1], pairCorner)
	e0 := circlePoints(pairAxes[0], pairRad, a0, a0-dA/pairRad)
	e1 := circlePoints(pairAxes[1], pairRad, a1+dB/pairRad, a1)
	return append(e0, e1...), e0[len(e0)-1], e1[0]
}

// requireBevelThrough checks that exactly one plane face of b, parallel to
// the axes, contains both setback generators at height z.
func requireBevelThrough(t *testing.T, b *decad.Body, p, q [2]float64, z float64) {
	t.Helper()
	pv, qv := r3.NewVec(p[0], p[1], z), r3.NewVec(q[0], q[1], z)
	hits := 0
	for _, f := range b.Faces() {
		pl, ok := f.Surface().(decad.Plane)
		if !ok || math.Abs(pl.Frame.N().Z) > 1e-9 {
			continue
		}
		n, o := pl.Frame.N(), pl.Frame.Origin()
		if math.Abs(n.Dot(pv.Sub(o))) < 1e-9 && math.Abs(n.Dot(qv.Sub(o))) < 1e-9 {
			hits++
		}
	}
	require.Equal(t, 1, hits, "one bevel through %v and %v", p, q)
}

func TestFacetedRetainedBlendParallelCylindersConcave(t *testing.T) {
	t.Parallel()

	t.Run("the valley is a published line", func(t *testing.T) {
		t.Parallel()
		b := figureEight(t)
		lines := 0
		for _, e := range b.Edges() {
			if _, ok := e.Curve().(decad.Line3); !ok {
				continue
			}
			p, q := e.Start().Position().Value, e.End().Position().Value
			if math.Abs(p.X-17.5) < 1e-2 && math.Abs(q.X-17.5) < 1e-2 && math.Abs(math.Abs(p.Sub(q).Z)-figureEightHeight) < 1e-2 {
				lines++
			}
		}
		require.Equal(t, 2, lines)
	})

	t.Run("fillet of the valley adds material", func(t *testing.T) {
		t.Parallel()
		b := figureEight(t)
		before, err := b.Volume()
		require.NoError(t, err)
		const r = 1.0
		filleted, err := decad.Fillet(t.Context(), b, valleyEdge(t, b), units.Millimeters(r))
		require.NoError(t, err)
		after, err := filleted.Volume()
		require.NoError(t, err)
		// The region between the walls and the arc is added, so the volume grows.
		requireVolumeChange(t, before, after, -figureEightHeight*cornerRegionArea(r))
		centre := pairCentre(r)
		cylinders := 0
		for _, f := range filleted.Faces() {
			if c, ok := f.Surface().(decad.Cylinder); ok && math.Abs(c.Radius.Base()-r) < 1e-12 {
				cylinders++
				require.InDelta(t, centre[0], c.Origin.X, 1e-9)
				require.InDelta(t, centre[1], c.Origin.Y, 1e-9)
			}
		}
		require.Equal(t, 1, cylinders)
		requireVerifiedMesh(t, filleted)
	})

	t.Run("chamfer of the valley adds material", func(t *testing.T) {
		t.Parallel()
		b := figureEight(t)
		before, err := b.Volume()
		require.NoError(t, err)
		const d = 0.7
		chamfered, err := decad.Chamfer(t.Context(), b, valleyEdge(t, b), units.Millimeters(d))
		require.NoError(t, err)
		after, err := chamfered.Volume()
		require.NoError(t, err)
		region, p, q := setbackRegion(d, d)
		requireVolumeChange(t, before, after, -figureEightHeight*shoelaceArea(region))
		requireBevelThrough(t, chamfered, p, q, 10)
		requireVerifiedMesh(t, chamfered)
	})
}

func TestFacetedRetainedBlendParallelCylindersTwoDistance(t *testing.T) {
	t.Parallel()

	// Each bore wall in turn is the reference face. The reference takes dRef and
	// the other wall dOther, so the bevel's setback generators swap sides.
	const dRef, dOther = 0.7, 1.1
	for _, wall := range []int{0, 1} {
		t.Run([]string{"reference the first bore", "reference the second bore"}[wall], func(t *testing.T) {
			t.Parallel()
			b := borePair(t)
			before, err := b.Volume()
			require.NoError(t, err)
			var ref decad.FeatureRef
			found := false
			for _, f := range b.Faces() {
				if c, ok := f.Surface().(decad.Cylinder); ok && math.Abs(c.Origin.X-pairAxes[wall][0]) < 1e-9 {
					require.Len(t, f.Origins(), 1)
					ref, found = f.Origins()[0], true
				}
			}
			require.True(t, found, "the reference bore wall exists")
			chamfered, err := decad.Chamfer(t.Context(), b, pairEdge(t, b), units.Millimeters(dRef),
				decad.WithAsymmetricChamfer(decad.Faces(decad.FaceCreatedBy(ref)), units.Millimeters(dOther)))
			require.NoError(t, err)
			after, err := chamfered.Volume()
			require.NoError(t, err)
			dA, dB := dRef, dOther
			if wall == 1 {
				dA, dB = dOther, dRef
			}
			region, p, q := setbackRegion(dA, dB)
			requireVolumeChange(t, before, after, pairLen*shoelaceArea(region))
			requireBevelThrough(t, chamfered, p, q, 10)
			requireVerifiedMesh(t, chamfered)
		})
	}
}

func TestFacetedRetainedBlendParallelCylindersGrazing(t *testing.T) {
	t.Parallel()

	// Two Ø8 bores whose axes are 7.997 mm apart overlap by 3 um, so their
	// walls meet at a ridge but at a sine of the tangent angle below the
	// blend's floor (SM7).
	sc := newRetentionScene()
	b := blendPlate(t, sc)
	b, err := decad.Cut(t.Context(), b, zCylinderAt(t, sc, 14, 10, pairRad, -10, 30))
	require.NoError(t, err)
	b, err = decad.Cut(t.Context(), b, zCylinderAt(t, sc, 21.997, 10, pairRad, -10, 30))
	require.NoError(t, err)
	var ridge r3.Vec
	found := false
	for _, e := range b.Edges() {
		if _, ok := e.Curve().(decad.Line3); !ok {
			continue
		}
		p, q := e.Start().Position().Value, e.End().Position().Value
		if math.Abs(p.X-17.9985) < 1e-2 && math.Abs(q.X-17.9985) < 1e-2 && p.Y > 10 && math.Abs(p.Sub(q).Z) > 19 {
			ridge, found = p, true
		}
	}
	require.True(t, found, "the walls meet in a published line")
	for _, blend := range []struct {
		name string
		run  func() error
	}{
		{"fillet", func() error {
			_, err := decad.Fillet(t.Context(), b, straightEdge(t, b, r3.NewVec(ridge.X, ridge.Y, 20), r3.NewVec(0, 0, 1)),
				units.Millimeters(0.5))
			return err
		}},
		{"chamfer", func() error {
			_, err := decad.Chamfer(t.Context(), b, straightEdge(t, b, r3.NewVec(ridge.X, ridge.Y, 20), r3.NewVec(0, 0, 1)),
				units.Millimeters(0.5))
			return err
		}},
	} {
		err := blend.run()
		require.ErrorIs(t, err, decad.ErrUnsupported, blend.name)
		require.ErrorContains(t, err, "the two cylinders do not cross at a clear angle", blend.name)
	}
}
