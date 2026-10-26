package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file is agent-docs/hole-design.md §7's public-surface tests for Hole on
// a cylindrical face: a radial hole through a rod, and one out of a bore.

// removedAcross is the volume a radius-r bore, its axis square to a rod of
// radius R and through the rod's axis, cuts out of the rod: the area of the
// bore's circle in the plane square to its axis, weighted by the rod's chord
// there. It is 4 ∫ sqrt(R² - y²) sqrt(r² - y²) dy over y in [-r, r], halved
// each side.
func removedAcross(radius, bore float64) float64 {
	const steps = 200000
	step := 2 * bore / steps
	total := 0.0
	for i := range steps {
		y := -bore + (float64(i)+0.5)*step
		total += 2 * math.Sqrt(radius*radius-y*y) * 2 * math.Sqrt(bore*bore-y*y) * step
	}
	return total
}

func requireCylinderHoleVolume(t *testing.T, m decad.Measurement, want float64) {
	t.Helper()
	require.LessOrEqual(t, math.Abs(m.Value.Base()-want), m.Bound.Base()+1e-6*want,
		"volume %v ± %v should enclose %v", m.Value.Base(), m.Bound.Base(), want)
	require.Less(t, m.Bound.Base(), 0.01*want)
}

func TestHoleRadialThroughARod(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	rod := circleBodyAtZ(t, doc, 0, 5, 0, 20) // radius 5 about Z, z from 0 to 20
	out, err := decad.Hole(t.Context(), rod, decad.HoleSpec{
		Face:     decad.Faces(decad.Cylindrical()),
		At:       r3.NewVec(5, 0, 10),
		Diameter: units.Millimeters(3),
		Depth:    decad.ThroughHole{},
	})
	require.NoError(t, err)
	require.Equal(t, []*decad.Body{out}, doc.Bodies())
	volume, err := out.Volume()
	require.NoError(t, err)
	requireCylinderHoleVolume(t, volume, math.Pi*25*20-removedAcross(5, 1.5))
	// The hole runs along X through the rod's middle: a point on its axis inside
	// the rod is empty, and one beside it is material.
	axisPoint, err := out.LocatePoint(t.Context(), r3.NewVec(0, 0, 10), units.Millimeters(0.01))
	require.NoError(t, err)
	require.Equal(t, decad.PointOutside, axisPoint)
	beside, err := out.LocatePoint(t.Context(), r3.NewVec(0, 3, 10), units.Millimeters(0.01))
	require.NoError(t, err)
	require.Equal(t, decad.PointInside, beside)
}

func TestHoleRadialBlindIntoARod(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	rod := circleBodyAtZ(t, doc, 0, 5, 0, 20)
	out, err := decad.Hole(t.Context(), rod, decad.HoleSpec{
		Face:     decad.Faces(decad.Cylindrical()),
		At:       r3.NewVec(0, 5, 10),
		Diameter: units.Millimeters(2),
		Depth:    decad.BlindHole{Depth: units.Millimeters(3)},
	})
	require.NoError(t, err)
	// The 3 mm blind hole reaches y = 2 along the radial +Y line from the
	// surface at (0, 5, 10): its axis point at y = 3 is empty, at y = 1.5 material.
	inside, err := out.LocatePoint(t.Context(), r3.NewVec(0, 3.5, 10), units.Millimeters(0.01))
	require.NoError(t, err)
	require.Equal(t, decad.PointOutside, inside, "in the hole")
	below, err := out.LocatePoint(t.Context(), r3.NewVec(0, 1.5, 10), units.Millimeters(0.01))
	require.NoError(t, err)
	require.Equal(t, decad.PointInside, below, "beyond the blind end")
}

func TestHoleRadialOutOfABore(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	block := boxBody(t, doc, -20, -20, 20, 20, 10)
	bore := circleBodyAtZ(t, doc, 0, 5, -1, 12)
	part, err := decad.Cut(t.Context(), block, bore)
	require.NoError(t, err)
	// A radial hole from the bore wall at (5, 0, 5) out through the wall along +X.
	out, err := decad.Hole(t.Context(), part, decad.HoleSpec{
		Face:     decad.Faces(decad.Cylindrical()),
		At:       r3.NewVec(5, 0, 5),
		Diameter: units.Millimeters(3),
		Depth:    decad.ThroughHole{},
	})
	require.NoError(t, err)
	// The hole removes the wall's material between the bore and the block's +X
	// face: along x from sqrt(25 - y²) to 20, over the hole's circle in y and z.
	const steps = 200000
	removed, step := 0.0, 3.0/steps
	for i := range steps {
		y := -1.5 + (float64(i)+0.5)*step
		removed += 2 * math.Sqrt(1.5*1.5-y*y) * (20 - math.Sqrt(25-y*y)) * step
	}
	volume, err := out.Volume()
	require.NoError(t, err)
	requireCylinderHoleVolume(t, volume, 40*40*10-math.Pi*25*10-removed)
}

// A hole nearly as wide as the bore it starts in meets a wall that rises
// toward the tangent plane by more than the default tool margin, so the tool must
// start farther out to leave no sliver of wall above its start.
func TestHoleRadialOutOfATightBore(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	block := boxBody(t, doc, -20, -20, 20, 20, 10)
	part, err := decad.Cut(t.Context(), block, circleBodyAtZ(t, doc, 0, 3, -1, 12))
	require.NoError(t, err)
	out, err := decad.Hole(t.Context(), part, decad.HoleSpec{
		Face:     decad.Faces(decad.Cylindrical()),
		At:       r3.NewVec(3, 0, 5),
		Diameter: units.Millimeters(5),
		Depth:    decad.ThroughHole{},
	})
	require.NoError(t, err)
	const steps = 200000
	removed, step := 0.0, 5.0/steps
	for i := range steps {
		y := -2.5 + (float64(i)+0.5)*step
		removed += 2 * math.Sqrt(2.5*2.5-y*y) * (20 - math.Sqrt(9-y*y)) * step
	}
	volume, err := out.Volume()
	require.NoError(t, err)
	requireCylinderHoleVolume(t, volume, 40*40*10-math.Pi*9*10-removed)
}

func TestHoleOnACylinderRefusals(t *testing.T) {
	t.Parallel()
	cases := []struct {
		name string
		edit func(*decad.HoleSpec)
		want error
	}{
		{"centre off the cylinder", func(s *decad.HoleSpec) { s.At = r3.NewVec(4, 0, 10) }, decad.ErrDegenerate},
		{"centre on the axis", func(s *decad.HoleSpec) { s.At = r3.NewVec(0, 0, 10) }, decad.ErrDegenerate},
		{"draft on a curve", func(s *decad.HoleSpec) { s.Draft = units.Degrees(2) }, decad.ErrUnsupported},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			doc := decad.New()
			rod := circleBodyAtZ(t, doc, 0, 5, 0, 20)
			spec := decad.HoleSpec{
				Face: decad.Faces(decad.Cylindrical()), At: r3.NewVec(5, 0, 10),
				Diameter: units.Millimeters(3), Depth: decad.ThroughHole{},
			}
			tc.edit(&spec)
			_, err := decad.Hole(t.Context(), rod, spec)
			require.ErrorIs(t, err, tc.want)
			require.Equal(t, []*decad.Body{rod}, doc.Bodies(), "a refused hole leaves the rod live and alone")
		})
	}
	t.Run("a hole wider than the bore it starts in", func(t *testing.T) {
		t.Parallel()
		doc := decad.New()
		block := boxBody(t, doc, -20, -20, 20, 20, 10)
		part, err := decad.Cut(t.Context(), block, circleBodyAtZ(t, doc, 0, 5, -1, 12))
		require.NoError(t, err)
		_, err = decad.Hole(t.Context(), part, decad.HoleSpec{
			Face: decad.Faces(decad.Cylindrical()), At: r3.NewVec(5, 0, 5),
			Diameter: units.Millimeters(12), Depth: decad.ThroughHole{},
		})
		require.ErrorIs(t, err, decad.ErrUnsupported)
	})
}
