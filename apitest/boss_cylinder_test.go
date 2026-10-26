package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file is agent-docs/boss-design.md §8's public-surface tests for a Boss on a
// cylindrical face.

// bossAboveCylinder is the volume of a round boss of radius r that stands h out of the
// tangent plane at a cylinder of radius R, outside the cylinder: ∫ 2√(r² − y²) ·
// (R + h − √(R² − y²)) dy over the boss's width, the pillar's top being the plane R + h
// from the cylinder's axis and its floor the cylinder's wall.
func bossAboveCylinder(R, r, h float64) float64 {
	const steps = 200000
	step := 2 * r / steps
	total := 0.0
	for i := range steps {
		y := -r + (float64(i)+0.5)*step
		total += 2 * math.Sqrt(r*r-y*y) * (R + h - math.Sqrt(R*R-y*y)) * step
	}
	return total
}

// bossInsideBore is the volume of a round boss of radius r that stands h in from the
// wall of a bore of radius R, inside the bore: ∫ 2√(r² − y²) · (√(R² − y²) − (R − h)) dy.
func bossInsideBore(R, r, h float64) float64 {
	const steps = 200000
	step := 2 * r / steps
	total := 0.0
	for i := range steps {
		y := -r + (float64(i)+0.5)*step
		total += 2 * math.Sqrt(r*r-y*y) * (math.Sqrt(R*R-y*y) - (R - h)) * step
	}
	return total
}

func TestBossStandsOnARod(t *testing.T) {
	t.Parallel()
	const R, L, r, h = 10.0, 16.0, 6.0, 20.0
	doc := decad.New()
	rod := circleBodyAtZ(t, doc, 0, R, 0, L)
	out, err := decad.Boss(t.Context(), rod, decad.BossSpec{
		Face: decad.Faces(decad.Cylindrical()), At: r3.NewVec(R, 0, L/2),
		Diameter: units.Millimeters(2 * r), Height: units.Millimeters(h)})
	require.NoError(t, err)
	require.Equal(t, []*decad.Body{out}, doc.Bodies(), "the rod and the pillar leave the document")
	volume, err := out.Volume()
	require.NoError(t, err)
	requireCylinderHoleVolume(t, volume, math.Pi*R*R*L+bossAboveCylinder(R, r, h))
	box, err := out.Bounds()
	require.NoError(t, err)
	require.InDelta(t, R+h, box.Max.X, 1e-6, "the boss's top is h above the tangent plane")
	require.InDelta(t, -R, box.Min.X, 0.1)
	require.InDelta(t, L, box.Max.Z, 1e-6)
	// A point inside the pillar, off the rod, is material; one beside it is not.
	in, err := out.LocatePoint(t.Context(), r3.NewVec(R+10, 0, L/2), units.Millimeters(0.01))
	require.NoError(t, err)
	require.Equal(t, decad.PointInside, in)
	beside, err := out.LocatePoint(t.Context(), r3.NewVec(R+10, 0, L/2+r+1), units.Millimeters(0.01))
	require.NoError(t, err)
	require.Equal(t, decad.PointOutside, beside)
}

func TestBossStandsOnTheWallOfABore(t *testing.T) {
	t.Parallel()
	const Ri, r, h = 5.0, 1.5, 3.0
	doc := decad.New()
	block := boxBody(t, doc, -20, -20, 20, 20, 10)
	part, err := decad.Cut(t.Context(), block, circleBodyAtZ(t, doc, 0, Ri, -1, 12))
	require.NoError(t, err)
	// The bore's wall looks toward the axis, so the boss grows from the wall into the bore.
	out, err := decad.Boss(t.Context(), part, decad.BossSpec{
		Face: decad.Faces(decad.Cylindrical()), At: r3.NewVec(Ri, 0, 5),
		Diameter: units.Millimeters(2 * r), Height: units.Millimeters(h)})
	require.NoError(t, err)
	volume, err := out.Volume()
	require.NoError(t, err)
	requireCylinderHoleVolume(t, volume, 40*40*10-math.Pi*Ri*Ri*10+bossInsideBore(Ri, r, h))
	// The boss's tip is h in from the wall, at x = 2: a point in it is material, and the
	// bore's middle beyond it is still empty.
	in, err := out.LocatePoint(t.Context(), r3.NewVec(Ri-h+0.5, 0, 5), units.Millimeters(0.01))
	require.NoError(t, err)
	require.Equal(t, decad.PointInside, in)
	axis, err := out.LocatePoint(t.Context(), r3.NewVec(0, 0, 5), units.Millimeters(0.01))
	require.NoError(t, err)
	require.Equal(t, decad.PointOutside, axis)
}

func TestBossOnACylinderRefusals(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	rod := circleBodyAtZ(t, doc, 0, 10, 0, 6)
	spec := decad.BossSpec{Face: decad.Faces(decad.Cylindrical()), At: r3.NewVec(10, 0, 3),
		Diameter: units.Millimeters(4), Height: units.Millimeters(5)}
	boss := func(edit func(*decad.BossSpec)) error {
		s := spec
		edit(&s)
		_, err := decad.Boss(t.Context(), rod, s)
		return err
	}
	require.ErrorIs(t, boss(func(s *decad.BossSpec) { s.RootFillet = units.Millimeters(1) }), decad.ErrUnsupported)
	require.ErrorIs(t, boss(func(s *decad.BossSpec) { s.Draft = units.Degrees(3) }), decad.ErrUnsupported)
	require.ErrorIs(t, boss(func(s *decad.BossSpec) { s.Diameter = units.Millimeters(20) }), decad.ErrDegenerate, "as wide as the rod")
	require.ErrorIs(t, boss(func(s *decad.BossSpec) { s.At = r3.NewVec(9, 0, 3) }), decad.ErrDegenerate, "off the cylinder")
	require.ErrorIs(t, boss(func(s *decad.BossSpec) { s.At = r3.NewVec(0, 0, 3) }), decad.ErrDegenerate, "on the axis")
	require.Len(t, doc.Bodies(), 1)
	require.True(t, doc.Bodies()[0] == rod, "every refusal leaves the rod live")
}
