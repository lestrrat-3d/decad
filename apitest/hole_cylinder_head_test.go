package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file is agent-docs/hole-design.md §10's public-surface tests for a head on a
// hole in a cylindrical face.

// chordWeighted is ∫ 2√(a² − y²)·f(y) dy over y in [-a, a]: the integral of f(y) over the
// disc of radius a, where f depends on the coordinate y across the disc and not on the one
// along the cylinder's axis.
func chordWeighted(a float64, f func(y float64) float64) float64 {
	const steps = 200000
	step := 2 * a / steps
	total := 0.0
	for i := range steps {
		y := -a + (float64(i)+0.5)*step
		total += 2 * math.Sqrt(a*a-y*y) * f(y) * step
	}
	return total
}

func TestHoleCounterboreInARod(t *testing.T) {
	t.Parallel()
	const R, L = 10.0, 16.0
	doc := decad.New()
	rod := circleBodyAtZ(t, doc, 0, R, 0, L)
	// A 3 mm hole 6 mm deep from the tangent plane at x = 10, with a 6 mm counterbore 2 mm deep
	// from it. The rod's wall at distance y across is x = √(R² − y²), so the bore removes
	// √(R² − y²) − 4 of length over its disc and the recess √(R² − y²) − 8 over its ring.
	out, err := decad.Hole(t.Context(), rod, decad.HoleSpec{
		Face: decad.Faces(decad.Cylindrical()), At: r3.NewVec(R, 0, L/2),
		Diameter: units.Millimeters(3), Depth: decad.BlindHole{Depth: units.Millimeters(6)},
		Head: decad.Counterbore{Diameter: units.Millimeters(6), Depth: units.Millimeters(2)}})
	require.NoError(t, err)
	require.Equal(t, []*decad.Body{out}, doc.Bodies())
	wall := func(y float64) float64 { return math.Sqrt(R*R - y*y) }
	removed := chordWeighted(1.5, func(y float64) float64 { return wall(y) - 4 }) +
		chordWeighted(3, func(y float64) float64 { return wall(y) - 8 }) -
		chordWeighted(1.5, func(y float64) float64 { return wall(y) - 8 })
	volume, err := out.Volume()
	require.NoError(t, err)
	requireCylinderHoleVolume(t, volume, math.Pi*R*R*L-removed)
	// The recess floor is at x = 8: a point 0.5 above it, off the bore, is empty; one below it is material.
	recess, err := out.LocatePoint(t.Context(), r3.NewVec(8.5, 2, L/2), units.Millimeters(0.01))
	require.NoError(t, err)
	require.Equal(t, decad.PointOutside, recess)
	below, err := out.LocatePoint(t.Context(), r3.NewVec(7.5, 2, L/2), units.Millimeters(0.01))
	require.NoError(t, err)
	require.Equal(t, decad.PointInside, below)
}

func TestHoleCounterboreOutOfABore(t *testing.T) {
	t.Parallel()
	const Rb = 5.0
	doc := decad.New()
	block := boxBody(t, doc, -20, -20, 20, 20, 10)
	part, err := decad.Cut(t.Context(), block, circleBodyAtZ(t, doc, 0, Rb, -1, 12))
	require.NoError(t, err)
	// A 2 mm hole through from the bore wall at (5, 0, 5) out along +X, with an 8 mm counterbore
	// 1.5 mm deep from the tangent plane x = 5; the wall rises 2 mm toward that plane at 4 mm across. The wall at y across is x = √(R² − y²), below
	// the tangent plane, so the recess removes 5 + 1.5 − √(R² − y²) of length over its disc and
	// the bore the rest of the way to the block's face at x = 20.
	out, err := decad.Hole(t.Context(), part, decad.HoleSpec{
		Face: decad.Faces(decad.Cylindrical()), At: r3.NewVec(Rb, 0, 5),
		Diameter: units.Millimeters(2), Depth: decad.ThroughHole{},
		Head: decad.Counterbore{Diameter: units.Millimeters(8), Depth: units.Millimeters(1.5)}})
	require.NoError(t, err)
	recess := chordWeighted(4, func(y float64) float64 { return 6.5 - math.Sqrt(Rb*Rb-y*y) })
	bore := math.Pi * 1 * (20 - 6.5)
	volume, err := out.Volume()
	require.NoError(t, err)
	requireCylinderHoleVolume(t, volume, 40*40*10-math.Pi*Rb*Rb*10-recess-bore)
}

func TestHoleCountersinkInARod(t *testing.T) {
	t.Parallel()
	const R, L = 10.0, 16.0
	doc := decad.New()
	rod := circleBodyAtZ(t, doc, 0, R, 0, L)
	// A 2 mm hole 8 mm deep with a 90 degree countersink to 6 mm across at the tangent plane
	// x = 10: the cone narrows 1 mm in radius for each mm of depth, from radius 3 at x = 10 to 1
	// at x = 8. The wall at distance ρ from the axis across is x = √(R² − y²), and the cone at
	// (y, z) with ρ = √(y² + z²) reaches x = 10 − (3 − ρ) down to the bore.
	out, err := decad.Hole(t.Context(), rod, decad.HoleSpec{
		Face: decad.Faces(decad.Cylindrical()), At: r3.NewVec(R, 0, L/2),
		Diameter: units.Millimeters(2), Depth: decad.BlindHole{Depth: units.Millimeters(8)},
		Head: decad.Countersink{Diameter: units.Millimeters(6), Angle: units.Degrees(90)}})
	require.NoError(t, err)
	// Integrate over the disc of radius 3 in polar coordinates about the hole axis: at
	// ρ the removed length along x is the wall's x there less the cone's floor
	// max(10 − (3 − ρ), 2) for ρ ≥ 1, and less 2 for the bore (ρ < 1).
	const steps = 4000
	removed := 0.0
	for i := range steps {
		rho := 3 * (float64(i) + 0.5) / steps
		var floor float64
		if rho < 1 {
			floor = 2 // the bore runs 8 deep from x = 10
		} else {
			floor = 10 - (3 - rho)
		}
		// Average the removed length over the angle about the axis: the wall is at
		// x = √(R² − (ρ sin φ)²), and the cone removes nothing where the wall has fallen
		// below its floor.
		const around = 720
		avg := 0.0
		for k := range around {
			phi := 2 * math.Pi * (float64(k) + 0.5) / around
			avg += math.Max(0, math.Sqrt(R*R-(rho*math.Sin(phi))*(rho*math.Sin(phi)))-floor)
		}
		avg /= around
		removed += 2 * math.Pi * rho * avg * (3.0 / steps)
	}
	volume, err := out.Volume()
	require.NoError(t, err)
	requireCylinderHoleVolume(t, volume, math.Pi*R*R*L-removed)
}

func TestHoleHeadOnACylinderRefusals(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	rod := circleBodyAtZ(t, doc, 0, 10, 0, 16)
	spec := decad.HoleSpec{Face: decad.Faces(decad.Cylindrical()), At: r3.NewVec(10, 0, 8),
		Diameter: units.Millimeters(3), Depth: decad.BlindHole{Depth: units.Millimeters(6)}}
	spec.Head = decad.Counterbore{Diameter: units.Millimeters(2), Depth: units.Millimeters(1)}
	_, err := decad.Hole(t.Context(), rod, spec)
	require.ErrorIs(t, err, decad.ErrDegenerate, "a counterbore narrower than the bore")
	spec.Head = decad.Counterbore{Diameter: units.Millimeters(6), Depth: units.Millimeters(8)}
	_, err = decad.Hole(t.Context(), rod, spec)
	require.ErrorIs(t, err, decad.ErrDegenerate, "a counterbore deeper than the hole")
	require.Len(t, doc.Bodies(), 1)
	require.True(t, doc.Bodies()[0] == rod)
}

func TestHoleCountersinkOutOfABore(t *testing.T) {
	t.Parallel()
	const Rb = 5.0
	doc := decad.New()
	block := boxBody(t, doc, -20, -20, 20, 20, 10)
	part, err := decad.Cut(t.Context(), block, circleBodyAtZ(t, doc, 0, Rb, -1, 12))
	require.NoError(t, err)
	// A 2 mm hole through from the bore wall at (5, 0, 5) out along +X, with a 90 degree countersink 4 mm
	// across at the tangent plane x = 5: the cone has radius 7 − x there, so 2 at x = 5 and 1 at x = 6,
	// and goes on toward the axis beyond the plane, where the wall rises into it. At distance ρ from the
	// hole axis and y across, the hole removes the material from the wall x = √(R² − y²) to 20 for ρ ≤ 1
	// (the bore), and to 7 − ρ for 1 < ρ ≤ 4 where the cone is (it stops 2 mm over the plane, at radius 4).
	const steps, around = 4000, 720
	removed := 0.0
	for i := range steps {
		rho := 4 * (float64(i) + 0.5) / steps
		avg := 0.0
		for k := range around {
			phi := 2 * math.Pi * (float64(k) + 0.5) / around
			wall := math.Sqrt(Rb*Rb - (rho*math.Sin(phi))*(rho*math.Sin(phi)))
			if rho <= 1 {
				avg += 20 - wall
			} else {
				avg += math.Max(0, 7-rho-wall)
			}
		}
		removed += 2 * math.Pi * rho * (avg / around) * (4.0 / steps)
	}
	out, err := decad.Hole(t.Context(), part, decad.HoleSpec{
		Face: decad.Faces(decad.Cylindrical()), At: r3.NewVec(Rb, 0, 5),
		Diameter: units.Millimeters(2), Depth: decad.ThroughHole{},
		Head: decad.Countersink{Diameter: units.Millimeters(4), Angle: units.Degrees(90)}})
	require.NoError(t, err)
	volume, err := out.Volume()
	require.NoError(t, err)
	requireCylinderHoleVolume(t, volume, 40*40*10-math.Pi*Rb*Rb*10-removed)
}
