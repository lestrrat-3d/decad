package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file is agent-docs/hole-design.md §12's public-surface tests for a thread in a
// hole in a cylindrical face.

// removedInternalOnWall is removedPartialInternal for a bore whose mouth lies in a curved
// wall: the groove's axial overlap with [lower, span], lower being how far below the
// tangent plane the wall is at the groove's place, sag(y) for y across the hole at
// ρ·sin φ, φ the angle about the bore from the cylinder's axis.
func removedInternalOnWall(radius, pitch, span, turns float64, sag func(y float64) float64) float64 {
	depth := threadHeight * 5 / 8 * pitch
	root := radius + depth
	mouth := radius - 0.05*pitch
	flat := pitch / 4
	half := flat/2 + math.Abs(mouth-root)*math.Tan(math.Pi/6)
	first := -half - 0.05*pitch
	const radii, angles = 200, 40000
	dRho := depth / radii
	dPhi := 2 * math.Pi * turns / angles
	total := 0.0
	for i := range radii {
		rho := radius + (float64(i)+0.5)*dRho
		h := flat/2 + (root-rho)*math.Tan(math.Pi/6)
		slice := 0.0
		for j := range angles {
			phi := dPhi * (float64(j) + 0.5)
			c := first + pitch*phi/(2*math.Pi)
			lower := math.Max(0, sag(rho*math.Sin(phi)))
			slice += max(0, min(span, c+h)-max(lower, c-h)) * dPhi
		}
		total += rho * slice * dRho
	}
	return total
}

func TestHoleThreadInARod(t *testing.T) {
	t.Parallel()
	const R, L, depth = 25.0, 16.0, 6.0
	doc := decad.New()
	rod := circleBodyAtZ(t, doc, 0, R, 0, L)
	out, err := decad.Hole(t.Context(), rod, decad.HoleSpec{
		Face: decad.Faces(decad.Cylindrical()), At: r3.NewVec(R, 0, L/2),
		Diameter: units.Millimeters(2 * tapRadius), Depth: decad.BlindHole{Depth: units.Millimeters(depth)},
		Thread: &decad.HoleThread{Pitch: units.Millimeters(tapPitch)}})
	require.NoError(t, err)
	require.Equal(t, []*decad.Body{out}, doc.Bodies())
	wall := func(y float64) float64 { return math.Sqrt(R*R - y*y) }
	bore := chordWeighted(tapRadius, func(y float64) float64 { return wall(y) - (R - depth) })
	turns := (depth + 0.05*tapPitch) / tapPitch
	thread := removedInternalOnWall(tapRadius, tapPitch, depth, turns, func(y float64) float64 { return R - wall(y) })
	volume, err := out.Volume()
	require.NoError(t, err)
	requireThreadVolume(t, volume, math.Pi*R*R*L-bore-thread)
}

func TestHoleThreadOutOfABore(t *testing.T) {
	t.Parallel()
	const Rb = 6.0
	doc := decad.New()
	block := boxBody(t, doc, -20, -20, 20, 20, 10)
	part, err := decad.Cut(t.Context(), block, circleBodyAtZ(t, doc, 0, Rb, -1, 12))
	require.NoError(t, err)
	// A tapped hole from the bore wall at (6, 0, 5) out along +X, 8 mm deep from the tangent plane.
	// The wall of a bore rises toward the plane, so the material starts at x = √(R² − y²), below
	// the plane by R − √(R² − y²) = -(sag): the groove's lower limit is the negative sag, which
	// the overlap with [0, span] clips to the plane.
	const depth = 8.0
	out, err := decad.Hole(t.Context(), part, decad.HoleSpec{
		Face: decad.Faces(decad.Cylindrical()), At: r3.NewVec(Rb, 0, 5),
		Diameter: units.Millimeters(2 * tapRadius), Depth: decad.BlindHole{Depth: units.Millimeters(depth)},
		Thread: &decad.HoleThread{Pitch: units.Millimeters(tapPitch)}})
	require.NoError(t, err)
	wall := func(y float64) float64 { return math.Sqrt(Rb*Rb - y*y) }
	// The material along the hole axis runs from the wall at x = wall(y) (below the plane x = 6) to
	// x = 6 + depth: the bore removes depth + 6 − wall(y) of it.
	bore := chordWeighted(tapRadius, func(y float64) float64 { return 6 + depth - wall(y) })
	turns := (depth + 0.05*tapPitch) / tapPitch
	// A groove at depth t below the plane lies in material wherever t ≥ 0, since the wall is
	// above that level: the lower limit is the plane itself.
	thread := removedInternalOnWall(tapRadius, tapPitch, depth, turns, func(float64) float64 { return 0 })
	volume, err := out.Volume()
	require.NoError(t, err)
	lead := math.Atan(tapPitch / (2 * math.Pi * tapRadius))
	requireThreadVolumeWithin(t, volume, 40*40*10-math.Pi*Rb*Rb*10-bore-thread, thread*(1/math.Cos(lead)-1))
}

func TestHoleThreadInAThinRodIsTheBooleansRefusal(t *testing.T) {
	t.Parallel()
	// A rod this thin has a pair tolerance finer than the united bore and coil tool holds, so the
	// cut is the evaluator's own ErrUnsupported and leaves the rod live.
	doc := decad.New()
	rod := circleBodyAtZ(t, doc, 0, 6, 0, 16)
	_, err := decad.Hole(t.Context(), rod, decad.HoleSpec{
		Face: decad.Faces(decad.Cylindrical()), At: r3.NewVec(6, 0, 8),
		Diameter: units.Millimeters(2 * tapRadius), Depth: decad.BlindHole{Depth: units.Millimeters(5)},
		Thread: &decad.HoleThread{Pitch: units.Millimeters(tapPitch)}})
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.Equal(t, []*decad.Body{rod}, doc.Bodies())
}
