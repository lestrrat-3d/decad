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

// This file is agent-docs/emboss-design.md §8's public-surface tests for an Emboss on a
// cylindrical face.

// tangentSketch is a sketch on the plane x = at tangent to a cylinder about the z axis,
// its U along +Y and V along +Z so its normal is +X, holding the rectangle from (-halfY,
// z0 - zc) to (halfY, z0 + zc) about the point (at, 0, zc) in world terms.
func tangentSketch(t *testing.T, at, zc, halfY, halfZ float64) (*sketch.Sketch, []*sketch.Profile) {
	t.Helper()
	frame, err := r3.NewFrame(r3.NewVec(at, 0, zc), r3.NewVec(0, 1, 0), r3.NewVec(0, 0, 1))
	require.NoError(t, err)
	w := sketch.NewWorld()
	plane, err := w.CreatePlaneFromFrame(frame)
	require.NoError(t, err)
	s, err := w.CreateSketch(plane)
	require.NoError(t, err)
	rect := s.CreateRectangle(-halfY, -halfZ, halfY, halfZ)
	s.Fix(rect.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	return s, s.Profiles()
}

// tangentSketchAt is a sketch on the plane tangent to a cylinder about the z axis at angle
// theta from +X, its U across the plane and V along +Z, holding the rectangle halfLateral
// across and halfAxial along about the point of contact at height zc.
func tangentSketchAt(t *testing.T, radius, theta, zc, halfLateral, halfAxial float64) (*sketch.Sketch, []*sketch.Profile) {
	t.Helper()
	normal := r3.NewVec(math.Cos(theta), math.Sin(theta), 0)
	along := r3.NewVec(0, 0, 1)
	frame, err := r3.NewFrame(normal.Scale(radius).Add(r3.NewVec(0, 0, zc)), along.Cross(normal), along)
	require.NoError(t, err)
	require.InDelta(t, 1, frame.N().Dot(normal), 1e-12, "the sketch looks out of the cylinder")
	w := sketch.NewWorld()
	plane, err := w.CreatePlaneFromFrame(frame)
	require.NoError(t, err)
	s, err := w.CreateSketch(plane)
	require.NoError(t, err)
	rect := s.CreateRectangle(-halfLateral, -halfAxial, halfLateral, halfAxial)
	s.Fix(rect.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	return s, s.Profiles()
}

func TestEmbossRaisesAReliefOnARod(t *testing.T) {
	t.Parallel()
	const R, L, h, halfY, halfZ = 10.0, 16.0, 4.0, 6.0, 3.0
	doc := decad.New()
	rod := circleBodyAtZ(t, doc, 0, R, 0, L)
	s, profiles := tangentSketch(t, R, L/2, halfY, halfZ)
	out, err := decad.Emboss(t.Context(), rod, s, profiles, decad.Faces(decad.Cylindrical()),
		decad.EmbossRaised, units.Millimeters(h))
	require.NoError(t, err)
	require.Equal(t, []*decad.Body{out}, doc.Bodies())
	// The relief stands from the rod's wall, x = √(R² − y²), up to the plane x = R + h, over a
	// 12 mm wide, 6 mm long rectangle: its width wide enough that the wall has fallen 2 mm away.
	added := 2 * halfZ * chordWeightedFlat(halfY, func(y float64) float64 { return R + h - math.Sqrt(R*R-y*y) })
	volume, err := out.Volume()
	require.NoError(t, err)
	requireCylinderHoleVolume(t, volume, math.Pi*R*R*L+added)
	box, err := out.Bounds()
	require.NoError(t, err)
	require.InDelta(t, R+h, box.Max.X, 1e-6)
}

// chordWeightedFlat is ∫ f(y) dy over [-a, a].
func chordWeightedFlat(a float64, f func(y float64) float64) float64 {
	const steps = 200000
	step := 2 * a / steps
	total := 0.0
	for i := range steps {
		total += f(-a+(float64(i)+0.5)*step) * step
	}
	return total
}

func TestEmbossEngravesIntoARod(t *testing.T) {
	t.Parallel()
	const R, L, h, halfY, halfZ = 10.0, 16.0, 2.0, 3.0, 3.0
	rod := circleBodyAtZ(t, decad.New(), 0, R, 0, L)
	s, profiles := tangentSketchAt(t, R, math.Pi/4, L/2, halfY, halfZ)
	out, err := decad.Emboss(t.Context(), rod, s, profiles, decad.Faces(decad.Cylindrical()),
		decad.EmbossEngraved, units.Millimeters(h))
	require.NoError(t, err)
	// The engraving is cut h deep from the tangent plane: over the width the wall is below the
	// plane, so what is removed is the wall's x above the floor R − h.
	removed := 2 * halfZ * chordWeightedFlat(halfY, func(y float64) float64 { return math.Max(0, math.Sqrt(R*R-y*y)-(R-h)) })
	volume, err := out.Volume()
	require.NoError(t, err)
	requireCylinderHoleVolume(t, volume, math.Pi*R*R*L-removed)
}

func TestEmbossRaisesAReliefInABore(t *testing.T) {
	t.Parallel()
	const Rb, h, halfY, halfZ = 8.0, 3.0, 3.0, 2.0
	doc := decad.New()
	block := boxBody(t, doc, -20, -20, 20, 20, 10)
	part, err := decad.Cut(t.Context(), block, circleBodyAtZ(t, doc, 0, Rb, -1, 12))
	require.NoError(t, err)
	// The bore's wall looks toward the axis, so a relief on the plane tangent at x = 8 grows
	// into the bore, toward −x, from the wall x = √(Rb² − y²) to the plane's x = Rb − h. The sketch
	// plane has its normal along +X, so the relief is on its outward-from-the-material side.
	s, profiles := tangentSketch(t, Rb, 5, halfY, halfZ)
	out, err := decad.Emboss(t.Context(), part, s, profiles, decad.Faces(decad.Cylindrical()),
		decad.EmbossRaised, units.Millimeters(h))
	require.NoError(t, err)
	added := 2 * halfZ * chordWeightedFlat(halfY, func(y float64) float64 { return math.Sqrt(Rb*Rb-y*y) - (Rb - h) })
	volume, err := out.Volume()
	require.NoError(t, err)
	requireCylinderHoleVolume(t, volume, 40*40*10-math.Pi*Rb*Rb*10+added)
}

func TestEmbossOnACylinderRefusals(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	rod := circleBodyAtZ(t, doc, 0, 10, 0, 16)
	face := decad.Faces(decad.Cylindrical())
	emboss := func(at, halfY float64, opts ...decad.EmbossOption) error {
		s, profiles := tangentSketch(t, at, 8, halfY, 3)
		_, err := decad.Emboss(t.Context(), rod, s, profiles, face, decad.EmbossRaised, units.Millimeters(3), opts...)
		return err
	}
	require.ErrorIs(t, emboss(9, 3), decad.ErrDegenerate, "a plane inside the rod is not tangent")
	require.ErrorIs(t, emboss(11, 3), decad.ErrDegenerate, "a plane off the rod is not tangent")
	require.ErrorIs(t, emboss(10, 11), decad.ErrDegenerate, "a relief as wide as the rod")
	require.ErrorIs(t, emboss(10, 3, decad.WithEmbossRimFillet(units.Millimeters(0.5))), decad.ErrUnsupported)
	require.Equal(t, []*decad.Body{rod}, doc.Bodies(), "every refusal leaves the rod live")
}
