package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file is agent-docs/emboss-design.md §7's public-surface tests for
// WithEmbossRimFillet, on the plate of emboss_test.go.

// embossCircle returns a sketch on the plane z = 10 with one circle of radius 4
// at (20, 15), and its profile.
func embossCircle(t *testing.T) (*sketch.Sketch, []*sketch.Profile) {
	t.Helper()
	w := sketch.NewWorld()
	plane, err := w.CreateOffsetPlane(w.XY(), 10)
	require.NoError(t, err)
	s, err := w.CreateSketch(plane)
	require.NoError(t, err)
	c := s.CreatePoint(20, 15)
	s.Fix(c)
	s.CreateCircle(c, 4)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	require.Len(t, s.Profiles(), 1)
	return s, s.Profiles()
}

// roundedRim is the volume a fillet of radius r takes off a circular edge of
// radius 4 between a wall and a flat, for a round of 0.5: at distance x from the wall the fillet
// cuts r − sqrt(r² − (r−x)²) deep, revolved about the axis at 4 ∓ x. inward is
// true when the material lies inside the circle, as a raised relief's does.
func roundedRim(inward bool) float64 {
	const R, r = 4.0, 0.5 // the relief's radius and the round's
	const steps = 200000
	step := r / steps
	total := 0.0
	for i := range steps {
		x := (float64(i) + 0.5) * step
		radius := R + x
		if inward {
			radius = R - x
		}
		total += 2 * math.Pi * radius * (r - math.Sqrt(r*r-(r-x)*(r-x))) * step
	}
	return total
}

func TestEmbossRimFilletRoundsARaisedRelief(t *testing.T) {
	t.Parallel()
	doc, plate := embossPlate(t)
	s, profiles := embossCircle(t)
	out, err := decad.Emboss(t.Context(), plate, s, profiles, embossFace(true, plate), decad.EmbossRaised, units.Millimeters(2),
		decad.WithEmbossRimFillet(units.Millimeters(0.5)))
	require.NoError(t, err)
	require.Len(t, doc.Bodies(), 1, "the plate, the relief and the trial all leave the document")
	require.True(t, doc.Bodies()[0] == out)
	want := embossPlateVolume + math.Pi*16*2 - roundedRim(true)
	v, err := out.Volume()
	require.NoError(t, err)
	require.InDelta(t, want, v.Value.Base(), 1e-3*roundedRim(true)+v.Bound.Base())
	var tori int
	for _, f := range out.Faces() {
		if _, ok := f.Surface().(decad.Torus); ok {
			tori++
		}
	}
	require.Equal(t, 1, tori, "one round at the relief's rim")
}

func TestEmbossRimFilletRoundsTheMouthOfAnEngraving(t *testing.T) {
	t.Parallel()
	_, plate := embossPlate(t)
	s, profiles := embossCircle(t)
	out, err := decad.Emboss(t.Context(), plate, s, profiles, embossFace(true, plate), decad.EmbossEngraved, units.Millimeters(3),
		decad.WithEmbossRimFillet(units.Millimeters(0.5)))
	require.NoError(t, err)
	want := embossPlateVolume - math.Pi*16*3 - roundedRim(false)
	v, err := out.Volume()
	require.NoError(t, err)
	require.InDelta(t, want, v.Value.Base(), 1e-3*roundedRim(false)+v.Bound.Base())
}

func TestEmbossRimFilletRefusalsLeaveTheBodyLive(t *testing.T) {
	t.Parallel()
	doc, plate := embossPlate(t)
	s, profiles := embossCircle(t)
	face := embossFace(true, plate)
	_, err := decad.Emboss(t.Context(), plate, s, profiles, face, decad.EmbossRaised, units.Millimeters(2),
		decad.WithEmbossRimFillet(units.Millimeters(-1)))
	require.ErrorIs(t, err, decad.ErrNegativeMagnitude)
	_, err = decad.Emboss(t.Context(), plate, s, profiles, face, decad.EmbossRaised, units.Millimeters(2),
		decad.WithEmbossRimFillet(units.Degrees(1)))
	require.ErrorIs(t, err, decad.ErrUnitKind)
	// A round wider than the relief itself is the fillet's own refusal, and the plate stays live.
	_, err = decad.Emboss(t.Context(), plate, s, profiles, face, decad.EmbossRaised, units.Millimeters(2),
		decad.WithEmbossRimFillet(units.Millimeters(5)))
	require.Error(t, err)
	require.Len(t, doc.Bodies(), 1)
	require.True(t, doc.Bodies()[0] == plate)
}
