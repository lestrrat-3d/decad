package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/decadtest"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file is agent-docs/emboss-design.md §6's public-surface tests for
// WithEmbossDraft, on the plate of emboss_test.go with a 10 x 10 mm square relief
// 3 mm deep or tall. A wall that leans by an angle θ moves in by t·tan θ at the
// depth t, so a square of side L has the side L − 2t·tan θ there and the relief
// the volume of ∫ (L − 2t·tan θ)² dt over its height.

// draftedSquare is that volume for a square with the given side and a relief of
// the given height, where a is the factor 2·tan θ.
func draftedSquare(side, height, a float64) float64 {
	if a == 0 {
		return side * side * height
	}
	inner := side - height*a
	return (side*side*side - inner*inner*inner) / (3 * a)
}

func TestEmbossDraftRaisedNarrowsTowardTheTop(t *testing.T) {
	t.Parallel()
	for _, deg := range []float64{3, 5, 10} {
		_, plate := embossPlate(t)
		s, profiles := embossSketch(t, 10, [4]float64{10, 10, 20, 20})
		out, err := decad.Emboss(t.Context(), plate, s, profiles, embossFace(true, plate), decad.EmbossRaised, units.Millimeters(3),
			decad.WithEmbossDraft(units.Degrees(deg)))
		require.NoError(t, err)
		a := 2 * math.Tan(deg*math.Pi/180)
		decadtest.MeasuresVolume(t, out, units.CubicMillimeters(embossPlateVolume+draftedSquare(10, 3, a)),
			decadtest.WithinRel(units.Scalar(1e-9)))
		decadtest.MeasuresBounds(t, out, r3.NewVec(0, 0, 0), r3.NewVec(40, 30, 13))
	}
}

func TestEmbossDraftNegativeWidensTowardTheTop(t *testing.T) {
	t.Parallel()
	_, plate := embossPlate(t)
	s, profiles := embossSketch(t, 10, [4]float64{10, 10, 20, 20})
	out, err := decad.Emboss(t.Context(), plate, s, profiles, embossFace(true, plate), decad.EmbossRaised, units.Millimeters(3),
		decad.WithEmbossDraft(units.Degrees(-5)))
	require.NoError(t, err)
	a := -2 * math.Tan(5*math.Pi/180)
	decadtest.MeasuresVolume(t, out, units.CubicMillimeters(embossPlateVolume+draftedSquare(10, 3, a)),
		decadtest.WithinRel(units.Scalar(1e-9)))
}

func TestEmbossDraftEngravedNarrowsTowardTheFloor(t *testing.T) {
	t.Parallel()
	_, plate := embossPlate(t)
	s, profiles := embossSketch(t, 10, [4]float64{10, 10, 20, 20})
	out, err := decad.Emboss(t.Context(), plate, s, profiles, embossFace(true, plate), decad.EmbossEngraved, units.Millimeters(3),
		decad.WithEmbossDraft(units.Degrees(5)))
	require.NoError(t, err)
	a := 2 * math.Tan(5*math.Pi/180)
	decadtest.MeasuresVolume(t, out, units.CubicMillimeters(embossPlateVolume-draftedSquare(10, 3, a)),
		decadtest.WithinRel(units.Scalar(1e-9)))
	// 3 mm down, the pocket's walls have moved in by 3·tan 5 degrees: a point
	// just inside the pocket's straight outline there is material.
	moved := 3 * math.Tan(5*math.Pi/180)
	inside, err := out.LocatePoint(t.Context(), r3.NewVec(10+moved/2, 15, 7.2), units.Millimeters(0.01))
	require.NoError(t, err)
	require.Equal(t, decad.PointInside, inside, "the drafted wall leaves material where a straight pocket would be empty")
	empty, err := out.LocatePoint(t.Context(), r3.NewVec(15, 15, 7.2), units.Millimeters(0.01))
	require.NoError(t, err)
	require.Equal(t, decad.PointOutside, empty)
}

func TestEmbossDraftZeroIsAStraightWall(t *testing.T) {
	t.Parallel()
	_, plate := embossPlate(t)
	s, profiles := embossSketch(t, 10, [4]float64{10, 10, 20, 20})
	out, err := decad.Emboss(t.Context(), plate, s, profiles, embossFace(true, plate), decad.EmbossRaised, units.Millimeters(3),
		decad.WithEmbossDraft(units.Degrees(0)))
	require.NoError(t, err)
	decadtest.MeasuresVolume(t, out, units.CubicMillimeters(embossPlateVolume+300), decadtest.WithinRel(units.Scalar(1e-9)))
}

func TestEmbossDraftRefusals(t *testing.T) {
	t.Parallel()
	for name, tc := range map[string]struct {
		opt  decad.EmbossOption
		want error
	}{
		"a right angle": {decad.WithEmbossDraft(units.Degrees(90)), decad.ErrDegenerate},
		"a nil option":  {nil, decad.ErrDegenerate},
		"a length":      {decad.WithEmbossDraft(units.Millimeters(1)), decad.ErrUnitKind},
	} {
		doc, plate := embossPlate(t)
		s, profiles := embossSketch(t, 10, [4]float64{10, 10, 20, 20})
		_, err := decad.Emboss(t.Context(), plate, s, profiles, embossFace(true, plate), decad.EmbossRaised, units.Millimeters(3), tc.opt)
		require.ErrorIs(t, err, tc.want, name)
		require.Equal(t, []*decad.Body{plate}, doc.Bodies(), "%s: a refused emboss leaves the plate live and alone", name)
	}
}
