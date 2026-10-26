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

// This file is agent-docs/rib-design.md §7's and agent-docs/boss-design.md §5's
// public-surface tests for WithRibDraft and BossSpec.Draft.

func TestRibDraftThinsTheRibAsItRises(t *testing.T) {
	t.Parallel()
	for _, deg := range []float64{3, 5} {
		_, part := ribPart(t)
		s, lines := ribSketch(t, [4]float64{3, 20, 37, 20})
		out, err := decad.Rib(t.Context(), part, s, lines[0], units.Millimeters(2),
			decad.Distance{D: units.Millimeters(11), Dir: decad.Along},
			decad.WithRibOverrun(units.Millimeters(1)), decad.WithRibDraft(units.Degrees(deg)))
		require.NoError(t, err)
		// The sketch plane is at z = 4 and the pocket floor at z = 5. At the height t
		// above the plane the rib is 2 − 2·t·tan(angle) thick, and its 30 mm
		// between the pocket walls rises from t = 1 to t = 11. The ends, buried in the
		// walls, add nothing.
		tan := math.Tan(deg * math.Pi / 180)
		added := 30 * ((2*11 - 121*tan) - (2*1 - 1*tan))
		decadtest.MeasuresVolume(t, out, units.CubicMillimeters(ribPartVolume+added), decadtest.WithinRel(units.Scalar(1e-9)))
	}
}

func TestRibDraftNegativeThickensTheRib(t *testing.T) {
	t.Parallel()
	_, part := ribPart(t)
	s, lines := ribSketch(t, [4]float64{3, 20, 37, 20})
	out, err := decad.Rib(t.Context(), part, s, lines[0], units.Millimeters(2),
		decad.Distance{D: units.Millimeters(11), Dir: decad.Along},
		decad.WithRibOverrun(units.Millimeters(1)), decad.WithRibDraft(units.Degrees(-4)))
	require.NoError(t, err)
	tan := math.Tan(-4 * math.Pi / 180)
	added := 30 * ((2*11 - 121*tan) - (2*1 - 1*tan))
	decadtest.MeasuresVolume(t, out, units.CubicMillimeters(ribPartVolume+added), decadtest.WithinRel(units.Scalar(1e-9)))
	// 10 mm above the floor the rib is wider than its 2 mm: a point just outside
	// the straight outline is material.
	wider, err := out.LocatePoint(t.Context(), r3.NewVec(20, 20+1.15, 14), units.Millimeters(0.01))
	require.NoError(t, err)
	require.Equal(t, decad.PointInside, wider)
}

func TestRibDraftOfAWebAndLastWins(t *testing.T) {
	t.Parallel()
	_, part := ribPart(t)
	s, lines := ribSketch(t, [4]float64{3, 12, 37, 12}, [4]float64{3, 28, 37, 28})
	out, err := decad.Web(t.Context(), part, s, lines, units.Millimeters(2),
		decad.Distance{D: units.Millimeters(11), Dir: decad.Along},
		decad.WithRibOverrun(units.Millimeters(1)), decad.WithRibDraft(units.Degrees(10)), decad.WithRibDraft(units.Degrees(5)))
	require.NoError(t, err)
	tan := math.Tan(5 * math.Pi / 180)
	added := 2 * 30 * ((2*11 - 121*tan) - (2*1 - 1*tan))
	decadtest.MeasuresVolume(t, out, units.CubicMillimeters(ribPartVolume+added), decadtest.WithinRel(units.Scalar(1e-9)))
}

func TestRibDraftRefusals(t *testing.T) {
	t.Parallel()
	for name, tc := range map[string]struct {
		draft units.Value
		want  error
	}{
		"a right angle":       {units.Degrees(90), decad.ErrDegenerate},
		"a draft as a length": {units.Millimeters(1), decad.ErrUnitKind},
	} {
		doc, part := ribPart(t)
		s, lines := ribSketch(t, [4]float64{3, 20, 37, 20})
		_, err := decad.Rib(t.Context(), part, s, lines[0], units.Millimeters(2),
			decad.Distance{D: units.Millimeters(11), Dir: decad.Along}, decad.WithRibDraft(tc.draft))
		require.ErrorIs(t, err, tc.want, name)
		require.Equal(t, []*decad.Body{part}, doc.Bodies(), "%s: a refused rib leaves the part live and alone", name)
	}
}

func TestBossDraftNarrowsTheBossTowardItsTop(t *testing.T) {
	t.Parallel()
	for _, deg := range []float64{2, 6} {
		_, plate := bossPlate(t)
		spec := bossSpec(plate, true, nil)
		spec.Draft = units.Degrees(deg)
		out, err := decad.Boss(t.Context(), plate, spec)
		require.NoError(t, err)
		// A frustum of radius 4 at the plate's face and 4 − 8·tan(angle) at its top, 8 mm tall.
		tan := math.Tan(deg * math.Pi / 180)
		decadtest.MeasuresVolume(t, out, units.CubicMillimeters(bossPlateVolume+cylinderFrustum(4, 4-8*tan, 8)),
			decadtest.WithinRel(units.Scalar(1e-3)))
		decadtest.MeasuresBounds(t, out, r3.Vec{X: -20, Y: -20}, r3.Vec{X: 20, Y: 20, Z: 13}, decadtest.Within(units.Millimeters(1e-6)))
	}
}

func TestBossDraftNegativeWidensIt(t *testing.T) {
	t.Parallel()
	_, plate := bossPlate(t)
	spec := bossSpec(plate, true, nil)
	spec.Draft = units.Degrees(-3)
	out, err := decad.Boss(t.Context(), plate, spec)
	require.NoError(t, err)
	tan := math.Tan(-3 * math.Pi / 180)
	decadtest.MeasuresVolume(t, out, units.CubicMillimeters(bossPlateVolume+cylinderFrustum(4, 4-8*tan, 8)),
		decadtest.WithinRel(units.Scalar(1e-3)))
}

func TestBossDraftKeepsThePilotInsideTheNarrowedTop(t *testing.T) {
	t.Parallel()
	_, plate := bossPlate(t)
	// A 8 mm boss drafted 10 degrees is 8 − 16·tan(10) = 5.18 mm across at its top,
	// so a 6 mm pilot would break through the wall there.
	spec := bossSpec(plate, true, &decad.BossBore{Diameter: units.Millimeters(6), Depth: units.Millimeters(4)})
	spec.Draft = units.Degrees(10)
	doc := plate
	_, err := decad.Boss(t.Context(), doc, spec)
	require.ErrorIs(t, err, decad.ErrDegenerate)
	// A 3 mm pilot fits.
	_, plate2 := bossPlate(t)
	spec2 := bossSpec(plate2, true, &decad.BossBore{Diameter: units.Millimeters(3), Depth: units.Millimeters(4)})
	spec2.Draft = units.Degrees(10)
	_, err = decad.Boss(t.Context(), plate2, spec2)
	require.NoError(t, err)
}

func TestBossDraftRefusals(t *testing.T) {
	t.Parallel()
	for name, tc := range map[string]struct {
		draft units.Value
		want  error
	}{
		"a right angle":               {units.Degrees(90), decad.ErrDegenerate},
		"a draft that closes the top": {units.Degrees(40), decad.ErrUnsupported},
		"a draft as a length":         {units.Millimeters(1), decad.ErrUnitKind},
	} {
		doc, plate := bossPlate(t)
		spec := bossSpec(plate, true, nil)
		spec.Draft = tc.draft
		_, err := decad.Boss(t.Context(), plate, spec)
		require.ErrorIs(t, err, tc.want, name)
		require.Equal(t, []*decad.Body{plate}, doc.Bodies(), "%s: a refused boss leaves the plate live and alone", name)
	}
}
