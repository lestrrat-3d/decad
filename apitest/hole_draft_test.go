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

// This file is agent-docs/hole-design.md §9's and agent-docs/boss-design.md §5's
// public-surface tests for HoleSpec.Draft and BossBore.Draft, on the plate of
// hole_test.go. A bore of radius 3 at the face that leans by θ has the radius
// 3 − t·tan θ at the depth t, so it removes a frustum.

func draftSpec(plate *decad.Body, depth decad.HoleDepth, draft units.Value) decad.HoleSpec {
	return decad.HoleSpec{
		Face:     capFace(true, plate),
		At:       r3.Vec{X: 10, Y: 10, Z: 10},
		Diameter: units.Millimeters(6),
		Depth:    depth,
		Draft:    draft,
	}
}

func TestHoleDraftBlindNarrowsTowardTheBottom(t *testing.T) {
	t.Parallel()
	for _, deg := range []float64{2, 5} {
		_, plate := drillPlate(t)
		out, err := decad.Hole(t.Context(), plate, draftSpec(plate, decad.BlindHole{Depth: units.Millimeters(6)}, units.Degrees(deg)))
		require.NoError(t, err)
		tan := math.Tan(deg * math.Pi / 180)
		decadtest.MeasuresVolume(t, out, units.CubicMillimeters(drillPlateVolume-cylinderFrustum(3, 3-6*tan, 6)),
			decadtest.WithinRel(units.Scalar(1e-3)))
		// 5 mm down the wall has moved in by 5·tan: a point between that wall and
		// the straight one is material, and a point inside the bore is empty.
		// Body.LocatePoint declines a point this close to a faceted wall, so the probe
		// reads the held mesh, whose parity is exact; the wall lies far from the probe.
		moved := 5 * tan
		held := heldMesh(t, out)
		wall, err := held.LocatePoint(t.Context(), r3.NewVec(10+3-moved/2, 10, 5))
		require.NoError(t, err)
		require.Equal(t, decad.PointInside, wall, "%v degrees: material where a straight bore would be empty", deg)
		bore, err := held.LocatePoint(t.Context(), r3.NewVec(10, 10, 5))
		require.NoError(t, err)
		require.Equal(t, decad.PointOutside, bore)
	}
}

func TestHoleDraftThroughAndNegative(t *testing.T) {
	t.Parallel()
	_, plate := drillPlate(t)
	out, err := decad.Hole(t.Context(), plate, draftSpec(plate, decad.ThroughHole{}, units.Degrees(2)))
	require.NoError(t, err)
	tan := math.Tan(2 * math.Pi / 180)
	decadtest.MeasuresVolume(t, out, units.CubicMillimeters(drillPlateVolume-cylinderFrustum(3, 3-10*tan, 10)),
		decadtest.WithinRel(units.Scalar(1e-3)))
	_, plate = drillPlate(t)
	out, err = decad.Hole(t.Context(), plate, draftSpec(plate, decad.BlindHole{Depth: units.Millimeters(6)}, units.Degrees(-4)))
	require.NoError(t, err)
	tan = math.Tan(-4 * math.Pi / 180)
	decadtest.MeasuresVolume(t, out, units.CubicMillimeters(drillPlateVolume-cylinderFrustum(3, 3-6*tan, 6)),
		decadtest.WithinRel(units.Scalar(1e-3)))
}

func TestHoleDraftWithACounterbore(t *testing.T) {
	t.Parallel()
	_, plate := drillPlate(t)
	spec := draftSpec(plate, decad.BlindHole{Depth: units.Millimeters(6)}, units.Degrees(3))
	spec.Head = decad.Counterbore{Diameter: units.Millimeters(8), Depth: units.Millimeters(2)}
	out, err := decad.Hole(t.Context(), plate, spec)
	require.NoError(t, err)
	// The recess is a straight cylinder of radius 4, 2 mm deep. Below it the bore, leaning
	// from the face, has radius 3 − 2·tan at depth 2 and 3 − 6·tan at its bottom.
	tan := math.Tan(3 * math.Pi / 180)
	want := drillPlateVolume - math.Pi*16*2 - cylinderFrustum(3-2*tan, 3-6*tan, 4)
	decadtest.MeasuresVolume(t, out, units.CubicMillimeters(want), decadtest.WithinRel(units.Scalar(1e-3)))
}

func TestHoleDraftRefusals(t *testing.T) {
	t.Parallel()
	type edit func(*decad.HoleSpec)
	for name, tc := range map[string]struct {
		edit edit
		want error
	}{
		"a right angle":  {func(s *decad.HoleSpec) { s.Draft = units.Degrees(90) }, decad.ErrDegenerate},
		"a draft length": {func(s *decad.HoleSpec) { s.Draft = units.Millimeters(1) }, decad.ErrUnitKind},
		"a shaped hole": {func(s *decad.HoleSpec) {
			s.Diameter = units.Value{}
			s.Shape = decad.SlotHole{Length: units.Millimeters(8), Width: units.Millimeters(4), Angle: units.Degrees(0)}
		}, decad.ErrDegenerate},
		"a tapped hole": {func(s *decad.HoleSpec) {
			s.Thread = &decad.HoleThread{Pitch: units.Millimeters(0.5)}
		}, decad.ErrDegenerate},
		"a draft that closes the bore": {func(s *decad.HoleSpec) { s.Draft = units.Degrees(40) }, decad.ErrUnsupported},
	} {
		doc, plate := drillPlate(t)
		spec := draftSpec(plate, decad.BlindHole{Depth: units.Millimeters(6)}, units.Degrees(3))
		tc.edit(&spec)
		_, err := decad.Hole(t.Context(), plate, spec)
		require.ErrorIs(t, err, tc.want, name)
		require.Equal(t, []*decad.Body{plate}, doc.Bodies(), "%s: a refused hole leaves the plate live and alone", name)
	}
	doc := decad.New()
	rod := circleBodyAtZ(t, doc, 0, 5, 0, 20)
	_, err := decad.Hole(t.Context(), rod, decad.HoleSpec{
		Face: decad.Faces(decad.Cylindrical()), At: r3.NewVec(5, 0, 10),
		Diameter: units.Millimeters(3), Depth: decad.ThroughHole{}, Draft: units.Degrees(2)})
	require.ErrorIs(t, err, decad.ErrUnsupported, "a drafted hole on a cylindrical face")
	require.Equal(t, []*decad.Body{rod}, doc.Bodies())
}

func TestBossPilotDraft(t *testing.T) {
	t.Parallel()
	_, plate := bossPlate(t)
	out, err := decad.Boss(t.Context(), plate, bossSpec(plate, true, &decad.BossBore{
		Diameter: units.Millimeters(3), Depth: units.Millimeters(6), Draft: units.Degrees(3)}))
	require.NoError(t, err)
	tan := math.Tan(3 * math.Pi / 180)
	want := bossPlateVolume + math.Pi*16*8 - cylinderFrustum(1.5, 1.5-6*tan, 6)
	decadtest.MeasuresVolume(t, out, units.CubicMillimeters(want), decadtest.WithinRel(units.Scalar(1e-3)))
	_, plate = bossPlate(t)
	_, err = decad.Boss(t.Context(), plate, bossSpec(plate, true, &decad.BossBore{
		Diameter: units.Millimeters(3), Depth: units.Millimeters(6), Draft: units.Degrees(3),
		Thread: &decad.HoleThread{Pitch: units.Millimeters(0.5)}}))
	require.ErrorIs(t, err, decad.ErrDegenerate, "a tapped pilot is straight")
}
