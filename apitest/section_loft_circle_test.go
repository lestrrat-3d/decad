package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestSectionDirectLoftAnnulusFeedsExtrude(t *testing.T) {
	source := directLoftCircles(t, true, 1, 2, 1, 0)
	require.NoError(t, decad.Remove(source))
	frame, err := r3.NewFrame(r3.NewVec(0, 0, 1), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	section, profiles, err := source.Section(t.Context(), frame)
	require.NoError(t, err)
	require.Len(t, profiles, 1)
	require.True(t, profiles[0].Valid)
	require.Len(t, profiles[0].Outer, 1)
	require.Len(t, profiles[0].Holes, 1)
	require.Len(t, profiles[0].Holes[0], 1)
	measured, err := decad.MeasureProfile(section, profiles[0])
	require.NoError(t, err)
	area, err := measured.Area()
	require.NoError(t, err)
	require.LessOrEqual(t, math.Abs(area.Value.Base()-13.75*math.Pi), area.Bound.Base()+1e-12)
	child, err := decad.Extrude(t.Context(), decad.New(), section, profiles[0],
		decad.Distance{D: units.Millimeters(2), Dir: decad.Along})
	require.NoError(t, err)
	volume, err := child.Volume()
	require.NoError(t, err)
	require.LessOrEqual(t, math.Abs(volume.Value.Base()-27.5*math.Pi), volume.Bound.Base()+1e-12)
	mesh, err := child.Tessellate(t.Context(), units.Millimeters(0.1), decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
}

func TestSectionDirectLoftRectangularOuterFeedsExtrude(t *testing.T) {
	source := directLoftCircles(t, false, 1, 2, 0, 0)
	frame, err := r3.NewFrame(r3.NewVec(0, 0, 1), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	section, profiles, err := source.Section(t.Context(), frame)
	require.NoError(t, err)
	require.Len(t, profiles, 1)
	require.Len(t, profiles[0].Outer, 4)
	require.Len(t, profiles[0].Holes, 1)
	measured, err := decad.MeasureProfile(section, profiles[0])
	require.NoError(t, err)
	area, err := measured.Area()
	require.NoError(t, err)
	require.LessOrEqual(t, math.Abs(area.Value.Base()-(100-2.25*math.Pi)), area.Bound.Base()+1e-12)
	child, err := decad.Extrude(t.Context(), decad.New(), section, profiles[0],
		decad.Distance{D: units.Millimeters(2), Dir: decad.Along})
	require.NoError(t, err)
	volume, err := child.Volume()
	require.NoError(t, err)
	require.LessOrEqual(t, math.Abs(volume.Value.Base()-(200-4.5*math.Pi)), volume.Bound.Base()+1e-12)
	mesh, err := child.Tessellate(t.Context(), units.Millimeters(0.1), decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
}

func TestSectionDirectLoftAnnulusRefusesCapAndTilt(t *testing.T) {
	source := directLoftCircles(t, true, 1, 2, 1, 0)
	for _, tc := range []struct {
		name   string
		origin r3.Vec
		u, v   r3.Vec
	}{
		{"bottom cap", r3.NewVec(0, 0, 0), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0)},
		{"top cap", r3.NewVec(0, 0, 2), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0)},
		{"nonexact circle interpolation", r3.NewVec(0, 0, 0.2), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0)},
		{"tilted section", r3.NewVec(0, 0, 1), r3.NewVec(1, 0, 1), r3.NewVec(0, 1, 0)},
	} {
		t.Run(tc.name, func(t *testing.T) {
			frame, err := r3.NewFrame(tc.origin, tc.u, tc.v)
			require.NoError(t, err)
			section, profiles, err := source.Section(t.Context(), frame)
			require.ErrorIs(t, err, decad.ErrUnsupported)
			require.Nil(t, section)
			require.Nil(t, profiles)
		})
	}
}

func requireDirectLoftSectionUnsupported(t *testing.T, body *decad.Body, origin r3.Vec) {
	t.Helper()
	frame, err := r3.NewFrame(origin, r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	section, profiles, err := body.Section(t.Context(), frame)
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.Nil(t, section)
	require.Nil(t, profiles)
}

func TestSectionDirectLoftRefusesPlacementAndShiftedOrigin(t *testing.T) {
	source := directLoftCircles(t, true, 1, 2, 1, 0)
	requireDirectLoftSectionUnsupported(t, source, r3.NewVec(1, 0, 1))
	move, err := r3.Translation(r3.NewVec(10, 0, 0))
	require.NoError(t, err)
	placed, err := decad.Transform(t.Context(), source, move)
	require.NoError(t, err)
	requireDirectLoftSectionUnsupported(t, placed, r3.NewVec(0, 0, 1))
}

func TestSectionDirectLoftRefusesNonExactCenter(t *testing.T) {
	source := directLoftCircles(t, false, 1, 1, 3, 0)
	requireDirectLoftSectionUnsupported(t, source, r3.NewVec(0, 0, 0.2))
}

func TestSectionDirectLoftRefusesHoleFreeAndCutSources(t *testing.T) {
	bottom, bottomProfile, top, topProfile := loftSquaresAt(t, r3.NewVec(0, 0, 0), 5, 5, 2)
	doc := decad.New()
	blank, err := doc.Loft(t.Context(), bottom, bottomProfile, top, topProfile)
	require.NoError(t, err)
	requireDirectLoftSectionUnsupported(t, blank, r3.NewVec(0, 0, 1))

	tool := verticalCylinder(t, doc, 0, 0, 1)
	cut, err := decad.Cut(t.Context(), blank, tool)
	require.NoError(t, err)
	faces, err := decad.Faces(decad.FaceCreatedBy(decad.CapStart(cut))).Exactly(1).SelectFaces(cut)
	require.NoError(t, err)
	require.Len(t, faces[0].Loops(), 2)
	require.IsType(t, decad.Circle3{}, faces[0].Loops()[1].Edges()[0].Curve())
	requireDirectLoftSectionUnsupported(t, cut, r3.NewVec(0, 0, 1))
}
