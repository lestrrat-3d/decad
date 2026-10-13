package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func obliqueCylinderPlane(t *testing.T, origin, u r3.Vec) r3.Frame {
	t.Helper()
	frame, err := r3.NewFrame(origin, u, r3.NewVec(0, 1, 1))
	require.NoError(t, err)
	return frame
}

func TestSectionObliqueCylinderFeedsVerifiedExtrude(t *testing.T) {
	doc, source := circleSectionSourceSized(t, 0, 0, 2, 10)
	frame := obliqueCylinderPlane(t, r3.NewVec(0, 0, 5), r3.NewVec(1, 0, 0))
	section, profiles, err := source.Section(t.Context(), frame)
	require.NoError(t, err)
	require.Len(t, profiles, 1)
	require.True(t, profiles[0].Valid)
	require.GreaterOrEqual(t, len(profiles[0].Outer), 32)

	var maxU, maxV float64
	for _, point := range section.Points() {
		if !point.IsReference() {
			continue
		}
		maxU = math.Max(maxU, math.Abs(point.X()))
		maxV = math.Max(maxV, math.Abs(point.Y()))
	}
	require.InDelta(t, 2.0, maxU, 1e-12)
	require.InDelta(t, 2*math.Sqrt2, maxV, 1e-12)
	_, err = decad.MeasureProfile(section, profiles[0])
	require.ErrorIs(t, err, decad.ErrUnsupported)

	child, err := decad.Extrude(t.Context(), decad.New(), section, profiles[0], decad.Distance{
		D: units.Millimeters(2), Dir: decad.Along,
	})
	require.NoError(t, err)
	volume, err := child.Volume()
	require.NoError(t, err)
	require.Equal(t, decad.Approximate, volume.Exactness)
	require.LessOrEqual(t, math.Abs(volume.Value.Base()-8*math.Pi*math.Sqrt2), volume.Bound.Base())
	require.Less(t, volume.Bound.Base(), 5.0)
	mesh, err := child.Tessellate(t.Context(), units.Millimeters(0.1),
		decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
	require.Less(t, mesh.Bound().Base(), 0.1)
	require.Equal(t, []*decad.Body{source}, doc.Bodies())
}

func TestSectionObliqueCylinderPlacementAndReverseAxis(t *testing.T) {
	_, source := circleSectionSourceSized(t, 0, 0, 2, 10)
	placement, err := r3.FromBasis(r3.Basis{
		EX: r3.Vec{X: 1}, EY: r3.Vec{Y: 1}, EZ: r3.Vec{Z: 1},
	}, r3.NewVec(3, 4, 7))
	require.NoError(t, err)
	placed, err := decad.Transform(t.Context(), source, placement)
	require.NoError(t, err)
	frame := obliqueCylinderPlane(t, r3.NewVec(3, 4, 12), r3.NewVec(-1, 0, 0))
	section, profiles, err := placed.Section(t.Context(), frame)
	require.NoError(t, err)
	require.Len(t, profiles, 1)
	child, err := decad.Extrude(t.Context(), decad.New(), section, profiles[0], decad.Distance{
		D: units.Millimeters(2), Dir: decad.Along,
	})
	require.NoError(t, err)
	mesh, err := child.Tessellate(t.Context(), units.Millimeters(0.1),
		decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
}

func TestSectionObliqueCylinderCapCrossingFeedsVerifiedExtrude(t *testing.T) {
	_, source := circleSectionSourceSized(t, 0, 0, 2, 10)
	frame := obliqueCylinderPlane(t, r3.NewVec(0, 0, 9), r3.NewVec(1, 0, 0))
	section, profiles, err := source.Section(t.Context(), frame)
	require.NoError(t, err)
	require.Len(t, profiles, 1)
	require.True(t, profiles[0].Valid)
	child, err := decad.Extrude(t.Context(), decad.New(), section, profiles[0], decad.Distance{
		D: units.Millimeters(2), Dir: decad.Along,
	})
	require.NoError(t, err)
	volume, err := child.Volume()
	require.NoError(t, err)
	want := 2 * math.Sqrt2 * (8*math.Pi/3 + math.Sqrt(3))
	require.LessOrEqual(t, math.Abs(volume.Value.Base()-want), volume.Bound.Base())
	mesh, err := child.Tessellate(t.Context(), units.Millimeters(0.1),
		decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
	require.Less(t, mesh.Bound().Base(), 0.1)
	t.Logf("cap crossing: child volume %.12f, volume bound %.12f, mesh bound %.12f",
		volume.Value.Base(), volume.Bound.Base(), mesh.Bound().Base())
}

func TestSectionObliqueCylinderRefusesTwoCapCrossing(t *testing.T) {
	_, source := circleSectionSourceSized(t, 0, 0, 2, 2)
	frame := obliqueCylinderPlane(t, r3.NewVec(0, 0, 1), r3.NewVec(1, 0, 0))
	section, profiles, err := source.Section(t.Context(), frame)
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.Nil(t, section)
	require.Nil(t, profiles)
}

func TestSectionObliqueCylinderBottomCapCrossingFeedsVerifiedExtrude(t *testing.T) {
	_, source := circleSectionSourceSized(t, 0, 0, 2, 10)
	frame := obliqueCylinderPlane(t, r3.NewVec(0, 0, 1), r3.NewVec(1, 0, 0))
	section, profiles, err := source.Section(t.Context(), frame)
	require.NoError(t, err)
	require.Len(t, profiles, 1)
	child, err := decad.Extrude(t.Context(), decad.New(), section, profiles[0], decad.Distance{
		D: units.Millimeters(2), Dir: decad.Along,
	})
	require.NoError(t, err)
	mesh, err := child.Tessellate(t.Context(), units.Millimeters(0.1),
		decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
}

func TestSectionObliqueCylinderPlacedCapCrossingFeedsVerifiedExtrude(t *testing.T) {
	_, source := circleSectionSourceSized(t, 1, -2, 2, 10)
	placement, err := r3.FromBasis(r3.Basis{
		EX: r3.Vec{X: 1}, EY: r3.Vec{Y: 1}, EZ: r3.Vec{Z: 1},
	}, r3.NewVec(3, 4, 7))
	require.NoError(t, err)
	placed, err := decad.Transform(t.Context(), source, placement)
	require.NoError(t, err)
	frame := obliqueCylinderPlane(t, r3.NewVec(4, 2, 16), r3.NewVec(1, 0, 0))
	section, profiles, err := placed.Section(t.Context(), frame)
	require.NoError(t, err)
	require.Len(t, profiles, 1)
	child, err := decad.Extrude(t.Context(), decad.New(), section, profiles[0], decad.Distance{
		D: units.Millimeters(2), Dir: decad.Along,
	})
	require.NoError(t, err)
	volume, err := child.Volume()
	require.NoError(t, err)
	want := 2 * math.Sqrt2 * (8*math.Pi/3 + math.Sqrt(3))
	require.LessOrEqual(t, math.Abs(volume.Value.Base()-want), volume.Bound.Base())
	mesh, err := child.Tessellate(t.Context(), units.Millimeters(0.1),
		decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
}

func TestSectionObliqueCylinderRefusesCapTangent(t *testing.T) {
	_, source := circleSectionSourceSized(t, 0, 0, 2, 10)
	frame := obliqueCylinderPlane(t, r3.NewVec(0, 0, 8), r3.NewVec(1, 0, 0))
	section, profiles, err := source.Section(t.Context(), frame)
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.Nil(t, section)
	require.Nil(t, profiles)
}
