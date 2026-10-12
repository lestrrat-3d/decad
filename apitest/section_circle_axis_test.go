package apitest_test

import (
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func circleSectionSource(t *testing.T, u, v float64) (*decad.Document, *decad.Body) {
	return circleSectionSourceSized(t, u, v, 1, 5)
}

func circleSectionSourceSized(t *testing.T, u, v, radius, height float64) (*decad.Document, *decad.Body) {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	center := s.CreatePoint(u, v)
	s.Fix(center)
	s.CreateCircle(center, radius)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	require.Len(t, s.Profiles(), 1)
	doc := decad.New()
	body, err := doc.Extrude(s, s.Profiles()[0], decad.Distance{D: units.Millimeters(height), Dir: decad.Along})
	require.NoError(t, err)
	return doc, body
}

func cardinalAxisSectionPlane(t *testing.T, origin, u r3.Vec) r3.Frame {
	t.Helper()
	frame, err := r3.NewFrame(origin, u, r3.NewVec(0, 0, 1))
	require.NoError(t, err)
	return frame
}

func verifyOffsetCylinderSection(t *testing.T, source *decad.Body, frame r3.Frame,
	halfChord, height float64) *decad.Body {
	t.Helper()
	section, profiles, err := source.Section(t.Context(), frame)
	require.NoError(t, err)
	require.Len(t, profiles, 1)
	require.True(t, profiles[0].Valid)
	require.Len(t, profiles[0].Outer, 4)
	measured, err := decad.MeasureProfile(section, profiles[0])
	require.NoError(t, err)
	area, err := measured.Area()
	require.NoError(t, err)
	require.Equal(t, units.SquareMillimeters(2*halfChord*height), area.Value)
	require.Equal(t, decad.Exact, area.Exactness)

	child, err := decad.New().Extrude(section, profiles[0], decad.Distance{
		D: units.Millimeters(2), Dir: decad.Along,
	})
	require.NoError(t, err)
	volume, err := child.Volume()
	require.NoError(t, err)
	require.Equal(t, units.CubicMillimeters(4*halfChord*height), volume.Value)
	require.Equal(t, decad.Exact, volume.Exactness)
	mesh, err := child.Tessellate(t.Context(), units.Millimeters(0.1),
		decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
	return child
}

func TestSectionCardinalCylinderExactOffsetChordsFeedVerifiedExtrude(t *testing.T) {
	t.Parallel()
	for _, tc := range []struct {
		name                 string
		radius, offset, half float64
		height               float64
		min, max             r3.Vec
	}{
		{name: "3-4-5", radius: 5, offset: 3, half: 4, height: 10,
			min: r3.NewVec(-4, 1, 0), max: r3.NewVec(4, 3, 10)},
		{name: "5-12-13", radius: 13, offset: 5, half: 12, height: 6,
			min: r3.NewVec(-12, 3, 0), max: r3.NewVec(12, 5, 6)},
	} {
		t.Run(tc.name, func(t *testing.T) {
			doc, source := circleSectionSourceSized(t, 0, 0, tc.radius, tc.height)
			frame := cardinalAxisSectionPlane(t,
				r3.NewVec(0, tc.offset, tc.height/2), r3.NewVec(1, 0, 0))
			child := verifyOffsetCylinderSection(t, source, frame, tc.half, tc.height)
			box, err := child.Bounds()
			require.NoError(t, err)
			require.Equal(t, tc.min, box.Min)
			require.Equal(t, tc.max, box.Max)
			require.Equal(t, []*decad.Body{source}, doc.Bodies())
		})
	}
}

func TestSectionCardinalCylinderOffsetSignsAndPlacement(t *testing.T) {
	t.Parallel()
	_, source := circleSectionSourceSized(t, 0, 0, 5, 10)
	for _, tc := range []struct {
		name   string
		origin r3.Vec
		u      r3.Vec
	}{
		{name: "negative Y", origin: r3.NewVec(0, -3, 5), u: r3.NewVec(1, 0, 0)},
		{name: "positive X", origin: r3.NewVec(3, 0, 5), u: r3.NewVec(0, 1, 0)},
		{name: "negative X", origin: r3.NewVec(-3, 0, 5), u: r3.NewVec(0, 1, 0)},
	} {
		t.Run(tc.name, func(t *testing.T) {
			frame := cardinalAxisSectionPlane(t, tc.origin, tc.u)
			verifyOffsetCylinderSection(t, source, frame, 4, 10)
		})
	}

	placement, err := r3.FromBasis(r3.Basis{
		EX: r3.Vec{Y: 1}, EY: r3.Vec{X: -1}, EZ: r3.Vec{Z: 1},
	}, r3.NewVec(3, 4, 7))
	require.NoError(t, err)
	placed, err := decad.Transform(t.Context(), source, placement)
	require.NoError(t, err)
	frame := cardinalAxisSectionPlane(t, r3.NewVec(0, 4, 12), r3.NewVec(0, 1, 0))
	child := verifyOffsetCylinderSection(t, placed, frame, 4, 10)
	box, err := child.Bounds()
	require.NoError(t, err)
	require.Equal(t, r3.NewVec(0, 0, 7), box.Min)
	require.Equal(t, r3.NewVec(2, 8, 17), box.Max)
}

func TestSectionCardinalCylinderRefusesUnprovedOffsetChords(t *testing.T) {
	t.Parallel()
	doc, source := circleSectionSourceSized(t, 0, 0, 5, 10)
	for _, tc := range []struct {
		name   string
		offset float64
	}{
		{name: "nonsquare chord", offset: 2},
		{name: "tangent", offset: 5},
		{name: "exterior", offset: 6},
	} {
		t.Run(tc.name, func(t *testing.T) {
			frame := cardinalAxisSectionPlane(t, r3.NewVec(0, tc.offset, 5), r3.NewVec(1, 0, 0))
			section, profiles, err := source.Section(t.Context(), frame)
			require.ErrorIs(t, err, decad.ErrUnsupported)
			require.Nil(t, section)
			require.Empty(t, profiles)
			require.Equal(t, []*decad.Body{source}, doc.Bodies())
		})
	}
}

func TestSectionCardinalCylinderSketchFeedsVerifiedExtrude(t *testing.T) {
	t.Parallel()
	doc, source := circleSectionSource(t, 0, 0)
	frame := cardinalAxisSectionPlane(t, r3.NewVec(0, 0, 2.5), r3.NewVec(1, 0, 0))
	section, profiles, err := source.Section(t.Context(), frame)
	require.NoError(t, err)
	require.Len(t, profiles, 1)
	require.True(t, profiles[0].Valid)
	require.Len(t, profiles[0].Outer, 4)
	sectionFrame, err := section.Plane().Frame()
	require.NoError(t, err)
	require.Equal(t, frame, sectionFrame)
	measured, err := decad.MeasureProfile(section, profiles[0])
	require.NoError(t, err)
	area, err := measured.Area()
	require.NoError(t, err)
	require.Equal(t, units.SquareMillimeters(10), area.Value)
	require.Equal(t, decad.Exact, area.Exactness)

	child, err := decad.New().Extrude(section, profiles[0], decad.Distance{
		D: units.Millimeters(2), Dir: decad.Along,
	})
	require.NoError(t, err)
	volume, err := child.Volume()
	require.NoError(t, err)
	require.Equal(t, units.CubicMillimeters(20), volume.Value)
	require.Equal(t, decad.Exact, volume.Exactness)
	box, err := child.Bounds()
	require.NoError(t, err)
	require.Equal(t, r3.NewVec(-1, -2, 0), box.Min)
	require.Equal(t, r3.NewVec(1, 0, 5), box.Max)
	mesh, err := child.Tessellate(t.Context(), units.Millimeters(0.1), decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
	require.Zero(t, mesh.Bound().Base())
	require.Equal(t, []*decad.Body{source}, doc.Bodies())
}

func TestSectionCardinalCylinderPlacedFrame(t *testing.T) {
	t.Parallel()
	doc, source := circleSectionSource(t, 0, 0)
	placement, err := r3.FromBasis(r3.Basis{
		EX: r3.Vec{Y: 1}, EY: r3.Vec{X: -1}, EZ: r3.Vec{Z: 1},
	}, r3.NewVec(3, 4, 7))
	require.NoError(t, err)
	placed, err := decad.Transform(t.Context(), source, placement)
	require.NoError(t, err)
	frame := cardinalAxisSectionPlane(t, r3.NewVec(3, 4, 9.5), r3.NewVec(0, 1, 0))
	section, profiles, err := placed.Section(t.Context(), frame)
	require.NoError(t, err)
	require.Len(t, profiles, 1)
	measured, err := decad.MeasureProfile(section, profiles[0])
	require.NoError(t, err)
	area, err := measured.Area()
	require.NoError(t, err)
	require.Equal(t, units.SquareMillimeters(10), area.Value)
	require.Equal(t, decad.Exact, area.Exactness)
	require.Equal(t, []*decad.Body{placed}, doc.Bodies())
}

func TestSectionCardinalCylinderOffsetCentreAndReverseAxis(t *testing.T) {
	t.Parallel()
	_, source := circleSectionSource(t, 2, 3)
	frame, err := r3.NewFrame(r3.NewVec(2, 3, 2.5), r3.NewVec(1, 0, 0), r3.NewVec(0, 0, -1))
	require.NoError(t, err)
	section, profiles, err := source.Section(t.Context(), frame)
	require.NoError(t, err)
	require.Len(t, profiles, 1)
	child, err := decad.New().Extrude(section, profiles[0], decad.Distance{
		D: units.Millimeters(1), Dir: decad.Along,
	})
	require.NoError(t, err)
	volume, err := child.Volume()
	require.NoError(t, err)
	require.Equal(t, units.CubicMillimeters(10), volume.Value)
	require.Equal(t, decad.Exact, volume.Exactness)
}

func TestSectionCardinalCylinderRefusesUnprovedPlanes(t *testing.T) {
	t.Parallel()
	doc, source := circleSectionSource(t, 0, 0)
	for _, tc := range []struct {
		name   string
		origin r3.Vec
		u      r3.Vec
	}{
		{name: "offset from axis", origin: r3.NewVec(0, 0.5, 2.5), u: r3.NewVec(1, 0, 0)},
		{name: "noncardinal radial", origin: r3.NewVec(0, 0, 2.5), u: r3.NewVec(3, 4, 0)},
		{name: "unrepresentable rectangle", origin: r3.NewVec(1e16, 0, 2.5), u: r3.NewVec(1, 0, 0)},
	} {
		t.Run(tc.name, func(t *testing.T) {
			frame := cardinalAxisSectionPlane(t, tc.origin, tc.u)
			section, profiles, err := source.Section(t.Context(), frame)
			require.ErrorIs(t, err, decad.ErrUnsupported)
			require.Nil(t, section)
			require.Empty(t, profiles)
			require.Equal(t, []*decad.Body{source}, doc.Bodies())
		})
	}
}

func TestSectionCardinalPlaneRefusesNoncircularSource(t *testing.T) {
	t.Parallel()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	rectangle := s.CreateRectangle(0, 0, 2, 2)
	s.Fix(rectangle.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	doc := decad.New()
	body, err := doc.Extrude(s, s.Profiles()[0], decad.Distance{D: units.Millimeters(5), Dir: decad.Along})
	require.NoError(t, err)
	frame := cardinalAxisSectionPlane(t, r3.NewVec(1, 1, 2.5), r3.NewVec(1, 0, 0))
	section, profiles, err := body.Section(t.Context(), frame)
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.Nil(t, section)
	require.Empty(t, profiles)
	require.Equal(t, []*decad.Body{body}, doc.Bodies())
}
