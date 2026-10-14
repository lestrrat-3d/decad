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

func TestSectionSmoothRectangularLoftRealFlow(t *testing.T) {
	sections := manyRectangleSections(t, []float64{2, 3, 4, 5, 6},
		[]float64{3, 4, 5, 7, 8}, []float64{0, 4, 8, 12, 16})
	body, err := decad.New().LoftSections(t.Context(), sections...)
	require.NoError(t, err)
	frame, err := r3.NewFrame(r3.NewVec(0, 0, 6), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	section, profiles, err := body.Section(t.Context(), frame)
	require.NoError(t, err)
	require.NotNil(t, section)
	require.Len(t, profiles, 1)
	require.True(t, profiles[0].Valid)
	measured, err := decad.MeasureProfile(section, profiles[0])
	require.NoError(t, err)
	area, err := measured.Area()
	require.NoError(t, err)
	require.LessOrEqual(t, math.Abs(area.Value.Base()-62.125), area.Bound.Base())
	child, err := decad.Extrude(t.Context(), decad.New(), section, profiles[0],
		decad.Distance{D: units.Millimeters(2), Dir: decad.Along})
	require.NoError(t, err)
	volume, err := child.Volume()
	require.NoError(t, err)
	require.LessOrEqual(t, math.Abs(volume.Value.Base()-124.25), volume.Bound.Base())
	box, err := child.Bounds()
	require.NoError(t, err)
	require.Less(t, box.Bound.Base(), 0.01)
	require.LessOrEqual(t, math.Abs(box.Min.X+3.5), box.Bound.Base())
	require.LessOrEqual(t, math.Abs(box.Max.X-3.5), box.Bound.Base())
	require.LessOrEqual(t, math.Abs(box.Min.Y+4.4375), box.Bound.Base())
	require.LessOrEqual(t, math.Abs(box.Max.Y-4.4375), box.Bound.Base())
	mesh, err := child.Tessellate(t.Context(), units.Millimeters(0.1),
		decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
}

func TestSectionSmoothRectangularLoftSourceCurves(t *testing.T) {
	for _, tc := range []struct {
		name              string
		widths, depths    []float64
		levels            []float64
		cut, wantW, wantH float64
	}{
		{"quadratic three", []float64{5, 4, 2.5}, []float64{4, 5, 6.5},
			[]float64{0, 5, 10}, 2.5, 4.5625, 4.4375},
		{"cubic four", []float64{2, 3, 4, 5}, []float64{3, 4, 5, 7},
			[]float64{0, 4, 8, 12}, 6, 3.5, 4.4375},
		{"piecewise source level", []float64{2, 3, 4, 5, 6}, []float64{3, 4, 5, 7, 8},
			[]float64{0, 4, 8, 12, 16}, 8, 4, 5},
	} {
		t.Run(tc.name, func(t *testing.T) {
			sections := manyRectangleSections(t, tc.widths, tc.depths, tc.levels)
			body, err := decad.New().LoftSections(t.Context(), sections...)
			require.NoError(t, err)
			frame, err := r3.NewFrame(r3.NewVec(0, 0, tc.cut), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
			require.NoError(t, err)
			section, profiles, err := body.Section(t.Context(), frame)
			require.NoError(t, err)
			require.Len(t, profiles, 1)
			measured, err := decad.MeasureProfile(section, profiles[0])
			require.NoError(t, err)
			area, err := measured.Area()
			require.NoError(t, err)
			wantArea := 4 * tc.wantW * tc.wantH
			require.LessOrEqual(t, math.Abs(area.Value.Base()-wantArea), area.Bound.Base())
		})
	}
}

func TestSectionSmoothRectangularLoftTranslatedSourceFrame(t *testing.T) {
	world := sketch.NewWorld()
	origin := r3.NewVec(10, 20, 3)
	base, err := r3.NewFrame(origin, r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	basePlane, err := world.CreatePlaneFromFrame(base)
	require.NoError(t, err)
	sections := make([]decad.LoftSection, 5)
	for i := range sections {
		plane := basePlane
		if i != 0 {
			plane, err = world.CreateOffsetPlane(basePlane, float64(4*i))
			require.NoError(t, err)
		}
		s, sketchErr := world.CreateSketch(plane)
		require.NoError(t, sketchErr)
		w, h := float64(i+2), []float64{3, 4, 5, 7, 8}[i]
		rect := s.CreateRectangle(-w, -h, w, h)
		s.Fix(rect.A)
		_, sketchErr = s.Solve(t.Context())
		require.NoError(t, sketchErr)
		profiles := s.Profiles()
		require.Len(t, profiles, 1)
		sections[i] = decad.LoftSection{Sketch: s, Profile: profiles[0]}
	}
	body, err := decad.New().LoftSections(t.Context(), sections...)
	require.NoError(t, err)
	frame, err := r3.NewFrame(r3.NewVec(10, 20, 9), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	section, profiles, err := body.Section(t.Context(), frame)
	require.NoError(t, err)
	require.Len(t, profiles, 1)
	measured, err := decad.MeasureProfile(section, profiles[0])
	require.NoError(t, err)
	area, err := measured.Area()
	require.NoError(t, err)
	require.LessOrEqual(t, math.Abs(area.Value.Base()-62.125), area.Bound.Base())
}

func TestSectionSmoothRectangularLoftRefusesUnprovedPlanesAndCopies(t *testing.T) {
	sections := manyRectangleSections(t, []float64{2, 3, 4, 5, 6},
		[]float64{3, 4, 5, 7, 8}, []float64{0, 4, 8, 12, 16})
	body, err := decad.New().LoftSections(t.Context(), sections...)
	require.NoError(t, err)
	for _, tc := range []struct {
		name   string
		copy   bool
		origin r3.Vec
		u, v   r3.Vec
	}{
		{"lower cap", false, r3.NewVec(0, 0, 0), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0)},
		{"upper cap", false, r3.NewVec(0, 0, 16), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0)},
		{"shifted origin", false, r3.NewVec(1, 0, 6), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0)},
		{"shifted Y origin", false, r3.NewVec(0, 1, 6), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0)},
		{"rotated plane", false, r3.NewVec(0, 0, 6), r3.NewVec(0, 1, 0), r3.NewVec(-1, 0, 0)},
		{"inexact section", false, r3.NewVec(0, 0, 0.1), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0)},
		{"placed body", true, r3.NewVec(1, 0, 6), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0)},
	} {
		t.Run(tc.name, func(t *testing.T) {
			source := body
			if tc.copy {
				move, err := r3.Translation(r3.NewVec(1, 0, 0))
				require.NoError(t, err)
				source, err = decad.Copy(t.Context(), body, decad.WithCopyPlacement(move))
				require.NoError(t, err)
			}
			frame, err := r3.NewFrame(tc.origin, tc.u, tc.v)
			require.NoError(t, err)
			_, _, err = source.Section(t.Context(), frame)
			require.ErrorIs(t, err, decad.ErrUnsupported)
		})
	}
	t.Run("identity copy", func(t *testing.T) {
		copied, err := decad.Copy(t.Context(), body)
		require.NoError(t, err)
		frame, err := r3.NewFrame(r3.NewVec(0, 0, 6), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
		require.NoError(t, err)
		_, _, err = copied.Section(t.Context(), frame)
		require.ErrorIs(t, err, decad.ErrUnsupported)
	})
}

func TestSectionSmoothRectangularLoftRefusesInexactWidth(t *testing.T) {
	sections := manyRectangleSections(t, []float64{2, 3, 4, 5, 6},
		[]float64{3, 3, 3, 3, 3}, []float64{0, 4, 8, 12, 16})
	body, err := decad.New().LoftSections(t.Context(), sections...)
	require.NoError(t, err)
	frame, err := r3.NewFrame(r3.NewVec(0, 0, 0.1), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	_, _, err = body.Section(t.Context(), frame)
	require.ErrorIs(t, err, decad.ErrUnsupported)
}

func TestSectionSmoothRectangularLoftRefusesInexactHeight(t *testing.T) {
	sections := manyRectangleSections(t, []float64{2, 2, 2, 2, 2},
		[]float64{3, 4, 5, 6, 7}, []float64{0, 4, 8, 12, 16})
	body, err := decad.New().LoftSections(t.Context(), sections...)
	require.NoError(t, err)
	frame, err := r3.NewFrame(r3.NewVec(0, 0, 0.1), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	_, _, err = body.Section(t.Context(), frame)
	require.ErrorIs(t, err, decad.ErrUnsupported)
}
