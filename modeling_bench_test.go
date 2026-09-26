package decad_test

import (
	"fmt"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func modelingBenchRect(tb testing.TB, doc *decad.Document, x0, y0, x1, y1, h float64) *decad.Body {
	tb.Helper()
	w := sketch.NewWorld()
	return modelingBenchRectOnWorld(tb, doc, w, x0, y0, x1, y1, h)
}

func modelingBenchRectOnWorld(tb testing.TB, doc *decad.Document, w *sketch.World, x0, y0, x1, y1, h float64) *decad.Body {
	tb.Helper()
	s, err := w.CreateSketch(w.XY())
	require.NoError(tb, err)
	rect := s.CreateRectangle(x0, y0, x1, y1)
	s.Fix(rect.A)
	_, err = s.Solve(tb.Context())
	require.NoError(tb, err)
	body, err := doc.Extrude(s, s.Profiles()[0], decad.Distance{D: units.Millimeters(h), Dir: decad.Along})
	require.NoError(tb, err)
	return body
}

func modelingBenchArch(tb testing.TB) (*sketch.Sketch, *sketch.Profile) {
	tb.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(tb, err)
	start := s.CreatePoint(0, 0)
	mid := s.CreatePoint(4, 3)
	end := s.CreatePoint(8, 0)
	_, err = s.CreateFitSpline(start, mid, end)
	require.NoError(tb, err)
	s.CreateLine(end, start)
	_, err = s.Solve(tb.Context())
	require.NoError(tb, err)
	profiles := s.Profiles()
	require.Len(tb, profiles, 1)
	return s, profiles[0]
}

func modelingRequireAnalytic(tb testing.TB, body *decad.Body) {
	tb.Helper()
	for _, face := range body.Faces() {
		require.NotEqual(tb, decad.KindFaceted, face.Surface().Kind())
	}
}

// BenchmarkModelingExtrudeFreeformCold measures a public free-form build from
// a solved sketch. Its result is consumed through Verify and Volume.
func BenchmarkModelingExtrudeFreeformCold(b *testing.B) {
	s, profile := modelingBenchArch(b)
	depth := decad.Distance{D: units.Millimeters(10), Dir: decad.Along}

	var doc *decad.Document
	var body *decad.Body
	var err error
	b.ResetTimer()
	for b.Loop() {
		b.StopTimer()
		doc = decad.New()
		b.StartTimer()
		body, err = doc.Extrude(s, profile, depth)
		if err != nil {
			b.Fatal(err)
		}
		b.StopTimer()
		report, verifyErr := doc.Verify(b.Context())
		require.NoError(b, verifyErr)
		require.True(b, report.Passed())
		volume, volumeErr := body.Volume()
		require.NoError(b, volumeErr)
		require.InDelta(b, 150.0, volume.Value.Base(), 1e-8)
		b.StartTimer()
	}
	b.StopTimer()
}

// BenchmarkModelingRevolveCold measures a public full-turn revolve build.
func BenchmarkModelingRevolveCold(b *testing.B) {
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(b, err)
	rect := s.CreateRectangle(2, 0, 6, 12)
	s.Fix(rect.A)
	_, err = s.Solve(b.Context())
	require.NoError(b, err)
	profile := s.Profiles()[0]
	axis := decad.SketchLine{Start: decad.Point2{U: 0, V: 0}, End: decad.Point2{U: 1, V: 0}}
	var body *decad.Body
	b.ResetTimer()
	for b.Loop() {
		b.StopTimer()
		doc := decad.New()
		b.StartTimer()
		body, err = doc.Revolve(s, profile, axis, decad.FullRevolution{})
		if err != nil {
			b.Fatal(err)
		}
		b.StopTimer()
		volume, volumeErr := body.Volume()
		require.NoError(b, volumeErr)
		require.InDelta(b, 576*3.141592653589793, volume.Value.Base(), 1e-8)
		b.StartTimer()
	}
	b.StopTimer()
}

// BenchmarkModelingAnalyticUnion measures the public prism-reduction path.
func BenchmarkModelingAnalyticUnion(b *testing.B) {
	for b.Loop() {
		b.StopTimer()
		doc := decad.New()
		a := modelingBenchRect(b, doc, 0, 0, 10, 10, 10)
		c := modelingBenchRect(b, doc, 5, 5, 15, 15, 10)
		b.StartTimer()
		got, err := decad.Union(b.Context(), a, c)
		if err != nil {
			b.Fatal(err)
		}
		b.StopTimer()
		require.Equal(b, decad.BodySolid, got.Kind())
		modelingRequireAnalytic(b, got)
		volume, err := got.Volume()
		require.NoError(b, err)
		require.InDelta(b, 1750.0, volume.Value.Base(), 1e-8)
		b.StartTimer()
	}
	b.StopTimer()
}

// BenchmarkModelingAnalyticCut measures a public crossing-prism cut.
func BenchmarkModelingAnalyticCut(b *testing.B) {
	for b.Loop() {
		b.StopTimer()
		doc := decad.New()
		target := modelingBenchRect(b, doc, 0, 0, 10, 10, 5)
		tool := modelingBenchRect(b, doc, 5, 5, 15, 15, 5)
		b.StartTimer()
		got, err := decad.Cut(b.Context(), target, tool)
		if err != nil {
			b.Fatal(err)
		}
		b.StopTimer()
		require.Equal(b, decad.BodySolid, got.Kind())
		modelingRequireAnalytic(b, got)
		volume, err := got.Volume()
		require.NoError(b, err)
		require.InDelta(b, 375.0, volume.Value.Base(), 1e-8)
		b.StartTimer()
	}
	b.StopTimer()
}

// BenchmarkModelingPlacedFreeform measures rebuilding a free-form body under a
// public placement transform.
func BenchmarkModelingPlacedFreeform(b *testing.B) {
	s, profile := modelingBenchArch(b)
	transform, err := r3.Translation(r3.Vec{X: 12, Y: 8, Z: 4})
	require.NoError(b, err)
	var placed *decad.Body
	b.ResetTimer()
	for b.Loop() {
		b.StopTimer()
		doc := decad.New()
		body, buildErr := doc.Extrude(s, profile, decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
		require.NoError(b, buildErr)
		b.StartTimer()
		placed, err = body.Placed(b.Context(), transform)
		if err != nil {
			b.Fatal(err)
		}
		b.StopTimer()
		require.NotNil(b, placed)
		volume, volumeErr := placed.Volume()
		require.NoError(b, volumeErr)
		require.InDelta(b, 150.0, volume.Value.Base(), 1e-8)
		b.StartTimer()
	}
	b.StopTimer()
}

// BenchmarkModelingVerifyDefaultWarm measures default verification on an
// increasing number of disjoint, already-built bodies.
func BenchmarkModelingVerifyDefaultWarmDisjoint(b *testing.B) {
	for _, count := range []int{2, 4, 8} {
		b.Run(fmt.Sprintf("bodies_%d", count), func(b *testing.B) {
			doc := decad.New()
			for i := range count {
				x := float64(i * 30)
				modelingBenchRect(b, doc, x, 0, x+20, 20, 10)
			}
			warmReport, err := doc.Verify(b.Context())
			require.NoError(b, err)
			require.Len(b, warmReport.Bodies, count)
			require.Empty(b, warmReport.Interferences)
			require.True(b, warmReport.Passed())
			b.ResetTimer()
			for b.Loop() {
				report, err := doc.Verify(b.Context())
				if err != nil {
					b.Fatal(err)
				}
				b.StopTimer()
				require.Len(b, report.Bodies, count)
				require.Empty(b, report.Interferences)
				require.True(b, report.Passed())
				b.StartTimer()
			}
		})
	}
}

// BenchmarkModelingPatchCircle measures building a public planar patch.
func BenchmarkModelingPatchCircle(b *testing.B) {
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(b, err)
	s.CreateCircle(s.CreatePoint(0, 0), 10)
	_, err = s.Solve(b.Context())
	require.NoError(b, err)
	profile := s.Profiles()[0]
	b.ResetTimer()
	for b.Loop() {
		b.StopTimer()
		doc := decad.New()
		b.StartTimer()
		patch, err := doc.Patch(b.Context(), s, profile)
		if err != nil {
			b.Fatal(err)
		}
		b.StopTimer()
		require.Equal(b, decad.BodySheet, patch.Kind())
		require.Len(b, patch.Faces(), 1)
		area, err := patch.Area()
		require.NoError(b, err)
		require.InDelta(b, 100*3.141592653589793, area.Value.Base(), 1e-8)
		b.StartTimer()
	}
	b.StopTimer()
}

// BenchmarkModelingThickenPatch measures a public planar-patch thickening.
func BenchmarkModelingThickenPatch(b *testing.B) {
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(b, err)
	s.CreateCircle(s.CreatePoint(0, 0), 10)
	_, err = s.Solve(b.Context())
	require.NoError(b, err)
	profile := s.Profiles()[0]
	b.ResetTimer()
	for b.Loop() {
		b.StopTimer()
		doc := decad.New()
		patch, err := doc.Patch(b.Context(), s, profile)
		require.NoError(b, err)
		b.StartTimer()
		body, err := patch.Thicken(b.Context(), units.Millimeters(2))
		if err != nil {
			b.Fatal(err)
		}
		b.StopTimer()
		require.Equal(b, decad.BodySolid, body.Kind())
		volume, err := body.Volume()
		require.NoError(b, err)
		require.InDelta(b, 200*3.141592653589793, volume.Value.Base(), 1e-8)
		b.StartTimer()
	}
	b.StopTimer()
}

// BenchmarkModelingPatchFreeEdges measures filling both free rims on an
// extruded cylindrical sheet through Body.Patch.
func BenchmarkModelingPatchFreeEdges(b *testing.B) {
	for b.Loop() {
		b.StopTimer()
		w := sketch.NewWorld()
		s, err := w.CreateSketch(w.XY())
		require.NoError(b, err)
		s.CreateCircle(s.CreatePoint(0, 0), 10)
		_, err = s.Solve(b.Context())
		require.NoError(b, err)
		doc := decad.New()
		sheet, err := doc.Extrude(s, s.Profiles()[0], decad.Distance{
			D: units.Millimeters(10), Dir: decad.Along,
		}, decad.WithSurfaceResult())
		require.NoError(b, err)
		b.StartTimer()
		patched, err := sheet.Patch(b.Context(), decad.Edges(decad.Free()).Exactly(2))
		if err != nil {
			b.Fatal(err)
		}
		b.StopTimer()
		require.Equal(b, decad.BodySheet, patched.Kind())
		require.Len(b, patched.Faces(), 3)
		area, err := patched.Area()
		require.NoError(b, err)
		require.InDelta(b, 400*3.141592653589793, area.Value.Base(), 1e-8)
		b.StartTimer()
	}
	b.StopTimer()
}

// BenchmarkModelingTrimRectangleSheet measures the public co-generated prism
// Trim operation on a sheet cut into two surviving ribbons.
func BenchmarkModelingTrimRectangleSheet(b *testing.B) {
	for b.Loop() {
		b.StopTimer()
		doc := decad.New()
		w := sketch.NewWorld()
		sheetSketch, err := w.CreateSketch(w.XY())
		require.NoError(b, err)
		rect := sheetSketch.CreateRectangle(0, 0, 100, 60)
		sheetSketch.Fix(rect.A)
		_, err = sheetSketch.Solve(b.Context())
		require.NoError(b, err)
		sheet, err := doc.Extrude(sheetSketch, sheetSketch.Profiles()[0],
			decad.Distance{D: units.Millimeters(10), Dir: decad.Along}, decad.WithSurfaceResult())
		require.NoError(b, err)
		toolSketch, err := w.CreateSketch(w.XY())
		require.NoError(b, err)
		toolRect := toolSketch.CreateRectangle(40, -10, 60, 70)
		toolSketch.Fix(toolRect.A)
		_, err = toolSketch.Solve(b.Context())
		require.NoError(b, err)
		tool, err := doc.Extrude(toolSketch, toolSketch.Profiles()[0], decad.TwoSided{
			One: decad.DistanceSide{D: units.Millimeters(15)},
			Two: decad.DistanceSide{D: units.Millimeters(5)},
		})
		require.NoError(b, err)
		b.StartTimer()
		trimmed, err := sheet.Trim(b.Context(), tool, decad.KeepOutside)
		if err != nil {
			b.Fatal(err)
		}
		b.StopTimer()
		require.Equal(b, decad.BodySheet, trimmed.Kind())
		require.Len(b, trimmed.Lumps(), 2)
		area, err := trimmed.Area()
		require.NoError(b, err)
		require.InDelta(b, 2800.0, area.Value.Base(), 1e-8)
		b.StartTimer()
	}
	b.StopTimer()
}

// BenchmarkModelingExtendChain measures extending a public straight ribbon to
// a crossing solid while reusing sketch's certified cut location.
func BenchmarkModelingExtendChain(b *testing.B) {
	query := decad.Edges(decad.Free(), decad.ParallelTo(r3.NewVec(0, 0, 1)),
		decad.EndpointAt(r3.NewVec(40, 0, 0))).Exactly(1)
	for b.Loop() {
		b.StopTimer()
		doc := decad.New()
		w := sketch.NewWorld()
		s, err := w.CreateSketch(w.XY())
		require.NoError(b, err)
		a := s.CreatePoint(0, 0)
		s.Fix(a)
		line := s.CreateLine(a, s.CreatePoint(100, 0))
		crossing := s.CreatePoint(40, -10)
		s.Fix(crossing)
		s.CreateLine(crossing, s.CreatePoint(40, 10))
		_, err = s.Solve(b.Context())
		require.NoError(b, err)
		var ribbon *decad.Body
		for _, chain := range s.Chains() {
			if len(chain.Edges) != 1 || chain.Edges[0].Entity != line || chain.Edges[0].TStart != 0 {
				continue
			}
			ribbon, err = doc.ExtrudeChain(s, chain, decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
			require.NoError(b, err)
			break
		}
		require.NotNil(b, ribbon)
		toolSketch, err := w.CreateSketch(w.XY())
		require.NoError(b, err)
		toolRect := toolSketch.CreateRectangle(70, -10, 90, 10)
		toolSketch.Fix(toolRect.A)
		_, err = toolSketch.Solve(b.Context())
		require.NoError(b, err)
		tool, err := doc.Extrude(toolSketch, toolSketch.Profiles()[0], decad.Symmetric{D: units.Millimeters(10)})
		require.NoError(b, err)
		b.StartTimer()
		extended, err := ribbon.Extend(b.Context(), query, tool)
		if err != nil {
			b.Fatal(err)
		}
		b.StopTimer()
		area, err := extended.Area()
		require.NoError(b, err)
		require.InDelta(b, 700.0, area.Value.Base(), 1e-8)
		b.StartTimer()
	}
	b.StopTimer()
}

// BenchmarkModelingUnstitchPrism measures splitting a solid's faces into
// independent sheets through the public Body.Unstitch operation.
func BenchmarkModelingUnstitchPrism(b *testing.B) {
	for b.Loop() {
		b.StopTimer()
		doc := decad.New()
		body := modelingBenchRect(b, doc, 0, 0, 20, 20, 10)
		b.StartTimer()
		sheets, err := body.Unstitch(b.Context())
		if err != nil {
			b.Fatal(err)
		}
		b.StopTimer()
		require.Len(b, sheets, 6)
		areaTotal := 0.0
		for _, sheet := range sheets {
			require.Equal(b, decad.BodySheet, sheet.Kind())
			area, err := sheet.Area()
			require.NoError(b, err)
			areaTotal += area.Value.Base()
		}
		require.Equal(b, 1600.0, areaTotal)
		b.StartTimer()
	}
	b.StopTimer()
}
