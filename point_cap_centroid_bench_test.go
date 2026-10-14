package decad

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// BenchmarkPointPlaneCapCentroidRealSplit times the cap reading from a real
// point loft and an authenticated planar sheet, excluding fixture construction.
func BenchmarkPointPlaneCapCentroidRealSplit(b *testing.B) {
	doc := New()
	world := sketch.NewWorld()
	frame, err := r3.NewFrame(r3.NewVec(10, 0, 0), r3.NewVec(0, 1, 0), r3.NewVec(0, 0, 1))
	require.NoError(b, err)
	profilePlane, err := world.CreatePlaneFromFrame(frame)
	require.NoError(b, err)
	profileSketch, err := world.CreateSketch(profilePlane)
	require.NoError(b, err)
	const stations = 64
	points := make([]*sketch.Point, stations)
	for i := range points {
		angle := 2 * math.Pi * float64(i) / stations
		points[i] = profileSketch.CreatePoint(2*math.Cos(angle), 2*math.Sin(angle))
		profileSketch.Fix(points[i])
	}
	for i := range points {
		profileSketch.CreateLine(points[i], points[(i+1)%len(points)])
	}
	_, err = profileSketch.Solve(b.Context())
	require.NoError(b, err)
	require.Len(b, profileSketch.Profiles(), 1)
	apex := r3.NewVec(1, 0, 0)
	tooth, err := doc.LoftFromPoint(b.Context(), apex, profileSketch, profileSketch.Profiles()[0])
	require.NoError(b, err)

	sheetSketch, err := world.CreateSketch(world.XY())
	require.NoError(b, err)
	a, c := sheetSketch.CreatePoint(5, -30), sheetSketch.CreatePoint(5, 30)
	sheetSketch.Fix(a)
	sheetSketch.Fix(c)
	sheetSketch.CreateLine(a, c)
	_, err = sheetSketch.Solve(b.Context())
	require.NoError(b, err)
	require.Len(b, sheetSketch.Chains(), 1)
	sheet, err := Extrude(b.Context(), doc, sheetSketch, sheetSketch.Chains()[0], TwoSided{
		One: DistanceSide{D: units.Millimeters(30)},
		Two: DistanceSide{D: units.Millimeters(30)},
	})
	require.NoError(b, err)
	plane, ok := exactPointPlaneSheet(sheet, apex)
	require.True(b, ok)
	base := tooth.n.Payload.(facetedPayload)
	require.NoError(b, validatePointPlane(base, plane))

	b.ReportAllocs()
	b.ResetTimer()
	for b.Loop() {
		_, proved, err := pointPlaneCapCentroid(b.Context(), base, plane)
		if err != nil || !proved {
			b.Fatalf("cap centroid was not certified: proved=%t err=%v", proved, err)
		}
	}
}
