package decad_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// BenchmarkShellCupWithPost measures a one-cap shell around a circular hole.
// Each iteration builds a fresh receiver because Shell retires its input.
func BenchmarkShellCupWithPost(b *testing.B) {
	for b.Loop() {
		b.StopTimer()
		doc, box := circleHoledBox(b, [3]float64{50, 30, 8})
		b.StartTimer()
		body, err := box.Shell(b.Context(), topCap(box), units.Millimeters(5))
		require.NoError(b, err)
		require.True(b, body.IsSolid())
		require.Len(b, body.Faces(), 14)
		require.Equal(b, []*decad.Body{body}, doc.Bodies())
	}
}

// benchmarkShellCupPosts keeps receiver construction outside the timed region.
// The one- and two-post cases use the same plate, thickness, and cap selector.
func benchmarkShellCupPosts(b *testing.B, outward bool, holes ...[3]float64) {
	b.Helper()
	const thickness = 5.0
	area := 100.0 * 60.0
	for _, hole := range holes {
		area -= math.Pi * hole[2] * hole[2]
	}
	var wantVolume float64
	if outward {
		outerArea := 110.0*70.0 - (4-math.Pi)*thickness*thickness
		for _, hole := range holes {
			outerArea -= math.Pi * math.Pow(hole[2]-thickness, 2)
		}
		wantVolume = outerArea*(shellBoxHeight+thickness) - area*shellBoxHeight
	} else {
		innerArea := 90.0 * 50.0
		for _, hole := range holes {
			innerArea -= math.Pi * math.Pow(hole[2]+thickness, 2)
		}
		wantVolume = area*shellBoxHeight - innerArea*(shellBoxHeight-thickness)
	}
	for b.Loop() {
		b.StopTimer()
		doc, box := circleHoledBox(b, holes...)
		b.StartTimer()
		var body *decad.Body
		var err error
		if outward {
			body, err = box.Shell(b.Context(), topCap(box), units.Millimeters(thickness),
				decad.WithShellSense(decad.Outward))
		} else {
			body, err = box.Shell(b.Context(), topCap(box), units.Millimeters(thickness))
		}
		require.NoError(b, err)
		require.True(b, body.IsSolid())
		require.Len(b, body.Lumps(), 1)
		require.Equal(b, []*decad.Body{body}, doc.Bodies())
		volume, err := body.Volume()
		require.NoError(b, err)
		require.InDelta(b, wantVolume, volume.Value.Base(), 1e-8)
	}
}

// BenchmarkShellCupOnePostOutward isolates outward shelling of the original
// one-post receiver, which does not run the section-inradius survey.
func BenchmarkShellCupOnePostOutward(b *testing.B) {
	benchmarkShellCupPosts(b, true, [3]float64{50, 30, 8})
}

// BenchmarkShellCupTwoPostsInward measures how the inward path scales when
// the section has two circular holes.
func BenchmarkShellCupTwoPostsInward(b *testing.B) {
	benchmarkShellCupPosts(b, false, [3]float64{30, 30, 6}, [3]float64{70, 30, 6})
}

// BenchmarkShellCupTwoPostsOutward pairs the two-post receiver with an
// outward shell, which skips the section-inradius survey.
func BenchmarkShellCupTwoPostsOutward(b *testing.B) {
	benchmarkShellCupPosts(b, true, [3]float64{30, 30, 6}, [3]float64{70, 30, 6})
}
