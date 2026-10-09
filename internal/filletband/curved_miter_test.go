package filletband_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/filletband"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/stretchr/testify/require"
)

// The line y=0 ending at (5,0) and clockwise radius-5 arc meet at a sharp
// corner. Their inward offsets meet at (sqrt(25+10t), -t), so the exact seam
// speed can be integrated independently of the production locus bound.
func TestCurvedMiterLengthEnclosesLineArcSeam(t *testing.T) {
	t.Parallel()
	line := survey2d.SideWalk{SegmentWalk: survey2d.SegmentWalk{
		StartU: 10, EndU: 5, StartV: 0, EndV: 0,
	}}
	arc := survey2d.SideWalk{SegmentWalk: survey2d.SegmentWalk{
		Kind:   survey2d.WalkCircular,
		StartU: 5, StartV: 0, EndU: -5, EndV: 0,
		CU: 0, CV: 0, Radius: 5, Th0: 0, Th1: -math.Pi,
	}}
	got, ok := filletband.CurvedMiterLength(line, arc, 1)
	require.True(t, ok)
	for _, offset := range []float64{0, 0.25, 0.5, 0.75, 1} {
		p, pointOK := filletband.CurvedMiterPoint(line, arc, 1, offset, 5, 0)
		require.True(t, pointOK)
		for axis, want := range [3]float64{math.Sqrt(25 + 10*offset), -offset,
			math.Sqrt(2*offset - offset*offset)} {
			lo, _ := p[axis].Lo.Float64()
			hi, _ := p[axis].Hi.Float64()
			require.LessOrEqual(t, lo-1e-14, want)
			require.GreaterOrEqual(t, hi+1e-14, want)
			require.Less(t, hi-lo, 1e-8)
		}
	}
	lo, _ := got.Lo.Float64()
	hi, _ := got.Hi.Float64()
	const steps = 4096
	var length float64
	for i := range steps {
		phi := (float64(i) + 0.5) * math.Pi / (2 * steps)
		tau := 1 - math.Cos(phi)
		footSpeedSquared := 1 + 25/(25+10*tau)
		length += math.Sqrt(footSpeedSquared*math.Sin(phi)*math.Sin(phi) + math.Cos(phi)*math.Cos(phi))
	}
	length *= math.Pi / (2 * steps)
	require.Less(t, lo, length)
	require.Greater(t, hi, length)
	require.Greater(t, hi, 1.5)
	const chords = 8
	gap, ok := filletband.CurvedMiterChordGap(line, arc, 1, chords)
	require.True(t, ok)
	gapUpper, _ := gap.Float64()
	seam := func(phi float64) [3]float64 {
		offset := 1 - math.Cos(phi)
		return [3]float64{math.Sqrt(25 + 10*offset), -offset, math.Sin(phi)}
	}
	for i := range chords {
		a := seam(float64(i) * math.Pi / (2 * chords))
		b := seam(float64(i+1) * math.Pi / (2 * chords))
		for j := 1; j < 16; j++ {
			p := seam((float64(i) + float64(j)/16) * math.Pi / (2 * chords))
			var dot, norm float64
			for axis := range 3 {
				d := b[axis] - a[axis]
				dot += (p[axis] - a[axis]) * d
				norm += d * d
			}
			fraction := math.Max(0, math.Min(1, dot/norm))
			var squared float64
			for axis := range 3 {
				d := p[axis] - (a[axis] + fraction*(b[axis]-a[axis]))
				squared += d * d
			}
			require.LessOrEqual(t, math.Sqrt(squared), gapUpper)
		}
	}
}

func TestCurvedMiterRefusesFoldedOffset(t *testing.T) {
	t.Parallel()
	line := survey2d.SideWalk{SegmentWalk: survey2d.SegmentWalk{
		StartU: 0, EndU: 10,
	}}
	arc := survey2d.SideWalk{SegmentWalk: survey2d.SegmentWalk{
		Kind: survey2d.WalkCircular,
		CU:   0, CV: -5, Radius: 5, Th0: math.Pi / 2, Th1: math.Pi,
	}}
	_, ok := filletband.CurvedMiterLength(line, arc, 1)
	require.False(t, ok)
	_, ok = filletband.CurvedMiterPoint(line, arc, 1, 0.5, 0, 0)
	require.False(t, ok)
}

func TestCurvedMiterPointEnclosesTwoCircularWalls(t *testing.T) {
	t.Parallel()
	prev := survey2d.SideWalk{SegmentWalk: survey2d.SegmentWalk{
		Kind:   survey2d.WalkCircular,
		StartU: 2, StartV: 0, EndU: 0, EndV: 4,
		CU: -3, CV: 0, Radius: 5, Th0: 0, Th1: math.Atan2(4, 3),
	}}
	cur := survey2d.SideWalk{SegmentWalk: survey2d.SegmentWalk{
		Kind:   survey2d.WalkCircular,
		StartU: 0, StartV: 4, EndU: -2, EndV: 0,
		CU: 3, CV: 0, Radius: 5, Th0: math.Atan2(4, -3), Th1: math.Pi,
	}}
	for _, offset := range []float64{0, 0.25, 0.5, 0.75, 1} {
		point, ok := filletband.CurvedMiterPoint(prev, cur, 1, offset, 0, 4)
		require.True(t, ok)
		want := [3]float64{0, math.Sqrt((5-offset)*(5-offset) - 9),
			math.Sqrt(2*offset - offset*offset)}
		for axis := range 3 {
			lo, _ := point[axis].Lo.Float64()
			hi, _ := point[axis].Hi.Float64()
			require.LessOrEqual(t, lo-1e-14, want[axis])
			require.GreaterOrEqual(t, hi+1e-14, want[axis])
			require.Less(t, hi-lo, 1e-8)
		}
	}
	length, ok := filletband.CurvedMiterLength(prev, cur, 1)
	require.True(t, ok)
	lo, _ := length.Lo.Float64()
	hi, _ := length.Hi.Float64()
	const steps = 4096
	var reference float64
	for i := range steps {
		phi := (float64(i) + 0.5) * math.Pi / (2 * steps)
		offset := 1 - math.Cos(phi)
		radius := 5 - offset
		y := math.Sqrt(radius*radius - 9)
		verticalSpeed := radius / y
		reference += math.Hypot(verticalSpeed*math.Sin(phi), math.Cos(phi))
	}
	reference *= math.Pi / (2 * steps)
	require.Less(t, lo, reference)
	require.Greater(t, hi, reference)
}
