package jointcheck_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/jointcheck"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func box(x0, y0, z0, x1, y1, z1 float64) jointcheck.Box {
	return jointcheck.Box{Min: r3.Vec{X: x0, Y: y0, Z: z0}, Max: r3.Vec{X: x1, Y: y1, Z: z1}}
}

func TestGapLowAndOverlap(t *testing.T) {
	a := box(0, 0, 0, 10, 10, 10)
	require.InDelta(t, 5.0, jointcheck.GapLow(a, box(15, 0, 0, 20, 10, 10), 0), 1e-12)
	require.InDelta(t, 4.9, jointcheck.GapLow(a, box(15, 0, 0, 20, 10, 10), 0.1), 1e-12)
	require.InDelta(t, 0.0, jointcheck.GapLow(a, box(5, 5, 5, 20, 20, 20), 0), 1e-12, "overlapping boxes")
	// The largest separation along one axis wins, not the diagonal distance.
	require.InDelta(t, 3.0, jointcheck.GapLow(a, box(13, 12, 0, 20, 20, 10), 0), 1e-12)
	require.InDelta(t, 0.0, jointcheck.GapLow(a, box(10.05, 0, 0, 20, 10, 10), 0.1), 1e-12, "slack closes a small gap")

	o, ok := a.Overlap(box(5, 8, -3, 20, 20, 4))
	require.True(t, ok)
	require.Equal(t, box(5, 8, 0, 10, 10, 4), o)
	_, ok = a.Overlap(box(11, 0, 0, 20, 10, 10))
	require.False(t, ok)
	require.True(t, a.Grow(1.5).Meets(box(11, 0, 0, 20, 10, 10)))
	require.False(t, a.Grow(0.5).Meets(box(11, 0, 0, 20, 10, 10)))
}

func TestGapHighAndPointDistance(t *testing.T) {
	a, b := box(0, 0, 0, 10, 10, 10), box(13, 0, 0, 20, 10, 10)
	// The farthest corners are (0,*,*) and (20,*,*): 20 along X and 10 along Y and Z.
	require.InDelta(t, math.Sqrt(400+100+100), jointcheck.GapHigh(a, b), 1e-12)
	require.InDelta(t, 3.0, a.PointDistance(r3.Vec{X: 13, Y: 5, Z: 5}), 1e-12)
	require.InDelta(t, 5.0, a.PointDistance(r3.Vec{X: 13, Y: 14, Z: 5}), 1e-12)
	require.InDelta(t, 0.0, a.PointDistance(r3.Vec{X: 1, Y: 5, Z: 5}), 1e-12)
}

func TestWithin(t *testing.T) {
	require.Equal(t, jointcheck.Inside, jointcheck.Within(0.2, 0.01, 0.15, 0.25))
	require.Equal(t, jointcheck.Outside, jointcheck.Within(0.3, 0.01, 0.15, 0.25))
	require.Equal(t, jointcheck.Straddle, jointcheck.Within(0.25, 0.02, 0.15, 0.25))
	require.Equal(t, jointcheck.Outside, jointcheck.Within(-0.1, 0.01, 0, math.Inf(1)))

	// A distance cannot be negative, so its interval has no lower limit: an exact match with a
	// positive bound is inside, a distance whose lower end passes the limit is outside, and one
	// whose ends straddle it is neither.
	lo := math.Inf(-1)
	require.Equal(t, jointcheck.Inside, jointcheck.Within(0, 0.01, lo, 0.1))
	require.Equal(t, jointcheck.Outside, jointcheck.Within(0.2, 0.01, lo, 0.1))
	require.Equal(t, jointcheck.Straddle, jointcheck.Within(0.1, 0.01, lo, 0.1))
	require.Equal(t, jointcheck.Straddle, jointcheck.Within(0, 0.01, 0, 0.1), "a lower limit of 0 cannot be met by a bounded zero")
}

func TestLineOffset(t *testing.T) {
	l := jointcheck.Line{Point: r3.Vec{X: 1}, Dir: r3.Vec{Z: 1}}
	angle, d := jointcheck.LineOffset(l, jointcheck.Line{Point: r3.Vec{X: 1, Y: 1, Z: 9}, Dir: r3.Vec{Z: -1}})
	require.InDelta(t, 0.0, angle, 1e-12, "direction sign does not matter")
	require.InDelta(t, 1.0, d, 1e-12)
	angle, _ = jointcheck.LineOffset(l, jointcheck.Line{Dir: r3.Vec{X: 1}})
	require.InDelta(t, math.Pi/2, angle, 1e-12)
}

func square(x0, y0, x1, y1 float64) [][3]r3.Vec {
	a, b, c, d := r3.Vec{X: x0, Y: y0}, r3.Vec{X: x1, Y: y0}, r3.Vec{X: x1, Y: y1}, r3.Vec{X: x0, Y: y1}
	return [][3]r3.Vec{{a, b, c}, {a, c, d}}
}

func TestOverlapWitness(t *testing.T) {
	n := r3.Vec{Z: 1}
	// Two 10 x 10 squares sharing a 5 x 10 strip: a triangle centroid of one lies in the other.
	p, ok := jointcheck.OverlapWitness(square(0, 0, 10, 10), square(5, 0, 15, 10), n, 0)
	require.True(t, ok)
	require.InDelta(t, 0.0, p.Z, 1e-12)
	// Squares that only touch along an edge share no area.
	_, ok = jointcheck.OverlapWitness(square(0, 0, 10, 10), square(10, 0, 20, 10), n, 0)
	require.False(t, ok)
	// An L-shaped face and a square in its notch: their boxes overlap, their faces do not.
	l := append(square(0, 0, 10, 4), square(0, 4, 4, 10)...)
	_, ok = jointcheck.OverlapWitness(l, square(6, 6, 10, 10), n, 0)
	require.False(t, ok)
	// A margin larger than the overlap refuses a thin strip.
	_, ok = jointcheck.OverlapWitness(square(0, 0, 10, 10), square(9.9, 0, 20, 10), n, 0.5)
	require.False(t, ok)
	// The swapped direction finds a small face inside a large one.
	_, ok = jointcheck.OverlapWitness(square(0, 0, 100, 100), square(40, 40, 60, 60), n, 0)
	require.True(t, ok)
	// A tilted plane works through the same projection.
	tilted := [][3]r3.Vec{{{X: 0, Y: 0, Z: 0}, {X: 10, Y: 0, Z: 10}, {X: 0, Y: 10, Z: 0}},
		{{X: 10, Y: 0, Z: 10}, {X: 10, Y: 10, Z: 10}, {X: 0, Y: 10, Z: 0}}}
	nt, _ := r3.Vec{X: -1, Z: 1}.Normalize()
	_, ok = jointcheck.OverlapWitness(tilted, tilted, nt, 0)
	require.True(t, ok)
}
