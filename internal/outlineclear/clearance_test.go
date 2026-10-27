package outlineclear_test

import (
	"math"
	"math/rand"
	"testing"
	"time"

	"github.com/lestrrat-3d/decad/internal/outlineclear"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

var (
	origin = r3.NewVec(0, 0, 0)
	axisU  = r3.NewVec(1, 0, 0)
	axisV  = r3.NewVec(0, 1, 0)
)

// star returns a counter-clockwise star-shaped polygon of n points about
// (cx, cy) whose radius lies between rmin and rmax.
func star(rng *rand.Rand, cx, cy, rmin, rmax float64, n int) []r3.Vec {
	pts := make([]r3.Vec, n)
	for i := range pts {
		a := 2 * math.Pi * (float64(i) + 0.8*rng.Float64()) / float64(n)
		r := rmin + (rmax-rmin)*rng.Float64()
		pts[i] = r3.NewVec(cx+r*math.Cos(a), cy+r*math.Sin(a), 0)
	}
	return pts
}

func circle(cx, cy, r float64, n int) []r3.Vec {
	pts := make([]r3.Vec, n)
	for i := range pts {
		a := 2 * math.Pi * float64(i) / float64(n)
		pts[i] = r3.NewVec(cx+r*math.Cos(a), cy+r*math.Sin(a), 0)
	}
	return pts
}

// bruteClearance is the O(n*m) reading: any proper crossing or a vertex inside
// the other polygon is overlap, else the least segment-pair distance.
func bruteClearance(a, b []r3.Vec) (float64, bool) {
	best := math.Inf(1)
	for i := range a {
		p, q := a[i], a[(i+1)%len(a)]
		for j := range b {
			r, s := b[j], b[(j+1)%len(b)]
			if properCross(p, q, r, s) {
				return 0, true
			}
			best = min(best, pointSeg(p, r, s), pointSeg(q, r, s), pointSeg(r, p, q), pointSeg(s, p, q))
		}
	}
	if contains(b, a[0]) || contains(a, b[0]) {
		return 0, true
	}
	return best, false
}

func cr(o, p, q r3.Vec) float64 { return (p.X-o.X)*(q.Y-o.Y) - (p.Y-o.Y)*(q.X-o.X) }

func properCross(p, q, r, s r3.Vec) bool {
	return cr(r, s, p)*cr(r, s, q) < 0 && cr(p, q, r)*cr(p, q, s) < 0
}

func pointSeg(c, p, q r3.Vec) float64 {
	d := q.Sub(p)
	t := math.Max(0, math.Min(1, c.Sub(p).Dot(d)/d.Dot(d)))
	return c.Sub(p.Add(d.Scale(t))).Len()
}

// contains is the even-odd test, valid for these simple polygons.
func contains(poly []r3.Vec, c r3.Vec) bool {
	in := false
	for i := range poly {
		p, q := poly[i], poly[(i+1)%len(poly)]
		if (p.Y > c.Y) != (q.Y > c.Y) && c.X < p.X+(c.Y-p.Y)/(q.Y-p.Y)*(q.X-p.X) {
			in = !in
		}
	}
	return in
}

func TestClearanceMatchesBruteForceOnRandomPolygons(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewSource(7))
	overlaps, apart := 0, 0
	for range 400 {
		n, m := 3+rng.Intn(60), 3+rng.Intn(60)
		a := star(rng, 0, 0, 1, 3, n)
		b := star(rng, 8*rng.Float64()-4, 8*rng.Float64()-4, 0.5, 3, m)
		want, wantOverlap := bruteClearance(a, b)
		got, err := outlineclear.Clearance(t.Context(), origin, axisU, axisV, [][]r3.Vec{a}, [][]r3.Vec{b})
		require.NoError(t, err)
		require.Equal(t, wantOverlap, got.Overlap)
		require.InDelta(t, want, got.Distance, 1e-9)
		if wantOverlap {
			overlaps++
			continue
		}
		apart++
		require.InDelta(t, got.Distance, got.A.Sub(got.B).Len(), 1e-9, "the reported points attain the distance")
	}
	require.Greater(t, overlaps, 20)
	require.Greater(t, apart, 20)
}

func TestClearanceOfLargeOutlinesTakesMilliseconds(t *testing.T) {
	t.Parallel()
	a := [][]r3.Vec{circle(0, 0, 10, 5000)}
	b := [][]r3.Vec{circle(25, 0, 10, 5000)}
	start := time.Now()
	got, err := outlineclear.Clearance(t.Context(), origin, axisU, axisV, a, b)
	require.NoError(t, err)
	require.Less(t, time.Since(start), 2*time.Second)
	require.False(t, got.Overlap)
	require.InDelta(t, 5, got.Distance, 1e-3)
	require.InDelta(t, 10, got.A.X, 1e-3)
	require.InDelta(t, 15, got.B.X, 1e-3)
}
