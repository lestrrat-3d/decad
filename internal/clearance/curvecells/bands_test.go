package curvecells_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/clearance"
	"github.com/lestrrat-3d/decad/internal/clearance/curvecells"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// rimOn is a circle edge of radius r about centre in the plane normal to
// axis, over the azimuth window ang.
func rimOn(centre, axis r3.Vec, r float64, ang clearance.AngWindow) *clearance.CEdge {
	u := clearance.PerpTo(axis)
	return &clearance.CEdge{
		Center: centre,
		Axis:   axis,
		RefU:   u,
		RefV:   axis.Cross(u),
		Radius: r,
		Ang:    ang,
		Box:    clearance.CircleBox(centre, axis, r),
	}
}

func fullRim(centre, axis r3.Vec, r float64) *clearance.CEdge {
	return rimOn(centre, axis, r, clearance.AngWindow{Full: true})
}

// sampledRimGap is the minimum distance over n azimuth samples of each edge's
// window: an upper bound on the edges' true gap.
func sampledRimGap(a, b *clearance.CEdge, n int) float64 {
	pts := func(e *clearance.CEdge) []r3.Vec {
		lo, hi := 0.0, 2*math.Pi
		if !e.Ang.Full {
			lo, hi = e.Ang.Lo, e.Ang.Hi
		}
		out := make([]r3.Vec, 0, n+1)
		for i := 0; i <= n; i++ {
			out = append(out, e.At(lo+(hi-lo)*float64(i)/float64(n)))
		}
		return out
	}
	pa, pb := pts(a), pts(b)
	best := math.Inf(1)
	for _, p := range pa {
		for _, q := range pb {
			best = math.Min(best, p.Sub(q).Len())
		}
	}
	return best
}

// band reads the one contribution a band cell made, which sink.Interval would
// clamp to its upper end, and checks the band never claims Exact.
func band(t *testing.T, sink *clearance.CellSink) (float64, float64) {
	t.Helper()
	require.Len(t, sink.Contribs, 1)
	c := sink.Contribs[0]
	require.False(t, c.Exact)
	return c.Lo, c.Hi
}

func edgeEdge(t *testing.T, a, b *clearance.CEdge, scale float64) (*clearance.CellSink, float64) {
	t.Helper()
	tol := 1e-9 * scale
	k := curvecells.New(t.Context(), tol, tol)
	sink := &clearance.CellSink{}
	k.EdgeEdge(a, b, sink)
	require.NoError(t, k.Err())
	return sink, tol
}

// TestCirclePairBand pins agent-docs/clearance-band-design.md §4.1: two
// circular edges whose spine pair has no closed form read the circle pair
// band. A bore's rim of radius 5.5 sits 1e-14 off a pin rim's axis, flush
// (gap 0.5) and 2 mm above it (gap √4.25), and each reads within 2·tol of the
// gap, never Exact. Arcs with disjoint windows keep a lower bound at or below
// the sampled gap and an upper bound at or above it. A rim of radius 5.5
// tilted 0.05 rad with its centre 1 above a rim of radius 5 on the same axis
// reads 0.8769 at or below the 4096-sample minimum; with the ρ_B·s term
// dropped from the height it reads 1.115 and fails the sampled check. Equal
// radii at equal height read a bound at or below the tolerance and keep the
// coarse enclosure.
func TestCirclePairBand(t *testing.T) {
	t.Parallel()
	z := r3.NewVec(0, 0, 1)
	centre := r3.NewVec(48, 0, 0)
	arc := func(c r3.Vec, r, lo, hi float64) *clearance.CEdge {
		return rimOn(c, z, r, clearance.NewAngWindow(lo, hi))
	}
	tiltAxis := r3.NewVec(0, -math.Sin(0.05), math.Cos(0.05))
	for _, c := range []struct {
		name string
		a, b *clearance.CEdge
		want float64
	}{
		{"flush rims", fullRim(centre, z, 5), fullRim(r3.NewVec(48, 1e-14, 0), z, 5.5), 0.5},
		{"rims 2 apart", fullRim(centre, z, 5), fullRim(r3.NewVec(48, 1e-14, 2), z, 5.5), math.Sqrt(4.25)},
		{"swapped rims", fullRim(r3.NewVec(48, 1e-14, 2), z, 5.5), fullRim(centre, z, 5), math.Sqrt(4.25)},
		{"matching arcs", arc(centre, 5, 0, 1), arc(r3.NewVec(48, 1e-14, 0), 5.5, 0, 1), 0.5},
	} {
		t.Run(c.name, func(t *testing.T) {
			t.Parallel()
			sink, tol := edgeEdge(t, c.a, c.b, 62)
			require.False(t, sink.Unsure)
			lo, hi := band(t, sink)
			require.InDelta(t, c.want, lo, 2*tol)
			require.InDelta(t, c.want, hi, 2*tol)
			require.LessOrEqual(t, lo, c.want)
		})
	}
	t.Run("disjoint arc windows", func(t *testing.T) {
		t.Parallel()
		a, b := arc(centre, 5, 0, 1), arc(r3.NewVec(48, 1e-14, 0), 5.5, 3, 4)
		sink, _ := edgeEdge(t, a, b, 62)
		require.False(t, sink.Unsure)
		lo, hi := band(t, sink)
		truth := sampledRimGap(a, b, 2048)
		require.LessOrEqual(t, lo, truth)
		require.InDelta(t, 0.5, lo, 1e-6, `the carriers' band bounds the arcs from below`)
		require.GreaterOrEqual(t, hi, truth-1e-6)
	})
	t.Run("tilted rim", func(t *testing.T) {
		t.Parallel()
		a, b := fullRim(r3.Vec{}, z, 5), fullRim(r3.NewVec(0, 0, 1), tiltAxis, 5.5)
		sink, _ := edgeEdge(t, a, b, 10)
		require.False(t, sink.Unsure)
		lo, hi := band(t, sink)
		truth := sampledRimGap(a, b, 4096)
		require.GreaterOrEqual(t, hi, truth-1e-12)
		require.LessOrEqual(t, lo, truth+1e-12)
		require.InDelta(t, 0.8769, lo, 1e-4)
	})
	t.Run("equal radii at equal height", func(t *testing.T) {
		t.Parallel()
		a, b := fullRim(centre, z, 5), fullRim(r3.NewVec(48, 1e-14, 0), z, 5)
		sink, tol := edgeEdge(t, a, b, 62)
		lo, hi := band(t, sink)
		require.LessOrEqual(t, lo, tol, `a rim that fills its partner certifies no gap`)
		var coarse clearance.CellSink
		coarse.Coarse(a.Box, b.Box, a.Witnesses(tol), b.Witnesses(tol))
		require.Equal(t, coarse.Contribs[0].Lo, lo, `the coarse enclosure stands`)
		require.Equal(t, coarse.Contribs[0].Hi, hi, `the coarse enclosure stands`)
	})
}

// planeFace is the plane through the origin whose unit normal is Z turned by
// angle about Y, trimmed to the square [-10, 10]² of its own frame. A turned
// plane has a wide box, so a rim above it reaches the cell rather than being
// pruned by box distance.
func planeFace(t *testing.T, angle float64) *clearance.CFace {
	t.Helper()
	var elems []survey2d.SurveyElem
	corners := [][2]float64{{-10, -10}, {10, -10}, {10, 10}, {-10, 10}}
	for i := range corners {
		a, b := corners[i], corners[(i+1)%len(corners)]
		e, ok := survey2d.LineElem(a[0], a[1], b[0], b[1])
		require.True(t, ok)
		elems = append(elems, e)
	}
	f := &clearance.CFace{
		Kind:   clearance.CkPlane,
		O:      r3.Vec{},
		U:      r3.NewVec(math.Cos(angle), 0, -math.Sin(angle)),
		V:      r3.NewVec(0, 1, 0),
		N:      r3.NewVec(math.Sin(angle), 0, math.Cos(angle)),
		Region: clearance.NewRegion2(elems),
	}
	f.Box = clearance.CapBox(f)
	f.Wit = []r3.Vec{{}}
	return f
}

// TestRimHeightBand pins §4.3: a rim of radius 3 a rounding off the normal of
// a plane turned 45 degrees, 2 above the plane, reads within 2·tol of 2 and
// decided. The same rim through the plane, and one a tilt-radius from it, stay
// undecided.
func TestRimHeightBand(t *testing.T) {
	t.Parallel()
	const angle = math.Pi / 4
	face := planeFace(t, angle)
	normal := face.N
	// The rim's axis is the normal turned 1e-12 rad about Y.
	tilted, ok := normal.Add(face.U.Scale(1e-12)).Normalize()
	require.True(t, ok)
	run := func(t *testing.T, e *clearance.CEdge) (*clearance.CellSink, float64) {
		t.Helper()
		tol := 1e-9 * 20
		k := curvecells.New(t.Context(), tol, tol)
		sink := &clearance.CellSink{}
		k.FaceEdge(face, e, sink)
		require.NoError(t, k.Err())
		return sink, tol
	}
	t.Run("rim over the plane", func(t *testing.T) {
		t.Parallel()
		sink, tol := run(t, fullRim(normal.Scale(2), tilted, 3))
		require.False(t, sink.Unsure)
		lo, hi := band(t, sink)
		require.InDelta(t, 2, lo, 2*tol)
		require.InDelta(t, 2, hi, 2*tol)
		require.LessOrEqual(t, lo, 2.0)
	})
	t.Run("rim through the plane", func(t *testing.T) {
		t.Parallel()
		sink, _ := run(t, fullRim(r3.Vec{}, tilted, 3))
		require.True(t, sink.Unsure)
	})
	t.Run("rim a tilt-radius from the plane", func(t *testing.T) {
		t.Parallel()
		// The rim's height spans b ± 3e-12 about b = 1e-12: it does not clear
		// the tolerance and stays undecided.
		sink, _ := run(t, fullRim(normal.Scale(1e-12), tilted, 3))
		require.True(t, sink.Unsure)
	})
}
