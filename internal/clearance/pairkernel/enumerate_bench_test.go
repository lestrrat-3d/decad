package pairkernel_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/boundaryaudit"
	"github.com/lestrrat-3d/decad/internal/clearance/pairkernel"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topo/topotest"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// tiltedBox builds the box [u0, u1] × [v0, v1] × [0, h] turned by angle
// radians about the diagonal (1, 1, 1) through the origin, the placement the
// tilted linkage scenes give every link.
func tiltedBox(tb testing.TB, u0, v0, u1, v1, h, angle float64) *topo.Body {
	tb.Helper()
	xform, err := r3.Rotation(r3.NewVec(1, 1, 1), units.Radians(angle))
	require.NoError(tb, err)
	return placedBox(tb, u0, v0, u1, v1, h, xform)
}

// placedBox builds the box [u0, u1] × [v0, v1] × [0, h] placed by xform.
func placedBox(tb testing.TB, u0, v0, u1, v1, h float64, xform r3.Transform) *topo.Body {
	tb.Helper()
	pts := []sectionrecord.Point2{{U: u0, V: v0}, {U: u1, V: v0}, {U: u1, V: v1}, {U: u0, V: v1}}
	segs := make([]sectionrecord.CurveSegment, len(pts))
	for i := range pts {
		segs[i] = sectionrecord.LineSeg{Start: pts[i], End: pts[(i+1)%len(pts)], TStart: 0, TEnd: 1}
	}
	frame, err := r3.NewFrame(r3.Vec{}, r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(tb, err)
	payload := prismrecord.Payload{
		Profile: momentinput.Profile{Outer: sectionrecord.LoopRecord{Segments: segs}},
		Frame:   frame,
		Z0:      0,
		Z1:      h,
		Xform:   xform,
	}
	env := topo.Env{Doc: &topo.Document{}, Ref: 7, Alloc: topotest.Alloc{}}
	body, err := prismrecord.Build(tb.Context(), env, payload, freeform.NewFreeformWork())
	require.NoError(tb, err)
	return body
}

// tiltedBoxPairs is a crank-and-stop scene at several crank angles: a long
// thin link turned with the crank against a fixed block it clears.
func tiltedBoxPairs(tb testing.TB) [][2]*topo.Body {
	tb.Helper()
	stop := tiltedBox(tb, 60, -20, 70, 20, 10, 0.4)
	var out [][2]*topo.Body
	for _, angle := range []float64{0.1, 0.25, 0.4, 0.55, 0.7} {
		out = append(out, [2]*topo.Body{tiltedBox(tb, 0, -3, 50, 3, 8, angle), stop})
	}
	return out
}

// BenchmarkPairTiltedBoxes times the pair kernel over the tilted scene with
// a warm carrier cache, so the cell walk is what it measures.
func BenchmarkPairTiltedBoxes(b *testing.B) {
	pairs := tiltedBoxPairs(b)
	cache := &pairkernel.Cache{}
	for _, p := range pairs {
		_, err := pairkernel.Pair(b.Context(), p[0], p[1], false, cache)
		require.NoError(b, err)
	}
	b.ReportAllocs()
	for b.Loop() {
		for _, p := range pairs {
			if _, err := pairkernel.Pair(b.Context(), p[0], p[1], false, cache); err != nil {
				b.Fatal(err)
			}
		}
	}
}

// TestPairTiltedBoxesDisjoint pins what the benchmark scene measures: every
// pair is proven disjoint with a positive gap interval.
func TestPairTiltedBoxesDisjoint(t *testing.T) {
	t.Parallel()
	for i, p := range tiltedBoxPairs(t) {
		res, err := pairkernel.Pair(t.Context(), p[0], p[1], false, &pairkernel.Cache{})
		require.NoError(t, err)
		require.Equal(t, boundaryaudit.PairDisjoint, res.Verdict, "pair %d", i)
		require.Greater(t, res.Lo, 0.0, "pair %d", i)
		require.LessOrEqual(t, res.Lo, res.Hi, "pair %d", i)
	}
}
