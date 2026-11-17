package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/decadtest"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// forkMeshResult is leverForkSketch's fork, 6.5 tall, with a round bore of
// radius 0.4 through it at (0, 33) drawn on a quarter-turned plane, so the
// result is a mesh Boolean result held as finely as its small rounds and bore
// need.
func forkMeshResult(t *testing.T) *decad.Body {
	t.Helper()
	mm := units.Millimeters
	doc := decad.New()
	fork := forkBody(t, doc, 0, mm(6.5), 0, leverForkSketch(false, forkOptions{}))
	w := sketch.NewWorld()
	frame, err := r3.NewFrame(r3.Vec{Z: -1}, r3.Vec{Y: 1}, r3.Vec{X: -1})
	require.NoError(t, err)
	plane, err := w.CreatePlaneFromFrame(frame)
	require.NoError(t, err)
	s, err := w.CreateSketch(plane)
	require.NoError(t, err)
	c := s.CreatePoint(33, 0)
	s.Fix(c)
	s.CreateCircle(c, 0.4)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	bore, err := decad.Extrude(t.Context(), doc, s, s.Profiles()[0], decad.Distance{D: mm(8.5), Dir: decad.Along})
	require.NoError(t, err)
	out, err := decad.Cut(t.Context(), fork, bore)
	require.NoError(t, err)
	require.True(t, tookMeshPath(out), "class %s", decadtest.PayloadClass(out))
	return out
}

// forkCapOuterLoops selects every edge of the outer loops of the fork's two
// caps, each by its start point and kind, as the gallery's edge finishing does.
func forkCapOuterLoops(t *testing.T, body *decad.Body) (*decad.EdgeQuery, int) {
	t.Helper()
	var q *decad.EdgeQuery
	n := 0
	for _, f := range body.Faces() {
		p, ok := f.Surface().(decad.Plane)
		if !ok || math.Abs(math.Abs(p.Frame.N().Z)-1) > 1e-9 {
			continue
		}
		if z := p.Frame.Origin().Z; math.Abs(z) > 1e-9 && math.Abs(z-6.5) > 1e-9 {
			continue
		}
		for _, l := range f.Loops() {
			if !l.IsOuter() {
				continue
			}
			for _, e := range l.Edges() {
				start := e.Start().Position().Value
				preds := []decad.EdgePredicate{decad.EndpointAt(start), decad.Circular()}
				if _, line := e.Curve().(decad.Line3); line {
					preds = []decad.EdgePredicate{decad.EndpointAt(start), decad.ParallelTo(e.End().Position().Value.Sub(start))}
				}
				if q == nil {
					q = decad.Edges(preds...)
				} else {
					q = q.Or(preds...)
				}
				n++
			}
		}
	}
	return q, n
}

// TestLoopChamferNarrowFeatures chamfers both cap outlines of a watch lever's
// fork, a mesh result, through the retained loop chamfer. The dart is 1.2 wide
// and the corner rounds have radius 0.666, so the default cutter's overshoot,
// moving the loop d/2 past the setback, crossed the moved loop over itself; and
// the fork's mesh bound (about 0.00066 mm) is finer than the default cutter's
// 0.015 mm, which the Cut's chain-depth gate refused.
//
// The closed form. The outline has perimeter P (lines and arcs), two 90° convex
// corners and a smooth turning of π, so its inward mitred offset by t has area
// A - P·t + (2 + π/2)·t² while the offset stays simple (t < 0.6). A 45° chamfer
// of depth d removes ∫₀^d P·u - (2 + π/2)·u² du = P·d²/2 - (2 + π/2)·d³/3 per cap.
//
// Shown red: without the bounded overshoot the 0.5 mm case refuses with the cutter's
// self-crossing; without the fine cutter they refuse at the chain-depth gate.
func TestLoopChamferNarrowFeatures(t *testing.T) {
	t.Parallel()
	const (
		ky  = 38.874650636459869
		gc  = 1.1338355972126268
		dr  = 0.59999999999999998
		tip = 42.149999999999999
		kc  = 2.3338355972126268
	)
	rc, rg := 3-kc, gc-dr
	perimeter := 2*(ky-30) + 6 + 2*math.Pi*rc + 2*math.Pi*rg + 2*(tip-ky) + math.Pi*dr
	for _, d := range []float64{0.3, 0.5} {
		t.Run(units.Millimeters(d).String(), func(t *testing.T) {
			t.Parallel()
			part := forkMeshResult(t)
			q, n := forkCapOuterLoops(t, part)
			require.Equal(t, 20, n)
			before, err := part.Volume()
			require.NoError(t, err)
			out, err := decad.Chamfer(t.Context(), part, q.Exactly(n), units.Millimeters(d))
			require.NoError(t, err)
			after, err := out.Volume()
			require.NoError(t, err)
			want := 2 * (perimeter*d*d/2 - (2+math.Pi/2)*d*d*d/3)
			removed := volumeMM(t, before) - volumeMM(t, after)
			require.InDelta(t, want, removed, boundMM3(t, before)+boundMM3(t, after))
			_, err = out.Tessellate(t.Context(), units.Millimeters(0.05), decad.WithVerification(decad.VerifyAll))
			require.NoError(t, err)
		})
	}
}

// TestLoopChamferDartWiderThanHalf refuses a 0.6 mm chamfer of the fork's cap
// outlines: the dart is 1.2 wide, so the bevels from its two sides meet at its
// centre line and the cap's moved loop crosses itself, and its tip round of
// radius 0.6 empties.
func TestLoopChamferDartWiderThanHalf(t *testing.T) {
	t.Parallel()
	part := forkMeshResult(t)
	q, n := forkCapOuterLoops(t, part)
	_, err := decad.Chamfer(t.Context(), part, q.Exactly(n), units.Millimeters(0.6))
	require.ErrorIs(t, err, decad.ErrDegenerate)
}
