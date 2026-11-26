package decad_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"

	"github.com/lestrrat-3d/decad"
)

// shallowConcaveTurn is the turn of the tongue's concave ends: one chord of a round held as 16 chords to the turn.
const shallowConcaveTurn = 5.625 * math.Pi / 180

// tongueOnBase is a block 5 mm thick with a tongue of the given width and height rising from its top at y = 5, and the
// selection of the tongue's right side, top and left side on the top cap. The tongue's sides meet the block's top
// through short walls turned shallowConcaveTurn off them, so both ends of the run are concave corners turned by that
// angle beside a line. When faceted is set the block is a mesh Boolean result: a tapered hole far from the tongue is
// cut from it, whose chords give the result's faces a bound, so the Cut cannot settle a touch on the next wall exactly.
func tongueOnBase(t *testing.T, width, height float64, faceted bool) (*decad.Body, *decad.EdgeQuery) {
	t.Helper()
	yb := 5 - math.Cos(shallowConcaveTurn)
	fR, vR := [2]float64{width + math.Sin(shallowConcaveTurn), yb}, [2]float64{width, 5}
	tR, tL := [2]float64{width, 5 + height}, [2]float64{0, 5 + height}
	vL, fL := [2]float64{0, 5}, [2]float64{-math.Sin(shallowConcaveTurn), yb}
	pts := [][2]float64{{-10, 0}, {12, 0}, {12, yb}, fR, vR, tR, tL, vL, fL, {-10, yb}}
	doc := decad.New()
	body := prismFrom(t, doc, 0, 5, func(s *sketch.Sketch) {
		ps := make([]*sketch.Point, len(pts))
		for i, p := range pts {
			ps[i] = s.CreatePoint(p[0], p[1])
			s.Fix(ps[i])
		}
		for i, p := range ps {
			s.CreateLine(p, ps[(i+1)%len(ps)])
		}
	})
	if faceted {
		// A tapered hole, a ruled loft between two circles, far from the tongue.
		w := sketch.NewWorld()
		circle := func(z, r float64) (*sketch.Sketch, *sketch.Profile) {
			pl, err := w.CreateOffsetPlane(w.XY(), z)
			require.NoError(t, err)
			s, err := w.CreateSketch(pl)
			require.NoError(t, err)
			c := s.CreatePoint(-7, 2)
			s.Fix(c)
			s.CreateCircle(c, r)
			_, err = s.Solve(t.Context())
			require.NoError(t, err)
			for _, p := range s.Profiles() {
				if p.Valid {
					return s, p
				}
			}
			require.Fail(t, "no profile")
			return nil, nil
		}
		s0, p0 := circle(-1, 1.5)
		s1, p1 := circle(6, 0.7)
		hole, err := decad.Loft(t.Context(), doc, s0, p0, s1, p1)
		require.NoError(t, err)
		body, err = decad.Cut(body, hole).Do(t.Context())
		require.NoError(t, err)
	}
	return body, topEdges(lineEdge(1, vR, tR), lineEdge(1, tR, tL), lineEdge(1, tL, vL))
}

// TestChamferCapRunShallowConcaveEnd chamfers a run that ends at concave corners turned by a few degrees beside a line
// and comes back past its start: up a tongue's side, across its top and down its other side. The end clip at such a
// corner stops the bevel on the next wall's plane carried back over the run, which leaves the run's band about
// reach/sin(turn) from the corner. The clip holds only the band within the cutter's reach of the line, so the tongue's
// top and other side, which lie inside the clip's sector but farther than that from the line, keep their bevel. The
// removed volume is the three wedges, less d^3/3 for each right-angle mitre at the top and d^3 cot(turn)/6 for each
// concave end, and the corner relief keeps the clipped cutter off the next wall on a prism and on a mesh Boolean result
// alike, so neither Cut takes a gap shift. A tongue shorter than the taper still comes back into the clip: the next
// wall's plane would cut the bevel of its top.
func TestChamferCapRunShallowConcaveEnd(t *testing.T) {
	t.Parallel()
	const d, width, height = 0.5, 2.1, 12.0
	for _, c := range []struct {
		name    string
		faceted bool
	}{{"prism", false}, {"mesh Boolean result", true}} {
		t.Run(c.name, func(t *testing.T) {
			t.Parallel()
			body, sel := tongueOnBase(t, width, height, c.faceted)
			before, err := body.Volume()
			require.NoError(t, err)
			out, err := decad.Chamfer(t.Context(), body, sel, units.Millimeters(d))
			require.NoError(t, err)
			require.True(t, out.IsSolid())
			for _, diag := range out.OperationDiagnostics() {
				require.NotEqual(t, decad.OpDiagGapShiftApplied, diag.Code)
			}
			m, err := out.Volume()
			require.NoError(t, err)
			removed := d*d/2*(height+width+height) - 2*d*d*d/3 - 2*d*d*d/math.Tan(shallowConcaveTurn)/6
			// The corner relief keeps at most 0.004^2 turn d / 2 at each end.
			kept := 2 * 0.004 * 0.004 * shallowConcaveTurn * d / 2
			bound := m.Bound.Base() + before.Bound.Base()
			require.Less(t, bound, 0.02)
			want := before.Value.Base() - removed
			require.GreaterOrEqual(t, m.Value.Base(), want-bound)
			require.LessOrEqual(t, m.Value.Base(), want+kept+bound)

			// Half the setback below the cap the bevel has moved the tongue's top corners d/2 in from both walls.
			o := sectionAt(t, out, d/2, 0.01)
			off, err := o.OffSurface.In(units.Millimeter)
			require.NoError(t, err)
			require.Less(t, nearestLoopPoint(o, width-d/2, 5+height-d/2), off+1e-3, "right corner")
			require.Less(t, nearestLoopPoint(o, d/2, 5+height-d/2), off+1e-3, "left corner")
		})
	}
	t.Run("a tongue shorter than the taper is refused", func(t *testing.T) {
		t.Parallel()
		// The next wall's plane carried back leaves the band d/tan(turn) = 5.1 mm from the corner, past the top.
		body, sel := tongueOnBase(t, width, 3, false)
		_, err := decad.Chamfer(t.Context(), body, sel, units.Millimeters(d))
		require.ErrorIs(t, err, decad.ErrUnsupported)
		require.ErrorContains(t, err, "comes back into the clip at its concave end")
	})
}
