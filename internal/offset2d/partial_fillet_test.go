package offset2d_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/capband"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/offset2d"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/stretchr/testify/require"
)

func partialLine(u0, v0, u1, v1 float64) sectionrecord.CurveSegment {
	return sectionrecord.LineSeg{Start: sectionrecord.Point2{U: u0, V: v0}, End: sectionrecord.Point2{U: u1, V: v1}, TStart: 0, TEnd: 1}
}

func partialArc(cu, cv, u0, v0, u1, v1 float64) sectionrecord.CurveSegment {
	return sectionrecord.ArcSeg{Center: sectionrecord.Point2{U: cu, V: cv}, Start: sectionrecord.Point2{U: u0, V: v0}, End: sectionrecord.Point2{U: u1, V: v1}, TStart: 0, TEnd: 1}
}

// TestPartialFilletContourHoldsExactFeet offsets two loops with every walk
// selected, so each corner takes the selected pair's CornerJoin arm. A
// partial fillet's meridians stand on filletband.OffsetJoins' feet, which
// hold the exact offset foot wherever a float64 holds it, so the contour
// must hold the same: P8's top loop at 2 puts its tangent foot at (0, 3) on
// (2, 3), and a loop whose quarter arc about the origin ends at (0, 2) on a
// reflex corner puts its connector arc's start, the arc's foot at 0.5, on
// (0, 1.5). CornerJoin alone steps both along an arc tangent read through
// math.Sincos and lands a rounding off them. Each connector segment runs
// between the join's two feet. Shown to fail with partialContour's
// ExactCornerFeet call deleted (the tangent foot and the connector's start
// kept CornerJoin's rounding).
func TestPartialFilletContourHoldsExactFeet(t *testing.T) {
	t.Parallel()
	for _, tc := range []struct {
		name   string
		loop   sectionrecord.LoopRecord
		amount float64
		vertex sectionrecord.Point2
		foot   func(offset2d.Join) sectionrecord.Point2
		want   sectionrecord.Point2
	}{
		{"tangent", sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{
			partialLine(3, 0, 37, 0), partialArc(37, 3, 37, 0, 40, 3),
			partialLine(40, 3, 40, 17), partialArc(37, 17, 40, 17, 37, 20),
			partialLine(37, 20, 3, 20), partialArc(3, 17, 3, 20, 0, 17),
			partialLine(0, 17, 0, 3), partialArc(3, 3, 0, 3, 3, 0),
		}}, 2, sectionrecord.Point2{U: 0, V: 3},
			func(j offset2d.Join) sectionrecord.Point2 { return j.M }, sectionrecord.Point2{U: 2, V: 3}},
		{"connector", sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{
			partialArc(0, 0, 2, 0, 0, 2), partialLine(0, 2, 0, 5), partialLine(0, 5, -5, 5),
			partialLine(-5, 5, -5, -5), partialLine(-5, -5, 2, -5), partialLine(2, -5, 2, 0),
		}}, 0.5, sectionrecord.Point2{U: 0, V: 2},
			func(j offset2d.Join) sectionrecord.Point2 { return j.PA }, sectionrecord.Point2{U: 0, V: 1.5}},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			budget := proofbound.NewWorkBudget(t.Context())
			walks, err := capband.LoopWalks(budget, tc.loop, freeform.NewFreeformWork())
			require.NoError(t, err)
			n := len(walks)
			selected, sphere, amounts := make([]bool, n), make([]bool, n), make([]float64, n)
			for i := range n {
				selected[i], amounts[i] = true, tc.amount
			}
			segs, joins, _, capArc, err := offset2d.PartialFilletContour(budget, walks, selected, sphere, amounts, offset2d.ShellTol)
			require.NoError(t, err)
			found := 0
			for k, j := range joins {
				if j.Arc {
					arc, ok := segs[capArc[k]].(sectionrecord.ArcSeg)
					require.True(t, ok)
					// A clockwise ArcSegment records its walk reversed.
					require.Equal(t, [2]sectionrecord.Point2{j.PB, j.PA}, [2]sectionrecord.Point2{arc.Start, arc.End})
				}
				if (sectionrecord.Point2{U: j.VertU, V: j.VertV}) != tc.vertex {
					continue
				}
				require.Equal(t, tc.want, tc.foot(j))
				found++
			}
			require.Equal(t, 1, found)
		})
	}
}
