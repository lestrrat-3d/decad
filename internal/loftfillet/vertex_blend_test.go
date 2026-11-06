package loftfillet_test

import (
	"math"
	"testing"

	"github.com/stretchr/testify/require"

	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/loftfillet"
	"github.com/lestrrat-3d/decad/internal/offset2d"
	"github.com/lestrrat-3d/decad/internal/sectionaudit"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/lestrrat-3d/units"
)

// vertexStep is a radius-3 root circle fragment about the origin, a radial line
// of the given length leaving its end at angle 2π·0.97, and a straight fit spline
// from the line's far end turned by lean from the radial direction toward the
// side away from the fragment. It returns the walks, the corner without its
// seed, the vertex and the fit spline's other end. mirrored reflects the section across the U axis
// and reverses its travel, so the fit spline arrives and the circle leaves.
func vertexStep(t *testing.T, length, lean float64, mirrored bool) ([]survey2d.SideWalk, loftfillet.VertexCorner,
	sectionrecord.Point2, sectionrecord.Point2) {
	t.Helper()
	const tEnd = 0.97
	a := 2 * math.Pi * tEnd
	sign := 1.0
	if mirrored {
		sign = -1
	}
	pt := func(rad, angle float64) sectionrecord.Point2 {
		return sectionrecord.Point2{U: rad * math.Cos(angle), V: sign * rad * math.Sin(angle)}
	}
	foot, vertex := pt(3, a), pt(3+length, a)
	tip := sectionrecord.Point2{U: vertex.U + math.Cos(a+lean), V: vertex.V + sign*math.Sin(a+lean)}
	circle := sectionrecord.CircleSeg{Radius: units.Millimeters(3), CCW: true, TStart: 0.9, TEnd: tEnd}
	line := sectionrecord.CurveSegment(sectionrecord.LineSeg{Start: foot, End: vertex, TEnd: 1})
	fit := sectionrecord.CurveSegment(sectionrecord.FitSplineSeg{Fit: []sectionrecord.Point2{vertex, tip}, TEnd: 1})
	segments := []sectionrecord.CurveSegment{circle, line, fit}
	if mirrored {
		circle = sectionrecord.CircleSeg{Radius: units.Millimeters(3), CCW: true, TStart: 1 - tEnd, TEnd: 0.1}
		line = sectionrecord.LineSeg{Start: vertex, End: foot, TEnd: 1}
		fit = sectionrecord.FitSplineSeg{Fit: []sectionrecord.Point2{tip, vertex}, TEnd: 1}
		segments = []sectionrecord.CurveSegment{fit, line, circle}
	}
	walks := make([]survey2d.SideWalk, len(segments))
	for i, seg := range segments {
		w, err := boundarywalk.WalkOf(seg, freeform.NewFreeformWork())
		require.NoError(t, err)
		walks[i] = survey2d.SideWalk{SegmentWalk: w, Segs: []int{i}}
	}
	c := loftfillet.VertexCorner{Circle: circle, CircleWalk: walks[0].SegmentWalk, Line: walks[1].SegmentWalk,
		Fit: walks[2].SegmentWalk, CircleArrives: true}
	if mirrored {
		c = loftfillet.VertexCorner{Circle: circle, CircleWalk: walks[2].SegmentWalk, Line: walks[1].SegmentWalk,
			Fit: walks[0].SegmentWalk}
	}
	return walks, c, vertex, tip
}

func TestVertexBlendRestsTheBallOnTheLineFlankVertex(t *testing.T) {
	t.Parallel()
	const length = 0.1
	for _, mirrored := range []bool{false, true} {
		name, corner := "circle, line, fit spline", 1
		if mirrored {
			name, corner = "fit spline, line, circle", 2
		}
		seeded := func(t *testing.T, lean, radius float64) (loftfillet.VertexCorner, sectionrecord.Point2, sectionrecord.Point2) {
			walks, c, vertex, tip := vertexStep(t, length, lean, mirrored)
			seed, err := offset2d.Fillet(walks, corner, radius, sectionaudit.Tolerance)
			require.NoError(t, err)
			c.Seed = seed
			return c, vertex, tip
		}
		t.Run(name, func(t *testing.T) {
			t.Parallel()
			t.Run("a flank leaning off the ball", func(t *testing.T) {
				t.Parallel()
				const radius, lean = 0.11, 0.1
				c, vertex, tip := seeded(t, lean, radius)
				blend, onVertex, err := loftfillet.VertexBlend(c, radius)
				require.NoError(t, err)
				require.True(t, onVertex)
				arc, ok := blend.Connector.(sectionrecord.ArcSeg)
				require.True(t, ok)
				foot, end := blend.FA, blend.FB
				if mirrored {
					foot, end = blend.FB, blend.FA
				}
				require.Equal(t, vertex, end, "the connector meets the fit spline at its own end")

				// An independent centre: walk the root circle's radius-(3+r) offset
				// from the vertex's angle toward the fragment until the ball reaches
				// the vertex.
				a := math.Atan2(vertex.V, vertex.U)
				back := -1.0
				if mirrored {
					back = 1
				}
				at := func(s float64) sectionrecord.Point2 {
					return sectionrecord.Point2{U: (3 + radius) * math.Cos(a+back*s), V: (3 + radius) * math.Sin(a+back*s)}
				}
				reach := func(s float64) float64 {
					p := at(s)
					return math.Hypot(p.U-vertex.U, p.V-vertex.V) - radius
				}
				lo, hi := 0.0, 0.5
				require.Negative(t, reach(lo))
				require.Positive(t, reach(hi))
				for range 200 {
					mid := (lo + hi) / 2
					if reach(mid) < 0 {
						lo = mid
					} else {
						hi = mid
					}
				}
				want := at(lo)
				require.InDelta(t, want.U, arc.Center.U, 1e-12)
				require.InDelta(t, want.V, arc.Center.V, 1e-12)
				require.InDelta(t, 3*want.U/(3+radius), foot.U, 1e-12, "the foot is the ball's tangent point")
				require.InDelta(t, 3*want.V/(3+radius), foot.V, 1e-12)

				// The ball's tangent at the vertex turns off the radial line toward
				// the flank, so the kink it leaves is less than the flank's lean.
				angle := func(au, av, bu, bv float64) float64 {
					return math.Atan2(math.Abs(au*bv-av*bu), au*bu+av*bv)
				}
				ballU, ballV := -(vertex.V - want.V), vertex.U-want.U
				if ballU*vertex.U+ballV*vertex.V < 0 {
					ballU, ballV = -ballU, -ballV
				}
				flankU, flankV := tip.U-vertex.U, tip.V-vertex.V
				turn := angle(ballU, ballV, vertex.U, vertex.V)
				kink := angle(ballU, ballV, flankU, flankV)
				require.InDelta(t, lean, angle(vertex.U, vertex.V, flankU, flankV), 1e-12)
				require.Greater(t, turn, 0.0)
				require.InDelta(t, lean-turn, kink, 1e-12)
			})
			t.Run("a flank leaning into the ball", func(t *testing.T) {
				t.Parallel()
				c, _, _ := seeded(t, -0.1, 0.11)
				blend, onVertex, err := loftfillet.VertexBlend(c, 0.11)
				require.NoError(t, err)
				require.False(t, onVertex, "the ball touches the fit spline beyond the vertex")
				require.Nil(t, blend)
			})
			t.Run("a kink no smaller than the line's own turn", func(t *testing.T) {
				// A ball smaller than the line is tangent to the circle below the
				// vertex, so through the vertex it turns away from the flank.
				t.Parallel()
				c, _, _ := seeded(t, 0.1, 0.09)
				_, _, err := loftfillet.VertexBlend(c, 0.09)
				require.ErrorContains(t, err, "kink")
			})
		})
	}
}
