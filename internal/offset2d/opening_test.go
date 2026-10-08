package offset2d_test

import (
	"fmt"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/offset2d"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/stretchr/testify/require"
)

// line is a straight walk from a to b.
func line(a, b [2]float64) survey2d.SideWalk {
	du, dv := b[0]-a[0], b[1]-a[1]
	return survey2d.SideWalk{SegmentWalk: survey2d.SegmentWalk{
		StartU: a[0], StartV: a[1], EndU: b[0], EndV: b[1],
		TanInU: du, TanInV: dv, TanOutU: du, TanOutV: dv,
	}}
}

// arc is a circular walk about c of radius r from angle th0 to th1,
// counterclockwise when th1 > th0, its endpoints and tangents read through
// the angles as the walk builder reads them.
func arc(c [2]float64, r, th0, th1 float64) survey2d.SideWalk {
	sense := 1.0
	if th1 < th0 {
		sense = -1
	}
	return survey2d.SideWalk{SegmentWalk: survey2d.SegmentWalk{
		Kind: survey2d.WalkCircular, CU: c[0], CV: c[1], Radius: r, Th0: th0, Th1: th1,
		StartU: c[0] + r*math.Cos(th0), StartV: c[1] + r*math.Sin(th0),
		EndU: c[0] + r*math.Cos(th1), EndV: c[1] + r*math.Sin(th1),
		TanInU: -sense * math.Sin(th0), TanInV: sense * math.Cos(th0),
		TanOutU: -sense * math.Sin(th1), TanOutV: sense * math.Cos(th1),
	}}
}

const openingTol = 1e-9

// TestOpeningJoinCorners reads docs/shell-opening-design.md §2.3's corner
// rows against the cuts §9 derives by hand. Each row names the kept walk k,
// the removed walk r beside it, the end of k the corner is, and the sense.
//
// Shown to fail: dropping the backward branch (always walking into r's span)
// sends the 225° reflex row and the bead walked back red; and reading the
// entering sense off r's own sense alone sends both bead rows walked against
// it red.
func TestOpeningJoinCorners(t *testing.T) {
	r24 := math.Sqrt(24)
	lensK := arc([2]float64{0, 0}, 5, -math.Atan2(4, 3), math.Atan2(4, 3))
	lensR := arc([2]float64{6, 0}, 5, math.Pi-math.Atan2(4, 3), math.Pi+math.Atan2(4, 3))
	for _, tc := range []struct {
		name  string
		k, r  survey2d.SideWalk
		atEnd bool
		s, t  float64
		want  [2]float64
	}{
		// The triangle (0,0), (12,0), (0,9) without its y = 0 leg, t = 1.5.
		{"acute", line([2]float64{12, 0}, [2]float64{0, 9}), line([2]float64{0, 0}, [2]float64{12, 0}), false, 1, 1.5, [2]float64{9.5, 0}},
		{"right", line([2]float64{0, 9}, [2]float64{0, 0}), line([2]float64{0, 0}, [2]float64{12, 0}), true, 1, 1.5, [2]float64{1.5, 0}},
		// The trapezoid (0,0), (14,0), (11,4), (3,4) without its y = 4 side.
		{"obtuse at the end", line([2]float64{14, 0}, [2]float64{11, 4}), line([2]float64{11, 4}, [2]float64{3, 4}), true, 1, 1, [2]float64{9.75, 4}},
		{"obtuse at the start", line([2]float64{3, 4}, [2]float64{0, 0}), line([2]float64{11, 4}, [2]float64{3, 4}), false, 1, 1, [2]float64{4.25, 4}},
		// The triangle without its hypotenuse, t = 3: an oblique removed walk.
		{"oblique removed at the end", line([2]float64{0, 0}, [2]float64{12, 0}), line([2]float64{12, 0}, [2]float64{0, 9}), true, 1, 3, [2]float64{8, 3}},
		{"oblique removed at the start", line([2]float64{0, 9}, [2]float64{0, 0}), line([2]float64{12, 0}, [2]float64{0, 9}), false, 1, 3, [2]float64{3, 6.75}},
		// The L section's 270° corner at (10, 10), t = 2: the perpendicular
		// back along the removed face's carrier.
		{"reflex right", line([2]float64{30, 10}, [2]float64{10, 10}), line([2]float64{10, 10}, [2]float64{10, 30}), true, 1, 2, [2]float64{10, 8}},
		// A 225° corner: the removed walk turns right off the kept one, so its
		// carrier is walked back from the corner into the material.
		{"reflex", line([2]float64{0, 0}, [2]float64{10, 0}), line([2]float64{10, 0}, [2]float64{15, -5}), true, 1, 2, [2]float64{8, 2}},
		// The bead: the chord ρ = 6 under the arc of radius 5 about (5, 6).
		{"line k, arc r, at the end", line([2]float64{0, 6}, [2]float64{10, 6}), arc([2]float64{5, 6}, 5, 0, math.Pi), true, 1, 1, [2]float64{5 + r24, 7}},
		{"line k, arc r, at the start", line([2]float64{0, 6}, [2]float64{10, 6}), arc([2]float64{5, 6}, 5, 0, math.Pi), false, 1, 1, [2]float64{5 - r24, 7}},
		{"line k, arc r, walked back", line([2]float64{0, 6}, [2]float64{10, 6}), arc([2]float64{5, 6}, 5, 0, math.Pi), true, -1, 1, [2]float64{5 + r24, 5}},
		// The D section without its arc, t = 3: the chord's offset x = 3
		// meets the removed circle at (3, ∓4).
		{"line k, arc r, D", line([2]float64{0, 5}, [2]float64{0, -5}), arc([2]float64{0, 0}, 5, -math.Pi/2, math.Pi/2), true, 1, 3, [2]float64{3, -4}},
		// The D section without its chord, t = 1: the arc's offset radius 4
		// on the chord's own line, through the arc's centre.
		{"arc k, line r", arc([2]float64{0, 0}, 5, -math.Pi/2, math.Pi/2), line([2]float64{0, 5}, [2]float64{0, -5}), true, 1, 1, [2]float64{0, 4}},
		// The lens of two radius-5 disks about (0, 0) and (6, 0), t = 1: the
		// right arc's offset radius 4 meets the left arc's circle at
		// (2.25, √10.9375), the crossing reached first from (3, 4).
		{"arc k, arc r", lensK, lensR, true, 1, 1, [2]float64{2.25, math.Sqrt(10.9375)}},
	} {
		t.Run(tc.name, func(t *testing.T) {
			j, err := offset2d.OpeningJoin(tc.k, tc.r, tc.atEnd, tc.s, tc.t, openingTol)
			require.NoError(t, err)
			require.False(t, j.Arc || j.G1)
			v := [2]float64{tc.k.StartU, tc.k.StartV}
			if tc.atEnd {
				v = [2]float64{tc.k.EndU, tc.k.EndV}
			}
			require.Equal(t, v, [2]float64{j.VertU, j.VertV})
			require.InDelta(t, tc.want[0], j.M.U, 1e-12)
			require.InDelta(t, tc.want[1], j.M.V, 1e-12)
		})
	}
}

// TestOpeningJoinKeepsTheFoot reads §2.4's exact-pair row: where the two
// walks' raw tangents have a float dot product of exactly zero, the cut is
// k's offset foot v + s*t along its held unit left normal, bit for bit the
// point the right-angle open chain wrote before the rim rule. Two
// axis-aligned walks are one such pair; (3, 4) against (−4, 3) is another
// whose held unit normal is not exact. An axis-aligned line through a circular
// k's centre takes the centre moved by the offset radius.
//
// Shown to fail: routing every pair here through the float solve moves each
// cut a unit in the last place off the bits asserted.
func TestOpeningJoinKeepsTheFoot(t *testing.T) {
	foot := func(k survey2d.SideWalk, s, tt float64) [2]uint64 {
		tx, ty, _ := offset2d.Normalize(k.TanOutU, k.TanOutV)
		return [2]uint64{math.Float64bits(k.EndU + s*tt*(-ty)), math.Float64bits(k.EndV + s*tt*tx)}
	}
	bits := func(p offset2d.Point) [2]uint64 { return [2]uint64{math.Float64bits(p.U), math.Float64bits(p.V)} }
	for _, tc := range []struct {
		name string
		k, r survey2d.SideWalk
	}{
		{"axis-aligned", line([2]float64{0, 0}, [2]float64{10, 0}), line([2]float64{10, 0}, [2]float64{10, 10})},
		{"oblique", line([2]float64{0, 0}, [2]float64{7, 3}), line([2]float64{7, 3}, [2]float64{4, 10})},
		{"oblique off the origin", line([2]float64{0.5, 0.25}, [2]float64{5.5, 12.25}), line([2]float64{5.5, 12.25}, [2]float64{-6.5, 17.25})},
	} {
		t.Run(tc.name, func(t *testing.T) {
			require.Zero(t, tc.k.TanOutU*tc.r.TanInU+tc.k.TanOutV*tc.r.TanInV)
			for _, s := range []float64{1, -1} {
				j, err := offset2d.OpeningJoin(tc.k, tc.r, true, s, 0.1, openingTol)
				require.NoError(t, err)
				require.Equal(t, foot(tc.k, s, 0.1), bits(j.M))
			}
		})
	}
	// A D section's arc at its top against its chord through the centre,
	// the arc's end the recorded point the walk reads verbatim.
	d := arc([2]float64{0.3, 0.1}, 1.1, -math.Pi/2, math.Pi/2)
	d.EndU, d.EndV = 0.3, 0.1+1.1
	j, err := offset2d.OpeningJoin(d, line([2]float64{d.EndU, d.EndV}, [2]float64{0.3, 0.1 - 1.1}), true, 1, 0.3, openingTol)
	require.NoError(t, err)
	require.Equal(t, offset2d.Point{U: 0.3, V: 0.1 + (1.1 - 0.3)}, j.M)
}

// TestOpeningJoinHoldsAnAxisLevel reads §2.4's float-solve row on a removed
// walk along a section axis: the cut lies on r's carrier, so it holds v's
// coordinate across that axis bit for bit, while the coordinate along r is
// the solve's. The trapezoid (0,0) (14,0) (11,4) (3,4) without its y = 4
// side and the triangle (0,0) (12,0) (0,9) without its x = 0 leg, over a
// range of t, each cut also near its closed form.
//
// Shown to fail: deleting the hold leaves the trapezoid's cut at (9.75, 4)
// for t = 1 a unit in the last place off y = 4.
func TestOpeningJoinHoldsAnAxisLevel(t *testing.T) {
	top := line([2]float64{11, 4}, [2]float64{3, 4})
	leg := line([2]float64{0, 9}, [2]float64{0, 0})
	for _, tt := range []float64{0.3, 0.7, 1, 1.3, 2.9} {
		for _, tc := range []struct {
			name   string
			k, r   survey2d.SideWalk
			atEnd  bool
			alongU bool
			want   [2]float64
		}{
			{"trapezoid at the end", line([2]float64{14, 0}, [2]float64{11, 4}), top, true, true, [2]float64{(44 - 5*tt) / 4, 4}},
			{"trapezoid at the start", line([2]float64{3, 4}, [2]float64{0, 0}), top, false, true, [2]float64{(12 + 5*tt) / 4, 4}},
			{"triangle at the end", line([2]float64{12, 0}, [2]float64{0, 9}), leg, true, false, [2]float64{0, (36 - 5*tt) / 4}},
		} {
			t.Run(fmt.Sprintf("%s, t = %g", tc.name, tt), func(t *testing.T) {
				j, err := offset2d.OpeningJoin(tc.k, tc.r, tc.atEnd, 1, tt, openingTol)
				require.NoError(t, err)
				if tc.alongU {
					require.Equal(t, math.Float64bits(j.VertV), math.Float64bits(j.M.V))
				} else {
					require.Equal(t, math.Float64bits(j.VertU), math.Float64bits(j.M.U))
				}
				require.InDelta(t, tc.want[0], j.M.U, 1e-12)
				require.InDelta(t, tc.want[1], j.M.V, 1e-12)
			})
		}
	}
}

func TestOpeningJoinRefusals(t *testing.T) {
	k := line([2]float64{0, 0}, [2]float64{10, 0})
	for _, tc := range []struct {
		name string
		r    survey2d.SideWalk
		t    float64
		want error
	}{
		{"smooth", line([2]float64{10, 0}, [2]float64{20, 0}), 1, offset2d.ErrOpeningCorner},
		{"cusp", line([2]float64{10, 0}, [2]float64{5, 0}), 1, offset2d.ErrOpeningCorner},
		// A circle of radius 0.5 through the corner rises 0.9 at most, short
		// of the 2 mm offset.
		{"no crossing", arc([2]float64{10.3, 0.4}, 0.5, math.Atan2(-0.4, -0.3), math.Atan2(-0.4, -0.3)+math.Pi), 2, offset2d.ErrOpeningCorner},
		// The right-angle rim of length 2 runs past a removed walk of length 1.
		{"past the span", line([2]float64{10, 0}, [2]float64{10, 1}), 2, offset2d.ErrOpeningSpan},
		{"past an arc's span", arc([2]float64{11, 1}, math.Sqrt2, -3*math.Pi/4, -3*math.Pi/4-0.5), 1.5, offset2d.ErrOpeningSpan},
	} {
		t.Run(tc.name, func(t *testing.T) {
			_, err := offset2d.OpeningJoin(k, tc.r, true, 1, tc.t, openingTol)
			require.ErrorIs(t, err, tc.want)
		})
	}
}

// TestRimSegment writes the rim of the bead's chord end both ways round: a
// counterclockwise arc about the removed walk's centre from the corner to the
// cut inward, and clockwise from the cut back to the corner, which the outward
// wall walks.
func TestRimSegment(t *testing.T) {
	k := line([2]float64{0, 6}, [2]float64{10, 6})
	r := arc([2]float64{5, 6}, 5, 0, math.Pi)
	j, err := offset2d.OpeningJoin(k, r, true, 1, 1, openingTol)
	require.NoError(t, err)
	seg, err := offset2d.RimSegment(k, r, true, 1, j, true)
	require.NoError(t, err)
	a, ok := seg.(sectionrecord.ArcSeg)
	require.True(t, ok)
	require.Equal(t, sectionrecord.Point2{U: 5, V: 6}, a.Center)
	// Counterclockwise from the corner (10, 6) to the cut.
	require.Equal(t, sectionrecord.Point2{U: 10, V: 6}, a.Start)
	require.Equal(t, sectionrecord.Point2{U: j.M.U, V: j.M.V}, a.End)
	require.Equal(t, [2]float64{0, 1}, [2]float64{a.TStart, a.TEnd})

	back, err := offset2d.RimSegment(k, r, true, 1, j, false)
	require.NoError(t, err)
	b, ok := back.(sectionrecord.ArcSeg)
	require.True(t, ok)
	require.Equal(t, a.Start, b.Start)
	require.Equal(t, a.End, b.End)
	require.Equal(t, [2]float64{1, 0}, [2]float64{b.TStart, b.TEnd}, `the same arc walked from the cut back to the corner`)

	straight, err := offset2d.RimSegment(line([2]float64{0, 0}, [2]float64{10, 0}), line([2]float64{10, 0}, [2]float64{10, 10}), true, 1,
		offset2d.Join{VertU: 10, M: offset2d.Point{U: 10, V: 1}}, false)
	require.NoError(t, err)
	require.Equal(t, sectionrecord.LineSeg{Start: sectionrecord.Point2{U: 10, V: 1}, End: sectionrecord.Point2{U: 10}, TStart: 0, TEnd: 1}, straight)
}
