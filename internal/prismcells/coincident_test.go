package prismcells

import (
	"context"
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/sketch"
	"github.com/stretchr/testify/require"
)

// sharedArcScene arranges a disk of radius 20 (operand A, its circle CCW)
// beside a region B outside it whose boundary runs along the disk's own
// circle from 90° to 170° as an arc, then out and around through
// (−30, ·), (−30, 30) and (0, 30) back to the arc's start. sketch names the
// shared span under B's arc and withdraws it from A's circle. B's outline is
// built clockwise, so its authored counter-clockwise walk runs every B
// entity backwards. It returns the cells and which one is the disk's.
func sharedArcScene(t *testing.T) ([]*sketch.Profile, map[sketch.Entity]Origin, int, *sketch.Arc, *sketch.Circle) {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	fixed := func(x, y float64) *sketch.Point {
		p := s.CreatePoint(x, y)
		s.Fix(p)
		return p
	}
	a170 := 170 * math.Pi / 180
	centre := fixed(0, 0)
	top := fixed(0, 20)
	p170 := fixed(20*math.Cos(a170), 20*math.Sin(a170))
	r := fixed(-30, 20*math.Sin(a170))
	u := fixed(-30, 30)
	v := fixed(0, 30)
	tags := map[sketch.Entity]Origin{}
	circle := s.CreateCircle(centre, 20)
	tags[circle] = Origin{Hole: -1}
	arc := s.CreateArc(centre, top, p170)
	tags[arc] = Origin{IsB: true, Hole: -1, AuthoredReversed: true}
	for _, seg := range [][2]*sketch.Point{{p170, r}, {r, u}, {u, v}, {v, top}} {
		tags[s.CreateLine(seg[0], seg[1])] = Origin{IsB: true, Hole: -1, AuthoredReversed: true}
	}
	_, err = s.Solve(context.Background())
	require.NoError(t, err)
	profiles := s.Profiles()
	require.Len(t, profiles, 2)
	disk := -1
	for i, p := range profiles {
		require.True(t, p.Valid)
		for _, e := range p.Outer {
			if e.Entity == sketch.Entity(circle) {
				disk = i
			}
		}
	}
	require.NotEqual(t, -1, disk)
	return profiles, tags, disk, arc, circle
}

// TestCoincidentEdgesReadsASharedArc pins the reading's premise on the
// shared-arc scene: one span, named under B's arc and withdrawn from A's
// circle, whose one edge both cells walk, and a gap inside sketch's own
// identity band (1e-12 of the radius): the arc and the circle share one
// recorded centre, and the arc's endpoints sit on radius 20 to within their
// own cos/sin rounding, which the gap reads.
func TestCoincidentEdgesReadsASharedArc(t *testing.T) {
	t.Parallel()
	profiles, tags, _, arc, circle := sharedArcScene(t)
	reading, ok, err := CoincidentEdges(proofbound.NewWorkBudget(t.Context()), tags, profiles)
	require.NoError(t, err)
	require.True(t, ok)
	require.Len(t, reading.Spans, 1)
	require.Equal(t, sketch.Entity(arc), reading.Spans[0].Named)
	require.Equal(t, sketch.Entity(circle), reading.Spans[0].Losing)
	require.Len(t, reading.Edges, 1)
	require.True(t, reading.Partners(arc, circle))
	require.Less(t, reading.Gap(), 20e-12)
}

func TestCoincidentReadingRequiresExactFiniteLineContact(t *testing.T) {
	t.Parallel()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	line := func(x, y0, y1 float64) sketch.Entity {
		return s.CreateLine(s.CreatePoint(x, y0), s.CreatePoint(x, y1))
	}
	a := line(4, 0, 3)
	exact := line(4, 1, 2)
	near := line(4.0000000000005, 1, 2)
	for _, tc := range []struct {
		name string
		span CoincidentSpan
		want bool
	}{
		{name: "exact overlap", span: CoincidentSpan{Named: a, Losing: exact,
			A: [2]float64{4, 1}, B: [2]float64{4, 2}}, want: true},
		{name: "sub-band gap", span: CoincidentSpan{Named: a, Losing: near,
			A: [2]float64{4, 1}, B: [2]float64{4, 2}}},
		{name: "outside finite segment", span: CoincidentSpan{Named: a, Losing: exact,
			A: [2]float64{4, 1}, B: [2]float64{4, 2.5}}},
	} {
		t.Run(tc.name, func(t *testing.T) {
			reading := CoincidentReading{Spans: []CoincidentSpan{tc.span}}
			got, err := reading.ExactLineCarriers(proofbound.NewWorkBudget(t.Context()))
			require.NoError(t, err)
			require.Equal(t, tc.want, got)
		})
	}
}

// TestClassifyReadsASharedArc is the flood the coincident reading stops.
// The cell outside the disk has no edge of A's own: its one A boundary is
// the shared span, emitted under B's arc. Propagating A's membership across
// that B edge from the disk would put the outside cell inside A, and an
// Intersect would publish it; read from A's circle instead, it is A's void.
// ClassifySplit, which reads the target side alone, must agree.
//
// Shown to fail with Classify's and ClassifySplit's coincident reading
// deleted (the outside cell read inside A).
func TestClassifyReadsASharedArc(t *testing.T) {
	t.Parallel()
	profiles, tags, disk, _, _ := sharedArcScene(t)
	outside := 1 - disk
	matterA, matterB, resolved, err := Classify(proofbound.NewWorkBudget(t.Context()), tags, profiles)
	require.NoError(t, err)
	require.True(t, resolved)
	require.True(t, matterA[disk])
	require.False(t, matterB[disk])
	require.False(t, matterA[outside])
	require.True(t, matterB[outside])

	target, err := ClassifySplit(proofbound.NewWorkBudget(t.Context()), tags, profiles)
	require.NoError(t, err)
	require.True(t, target[disk])
	require.False(t, target[outside])
}

// TestCoincidentReadingRefusesSharedArcInsideIdentityBand reads the
// shared-arc scene through ExactCarriers. Sketch resolves B's arc onto A's
// circle, but the arc's 170° endpoint is a cos/sin rounding off radius 20, so
// the two entities hold no one exact carrier and the reading refuses.
//
// Shown to fail with exactCircleSpan's squared-radius comparison removed.
func TestCoincidentReadingRefusesSharedArcInsideIdentityBand(t *testing.T) {
	t.Parallel()
	profiles, tags, _, arc, circle := sharedArcScene(t)
	budget := proofbound.NewWorkBudget(t.Context())
	reading, ok, err := CoincidentEdges(budget, tags, profiles)
	require.NoError(t, err)
	require.True(t, ok)
	require.Len(t, reading.Spans, 1)
	require.Equal(t, sketch.Entity(arc), reading.Spans[0].Named)
	require.Equal(t, sketch.Entity(circle), reading.Spans[0].Losing)
	exact, err := reading.ExactCarriers(budget)
	require.NoError(t, err)
	require.False(t, exact)
}

// TestCoincidentReadingRequiresExactCircleCarrier is one row per refusal of
// exactCircleSpan, each beside the exact span it departs from. Every refusing
// row was shown to pass (read exact) with its own check removed: the centre
// comparison, the squared-radius comparison, spanEnd's agreement of two own
// endpoints, either arc test, and the whole-span circle check.
func TestCoincidentReadingRequiresExactCircleCarrier(t *testing.T) {
	t.Parallel()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	pt := func(x, y float64) *sketch.Point { return s.CreatePoint(x, y) }
	o := pt(0, 0)
	half := s.CreateArc(o, pt(10, 0), pt(-10, 0))
	quarter := s.CreateArc(o, pt(10, 0), pt(0, 10))
	chord := s.CreateArc(o, pt(6, 8), pt(-6, 8))
	shifted := s.CreateArc(pt(1, 0), pt(11, 0), pt(1, 10))
	wider := s.CreateArc(o, pt(10.5, 0), pt(0, 10.5))
	circle := s.CreateCircle(o, 10)
	twin := s.CreateCircle(pt(0, 0), 10)
	for _, tc := range []struct {
		name string
		span CoincidentSpan
		want bool
	}{
		{name: "exact quarter of a half", want: true, span: CoincidentSpan{Named: half, Losing: quarter,
			A: [2]float64{10, 0}, B: [2]float64{0, 10}, NamedT: [2]float64{0, 0.5}, LosingT: [2]float64{0, 1}}},
		{name: "exact cut ends", want: true, span: CoincidentSpan{Named: half, Losing: chord,
			A: [2]float64{6, 8}, B: [2]float64{-6, 8}, NamedT: [2]float64{0.3, 0.7}, LosingT: [2]float64{0, 1}}},
		{name: "exact arc on circle seam", want: true, span: CoincidentSpan{Named: quarter, Losing: circle,
			A: [2]float64{10, 1e-15}, B: [2]float64{0, 10}, NamedT: [2]float64{0, 1}, LosingT: [2]float64{0, 0.25}}},
		{name: "exact whole circles", want: true, span: CoincidentSpan{Named: twin, Losing: circle, Whole: true}},
		{name: "centre differs", span: CoincidentSpan{Named: half, Losing: shifted,
			A: [2]float64{11, 0}, B: [2]float64{1, 10}, NamedT: [2]float64{0.1, 0.5}, LosingT: [2]float64{0, 1}}},
		{name: "radius differs", span: CoincidentSpan{Named: half, Losing: wider,
			A: [2]float64{10.5, 0}, B: [2]float64{0, 10.5}, NamedT: [2]float64{0.1, 0.5}, LosingT: [2]float64{0, 1}}},
		{name: "own ends disagree", span: CoincidentSpan{Named: half, Losing: circle,
			A: [2]float64{10, 0}, B: [2]float64{0, 10}, NamedT: [2]float64{1, 0.5}, LosingT: [2]float64{0, 0.25}}},
		{name: "start outside arc", span: CoincidentSpan{Named: quarter, Losing: circle,
			A: [2]float64{6, -8}, B: [2]float64{0, 10}, NamedT: [2]float64{0.1, 1}, LosingT: [2]float64{0.9, 0.25}}},
		{name: "end outside arc", span: CoincidentSpan{Named: quarter, Losing: half,
			A: [2]float64{10, 0}, B: [2]float64{-6, 8}, NamedT: [2]float64{0, 0.7}, LosingT: [2]float64{0, 0.7}}},
		{name: "span runs backwards", span: CoincidentSpan{Named: half, Losing: circle,
			A: [2]float64{0, 10}, B: [2]float64{6, 8}, NamedT: [2]float64{0.5, 0.2}, LosingT: [2]float64{0.25, 0.4}}},
		{name: "whole span on an arc", span: CoincidentSpan{Named: half, Losing: circle, Whole: true}},
		{name: "zero span", span: CoincidentSpan{Named: half, Losing: circle,
			A: [2]float64{0, 10}, B: [2]float64{0, 10}, NamedT: [2]float64{0.5, 0.5}, LosingT: [2]float64{0.25, 0.25}}},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			reading := CoincidentReading{Spans: []CoincidentSpan{tc.span}}
			got, err := reading.ExactCarriers(proofbound.NewWorkBudget(t.Context()))
			require.NoError(t, err)
			require.Equal(t, tc.want, got)
			lineOnly, err := reading.ExactLineCarriers(proofbound.NewWorkBudget(t.Context()))
			require.NoError(t, err)
			require.False(t, lineOnly, "ExactLineCarriers refuses every circular span")
		})
	}
}

// TestOnCCWArc covers each sweep class of onCCWArc's exact sign test: under
// a half turn, over one, exactly one, and a zero sweep, each with a direction
// inside, one at an end, and one outside.
func TestOnCCWArc(t *testing.T) {
	t.Parallel()
	v := func(x, y int64) [2]*big.Rat { return [2]*big.Rat{big.NewRat(x, 1), big.NewRat(y, 1)} }
	for _, tc := range []struct {
		name    string
		p, s, e [2]*big.Rat
		want    bool
	}{
		{name: "quarter inside", p: v(6, 8), s: v(10, 0), e: v(0, 10), want: true},
		{name: "quarter at start", p: v(10, 0), s: v(10, 0), e: v(0, 10), want: true},
		{name: "quarter outside", p: v(-6, 8), s: v(10, 0), e: v(0, 10)},
		{name: "three quarters inside", p: v(-6, -8), s: v(10, 0), e: v(0, -10), want: true},
		{name: "three quarters at end", p: v(0, -10), s: v(10, 0), e: v(0, -10), want: true},
		{name: "three quarters outside", p: v(6, -8), s: v(10, 0), e: v(0, -10)},
		{name: "half inside", p: v(0, 10), s: v(10, 0), e: v(-10, 0), want: true},
		{name: "half outside", p: v(0, -10), s: v(10, 0), e: v(-10, 0)},
		{name: "zero sweep", p: v(10, 0), s: v(10, 0), e: v(10, 0)},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			require.Equal(t, tc.want, onCCWArc(tc.p, tc.s, tc.e))
		})
	}
}
