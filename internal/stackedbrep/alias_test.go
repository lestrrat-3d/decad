package stackedbrep

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/brepgeom"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/stretchr/testify/require"
)

func lineSeg(from, to Point2) CurveSegment {
	return LineSeg{Start: from, End: to, TStart: 0, TEnd: 1}
}

func ccwArc(c, start, end Point2) CurveSegment {
	return ArcSeg{Center: c, Start: start, End: end, TStart: 0, TEnd: 1}
}

func TestCarrierOfAlias(t *testing.T) {
	t.Parallel()
	o := Point2{}
	rounded := 7.0000000000000009
	exact := ccwArc(o, Point2{U: 7}, Point2{V: 7})
	off := ccwArc(o, Point2{U: rounded}, Point2{V: rounded})
	ca, err := CarrierOf(exact)
	require.NoError(t, err)
	cb, err := CarrierOf(off)
	require.NoError(t, err)
	require.NotEqual(t, ca, cb, "the two radii are two carriers")

	loop := func(b *Engine, r float64) Loop {
		t.Helper()
		start, end := Point2{U: r}, Point2{V: r}
		l, err := b.LoopOf(brepgeom.Profile{Outer: LoopRecord{Segments: []CurveSegment{
			ccwArc(o, start, end), lineSeg(end, start),
		}}}, proofbound.NewWorkBudget(t.Context()))
		require.NoError(t, err)
		return l
	}
	t.Run("without an alias the unit keeps its own carrier", func(t *testing.T) {
		t.Parallel()
		l := loop(NewEngine([]float64{0, 1}), rounded)
		require.Equal(t, cb, l.Units[0].Carrier)
	})
	t.Run("an alias reports the canonical carrier", func(t *testing.T) {
		t.Parallel()
		b := NewEngine([]float64{0, 1})
		b.SetAlias(map[Carrier]Carrier{cb: ca})
		require.Equal(t, ca, b.Canonical(cb))
		require.Equal(t, ca, b.Canonical(ca))
		l := loop(b, rounded)
		require.Equal(t, ca, l.Units[0].Carrier)
	})
	t.Run("SetAlias copies its argument", func(t *testing.T) {
		t.Parallel()
		b := NewEngine([]float64{0, 1})
		m := map[Carrier]Carrier{cb: ca}
		b.SetAlias(m)
		delete(m, cb)
		require.Equal(t, ca, b.Canonical(cb))
	})
}

// TestSpanEndJunction covers the circle-circle junction at a recorded span
// end. Circles A (centre 0, R 7) and B (centre c, R 2) touch at V, 7/9 of the
// way to c, where the shared span ends.
func TestSpanEndJunction(t *testing.T) {
	t.Parallel()
	cb := Point2{U: 2.8724500007495033, V: 8.529304250241882}
	carA := Carrier{Kind: ubCircle, R: 7}
	carB := Carrier{Kind: ubCircle, A: cb, R: 2}
	v := Point2{U: 7 * cb.U / 9, V: 7 * cb.V / 9}
	q := v
	for range 3 {
		q.U = math.Nextafter(q.U, math.Inf(1))
	}
	natural := func(p Point2) ubEnd { return ubEnd{p: p, allow: 1e-12} }
	cut := func(p Point2) ubEnd { return ubEnd{p: p, allow: 4e-12, cut: true} }
	engine := func(holed bool, spanEnd ...Carrier) *Engine {
		b := NewEngine([]float64{0, 1})
		if holed {
			b.SetHoledUnion(nil)
		}
		for _, c := range spanEnd {
			b.AddSpanEnd(c, v)
		}
		return b
	}

	t.Run("a cut end beside a recorded natural end joins at the natural point", func(t *testing.T) {
		t.Parallel()
		b := engine(true, carA)
		p, delta, err := b.junction(carA, cut(q), carB, natural(v))
		require.NoError(t, err)
		require.Equal(t, v, p)
		require.GreaterOrEqual(t, delta, proofbound.AbsSumUpper(4e-12, q.U-v.U))
		require.Equal(t, delta, b.Allow())
	})
	t.Run("the order of the two segments does not matter", func(t *testing.T) {
		t.Parallel()
		b := engine(true, carB)
		p, _, err := b.junction(carA, natural(v), carB, cut(q))
		require.NoError(t, err)
		require.Equal(t, v, p)
	})
	t.Run("an alias reads the span end on the canonical carrier", func(t *testing.T) {
		t.Parallel()
		other := Carrier{Kind: ubCircle, R: 7.0000000000000009}
		b := engine(true)
		b.SetAlias(map[Carrier]Carrier{other: carA})
		b.AddSpanEnd(other, v)
		_, _, err := b.junction(carA, cut(q), carB, natural(v))
		require.NoError(t, err)
	})
	for name, tc := range map[string]struct {
		b      *Engine
		ei, sj ubEnd
		ci, cj Carrier
	}{
		"no holed union":            {engine(false, carA), cut(q), natural(v), carA, carB},
		"no recorded span end":      {engine(true), cut(q), natural(v), carA, carB},
		"both ends cut":             {engine(true, carA, carB), cut(q), cut(v), carA, carB},
		"both ends natural":         {engine(true, carA, carB), natural(q), natural(v), carA, carB},
		"span end on the wrong one": {engine(true, carA), natural(v), cut(q), carA, carB},
		// The corner is recorded under another class (a circle of radius 9 about
		// the origin), so the cut carrier's class holds no span end there.
		"span end in another class": {engine(true, Carrier{Kind: ubCircle, R: 9}), cut(q), natural(v), carA, carB},
	} {
		t.Run("misses with "+name, func(t *testing.T) {
			t.Parallel()
			_, _, err := tc.b.junction(tc.ci, tc.ei, tc.cj, tc.sj)
			require.ErrorIs(t, err, brepgeom.ErrStackedWallMiss)
		})
	}
	t.Run("a non-finite separation misses", func(t *testing.T) {
		t.Parallel()
		b := engine(true, carA)
		_, ok := b.spanEndJunction(carA, cut(Point2{U: math.Inf(1)}), carB, natural(v))
		require.False(t, ok)
	})
}

func TestJunctionTableMax(t *testing.T) {
	t.Parallel()
	wall := Carrier{Kind: ubPlane, Axis: 0, Level: 3}
	round := Carrier{Kind: ubCircle, R: 5}
	crossing := Point2{U: 3, V: 4}
	read := func(b *Engine, allow float64) float64 {
		t.Helper()
		_, delta, err := b.junction(wall, ubEnd{p: crossing, allow: allow}, round, ubEnd{p: crossing, allow: allow})
		require.NoError(t, err)
		return delta
	}
	t.Run("a holed union keeps the larger allowance", func(t *testing.T) {
		t.Parallel()
		b := NewEngine([]float64{0, 1})
		b.SetHoledUnion(nil)
		require.Equal(t, 1e-12, read(b, 1e-12))
		require.Equal(t, 5e-12, read(b, 5e-12))
		require.Equal(t, 5e-12, b.Allow())
		require.Equal(t, 5e-12, read(b, 1e-12))
	})
	t.Run("a hole-free build keeps the first entry", func(t *testing.T) {
		t.Parallel()
		b := NewEngine([]float64{0, 1})
		require.Equal(t, 1e-12, read(b, 1e-12))
		require.Equal(t, 1e-12, read(b, 5e-12))
		require.Equal(t, 1e-12, b.Allow())
	})
}

func TestAuditedRegion(t *testing.T) {
	t.Parallel()
	arcSeg := ccwArc(Point2{}, Point2{U: 1}, Point2{V: 1})
	l1 := lineSeg(Point2{V: 1}, Point2{U: 2})
	l2 := lineSeg(Point2{U: 2}, Point2{U: 1})
	region := brepgeom.Profile{
		Outer: LoopRecord{Segments: []CurveSegment{l1, l2, arcSeg}},
		Holes: []LoopRecord{{Segments: []CurveSegment{arcSeg, l1, l2}}},
	}
	startsWithLine := func(p brepgeom.Profile) error {
		if _, ok := p.Holes[0].Segments[0].(LineSeg); ok {
			return nil
		}
		return brepgeom.ErrStackedWallMiss
	}
	t.Run("picks the first rotation the audit passes", func(t *testing.T) {
		t.Parallel()
		b := NewEngine([]float64{0, 1})
		b.SetHoledUnion(startsWithLine)
		got := b.auditedRegion(region)
		require.Equal(t, []CurveSegment{l1, l2, arcSeg}, got.Holes[0].Segments)
		require.Equal(t, []CurveSegment{l2, arcSeg, l1}, got.Outer.Segments)
		require.Equal(t, []CurveSegment{arcSeg, l1, l2}, region.Holes[0].Segments, "the input is untouched")
	})
	t.Run("a refusing audit leaves the region", func(t *testing.T) {
		t.Parallel()
		calls := 0
		b := NewEngine([]float64{0, 1})
		b.SetHoledUnion(func(brepgeom.Profile) error {
			calls++
			return brepgeom.ErrStackedWallMiss
		})
		require.Equal(t, region, b.auditedRegion(region))
		require.Equal(t, 3, calls, "one try per start of the longest ring")
	})
	t.Run("a build with no holed union tries nothing", func(t *testing.T) {
		t.Parallel()
		b := NewEngine([]float64{0, 1})
		b.audit = startsWithLine
		require.Equal(t, region, b.auditedRegion(region))
	})
	t.Run("rotateRing wraps the index", func(t *testing.T) {
		t.Parallel()
		loop := LoopRecord{Segments: []CurveSegment{l1, l2, arcSeg}}
		require.Equal(t, []CurveSegment{l2, arcSeg, l1}, RotateRing(loop, 4).Segments)
		require.Equal(t, LoopRecord{}, RotateRing(LoopRecord{}, 2))
	})
}
