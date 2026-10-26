package sweepinput_test

import (
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/sweeparc"
	"github.com/lestrrat-3d/decad/internal/sweepinput"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func record(t *testing.T, start r3.Vec, seg sweepinput.Segment) sweepinput.PathRecord {
	t.Helper()
	rec, err := sweepinput.RecordSegment(start, seg, 0)
	require.NoError(t, err)
	return rec
}

// chain records consecutive segments starting at start.
func chain(t *testing.T, start r3.Vec, segs ...sweepinput.Segment) []sweepinput.PathRecord {
	t.Helper()
	records := make([]sweepinput.PathRecord, len(segs))
	current := start
	for i, seg := range segs {
		records[i] = record(t, current, seg)
		current = records[i].End
	}
	return records
}

func line(end r3.Vec) sweepinput.Segment { return sweepinput.Segment{End: end} }

func arc(through, end r3.Vec) sweepinput.Segment {
	return sweepinput.Segment{Through: through, End: end, Arc: true}
}

// racetrack is a height-4 loop of two half-turn arcs of radius 3 about Y.
func racetrack(t *testing.T) []sweepinput.PathRecord {
	t.Helper()
	return chain(t, r3.Vec{},
		line(r3.NewVec(0, 0, 4)),
		arc(r3.NewVec(3, 0, 7), r3.NewVec(6, 0, 4)),
		line(r3.NewVec(6, 0, 0)),
		arc(r3.NewVec(3, 0, -3), r3.NewVec(0, 0, 0)),
	)
}

func TestClosedCircle(t *testing.T) {
	a, b := r3.NewVec(0, 0, 0), r3.NewVec(0, 0, 2)
	t.Run("two codirectional half arcs", func(t *testing.T) {
		records := chain(t, a, arc(r3.NewVec(1, 0, 1), b), arc(r3.NewVec(-1, 0, 1), a))
		carrier, ok := sweepinput.ClosedCircle(records)
		require.True(t, ok)
		require.Same(t, records[0].Arc, carrier)
		require.Zero(t, carrier.Center[0].Sign())
		require.Equal(t, 0, carrier.Center[2].Cmp(big.NewRat(1, 1)))
	})
	t.Run("opposite second arc", func(t *testing.T) {
		records := chain(t, a, arc(r3.NewVec(1, 0, 1), b), arc(r3.NewVec(1, 0, 1), a))
		_, ok := sweepinput.ClosedCircle(records)
		require.False(t, ok)
	})
	t.Run("seam endpoints differ", func(t *testing.T) {
		records := chain(t, a, arc(r3.NewVec(1, 0, 1), b), arc(r3.NewVec(-1, 0, 1), a))
		records[1].End = r3.NewVec(0, 0, 0.5)
		_, ok := sweepinput.ClosedCircle(records)
		require.False(t, ok)
	})
	t.Run("different circle centers", func(t *testing.T) {
		records := chain(t, a, arc(r3.NewVec(1, 0, 1), b), arc(r3.NewVec(-2, 0, 1), a))
		_, ok := sweepinput.ClosedCircle(records)
		require.False(t, ok)
	})
	t.Run("not two arcs", func(t *testing.T) {
		records := chain(t, a, arc(r3.NewVec(1, 0, 1), b), line(a))
		_, ok := sweepinput.ClosedCircle(records)
		require.False(t, ok)
		_, ok = sweepinput.ClosedCircle(records[:1])
		require.False(t, ok)
	})
}

func TestClosedCardinalLoop(t *testing.T) {
	loop := func(t *testing.T) []sweepinput.PathRecord {
		t.Helper()
		return chain(t, r3.Vec{},
			line(r3.NewVec(0, 0, 2)),
			line(r3.NewVec(0, 0, 4)),
			arc(r3.NewVec(3, 0, 7), r3.NewVec(6, 0, 4)),
			line(r3.NewVec(6, 0, 0)),
			arc(r3.NewVec(3, 0, -3), r3.NewVec(0, 0, 0)),
		)
	}
	t.Run("admitted racetrack", func(t *testing.T) {
		require.NoError(t, sweepinput.ClosedCardinalLoop(racetrack(t)))
	})
	t.Run("admitted loop with a split line", func(t *testing.T) {
		require.NoError(t, sweepinput.ClosedCardinalLoop(loop(t)))
	})
	refusals := []struct {
		name   string
		mutate func(*testing.T, []sweepinput.PathRecord) []sweepinput.PathRecord
		want   string
	}{
		{"too few spans", func(t *testing.T, r []sweepinput.PathRecord) []sweepinput.PathRecord { return r[:3] },
			"fewer than four"},
		{"quarter turn instead of half", func(t *testing.T, r []sweepinput.PathRecord) []sweepinput.PathRecord {
			r[1].ArcAngle.Turn = big.NewRat(1, 4)
			return r
		}, "not one whole turn"},
		{"arc turn is not cardinal", func(t *testing.T, r []sweepinput.PathRecord) []sweepinput.PathRecord {
			r[1].ArcAngle.Turn = big.NewRat(1, 3)
			return r
		}, "not an exact quarter or half turn"},
		{"arc rounded to a radian angle", func(t *testing.T, r []sweepinput.PathRecord) []sweepinput.PathRecord {
			r[1].ArcAngle.Rad = big.NewRat(1, 1)
			return r
		}, "not an exact quarter or half turn"},
		{"line leaves the plane", func(t *testing.T, r []sweepinput.PathRecord) []sweepinput.PathRecord {
			r[0].End.Y = 1
			return r
		}, "leaves the plane of the loop"},
		{"arc centre leaves the plane", func(t *testing.T, r []sweepinput.PathRecord) []sweepinput.PathRecord {
			r[1].Arc.Center[1] = big.NewRat(1, 1)
			return r
		}, "leaves the plane of the loop"},
		{"arc axis is not a coordinate axis", func(t *testing.T, r []sweepinput.PathRecord) []sweepinput.PathRecord {
			r[1].Arc.Axis[0] = big.NewRat(1, 1)
			return r
		}, "non-coordinate axis"},
		{"arcs turn about different axes", func(t *testing.T, r []sweepinput.PathRecord) []sweepinput.PathRecord {
			r[3].Arc.Axis = sweeparc.RatVec{new(big.Rat), new(big.Rat), big.NewRat(1, 1)}
			return r
		}, "different coordinate axis"},
		{"arcs with opposite senses", func(t *testing.T, r []sweepinput.PathRecord) []sweepinput.PathRecord {
			r[3].Arc.Axis[1] = big.NewRat(-1, 1)
			return r
		}, "not one whole turn"},
		{"only lines", func(t *testing.T, r []sweepinput.PathRecord) []sweepinput.PathRecord {
			for i := range r {
				r[i].Arc = nil
			}
			return r
		}, "fewer than two"},
	}
	for _, tc := range refusals {
		t.Run(tc.name, func(t *testing.T) {
			err := sweepinput.ClosedCardinalLoop(tc.mutate(t, racetrack(t)))
			require.ErrorIs(t, err, decaderr.ErrUnsupported)
			require.ErrorContains(t, err, tc.want)
		})
	}
}
