package sweepinput_test

import (
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/sweepinput"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
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

func xyPlane() sectionrecord.PlaneRecord {
	return sectionrecord.PlaneRecord{U: r3.NewVec(1, 0, 0), V: r3.NewVec(0, 1, 0)}
}

func circleProfile(radius float64) momentinput.Profile {
	return momentinput.Profile{Outer: sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{
		sectionrecord.CircleSeg{Radius: units.Millimeters(radius), CCW: true, TStart: 0, TEnd: 1},
	}}}
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

func TestClosedRacetrack(t *testing.T) {
	plane := xyPlane()
	t.Run("admitted", func(t *testing.T) {
		require.True(t, sweepinput.ClosedRacetrack(circleProfile(1), plane, r3.Vec{}, racetrack(t)))
	})
	t.Run("radius too large for the height", func(t *testing.T) {
		require.False(t, sweepinput.ClosedRacetrack(circleProfile(2), plane, r3.Vec{}, racetrack(t)))
	})
	t.Run("offset start", func(t *testing.T) {
		require.False(t, sweepinput.ClosedRacetrack(circleProfile(1), plane, r3.NewVec(1, 0, 0), racetrack(t)))
	})
	t.Run("quarter turn instead of half", func(t *testing.T) {
		records := racetrack(t)
		records[1].ArcAngle.Turn = big.NewRat(1, 4)
		require.False(t, sweepinput.ClosedRacetrack(circleProfile(1), plane, r3.Vec{}, records))
	})
	t.Run("wrong record count", func(t *testing.T) {
		require.False(t, sweepinput.ClosedRacetrack(circleProfile(1), plane, r3.Vec{}, racetrack(t)[:3]))
	})
}

func TestClosedPlanarTangentLoop(t *testing.T) {
	plane := xyPlane()
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
	t.Run("admitted", func(t *testing.T) {
		require.True(t, sweepinput.ClosedPlanarTangentLoop(circleProfile(1), plane, r3.Vec{}, loop(t)))
	})
	t.Run("turns sum to three quarters", func(t *testing.T) {
		records := loop(t)
		records[2].ArcAngle.Turn = big.NewRat(1, 4)
		require.False(t, sweepinput.ClosedPlanarTangentLoop(circleProfile(1), plane, r3.Vec{}, records))
	})
	t.Run("record off the Y=0 plane", func(t *testing.T) {
		records := loop(t)
		records[0].End.Y = 1
		require.False(t, sweepinput.ClosedPlanarTangentLoop(circleProfile(1), plane, r3.Vec{}, records))
	})
	t.Run("too few records", func(t *testing.T) {
		require.False(t, sweepinput.ClosedPlanarTangentLoop(circleProfile(1), plane, r3.Vec{}, racetrack(t)))
	})
	t.Run("profile with a hole", func(t *testing.T) {
		profile := circleProfile(1)
		profile.Holes = []sectionrecord.LoopRecord{profile.Outer}
		require.False(t, sweepinput.ClosedPlanarTangentLoop(profile, plane, r3.Vec{}, loop(t)))
	})
}
