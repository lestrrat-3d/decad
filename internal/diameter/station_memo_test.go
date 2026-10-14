package diameter_test

import (
	"context"
	"math"
	"math/rand/v2"
	"sync"
	"testing"

	"github.com/lestrrat-3d/decad/internal/diameter"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

type loops = []sectionrecord.LoopRecord

func bitsEqual(t *testing.T, want, got []diameter.Station) {
	t.Helper()
	require.Len(t, got, len(want))
	for i := range want {
		require.Equal(t, math.Float64bits(want[i].U), math.Float64bits(got[i].U))
		require.Equal(t, math.Float64bits(want[i].V), math.Float64bits(got[i].V))
		require.Equal(t, want[i].Bound, got[i].Bound)
	}
}

func fresh(t *testing.T, l loops) ([]diameter.Station, bool) {
	t.Helper()
	s, ok, err := diameter.SectionStations(proofbound.NewWorkBudget(t.Context()), l, freeform.NewFreeformWork())
	require.NoError(t, err)
	return s, ok
}

func memoRead(t *testing.T, m *diameter.StationMemo, l loops) ([]diameter.Station, bool) {
	t.Helper()
	s, ok, err := m.SectionStations(proofbound.NewWorkBudget(t.Context()), l, freeform.NewFreeformWork())
	require.NoError(t, err)
	return s, ok
}

func circle(cx, cy, r float64) sectionrecord.CircleSeg {
	return sectionrecord.CircleSeg{
		Center: sectionrecord.Point2{U: cx, V: cy}, Radius: units.Millimeters(r), CCW: true, TStart: 0, TEnd: 2 * math.Pi,
	}
}

func rect(x0, y0, x1, y1 float64) sectionrecord.LoopRecord {
	c := [4]sectionrecord.Point2{{U: x0, V: y0}, {U: x1, V: y0}, {U: x1, V: y1}, {U: x0, V: y1}}
	var segs []sectionrecord.CurveSegment
	for i := range c {
		segs = append(segs, sectionrecord.LineSeg{Start: c[i], End: c[(i+1)%4], TStart: 0, TEnd: 1})
	}
	return sectionrecord.LoopRecord{Segments: segs}
}

func arcLoop(r, a float64) sectionrecord.LoopRecord {
	o := sectionrecord.Point2{}
	s := sectionrecord.Point2{U: r, V: 0}
	e := sectionrecord.Point2{U: r * math.Cos(a), V: r * math.Sin(a)}
	return sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{
		sectionrecord.ArcSeg{Center: o, Start: s, End: e, TStart: 0, TEnd: 1},
		sectionrecord.LineSeg{Start: e, End: s, TStart: 0, TEnd: 1},
	}}
}

// TestStationMemoMatchesSectionStations checks that a hit gives the stations
// and ok a fresh SectionStations gives, bit for bit, on line, circle, arc and
// mixed loops with holes, and on random inputs.
//
// Shown to fail: returning the entry of the first loop set for every key
// (comparing only the length) makes the random cases differ.
func TestStationMemoMatchesSectionStations(t *testing.T) {
	t.Parallel()
	cases := map[string]loops{
		"circle":   {{Segments: []sectionrecord.CurveSegment{circle(1, 2, 3)}}},
		"rect":     {rect(0, 0, 4, 2)},
		"arc":      {arcLoop(2, 4)},
		"holes":    {rect(0, 0, 10, 10), {Segments: []sectionrecord.CurveSegment{circle(5, 5, 1)}}, rect(1, 1, 2, 2)},
		"arc+hole": {arcLoop(3, 2), {Segments: []sectionrecord.CurveSegment{circle(1, 0, 0.25)}}},
	}
	rng := rand.New(rand.NewPCG(7, 11))
	for i := range 20 {
		cases[string(rune('A'+i))] = loops{
			rect(rng.Float64(), rng.Float64(), 3+rng.Float64(), 3+rng.Float64()),
			{Segments: []sectionrecord.CurveSegment{circle(rng.Float64(), rng.Float64(), 0.1+rng.Float64())}},
		}
	}
	m := &diameter.StationMemo{}
	for name, l := range cases {
		want, wantOK := fresh(t, l)
		for range 2 { // the second read is a hit
			got, ok := memoRead(t, m, l)
			require.Equal(t, wantOK, ok, name)
			bitsEqual(t, want, got)
		}
	}
	require.Equal(t, len(cases), m.Len())
}

// TestStationMemoKeysOnBits checks that inputs differing in one float bit,
// a signed zero, a winding flag or an added hole each make a new entry.
//
// Shown to fail: comparing floats with == merges the signed-zero case;
// dropping Center from the circle comparison merges the moved circle.
func TestStationMemoKeysOnBits(t *testing.T) {
	t.Parallel()
	base := circle(1, 2, 3)
	line := func(ts float64) loops {
		return loops{{Segments: []sectionrecord.CurveSegment{
			sectionrecord.LineSeg{Start: sectionrecord.Point2{}, End: sectionrecord.Point2{U: 1, V: 1}, TStart: ts, TEnd: 1},
		}}}
	}
	wrap := func(c sectionrecord.CircleSeg) loops { return loops{{Segments: []sectionrecord.CurveSegment{c}}} }
	moved := base
	moved.Center.U = math.Nextafter(1, 2)
	flipped := base
	flipped.CCW = false
	radius := circle(1, 2, math.Nextafter(3, 4))
	withHole := append(wrap(base), rect(0, 0, 1, 1))
	negZero := math.Copysign(0, -1)

	m := &diameter.StationMemo{}
	steps := []loops{wrap(base), wrap(moved), wrap(flipped), wrap(radius), withHole, line(0), line(negZero)}
	for i, l := range steps {
		got, ok := memoRead(t, m, l)
		want, wantOK := fresh(t, l)
		require.Equal(t, wantOK, ok)
		bitsEqual(t, want, got)
		require.Equal(t, i+1, m.Len(), "input %d must be a new entry", i)
	}
}

// TestStationMemoSkipsOtherKinds checks that a loop with a segment of another
// kind is read fresh and never enters the memo, and that a nil memo reads fresh.
//
// Shown to fail: adding an EllipseSeg case to memoizableLoops makes Len 1.
func TestStationMemoSkipsOtherKinds(t *testing.T) {
	t.Parallel()
	ellipse := loops{{Segments: []sectionrecord.CurveSegment{sectionrecord.EllipseSeg{
		Rx: units.Millimeters(2), Ry: units.Millimeters(1), Rotation: units.Radians(0), TStart: 0, TEnd: 2 * math.Pi,
	}}}}
	m := &diameter.StationMemo{}
	got, ok := memoRead(t, m, ellipse)
	want, wantOK := fresh(t, ellipse)
	require.Equal(t, wantOK, ok)
	bitsEqual(t, want, got)
	require.Zero(t, m.Len())

	var nilMemo *diameter.StationMemo
	l := loops{rect(0, 0, 1, 1)}
	got, ok = memoRead(t, nilMemo, l)
	want, wantOK = fresh(t, l)
	require.Equal(t, wantOK, ok)
	bitsEqual(t, want, got)
}

// TestStationMemoIsBoundedAndKeepsItsOwnKey checks the memo stops storing
// after 64 entries while still answering, and that changing the caller's
// segment slice after a read cannot change what a hit compares against.
//
// Shown to fail: removing the entry limit makes Len exceed 64; storing the
// caller's slice makes the mutated lookup return the stale stations.
func TestStationMemoIsBoundedAndKeepsItsOwnKey(t *testing.T) {
	t.Parallel()
	m := &diameter.StationMemo{}
	for i := range 80 {
		l := loops{rect(0, 0, 1+float64(i), 1)}
		got, ok := memoRead(t, m, l)
		want, wantOK := fresh(t, l)
		require.Equal(t, wantOK, ok)
		bitsEqual(t, want, got)
	}
	require.Equal(t, 64, m.Len())

	k := &diameter.StationMemo{}
	l := loops{rect(0, 0, 2, 2)}
	memoRead(t, k, l)
	l[0].Segments[0] = sectionrecord.LineSeg{End: sectionrecord.Point2{U: 9, V: 9}, TEnd: 1}
	got, ok := memoRead(t, k, l)
	want, wantOK := fresh(t, l)
	require.Equal(t, wantOK, ok)
	bitsEqual(t, want, got)
}

// TestStationMemoConcurrent checks that eight goroutines reading four
// profiles through one memo get the fresh answers (run under -race).
func TestStationMemoConcurrent(t *testing.T) {
	t.Parallel()
	profiles := []loops{
		{rect(0, 0, 4, 2)}, {{Segments: []sectionrecord.CurveSegment{circle(0, 0, 2)}}},
		{arcLoop(2, 3)}, {rect(0, 0, 3, 3), {Segments: []sectionrecord.CurveSegment{circle(1, 1, 0.5)}}},
	}
	wants := make([][]diameter.Station, len(profiles))
	for i, p := range profiles {
		wants[i], _ = fresh(t, p)
	}
	m := &diameter.StationMemo{}
	var wg sync.WaitGroup
	for g := range 8 {
		wg.Go(func() {
			for r := range 8 {
				i := (g + r) % len(profiles)
				got, _, err := m.SectionStations(proofbound.NewWorkBudget(t.Context()), profiles[i], freeform.NewFreeformWork())
				if err != nil || len(got) != len(wants[i]) {
					t.Errorf("profile %d: err %v, %d stations, want %d", i, err, len(got), len(wants[i]))
				}
			}
		})
	}
	wg.Wait()
	require.Equal(t, len(profiles), m.Len())
}

// TestStationMemoNeverStoresOrReturnsUnderCancel checks the memo with real
// cancelled contexts. NewWorkBudget polls the context once per 256 steps, so a
// four-line rectangle reads to the end under a cancelled context; the memo
// itself must still refuse to store the reading (cold), refuse to return a
// stored one (warm), and refuse a reading that finished after a cancel that
// came during the computation.
//
// Shown to fail: dropping the budget check before a hit makes the warm case
// return stations; dropping the check under the lock makes the cold case
// store the rectangle and the during-computation case store it too.
func TestStationMemoNeverStoresOrReturnsUnderCancel(t *testing.T) {
	t.Parallel()
	l := loops{rect(0, 0, 4, 2)}
	read := func(m *diameter.StationMemo, b *proofbound.WorkBudget) error {
		_, _, err := m.SectionStations(b, l, freeform.NewFreeformWork())
		return err
	}
	t.Run("cold", func(t *testing.T) {
		t.Parallel()
		ctx, cancel := context.WithCancel(t.Context())
		cancel()
		m := &diameter.StationMemo{}
		require.ErrorIs(t, read(m, proofbound.NewWorkBudget(ctx)), context.Canceled)
		require.Zero(t, m.Len())
	})
	t.Run("warm", func(t *testing.T) {
		t.Parallel()
		m := &diameter.StationMemo{}
		require.NoError(t, read(m, proofbound.NewWorkBudget(t.Context())))
		require.Equal(t, 1, m.Len())
		ctx, cancel := context.WithCancel(t.Context())
		cancel()
		require.ErrorIs(t, read(m, proofbound.NewWorkBudget(ctx)), context.Canceled)
		require.NoError(t, read(m, proofbound.NewWorkBudget(t.Context())), "a live call still hits")
	})
	t.Run("during computation", func(t *testing.T) {
		t.Parallel()
		ctx, cancel := context.WithCancel(t.Context())
		steps := 0
		b := &proofbound.WorkBudget{
			StepFn: func() error {
				if steps++; steps == 4 { // the last of the rectangle's steps
					cancel()
				}
				return nil
			},
			ErrFn: ctx.Err,
		}
		m := &diameter.StationMemo{}
		require.ErrorIs(t, read(m, b), context.Canceled)
		require.Equal(t, 4, steps, "the reading ran to the end before the cancel was seen")
		require.Zero(t, m.Len())
	})
}
