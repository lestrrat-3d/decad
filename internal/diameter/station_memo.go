package diameter

import (
	"math"
	"sync"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
)

// stationMemoEntries bounds how many distinct loop sets one StationMemo keeps.
// Past it, SectionStations reads fresh without storing.
const stationMemoEntries = 64

// StationMemo keeps SectionStations' answers within one call, keyed by the
// loops' content. Only loops made entirely of line, circle and arc segments
// are kept, compared field by field on their float bits (math.Float64bits),
// so a hit is a call with identical input and SectionStations, a pure
// function of its loops for these kinds, gives the identical answer. Other
// kinds are read fresh. A nil memo reads fresh. It is safe for concurrent
// use, and it keeps at most stationMemoEntries entries. The budget's context
// is checked before a hit is returned and, after the lock is taken, before a
// reading is stored or a concurrent winner returned, so a cancelled call
// neither stores nor returns a reading.
//
// A memo belongs to one top-level call (the call's pairkernel.Cache owns it) and
// is dropped with it. The returned stations are shared between hits, so a
// caller must not modify them.
type StationMemo struct {
	mu      sync.Mutex
	entries []stationMemoEntry
}

type stationMemoEntry struct {
	loops    []sectionrecord.LoopRecord
	stations []Station
	ok       bool
}

// SectionStations is the package-level SectionStations with its answer kept
// for loops it has read before. The stations are shared; do not modify them.
func (m *StationMemo) SectionStations(budget *proofbound.WorkBudget, loops []sectionrecord.LoopRecord,
	work *freeform.FreeformWork) ([]Station, bool, error) {
	if m == nil || !memoizableLoops(loops) {
		return SectionStations(budget, loops, work)
	}
	m.mu.Lock()
	for _, e := range m.entries {
		if loopsEqual(e.loops, loops) {
			m.mu.Unlock()
			if err := budget.Err(); err != nil {
				return nil, false, err
			}
			return e.stations, e.ok, nil
		}
	}
	m.mu.Unlock()
	stations, ok, err := SectionStations(budget, loops, work)
	if err != nil {
		return nil, false, err
	}
	m.mu.Lock()
	defer m.mu.Unlock()
	// A reading is stored and returned only while the budget is live: a
	// short read polls the context too rarely to have noticed a cancel.
	if err := budget.Err(); err != nil {
		return nil, false, err
	}
	for _, e := range m.entries {
		if loopsEqual(e.loops, loops) {
			return e.stations, e.ok, nil
		}
	}
	if len(m.entries) < stationMemoEntries {
		// The key owns its own segment slices, so a later change to the
		// caller's loops cannot change what a hit compares against.
		key := make([]sectionrecord.LoopRecord, len(loops))
		for i, l := range loops {
			key[i].Segments = append([]sectionrecord.CurveSegment(nil), l.Segments...)
		}
		m.entries = append(m.entries, stationMemoEntry{loops: key, stations: stations, ok: ok})
	}
	return stations, ok, nil
}

// Len reports how many loop sets the memo holds. A nil memo holds none.
func (m *StationMemo) Len() int {
	if m == nil {
		return 0
	}
	m.mu.Lock()
	defer m.mu.Unlock()
	return len(m.entries)
}

func memoizableLoops(loops []sectionrecord.LoopRecord) bool {
	for _, l := range loops {
		for _, s := range l.Segments {
			switch s.(type) {
			case sectionrecord.LineSeg, sectionrecord.CircleSeg, sectionrecord.ArcSeg:
			default:
				return false
			}
		}
	}
	return true
}

func sameBits(a, b float64) bool { return math.Float64bits(a) == math.Float64bits(b) }

func loopsEqual(a, b []sectionrecord.LoopRecord) bool {
	if len(a) != len(b) {
		return false
	}
	for i := range a {
		if len(a[i].Segments) != len(b[i].Segments) {
			return false
		}
		for k := range a[i].Segments {
			if !segmentsEqual(a[i].Segments[k], b[i].Segments[k]) {
				return false
			}
		}
	}
	return true
}

func segmentsEqual(x, y sectionrecord.CurveSegment) bool {
	switch p := x.(type) {
	case sectionrecord.LineSeg:
		q, ok := y.(sectionrecord.LineSeg)
		return ok && pointsEqual(p.Start, q.Start) && pointsEqual(p.End, q.End) &&
			sameBits(p.TStart, q.TStart) && sameBits(p.TEnd, q.TEnd)
	case sectionrecord.CircleSeg:
		q, ok := y.(sectionrecord.CircleSeg)
		// A units.Value never holds -0 and never holds NaN, so == is exact.
		return ok && pointsEqual(p.Center, q.Center) && p.Radius == q.Radius && p.CCW == q.CCW &&
			sameBits(p.TStart, q.TStart) && sameBits(p.TEnd, q.TEnd)
	case sectionrecord.ArcSeg:
		q, ok := y.(sectionrecord.ArcSeg)
		return ok && pointsEqual(p.Center, q.Center) && pointsEqual(p.Start, q.Start) && pointsEqual(p.End, q.End) &&
			sameBits(p.TStart, q.TStart) && sameBits(p.TEnd, q.TEnd)
	}
	return false
}

func pointsEqual(a, b sectionrecord.Point2) bool {
	return sameBits(a.U, b.U) && sameBits(a.V, b.V)
}
