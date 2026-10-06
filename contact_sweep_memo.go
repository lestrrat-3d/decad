package decad

import (
	"math"
	"math/big"
	"slices"
	"sync"

	"github.com/lestrrat-3d/decad/internal/motionbound"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
)

// sweepMemoCap bounds each table of one path's sweep memo. A full table is
// emptied and refilled; a run reads a few hundred fractions at most, so this
// keeps every repeat of a typical run.
const sweepMemoCap = 1024

// sweepPathMemo holds one prepared path's repeated readings during a sweep
// run: its ideal pose by fraction (idealAt), its staged points' deviation by
// pose and fraction (pointDeviation), and its point spans by fraction span
// (cornerSpan). The sweep refines, re-samples and re-reads the same fractions
// and poses many times over; each reading is a pure function of its key and
// of path fields no copy changes once the run attaches the memo, so the memo
// changes no value. A copy may swap its point sets (the rolling column test
// reads other corners through the same path), so the point-set keys carry
// the set's identity.
//
// The run attaches one memo per path (attachSweepMemos) and closes it before
// it returns (close). A closed memo holds nothing and serves nothing, so the
// copies a proof keeps compute afresh, keep no memory alive, and compare equal
// to a run without memos, whose paths carry an already closed memo. A memo is
// read and written by its run's goroutine only, and only read once closed.
// Every reading it serves is a copy the caller owns.
type sweepPathMemo struct {
	closed    bool
	ideal     map[string]sweepIdealPose
	deviation map[sweepDeviationKey]sweepDeviation
	spans     map[sweepSpanKey]cornerSpans
	stats     sweepMemoStats
}

// sweepMemoStats counts an open memo's hits and misses, for tests.
type sweepMemoStats struct {
	idealHits, idealMisses         int
	deviationHits, deviationMisses int
	spanHits, spanMisses           int
}

// newSweepPathMemo returns an open memo, or a closed one while the memos are
// switched off (motionbound.SetMemos).
func newSweepPathMemo() *sweepPathMemo {
	return &sweepPathMemo{closed: !motionbound.MemosOn()}
}

func (m *sweepPathMemo) open() bool { return m != nil && !m.closed }

// close drops every entry; the memo serves nothing afterwards.
func (m *sweepPathMemo) close() { *m = sweepPathMemo{closed: true} }

// pointSetID names a point set by its backing array and length.
type pointSetID struct {
	first *proofarith.DyV3
	count int
}

func pointSetOf(points []proofarith.DyV3) pointSetID {
	if len(points) == 0 {
		return pointSetID{}
	}
	return pointSetID{first: &points[0], count: len(points)}
}

// sweepDeviationKey names one pointDeviation reading: the source points, the
// pose's bits (a −0 apart from a +0) and the exact fraction.
type sweepDeviationKey struct {
	points pointSetID
	pose   [12]uint64
	f      string
}

// sweepDeviation is one pointDeviation result. Only a reading whose ideal
// pose exists and whose poll never failed is stored.
type sweepDeviation struct {
	points []proofarith.DyV3
	bound  float64
	ok     bool
}

// sweepSpanKey names one cornerSpan reading: the start points and the exact
// fraction span.
type sweepSpanKey struct {
	points   pointSetID
	from, to string
}

// poseBits reads a pose's basis and translation as bits, so −0 and +0 differ.
func poseBits(pose r3.Transform) [12]uint64 {
	var bits [12]uint64
	basis := pose.Basis()
	for i, v := range [4]r3.Vec{basis.EX, basis.EY, basis.EZ, pose.Translation()} {
		bits[3*i] = math.Float64bits(v.X)
		bits[3*i+1] = math.Float64bits(v.Y)
		bits[3*i+2] = math.Float64bits(v.Z)
	}
	return bits
}

func ratKey(r *big.Rat) string {
	return string(motionbound.AppendRatKey(nil, r))
}

func spanKey(points []proofarith.DyV3, from, to *big.Rat) sweepSpanKey {
	return sweepSpanKey{points: pointSetOf(points), from: ratKey(from), to: ratKey(to)}
}

func (m *sweepPathMemo) loadIdeal(f string) (sweepIdealPose, bool) {
	ideal, ok := m.ideal[f]
	if !ok {
		m.stats.idealMisses++
		return sweepIdealPose{}, false
	}
	m.stats.idealHits++
	return ideal.clone(), true
}

func (m *sweepPathMemo) storeIdeal(f string, ideal sweepIdealPose) {
	if m.ideal == nil || len(m.ideal) >= sweepMemoCap {
		m.ideal = make(map[string]sweepIdealPose)
	}
	m.ideal[f] = ideal.clone()
}

func (m *sweepPathMemo) loadDeviation(key sweepDeviationKey) (sweepDeviation, bool) {
	entry, ok := m.deviation[key]
	if !ok {
		m.stats.deviationMisses++
		return sweepDeviation{}, false
	}
	m.stats.deviationHits++
	entry.points = slices.Clone(entry.points)
	return entry, true
}

func (m *sweepPathMemo) storeDeviation(key sweepDeviationKey, entry sweepDeviation) {
	if m.deviation == nil || len(m.deviation) >= sweepMemoCap {
		m.deviation = make(map[sweepDeviationKey]sweepDeviation)
	}
	entry.points = slices.Clone(entry.points)
	m.deviation[key] = entry
}

func (m *sweepPathMemo) loadSpans(key sweepSpanKey) (cornerSpans, bool) {
	spans, ok := m.spans[key]
	if !ok {
		m.stats.spanMisses++
		return cornerSpans{}, false
	}
	m.stats.spanHits++
	return spans.clone(), true
}

func (m *sweepPathMemo) storeSpans(key sweepSpanKey, spans cornerSpans) {
	if m.spans == nil || len(m.spans) >= sweepMemoCap {
		m.spans = make(map[sweepSpanKey]cornerSpans)
	}
	m.spans[key] = spans.clone()
}

// attachSweepMemos gives both of a run's paths their own memo; closeSweepMemos
// closes them. Each sweep entry point attaches before its run and closes,
// deferred, before it returns.
func attachSweepMemos(paths ...*rotationalSweepPath) {
	for _, path := range paths {
		path.memo = newSweepPathMemo()
	}
}

func closeSweepMemos(paths ...*rotationalSweepPath) {
	for _, path := range paths {
		if path.memo != nil {
			path.memo.close()
		}
	}
}

func (p sweepIdealPose) clone() sweepIdealPose {
	out := sweepIdealPose{rot: motionbound.ScaledIvMat{Den: new(big.Int).Set(p.rot.Den)}}
	for i := range 3 {
		for j := range 3 {
			out.rot.Lo[i][j] = new(big.Int).Set(p.rot.Lo[i][j])
			out.rot.Hi[i][j] = new(big.Int).Set(p.rot.Hi[i][j])
		}
		out.shift[i] = motionbound.CloneInterval(p.shift[i])
	}
	return out
}

func (c cornerSpans) clone() cornerSpans {
	out := cornerSpans{lo: make([][3]*big.Int, len(c.lo)), hi: make([][3]*big.Int, len(c.hi))}
	for axis := range 3 {
		if c.den[axis] != nil {
			out.den[axis] = new(big.Int).Set(c.den[axis])
		}
	}
	for index := range c.lo {
		for axis := range 3 {
			out.lo[index][axis] = new(big.Int).Set(c.lo[index][axis])
			out.hi[index][axis] = new(big.Int).Set(c.hi[index][axis])
		}
	}
	return out
}

// sweepRadiusMemoCap bounds how many radii one body's memo keeps. A step
// reads a path's radius for its swept box and again for its sweep, so a
// body's repeats lie within the few paths of one or two steps.
const sweepRadiusMemoCap = 256

// sweepRadiusKey names one rotationalSweepRadius reading on its body: the
// From pose's and the pivot's bits and the exact dyadic axis.
type sweepRadiusKey struct {
	from   [12]uint64
	center [3]uint64
	axis   string
}

func newSweepRadiusKey(from r3.Transform, center r3.Vec, axis proofarith.DyV3) sweepRadiusKey {
	var b []byte
	for _, component := range axis {
		b = component.AppendKey(b)
	}
	return sweepRadiusKey{from: poseBits(from),
		center: [3]uint64{math.Float64bits(center.X), math.Float64bits(center.Y), math.Float64bits(center.Z)},
		axis:   string(b)}
}

type sweepRadius struct {
	radius float64 // the float rotationalSweepRadius returns exactly
	ok     bool
}

// sweepRadiusMemo holds a body's recent rotationalSweepRadius readings. A body
// is immutable, so a reading depends on nothing but its key; the memo changes
// no value, and each caller gets a fresh rational. When full, the oldest
// entry leaves first.
type sweepRadiusMemo struct {
	mu      sync.Mutex
	entries map[sweepRadiusKey]sweepRadius
	order   []sweepRadiusKey // insertion order; a ring once it holds sweepRadiusMemoCap keys
	next    int
	hits    int
	misses  int
}

// load returns the radius stored under key; hit is false when none is.
func (m *sweepRadiusMemo) load(key sweepRadiusKey) (float64, bool, bool) {
	m.mu.Lock()
	defer m.mu.Unlock()
	entry, hit := m.entries[key]
	if !hit {
		m.misses++
		return 0, false, false
	}
	m.hits++
	return entry.radius, entry.ok, true
}

// store keeps a reading under key, evicting the oldest entry when full. A key
// already present keeps its reading: two concurrent misses computed the same
// one.
func (m *sweepRadiusMemo) store(key sweepRadiusKey, radius float64, ok bool) {
	m.mu.Lock()
	defer m.mu.Unlock()
	if _, hit := m.entries[key]; hit {
		return
	}
	if m.entries == nil {
		m.entries = make(map[sweepRadiusKey]sweepRadius)
	}
	if len(m.order) < sweepRadiusMemoCap {
		m.order = append(m.order, key)
	} else {
		delete(m.entries, m.order[m.next])
		m.order[m.next] = key
		m.next = (m.next + 1) % sweepRadiusMemoCap
	}
	m.entries[key] = sweepRadius{radius: radius, ok: ok}
}
