// Package sweepmemo stores certified sweep path readings.
package sweepmemo

import (
	"math"
	"math/big"
	"slices"

	"github.com/lestrrat-3d/decad/internal/motionbound"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// PathCap bounds each table of one path's sweep memo. A full table is
// emptied and refilled; a run reads a few hundred fractions at most, so this
// keeps every repeat of a typical run.
const PathCap = 1024

// PathMemo holds one prepared path's repeated readings during a sweep
// run: its ideal pose by fraction (planarsweep.BodyPath.IdealAt), its staged
// points' deviation by pose and fraction (PointDeviation), and its point
// spans by fraction span (CornerSpan). The sweep refines, re-samples and
// re-reads the same fractions and poses many times over; each reading is a
// pure function of its key and of path fields no copy changes once the run
// attaches the memo, so the memo changes no value. A copy may swap its point
// sets (the rolling column test reads other corners through the same path),
// so the point-set keys carry the set's identity.
//
// The run attaches one memo per path (planarsweep.AttachMemos) and closes it
// before it returns (Close). A closed memo holds nothing and serves nothing,
// so the copies a proof keeps compute afresh, keep no memory alive, and
// compare equal to a run without memos, whose paths carry an already closed
// memo. A memo is read and written by its run's goroutine only, and only read
// once closed. Every reading it serves is a copy the caller owns.
type PathMemo struct {
	Closed    bool
	Ideal     map[string]IdealPose
	Deviation map[DeviationKey]Deviation
	Spans     map[SpanKey]CornerSpans
	Stats     Stats
}

// Stats counts an open memo's hits and misses, for tests.
type Stats struct {
	IdealHits, IdealMisses         int
	DeviationHits, DeviationMisses int
	SpanHits, SpanMisses           int
}

// NewPathMemo returns an open memo, or a closed one while the memos are
// switched off (motionbound.SetMemos).
func NewPathMemo() *PathMemo {
	return &PathMemo{Closed: !motionbound.MemosOn()}
}

func (m *PathMemo) Open() bool { return m != nil && !m.Closed }

// Close drops every entry; the memo serves nothing afterwards.
func (m *PathMemo) Close() { *m = PathMemo{Closed: true} }

// PointSetID names a point set by its backing array and length.
type PointSetID struct {
	first *proofarith.DyV3
	count int
}

func PointSetOf(points []proofarith.DyV3) PointSetID {
	if len(points) == 0 {
		return PointSetID{}
	}
	return PointSetID{first: &points[0], count: len(points)}
}

// DeviationKey names one PointDeviation reading: the source points, the
// pose's bits (a −0 apart from a +0) and the exact fraction.
type DeviationKey struct {
	points PointSetID
	pose   [12]uint64
	f      string
}

// Deviation is one PointDeviation result. Only a reading whose ideal
// pose exists and whose poll never failed is stored.
type Deviation struct {
	Points []proofarith.DyV3
	Bound  float64
	OK     bool
}

// SpanKey names one CornerSpan reading: the start points and the exact
// fraction span.
type SpanKey struct {
	points   PointSetID
	from, to string
}

// PoseBits reads a pose's basis and translation as bits, so −0 and +0 differ.
func PoseBits(pose r3.Transform) [12]uint64 {
	var bits [12]uint64
	basis := pose.Basis()
	for i, v := range [4]r3.Vec{basis.EX, basis.EY, basis.EZ, pose.Translation()} {
		bits[3*i] = math.Float64bits(v.X)
		bits[3*i+1] = math.Float64bits(v.Y)
		bits[3*i+2] = math.Float64bits(v.Z)
	}
	return bits
}

func RatKey(r *big.Rat) string {
	return string(motionbound.AppendRatKey(nil, r))
}

func SpanKeyOf(points []proofarith.DyV3, from, to *big.Rat) SpanKey {
	return SpanKey{points: PointSetOf(points), from: RatKey(from), to: RatKey(to)}
}

func (m *PathMemo) LoadIdeal(f string) (IdealPose, bool) {
	ideal, ok := m.Ideal[f]
	if !ok {
		m.Stats.IdealMisses++
		return IdealPose{}, false
	}
	m.Stats.IdealHits++
	return ideal.clone(), true
}

func (m *PathMemo) StoreIdeal(f string, ideal IdealPose) {
	if m.Ideal == nil || len(m.Ideal) >= PathCap {
		m.Ideal = make(map[string]IdealPose)
	}
	m.Ideal[f] = ideal.clone()
}

// DeviationKeyOf keys a path's point deviation by source set, pose bits and fraction.
func DeviationKeyOf(points []proofarith.DyV3, pose r3.Transform, f *big.Rat) DeviationKey {
	return DeviationKey{points: PointSetOf(points), pose: PoseBits(pose), f: RatKey(f)}
}

func (m *PathMemo) LoadDeviation(key DeviationKey) (Deviation, bool) {
	entry, ok := m.Deviation[key]
	if !ok {
		m.Stats.DeviationMisses++
		return Deviation{}, false
	}
	m.Stats.DeviationHits++
	entry.Points = slices.Clone(entry.Points)
	return entry, true
}

func (m *PathMemo) StoreDeviation(key DeviationKey, entry Deviation) {
	if m.Deviation == nil || len(m.Deviation) >= PathCap {
		m.Deviation = make(map[DeviationKey]Deviation)
	}
	entry.Points = slices.Clone(entry.Points)
	m.Deviation[key] = entry
}

func (m *PathMemo) LoadSpans(key SpanKey) (CornerSpans, bool) {
	spans, ok := m.Spans[key]
	if !ok {
		m.Stats.SpanMisses++
		return CornerSpans{}, false
	}
	m.Stats.SpanHits++
	return spans.clone(), true
}

func (m *PathMemo) StoreSpans(key SpanKey, spans CornerSpans) {
	if m.Spans == nil || len(m.Spans) >= PathCap {
		m.Spans = make(map[SpanKey]CornerSpans)
	}
	m.Spans[key] = spans.clone()
}

// IdealPose encloses one sweep path's ideal rotation and shift.
type IdealPose struct {
	Rot   motionbound.ScaledIvMat
	Shift motionbound.IvVec
}

// CornerSpans encloses every source point's ideal path over one interval.
type CornerSpans struct {
	Den [3]*big.Int
	Lo  [][3]*big.Int
	Hi  [][3]*big.Int
}

func (c CornerSpans) Len() int { return len(c.Lo) }

func (c CornerSpans) Hull(axis int) (*big.Rat, *big.Rat) {
	low, high := c.Lo[0][axis], c.Hi[0][axis]
	for index := 1; index < len(c.Lo); index++ {
		if c.Lo[index][axis].Cmp(low) < 0 {
			low = c.Lo[index][axis]
		}
		if c.Hi[index][axis].Cmp(high) > 0 {
			high = c.Hi[index][axis]
		}
	}
	return new(big.Rat).SetFrac(low, c.Den[axis]), new(big.Rat).SetFrac(high, c.Den[axis])
}

// HullGap is the widest strict gap between a's and b's coordinate hulls on
// any axis, less displacement, or nil when either span set is empty or no
// such gap stays positive. Each body lies in the hull of its source points,
// so a non-nil gap is a lower bound on the pair's separation over the span.
func HullGap(a, b CornerSpans, displacement *big.Rat) *big.Rat {
	if a.Len() == 0 || b.Len() == 0 {
		return nil
	}
	var widest *big.Rat
	for axis := range 3 {
		aLow, aHigh := a.Hull(axis)
		bLow, bHigh := b.Hull(axis)
		for _, gap := range []*big.Rat{new(big.Rat).Sub(bLow, aHigh), new(big.Rat).Sub(aLow, bHigh)} {
			gap.Sub(gap, displacement)
			if gap.Sign() > 0 && (widest == nil || gap.Cmp(widest) > 0) {
				widest = gap
			}
		}
	}
	return widest
}

func (c CornerSpans) Span(index, axis int) proofbound.RatInterval {
	return proofbound.IntervalOwned(new(big.Rat).SetFrac(c.Lo[index][axis], c.Den[axis]),
		new(big.Rat).SetFrac(c.Hi[index][axis], c.Den[axis]))
}

func (p IdealPose) clone() IdealPose {
	out := IdealPose{Rot: motionbound.ScaledIvMat{Den: new(big.Int).Set(p.Rot.Den)}}
	for i := range 3 {
		for j := range 3 {
			out.Rot.Lo[i][j] = new(big.Int).Set(p.Rot.Lo[i][j])
			out.Rot.Hi[i][j] = new(big.Int).Set(p.Rot.Hi[i][j])
		}
		out.Shift[i] = motionbound.CloneInterval(p.Shift[i])
	}
	return out
}

func (c CornerSpans) clone() CornerSpans {
	out := CornerSpans{Lo: make([][3]*big.Int, len(c.Lo)), Hi: make([][3]*big.Int, len(c.Hi))}
	for axis := range 3 {
		if c.Den[axis] != nil {
			out.Den[axis] = new(big.Int).Set(c.Den[axis])
		}
	}
	for index := range c.Lo {
		for axis := range 3 {
			out.Lo[index][axis] = new(big.Int).Set(c.Lo[index][axis])
			out.Hi[index][axis] = new(big.Int).Set(c.Hi[index][axis])
		}
	}
	return out
}
