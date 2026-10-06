package decad

import (
	"math"
	"slices"
	"sync"

	"github.com/lestrrat-3d/r3"
)

// pairReportMemoCap bounds how many reports one body's memo keeps. The dynamics
// scenes repeat a query within a few hundred later queries on the same first
// body (a resting pair re-read every step, a sweep sample re-read by the next
// slice), so this keeps nearly every repeat while holding a few megabytes at
// most per body.
const pairReportMemoCap = 1024

// pairReportKey names one ContactPair query on its first body: the second
// body, the bits of both poses, and the request. Pose bits keep a −0 apart
// from a +0. A validated request holds no NaN and no −0 (units canonicalizes
// zeros), so == on it is equality of its bits.
type pairReportKey struct {
	b     *Body
	poses [24]uint64
	req   ContactRequest
}

func newPairReportKey(b *Body, poseA, poseB r3.Transform, req ContactRequest) pairReportKey {
	key := pairReportKey{b: b, req: req}
	i := 0
	for _, pose := range [2]r3.Transform{poseA, poseB} {
		basis := pose.Basis()
		for _, v := range [4]r3.Vec{basis.EX, basis.EY, basis.EZ, pose.Translation()} {
			key.poses[i] = math.Float64bits(v.X)
			key.poses[i+1] = math.Float64bits(v.Y)
			key.poses[i+2] = math.Float64bits(v.Z)
			i += 3
		}
	}
	return key
}

// pairReportMemo holds a body's recent complete ContactPair reports as the first
// operand. A body is immutable, so a report depends on nothing but its key;
// like the planar snapshot beside it the memo changes no outcome. Only a
// report the query completed enters it: an error, a cancellation among them,
// is never stored. The stored reports are never handed out; each caller gets
// its own copy (cloneContactReport). When full, the oldest entry leaves first.
type pairReportMemo struct {
	mu      sync.Mutex
	entries map[pairReportKey]*ContactReport
	order   []pairReportKey // insertion order; a ring once it holds pairReportMemoCap keys
	next    int
}

// load returns a copy of the report stored under key.
func (m *pairReportMemo) load(key pairReportKey) (*ContactReport, bool) {
	m.mu.Lock()
	report, ok := m.entries[key]
	m.mu.Unlock()
	if !ok {
		return nil, false
	}
	return cloneContactReport(report), true
}

// store keeps report under key, evicting the oldest entry when full. A key
// already present keeps its report: two concurrent misses computed the same
// one.
func (m *pairReportMemo) store(key pairReportKey, report *ContactReport) {
	m.mu.Lock()
	defer m.mu.Unlock()
	if _, ok := m.entries[key]; ok {
		return
	}
	if m.entries == nil {
		m.entries = make(map[pairReportKey]*ContactReport)
	}
	if len(m.order) < pairReportMemoCap {
		m.order = append(m.order, key)
	} else {
		delete(m.entries, m.order[m.next])
		m.order[m.next] = key
		m.next = (m.next + 1) % pairReportMemoCap
	}
	m.entries[key] = report
}

// cloneContactReport copies a report down to its manifold's point slice, so
// no two callers share a value they could write. Bodies and faces stay
// shared: they are immutable. A nil slice stays nil.
func cloneContactReport(r *ContactReport) *ContactReport {
	out := *r
	if r.Gap != nil {
		gap := *r.Gap
		out.Gap = &gap
	}
	if r.Overlap != nil {
		overlap := *r.Overlap
		out.Overlap = &overlap
	}
	if r.Manifold != nil {
		out.Manifold = &ContactManifold{Points: slices.Clone(r.Manifold.Points)}
	}
	return &out
}
