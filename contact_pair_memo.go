package decad

import (
	"math"
	"slices"
	"sync"

	"github.com/lestrrat-3d/r3"
)

// pairReportMemoCap bounds one body's completed contact reports.
const pairReportMemoCap = 1024

// pairReportKey preserves pose bits, including the sign of zero.
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

// pairReportMemo stores completed reports and returns independent copies.
type pairReportMemo struct {
	mu      sync.Mutex
	entries map[pairReportKey]*ContactReport
	order   []pairReportKey
	next    int
}

func (m *pairReportMemo) Load(key pairReportKey) (*ContactReport, bool) {
	m.mu.Lock()
	report, ok := m.entries[key]
	m.mu.Unlock()
	if !ok {
		return nil, false
	}
	return cloneContactReport(report), true
}

func (m *pairReportMemo) Store(key pairReportKey, report *ContactReport) {
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
