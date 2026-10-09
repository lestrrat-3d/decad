package reportvocab

import (
	"math"
	"slices"
	"sync"

	"github.com/lestrrat-3d/r3"
)

// PairReportMemoCap bounds how many reports one body's memo keeps. Resting
// pairs and sweep samples can reread the same query after hundreds of others.
const PairReportMemoCap = 1024

// PairReportKey names one ContactPair query on its first body: the second
// body, the bits of both poses, and the request. Pose bits keep −0 apart
// from +0. A validated request holds no NaN or −0.
type PairReportKey[BodyT comparable] struct {
	b     BodyT
	poses [24]uint64
	req   ContactRequest
}

// NewPairReportKey reads the pose entries as bits, preserving signed zero.
func NewPairReportKey[BodyT comparable](b BodyT, poseA, poseB r3.Transform, req ContactRequest) PairReportKey[BodyT] {
	key := PairReportKey[BodyT]{b: b, req: req}
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

// PairReportMemo holds a body's recent complete ContactPair reports as the
// first operand. Only completed queries enter it. Each caller receives a copy.
type PairReportMemo[BodyT, FaceT, EdgeT, VertexT comparable, RelationT, ReasonT any] struct {
	mu      sync.Mutex
	entries map[PairReportKey[BodyT]]*ContactReport[BodyT, FaceT, EdgeT, VertexT, RelationT, ReasonT]
	order   []PairReportKey[BodyT]
	next    int
}

// Load returns an independent copy of a stored report.
func (m *PairReportMemo[BodyT, FaceT, EdgeT, VertexT, RelationT, ReasonT]) Load(
	key PairReportKey[BodyT],
) (*ContactReport[BodyT, FaceT, EdgeT, VertexT, RelationT, ReasonT], bool) {
	m.mu.Lock()
	report, ok := m.entries[key]
	m.mu.Unlock()
	if !ok {
		return nil, false
	}
	return CloneContactReport(report), true
}

// Store keeps the first report for a key and evicts the oldest entry when full.
func (m *PairReportMemo[BodyT, FaceT, EdgeT, VertexT, RelationT, ReasonT]) Store(
	key PairReportKey[BodyT], report *ContactReport[BodyT, FaceT, EdgeT, VertexT, RelationT, ReasonT],
) {
	m.mu.Lock()
	defer m.mu.Unlock()
	if _, ok := m.entries[key]; ok {
		return
	}
	if m.entries == nil {
		m.entries = make(map[PairReportKey[BodyT]]*ContactReport[BodyT, FaceT, EdgeT, VertexT, RelationT, ReasonT])
	}
	if len(m.order) < PairReportMemoCap {
		m.order = append(m.order, key)
	} else {
		delete(m.entries, m.order[m.next])
		m.order[m.next] = key
		m.next = (m.next + 1) % PairReportMemoCap
	}
	m.entries[key] = report
}

// CloneContactReport copies the gap, overlap, and manifold point slice. Bodies
// and faces remain shared because they are immutable.
func CloneContactReport[BodyT, FaceT, EdgeT, VertexT comparable, RelationT, ReasonT any](
	r *ContactReport[BodyT, FaceT, EdgeT, VertexT, RelationT, ReasonT],
) *ContactReport[BodyT, FaceT, EdgeT, VertexT, RelationT, ReasonT] {
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
		out.Manifold = &ContactManifold[FaceT, EdgeT, VertexT]{Points: slices.Clone(r.Manifold.Points)}
	}
	return &out
}
