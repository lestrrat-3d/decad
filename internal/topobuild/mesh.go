package topobuild

import (
	"errors"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/decad/internal/topo"
)

// AddTriangle appends one facet of m and its source face.
func AddTriangle(m *topo.Mesh, tri [3]int, src *topo.Face) {
	m.Triangles = append(m.Triangles, tri)
	m.Source = append(m.Source, src)
}

// SetFaceBound records one source face's proven displacement, keeping the
// largest a face accumulates over the several walks that can build it (a
// coalesced side face) and lifting bound to match.
func SetFaceBound(m *topo.Mesh, f *topo.Face, delta float64) {
	if m.FaceBound == nil {
		m.FaceBound = map[*topo.Face]float64{}
	}
	// A zero is a published proof, not an absent one, so the entry is always
	// written: the record's own contract is that every source face is present,
	// and a missing key is an invariant failure the consumers refuse on.
	if prev, ok := m.FaceBound[f]; !ok || delta > prev {
		m.FaceBound[f] = delta
	}
	if delta > m.Bound {
		m.Bound = delta
	}
}

// SetVertexBounds publishes a per-vertex record (agent-docs/faceted-vertex-bounds-
// design.md §2.1) and, from it, every source face's bound as the largest
// corner entry over that face's own triangles, lifting Bound to match. The
// caller's record carries exactly one entry per vertex, and Source is
// already set; the mesh keeps its own copy.
func SetVertexBounds(m *topo.Mesh, beta []float64) {
	m.VertexBound = append([]float64(nil), beta...)
	for t, tri := range m.Triangles {
		SetFaceBound(m, m.Source[t], max(beta[tri[0]], beta[tri[1]], beta[tri[2]]))
	}
}

// VertexBounds reads β(v) for every vertex (agent-docs/faceted-vertex-bounds-
// design.md §2.1): the mesh's own record when its payload publishes one, and
// otherwise the largest FaceBound over the faces whose facets touch v. The
// derived reading is a claim the mesh already made — v lies on a facet whose
// face's true patch is within that face's bound of it — and taking the
// maximum over every incident face keeps each facet's own displacement,
// chord sagitta included, as its corner maximum. A source face missing from
// FaceBound is a broken evaluator (ErrBooleanFailed, as
// meshbool.RefuseUndecidableProximity reports it), never a zero. The returned
// slice is the caller's own.
func VertexBounds(m *topo.Mesh) ([]float64, error) {
	if m.VertexBound != nil {
		return append([]float64(nil), m.VertexBound...), nil
	}
	beta := make([]float64, len(m.Vertices))
	for t, tri := range m.Triangles {
		d, ok := m.FaceBound[m.Source[t]]
		if !ok {
			return nil, fmt.Errorf(`%w: a mesh facet's source face states no displacement bound`, decaderr.ErrBooleanFailed)
		}
		for _, v := range tri {
			beta[v] = max(beta[v], d)
		}
	}
	return beta, nil
}

// RequireClosedMesh runs agent-docs/tessellation-design.md §1's closed-mesh audit
// (tessellation.RequireClosedMesh) over m's facets, its refusal lifted by
// LiftAuditError.
func RequireClosedMesh(m *topo.Mesh) error {
	return LiftAuditError(tessellation.RequireClosedMesh(m.Triangles))
}

// LiftAuditError maps an internal/tessellation refusal onto the public
// sentinel it names and keeps the audit's own text after it, so the published
// message is unchanged. Every other error, a cancelled context above all,
// passes through untouched.
func LiftAuditError(err error) error {
	var audit *tessellation.AuditError
	if !errors.As(err, &audit) {
		return err
	}
	if audit.Sentinel == tessellation.Unsupported {
		return fmt.Errorf(`%w: %s`, decaderr.ErrUnsupported, audit.Detail)
	}
	return fmt.Errorf(`%w: %s`, decaderr.ErrDegenerate, audit.Detail)
}
