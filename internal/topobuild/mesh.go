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

// RequireClosedMesh runs docs/tessellation-design.md §1's closed-mesh audit
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
