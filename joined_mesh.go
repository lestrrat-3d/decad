package decad

import (
	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/r3"
)

// joinedMesh collects source-labeled facets for structural joins and rewrites.
// Each caller supplies its own geometric admission and boundary proofs.
type joinedMesh struct {
	Verts  []r3.Vec
	Tris   [][3]int
	Src    []int
	Groups []facetGroup
}

func (m *joinedMesh) Add(t [3]int, group int) {
	m.Tris = append(m.Tris, t)
	m.Src = append(m.Src, group)
}

// orientJoinedMesh proves edge pairing and one connected component before the
// caller's geometric audit. It makes each pair's directed edge opposite.
func orientJoinedMesh(m *joinedMesh) error { return tessellation.OrientJoinedMesh(m.Verts, m.Tris) }
