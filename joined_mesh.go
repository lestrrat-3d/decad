package decad

import (
	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/r3"
)

// joinedMesh collects source-labeled facets for structural joins and rewrites.
// Each caller supplies its own geometric admission and boundary proofs.
type joinedMesh struct {
	verts  []r3.Vec
	tris   [][3]int
	src    []int
	groups []facetGroup
}

func (m *joinedMesh) add(t [3]int, group int) {
	m.tris = append(m.tris, t)
	m.src = append(m.src, group)
}

// orientJoinedMesh proves edge pairing and one connected component before the
// caller's geometric audit. It makes each pair's directed edge opposite.
func orientJoinedMesh(m *joinedMesh) error { return tessellation.OrientJoinedMesh(m.verts, m.tris) }
