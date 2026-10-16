package decad

import (
	"github.com/lestrrat-3d/decad/internal/tessellation"
)

// orientJoinedMesh proves edge pairing and one connected component before the
// caller's geometric audit. It makes each pair's directed edge opposite.
func orientJoinedMesh(m *joinedMesh) error { return tessellation.OrientJoinedMesh(m.Verts, m.Tris) }
