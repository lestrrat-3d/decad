package decad

import (
	"fmt"

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
func orientJoinedMesh(m *joinedMesh) error {
	type use struct{ tri, direction int }
	edges := map[[2]int][]use{}
	for i, tri := range m.tris {
		if tri[0] == tri[1] || tri[1] == tri[2] || tri[2] == tri[0] {
			return fmt.Errorf("%w: joined mesh has a collapsed triangle", ErrUnsupported)
		}
		for j := range 3 {
			a, b := tri[j], tri[(j+1)%3]
			key := [2]int{min(a, b), max(a, b)}
			direction := 1
			if key[0] != a {
				direction = -1
			}
			edges[key] = append(edges[key], use{tri: i, direction: direction})
		}
	}
	adj := make([][]struct{ tri, same int }, len(m.tris))
	for _, pair := range edges {
		if len(pair) != 2 {
			return fmt.Errorf("%w: joined seam does not pair every mesh edge", ErrUnsupported)
		}
		same := 0
		if pair[0].direction == pair[1].direction {
			same = 1
		}
		a, b := pair[0].tri, pair[1].tri
		adj[a] = append(adj[a], struct{ tri, same int }{b, same})
		adj[b] = append(adj[b], struct{ tri, same int }{a, same})
	}
	flip := make([]int, len(m.tris))
	for i := range flip {
		flip[i] = -1
	}
	if len(flip) == 0 {
		return fmt.Errorf("%w: joined mesh is empty", ErrUnsupported)
	}
	flip[0] = 0
	queue := []int{0}
	for len(queue) != 0 {
		i := queue[0]
		queue = queue[1:]
		for _, next := range adj[i] {
			want := flip[i] ^ next.same
			if flip[next.tri] < 0 {
				flip[next.tri] = want
				queue = append(queue, next.tri)
			} else if flip[next.tri] != want {
				return fmt.Errorf("%w: joined mesh cannot be oriented", ErrUnsupported)
			}
		}
	}
	for i, f := range flip {
		if f < 0 {
			return fmt.Errorf("%w: joined mesh has another component", ErrUnsupported)
		}
		if f != 0 {
			m.tris[i][1], m.tris[i][2] = m.tris[i][2], m.tris[i][1]
		}
	}
	if tessellation.OrientationSign(m.verts, m.tris, r3.Vec{}) < 0 {
		for i := range m.tris {
			m.tris[i][1], m.tris[i][2] = m.tris[i][2], m.tris[i][1]
		}
	}
	return nil
}
