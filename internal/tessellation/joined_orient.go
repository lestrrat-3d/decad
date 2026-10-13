package tessellation

import (
	"fmt"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/r3"
)

// OrientJoinedMesh proves edge pairing and one connected component before the
// caller's geometric audit. It makes each pair's directed edge opposite.
func OrientJoinedMesh(verts []r3.Vec, tris [][3]int) error {
	type use struct{ tri, direction int }
	edges := map[[2]int][]use{}
	for i, tri := range tris {
		if tri[0] == tri[1] || tri[1] == tri[2] || tri[2] == tri[0] {
			return fmt.Errorf("%w: joined mesh has a collapsed triangle", decaderr.ErrUnsupported)
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
	adj := make([][]struct{ tri, same int }, len(tris))
	for _, pair := range edges {
		if len(pair) != 2 {
			return fmt.Errorf("%w: joined seam does not pair every mesh edge", decaderr.ErrUnsupported)
		}
		same := 0
		if pair[0].direction == pair[1].direction {
			same = 1
		}
		a, b := pair[0].tri, pair[1].tri
		adj[a] = append(adj[a], struct{ tri, same int }{b, same})
		adj[b] = append(adj[b], struct{ tri, same int }{a, same})
	}
	flip := make([]int, len(tris))
	for i := range flip {
		flip[i] = -1
	}
	if len(flip) == 0 {
		return fmt.Errorf("%w: joined mesh is empty", decaderr.ErrUnsupported)
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
				return fmt.Errorf("%w: joined mesh cannot be oriented", decaderr.ErrUnsupported)
			}
		}
	}
	for i, f := range flip {
		if f < 0 {
			return fmt.Errorf("%w: joined mesh has another component", decaderr.ErrUnsupported)
		}
		if f != 0 {
			tris[i][1], tris[i][2] = tris[i][2], tris[i][1]
		}
	}
	if OrientationSign(verts, tris, r3.Vec{}) < 0 {
		for i := range tris {
			tris[i][1], tris[i][2] = tris[i][2], tris[i][1]
		}
	}
	return nil
}
