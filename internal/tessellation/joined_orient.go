package tessellation

import (
	"fmt"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/r3"
)

// OrientJoinedMesh proves edge pairing and one connected component before the
// caller's geometric audit. It makes each pair's directed edge opposite.
func OrientJoinedMesh(verts []r3.Vec, tris [][3]int) error {
	adj, err := joinedEdgePairs(tris)
	if err != nil {
		return err
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

type joinedNeighbor struct{ tri, same int }

// joinedEdgePairs records each edge's first signed local ordinal, then writes
// its two neighboring facets into flat storage. A paired edge keeps a zero
// sentinel so a third owner fails without allocating a slice per edge.
func joinedEdgePairs(tris [][3]int) ([][3]joinedNeighbor, error) {
	edges := make(map[[2]int]int, 3*len(tris)/2)
	adj := make([][3]joinedNeighbor, len(tris))
	paired := true
	for i, tri := range tris {
		if tri[0] == tri[1] || tri[1] == tri[2] || tri[2] == tri[0] {
			return nil, fmt.Errorf("%w: joined mesh has a collapsed triangle", decaderr.ErrUnsupported)
		}
		for j := range 3 {
			a, b := tri[j], tri[(j+1)%3]
			key := [2]int{min(a, b), max(a, b)}
			code := 3*i + j + 1
			if key[0] != a {
				code = -code
			}
			first, seen := edges[key]
			if !seen {
				edges[key] = code
				continue
			}
			if first == 0 {
				paired = false
				continue
			}
			same := 0
			if (first < 0) == (code < 0) {
				same = 1
			}
			if first < 0 {
				first = -first
			}
			first--
			adj[first/3][first%3] = joinedNeighbor{tri: i, same: same}
			adj[i][j] = joinedNeighbor{tri: first / 3, same: same}
			edges[key] = 0
		}
	}
	for _, first := range edges {
		if first != 0 {
			paired = false
			break
		}
	}
	if !paired {
		return nil, fmt.Errorf("%w: joined seam does not pair every mesh edge", decaderr.ErrUnsupported)
	}
	return adj, nil
}
