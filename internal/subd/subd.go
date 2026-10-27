// Package subd subdivides a polygon cage into its Catmull-Clark surface
// (agent-docs/subd-design.md). It works on plain millimetre data and touches no
// body, document or sketch.
package subd

import (
	"errors"
	"fmt"

	"github.com/lestrrat-3d/r3"
)

// ErrDegenerate is returned for a cage that is not a manifold of polygons, or one
// too large to subdivide.
var ErrDegenerate = errors.New("subd: degenerate cage")

// MaxFaces bounds the faces a cage may hold at any level, and MaxLevels the levels
// Subdivide may be asked for.
const (
	MaxFaces  = 1_000_000
	MaxLevels = 8
)

type edgeKey [2]int

func keyOf(a, b int) edgeKey {
	if a > b {
		a, b = b, a
	}
	return edgeKey{a, b}
}

// Cage is a polygon mesh: Faces list vertex indices of Vertices, each face counter-clockwise
// seen from outside.
type Cage struct {
	Vertices []r3.Vec
	Faces    [][]int
}

// Check refuses a cage that is not a manifold: a face of fewer than three distinct vertices
// or naming a missing vertex, an edge used by more than two faces, and an edge two faces walk
// the same way (they disagree on which side is outside).
func (c Cage) Check() error {
	if len(c.Vertices) == 0 || len(c.Faces) == 0 {
		return fmt.Errorf("%w: the cage has no faces", ErrDegenerate)
	}
	if len(c.Faces) > MaxFaces {
		return fmt.Errorf("%w: more than %d faces", ErrDegenerate, MaxFaces)
	}
	directed := map[[2]int]int{}
	uses := map[edgeKey]int{}
	for fi, f := range c.Faces {
		if len(f) < 3 {
			return fmt.Errorf("%w: face %d has %d vertices", ErrDegenerate, fi, len(f))
		}
		seen := map[int]bool{}
		for i, v := range f {
			if v < 0 || v >= len(c.Vertices) {
				return fmt.Errorf("%w: face %d names vertex %d of %d", ErrDegenerate, fi, v, len(c.Vertices))
			}
			if seen[v] {
				return fmt.Errorf("%w: face %d repeats vertex %d", ErrDegenerate, fi, v)
			}
			seen[v] = true
			w := f[(i+1)%len(f)]
			directed[[2]int{v, w}]++
			uses[keyOf(v, w)]++
		}
	}
	for e, n := range uses {
		if n > 2 {
			return fmt.Errorf("%w: edge %d-%d is used by %d faces", ErrDegenerate, e[0], e[1], n)
		}
	}
	for d, n := range directed {
		if n > 1 {
			return fmt.Errorf("%w: two faces walk edge %d-%d the same way, so they disagree about which side is outside", ErrDegenerate, d[0], d[1])
		}
	}
	return nil
}

// Subdivide applies levels rounds of Catmull-Clark subdivision to the cage and returns the
// refined cage. Each round replaces an n-gon by n quads. A face point is its face's mean; an
// interior edge point is the mean of the edge's ends and the two face points beside it, and a
// boundary edge point the edge's midpoint; an interior vertex of valence n moves to
// (Q + 2R + (n − 3)P)/n, Q the mean of the face points around it, R the mean of the midpoints
// of the edges around it, and a boundary vertex to (6P + A + B)/8, A and B its boundary
// neighbours. A vertex on the boundary that does not touch exactly two boundary edges is
// ErrDegenerate.
func Subdivide(c Cage, levels int) (Cage, error) {
	if levels < 0 || levels > MaxLevels {
		return Cage{}, fmt.Errorf("%w: levels must lie from 0 to %d, got %d", ErrDegenerate, MaxLevels, levels)
	}
	if err := c.Check(); err != nil {
		return Cage{}, err
	}
	for range levels {
		next, err := step(c)
		if err != nil {
			return Cage{}, err
		}
		c = next
		if len(c.Faces) > MaxFaces {
			return Cage{}, fmt.Errorf("%w: more than %d faces", ErrDegenerate, MaxFaces)
		}
	}
	return c, nil
}

// step is one round of Subdivide on a checked cage.
func step(c Cage) (Cage, error) {
	nv := len(c.Vertices)
	facePoint := make([]r3.Vec, len(c.Faces))
	adjacent := map[edgeKey][]int{}
	vertexFaces := make([][]int, nv)
	edgeIndex := map[edgeKey]int{}
	var edgeKeys []edgeKey
	for fi, f := range c.Faces {
		var s r3.Vec
		for i, v := range f {
			s = s.Add(c.Vertices[v])
			k := keyOf(v, f[(i+1)%len(f)])
			adjacent[k] = append(adjacent[k], fi)
			vertexFaces[v] = append(vertexFaces[v], fi)
			if _, ok := edgeIndex[k]; !ok {
				edgeIndex[k] = len(edgeKeys)
				edgeKeys = append(edgeKeys, k)
			}
		}
		facePoint[fi] = s.Scale(1 / float64(len(f)))
	}
	boundary := make([][]int, nv)
	vertexEdges := make([][]int, nv)
	edgePoints := make([]r3.Vec, len(edgeKeys))
	edgeMid := make([]r3.Vec, len(edgeKeys))
	for ei, k := range edgeKeys {
		a, b := c.Vertices[k[0]], c.Vertices[k[1]]
		mid := a.Add(b).Scale(0.5)
		edgeMid[ei] = mid
		vertexEdges[k[0]] = append(vertexEdges[k[0]], ei)
		vertexEdges[k[1]] = append(vertexEdges[k[1]], ei)
		faces := adjacent[k]
		if len(faces) == 2 {
			edgePoints[ei] = a.Add(b).Add(facePoint[faces[0]]).Add(facePoint[faces[1]]).Scale(0.25)
		} else {
			edgePoints[ei] = mid
			boundary[k[0]] = append(boundary[k[0]], k[1])
			boundary[k[1]] = append(boundary[k[1]], k[0])
		}
	}
	out := Cage{Vertices: make([]r3.Vec, 0, nv+len(c.Faces)+len(edgeKeys))}
	for v := range nv {
		p := c.Vertices[v]
		switch len(boundary[v]) {
		case 0:
			n := float64(len(vertexFaces[v]))
			if n < 3 {
				return Cage{}, fmt.Errorf("%w: vertex %d is in %d faces and has no closed ring of them", ErrDegenerate, v, len(vertexFaces[v]))
			}
			var q, r r3.Vec
			for _, fi := range vertexFaces[v] {
				q = q.Add(facePoint[fi])
			}
			for _, ei := range vertexEdges[v] {
				r = r.Add(edgeMid[ei])
			}
			q = q.Scale(1 / n)
			r = r.Scale(1 / float64(len(vertexEdges[v])))
			out.Vertices = append(out.Vertices, q.Add(r.Scale(2)).Add(p.Scale(n-3)).Scale(1/n))
		case 2:
			a, b := c.Vertices[boundary[v][0]], c.Vertices[boundary[v][1]]
			out.Vertices = append(out.Vertices, p.Scale(6).Add(a).Add(b).Scale(1.0/8))
		default:
			return Cage{}, fmt.Errorf("%w: vertex %d touches %d boundary edges, and only a boundary of simple curves is subdivided",
				ErrDegenerate, v, len(boundary[v]))
		}
	}
	faceBase := len(out.Vertices)
	out.Vertices = append(out.Vertices, facePoint...)
	edgeBase := len(out.Vertices)
	out.Vertices = append(out.Vertices, edgePoints...)
	for fi, f := range c.Faces {
		for i, v := range f {
			next := f[(i+1)%len(f)]
			prev := f[(i+len(f)-1)%len(f)]
			out.Faces = append(out.Faces, []int{
				v, edgeBase + edgeIndex[keyOf(v, next)], faceBase + fi, edgeBase + edgeIndex[keyOf(prev, v)],
			})
		}
	}
	return out, nil
}

// Triangles cuts every face of the cage into triangles by fanning from its first vertex.
func (c Cage) Triangles() [][3]int {
	var out [][3]int
	for _, f := range c.Faces {
		for i := 1; i+1 < len(f); i++ {
			out = append(out, [3]int{f[0], f[i], f[i+1]})
		}
	}
	return out
}
