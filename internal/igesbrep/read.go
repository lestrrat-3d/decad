// Package igesbrep reads the planar-faced solids of an IGES model into a triangle mesh
// (agent-docs/iges-import-design.md). It reads the manifold solid B-rep entity and the shells, faces,
// loops, edges and vertices below it, whose surfaces are planes and whose edges are straight lines, and
// refuses the rest.
package igesbrep

import (
	"errors"
	"fmt"
	"math"
	"sort"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/igesread"
	"github.com/lestrrat-3d/decad/internal/polytri"
	"github.com/lestrrat-3d/r3"
)

// ErrMalformed marks a model whose entities do not form the B-rep they claim.
var ErrMalformed = errors.New("igesbrep: malformed B-rep")

// ErrTooLarge marks a model that triangulates to more than the limit.
var ErrTooLarge = errors.New("igesbrep: too many triangles")

// Mesh is the triangulated solids, millimetres, vertices welded by coordinate and every triangle wound
// outward from its solid's material.
type Mesh struct {
	Vertices  []r3.Vec
	Triangles [][3]int
}

// IGES entity type numbers read here.
const (
	typeLine         = 110
	typePlane        = 108
	typeVertexList   = 502
	typeEdgeList     = 504
	typeLoop         = 508
	typeFace         = 510
	typeShell        = 514
	typeMSBO         = 186
	typePlaneSurface = 190
)

type reader struct {
	m     *igesread.Model
	limit int
	verts []r3.Vec
	index map[[3]float64]int
	tris  [][3]int
}

// Read triangulates every manifold solid B-rep of the model. limit is the most triangles it accepts.
func Read(m *igesread.Model, limit int) (Mesh, error) {
	r := &reader{m: m, limit: limit, index: map[[3]float64]int{}}
	if m.ModelScale != 1 {
		return Mesh{}, fmt.Errorf("%w: the model space scale is %g, and a scaled model space is not read", decaderr.ErrUnsupported, m.ModelScale)
	}
	for _, e := range m.Entities {
		if e.Transform != 0 {
			return Mesh{}, fmt.Errorf("%w: entity %d is placed by a transformation matrix, and placements are not read", decaderr.ErrUnsupported, e.Seq)
		}
	}
	seqs := make([]int, 0, len(m.Entities))
	for seq := range m.Entities {
		seqs = append(seqs, seq)
	}
	sort.Ints(seqs)
	solids := 0
	for _, seq := range seqs {
		e := m.Entities[seq]
		if e.Type != typeMSBO {
			continue
		}
		if len(e.Params) < 3 {
			return Mesh{}, fmt.Errorf("%w: solid %d has too few parameters", ErrMalformed, seq)
		}
		start := len(r.tris)
		shells := [][2]int{}
		first, err := ptr(e.Params[0])
		if err != nil {
			return Mesh{}, err
		}
		flag, err := igesread.Int(e.Params[1])
		if err != nil {
			return Mesh{}, err
		}
		shells = append(shells, [2]int{first, flag})
		voids, err := igesread.Int(e.Params[2])
		if err != nil || voids < 0 || 3+2*voids > len(e.Params) {
			return Mesh{}, fmt.Errorf("%w: solid %d names %d voids and holds the parameters for fewer", ErrMalformed, seq, voids)
		}
		for k := range voids {
			p, err := ptr(e.Params[3+2*k])
			if err != nil {
				return Mesh{}, err
			}
			f, err := igesread.Int(e.Params[4+2*k])
			if err != nil {
				return Mesh{}, err
			}
			shells = append(shells, [2]int{p, f})
		}
		for _, sh := range shells {
			if err := r.shell(sh[0]); err != nil {
				return Mesh{}, err
			}
		}
		// The solid's own triangles are oriented as one mesh: each shell outward from the solid's material, a void's
		// shell into its cavity.
		if err := r.orient(start, len(shells)); err != nil {
			return Mesh{}, err
		}
		solids++
	}
	if solids == 0 {
		return Mesh{}, fmt.Errorf("%w: the model holds no manifold solid B-rep entity", ErrMalformed)
	}
	return Mesh{Vertices: r.verts, Triangles: r.tris}, nil
}

func ptr(s string) (int, error) {
	n, err := igesread.Int(s)
	if err != nil {
		return 0, err
	}
	if n <= 0 {
		return 0, fmt.Errorf("%w: %d is not a directory pointer", ErrMalformed, n)
	}
	return n, nil
}

func (r *reader) entity(seq, want int, what string) (*igesread.Entity, error) {
	e, ok := r.m.Entities[seq]
	if !ok {
		return nil, fmt.Errorf("%w: %s points at directory line %d, which holds no entity", ErrMalformed, what, seq)
	}
	if e.Type != want {
		return nil, fmt.Errorf("%w: %s points at an entity of type %d, not %d", ErrMalformed, what, e.Type, want)
	}
	return e, nil
}

// vertex welds a point to a mesh vertex by coordinate.
func (r *reader) vertex(p r3.Vec) int {
	key := [3]float64{p.X, p.Y, p.Z}
	if i, ok := r.index[key]; ok {
		return i
	}
	r.verts = append(r.verts, p)
	r.index[key] = len(r.verts) - 1
	return len(r.verts) - 1
}

// vertexAt reads vertex number k (from one) of the vertex list at seq, in millimetres.
func (r *reader) vertexAt(seq, k int) (r3.Vec, error) {
	e, err := r.entity(seq, typeVertexList, "an edge")
	if err != nil {
		return r3.Vec{}, err
	}
	n, err := igesread.Int(first(e.Params))
	if err != nil || n < 0 || 1+3*n > len(e.Params) {
		return r3.Vec{}, fmt.Errorf("%w: vertex list %d is malformed", ErrMalformed, seq)
	}
	if k < 1 || k > n {
		return r3.Vec{}, fmt.Errorf("%w: vertex %d is not in the list of %d at %d", ErrMalformed, k, n, seq)
	}
	xyz, err := igesread.Reals(e.Params[1+3*(k-1) : 1+3*k])
	if err != nil {
		return r3.Vec{}, err
	}
	s := r.m.Scale
	for _, c := range xyz {
		if math.IsNaN(c) || math.IsInf(c, 0) {
			return r3.Vec{}, fmt.Errorf("%w: vertex list %d holds a coordinate that is not finite", ErrMalformed, seq)
		}
	}
	return r3.Vec{X: xyz[0] * s, Y: xyz[1] * s, Z: xyz[2] * s}, nil
}

func first(p []string) string {
	if len(p) == 0 {
		return ""
	}
	return p[0]
}

// edge is the two end vertices of one edge-list entry, from its start to its terminate.
func (r *reader) edge(list, k int) ([2]r3.Vec, error) {
	e, err := r.entity(list, typeEdgeList, "a loop")
	if err != nil {
		return [2]r3.Vec{}, err
	}
	n, err := igesread.Int(first(e.Params))
	if err != nil || n < 0 || 1+5*n > len(e.Params) {
		return [2]r3.Vec{}, fmt.Errorf("%w: edge list %d is malformed", ErrMalformed, list)
	}
	if k < 1 || k > n {
		return [2]r3.Vec{}, fmt.Errorf("%w: edge %d is not in the list of %d at %d", ErrMalformed, k, n, list)
	}
	rec := e.Params[1+5*(k-1) : 1+5*k]
	curve, err := ptr(rec[0])
	if err != nil {
		return [2]r3.Vec{}, err
	}
	c, ok := r.m.Entities[curve]
	if !ok {
		return [2]r3.Vec{}, fmt.Errorf("%w: edge %d of list %d points at no curve", ErrMalformed, k, list)
	}
	if c.Type != typeLine {
		return [2]r3.Vec{}, fmt.Errorf("%w: an edge is a curve of type %d, and only straight lines (110) are read", decaderr.ErrUnsupported, c.Type)
	}
	var ends [2]r3.Vec
	for i := range 2 {
		lp, err := ptr(rec[1+2*i])
		if err != nil {
			return [2]r3.Vec{}, err
		}
		vi, err := igesread.Int(rec[2+2*i])
		if err != nil {
			return [2]r3.Vec{}, err
		}
		if ends[i], err = r.vertexAt(lp, vi); err != nil {
			return [2]r3.Vec{}, err
		}
	}
	return ends, nil
}

// loop reads the vertices of the loop at seq in order round it.
func (r *reader) loop(seq int) ([]r3.Vec, error) {
	e, err := r.entity(seq, typeLoop, "a face")
	if err != nil {
		return nil, err
	}
	n, err := igesread.Int(first(e.Params))
	if err != nil || n < 3 {
		return nil, fmt.Errorf("%w: loop %d has fewer than three edges", ErrMalformed, seq)
	}
	at := 1
	var verts []r3.Vec
	var prevEnd *r3.Vec
	var firstStart r3.Vec
	for k := range n {
		if at+4 >= len(e.Params) {
			return nil, fmt.Errorf("%w: loop %d is cut short", ErrMalformed, seq)
		}
		kind, err := igesread.Int(e.Params[at])
		if err != nil {
			return nil, err
		}
		if kind != 0 {
			return nil, fmt.Errorf("%w: loop %d holds a vertex entry, and only edge entries are read", decaderr.ErrUnsupported, seq)
		}
		list, err := ptr(e.Params[at+1])
		if err != nil {
			return nil, err
		}
		idx, err := igesread.Int(e.Params[at+2])
		if err != nil {
			return nil, err
		}
		orient, err := igesread.Int(e.Params[at+3])
		if err != nil {
			return nil, err
		}
		iso, err := igesread.Int(e.Params[at+4])
		if err != nil || iso < 0 {
			return nil, fmt.Errorf("%w: loop %d has a bad parameter-space curve count", ErrMalformed, seq)
		}
		at += 5 + 2*iso
		ends, err := r.edge(list, idx)
		if err != nil {
			return nil, err
		}
		start, end := ends[0], ends[1]
		if orient == 0 {
			start, end = end, start
		}
		if prevEnd != nil && *prevEnd != start {
			return nil, fmt.Errorf("%w: edge %d of loop %d does not start where the one before it ends", ErrMalformed, k+1, seq)
		}
		if k == 0 {
			firstStart = start
		}
		verts = append(verts, start)
		e := end
		prevEnd = &e
	}
	if *prevEnd != firstStart {
		return nil, fmt.Errorf("%w: loop %d does not close", ErrMalformed, seq)
	}
	return verts, nil
}

// shell triangulates the faces of the shell at seq.
func (r *reader) shell(seq int) error {
	e, err := r.entity(seq, typeShell, "a solid")
	if err != nil {
		return err
	}
	n, err := igesread.Int(first(e.Params))
	if err != nil || n < 4 || 1+2*n > len(e.Params) {
		return fmt.Errorf("%w: shell %d has fewer than four faces or too few parameters", ErrMalformed, seq)
	}
	for k := range n {
		fp, err := ptr(e.Params[1+2*k])
		if err != nil {
			return err
		}
		if err := r.face(fp); err != nil {
			return err
		}
	}
	return nil
}

// face triangulates one planar face.
func (r *reader) face(seq int) error {
	e, err := r.entity(seq, typeFace, "a shell")
	if err != nil {
		return err
	}
	if len(e.Params) < 3 {
		return fmt.Errorf("%w: face %d has too few parameters", ErrMalformed, seq)
	}
	surface, err := ptr(e.Params[0])
	if err != nil {
		return err
	}
	if s, ok := r.m.Entities[surface]; !ok || (s.Type != typePlane && s.Type != typePlaneSurface) {
		kind := 0
		if ok {
			kind = s.Type
		}
		return fmt.Errorf("%w: face %d lies on a surface of type %d, and only planes (108, 190) are read", decaderr.ErrUnsupported, seq, kind)
	}
	nl, err := igesread.Int(e.Params[1])
	if err != nil || nl < 1 || 3+nl > len(e.Params) {
		return fmt.Errorf("%w: face %d has a bad loop count", ErrMalformed, seq)
	}
	outerFlag, err := igesread.Int(e.Params[2])
	if err != nil {
		return err
	}
	if outerFlag != 1 {
		return fmt.Errorf("%w: face %d has no outer loop first, and a face without an outer boundary is not read", decaderr.ErrUnsupported, seq)
	}
	var rings [][]r3.Vec
	for k := range nl {
		lp, err := ptr(e.Params[3+k])
		if err != nil {
			return err
		}
		ring, err := r.loop(lp)
		if err != nil {
			return err
		}
		rings = append(rings, ring)
	}
	return r.triangulate(seq, rings)
}

// triangulate cuts a planar polygon with holes into triangles in the plane of its outer loop, wound as that loop runs.
func (r *reader) triangulate(face int, rings [][]r3.Vec) error {
	outer := rings[0]
	// The outer loop's Newell normal gives the plane and the winding.
	var nx, ny, nz float64
	for i, p := range outer {
		q := outer[(i+1)%len(outer)]
		nx += (p.Y - q.Y) * (p.Z + q.Z)
		ny += (p.Z - q.Z) * (p.X + q.X)
		nz += (p.X - q.X) * (p.Y + q.Y)
	}
	normal, ok := r3.Vec{X: nx, Y: ny, Z: nz}.Normalize()
	if !ok {
		return fmt.Errorf("%w: the outer loop of face %d encloses no area", ErrMalformed, face)
	}
	u, ok := perpendicular(normal).Normalize()
	if !ok {
		return fmt.Errorf("%w: face %d has no plane frame", ErrMalformed, face)
	}
	v := normal.Cross(u)
	scale := 0.0
	for _, p := range outer {
		scale = math.Max(scale, math.Max(math.Abs(p.X), math.Max(math.Abs(p.Y), math.Abs(p.Z))))
	}
	tol := 1e-9 * math.Max(1, scale)
	flat := func(ring []r3.Vec) ([]polytri.Point, error) {
		out := make([]polytri.Point, len(ring))
		for i, p := range ring {
			if math.Abs(p.Sub(outer[0]).Dot(normal)) > tol {
				return nil, fmt.Errorf("%w: a loop of face %d leaves the plane of its outer loop", ErrMalformed, face)
			}
			out[i] = polytri.Point{p.Dot(u), p.Dot(v)}
		}
		return out, nil
	}
	outer2, err := flat(outer)
	if err != nil {
		return err
	}
	var holes [][]polytri.Point
	holeVerts := [][]r3.Vec{}
	for _, ring := range rings[1:] {
		h, err := flat(ring)
		if err != nil {
			return err
		}
		// A hole runs clockwise: turn one that does not.
		if area2(h) > 0 {
			for i, j := 0, len(h)-1; i < j; i, j = i+1, j-1 {
				h[i], h[j] = h[j], h[i]
			}
			rev := append([]r3.Vec(nil), ring...)
			for i, j := 0, len(rev)-1; i < j; i, j = i+1, j-1 {
				rev[i], rev[j] = rev[j], rev[i]
			}
			ring = rev
		}
		holes = append(holes, h)
		holeVerts = append(holeVerts, ring)
	}
	_, tris, err := polytri.Triangulate(outer2, holes)
	if err != nil {
		return fmt.Errorf("%w: face %d: %s", ErrMalformed, face, err)
	}
	all := append([]r3.Vec(nil), outer...)
	for _, h := range holeVerts {
		all = append(all, h...)
	}
	for _, t := range tris {
		r.tris = append(r.tris, [3]int{r.vertex(all[t[0]]), r.vertex(all[t[1]]), r.vertex(all[t[2]])})
	}
	if len(r.tris) > r.limit {
		return fmt.Errorf("%w: more than %d triangles", ErrTooLarge, r.limit)
	}
	return nil
}

func area2(p []polytri.Point) float64 {
	a := 0.0
	for i, c := range p {
		d := p[(i+1)%len(p)]
		a += c[0]*d[1] - d[0]*c[1]
	}
	return a
}

// perpendicular returns a vector square to n.
func perpendicular(n r3.Vec) r3.Vec {
	if math.Abs(n.X) <= math.Abs(n.Y) && math.Abs(n.X) <= math.Abs(n.Z) {
		return n.Cross(r3.Vec{X: 1})
	}
	if math.Abs(n.Y) <= math.Abs(n.Z) {
		return n.Cross(r3.Vec{Y: 1})
	}
	return n.Cross(r3.Vec{Z: 1})
}

// orient winds the triangles from index start on consistently and outward. The faces' own windings follow the
// loops the file gives, which a file need not keep agreeing across faces, so each connected piece is wound by
// crossing shared edges, and each is then turned so that its signed volume is positive for the solid's own shell
// and negative for a void's: shells are told apart by connectivity, the first piece being the solid.
func (r *reader) orient(start, shells int) error {
	tris := r.tris[start:]
	type edge struct{ a, b int }
	owner := map[edge][]int{}
	for i, t := range tris {
		for k := range 3 {
			a, b := t[k], t[(k+1)%3]
			if a > b {
				a, b = b, a
			}
			owner[edge{a, b}] = append(owner[edge{a, b}], i)
		}
	}
	for e, o := range owner {
		if len(o) != 2 {
			return fmt.Errorf("%w: an edge between vertices %d and %d is used by %d faces, not two", ErrMalformed, e.a, e.b, len(o))
		}
	}
	piece := make([]int, len(tris))
	for i := range piece {
		piece[i] = -1
	}
	pieces := 0
	directed := func(t [3]int, a, b int) int { // +1 when t runs a to b, -1 when b to a, 0 otherwise
		for k := range 3 {
			x, y := t[k], t[(k+1)%3]
			if x == a && y == b {
				return 1
			}
			if x == b && y == a {
				return -1
			}
		}
		return 0
	}
	for s := range tris {
		if piece[s] >= 0 {
			continue
		}
		piece[s] = pieces
		queue := []int{s}
		for len(queue) > 0 {
			i := queue[0]
			queue = queue[1:]
			t := tris[i]
			for k := range 3 {
				a, b := t[k], t[(k+1)%3]
				lo, hi := a, b
				if lo > hi {
					lo, hi = hi, lo
				}
				for _, j := range owner[edge{lo, hi}] {
					if j == i {
						continue
					}
					// Two triangles sharing an edge are wound alike when they run it in opposite senses.
					if piece[j] < 0 {
						if directed(tris[j], a, b) == 1 {
							tris[j][1], tris[j][2] = tris[j][2], tris[j][1]
						}
						piece[j] = pieces
						queue = append(queue, j)
					} else if directed(tris[j], a, b) == 1 {
						return fmt.Errorf("%w: the faces cannot be wound consistently, so the shell is not orientable", ErrMalformed)
					}
				}
			}
		}
		pieces++
	}
	if pieces != shells {
		return fmt.Errorf("%w: the solid names %d shells and its faces make %d closed pieces", ErrMalformed, shells, pieces)
	}
	// A piece's volume decides its sense: the first is the solid and takes a positive volume, the rest are voids.
	volume := make([]float64, pieces)
	for i, t := range tris {
		a, b, c := r.verts[t[0]], r.verts[t[1]], r.verts[t[2]]
		volume[piece[i]] += a.Dot(b.Cross(c)) / 6
	}
	for i := range tris {
		want := 1.0
		if piece[i] > 0 {
			want = -1
		}
		if volume[piece[i]]*want < 0 {
			tris[i][1], tris[i][2] = tris[i][2], tris[i][1]
		}
	}
	return nil
}
