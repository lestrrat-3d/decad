// Package meshthick samples the wall thickness of a closed triangle mesh: from
// the centre of each triangle it casts a ray along the inward normal and
// measures how far the ray travels before it leaves the material
// (agent-docs/mesh-wall-survey-design.md). It works on plain millimetre data
// and touches no body or document.
package meshthick

import (
	"math"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// Result is the thinnest wall the samples found.
type Result struct {
	// Thickness is the shortest inward ray, in millimetres. It is meaningful
	// only when Found is true.
	Thickness float64
	// Found reports that at least one ray left the material through another
	// triangle.
	Found bool
}

// leafSize is the most triangles a bounding-volume leaf holds.
const leafSize = 4

type box struct{ lo, hi r3.Vec }

type node struct {
	box         box
	left, right int // child node indices, or -1 for a leaf
	first, end  int // the leaf's range in order
}

// Tree is a bounding-volume tree over a triangle mesh that answers nearest-hit
// ray queries.
type Tree struct {
	verts   []r3.Vec
	tris    [][3]int
	normals []r3.Vec // each triangle's unit normal, the zero vector for one of no area
	order   []int    // triangle indices, grouped by leaf
	nodes   []node
}

// Side says which way a hit triangle must face the ray to count.
type Side int

const (
	// Exiting counts a triangle whose outward normal points along the ray: the
	// ray leaves material through it.
	Exiting Side = 1
	// Entering counts a triangle whose outward normal points against the ray:
	// the ray enters material through it.
	Entering Side = -1
)

// edgeOn is the least sine of the angle between a ray and a triangle's plane for
// a hit to count: a triangle closer than this to parallel to the ray is seen
// edge on, and a hit on it is a graze, not a crossing. A ray that runs along a
// wall of a tessellated cylinder is within float rounding of every facet's
// plane, so without this cut it reports a hit on a neighbouring facet that the
// ray only brushes.
const edgeOn = 1e-6

// NewTree sorts the mesh into a bounding-volume tree. An empty mesh gives an
// empty tree, for which Empty is true. Cancellation is polled through budget.
func NewTree(budget *proofbound.WorkBudget, verts []r3.Vec, tris [][3]int) (*Tree, error) {
	if len(verts) == 0 || len(tris) == 0 {
		return &Tree{}, nil
	}
	t := &Tree{verts: verts, tris: tris, order: make([]int, len(tris)), normals: make([]r3.Vec, len(tris))}
	for i, tri := range tris {
		t.order[i] = i
		a, b, c := verts[tri[0]], verts[tri[1]], verts[tri[2]]
		t.normals[i], _ = b.Sub(a).Cross(c.Sub(a)).Normalize()
	}
	if err := budget.Err(); err != nil {
		return nil, err
	}
	if _, err := t.build(budget, 0, len(tris)); err != nil {
		return nil, err
	}
	if err := budget.Err(); err != nil {
		return nil, err
	}
	return t, nil
}

// Empty reports a tree over no triangles.
func (t *Tree) Empty() bool { return len(t.nodes) == 0 }

// Diameter is the length of the diagonal of the mesh's bounding box.
func (t *Tree) Diameter() float64 { return t.nodes[0].box.hi.Sub(t.nodes[0].box.lo).Len() }

// Hit returns the nearest triangle, other than skip, that the ray from origin in
// the unit direction dir crosses at least minDist out and less than limit out,
// among those facing the ray as side says and not within edgeOn of parallel to
// it, with its distance. ok is false when there is none.
func (t *Tree) Hit(origin, dir r3.Vec, skip int, minDist, limit float64, side Side) (dist float64, tri int, ok bool) {
	return t.nearestHit(origin, dir, skip, minDist, limit, side)
}

// Min casts the rays. The mesh must be closed and wound counter-clockwise seen
// from outside, so each triangle's inward normal points into material; a mesh
// that is not leaves the result without meaning. A triangle of zero area casts
// no ray. Found is false when no ray hit anything. An empty mesh is Found
// false.
func Min(budget *proofbound.WorkBudget, verts []r3.Vec, tris [][3]int) (Result, error) {
	t, err := NewTree(budget, verts, tris)
	if err != nil || t.Empty() {
		return Result{}, err
	}
	scale := t.Diameter()
	if !(scale > 0) {
		return Result{}, nil
	}
	// A hit closer than this is the ray's own start, never a wall.
	nearest := 1e-9 * scale
	best := Result{Thickness: math.Inf(1)}
	for i, tri := range tris {
		if err := budget.Step(); err != nil {
			return Result{}, err
		}
		a, b, c := verts[tri[0]], verts[tri[1]], verts[tri[2]]
		normal, ok := b.Sub(a).Cross(c.Sub(a)).Normalize()
		if !ok {
			continue
		}
		origin := a.Add(b).Add(c).Scale(1.0 / 3)
		if d, _, hit := t.nearestHit(origin, normal.Scale(-1), i, nearest, best.Thickness, Exiting); hit {
			best = Result{Thickness: d, Found: true}
		}
	}
	if !best.Found {
		return Result{}, nil
	}
	return best, nil
}

// build makes the node over order[first:end] and returns its index.
func (t *Tree) build(budget *proofbound.WorkBudget, first, end int) (int, error) {
	if err := budget.Step(); err != nil {
		return 0, err
	}
	b := t.rangeBox(first, end)
	idx := len(t.nodes)
	t.nodes = append(t.nodes, node{box: b, left: -1, right: -1, first: first, end: end})
	if end-first <= leafSize {
		return idx, nil
	}
	// Split at the median of the triangle centres along the longest axis.
	extent := b.hi.Sub(b.lo)
	axis := func(v r3.Vec) float64 { return v.X }
	switch {
	case extent.Y >= extent.X && extent.Y >= extent.Z:
		axis = func(v r3.Vec) float64 { return v.Y }
	case extent.Z >= extent.X && extent.Z >= extent.Y:
		axis = func(v r3.Vec) float64 { return v.Z }
	}
	centre := func(i int) float64 {
		tri := t.tris[i]
		return axis(t.verts[tri[0]].Add(t.verts[tri[1]]).Add(t.verts[tri[2]]))
	}
	sortByKey(t.order[first:end], centre)
	mid := (first + end) / 2
	left, err := t.build(budget, first, mid)
	if err != nil {
		return 0, err
	}
	right, err := t.build(budget, mid, end)
	if err != nil {
		return 0, err
	}
	t.nodes[idx].left, t.nodes[idx].right = left, right
	return idx, nil
}

func (t *Tree) rangeBox(first, end int) box {
	lo := r3.Vec{X: math.Inf(1), Y: math.Inf(1), Z: math.Inf(1)}
	hi := r3.Vec{X: math.Inf(-1), Y: math.Inf(-1), Z: math.Inf(-1)}
	for _, i := range t.order[first:end] {
		for _, vi := range t.tris[i] {
			v := t.verts[vi]
			lo = r3.Vec{X: math.Min(lo.X, v.X), Y: math.Min(lo.Y, v.Y), Z: math.Min(lo.Z, v.Z)}
			hi = r3.Vec{X: math.Max(hi.X, v.X), Y: math.Max(hi.Y, v.Y), Z: math.Max(hi.Z, v.Z)}
		}
	}
	return box{lo: lo, hi: hi}
}

// nearestHit returns the distance along the ray from origin in direction dir
// (a unit vector) to the nearest triangle other than skip that it crosses at
// least minDist out and no farther than limit out.
func (t *Tree) nearestHit(origin, dir r3.Vec, skip int, minDist, limit float64, side Side) (float64, int, bool) {
	best, bestTri, found := limit, -1, false
	stack := []int{0}
	for len(stack) > 0 {
		n := t.nodes[stack[len(stack)-1]]
		stack = stack[:len(stack)-1]
		if !rayMeetsBox(origin, dir, n.box, best) {
			continue
		}
		if n.left < 0 {
			for _, i := range t.order[n.first:n.end] {
				if i == skip {
					continue
				}
				if facing := t.normals[i].Dot(dir) * float64(side); !(facing > edgeOn) {
					continue
				}
				if d, ok := t.hitTriangle(origin, dir, i); ok && d >= minDist && d < best {
					best, bestTri, found = d, i, true
				}
			}
			continue
		}
		stack = append(stack, n.left, n.right)
	}
	return best, bestTri, found
}

// rayMeetsBox reports whether the ray reaches the box within maxDist, by the
// slab test.
func rayMeetsBox(o, d r3.Vec, b box, maxDist float64) bool {
	tMin, tMax := 0.0, maxDist
	for _, s := range [3][3]float64{{o.X, d.X, 0}, {o.Y, d.Y, 1}, {o.Z, d.Z, 2}} {
		lo, hi := [3]float64{b.lo.X, b.lo.Y, b.lo.Z}[int(s[2])], [3]float64{b.hi.X, b.hi.Y, b.hi.Z}[int(s[2])]
		if s[1] == 0 {
			if s[0] < lo || s[0] > hi {
				return false
			}
			continue
		}
		t0, t1 := (lo-s[0])/s[1], (hi-s[0])/s[1]
		if t0 > t1 {
			t0, t1 = t1, t0
		}
		tMin, tMax = math.Max(tMin, t0), math.Min(tMax, t1)
		if tMin > tMax {
			return false
		}
	}
	return true
}

// hitTriangle is the Möller–Trumbore ray-triangle test: the distance along the
// ray to triangle i, if the ray crosses it.
func (t *Tree) hitTriangle(o, d r3.Vec, i int) (float64, bool) {
	tri := t.tris[i]
	a, b, c := t.verts[tri[0]], t.verts[tri[1]], t.verts[tri[2]]
	e1, e2 := b.Sub(a), c.Sub(a)
	p := d.Cross(e2)
	det := e1.Dot(p)
	if det == 0 {
		return 0, false
	}
	inv := 1 / det
	s := o.Sub(a)
	u := s.Dot(p) * inv
	if u < 0 || u > 1 {
		return 0, false
	}
	q := s.Cross(e1)
	v := d.Dot(q) * inv
	if v < 0 || u+v > 1 {
		return 0, false
	}
	return e2.Dot(q) * inv, true
}
