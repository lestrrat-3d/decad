// Package explodeblock decides, from tessellations alone, which part stands in
// the way of another along a direction (agent-docs/parts-design.md §5.7). It
// works on plain vertices, triangles and bounds and touches no body.
//
// A [Projection] is the per-mesh half: the triangles projected onto the plane
// perpendicular to one direction, with their heights along it. It is immutable
// after [Project], so any number of goroutines and any number of identical
// instances can share one. [Block] is the per-pair half.
package explodeblock

import (
	"errors"
	"fmt"
	"math"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// Mesh is a triangulated closed boundary in millimetres. Bound is the proven
// deviation of the mesh from the body (Mesh.Bound of the root package).
type Mesh struct {
	Vertices  []r3.Vec
	Triangles [][3]int
	Bound     units.Value
}

// Projection is a mesh projected along one direction.
type Projection struct {
	dir   r3.Vec
	u, v  r3.Vec
	bound float64
	tris  []tri
	grid  grid
	// topZ is the highest corner of any kept triangle along the direction.
	topZ float64
	// local box of every vertex, including skipped parallel triangles
	minU, maxU, minV, maxV, minW, maxW float64
}

// Dir returns the unit direction of the projection.
func (p *Projection) Dir() r3.Vec { return p.dir }

// Bound returns the mesh bound the projection was built with.
func (p *Projection) Bound() units.Value { return units.Millimeters(p.bound) }

// basis returns an orthonormal (u, v, d) with u x v = d.
func basis(d r3.Vec) (r3.Vec, r3.Vec) {
	axis := r3.NewVec(1, 0, 0)
	if math.Abs(d.Y) < math.Abs(d.X) && math.Abs(d.Y) <= math.Abs(d.Z) {
		axis = r3.NewVec(0, 1, 0)
	} else if math.Abs(d.Z) < math.Abs(d.X) && math.Abs(d.Z) < math.Abs(d.Y) {
		axis = r3.NewVec(0, 0, 1)
	}
	u, _ := d.Cross(axis).Normalize()
	v := d.Cross(u)
	return u, v
}

// Project projects m along dir. dir need not be unit but must be finite and
// non-zero. Triangles whose shadow is empty (parallel to dir) are dropped.
func Project(m Mesh, dir r3.Vec) (*Projection, error) {
	d, ok := dir.Normalize()
	if !ok {
		return nil, errors.New("explodeblock: direction is zero or not finite")
	}
	if m.Bound.Base() < 0 || math.IsNaN(m.Bound.Base()) || math.IsInf(m.Bound.Base(), 0) {
		return nil, errors.New("explodeblock: bound is negative or not finite")
	}
	u, v := basis(d)
	p := &Projection{dir: d, u: u, v: v, bound: m.Bound.Base(), tris: make([]tri, 0, len(m.Triangles)), topZ: math.Inf(-1)}
	p.minU, p.minV, p.minW = math.Inf(1), math.Inf(1), math.Inf(1)
	p.maxU, p.maxV, p.maxW = math.Inf(-1), math.Inf(-1), math.Inf(-1)
	for i, x := range m.Vertices {
		if math.IsNaN(x.X+x.Y+x.Z) || math.IsInf(x.X+x.Y+x.Z, 0) {
			return nil, fmt.Errorf("explodeblock: vertex %d is not finite", i)
		}
		a, b, c := x.Dot(u), x.Dot(v), x.Dot(d)
		p.minU, p.maxU = math.Min(p.minU, a), math.Max(p.maxU, a)
		p.minV, p.maxV = math.Min(p.minV, b), math.Max(p.maxV, b)
		p.minW, p.maxW = math.Min(p.minW, c), math.Max(p.maxW, c)
	}
	for i, t := range m.Triangles {
		for _, k := range t {
			if k < 0 || k >= len(m.Vertices) {
				return nil, fmt.Errorf("explodeblock: triangle %d indexes vertex %d of %d", i, k, len(m.Vertices))
			}
		}
		if tr, ok := project(m.Vertices, t, u, v, d); ok {
			p.tris = append(p.tris, tr)
			p.topZ = math.Max(p.topZ, tr.maxZ)
		}
	}
	p.grid = newGrid(p.tris, p.minU, p.maxU, p.minV, p.maxV)
	return p, nil
}

func project(vs []r3.Vec, t [3]int, u, v, d r3.Vec) (tri, bool) {
	var tr tri
	for i, k := range t {
		x := vs[k]
		tr.v[i] = p2{x.Dot(u), x.Dot(v)}
		tr.z[i] = x.Dot(d)
	}
	area2 := cross2(tr.v[0], tr.v[1], tr.v[2])
	longest := 0.0
	for i := range 3 {
		a, b := tr.v[i], tr.v[(i+1)%3]
		longest = math.Max(longest, (a.x-b.x)*(a.x-b.x)+(a.y-b.y)*(a.y-b.y))
	}
	if math.Abs(area2) <= 1e-9*longest || area2 == 0 {
		return tr, false
	}
	if area2 < 0 {
		tr.v[1], tr.v[2] = tr.v[2], tr.v[1]
		tr.z[1], tr.z[2] = tr.z[2], tr.z[1]
	}
	tr.minX = math.Min(tr.v[0].x, math.Min(tr.v[1].x, tr.v[2].x))
	tr.maxX = math.Max(tr.v[0].x, math.Max(tr.v[1].x, tr.v[2].x))
	tr.minY = math.Min(tr.v[0].y, math.Min(tr.v[1].y, tr.v[2].y))
	tr.maxY = math.Max(tr.v[0].y, math.Max(tr.v[1].y, tr.v[2].y))
	tr.minZ = math.Min(tr.z[0], math.Min(tr.z[1], tr.z[2]))
	tr.maxZ = math.Max(tr.z[0], math.Max(tr.z[1], tr.z[2]))
	return tr, true
}

// Instance is a projection placed at a translation. Identical bodies at
// different positions share one Projection and differ only in Offset (a world
// translation in millimetres).
type Instance struct {
	P      *Projection
	Offset r3.Vec
}

// local returns the offset in projection coordinates (u, v, w).
func (in Instance) local() (du, dv, dw float64) {
	return in.Offset.Dot(in.P.u), in.Offset.Dot(in.P.v), in.Offset.Dot(in.P.dir)
}

// Span returns the lowest and highest position of the instance along the
// direction.
func (in Instance) Span() (lo, hi units.Value) {
	_, _, dw := in.local()
	return units.Millimeters(in.P.minW + dw), units.Millimeters(in.P.maxW + dw)
}
