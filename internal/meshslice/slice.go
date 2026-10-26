// Package meshslice cuts a triangle mesh with a plane and chains the cut into
// closed outlines (agent-docs/section-outline-design.md). It works on plain
// millimetre data and touches no body or document.
package meshslice

import (
	"cmp"
	"context"
	"errors"
	"fmt"
	"math"
	"slices"

	"github.com/lestrrat-3d/r3"
)

// ErrOpen marks a mesh whose cut does not close into outlines: a boundary
// edge, a non-manifold edge or a triangle that winds against its neighbour.
var ErrOpen = errors.New("meshslice: the mesh cut does not close")

// edgeKey names a mesh edge by its vertex indices, smaller first, so both
// triangles that share the edge name it the same way.
type edgeKey [2]int

func keyOf(a, b int) edgeKey {
	if a > b {
		a, b = b, a
	}
	return edgeKey{a, b}
}

type segment struct{ from, to edgeKey }

// Outline is one closed polyline of the cut, in world coordinates. The last
// point joins the first.
type Outline struct {
	Points []r3.Vec
}

// Slice cuts the triangles with the plane through origin whose normal is
// normal, a nonzero vector that need not be a unit vector. A vertex on the plane counts as on the normal's side, as if the plane
// were moved a hair against the normal, so a cut never passes exactly through a
// vertex and every outline closes. Each outline runs counter-clockwise seen
// from the normal's tip around material that lies inside it, when the
// triangles wind counter-clockwise seen from outside the material: an outer
// outline is counter-clockwise and the outline of a hole clockwise.
//
// A mesh whose cut leaves a loose end is ErrOpen. verts must be finite and
// every triangle index in range, which Slice does not recheck.
func Slice(ctx context.Context, verts []r3.Vec, tris [][3]int, origin, normal r3.Vec) ([]Outline, error) {
	above := make([]bool, len(verts))
	dist := make([]float64, len(verts))
	for i, v := range verts {
		dist[i] = normal.Dot(v.Sub(origin))
		above[i] = dist[i] >= 0
	}
	point := func(k edgeKey) r3.Vec {
		a, b := verts[k[0]], verts[k[1]]
		t := dist[k[0]] / (dist[k[0]] - dist[k[1]])
		return a.Add(b.Sub(a).Scale(t))
	}
	segments := make(map[edgeKey]segment)
	for ti, tri := range tris {
		if ti%4096 == 0 {
			if err := ctx.Err(); err != nil {
				return nil, err
			}
		}
		// The plane crosses a triangle on two edges or none. Walking the triangle
		// counter-clockwise seen from outside, one crossing goes from the far side
		// of the plane to the normal's side (up) and the other comes back (down).
		// The outline, with material on its left seen from the normal's tip, runs
		// from the down crossing to the up crossing.
		var up, down edgeKey
		crossings := 0
		for k := range tri {
			a, b := tri[k], tri[(k+1)%3]
			if above[a] == above[b] {
				continue
			}
			crossings++
			if above[b] {
				up = keyOf(a, b)
			} else {
				down = keyOf(a, b)
			}
		}
		if crossings != 2 {
			continue
		}
		if _, dup := segments[down]; dup {
			return nil, fmt.Errorf("%w: an edge starts two segments", ErrOpen)
		}
		segments[down] = segment{from: down, to: up}
	}
	// Start each outline at the smallest unused edge key, so the result does not
	// depend on map order.
	keys := make([]edgeKey, 0, len(segments))
	for k := range segments {
		keys = append(keys, k)
	}
	slices.SortFunc(keys, func(a, b edgeKey) int {
		if c := cmp.Compare(a[0], b[0]); c != 0 {
			return c
		}
		return cmp.Compare(a[1], b[1])
	})
	var outlines []Outline
	for _, start := range keys {
		if _, unused := segments[start]; !unused {
			continue
		}
		var points []r3.Vec
		at := start
		for {
			s, ok := segments[at]
			if !ok {
				return nil, fmt.Errorf("%w: an outline stops at an edge no segment leaves", ErrOpen)
			}
			delete(segments, at)
			points = append(points, point(at))
			at = s.to
			if at == start {
				break
			}
		}
		outlines = append(outlines, Outline{Points: points})
	}
	return outlines, nil
}

// Area is the signed area of the outline seen from the normal's tip: positive
// for a counter-clockwise outline. It is the shoelace sum in the plane's
// frame, so it needs the plane's unit normal.
func (o Outline) Area(normal r3.Vec) float64 {
	var twice r3.Vec
	for i, p := range o.Points {
		q := o.Points[(i+1)%len(o.Points)]
		twice = twice.Add(p.Cross(q))
	}
	return twice.Dot(normal) / 2
}

// MaxAbs returns the largest absolute coordinate among the points of the
// outlines, or 0 for none.
func MaxAbs(outlines []Outline) float64 {
	m := 0.0
	for _, o := range outlines {
		for _, p := range o.Points {
			m = math.Max(m, math.Max(math.Abs(p.X), math.Max(math.Abs(p.Y), math.Abs(p.Z))))
		}
	}
	return m
}
