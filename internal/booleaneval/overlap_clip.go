package booleaneval

import (
	"math"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"

	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/topo"
)

// clipOverlapDiameter estimates the diameter of the part of a Cut target's
// boundary that lies in a prism tool's own box: the tool's section box over
// its levels, in the tool's frame. Only there can the two boundaries meet and
// a rim form. The target's boundary is its held mesh when it holds one, else
// its own box's faces. Each facet is clipped to the tool's box in the tool's
// coordinates and the clipped points' box measured, so a straight tool that
// crosses a long diagonal bar reads the short crossing, not the boxes'
// common part along the bar. It is a chord choice only, read in floats: the
// meshes chorded by it publish the bounds they take, and a Cut refused at this
// chord is run again at the pair's own (booleandispatch.BodyScaled). It
// answers false when the tool is not a prism or nothing of the target's
// boundary lies in the tool's box.
func clipOverlapDiameter(a, b *topo.Body) (float64, bool) {
	tool, ok := b.Payload.(prismrecord.Payload)
	if !ok {
		return 0, false
	}
	lo, hi, ok := sectionBox(tool.Profile.Outer)
	if !ok {
		return 0, false
	}
	z0, z1 := min(tool.Z0, tool.Z1), max(tool.Z0, tool.Z1)
	inv, err := tool.Xform.Inverse()
	if err != nil {
		return 0, false
	}
	o, u, v, n := tool.Frame.Origin(), tool.Frame.U(), tool.Frame.V(), tool.Frame.N()
	local := func(p r3.Vec) r3.Vec {
		q := inv.Apply(p).Sub(o)
		return r3.Vec{X: q.Dot(u), Y: q.Dot(v), Z: q.Dot(n)}
	}
	boxLo, boxHi := r3.Vec{X: lo[0], Y: lo[1], Z: z0}, r3.Vec{X: hi[0], Y: hi[1], Z: z1}
	fp, isF := a.Payload.(facetedrecord.Payload)
	useFacets := isF && len(fp.Tris) > 0
	var boxTris [][3]r3.Vec
	triangleCount := len(fp.Tris)
	if !useFacets {
		boxTris = boxFaces(a.Bounds.Min, a.Bounds.Max)
		triangleCount = len(boxTris)
	}
	seenLo := r3.Vec{X: math.Inf(1), Y: math.Inf(1), Z: math.Inf(1)}
	seenHi := r3.Vec{X: math.Inf(-1), Y: math.Inf(-1), Z: math.Inf(-1)}
	seen := false
	for i := range triangleCount {
		var t [3]r3.Vec
		if useFacets {
			indices := fp.Tris[i]
			t = [3]r3.Vec{fp.Verts[indices[0]], fp.Verts[indices[1]], fp.Verts[indices[2]]}
		} else {
			t = boxTris[i]
		}
		poly := []r3.Vec{local(t[0]), local(t[1]), local(t[2])}
		for axis := range 3 {
			poly = clipAxis(poly, axis, coord(boxLo, axis), true)
			poly = clipAxis(poly, axis, coord(boxHi, axis), false)
		}
		for _, p := range poly {
			seen = true
			seenLo = r3.Vec{X: math.Min(seenLo.X, p.X), Y: math.Min(seenLo.Y, p.Y), Z: math.Min(seenLo.Z, p.Z)}
			seenHi = r3.Vec{X: math.Max(seenHi.X, p.X), Y: math.Max(seenHi.Y, p.Y), Z: math.Max(seenHi.Z, p.Z)}
		}
	}
	if !seen {
		return 0, false
	}
	d := seenHi.Sub(seenLo).Len()
	if !(d > 0) || math.IsInf(d, 0) || math.IsNaN(d) {
		return 0, false
	}
	return d, true
}

func coord(v r3.Vec, axis int) float64 {
	switch axis {
	case 0:
		return v.X
	case 1:
		return v.Y
	}
	return v.Z
}

// clipAxis keeps the part of a convex polygon on one side of the plane
// coordinate(axis) = c: above it when keepAbove, else below.
func clipAxis(poly []r3.Vec, axis int, c float64, keepAbove bool) []r3.Vec {
	if len(poly) == 0 {
		return nil
	}
	inside := func(p r3.Vec) bool {
		if keepAbove {
			return coord(p, axis) >= c
		}
		return coord(p, axis) <= c
	}
	var out []r3.Vec
	for i, p := range poly {
		q := poly[(i+1)%len(poly)]
		pin, qin := inside(p), inside(q)
		if pin {
			out = append(out, p)
		}
		if pin != qin {
			t := (c - coord(p, axis)) / (coord(q, axis) - coord(p, axis))
			out = append(out, p.Add(q.Sub(p).Scale(t)))
		}
	}
	return out
}

// boxFaces is the twelve triangles of the box lo-hi.
func boxFaces(lo, hi r3.Vec) [][3]r3.Vec {
	c := func(i int) r3.Vec {
		v := lo
		if i&1 != 0 {
			v.X = hi.X
		}
		if i&2 != 0 {
			v.Y = hi.Y
		}
		if i&4 != 0 {
			v.Z = hi.Z
		}
		return v
	}
	quads := [][4]int{{0, 1, 3, 2}, {4, 5, 7, 6}, {0, 1, 5, 4}, {2, 3, 7, 6}, {0, 2, 6, 4}, {1, 3, 7, 5}}
	var out [][3]r3.Vec
	for _, q := range quads {
		out = append(out, [3]r3.Vec{c(q[0]), c(q[1]), c(q[2])}, [3]r3.Vec{c(q[0]), c(q[2]), c(q[3])})
	}
	return out
}

// sectionBox bounds a section loop in its own coordinates: lines by their
// ends, circles and arcs by their whole circle.
func sectionBox(loop sectionrecord.LoopRecord) ([2]float64, [2]float64, bool) {
	lo := [2]float64{math.Inf(1), math.Inf(1)}
	hi := [2]float64{math.Inf(-1), math.Inf(-1)}
	add := func(u, v float64) {
		lo = [2]float64{math.Min(lo[0], u), math.Min(lo[1], v)}
		hi = [2]float64{math.Max(hi[0], u), math.Max(hi[1], v)}
	}
	for _, seg := range loop.Segments {
		switch s := seg.(type) {
		case sectionrecord.LineSeg:
			add(s.Start.U, s.Start.V)
			add(s.End.U, s.End.V)
		case sectionrecord.CircleSeg:
			r, err := s.Radius.In(units.Millimeter)
			if err != nil {
				return lo, hi, false
			}
			add(s.Center.U-r, s.Center.V-r)
			add(s.Center.U+r, s.Center.V+r)
		case sectionrecord.ArcSeg:
			r := math.Hypot(s.Start.U-s.Center.U, s.Start.V-s.Center.V)
			add(s.Center.U-r, s.Center.V-r)
			add(s.Center.U+r, s.Center.V+r)
		default:
			return lo, hi, false
		}
	}
	return lo, hi, lo[0] <= hi[0] && lo[1] <= hi[1]
}
