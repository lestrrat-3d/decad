// Package jointinfer matches the cylinder and plane faces of two parts into
// pin-in-bore, pin-in-slot and face-contact joints (agent-docs/parts-design.md
// §7.1). It works on plain millimetre records and touches no body.
package jointinfer

import (
	"cmp"
	"math"
	"slices"

	"github.com/lestrrat-3d/r3"
)

// Params are the matching tolerances. Lengths are millimetres, AxisTol is
// radians.
type Params struct {
	MaxFit  float64 // largest clearance matched
	FitTol  float64 // largest axis distance, and the radius agreement within one match
	AxisTol float64 // largest angle between axes that count as parallel
}

// Cylinder is one cylindrical face. Axis is a unit direction, Lo and Hi are the
// face's extent along Axis measured from Origin, and Bound is the reading
// bound of the face's tag in millimetres, charged to every test it takes part
// in.
type Cylinder struct {
	Origin, Axis r3.Vec
	Radius       float64
	Convex       bool
	Lo, Hi       float64
	Bound        float64
}

// Plane is one planar face. Normal is the outward unit normal, Offset is
// Normal·p for a point p of the plane, and Min and Max are the box of the
// face's mesh vertices.
type Plane struct {
	Normal   r3.Vec
	Offset   float64
	Min, Max r3.Vec
	Bound    float64
}

// Line is a point on a line and a unit direction.
type Line struct{ Point, Dir r3.Vec }

// BoreMatch is a convex cylinder group inside a coaxial concave group. Lo and
// Hi bound the engagement along Axis.Dir measured from Axis.Point's origin
// projection: Axis.Point sits at the engagement midpoint.
type BoreMatch struct {
	Pin, Bore []int
	Axis      Line
	Length    float64 // Hi - Lo of the axial overlap
	Clearance float64 // bore radius less pin radius
	Bound     float64
}

// SlotMatch is a convex cylinder between two opposed parallel planes.
type SlotMatch struct {
	Pin       int
	Walls     [2]int
	Axis      Line
	Length    float64
	Clearance float64 // sum of the wall distances less the pin diameter
	Bound     float64
}

// FaceMatch is a pair of opposed coplanar planes that overlap.
type FaceMatch struct {
	A, B   int
	Normal r3.Vec // A's outward normal: it points into B
	Centre r3.Vec // centre of the overlap, on A's plane
	Area   float64
	Gap    float64
}

const minOverlap = 1e-9

func (p Params) sinTol(extra float64) float64 { return math.Sin(math.Min(p.AxisTol+extra, math.Pi/2)) }

// MatchBore returns the pin-in-bore matches of pins (convex faces of one part)
// and bores (concave faces of the other), longest engagement first, then
// smallest clearance, then input order.
func MatchBore(pins, bores []Cylinder, p Params) []BoreMatch {
	var out []BoreMatch
	for i, a := range pins {
		if !a.Convex {
			continue
		}
		for j, b := range bores {
			if b.Convex {
				continue
			}
			m, ok := boreOverlap(a, b, p)
			if !ok {
				continue
			}
			m.Pin, m.Bore = []int{i}, []int{j}
			merged := false
			for k := range out {
				if sameBore(out[k], m, pins, bores, p) {
					out[k] = mergeBore(out[k], m)
					merged = true
					break
				}
			}
			if !merged {
				out = append(out, m)
			}
		}
	}
	slices.SortStableFunc(out, func(x, y BoreMatch) int {
		if c := cmp.Compare(y.Length, x.Length); c != 0 {
			return c
		}
		return cmp.Compare(x.Clearance, y.Clearance)
	})
	return out
}

func boreOverlap(a, b Cylinder, p Params) (BoreMatch, bool) {
	bound := a.Bound + b.Bound
	if a.Axis.Cross(b.Axis).Len() > p.sinTol(bound) {
		return BoreMatch{}, false
	}
	d := b.Origin.Sub(a.Origin)
	dist := d.Cross(a.Axis).Len()
	if dist > p.FitTol+bound {
		return BoreMatch{}, false
	}
	fit := b.Radius - a.Radius
	if fit > p.MaxFit+bound || fit < -p.MaxFit-bound {
		return BoreMatch{}, false
	}
	s0 := d.Dot(a.Axis)
	bLo, bHi := s0+b.Lo, s0+b.Hi
	if b.Axis.Dot(a.Axis) < 0 {
		bLo, bHi = s0-b.Hi, s0-b.Lo
	}
	lo, hi := math.Max(a.Lo, bLo), math.Min(a.Hi, bHi)
	if hi-lo <= minOverlap {
		return BoreMatch{}, false
	}
	mid := a.Origin.Add(a.Axis.Scale((lo + hi) / 2))
	return BoreMatch{Axis: Line{Point: mid, Dir: a.Axis}, Length: hi - lo, Clearance: fit,
		Bound: bound + dist}, true
}

// sameBore reports whether m lies on the same cylinder pair as g: the same
// line, and radii that agree, so two half-faces of one cylinder join.
func sameBore(g, m BoreMatch, pins, bores []Cylinder, p Params) bool {
	if math.Abs(g.Clearance-m.Clearance) > p.FitTol/100 {
		return false
	}
	if g.Axis.Dir.Cross(m.Axis.Dir).Len() > p.sinTol(0) {
		return false
	}
	if m.Axis.Point.Sub(g.Axis.Point).Cross(g.Axis.Dir).Len() > p.FitTol {
		return false
	}
	return math.Abs(pins[g.Pin[0]].Radius-pins[m.Pin[0]].Radius) <= p.FitTol/100 &&
		math.Abs(bores[g.Bore[0]].Radius-bores[m.Bore[0]].Radius) <= p.FitTol/100
}

func mergeBore(g, m BoreMatch) BoreMatch {
	// Axial hull of the two engagements, measured along g's direction.
	gLo := -g.Length / 2
	gHi := g.Length / 2
	off := m.Axis.Point.Sub(g.Axis.Point).Dot(g.Axis.Dir)
	mLo, mHi := off-m.Length/2, off+m.Length/2
	lo, hi := math.Min(gLo, mLo), math.Max(gHi, mHi)
	g.Pin = appendNew(g.Pin, m.Pin[0])
	g.Bore = appendNew(g.Bore, m.Bore[0])
	g.Axis.Point = g.Axis.Point.Add(g.Axis.Dir.Scale((lo + hi) / 2))
	g.Length = hi - lo
	g.Bound = math.Max(g.Bound, m.Bound)
	return g
}

func appendNew(s []int, v int) []int {
	if slices.Contains(s, v) {
		return s
	}
	return append(s, v)
}

// MatchSlot returns the pin-in-slot matches of pins between walls, longest
// engagement first, then smallest clearance, then input order.
func MatchSlot(pins []Cylinder, walls []Plane, p Params) []SlotMatch {
	var out []SlotMatch
	for i, a := range pins {
		if !a.Convex {
			continue
		}
		for j := range walls {
			for k := j + 1; k < len(walls); k++ {
				if m, ok := slotMatch(a, walls[j], walls[k], p); ok {
					m.Pin, m.Walls = i, [2]int{j, k}
					out = append(out, m)
				}
			}
		}
	}
	slices.SortStableFunc(out, func(x, y SlotMatch) int {
		if c := cmp.Compare(y.Length, x.Length); c != 0 {
			return c
		}
		return cmp.Compare(x.Clearance, y.Clearance)
	})
	return out
}

func slotMatch(a Cylinder, w1, w2 Plane, p Params) (SlotMatch, bool) {
	bound := a.Bound + w1.Bound + w2.Bound
	s := p.sinTol(bound)
	if math.Abs(w1.Normal.Dot(a.Axis)) > s || math.Abs(w2.Normal.Dot(a.Axis)) > s {
		return SlotMatch{}, false
	}
	if w1.Normal.Add(w2.Normal).Len() > 2*s {
		return SlotMatch{}, false
	}
	d1 := w1.Normal.Dot(a.Origin) - w1.Offset
	d2 := w2.Normal.Dot(a.Origin) - w2.Offset
	if d1 <= 0 || d2 <= 0 {
		return SlotMatch{}, false
	}
	fit := d1 + d2 - 2*a.Radius
	if fit > p.MaxFit+bound || fit < -p.MaxFit-bound {
		return SlotMatch{}, false
	}
	lo, hi := a.Lo, a.Hi
	for _, w := range []Plane{w1, w2} {
		wLo, wHi := boxExtent(w, a)
		lo, hi = math.Max(lo, wLo), math.Min(hi, wHi)
	}
	if hi-lo <= minOverlap {
		return SlotMatch{}, false
	}
	mid := a.Origin.Add(a.Axis.Scale((lo + hi) / 2))
	return SlotMatch{Axis: Line{Point: mid, Dir: a.Axis}, Length: hi - lo, Clearance: fit,
		Bound: bound}, true
}

// boxExtent is the extent of the plane's vertex box along the cylinder's axis,
// measured from the cylinder's origin.
func boxExtent(w Plane, a Cylinder) (float64, float64) {
	lo, hi := math.Inf(1), math.Inf(-1)
	for i := range 8 {
		c := r3.Vec{X: pick(i&1, w.Min.X, w.Max.X), Y: pick(i&2, w.Min.Y, w.Max.Y), Z: pick(i&4, w.Min.Z, w.Max.Z)}
		s := c.Sub(a.Origin).Dot(a.Axis)
		lo, hi = math.Min(lo, s), math.Max(hi, s)
	}
	return lo, hi
}

func pick(bit int, lo, hi float64) float64 {
	if bit == 0 {
		return lo
	}
	return hi
}

// MatchFace returns the overlapping opposed coplanar pairs of as and bs,
// largest overlap first. The overlap is the rectangle common to the two faces'
// vertex boxes projected onto the plane, so it never under-reports the shared
// area.
func MatchFace(as, bs []Plane, p Params) []FaceMatch {
	var out []FaceMatch
	for i, a := range as {
		for j, b := range bs {
			if m, ok := faceMatch(a, b, p); ok {
				m.A, m.B = i, j
				out = append(out, m)
			}
		}
	}
	slices.SortStableFunc(out, func(x, y FaceMatch) int { return cmp.Compare(y.Area, x.Area) })
	return out
}

func faceMatch(a, b Plane, p Params) (FaceMatch, bool) {
	bound := a.Bound + b.Bound
	if a.Normal.Add(b.Normal).Len() > 2*p.sinTol(bound) {
		return FaceMatch{}, false
	}
	// B's plane is Normal_B·x = Offset_B; with Normal_B = -Normal_A the plane
	// sits at Normal_A·x = -Offset_B.
	gap := math.Abs(a.Offset + b.Offset)
	if gap > p.MaxFit+bound {
		return FaceMatch{}, false
	}
	eu, ev := basis(a.Normal)
	au0, au1, av0, av1 := project(a, eu, ev)
	bu0, bu1, bv0, bv1 := project(b, eu, ev)
	u0, u1 := math.Max(au0, bu0), math.Min(au1, bu1)
	v0, v1 := math.Max(av0, bv0), math.Min(av1, bv1)
	if u1-u0 <= minOverlap || v1-v0 <= minOverlap {
		return FaceMatch{}, false
	}
	uc, vc := (u0+u1)/2, (v0+v1)/2
	centre := a.Normal.Scale(a.Offset).Add(eu.Scale(uc)).Add(ev.Scale(vc))
	return FaceMatch{Normal: a.Normal, Centre: centre, Area: (u1 - u0) * (v1 - v0), Gap: gap}, true
}

func basis(n r3.Vec) (r3.Vec, r3.Vec) {
	ref := r3.Vec{X: 1}
	if math.Abs(n.X) > 0.9 {
		ref = r3.Vec{Y: 1}
	}
	eu, _ := n.Cross(ref).Normalize()
	ev := n.Cross(eu)
	return eu, ev
}

func project(w Plane, eu, ev r3.Vec) (float64, float64, float64, float64) {
	u0, u1, v0, v1 := math.Inf(1), math.Inf(-1), math.Inf(1), math.Inf(-1)
	for i := range 8 {
		c := r3.Vec{X: pick(i&1, w.Min.X, w.Max.X), Y: pick(i&2, w.Min.Y, w.Max.Y), Z: pick(i&4, w.Min.Z, w.Max.Z)}
		u, v := c.Dot(eu), c.Dot(ev)
		u0, u1 = math.Min(u0, u), math.Max(u1, u)
		v0, v1 = math.Min(v0, v), math.Max(v1, v)
	}
	return u0, u1, v0, v1
}
