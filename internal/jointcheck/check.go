// Package jointcheck holds the plain-data readings the structural joint check
// of agent-docs/parts-design.md §7.2 is built from: box gaps, interval
// verdicts, line comparisons and the overlap witness of two planar faces. It
// touches no body.
package jointcheck

import (
	"math"

	"github.com/lestrrat-3d/r3"
)

// Box is an axis-aligned box.
type Box struct{ Min, Max r3.Vec }

// Grow returns b grown by g on every side.
func (b Box) Grow(g float64) Box {
	d := r3.Vec{X: g, Y: g, Z: g}
	return Box{Min: b.Min.Sub(d), Max: b.Max.Add(d)}
}

// Meets reports whether two boxes touch or overlap.
func (b Box) Meets(o Box) bool {
	return b.Min.X <= o.Max.X && o.Min.X <= b.Max.X &&
		b.Min.Y <= o.Max.Y && o.Min.Y <= b.Max.Y &&
		b.Min.Z <= o.Max.Z && o.Min.Z <= b.Max.Z
}

// Overlap returns the box two boxes share, and false when they do not meet.
func (b Box) Overlap(o Box) (Box, bool) {
	if !b.Meets(o) {
		return Box{}, false
	}
	return Box{
		Min: r3.Vec{X: math.Max(b.Min.X, o.Min.X), Y: math.Max(b.Min.Y, o.Min.Y), Z: math.Max(b.Min.Z, o.Min.Z)},
		Max: r3.Vec{X: math.Min(b.Max.X, o.Max.X), Y: math.Min(b.Max.Y, o.Max.Y), Z: math.Min(b.Max.Z, o.Max.Z)},
	}, true
}

// GapLow is the lower end of the gap between two bodies lying inside boxes a and
// b: the greatest separation along one axis, less slack (the boxes' bounds),
// and zero when the boxes meet. It is sound because a body lies inside its box.
func GapLow(a, b Box, slack float64) float64 {
	gap := math.Max(0, math.Max(
		math.Max(a.Min.X-b.Max.X, b.Min.X-a.Max.X),
		math.Max(math.Max(a.Min.Y-b.Max.Y, b.Min.Y-a.Max.Y), math.Max(a.Min.Z-b.Max.Z, b.Min.Z-a.Max.Z))))
	return math.Max(0, gap-slack)
}

// Verdict is how a reading with a bound stands against an allowed interval.
type Verdict int

const (
	Inside   Verdict = iota // the whole reading lies inside the interval
	Outside                 // the whole reading lies outside it
	Straddle                // the reading crosses an end of the interval
)

// Within judges the reading [m − b, m + b] against [lo, hi] (G4).
func Within(m, b, lo, hi float64) Verdict {
	switch {
	case m-b >= lo && m+b <= hi:
		return Inside
	case m+b < lo || m-b > hi:
		return Outside
	}
	return Straddle
}

// Line is a point and a unit direction.
type Line struct{ Point, Dir r3.Vec }

// LineOffset returns the angle in radians between two lines' directions, taken
// as undirected, and the distance between them measured from l's point to m's
// line.
func LineOffset(l, m Line) (float64, float64) {
	c := math.Abs(l.Dir.Dot(m.Dir))
	angle := math.Acos(math.Min(1, c))
	d := l.Point.Sub(m.Point).Cross(m.Dir).Len()
	return angle, d
}

// OverlapWitness looks for proof that two planar faces, given as mesh triangles,
// share area. n is the plane's unit normal; a witness is the centroid of a
// triangle of one face that lies inside a triangle of the other in projection
// onto the plane, farther than margin from that triangle's edges, tried both
// ways round. It reports the witness point (on the first face's plane) and
// whether one was found. Finding none proves nothing.
func OverlapWitness(a, b [][3]r3.Vec, n r3.Vec, margin float64) (r3.Vec, bool) {
	eu, ev := planeBasis(n)
	if p, ok := centroidInside(a, b, eu, ev, margin); ok {
		return p, true
	}
	return centroidInside(b, a, eu, ev, margin)
}

func planeBasis(n r3.Vec) (r3.Vec, r3.Vec) {
	ref := r3.Vec{X: 1}
	if math.Abs(n.X) > 0.9 {
		ref = r3.Vec{Y: 1}
	}
	eu, _ := n.Cross(ref).Normalize()
	return eu, n.Cross(eu)
}

type tri2 struct {
	p          [3][2]float64
	minU, maxU float64
	minV, maxV float64
}

func project(ts [][3]r3.Vec, eu, ev r3.Vec) []tri2 {
	out := make([]tri2, 0, len(ts))
	for _, t := range ts {
		var q tri2
		q.minU, q.minV = math.Inf(1), math.Inf(1)
		q.maxU, q.maxV = math.Inf(-1), math.Inf(-1)
		for i, v := range t {
			u, w := v.Dot(eu), v.Dot(ev)
			q.p[i] = [2]float64{u, w}
			q.minU, q.maxU = math.Min(q.minU, u), math.Max(q.maxU, u)
			q.minV, q.maxV = math.Min(q.minV, w), math.Max(q.maxV, w)
		}
		out = append(out, q)
	}
	return out
}

func centroidInside(a, b [][3]r3.Vec, eu, ev r3.Vec, margin float64) (r3.Vec, bool) {
	pa, pb := project(a, eu, ev), project(b, eu, ev)
	for i, t := range pa {
		cu := (t.p[0][0] + t.p[1][0] + t.p[2][0]) / 3
		cv := (t.p[0][1] + t.p[1][1] + t.p[2][1]) / 3
		for _, o := range pb {
			if cu < o.minU+margin || cu > o.maxU-margin || cv < o.minV+margin || cv > o.maxV-margin {
				continue
			}
			if insideBy(o, cu, cv, margin) {
				c := a[i][0].Add(a[i][1]).Add(a[i][2]).Scale(1.0 / 3)
				return c, true
			}
		}
	}
	return r3.Vec{}, false
}

// insideBy reports whether (u, v) lies inside triangle t, farther than margin
// from each edge.
func insideBy(t tri2, u, v, margin float64) bool {
	sign := 0.0
	for i := range 3 {
		a, b := t.p[i], t.p[(i+1)%3]
		ex, ey := b[0]-a[0], b[1]-a[1]
		length := math.Hypot(ex, ey)
		if length == 0 {
			return false
		}
		d := (ex*(v-a[1]) - ey*(u-a[0])) / length // signed distance from the edge's line
		if sign == 0 {
			sign = math.Copysign(1, d)
			if d == 0 {
				return false
			}
		}
		if d*sign <= margin {
			return false
		}
	}
	return true
}
