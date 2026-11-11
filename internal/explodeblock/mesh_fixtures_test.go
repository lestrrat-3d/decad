package explodeblock_test

import (
	"math"

	"github.com/lestrrat-3d/decad/internal/explodeblock"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

const sides = 48

var (
	up   = r3.NewVec(0, 0, 1)
	down = r3.NewVec(0, 0, -1)
)

// revolve builds the closed boundary of a solid of revolution about Z from a
// closed (radius, z) profile. Triangles collapsed on the axis are left in; the
// projection drops them.
func revolve(profile [][2]float64, bound float64) explodeblock.Mesh {
	var vs []r3.Vec
	for _, p := range profile {
		for k := range sides {
			a := 2 * math.Pi * float64(k) / sides
			vs = append(vs, r3.NewVec(p[0]*math.Cos(a), p[0]*math.Sin(a), p[1]))
		}
	}
	var ts [][3]int
	np := len(profile)
	for i := range np {
		j := (i + 1) % np
		for k := range sides {
			k2 := (k + 1) % sides
			a, a2, b, b2 := i*sides+k, i*sides+k2, j*sides+k, j*sides+k2
			ts = append(ts, [3]int{a, b, b2}, [3]int{a, b2, a2})
		}
	}
	return explodeblock.Mesh{Vertices: vs, Triangles: ts, Bound: units.Millimeters(bound)}
}

// shaftBottom is the lower end of every test shaft; boreRadius is the bore of
// every test bearing and neckRadius the neck of every collar shaft.
const (
	shaftBottom  = -6.0
	boreRadius   = 2.5
	neckRadius   = 2.4
	collarRadius = 4.95
	collarTop    = -2.0
	shaftTop     = 12.0
	ringOuter    = 8.0
	ringBottom   = 0.0
	ringTop      = 4.0
)

// shaft is a plain cylinder of radius r from the bottom to z1.
func shaft(r, z1, bound float64) explodeblock.Mesh {
	return revolve([][2]float64{{0, shaftBottom}, {r, shaftBottom}, {r, z1}, {0, z1}}, bound)
}

// collarShaft has a neck up to shaftTop and a collar from the bottom up to
// collarTop.
func collarShaft() explodeblock.Mesh {
	return revolve([][2]float64{
		{0, shaftBottom}, {collarRadius, shaftBottom}, {collarRadius, collarTop},
		{neckRadius, collarTop}, {neckRadius, shaftTop}, {0, shaftTop},
	}, 0.02)
}

// ring is the test bearing: an annulus with the standard bore, from ringBottom
// to ringTop.
func ring(bound float64) explodeblock.Mesh {
	return revolve([][2]float64{{boreRadius, ringBottom}, {ringOuter, ringBottom}, {ringOuter, ringTop}, {boreRadius, ringTop}}, bound)
}

// box is an axis-aligned box.
func box(lo, hi r3.Vec) explodeblock.Mesh {
	var vs []r3.Vec
	for i := range 8 {
		p := lo
		if i&1 != 0 {
			p.X = hi.X
		}
		if i&2 != 0 {
			p.Y = hi.Y
		}
		if i&4 != 0 {
			p.Z = hi.Z
		}
		vs = append(vs, p)
	}
	quads := [][4]int{{0, 1, 3, 2}, {4, 5, 7, 6}, {0, 1, 5, 4}, {2, 3, 7, 6}, {0, 2, 6, 4}, {1, 3, 7, 5}}
	var ts [][3]int
	for _, q := range quads {
		ts = append(ts, [3]int{q[0], q[1], q[2]}, [3]int{q[0], q[2], q[3]})
	}
	return explodeblock.Mesh{Vertices: vs, Triangles: ts}
}

// tri is a single open triangle, enough for a face-level test.
func tri(a, b, c r3.Vec) explodeblock.Mesh {
	return explodeblock.Mesh{Vertices: []r3.Vec{a, b, c}, Triangles: [][3]int{{0, 1, 2}}}
}
