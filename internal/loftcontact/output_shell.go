package loftcontact

import (
	"math"
	"math/big"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// OutputPoint is an exact plane-local point (U, V) at height Z in the prism
// frame. All coordinates here are exact rationals over authenticated source
// floats. The output certificate and its source/held pairing die with the call.
type OutputPoint struct{ U, V, Z *big.Rat }

// OutputFace labels the source patch of an output triangle. Kind is 0 for a
// cap and 1 for a side wall; Owner is 0 for the prism and 1 for the Loft.
type OutputFace struct {
	Kind, Owner, Segment, Level int
	Cell                        int
}

// OutputVertex pairs an exact shell point with its exact world point, the
// held float point and the bound on their distance.
type OutputVertex struct {
	Point      OutputPoint
	World      [3]*big.Rat
	Held       r3.Vec
	Bound      float64
	Incidences map[OutputFace]struct{}
}

// OutputShell is the transient exact Loft/prism output shell. Build it with
// NewOutputShell and add triangles with Triangle.
type OutputShell struct {
	Vertices       []OutputVertex
	Triangles      [][3]int
	Faces          []OutputFace
	TriangleBounds []float64
	index          map[string]int
	heldIndex      map[[3]uint64]string
	MaxBound       float64
}

// ReplayArithmetic returns the first optional arithmetic state, or a fresh one.
func ReplayArithmetic(optional []*Arithmetic) *Arithmetic {
	if len(optional) != 0 {
		return optional[0]
	}
	return &Arithmetic{OK: true}
}

// OutputP2 places the plane point p at height z.
func OutputP2(p Point, z *big.Rat) OutputPoint {
	return OutputPoint{p.U, p.V, z}
}

// OutputPointKey returns the exact text key of p.
func OutputPointKey(p OutputPoint) string {
	return p.U.RatString() + "," + p.V.RatString() + "," + p.Z.RatString()
}

// OutputSamePoint reports whether a and b are the same exact point.
func OutputSamePoint(a, b OutputPoint) bool {
	arithmetic := &Arithmetic{OK: true}
	same := arithmetic.Cmp(a.U, b.U) == 0 && arithmetic.Cmp(a.V, b.V) == 0 &&
		arithmetic.Cmp(a.Z, b.Z) == 0
	return arithmetic.OK && same
}

// OutputWorld maps p through frame exactly and returns the exact world point,
// the nearest held point and an upper bound on their distance.
func OutputWorld(frame r3.Frame, p OutputPoint) ([3]*big.Rat, r3.Vec, float64, bool) {
	var exact [3]*big.Rat
	c := &Arithmetic{OK: true}
	if !c.Valid(p.U) || !c.Valid(p.V) || !c.Valid(p.Z) {
		return exact, r3.Vec{}, 0, false
	}
	o, u, v, n := frame.Origin(), frame.U(), frame.V(), frame.N()
	if !proofbound.FiniteVec(o) || !proofbound.FiniteVec(u) ||
		!proofbound.FiniteVec(v) || !proofbound.FiniteVec(n) {
		return exact, r3.Vec{}, 0, false
	}
	coords := [4][3]float64{{o.X, o.Y, o.Z}, {u.X, u.Y, u.Z},
		{v.X, v.Y, v.Z}, {n.X, n.Y, n.Z}}
	params := [3]*big.Rat{p.U, p.V, p.Z}
	var held [3]float64
	gap2 := new(big.Rat)
	for i := range exact {
		exact[i] = proofarith.FloatRat(coords[0][i])
		for j, param := range params {
			term := c.Mul(proofarith.FloatRat(coords[j+1][i]), param)
			exact[i] = c.Add(exact[i], term)
			if !c.OK {
				return exact, r3.Vec{}, 0, false
			}
		}
		held[i], _ = c.Float64(exact[i])
		if !c.OK || proofbound.IsNonFinite(held[i]) {
			return exact, r3.Vec{}, 0, false
		}
		delta := c.Sub(exact[i], proofarith.FloatRat(held[i]))
		gap2 = c.Add(gap2, c.Mul(delta, delta))
		if !c.OK {
			return exact, r3.Vec{}, 0, false
		}
	}
	// RatSqrtUp compares a finite float square with gap2 using up to 2,252
	// shift bits on its numerator or 2,048 on its denominator.
	if gap2.Num().BitLen() > RationalBits-2252 ||
		gap2.Denom().BitLen() > RationalBits-2048 {
		return exact, r3.Vec{}, 0, false
	}
	bound := proofbound.RatSqrtUp(gap2)
	if proofbound.IsNonFinite(bound) {
		return exact, r3.Vec{}, 0, false
	}
	return exact, r3.Vec{X: held[0], Y: held[1], Z: held[2]}, bound, true
}

func heldKey(v r3.Vec) [3]uint64 {
	coords := [3]float64{v.X, v.Y, v.Z}
	var key [3]uint64
	for i, x := range coords {
		if x != 0 {
			key[i] = math.Float64bits(x)
		}
	}
	return key
}

// Vertex returns the index of p, adding it with face as an incidence. It
// refuses a 513th vertex and a new point whose held coordinates repeat
// another point's.
func (s *OutputShell) Vertex(frame r3.Frame, p OutputPoint, face OutputFace) (int, bool) {
	c := &Arithmetic{OK: true}
	if !c.Valid(p.U) || !c.Valid(p.V) || !c.Valid(p.Z) {
		return 0, false
	}
	key := OutputPointKey(p)
	if index, exists := s.index[key]; exists {
		s.Vertices[index].Incidences[face] = struct{}{}
		return index, true
	}
	if len(s.Vertices) == 512 {
		return 0, false
	}
	world, held, bound, ok := OutputWorld(frame, p)
	if !ok {
		return 0, false
	}
	heldKey := heldKey(held)
	if prior, exists := s.heldIndex[heldKey]; exists && prior != key {
		return 0, false
	}
	index := len(s.Vertices)
	s.index[key], s.heldIndex[heldKey] = index, key
	s.Vertices = append(s.Vertices, OutputVertex{Point: p, World: world, Held: held,
		Bound: bound, Incidences: map[OutputFace]struct{}{face: {}}})
	s.MaxBound = max(s.MaxBound, bound)
	return index, true
}

// Triangle adds one triangle labelled face. It refuses a 513th triangle, a
// refused vertex and a degenerate triangle.
func (s *OutputShell) Triangle(frame r3.Frame, points [3]OutputPoint,
	face OutputFace) bool {
	if len(s.Triangles) == 512 {
		return false
	}
	var ids [3]int
	for i, p := range points {
		var ok bool
		ids[i], ok = s.Vertex(frame, p, face)
		if !ok {
			return false
		}
	}
	if ids[0] == ids[1] || ids[1] == ids[2] || ids[2] == ids[0] ||
		OutputCrossZero(points) {
		return false
	}
	s.Triangles = append(s.Triangles, ids)
	s.Faces = append(s.Faces, face)
	s.TriangleBounds = append(s.TriangleBounds, max(s.Vertices[ids[0]].Bound,
		s.Vertices[ids[1]].Bound, s.Vertices[ids[2]].Bound))
	return true
}

// OutputCrossZero reports whether the three points are collinear, or whether
// the arithmetic refused.
func OutputCrossZero(p [3]OutputPoint) bool {
	c := &Arithmetic{OK: true}
	a := [3]*big.Rat{c.Sub(p[1].U, p[0].U),
		c.Sub(p[1].V, p[0].V), c.Sub(p[1].Z, p[0].Z)}
	b := [3]*big.Rat{c.Sub(p[2].U, p[0].U),
		c.Sub(p[2].V, p[0].V), c.Sub(p[2].Z, p[0].Z)}
	if !c.OK {
		return true
	}
	for i := range a {
		j, k := (i+1)%3, (i+2)%3
		cross := c.Sub(c.Mul(a[j], b[k]), c.Mul(a[k], b[j]))
		if !c.OK {
			return true
		}
		if cross.Sign() != 0 {
			return false
		}
	}
	return true
}

// NewOutputShell returns an empty shell.
func NewOutputShell() OutputShell {
	return OutputShell{index: make(map[string]int), heldIndex: make(map[[3]uint64]string)}
}
