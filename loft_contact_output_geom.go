package decad

import (
	"math"
	"math/big"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// All coordinates here are exact rationals over authenticated source floats.
// The output certificate and its source/held pairing die with the call.
type loftOutputPoint struct{ u, v, z *big.Rat }

type loftOutputFace struct {
	kind, owner, segment, level int
	cell                        int
}

type loftOutputVertex struct {
	point      loftOutputPoint
	world      [3]*big.Rat
	held       r3.Vec
	bound      float64
	incidences map[loftOutputFace]struct{}
}

type loftOutputShell struct {
	vertices       []loftOutputVertex
	triangles      [][3]int
	faces          []loftOutputFace
	triangleBounds []float64
	index          map[string]int
	heldIndex      map[[3]uint64]string
	maxBound       float64
}

func loftOutputRat(v int64) *big.Rat { return big.NewRat(v, 1) }

func loftOutputP2(p loftPartitionPoint, z *big.Rat) loftOutputPoint {
	return loftOutputPoint{p.u, p.v, z}
}

func loftOutputPointKey(p loftOutputPoint) string {
	return p.u.RatString() + "," + p.v.RatString() + "," + p.z.RatString()
}

func loftOutputSamePoint(a, b loftOutputPoint) bool {
	return a.u.Cmp(b.u) == 0 && a.v.Cmp(b.v) == 0 && a.z.Cmp(b.z) == 0
}

func loftOutputWorld(frame r3.Frame, p loftOutputPoint) ([3]*big.Rat, r3.Vec, float64, bool) {
	var exact [3]*big.Rat
	o, u, v, n := frame.Origin(), frame.U(), frame.V(), frame.N()
	if !proofbound.FiniteVec(o) || !proofbound.FiniteVec(u) ||
		!proofbound.FiniteVec(v) || !proofbound.FiniteVec(n) {
		return exact, r3.Vec{}, 0, false
	}
	coords := [4][3]float64{{o.X, o.Y, o.Z}, {u.X, u.Y, u.Z},
		{v.X, v.Y, v.Z}, {n.X, n.Y, n.Z}}
	params := [3]*big.Rat{p.u, p.v, p.z}
	var held [3]float64
	gap2 := new(big.Rat)
	for i := range exact {
		exact[i] = proofarith.FloatRat(coords[0][i])
		for j, param := range params {
			term := new(big.Rat).Mul(proofarith.FloatRat(coords[j+1][i]), param)
			exact[i].Add(exact[i], term)
		}
		held[i], _ = exact[i].Float64()
		if proofbound.IsNonFinite(held[i]) {
			return exact, r3.Vec{}, 0, false
		}
		delta := new(big.Rat).Sub(exact[i], proofarith.FloatRat(held[i]))
		gap2.Add(gap2, new(big.Rat).Mul(delta, delta))
	}
	bound := proofbound.RatSqrtUp(gap2)
	if proofbound.IsNonFinite(bound) {
		return exact, r3.Vec{}, 0, false
	}
	return exact, r3.Vec{X: held[0], Y: held[1], Z: held[2]}, bound, true
}

func loftOutputHeldKey(v r3.Vec) [3]uint64 {
	coords := [3]float64{v.X, v.Y, v.Z}
	var key [3]uint64
	for i, x := range coords {
		if x != 0 {
			key[i] = math.Float64bits(x)
		}
	}
	return key
}

func (s *loftOutputShell) vertex(frame r3.Frame, p loftOutputPoint, face loftOutputFace) (int, bool) {
	key := loftOutputPointKey(p)
	if index, exists := s.index[key]; exists {
		s.vertices[index].incidences[face] = struct{}{}
		return index, true
	}
	if len(s.vertices) == 512 {
		return 0, false
	}
	world, held, bound, ok := loftOutputWorld(frame, p)
	if !ok {
		return 0, false
	}
	heldKey := loftOutputHeldKey(held)
	if prior, exists := s.heldIndex[heldKey]; exists && prior != key {
		return 0, false
	}
	index := len(s.vertices)
	s.index[key], s.heldIndex[heldKey] = index, key
	s.vertices = append(s.vertices, loftOutputVertex{point: p, world: world, held: held,
		bound: bound, incidences: map[loftOutputFace]struct{}{face: {}}})
	s.maxBound = max(s.maxBound, bound)
	return index, true
}

func (s *loftOutputShell) triangle(frame r3.Frame, points [3]loftOutputPoint,
	face loftOutputFace) (bool, error) {
	if len(s.triangles) == 512 {
		return false, nil
	}
	var ids [3]int
	for i, p := range points {
		var ok bool
		ids[i], ok = s.vertex(frame, p, face)
		if !ok {
			return false, nil
		}
	}
	if ids[0] == ids[1] || ids[1] == ids[2] || ids[2] == ids[0] ||
		loftOutputCrossZero(points) {
		return false, nil
	}
	s.triangles = append(s.triangles, ids)
	s.faces = append(s.faces, face)
	s.triangleBounds = append(s.triangleBounds, max(s.vertices[ids[0]].bound,
		s.vertices[ids[1]].bound, s.vertices[ids[2]].bound))
	return true, nil
}

func loftOutputCrossZero(p [3]loftOutputPoint) bool {
	a := [3]*big.Rat{new(big.Rat).Sub(p[1].u, p[0].u),
		new(big.Rat).Sub(p[1].v, p[0].v), new(big.Rat).Sub(p[1].z, p[0].z)}
	b := [3]*big.Rat{new(big.Rat).Sub(p[2].u, p[0].u),
		new(big.Rat).Sub(p[2].v, p[0].v), new(big.Rat).Sub(p[2].z, p[0].z)}
	for i := range a {
		j, k := (i+1)%3, (i+2)%3
		cross := new(big.Rat).Sub(new(big.Rat).Mul(a[j], b[k]), new(big.Rat).Mul(a[k], b[j]))
		if cross.Sign() != 0 {
			return false
		}
	}
	return true
}

func loftOutputNewShell() loftOutputShell {
	return loftOutputShell{index: make(map[string]int), heldIndex: make(map[[3]uint64]string)}
}
