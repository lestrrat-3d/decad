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

func loftReplayArithmetic(optional []*loftVolumeArithmetic) *loftVolumeArithmetic {
	if len(optional) != 0 {
		return optional[0]
	}
	return &loftVolumeArithmetic{ok: true}
}

// float64 checks the alignment performed by big.Rat.Float64 before it can
// allocate a temporary wider than the replay rational limit.
func (c *loftVolumeArithmetic) float64(a *big.Rat) (float64, bool) {
	if !c.valid(a) {
		return 0, false
	}
	if numeratorBits := a.Num().BitLen(); numeratorBits != 0 {
		denominatorBits := a.Denom().BitLen()
		if shift := 54 - (numeratorBits - denominatorBits); shift > 0 &&
			numeratorBits+shift > loftVolumeRationalBits {
			c.ok = false
			return 0, false
		}
	}
	return a.Float64()
}

func loftOutputP2(p loftPartitionPoint, z *big.Rat) loftOutputPoint {
	return loftOutputPoint{p.u, p.v, z}
}

func loftOutputPointKey(p loftOutputPoint) string {
	return p.u.RatString() + "," + p.v.RatString() + "," + p.z.RatString()
}

func loftOutputSamePoint(a, b loftOutputPoint) bool {
	arithmetic := &loftVolumeArithmetic{ok: true}
	same := arithmetic.cmp(a.u, b.u) == 0 && arithmetic.cmp(a.v, b.v) == 0 &&
		arithmetic.cmp(a.z, b.z) == 0
	return arithmetic.ok && same
}

func loftOutputWorld(frame r3.Frame, p loftOutputPoint) ([3]*big.Rat, r3.Vec, float64, bool) {
	var exact [3]*big.Rat
	c := &loftVolumeArithmetic{ok: true}
	if !c.valid(p.u) || !c.valid(p.v) || !c.valid(p.z) {
		return exact, r3.Vec{}, 0, false
	}
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
			term := c.mul(proofarith.FloatRat(coords[j+1][i]), param)
			exact[i] = c.add(exact[i], term)
			if !c.ok {
				return exact, r3.Vec{}, 0, false
			}
		}
		held[i], _ = c.float64(exact[i])
		if !c.ok || proofbound.IsNonFinite(held[i]) {
			return exact, r3.Vec{}, 0, false
		}
		delta := c.sub(exact[i], proofarith.FloatRat(held[i]))
		gap2 = c.add(gap2, c.mul(delta, delta))
		if !c.ok {
			return exact, r3.Vec{}, 0, false
		}
	}
	// RatSqrtUp compares a finite float square with gap2 using up to 2,252
	// shift bits on its numerator or 2,048 on its denominator.
	if gap2.Num().BitLen() > loftVolumeRationalBits-2252 ||
		gap2.Denom().BitLen() > loftVolumeRationalBits-2048 {
		return exact, r3.Vec{}, 0, false
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
	c := &loftVolumeArithmetic{ok: true}
	if !c.valid(p.u) || !c.valid(p.v) || !c.valid(p.z) {
		return 0, false
	}
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
	face loftOutputFace) bool {
	if len(s.triangles) == 512 {
		return false
	}
	var ids [3]int
	for i, p := range points {
		var ok bool
		ids[i], ok = s.vertex(frame, p, face)
		if !ok {
			return false
		}
	}
	if ids[0] == ids[1] || ids[1] == ids[2] || ids[2] == ids[0] ||
		loftOutputCrossZero(points) {
		return false
	}
	s.triangles = append(s.triangles, ids)
	s.faces = append(s.faces, face)
	s.triangleBounds = append(s.triangleBounds, max(s.vertices[ids[0]].bound,
		s.vertices[ids[1]].bound, s.vertices[ids[2]].bound))
	return true
}

func loftOutputCrossZero(p [3]loftOutputPoint) bool {
	c := &loftVolumeArithmetic{ok: true}
	a := [3]*big.Rat{c.sub(p[1].u, p[0].u),
		c.sub(p[1].v, p[0].v), c.sub(p[1].z, p[0].z)}
	b := [3]*big.Rat{c.sub(p[2].u, p[0].u),
		c.sub(p[2].v, p[0].v), c.sub(p[2].z, p[0].z)}
	if !c.ok {
		return true
	}
	for i := range a {
		j, k := (i+1)%3, (i+2)%3
		cross := c.sub(c.mul(a[j], b[k]), c.mul(a[k], b[j]))
		if !c.ok {
			return true
		}
		if cross.Sign() != 0 {
			return false
		}
	}
	return true
}

func loftOutputNewShell() loftOutputShell {
	return loftOutputShell{index: make(map[string]int), heldIndex: make(map[[3]uint64]string)}
}
