package decad

import (
	"math/big"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// A LineSeg Loft wall is the held triangle pair, so its section at height t
// is a two-segment chain through the triangles' common diagonal. Prove that
// both segments increase in projected radius for every t. The inside root
// ruling and outside flank ruling then give exactly one circle crossing.
func loftCylinderConnectorGate(lp loftPayload, root int, radius, height, sourceBound float64) (float64, bool) {
	n := len(lp.verts) / 2
	if root < 3 || lp.segment[0][0] != 0 || lp.segment[0][1] != 1 ||
		lp.segment[0][root-2] != 3 || lp.segment[0][root-1] != 4 {
		return 0, false
	}
	collar := 0.0
	for _, spec := range [][2]int{{0, 1}, {root, root - 1}} {
		near, far := spec[0], spec[1]
		j := min(near, far)
		a := [2]r3.Vec{lp.verts[near], lp.verts[n+near]}
		f := [2]r3.Vec{lp.verts[far], lp.verts[n+far]}
		minimum := loftCylinderTriangleMinRadius(f[:], [3]int{0, 1, 1})
		threshold := new(big.Rat).Add(proofarith.FloatRat(radius), proofarith.FloatRat(sourceBound))
		if proofarith.FloatRat(minimum).Cmp(threshold) <= 0 {
			return 0, false
		}
		var shared []int
		for _, v := range lp.tris[2*j] {
			for _, w := range lp.tris[2*j+1] {
				if v == w {
					shared = append(shared, v)
				}
			}
		}
		if len(shared) != 2 {
			return 0, false
		}
		diagonal := [2]int{-1, -1}
		for _, v := range shared {
			switch v {
			case near, far:
				diagonal[0] = v
			case n + near, n + far:
				diagonal[1] = v
			default:
				return 0, false
			}
		}
		if diagonal[0] < 0 || diagonal[1] < 0 {
			return 0, false
		}
		d := [2]r3.Vec{lp.verts[diagonal[0]], lp.verts[diagonal[1]]}
		if !loftCylinderRadialMonotone(a, d) || !loftCylinderRadialMonotone(d, f) {
			return 0, false
		}
		nearCollar, ok := loftCylinderHeldRootCollar(a, radius, height)
		if !ok {
			return 0, false
		}
		collar = max(collar, nearCollar)
	}
	return collar, true
}

func loftCylinderRadialMonotone(u, v [2]r3.Vec) bool {
	dot := func(a, b loftCylinderXY) *big.Rat {
		return new(big.Rat).Add(
			new(big.Rat).Mul(a.x, b.x), new(big.Rat).Mul(a.y, b.y))
	}
	delta := func(a, b loftCylinderXY) loftCylinderXY {
		return loftCylinderXY{new(big.Rat).Sub(b.x, a.x), new(big.Rat).Sub(b.y, a.y)}
	}
	u0, u1 := loftCylinderExactXY(u[0]), loftCylinderExactXY(u[1])
	d0, d1 := delta(u0, loftCylinderExactXY(v[0])), delta(u1, loftCylinderExactXY(v[1]))
	g0, g2 := dot(u0, d0), dot(u1, d1)
	g1 := new(big.Rat).Add(dot(u0, d1), dot(u1, d0))
	return g0.Sign() >= 0 && g1.Sign() >= 0 && g2.Sign() >= 0
}

func loftCylinderHeldRootCollar(near [2]r3.Vec, radius, height float64) (float64, bool) {
	q := func(p loftCylinderXY) *big.Rat {
		return new(big.Rat).Add(new(big.Rat).Mul(p.x, p.x), new(big.Rat).Mul(p.y, p.y))
	}
	a, b := loftCylinderExactXY(near[0]), loftCylinderExactXY(near[1])
	r := proofarith.FloatRat(radius)
	r2 := new(big.Rat).Mul(r, r)
	if q(a).Cmp(r2) <= 0 && q(b).Cmp(r2) <= 0 {
		return 0, true
	}
	for shift := 52; shift >= 8; shift-- {
		epsilon := new(big.Rat).SetFrac(big.NewInt(1), new(big.Int).Lsh(big.NewInt(1), uint(shift)))
		oneMinus := new(big.Rat).Sub(big.NewRat(1, 1), epsilon)
		bottom := loftCylinderXY{
			x: new(big.Rat).Add(new(big.Rat).Mul(oneMinus, a.x), new(big.Rat).Mul(epsilon, b.x)),
			y: new(big.Rat).Add(new(big.Rat).Mul(oneMinus, a.y), new(big.Rat).Mul(epsilon, b.y)),
		}
		top := loftCylinderXY{
			x: new(big.Rat).Add(new(big.Rat).Mul(epsilon, a.x), new(big.Rat).Mul(oneMinus, b.x)),
			y: new(big.Rat).Add(new(big.Rat).Mul(epsilon, a.y), new(big.Rat).Mul(oneMinus, b.y)),
		}
		if q(bottom).Cmp(r2) < 0 && q(top).Cmp(r2) < 0 {
			collar := proofbound.ProductUpper(proofbound.RatFloatUp(epsilon), height)
			return collar, !proofbound.IsNonFinite(collar)
		}
	}
	return 0, false
}
