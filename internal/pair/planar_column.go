package pair

import (
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/proof"
)

// PlanarColumnClear is the column test of docs/multibody-dynamics-design.md
// §10.6. The plane is n·(x − q) = 0 with n a nonzero normal of s, and the box
// [lo, hi] is an axis-aligned 3D box. P projects along n onto the plane's
// frame by dropping the axis of n's largest component (§9.4's frame), which
// shortens no distance. Every triangle of s with a vertex strictly in front
// of the plane must have P(triangle) strictly apart from P(box), decided by
// the separating axes of the projected box and the triangle's edge normals in
// exact arithmetic; a triangle whose projection is a segment keeps the
// segment's normal, and a zero edge gives no axis.
//
// clear reports whether the test holds. When it does, clearance is the
// lateral clearance m: the least, over those triangles, of the largest
// separating gap, each gap divided by an upper bound on its axis's length, so
// m is a lower bound on the distance between the two projections. clearance
// is nil when no triangle lies in front of the plane, which stands for an
// unbounded m. Material of s in front of the plane then lies at least m away
// from the column over P(box): the part of that column in front of the plane
// is unbounded along the dropped axis, so the bounded solid s can reach it
// only through a boundary triangle that has a vertex in front. poll is
// charged once per triangle.
func PlanarColumnClear(s *PlanarSolid, n, q proof.DyV3, lo, hi [3]*big.Rat, poll func() error) (*big.Rat, bool, error) {
	drop := 0
	for k := 1; k < 3; k++ {
		if proof.DyCmp(proof.DyAbs(n[k]), proof.DyAbs(n[drop])) > 0 {
			drop = k
		}
	}
	i, j := (drop+1)%3, (drop+2)%3
	box := [4][2]*big.Rat{{lo[i], lo[j]}, {hi[i], lo[j]}, {lo[i], hi[j]}, {hi[i], hi[j]}}
	var clearance *big.Rat
	for _, tri := range s.Tris {
		if err := poll(); err != nil {
			return nil, false, err
		}
		front := false
		for _, v := range tri {
			if proof.DvDot(n, proof.DvSub(s.Verts[v], q)).Sign() > 0 {
				front = true
				break
			}
		}
		if !front {
			continue
		}
		var corners [3][2]proof.Dyadic
		for k, v := range tri {
			corners[k] = [2]proof.Dyadic{s.Verts[v][i], s.Verts[v][j]}
		}
		gap, ok := columnTriangleGap(corners, box)
		if !ok {
			return nil, false, nil
		}
		if clearance == nil || gap.Cmp(clearance) < 0 {
			clearance = gap
		}
	}
	return clearance, true, nil
}

// columnTriangleGap returns the largest separating gap between a projected
// triangle and a projected box, over the box's two axes and the triangle's
// edge normals, each gap over an upper bound on its axis's length. ok is false
// when no axis separates them strictly.
func columnTriangleGap(tri [3][2]proof.Dyadic, box [4][2]*big.Rat) (*big.Rat, bool) {
	one := proof.DyInt(1)
	axes := [][2]proof.Dyadic{{one, proof.DyZero()}, {proof.DyZero(), one}}
	for k := range 3 {
		from, to := tri[k], tri[(k+1)%3]
		axis := [2]proof.Dyadic{proof.DyNeg(proof.DySubScalar(to[1], from[1])), proof.DySubScalar(to[0], from[0])}
		if axis[0].Sign() == 0 && axis[1].Sign() == 0 {
			continue
		}
		axes = append(axes, axis)
	}
	var best *big.Rat
	for slot, axis := range axes {
		ax, ay := axis[0].Rat(), axis[1].Rat()
		var triLo, triHi, boxLo, boxHi *big.Rat
		for _, corner := range tri {
			value := proof.DyAdd(proof.DyMul(axis[0], corner[0]), proof.DyMul(axis[1], corner[1])).Rat()
			triLo, triHi = ratLower(triLo, value), ratUpper(triHi, value)
		}
		for _, corner := range box {
			value := new(big.Rat).Add(new(big.Rat).Mul(ax, corner[0]), new(big.Rat).Mul(ay, corner[1]))
			boxLo, boxHi = ratLower(boxLo, value), ratUpper(boxHi, value)
		}
		gap := new(big.Rat).Sub(triLo, boxHi)
		if other := new(big.Rat).Sub(boxLo, triHi); other.Cmp(gap) > 0 {
			gap = other
		}
		if gap.Sign() <= 0 {
			continue
		}
		// The box axes are unit vectors; an edge normal's length is bounded
		// above by an exactly checked float square root.
		if slot >= 2 {
			length := proof.DySqrtUp(proof.DyAdd(proof.DyMul(axis[0], axis[0]), proof.DyMul(axis[1], axis[1])))
			if math.IsInf(length, 0) || math.IsNaN(length) || length <= 0 {
				continue
			}
			gap.Quo(gap, proof.FloatRat(length))
		}
		if best == nil || gap.Cmp(best) > 0 {
			best = gap
		}
	}
	return best, best != nil
}

func ratLower(current, value *big.Rat) *big.Rat {
	if current == nil || value.Cmp(current) < 0 {
		return value
	}
	return current
}

func ratUpper(current, value *big.Rat) *big.Rat {
	if current == nil || value.Cmp(current) > 0 {
		return value
	}
	return current
}
