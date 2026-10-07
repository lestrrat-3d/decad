package planar

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
	box := newColumnBox([2]*big.Rat{lo[i], lo[j]}, [2]*big.Rat{hi[i], hi[j]})
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

// columnBox is the projected box [lo, hi] in the common-denominator form
// (proof.CommonDenom): its two in-plane extents as integer numerators over one
// shared positive denominator.
type columnBox struct {
	den    *big.Int
	lo, hi [2]*big.Int
}

func newColumnBox(lo, hi [2]*big.Rat) columnBox {
	den := proof.CommonDenom(lo[0], lo[1], hi[0], hi[1])
	box := columnBox{den: den}
	for k := range 2 {
		box.lo[k], box.hi[k] = proof.ScaledNum(lo[k], den), proof.ScaledNum(hi[k], den)
		// The corners are the same four whichever end is larger.
		if box.lo[k].Cmp(box.hi[k]) > 0 {
			box.lo[k], box.hi[k] = box.hi[k], box.lo[k]
		}
	}
	return box
}

// project is the box's extent along the axis (x, y)/q, as numerators over
// den·q: a linear function over a box takes its least value at the corner that
// picks, per coordinate, the low end for a nonnegative coefficient and the high
// end for a negative one, and its greatest at the opposite corner, so the
// extent over the four corners is exact.
func (b columnBox) project(x, y *big.Int) (*big.Int, *big.Int) {
	lo, hi := new(big.Int), new(big.Int)
	term := new(big.Int)
	for k, coefficient := range [2]*big.Int{x, y} {
		low, high := b.lo[k], b.hi[k]
		if coefficient.Sign() < 0 {
			low, high = high, low
		}
		lo.Add(lo, term.Mul(coefficient, low))
		hi.Add(hi, term.Mul(coefficient, high))
	}
	return lo, hi
}

// columnTriangleGap returns the largest separating gap between a projected
// triangle and a projected box, over the box's two axes and the triangle's
// edge normals, each gap over an upper bound on its axis's length. ok is false
// when no axis separates them strictly. The gaps compare in the
// common-denominator form, and only a positive one becomes a big.Rat.
func columnTriangleGap(tri [3][2]proof.Dyadic, box columnBox) (*big.Rat, bool) {
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
		var triLo, triHi proof.Dyadic
		for k, corner := range tri {
			value := proof.DyAdd(proof.DyMul(axis[0], corner[0]), proof.DyMul(axis[1], corner[1]))
			if k == 0 || proof.DyCmp(value, triLo) < 0 {
				triLo = value
			}
			if k == 0 || proof.DyCmp(value, triHi) > 0 {
				triHi = value
			}
		}
		ax, ay := axis[0].Rat(), axis[1].Rat()
		axisDen := proof.LcmInt(new(big.Int).Set(ax.Denom()), ay.Denom())
		boxLo, boxHi := box.project(proof.ScaledNum(ax, axisDen), proof.ScaledNum(ay, axisDen))
		boxDen := new(big.Int).Mul(box.den, axisDen)
		low, high := triLo.Rat(), triHi.Rat()
		den := proof.LcmInt(proof.LcmInt(boxDen, low.Denom()), high.Denom())
		scale := new(big.Int).Quo(den, boxDen)
		// The gaps triLo − boxHi and boxLo − triHi over den.
		gap := proof.ScaledNum(low, den)
		gap.Sub(gap, boxHi.Mul(boxHi, scale))
		other := boxLo.Mul(boxLo, scale)
		other.Sub(other, proof.ScaledNum(high, den))
		if other.Cmp(gap) > 0 {
			gap = other
		}
		if gap.Sign() <= 0 {
			continue
		}
		separation := new(big.Rat).SetFrac(gap, den)
		// The box axes are unit vectors; an edge normal's length is bounded
		// above by an exactly checked float square root.
		if slot >= 2 {
			length := proof.DySqrtUp(proof.DyAdd(proof.DyMul(axis[0], axis[0]), proof.DyMul(axis[1], axis[1])))
			if math.IsInf(length, 0) || math.IsNaN(length) || length <= 0 {
				continue
			}
			separation.Quo(separation, proof.FloatRat(length))
		}
		if best == nil || separation.Cmp(best) > 0 {
			best = separation
		}
	}
	return best, best != nil
}
