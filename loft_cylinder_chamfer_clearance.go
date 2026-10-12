package decad

import (
	"math"
	"math/big"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// The first box encloses an exact triangle by outward rounding; the second
// holds binary64 vertices exactly. A one-ulp-down coordinate gap strictly
// above the source displacement separates the full triangle pair.
func loftCylinderChamferBoxGapAbove(a, b [2]r3.Vec, sourceBound float64) bool {
	if sourceBound < 0 || proofbound.IsNonFinite(sourceBound) {
		return false
	}
	apart := func(lo, hi float64) bool {
		return math.Nextafter(lo-hi, math.Inf(-1)) > sourceBound
	}
	return apart(a[0].X, b[1].X) || apart(b[0].X, a[1].X) ||
		apart(a[0].Y, b[1].Y) || apart(b[0].Y, a[1].Y) ||
		apart(a[0].Z, b[1].Z) || apart(b[0].Z, a[1].Z)
}

// An axis whose projected intervals have a gap larger than sourceBound
// separates the whole ideal band triangle from the source wall envelope.
// Cross-multiplication keeps the exact comparison in integers.
func loftCylinderChamferAxisGapAbove(a, b [3]proofarith.Xpt,
	axis proofarith.Xpt, sourceBound *big.Rat) bool {
	normNum := proofarith.XdotNum(axis, axis)
	if normNum.Sign() == 0 {
		return false
	}
	// The axis weight is common to every projection, so only the point
	// weights matter when ordering them.
	project := func(tri [3]proofarith.Xpt) (int, int, [3]*big.Int) {
		var numerators [3]*big.Int
		minIndex, maxIndex := 0, 0
		for i, point := range tri {
			numerators[i] = proofarith.XdotNum(axis, point)
			if i == 0 {
				continue
			}
			compare := func(j int) int {
				if point.W.Cmp(tri[j].W) == 0 {
					return numerators[i].Cmp(numerators[j])
				}
				var left, right big.Int
				left.Mul(numerators[i], tri[j].W)
				right.Mul(numerators[j], point.W)
				return left.Cmp(&right)
			}
			if compare(minIndex) < 0 {
				minIndex = i
			}
			if compare(maxIndex) > 0 {
				maxIndex = i
			}
		}
		return minIndex, maxIndex, numerators
	}
	minA, maxA, aNum := project(a)
	minB, maxB, bNum := project(b)
	gap := func(loNum, loWeight, hiNum, hiWeight *big.Int) (*big.Int, *big.Int) {
		var left, right big.Int
		left.Mul(loNum, hiWeight)
		right.Mul(hiNum, loWeight)
		return left.Sub(&left, &right), new(big.Int).Mul(loWeight, hiWeight)
	}
	gapNum, gapDen := gap(aNum[minA], a[minA].W, bNum[maxB], b[maxB].W)
	if gapNum.Sign() <= 0 {
		gapNum, gapDen = gap(bNum[minB], b[minB].W, aNum[maxA], a[maxA].W)
		if gapNum.Sign() <= 0 {
			return false
		}
	}
	// gap = gapNum/(axis.W*gapDen), norm² = normNum/axis.W².
	// Squaring and cancelling axis.W² leaves this strict integer comparison.
	var left, right, factor big.Int
	left.Mul(gapNum, sourceBound.Denom())
	left.Mul(&left, &left)
	factor.Mul(sourceBound.Num(), gapDen)
	right.Mul(&factor, &factor)
	right.Mul(&right, normNum)
	return left.Cmp(&right) > 0
}

// This is a reject-only triangle clearance certificate. Face normals,
// cross-edge axes, and in-plane edge normals include the coplanar case.
// Failure to find a separating axis does not assert contact.
func loftCylinderChamferTriangleGapAbove(a, b [3]proofarith.Xpt, sourceBound float64) bool {
	if sourceBound < 0 || proofbound.IsNonFinite(sourceBound) {
		return false
	}
	bound := proofarith.FloatRat(sourceBound)
	var ae, be [3]proofarith.Xpt
	for i := range 3 {
		ae[i] = proofarith.Xsub(a[(i+1)%3], a[i])
		be[i] = proofarith.Xsub(b[(i+1)%3], b[i])
	}
	na := proofarith.Xcross(ae[0], ae[1])
	nb := proofarith.Xcross(be[0], be[1])
	for _, axis := range [2]proofarith.Xpt{na, nb} {
		if loftCylinderChamferAxisGapAbove(a, b, axis, bound) {
			return true
		}
	}
	for i := range 3 {
		for j := range 3 {
			if loftCylinderChamferAxisGapAbove(a, b, proofarith.Xcross(ae[i], be[j]), bound) {
				return true
			}
		}
		if loftCylinderChamferAxisGapAbove(a, b, proofarith.Xcross(ae[i], na), bound) ||
			loftCylinderChamferAxisGapAbove(a, b, proofarith.Xcross(be[i], nb), bound) {
			return true
		}
	}
	return false
}

// Splitting a triangle makes the separating-axis certificate complete for
// many close but disjoint pairs while preserving an exact partition.
func loftCylinderChamferTriangleGapSplit(a, b [3]proofarith.Xpt,
	sourceBound float64, depth int) bool {
	if loftCylinderChamferTriangleGapAbove(a, b, sourceBound) {
		return true
	}
	if depth == 0 {
		return false
	}
	longest := 0
	best := -1.0
	for i := range 3 {
		p, q := a[i].Vec(), a[(i+1)%3].Vec()
		dx, dy, dz := p.X-q.X, p.Y-q.Y, p.Z-q.Z
		if length2 := dx*dx + dy*dy + dz*dz; length2 > best {
			best, longest = length2, i
		}
	}
	i, j, k := longest, (longest+1)%3, (longest+2)%3
	mid := proofarith.Xlerp(a[i], a[j], big.NewInt(1), big.NewInt(2))
	return loftCylinderChamferTriangleGapSplit([3]proofarith.Xpt{a[i], mid, a[k]}, b,
		sourceBound, depth-1) &&
		loftCylinderChamferTriangleGapSplit([3]proofarith.Xpt{mid, a[j], a[k]}, b,
			sourceBound, depth-1)
}
