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

type loftCylinderChamferBoundSquare struct {
	num, den *big.Int
}

// An axis whose projected intervals have a gap larger than sourceBound
// separates the whole ideal band triangle from the source wall envelope.
// Homogeneous integer comparisons avoid normalizing every projected dot
// product as a rational. The squared comparison remains exact.
func loftCylinderChamferAxisGapAbove(a, b [3]proofarith.Xpt,
	axis proofarith.Xpt, boundSquare loftCylinderChamferBoundSquare) bool {
	minA, maxA, minB, maxB := a[0], a[0], b[0], b[0]
	for i := 1; i < 3; i++ {
		if proofarith.XdotDiffSign(axis, a[i], minA) < 0 {
			minA = a[i]
		}
		if proofarith.XdotDiffSign(axis, a[i], maxA) > 0 {
			maxA = a[i]
		}
		if proofarith.XdotDiffSign(axis, b[i], minB) < 0 {
			minB = b[i]
		}
		if proofarith.XdotDiffSign(axis, b[i], maxB) > 0 {
			maxB = b[i]
		}
	}
	var far, near proofarith.Xpt
	switch {
	case proofarith.XdotDiffSign(axis, minA, maxB) > 0:
		far, near = minA, maxB
	case proofarith.XdotDiffSign(axis, minB, maxA) > 0:
		far, near = minB, maxA
	default:
		return false
	}
	// gap = dot(axis, far-near) / (axis.W * diff.W), while
	// |axis|² = dot(axis, axis) / axis.W². Cancel axis.W² and compare
	// integer numerators after multiplying through positive denominators.
	diff := proofarith.Xsub(far, near)
	gapNum := proofarith.XdotNum(axis, diff)
	normNum := proofarith.XdotNum(axis, axis)
	var lhs, rhs big.Int
	lhs.Mul(gapNum, gapNum).Mul(&lhs, boundSquare.den)
	rhs.Mul(boundSquare.num, normNum).Mul(&rhs, diff.W).Mul(&rhs, diff.W)
	return lhs.Cmp(&rhs) > 0
}

// This is a reject-only triangle clearance certificate. Face normals,
// cross-edge axes, and in-plane edge normals include the coplanar case.
// Failure to find a separating axis does not assert contact.
func loftCylinderChamferTriangleGapAbove(a, b [3]proofarith.Xpt, sourceBound float64) bool {
	if sourceBound < 0 || proofbound.IsNonFinite(sourceBound) {
		return false
	}
	bound := proofarith.FloatRat(sourceBound)
	boundSquare := loftCylinderChamferBoundSquare{
		num: new(big.Int).Mul(bound.Num(), bound.Num()),
		den: new(big.Int).Mul(bound.Denom(), bound.Denom()),
	}
	var ae, be [3]proofarith.Xpt
	for i := range 3 {
		ae[i] = proofarith.Xsub(a[(i+1)%3], a[i])
		be[i] = proofarith.Xsub(b[(i+1)%3], b[i])
	}
	na := proofarith.Xcross(ae[0], ae[1])
	nb := proofarith.Xcross(be[0], be[1])
	for _, axis := range [2]proofarith.Xpt{na, nb} {
		if loftCylinderChamferAxisGapAbove(a, b, axis, boundSquare) {
			return true
		}
	}
	for i := range 3 {
		for j := range 3 {
			if loftCylinderChamferAxisGapAbove(a, b, proofarith.Xcross(ae[i], be[j]), boundSquare) {
				return true
			}
		}
		if loftCylinderChamferAxisGapAbove(a, b, proofarith.Xcross(ae[i], na), boundSquare) ||
			loftCylinderChamferAxisGapAbove(a, b, proofarith.Xcross(be[i], nb), boundSquare) {
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
