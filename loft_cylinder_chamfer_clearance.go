package decad

import (
	"math/big"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// An axis whose projected intervals have a gap larger than sourceBound
// separates the whole ideal band triangle from the source wall envelope.
// All coordinates and the squared distance comparison remain rational.
func loftCylinderChamferAxisGapAbove(a, b [3]proofarith.Xpt,
	axis proofarith.Xpt, sourceBound *big.Rat) bool {
	norm2 := proofarith.XdotRat(axis, axis)
	if norm2.Sign() == 0 {
		return false
	}
	minA, maxA := proofarith.XdotRat(axis, a[0]), proofarith.XdotRat(axis, a[0])
	minB, maxB := proofarith.XdotRat(axis, b[0]), proofarith.XdotRat(axis, b[0])
	for i := 1; i < 3; i++ {
		p := proofarith.XdotRat(axis, a[i])
		if p.Cmp(minA) < 0 {
			minA = p
		}
		if p.Cmp(maxA) > 0 {
			maxA = p
		}
		p = proofarith.XdotRat(axis, b[i])
		if p.Cmp(minB) < 0 {
			minB = p
		}
		if p.Cmp(maxB) > 0 {
			maxB = p
		}
	}
	gap := new(big.Rat).Sub(minA, maxB)
	if reverse := new(big.Rat).Sub(minB, maxA); reverse.Cmp(gap) > 0 {
		gap = reverse
	}
	if gap.Sign() <= 0 {
		return false
	}
	gap2 := new(big.Rat).Mul(gap, gap)
	bound2 := new(big.Rat).Mul(sourceBound, sourceBound)
	return gap2.Cmp(new(big.Rat).Mul(bound2, norm2)) > 0
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
