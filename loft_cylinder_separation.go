package decad

import (
	"math"
	"math/big"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// loftCylinderMovingSeparate proves two axial Loft teeth are disjoint at
// every height. Its chosen through-axis normal is linear in z. The dot
// product with a triangle is quadratic in barycentric coordinates, so all
// six degree-two Bernstein coefficients must have the required strict sign.
// The Loft's matched wall correspondence preserves z and reaches both ways;
// its boundary departure widens each plane by its maximum normal length.
// At fixed z the plane is linear in XY, so each filled Loft section lies on
// the same side as its entire boundary.
func loftCylinderMovingSeparate(a, b loftCylinderTooth, remaining *int) bool {
	if len(a.loft.Tris) > (*remaining)/6-len(b.loft.Tris) {
		return false
	}
	*remaining -= 6 * (len(a.loft.Tris) + len(b.loft.Tris))

	center := func(part loftCylinderTooth, endIndex int) (float64, float64) {
		angles := part.bottom
		if endIndex == 1 {
			angles = part.top
		}
		angle := (angles[0] + angles[1]) / 2
		return math.Cos(angle), math.Sin(angle)
	}
	var normals [2][2]*big.Rat
	maxNorm := 0.0
	for cap := range 2 {
		ax, ay := center(a, cap)
		bx, by := center(b, cap)
		nx, ny := bx-ax, by-ay
		if !proofbound.FiniteVec(r3.Vec{X: nx, Y: ny}) {
			return false
		}
		normals[cap] = [2]*big.Rat{proofarith.FloatRat(nx), proofarith.FloatRat(ny)}
		squared := new(big.Rat).Add(
			new(big.Rat).Mul(normals[cap][0], normals[cap][0]),
			new(big.Rat).Mul(normals[cap][1], normals[cap][1]))
		maxNorm = max(maxNorm, proofbound.RatSqrtUp(squared))
	}
	if maxNorm <= 0 || proofbound.IsNonFinite(maxNorm) {
		return false
	}

	z0 := a.loft.Frame0.Origin().Z
	z1 := a.loft.Frame1.Origin().Z
	if !(z1 > z0) || b.loft.Frame0.Origin().Z != z0 || b.loft.Frame1.Origin().Z != z1 {
		return false
	}
	zStart := proofarith.FloatRat(z0)
	zSpan := new(big.Rat).Sub(proofarith.FloatRat(z1), zStart)
	deltaX := new(big.Rat).Sub(normals[1][0], normals[0][0])
	deltaY := new(big.Rat).Sub(normals[1][1], normals[0][1])
	deltaNorm := proofbound.RatSqrtUp(new(big.Rat).Add(
		new(big.Rat).Mul(deltaX, deltaX), new(big.Rat).Mul(deltaY, deltaY)))
	one := big.NewRat(1, 1)
	two := big.NewRat(2, 1)
	for side, part := range []loftCylinderTooth{a, b} {
		bound := max(part.mesh.Bound().Base(), part.loft.Proof.FacetDeparture)
		if bound < 0 || proofbound.IsNonFinite(bound) {
			return false
		}
		margin := proofbound.ProductUpper(maxNorm, bound)
		// A bridged source point differs in height from its proof proxy.
		// A mirrored upper source can add a second constant-height shift.
		// The moving normal changes by at most |N1-N0|*shift/height;
		// charge its dot product with the source's maximum XY radius.
		shift := proofbound.AbsSumUpper(part.bridge, part.upperSeparationBridge)
		if shift > 0 {
			radius := 0.0
			for _, p := range part.loft.Verts {
				q := new(big.Rat).Add(
					new(big.Rat).Mul(proofarith.FloatRat(p.X), proofarith.FloatRat(p.X)),
					new(big.Rat).Mul(proofarith.FloatRat(p.Y), proofarith.FloatRat(p.Y)))
				radius = max(radius, proofbound.RatSqrtUp(q))
			}
			radius = proofbound.AbsSumUpper(radius, part.loft.Proof.FacetDeparture)
			fraction := proofbound.RatFloatUp(new(big.Rat).Quo(
				proofarith.FloatRat(shift), zSpan))
			margin = proofbound.AbsSumUpper(margin,
				proofbound.ProductUpper(proofbound.ProductUpper(deltaNorm, radius), fraction))
		}
		if proofbound.IsNonFinite(margin) {
			return false
		}
		limit := proofarith.FloatRat(margin)
		if side == 0 {
			limit.Neg(limit)
		}
		for _, tri := range part.loft.Tris {
			var points [3][2]*big.Rat
			var moving [3][2]*big.Rat
			for corner, index := range tri {
				point := part.loft.Verts[index]
				if !proofbound.FiniteVec(point) {
					return false
				}
				points[corner] = [2]*big.Rat{proofarith.FloatRat(point.X), proofarith.FloatRat(point.Y)}
				t := new(big.Rat).Quo(
					new(big.Rat).Sub(proofarith.FloatRat(point.Z), zStart), zSpan)
				if t.Sign() < 0 || t.Cmp(one) > 0 {
					return false
				}
				for axis := range 2 {
					moving[corner][axis] = new(big.Rat).Add(normals[0][axis],
						new(big.Rat).Mul(t, new(big.Rat).Sub(normals[1][axis], normals[0][axis])))
				}
			}
			dot := func(i, j int) *big.Rat {
				return new(big.Rat).Add(
					new(big.Rat).Mul(moving[i][0], points[j][0]),
					new(big.Rat).Mul(moving[i][1], points[j][1]))
			}
			for i := range 3 {
				for j := i; j < 3; j++ {
					coefficient := dot(i, j)
					if i != j {
						coefficient.Quo(coefficient.Add(coefficient, dot(j, i)), two)
					}
					if side == 0 && coefficient.Cmp(limit) >= 0 ||
						side == 1 && coefficient.Cmp(limit) <= 0 {
						return false
					}
				}
			}
		}
	}
	return true
}
