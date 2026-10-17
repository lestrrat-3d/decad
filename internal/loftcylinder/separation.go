package loftcylinder

import (
	"math"
	"math/big"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// SeparationTooth carries the certified geometry read by the fixed and
// moving through-axis separation proofs. The caller keeps the slices unchanged.
type SeparationTooth struct {
	Bottom, Top          [2]float64
	Verts                []r3.Vec
	Tris                 [][3]int
	Z0, Z1               float64
	MeshBound, WallBound float64
	Bridge, UpperBridge  float64
}

// SeparateTeeth checks fixed or moving through-axis planes for every pair,
// then checks the adjacent angular gaps at both caps.
func SeparateTeeth(parts []SeparationTooth) bool {
	// Try a fixed through-axis plane first. A twisted pair may need a plane
	// whose normal changes with height; that case checks whole triangles.
	movingWork := 8_000_000
	for i := range parts {
		for j := i + 1; j < len(parts); j++ {
			center := func(part SeparationTooth) (float64, float64) {
				angle := (part.Bottom[0] + part.Bottom[1] +
					part.Top[0] + part.Top[1]) / 4
				return math.Cos(angle), math.Sin(angle)
			}
			ax, ay := center(parts[i])
			bx, by := center(parts[j])
			nx, ny := bx-ax, by-ay
			if !proofbound.FiniteVec(r3.Vec{X: nx, Y: ny}) {
				return false
			}
			rx, ry := proofarith.FloatRat(nx), proofarith.FloatRat(ny)
			length2 := new(big.Rat).Add(new(big.Rat).Mul(rx, rx),
				new(big.Rat).Mul(ry, ry))
			length := proofbound.RatSqrtUp(length2)
			if length <= 0 || proofbound.IsNonFinite(length) {
				return false
			}
			fixed := true
			for k, part := range []SeparationTooth{parts[i], parts[j]} {
				margin := proofbound.ProductUpper(length, part.MeshBound)
				marginRat := proofarith.FloatRat(margin)
				for _, p := range part.Verts {
					dot := new(big.Rat).Add(new(big.Rat).Mul(rx, proofarith.FloatRat(p.X)),
						new(big.Rat).Mul(ry, proofarith.FloatRat(p.Y)))
					if k == 0 && dot.Cmp(new(big.Rat).Neg(marginRat)) >= 0 ||
						k == 1 && dot.Cmp(marginRat) <= 0 {
						fixed = false
						break
					}
				}
				if !fixed {
					break
				}
			}
			if !fixed && !movingSeparate(parts[i], parts[j], &movingWork) {
				return false
			}
		}
	}
	for i := range parts {
		j := (i + 1) % len(parts)
		for cap := range 2 {
			a, b := parts[i].Bottom, parts[j].Bottom
			if cap == 1 {
				a, b = parts[i].Top, parts[j].Top
			}
			for b[0] <= a[0] {
				b[0] += 2 * math.Pi
				b[1] += 2 * math.Pi
			}
			if a[1]+0.02 >= b[0] || b[0]-a[1] >= 2*math.Pi {
				return false
			}
		}
	}
	return true
}

// movingSeparate proves two axial Loft teeth are disjoint at every height.
// Its chosen through-axis normal is linear in z. The dot product with a
// triangle is quadratic in barycentric coordinates, so all six degree-two
// Bernstein coefficients must have the required strict sign. The Loft's
// matched wall correspondence preserves z and reaches both ways; its boundary
// departure widens each plane by its maximum normal length. At fixed z the
// plane is linear in XY, so each filled Loft section lies on the same side as
// its entire boundary.
func movingSeparate(a, b SeparationTooth, remaining *int) bool {
	if len(a.Tris) > (*remaining)/6-len(b.Tris) {
		return false
	}
	*remaining -= 6 * (len(a.Tris) + len(b.Tris))

	center := func(part SeparationTooth, endIndex int) (float64, float64) {
		angles := part.Bottom
		if endIndex == 1 {
			angles = part.Top
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

	z0 := a.Z0
	z1 := a.Z1
	if !(z1 > z0) || b.Z0 != z0 || b.Z1 != z1 {
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
	for side, part := range []SeparationTooth{a, b} {
		bound := max(part.MeshBound, part.WallBound)
		if bound < 0 || proofbound.IsNonFinite(bound) {
			return false
		}
		margin := proofbound.ProductUpper(maxNorm, bound)
		// A bridged source point differs in height from its proof proxy.
		// A mirrored upper source can add a second constant-height shift.
		// The moving normal changes by at most |N1-N0|*shift/height;
		// charge its dot product with the source's maximum XY radius.
		shift := proofbound.AbsSumUpper(part.Bridge, part.UpperBridge)
		if shift > 0 {
			radius := 0.0
			for _, p := range part.Verts {
				q := new(big.Rat).Add(
					new(big.Rat).Mul(proofarith.FloatRat(p.X), proofarith.FloatRat(p.X)),
					new(big.Rat).Mul(proofarith.FloatRat(p.Y), proofarith.FloatRat(p.Y)))
				radius = max(radius, proofbound.RatSqrtUp(q))
			}
			radius = proofbound.AbsSumUpper(radius, part.WallBound)
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
		for _, tri := range part.Tris {
			var points [3][2]*big.Rat
			var moving [3][2]*big.Rat
			for corner, index := range tri {
				point := part.Verts[index]
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
