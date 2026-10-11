package decad

import (
	"context"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

func spiralRationalDirection(sumY, sumZ float64) (*big.Rat, *big.Rat, bool) {
	flip := sumY < -0.5*math.Hypot(sumY, sumZ)
	if flip {
		sumY, sumZ = -sumY, -sumZ
	}
	den := math.Hypot(sumY, sumZ) + sumY
	if den <= 0 || math.IsNaN(den) || math.IsInf(den, 0) {
		return nil, nil, false
	}
	t := sumZ / den
	if math.IsNaN(t) || math.IsInf(t, 0) || math.Abs(t) > 1e6 {
		return nil, nil, false
	}
	n := int64(math.Round(t * 1e6))
	d := int64(1e6)
	nn, dd := new(big.Int).Mul(big.NewInt(n), big.NewInt(n)),
		new(big.Int).Mul(big.NewInt(d), big.NewInt(d))
	denom := new(big.Int).Add(nn, dd)
	unitY := new(big.Rat).SetFrac(new(big.Int).Sub(dd, nn), denom)
	unitZ := new(big.Rat).SetFrac(new(big.Int).Mul(big.NewInt(2*n), big.NewInt(d)), denom)
	if flip {
		unitY.Neg(unitY)
		unitZ.Neg(unitZ)
	}
	return unitY, unitZ, true
}

// The rounded angle only selects an exact rational support direction. It
// never enters the final bound; the exact unit vector bounds each triangle.
func spiralTriangleDirection(v [3]r3.Vec) (*big.Rat, *big.Rat, bool) {
	base := math.Atan2(v[0].Z, v[0].Y)
	lo, hi := base, base
	for _, p := range v[1:] {
		a := math.Atan2(p.Z, p.Y)
		for a-base > math.Pi {
			a -= 2 * math.Pi
		}
		for a-base < -math.Pi {
			a += 2 * math.Pi
		}
		lo, hi = min(lo, a), max(hi, a)
	}
	if hi-lo >= math.Pi {
		return nil, nil, false
	}
	mid := (lo + hi) / 2
	return spiralRationalDirection(math.Cos(mid), math.Sin(mid))
}

func spiralTriangleMeridianError(points [3]r3.Vec, m, b *big.Rat) (*big.Rat, bool) {
	unitY, unitZ, ok := spiralTriangleDirection(points)
	if !ok {
		return nil, false
	}
	limit := new(big.Rat)
	for _, v := range points {
		meridian := lfAdd(lfMul(m, lfF(v.X)), b)
		lowerRadius := lfAdd(lfMul(unitY, lfF(v.Y)), lfMul(unitZ, lfF(v.Z)))
		below := lfSub(meridian, lowerRadius)
		if below.Cmp(limit) > 0 {
			limit = below
		}
		r2 := lfAdd(lfMul(lfF(v.Y), lfF(v.Y)), lfMul(lfF(v.Z), lfF(v.Z)))
		radialUp := proofbound.RatSqrtUp(r2)
		if proofbound.IsNonFinite(radialUp) {
			return nil, false
		}
		above := lfSub(lfF(radialUp), meridian)
		if above.Cmp(limit) > 0 {
			limit = above
		}
	}
	return limit, true
}

// spiralHeldBlankRadialAllowance bounds the actual held face's radius at
// each X against its exact source meridian. The caller separately proves
// this face is a full radial graph with spiralHeldBlankCellGraph.
func spiralHeldBlankRadialAllowance(ctx context.Context, blank *Body,
	held *meshbool.BoolMesh, source *bevelBlankSource, edge int) (float64, bool, error) {
	if ctx == nil || blank == nil || held == nil || source == nil ||
		edge < 0 || edge >= len(source.edgeIndex) || len(held.Src) != len(held.Tris) {
		return 0, false, nil
	}
	group := spiralBlankFaceGroup(blank, source, edge)
	if group < 0 {
		return 0, false, nil
	}
	cycle := [6]Point2{source.toeAxis, source.heelAxis, source.heelOuter,
		source.ded, source.toeRoot, source.toeOuter}
	a, z := cycle[edge], cycle[(edge+1)%len(cycle)]
	if a.U == z.U {
		return 0, false, nil
	}
	m := lfDiv(lfSub(lfF(z.V), lfF(a.V)), lfSub(lfF(z.U), lfF(a.U)))
	b := lfSub(lfF(a.V), lfMul(m, lfF(a.U)))
	limit := new(big.Rat)
	seen := false
	for i, tri := range held.Tris {
		if i%256 == 0 {
			if err := ctx.Err(); err != nil {
				return 0, false, err
			}
		}
		if held.Src[i] != group {
			continue
		}
		seen = true
		var points [3]r3.Vec
		for k, index := range tri {
			if index < 0 || index >= len(held.Verts) ||
				!proofbound.FiniteVec(held.Verts[index]) {
				return 0, false, nil
			}
			points[k] = held.Verts[index]
		}
		triangleLimit, ok := spiralTriangleMeridianError(points, m, b)
		if !ok {
			return 0, false, nil
		}
		if triangleLimit.Cmp(limit) > 0 {
			limit = triangleLimit
		}
	}
	if !seen {
		return 0, false, nil
	}
	allowance := proofbound.RatFloatUp(limit)
	return allowance, !proofbound.IsNonFinite(allowance), nil
}
