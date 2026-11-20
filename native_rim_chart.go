package decad

import (
	"context"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// rimBandChart certifies the entire held band has a one-to-one map to a
// finite cylindrical parameter domain. Its horizontal boundary walks each
// circle once. On each affine facet the angular/height Jacobian numerator
// is affine in X and Z. A strict sign at all three corners proves the
// orientation throughout. The boundary degree is one, hence every interior
// regular parameter has exactly one preimage. Shared edges use the same
// angular evaluation from either facet, extending the map to their closures.
func rimBandChart(ctx context.Context, p *facetedrecord.Payload, group int, center rimPoint, budget *proofbound.ReplayBudget) (bool, error) {
	edges := map[[2]int]bool{}
	height := map[float64]map[int]int{}
	ray := func(i int) (*big.Rat, *big.Rat) {
		v := p.Verts[i]
		return rimSub(rimRat(v.X), center[0]), rimSub(rimRat(v.Z), center[2])
	}
	for i, f := range p.Tris {
		if p.Src[i] != group {
			continue
		}
		if err := budget.SpendWork(ctx, 32); err != nil {
			return false, err
		}
		a, b, c := rimVec(p.Verts[f[0]]), rimVec(p.Verts[f[1]]), rimVec(p.Verts[f[2]])
		term := func(i int) *big.Rat {
			return rimSub(rimMul(rimSub(b[i], a[i]), rimSub(c[1], a[1])), rimMul(rimSub(c[i], a[i]), rimSub(b[1], a[1])))
		}
		vx, vz := term(0), term(2)
		for j, id := range f {
			x, z := ray(id)
			if rimSub(rimMul(x, vz), rimMul(z, vx)).Sign() >= 0 {
				return false, nil
			}
			k := f[(j+1)%3]
			if edges[[2]int{id, k}] {
				return false, nil
			}
			edges[[2]int{id, k}] = true
		}
	}
	for e := range edges {
		if edges[[2]int{e[1], e[0]}] {
			continue
		}
		a, b := p.Verts[e[0]], p.Verts[e[1]]
		if a.Y != b.Y {
			return false, nil
		}
		if height[a.Y] == nil {
			height[a.Y] = map[int]int{}
		}
		next := height[a.Y]
		if _, ok := next[e[0]]; ok {
			return false, nil
		}
		next[e[0]] = e[1]
	}
	if len(height) != 2 {
		return false, nil
	}
	totalWinding := 0
	for _, next := range height {
		start := -1
		for id := range next {
			start = id
			break
		}
		i := start
		sign, winding := 0, 0
		for step := range len(next) {
			j, ok := next[i]
			if !ok {
				return false, nil
			}
			x, z := ray(i)
			u, w := ray(j)
			cross := rimSub(rimMul(x, w), rimMul(z, u)).Sign()
			if cross == 0 || sign != 0 && cross != sign {
				return false, nil
			}
			sign = cross
			if z.Sign() <= 0 && w.Sign() > 0 && cross > 0 {
				winding++
			}
			if w.Sign() <= 0 && z.Sign() > 0 && cross < 0 {
				winding--
			}
			i = j
			if i == start && step+1 < len(next) {
				return false, nil
			}
		}
		if i != start || winding != sign {
			return false, nil
		}
		totalWinding += winding
	}
	return totalWinding == 0, nil
}

// rimBandError encloses the source cone mapping by the same angle and axial
// fraction as the chart. A rational unit vector gives a supporting line on
// each facet. Its projection plus axial depth is affine. The perpendicular
// departure is at most max_perpendicular² / (2 min_projection).
func rimBandError(ctx context.Context, p *facetedrecord.Payload, group int, center rimPoint, hinge *big.Rat, radius, d float64) (float64, bool) {
	low, high := math.Inf(1), math.Inf(-1)
	for i, t := range p.Tris {
		if i%128 == 0 && ctx.Err() != nil {
			return 0, false
		}
		if p.Src[i] == group {
			for _, id := range t {
				y := p.Verts[id].Y
				low = min(low, y)
				high = max(high, y)
			}
		}
	}
	if !(high > low) {
		return 0, false
	}
	axial := max(proofbound.RatFloatUp(new(big.Rat).Abs(rimSub(rimRat(low), hinge))),
		proofbound.RatFloatUp(new(big.Rat).Abs(rimSub(rimRat(high), rimAdd(hinge, rimRat(d))))))
	dy := rimSub(rimRat(high), rimRat(low))
	var radial float64
	for i, t := range p.Tris {
		if i%128 == 0 && ctx.Err() != nil {
			return 0, false
		}
		if p.Src[i] != group {
			continue
		}
		a := p.Verts[t[0]]
		cx, _ := center[0].Float64()
		cz, _ := center[2].Float64()
		x, z := a.X-cx, a.Z-cz
		sign := int64(1)
		if x < 0 {
			x = -x
			z = -z
			sign = -1
		}
		s := rimRat(math.Tan(math.Atan2(z, x) / 2))
		s2 := rimMul(s, s)
		one := big.NewRat(1, 1)
		den := rimAdd(one, s2)
		nx := new(big.Rat).Quo(rimMul(big.NewRat(sign, 1), rimSub(one, s2)), den)
		nz := new(big.Rat).Quo(rimMul(big.NewRat(2*sign, 1), s), den)
		var least, perp, exact *big.Rat
		for _, id := range t {
			v := p.Verts[id]
			rx, rz := rimSub(rimRat(v.X), center[0]), rimSub(rimRat(v.Z), center[2])
			along := rimAdd(rimMul(nx, rx), rimMul(nz, rz))
			across := new(big.Rat).Abs(rimSub(rimMul(nx, rz), rimMul(nz, rx)))
			depth := new(big.Rat).Quo(rimMul(rimRat(d), rimSub(rimRat(v.Y), rimRat(low))), dy)
			e := new(big.Rat).Abs(rimSub(rimAdd(along, depth), rimRat(radius)))
			if least == nil || along.Cmp(least) < 0 {
				least = along
			}
			if perp == nil || across.Cmp(perp) > 0 {
				perp = across
			}
			if exact == nil || e.Cmp(exact) > 0 {
				exact = e
			}
		}
		if least.Sign() <= 0 {
			return 0, false
		}
		allowance := new(big.Rat).Quo(rimMul(perp, perp), rimMul(big.NewRat(2, 1), least))
		radial = max(radial, proofbound.RatFloatUp(rimAdd(exact, allowance)))
	}
	return proofbound.AbsSumUpper(axial, radial), true
}

// rimCapChart proves that the retained cap covers the disk with exactly the
// upper band ring as its boundary. Its radial map rescales each ray between
// the center and that unique polygon intersection to the true cap radius.
func rimCapChart(ctx context.Context, p *facetedrecord.Payload, band int, caps map[int]bool, lower map[int]bool) bool {
	return rimCapChartAt(ctx, p, band, caps, lower, nil, nil)
}
func rimCapChartAt(ctx context.Context, p *facetedrecord.Payload, band int, caps, lower, cheek map[int]bool, heldHinge *big.Rat) bool {
	type edge [2]int
	type row struct {
		face int
		a, b int
	}
	links := map[edge][]row{}
	top := math.Inf(-1)
	for i, f := range p.Tris {
		if i%128 == 0 && ctx.Err() != nil {
			return false
		}
		if p.Src[i] == band {
			for _, v := range f {
				top = max(top, p.Verts[v].Y)
			}
		}
	}
	count := 0
	for i, f := range p.Tris {
		if i%128 == 0 && ctx.Err() != nil {
			return false
		}
		g := p.Src[i]
		if caps[g] {
			a, b, c := rimVec(p.Verts[f[0]]), rimVec(p.Verts[f[1]]), rimVec(p.Verts[f[2]])
			for _, v := range f {
				if p.Verts[v].Y != top {
					return false
				}
			}
			if rimSub(rimMul(rimSub(b[2], a[2]), rimSub(c[0], a[0])), rimMul(rimSub(c[2], a[2]), rimSub(b[0], a[0]))).Sign() <= 0 {
				return false
			}
			count++
		}
		for j, a := range f {
			b := f[(j+1)%3]
			k := edge{min(a, b), max(a, b)}
			links[k] = append(links[k], row{g, a, b})
		}
	}
	for _, pair := range links {
		if ctx.Err() != nil {
			return false
		}
		if len(pair) != 2 || pair[0].a != pair[1].b || pair[0].b != pair[1].a {
			return false
		}
		f, g := pair[0].face, pair[1].face
		if caps[f] != caps[g] && f != band && g != band {
			return false
		}
		if (f == band) != (g == band) {
			other := f
			if f == band {
				other = g
			}
			if p.Verts[pair[0].a].Y == top && p.Verts[pair[0].b].Y == top {
				if !caps[other] {
					return false
				}
			} else if !lower[other] {
				if !cheek[other] || heldHinge == nil {
					return false
				}
				y, _ := heldHinge.Float64()
				if p.Verts[pair[0].a].Y != y || p.Verts[pair[0].b].Y != y {
					return false
				}
			}
		}
	}
	return count > 0
}
