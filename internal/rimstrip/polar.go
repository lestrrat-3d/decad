package rimstrip

import (
	"context"
	"math/big"
	"slices"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// Polar describes a finite outward-facing cylindrical strip. NormalX and
// NormalZ select an open half-circle containing the two endpoint rays.
type Polar struct {
	CenterX, CenterZ, RadiusSquared, Bottom, Top *big.Rat
	NormalX, NormalZ                             *big.Rat
}
type span struct{ lo, hi *big.Rat }

// Chart proves that radial projection of the held facets covers the complete
// azimuth/height rectangle once. Slopes parameterize angle monotonically in
// the admitted half-plane. Positive local orientation and unit degree on its
// boundary exclude a fold or a second sheet.
func (g Polar) Chart(ctx context.Context, verts []r3.Vec, tris [][3]int, selected []int, m *proofbound.ReplayBudget) (map[int]float64, [2]*big.Rat, bool, error) {
	var ends [2]*big.Rat
	if m == nil || len(selected) == 0 || len(selected) > 65536 {
		return nil, ends, false, nil
	}
	for _, q := range []*big.Rat{g.CenterX, g.CenterZ, g.RadiusSquared, g.Bottom, g.Top, g.NormalX, g.NormalZ} {
		if q == nil || q.Num().BitLen() > 256 || q.Denom().BitLen() > 256 {
			return nil, ends, false, nil
		}
	}
	if g.RadiusSquared.Sign() <= 0 || g.Bottom.Cmp(g.Top) >= 0 {
		return nil, ends, false, nil
	}
	rlo, rhi := rat(proofbound.RatSqrtDown(g.RadiusSquared)), rat(proofbound.RatSqrtUp(g.RadiusSquared))
	if rlo == nil || rhi == nil {
		return nil, ends, false, nil
	}
	ids := map[[3]string]int{}
	var points []point
	var slope []*big.Rat
	counts := map[[2]int]int{}
	result := map[int]float64{}
	for _, i := range selected {
		if err := m.ReserveBundle(ctx, 4096, 32, 1<<17); err != nil {
			return nil, ends, false, err
		}
		if i < 0 || i >= len(tris) {
			return nil, ends, false, nil
		}
		var t [3]point
		for j, id := range tris[i] {
			if id < 0 || id >= len(verts) || !proofbound.FiniteVec(verts[id]) {
				return nil, ends, false, nil
			}
			v := verts[id]
			t[j] = point{sub(rat(v.X), g.CenterX), rat(v.Y), sub(rat(v.Z), g.CenterZ)}
		}
		p := cutY(cutY(t[:], g.Bottom, 1), g.Top, -1)
		if len(p) < 3 {
			continue
		}
		a, b, c := t[0], t[1], t[2]
		nx := sub(mul(sub(b[1], a[1]), sub(c[2], a[2])), mul(sub(b[2], a[2]), sub(c[1], a[1])))
		nz := sub(mul(sub(b[0], a[0]), sub(c[1], a[1])), mul(sub(b[1], a[1]), sub(c[0], a[0])))
		var vertices []int
		worst := new(big.Rat)
		for j, u := range p {
			v := p[(j+1)%len(p)]
			normal := add(mul(g.NormalX, u[0]), mul(g.NormalZ, u[2]))
			if normal.Sign() <= 0 || add(mul(nx, u[0]), mul(nz, u[2])).Sign() <= 0 {
				return nil, ends, false, nil
			}
			k := [3]string{u[0].RatString(), u[1].RatString(), u[2].RatString()}
			id, ok := ids[k]
			if !ok {
				id = len(points)
				ids[k] = id
				points = append(points, u)
				slope = append(slope, quo(sub(mul(g.NormalX, u[2]), mul(g.NormalZ, u[0])), normal))
			}
			vertices = append(vertices, id)
			// The norm is convex, so its largest value is at a vertex. The
			// projected polygon excludes the axis, so its minimum is on an edge.
			es := add(mul(u[0], u[0]), mul(u[2], u[2]))
			closest := es
			dx, dz := sub(v[0], u[0]), sub(v[2], u[2])
			n := add(mul(dx, dx), mul(dz, dz))
			dot := add(mul(u[0], dx), mul(u[2], dz))
			if dot.Sign() < 0 && add(n, dot).Sign() > 0 {
				closest = sub(es, quo(mul(dot, dot), n))
			}
			lo, hi := rat(proofbound.RatSqrtDown(closest)), rat(proofbound.RatSqrtUp(es))
			if lo == nil || hi == nil {
				return nil, ends, false, nil
			}
			worst = maxRat(worst, maxRat(sub(rhi, lo), sub(hi, rlo)))
		}
		for j, a := range vertices {
			b := vertices[(j+1)%len(vertices)]
			if a == b {
				return nil, ends, false, nil
			}
			key := [2]int{a, b}
			counts[key]++
			if counts[key] != 1 {
				return nil, ends, false, nil
			}
		}
		result[i] = proofbound.RatFloatUp(worst)
	}
	var sides [4][]span
	for edge := range counts {
		a, b := edge[0], edge[1]
		if counts[[2]int{b, a}] == 1 {
			continue
		}
		h, k := points[a][1], points[b][1]
		u, v := slope[a], slope[b]
		switch {
		case h.Cmp(g.Bottom) == 0 && k.Cmp(h) == 0 && u.Cmp(v) > 0:
			sides[0] = append(sides[0], span{v, u})
		case h.Cmp(g.Top) == 0 && k.Cmp(h) == 0 && u.Cmp(v) < 0:
			sides[1] = append(sides[1], span{u, v})
		case u.Cmp(v) == 0 && h.Cmp(k) != 0:
			idx := 0
			if h.Cmp(k) > 0 {
				idx = 1
				h, k = k, h
			}
			if ends[idx] != nil && ends[idx].Cmp(u) != 0 {
				return nil, ends, false, nil
			}
			ends[idx] = u
			sides[2+idx] = append(sides[2+idx], span{h, k})
		default:
			return nil, ends, false, nil
		}
	}
	if ends[0] == nil || ends[1] == nil || ends[0].Cmp(ends[1]) >= 0 {
		return nil, ends, false, nil
	}
	for i, v := range sides {
		lo, hi := ends[0], ends[1]
		if i >= 2 {
			lo, hi = g.Bottom, g.Top
		}
		slices.SortFunc(v, func(a, b span) int { return a.lo.Cmp(b.lo) })
		for _, s := range v {
			if s.lo.Cmp(lo) != 0 {
				return nil, ends, false, nil
			}
			lo = s.hi
		}
		if lo.Cmp(hi) != 0 {
			return nil, ends, false, nil
		}
	}
	return result, ends, true, nil
}

// EndpointRegistration bounds displacement between corresponding angles
// under affine angular registration. For an acute angle its tangent bounds
// its angle, and arc length bounds the distance between the two points.
func (g Polar) EndpointRegistration(held [2]*big.Rat, native [2][2]*big.Rat) (*big.Rat, bool) {
	if g.RadiusSquared == nil || g.RadiusSquared.Sign() <= 0 || g.NormalX == nil || g.NormalZ == nil {
		return nil, false
	}
	radius := rat(proofbound.RatSqrtUp(g.RadiusSquared))
	if radius == nil {
		return nil, false
	}
	maxTangent := new(big.Rat)
	for i, h := range held {
		u, v := native[i][0], native[i][1]
		if h == nil || u == nil || v == nil {
			return nil, false
		}
		rho := add(mul(g.NormalX, u), mul(g.NormalZ, v))
		if rho.Sign() <= 0 {
			return nil, false
		}
		s := quo(sub(mul(g.NormalX, v), mul(g.NormalZ, u)), rho)
		dot := add(big.NewRat(1, 1), mul(h, s))
		if dot.Sign() <= 0 {
			return nil, false
		}
		maxTangent = maxRat(maxTangent, quo(abs(sub(h, s)), dot))
	}
	return mul(radius, maxTangent), true
}
