package rimstrip

import (
	"context"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// PolarDomain encloses the finite angular and axial interval of a native arc.
type PolarDomain struct {
	Polar
	Rays [2][2]*big.Rat
}

// Away proves that the complete beta-neighborhood of a held triangle misses
// the closed native band. Axis boxes, endpoint half-planes and radial bounds
// each give independent sufficient separation tests.
func (g PolarDomain) Away(ctx context.Context, t [3]r3.Vec, beta float64, m *proofbound.ReplayBudget) (bool, error) {
	if beta < 0 || proofbound.IsNonFinite(beta) {
		return false, nil
	}
	for _, v := range t {
		if !proofbound.FiniteVec(v) {
			return false, nil
		}
	}
	if err := m.SpendWork(ctx, 32); err != nil {
		return false, err
	}
	lo, hi := proofbound.RatFloatDown(g.Bottom), proofbound.RatFloatUp(g.Top)
	if math.Nextafter(max(t[0].Y, t[1].Y, t[2].Y)+beta, math.Inf(1)) < lo || math.Nextafter(min(t[0].Y, t[1].Y, t[2].Y)-beta, math.Inf(-1)) > hi {
		return true, nil
	}
	if err := m.ReserveBundle(ctx, 256, 0, 16384); err != nil {
		return false, err
	}
	e := rat(beta)
	var q [3][2]*big.Rat
	for i, v := range t {
		q[i] = [2]*big.Rat{sub(rat(v.X), g.CenterX), sub(rat(v.Z), g.CenterZ)}
	}
	for i, ray := range g.Rays {
		var best *big.Rat
		for _, p := range q {
			v := sub(mul(ray[0], p[1]), mul(ray[1], p[0]))
			if i == 1 {
				v.Neg(v)
			}
			if best == nil || v.Cmp(best) > 0 {
				best = v
			}
		}
		if best.Sign() < 0 && mul(best, best).Cmp(mul(mul(e, e), add(mul(ray[0], ray[0]), mul(ray[1], ray[1])))) > 0 {
			return true, nil
		}
	}
	rlo, rhi := sub(rat(proofbound.RatSqrtDown(g.RadiusSquared)), e), add(rat(proofbound.RatSqrtUp(g.RadiusSquared)), e)
	if rlo.Sign() > 0 {
		inside := true
		for _, p := range q {
			if add(mul(p[0], p[0]), mul(p[1], p[1])).Cmp(mul(rlo, rlo)) >= 0 {
				inside = false
				break
			}
		}
		if inside {
			return true, nil
		}
	}
	var d [2]*big.Rat
	for i := range 2 {
		a, b := q[0][i], q[0][i]
		for _, p := range q[1:] {
			if p[i].Cmp(a) < 0 {
				a = p[i]
			}
			if p[i].Cmp(b) > 0 {
				b = p[i]
			}
		}
		d[i] = new(big.Rat)
		if a.Sign() > 0 {
			d[i] = a
		} else if b.Sign() < 0 {
			d[i] = b
		}
	}
	return add(mul(d[0], d[0]), mul(d[1], d[1])).Cmp(mul(rhi, rhi)) > 0, nil
}

// Witness proves that the original two-sided bound of the held centroid
// forces a same-face true point strictly inside this finite parameter band.
func (g PolarDomain) Witness(t [3]r3.Vec, beta float64) bool {
	if beta < 0 || proofbound.IsNonFinite(beta) {
		return false
	}
	x, y, z := new(big.Rat), new(big.Rat), new(big.Rat)
	for _, v := range t {
		if !proofbound.FiniteVec(v) {
			return false
		}
		x.Add(x, rat(v.X))
		y.Add(y, rat(v.Y))
		z.Add(z, rat(v.Z))
	}
	three := big.NewRat(3, 1)
	x.Sub(x.Quo(x, three), g.CenterX)
	y.Quo(y, three)
	z.Sub(z.Quo(z, three), g.CenterZ)
	e := rat(beta)
	if sub(y, e).Cmp(g.Bottom) <= 0 || add(y, e).Cmp(g.Top) >= 0 {
		return false
	}
	for i, r := range g.Rays {
		v := sub(mul(r[0], z), mul(r[1], x))
		if i == 1 {
			v.Neg(v)
		}
		if v.Sign() <= 0 || mul(v, v).Cmp(mul(mul(e, e), add(mul(r[0], r[0]), mul(r[1], r[1])))) <= 0 {
			return false
		}
	}
	return true
}

// InsetEnd rotates this minor arc's terminal ray toward its interior. The
// angular correspondence and mesh remain those of the full registered arc.
func (g PolarDomain) InsetEnd(end int) (PolarDomain, bool) {
	t := big.NewRat(1, 1<<22)
	if end == 1 {
		t.Neg(t)
	}
	x, z := g.Rays[end][0], g.Rays[end][1]
	g.Rays[end] = [2]*big.Rat{sub(x, mul(t, z)), add(z, mul(t, x))}
	a, b := g.Rays[0], g.Rays[1]
	return g, sub(mul(a[0], b[1]), mul(a[1], b[0])).Sign() > 0
}

// InsideAngularBoundary proves that the whole triangle's enlarged ball is
// strictly on the retained side of one radial plane.
func (g PolarDomain) InsideAngularBoundary(t [3]r3.Vec, e float64, end int) bool {
	r := g.Rays[end]
	beta := rat(e)
	if beta == nil || beta.Sign() < 0 {
		return false
	}
	limit := mul(mul(beta, beta), add(mul(r[0], r[0]), mul(r[1], r[1])))
	for _, v := range t {
		x, z := sub(rat(v.X), g.CenterX), sub(rat(v.Z), g.CenterZ)
		c := sub(mul(r[0], z), mul(r[1], x))
		if end == 1 {
			c.Neg(c)
		}
		if c.Sign() <= 0 || mul(c, c).Cmp(limit) <= 0 {
			return false
		}
	}
	return true
}

// ExteriorEnd encloses the angular collar outside one native endpoint.
// Its far radial plane must lie beyond every original facet's uncertainty.
func (g PolarDomain) ExteriorEnd(end int) PolarDomain {
	x, z := g.Rays[end][0], g.Rays[end][1]
	t := big.NewRat(1, 32)
	if end == 0 {
		t.Neg(t)
	}
	remote := [2]*big.Rat{sub(x, mul(t, z)), add(z, mul(t, x))}
	if end == 0 {
		g.Rays = [2][2]*big.Rat{remote, g.Rays[0]}
	} else {
		g.Rays = [2][2]*big.Rat{g.Rays[1], remote}
	}
	return g
}
