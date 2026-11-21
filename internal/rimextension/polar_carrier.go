package rimextension

import (
	"context"
	"math/big"
	"strconv"

	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// PolarCarrierAway proves a whole original prism face carrier misses the open
// cylindrical strip. A tangent at an angular endpoint or a cap at either
// axial endpoint may meet its boundary.
func PolarCarrierAway(ctx context.Context, f BoundFace, n PolarSource, lo, hi *big.Rat, m *proofbound.ReplayBudget) (bool, error) {
	p, ok, err := windowPrism(ctx, f, m)
	if !ok || err != nil {
		return false, err
	}
	if windowRole(f, "capStart") || windowRole(f, "capEnd") {
		z := p.DenotedZ0()
		if windowRole(f, "capEnd") {
			z = p.DenotedZ1()
		}
		if z == nil {
			return false, nil
		}
		y := minus(scalar(p.Frame.Origin().Y), z)
		return y.Cmp(lo) <= 0 || y.Cmp(hi) >= 0, nil
	}
	for i, seg := range p.Profile.Outer.Segments {
		if !windowRole(f, "side(0,"+strconv.Itoa(i)+")") {
			continue
		}
		line, ok := seg.(momentinput.LineSeg)
		if !ok {
			return false, nil
		}
		x, z, u, v := scalar(line.Start.U), scalar(line.Start.V), scalar(line.End.U), scalar(line.End.V)
		if x == nil || z == nil || u == nil || v == nil {
			return false, nil
		}
		nx, nz := minus(z, v), minus(u, x)
		if nx.Sign() == 0 && nz.Sign() == 0 {
			return false, nil
		}
		o := p.Frame.Origin()
		h := plus(times(nx, minus(plus(scalar(o.X), x), n.CenterX)), times(nz, minus(plus(scalar(o.Z), z), n.CenterZ)))
		return lineOffOpenArc(n.RadiusSquared, n.Rays, nx, nz, h), nil
	}
	return false, nil
}

func lineOffOpenArc(r2 *big.Rat, rays [2][2]*big.Rat, nx, nz, h *big.Rat) bool {
	// A linear functional on the arc has extrema at the two endpoints and
	// at the parallel or antiparallel normal if it lies in the open sector.
	above, below := true, true
	compare := func(x, z *big.Rat) int {
		d := plus(times(nx, x), times(nz, z))
		if d.Sign() != h.Sign() {
			return d.Sign() - h.Sign()
		}
		return d.Sign() * times(r2, times(d, d)).Cmp(times(times(h, h), plus(times(x, x), times(z, z))))
	}
	for _, r := range rays {
		c := compare(r[0], r[1])
		above = above && c >= 0
		below = below && c <= 0
	}
	for _, s := range []int64{-1, 1} {
		x, z := times(nx, big.NewRat(s, 1)), times(nz, big.NewRat(s, 1))
		if minus(times(rays[0][0], z), times(rays[0][1], x)).Sign() <= 0 || minus(times(x, rays[1][1]), times(z, rays[1][0])).Sign() <= 0 {
			continue
		}
		c := compare(x, z)
		above = above && c > 0
		below = below && c < 0
	}
	return above || below
}
