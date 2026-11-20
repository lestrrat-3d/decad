package rimextension

import (
	"context"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// ExtensionBound encloses the fixed-X change in ordinate on each extended
// cylinder triangle and the corresponding displacement on the sloping bevel.
func ExtensionBound(ctx context.Context, a Admission, m Mesh, group int, budget *proofbound.ReplayBudget) (float64, bool, error) {
	n, ok, e := windowNativeFaces(ctx, a, budget)
	if !ok || e != nil {
		return 0, false, e
	}
	h, ok := newChart(a.Section)
	if !ok {
		return 0, false, nil
	}
	r2 := times(n.r, n.r)
	worst := new(big.Rat)
	found := false
	for i, t := range m.Triangles {
		if i%256 == 0 && ctx.Err() != nil {
			return 0, false, ctx.Err()
		}
		if m.Sources[i] != group {
			continue
		}
		if e := budget.ReserveBundle(ctx, 128, 3, 32768); e != nil {
			return 0, false, e
		}
		found = true
		var x, z, norm [3]*big.Rat
		for j, k := range t {
			v := m.Vertices[k]
			x[j] = minus(scalar(v.X), n.cx)
			z[j] = signed(minus(scalar(v.Z), n.cz), n.out)
			if z[j].Sign() <= 0 {
				return 0, false, nil
			}
			norm[j] = plus(times(x[j], x[j]), times(z[j], z[j]))
		}
		lo, hi := norm[0], norm[0]
		minz, maxx := z[0], wabs(x[0])
		for j := range t {
			k := (j + 1) % 3
			lo = wmin(lo, norm[j])
			hi = wmax(hi, norm[j])
			minz = wmin(minz, z[j])
			maxx = wmax(maxx, wabs(x[j]))
			dx, dz := minus(x[k], x[j]), minus(z[k], z[j])
			l2 := plus(times(dx, dx), times(dz, dz))
			if l2.Sign() == 0 {
				continue
			}
			u := new(big.Rat).Neg(plus(times(x[j], dx), times(z[j], dz)))
			if u.Sign() > 0 && u.Cmp(l2) < 0 {
				lo = wmin(lo, minus(norm[j], divide(times(u, u), l2)))
			}
		}
		rad := minus(r2, times(maxx, maxx))
		if rad.Sign() <= 0 {
			return 0, false, nil
		}
		sep := divide(wmax(wabs(minus(r2, lo)), wabs(minus(r2, hi))), plus(minz, scalar(proofbound.RatSqrtDown(rad))))
		worst = wmax(worst, sep)
	}
	return proofbound.RatFloatUp(times(worst, plus(scalar(1), wabs(h.gradient)))), found, nil
}
