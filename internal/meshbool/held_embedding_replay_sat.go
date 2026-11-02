package meshbool

import (
	"context"
	"math"

	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
)

type heldSATAxis struct {
	x, y   FloatInterval
	lo, hi float64
}
type heldSATFacet [3][3]heldSATAxis

func heldSATDot(x, y FloatInterval, p r3.Vec, u, v int) FloatInterval {
	return x.Mul(FivPoint(AxisCoord(p, u))).Add(y.Mul(FivPoint(AxisCoord(p, v))))
}

// heldSATBuild encloses every edge-axis and its own triangle span.
func heldSATBuild(t [3]r3.Vec) heldSATFacet {
	var out heldSATFacet
	for axis := range 3 {
		u, v := (axis+1)%3, (axis+2)%3
		for e := range 3 {
			a, b := t[e], t[(e+1)%3]
			dx := FivPoint(AxisCoord(b, u)).Sub(FivPoint(AxisCoord(a, u)))
			dy := FivPoint(AxisCoord(b, v)).Sub(FivPoint(AxisCoord(a, v)))
			x, y := FloatInterval{Lo: -dy.Hi, Hi: -dy.Lo}, dx
			lo, hi := math.Inf(1), math.Inf(-1)
			for _, p := range t {
				z := heldSATDot(x, y, p, u, v)
				lo = min(lo, z.Lo)
				hi = max(hi, z.Hi)
			}
			out[axis][e] = heldSATAxis{x: x, y: y, lo: lo, hi: hi}
		}
	}
	return out
}

// heldSATSeparated only proves disjointness; uncertainty reaches the old path.
func heldSATSeparated(ctx context.Context, a, b [3]r3.Vec, ca, cb *heldSATFacet, normal proof.Xpt, m proof.ReplayKeyBudget) (bool, error) {
	u, v, err := replayProjectionAxes(ctx, normal, m)
	if err != nil {
		return false, err
	}
	axis := 3 - u - v
	for side := range 2 {
		own, other := ca, b
		if side == 1 {
			own, other = cb, a
		}
		for _, ax := range own[axis] {
			if err := m.SpendWork(ctx, 256); err != nil {
				return false, err
			}
			lo, hi := math.Inf(1), math.Inf(-1)
			for _, p := range other {
				z := heldSATDot(ax.x, ax.y, p, u, v)
				lo = min(lo, z.Lo)
				hi = max(hi, z.Hi)
			}
			if ax.hi < lo || hi < ax.lo {
				return true, nil
			}
		}
	}
	return false, nil
}
