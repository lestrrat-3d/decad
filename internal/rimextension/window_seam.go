package rimextension

import (
	"context"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/loftrecord"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

type nativeSeam struct{ x, toeZ, heelY *big.Rat }

// The X-maximum of a ruled solid is reached only at paired section
// maximizers. Require identical extremal segment structure and one exposed
// vertical lobe in the selected ordinate interval. Its inner end moves
// affinely between the two sections.
func loftWindowSeam(ctx context.Context, p *loftrecord.Payload, shift r3.Vec, capY, sideZ, lowZ, highZ *big.Rat, out int, meter *proofbound.ReplayBudget) (nativeSeam, bool, error) {
	var none nativeSeam
	x, ok, err := LoftWindowXMax(ctx, p, shift.X, meter)
	if err != nil || !ok {
		return none, false, err
	}
	if (out != 1 && out != -1) || p.ConstructionDelta != 0 || len(p.Alignment) > 1 || len(p.Alignment) == 1 && p.Alignment[0] != 0 ||
		len(p.Profile0.Outer.Segments) != len(p.Profile1.Outer.Segments) || lowZ.Cmp(highZ) >= 0 {
		return none, false, nil
	}
	sy, sz := scalar(shift.Y), scalar(shift.Z)
	if sy == nil || sz == nil {
		return none, false, nil
	}
	frames := [2]r3.Frame{p.Frame0, p.Frame1}
	var maxU, baseZ, levelY [2]*big.Rat
	for j, f := range frames {
		o := lift(f.Origin())
		if o[0] == nil || o[1] == nil || o[2] == nil {
			return none, false, nil
		}
		maxU[j] = minus(minus(x, scalar(shift.X)), o[0])
		baseZ[j] = plus(o[2], sz)
		levelY[j] = plus(o[1], sy)
	}
	if levelY[0].Cmp(levelY[1]) == 0 {
		return none, false, nil
	}
	segs := [2][]momentinput.CurveSegment{p.Profile0.Outer.Segments, p.Profile1.Outer.Segments}
	if err := meter.ReserveBundle(ctx, uint64(len(segs[0])), uint64(len(segs[0])), uint64(len(segs[0]))); err != nil {
		return none, false, err
	}
	masks := make([]uint8, len(segs[0]))
	found := false
	var inner [2]*big.Rat
	for i := range segs[0] {
		if err := meter.ReserveBundle(ctx, 48, 0, 49152); err != nil {
			return none, false, err
		}
		var ends [2][2]*big.Rat
		mask := [2]int{}
		for j := range 2 {
			if line, yes := segs[j][i].(momentinput.LineSeg); yes {
				if !(line.TStart == 0 && line.TEnd == 1 || line.TStart == 1 && line.TEnd == 0) {
					return none, false, nil
				}
				a, b := line.Start, line.End
				if line.TStart == 1 {
					a, b = b, a
				}
				for k, pt := range [2]momentinput.Point2{a, b} {
					u, v := scalar(pt.U), scalar(pt.V)
					if u == nil || v == nil {
						return none, false, nil
					}
					if u.Cmp(maxU[j]) == 0 {
						mask[j] |= 1 << k
						ends[j][k] = plus(baseZ[j], v)
					}
				}
			} else if bound := windowLoftSegmentUpper(segs[j][i]); bound == nil || bound.Cmp(maxU[j]) >= 0 {
				return none, false, nil
			}
		}
		if mask[0] != mask[1] {
			return none, false, nil
		}
		masks[i] = uint8(mask[0])
		if mask[0] != 3 {
			continue
		}
		z0, z1 := ends[0], ends[1]
		if z0[0].Cmp(z0[1]) >= 0 || z1[0].Cmp(z1[1]) >= 0 {
			return none, false, nil
		}
		if z0[1].Cmp(lowZ) < 0 && z1[1].Cmp(lowZ) < 0 || z0[0].Cmp(highZ) > 0 && z1[0].Cmp(highZ) > 0 {
			continue
		}
		if found {
			return none, false, nil
		}
		found = true
		if out == 1 {
			if z0[1].Cmp(highZ) <= 0 || z1[1].Cmp(highZ) <= 0 {
				return none, false, nil
			}
			inner = [2]*big.Rat{z0[0], z1[0]}
		} else {
			if z0[0].Cmp(lowZ) >= 0 || z1[0].Cmp(lowZ) >= 0 {
				return none, false, nil
			}
			inner = [2]*big.Rat{z0[1], z1[1]}
		}
	}
	for i, c := range masks {
		if c == 1 && masks[(i+len(masks)-1)%len(masks)] != 3 || c == 2 && masks[(i+1)%len(masks)] != 3 {
			return none, false, nil
		}
	}
	if !found || inner[0].Cmp(inner[1]) == 0 {
		return none, false, nil
	}
	t := divide(minus(capY, levelY[0]), minus(levelY[1], levelY[0]))
	u := divide(minus(sideZ, inner[0]), minus(inner[1], inner[0]))
	for _, q := range []*big.Rat{t, u} {
		if q.Sign() <= 0 || q.Cmp(scalar(1)) >= 0 {
			return none, false, nil
		}
	}
	return nativeSeam{x: x, toeZ: plus(inner[0], times(t, minus(inner[1], inner[0]))), heelY: plus(levelY[0], times(u, minus(levelY[1], levelY[0])))}, true, nil
}
