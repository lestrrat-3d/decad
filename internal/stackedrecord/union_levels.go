package stackedrecord

import (
	"math/big"
	"sort"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// UnionLevel is one distinct boundary level of two stacked operands.
type UnionLevel struct {
	Exact *big.Rat
	Held  float64
	Delta float64
}

// UnionLevels sorts both operands' slab boundaries on A's axis exactly.
// B's shifted levels round once, with that rounding added to their axial
// displacement. An exact tie keeps A's held value and the larger bound.
// Distinct exact levels that round to one held value cannot form a slab.
func UnionLevels(a, b []Slab, shift *big.Rat) ([]UnionLevel, bool) {
	var levels []UnionLevel
	add := func(z, delta float64, shifted bool) bool {
		exact := proofarith.FloatRat(z)
		if exact == nil {
			return false
		}
		held := z
		if shifted {
			exact.Add(exact, shift)
			held, _ = exact.Float64()
			if round := proofarith.RationalFloatError(exact, held); round != 0 {
				if delta == 0 {
					delta = round
				} else {
					delta = proofbound.AbsSumUpper(delta, round)
				}
			}
		}
		levels = append(levels, UnionLevel{Exact: exact, Held: held, Delta: delta})
		return true
	}
	for _, op := range []struct {
		slabs   []Slab
		shifted bool
	}{{a, false}, {b, true}} {
		for i, slab := range op.slabs {
			if i == 0 && !add(slab.Z0, slab.Z0Delta, op.shifted) {
				return nil, false
			}
			if !add(slab.Z1, slab.Z1Delta, op.shifted) {
				return nil, false
			}
		}
	}
	sort.SliceStable(levels, func(i, j int) bool { return levels[i].Exact.Cmp(levels[j].Exact) < 0 })
	out := levels[:0]
	for _, level := range levels {
		if n := len(out); n > 0 && out[n-1].Exact.Cmp(level.Exact) == 0 {
			out[n-1].Delta = max(out[n-1].Delta, level.Delta)
			continue
		}
		out = append(out, level)
	}
	for i := 1; i < len(out); i++ {
		if out[i-1].Held >= out[i].Held {
			return nil, false
		}
	}
	return out, true
}

// UnionSlabOf finds the slab covering [lo, hi] after shifting its levels.
// It returns -1 when the operand does not reach that interval.
func UnionSlabOf(slabs []Slab, shift, lo, hi *big.Rat) int {
	for i, slab := range slabs {
		z0, z1 := proofarith.FloatRat(slab.Z0), proofarith.FloatRat(slab.Z1)
		z0.Add(z0, shift)
		z1.Add(z1, shift)
		if z0.Cmp(lo) <= 0 && z1.Cmp(hi) >= 0 {
			return i
		}
	}
	return -1
}
