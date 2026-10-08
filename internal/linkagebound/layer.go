package linkagebound

import (
	"math/big"

	"github.com/lestrrat-3d/decad/internal/motionbound"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// LayerJoint is the exact axis and motion classification of one linkage joint.
type LayerJoint struct {
	Axis       motionbound.RatVec
	Revolute   bool
	HeldAtZero bool
}

// layerKeeps reports whether a joint preserves a·x for every point x.
func layerKeeps(j LayerJoint, a motionbound.RatVec) bool {
	if j.Revolute {
		return ZeroVec(Cross(a, j.Axis))
	}
	return Dot(a, j.Axis).Sign() == 0
}

// LayerAxes returns the directions that every moving joint on path preserves.
// A path with no moving revolute tries the coordinate axes and the cross
// product of its first two non-parallel slide directions.
func LayerAxes(joints []LayerJoint, path []int) []motionbound.RatVec {
	var moving []int
	for _, i := range path {
		if !joints[i].HeldAtZero {
			moving = append(moving, i)
		}
	}
	var candidates []motionbound.RatVec
	for _, i := range moving {
		if joints[i].Revolute {
			candidates = []motionbound.RatVec{joints[i].Axis}
			break
		}
	}
	if candidates == nil {
		one, zero := big.NewRat(1, 1), new(big.Rat)
		candidates = []motionbound.RatVec{{one, zero, zero}, {zero, one, zero}, {zero, zero, one}}
		for n, i := range moving {
			for _, j := range moving[n+1:] {
				if c := Cross(joints[i].Axis, joints[j].Axis); !ZeroVec(c) {
					candidates = append(candidates, c)
					break
				}
			}
		}
	}
	var out []motionbound.RatVec
	for _, a := range candidates {
		keeps := true
		for _, i := range moving {
			if !layerKeeps(joints[i], a) {
				keeps = false
				break
			}
		}
		if keeps {
			out = append(out, a)
		}
	}
	return out
}

// LayerExtent returns the exact a-extents of an inflated box.
func LayerExtent(boxLo, boxHi, a motionbound.RatVec) (lo, hi *big.Rat) {
	for n, x := range BoxCorners(boxLo, boxHi) {
		v := Dot(a, x)
		if n == 0 || v.Cmp(lo) < 0 {
			lo = v
		}
		if n == 0 || v.Cmp(hi) > 0 {
			hi = v
		}
	}
	return lo, hi
}

// LayerLower returns the largest proven layer gap over all admitted axes.
// Extent reads the two bodies in that order for each candidate axis.
func LayerLower(joints []LayerJoint, path []int,
	extent func(motionbound.RatVec) (xLo, xHi, yLo, yHi *big.Rat, ok bool),
) (float64, bool) {
	best := 0.0
	for _, a := range LayerAxes(joints, path) {
		xLo, xHi, yLo, yHi, ok := extent(a)
		if !ok {
			continue
		}
		w := new(big.Rat).Sub(yLo, xHi)
		if alt := new(big.Rat).Sub(xLo, yHi); alt.Cmp(w) > 0 {
			w = alt
		}
		if w.Sign() <= 0 {
			continue
		}
		norm := sqrtUpRat(axisSq(a))
		if norm == nil {
			continue
		}
		if lower := proofbound.RatFloatDown(w.Quo(w, norm)); lower > best {
			best = lower
		}
	}
	return best, best > 0
}

// SweptBoxesLower returns the largest positive per-axis gap, rounded down.
func SweptBoxesLower(aLo, aHi, bLo, bHi motionbound.RatVec) (float64, bool) {
	var best *big.Rat
	for i := range 3 {
		for _, gap := range []*big.Rat{new(big.Rat).Sub(bLo[i], aHi[i]), new(big.Rat).Sub(aLo[i], bHi[i])} {
			if gap.Sign() > 0 && (best == nil || gap.Cmp(best) > 0) {
				best = gap
			}
		}
	}
	if best == nil {
		return 0, false
	}
	lower := proofbound.RatFloatDown(best)
	return lower, lower > 0
}
