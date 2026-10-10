package box

import (
	"math"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// DisjointGap carries the minimum gap and a lower bound on the pair diameter.
type DisjointGap struct {
	Lo, Hi   float64
	Exact    bool
	Diameter float64
}

// CertifiedDisjointGap computes the bounded plane gap of two admitted boxes.
// Near contact is left to the general pair kernel.
func CertifiedDisjointGap(amin, amax, bmin, bmax r3.Vec) (DisjointGap, bool) {
	axisGap := func(amin, amax, bmin, bmax float64) proofbound.BoundedScalar {
		if amax < bmin {
			return proofbound.BoundedSub(proofbound.ExactScalar(bmin), proofbound.ExactScalar(amax))
		}
		if bmax < amin {
			return proofbound.BoundedSub(proofbound.ExactScalar(amin), proofbound.ExactScalar(bmax))
		}
		return proofbound.ExactScalar(0)
	}
	dx := axisGap(amin.X, amax.X, bmin.X, bmax.X)
	dy := axisGap(amin.Y, amax.Y, bmin.Y, bmax.Y)
	dz := axisGap(amin.Z, amax.Z, bmin.Z, bmax.Z)
	squared := proofbound.BoundedAdd(proofbound.BoundedAdd(proofbound.BoundedMul(dx, dx), proofbound.BoundedMul(dy, dy)), proofbound.BoundedMul(dz, dz))
	gap := proofbound.BoundedSqrt(squared)
	lo, hi := proofbound.BoundedEnds(gap)
	lo = math.Max(0, lo)
	scale := max(1, math.Abs(amin.X), math.Abs(amin.Y), math.Abs(amin.Z),
		math.Abs(amax.X), math.Abs(amax.Y), math.Abs(amax.Z),
		math.Abs(bmin.X), math.Abs(bmin.Y), math.Abs(bmin.Z),
		math.Abs(bmax.X), math.Abs(bmax.Y), math.Abs(bmax.Z))
	if proofbound.IsNonFinite(hi) || lo <= 1e-9*scale {
		return DisjointGap{}, false
	}
	// The farthest distance within or between two boxes occurs at corners.
	// Use downward endpoints so the reference diameter never overstates it.
	distanceLower := func(x, y, z proofbound.BoundedScalar) float64 {
		sum := proofbound.BoundedAdd(proofbound.BoundedAdd(proofbound.BoundedMul(x, x), proofbound.BoundedMul(y, y)), proofbound.BoundedMul(z, z))
		lower, _ := proofbound.BoundedEnds(proofbound.BoundedSqrt(sum))
		return math.Max(0, lower)
	}
	span := func(lo, hi float64) proofbound.BoundedScalar {
		return proofbound.BoundedSub(proofbound.ExactScalar(hi), proofbound.ExactScalar(lo))
	}
	far := func(amin, amax, bmin, bmax float64) proofbound.BoundedScalar {
		left := span(bmin, amax)
		right := span(amin, bmax)
		if left.Value >= right.Value {
			return left
		}
		return right
	}
	diam := max(
		distanceLower(span(amin.X, amax.X), span(amin.Y, amax.Y), span(amin.Z, amax.Z)),
		distanceLower(span(bmin.X, bmax.X), span(bmin.Y, bmax.Y), span(bmin.Z, bmax.Z)),
		distanceLower(
			far(amin.X, amax.X, bmin.X, bmax.X),
			far(amin.Y, amax.Y, bmin.Y, bmax.Y),
			far(amin.Z, amax.Z, bmin.Z, bmax.Z),
		),
	)
	if proofbound.IsNonFinite(diam) {
		return DisjointGap{}, false
	}
	return DisjointGap{Lo: lo, Hi: hi, Exact: gap.Bound == 0, Diameter: diam}, true
}
