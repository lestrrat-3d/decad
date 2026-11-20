package rimextension

import (
	"context"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// SourceWindow encloses the requested region in X, Y and signed Z offset
// from the line through Side0 and Side1, all in world coordinates.
type SourceWindow struct {
	sideError                                    *big.Rat
	Side0, Side1                                 r3.Vec
	XMin, XMax, YMin, YMax, OffsetMin, OffsetMax float64
}

// WindowSourcesCovered checks the preimage of the window against the
// complete held shell. Each vertex bound must cover its incident true facet
// under the caller's two-sided local correspondence.
func WindowSourcesCovered(ctx context.Context, verts []r3.Vec, tris [][3]int, groups []int, beta []float64, allowed map[int]bool, w SourceWindow, meter *proofbound.ReplayBudget) (bool, error) {
	if meter == nil || len(tris) == 0 || len(groups) != len(tris) || len(verts) != len(beta) {
		return false, nil
	}
	if err := meter.ReserveBytes(ctx, 16384); err != nil {
		return false, err
	}
	lo := [3]*big.Rat{scalar(w.XMin), scalar(w.YMin), scalar(w.OffsetMin)}
	hi := [3]*big.Rat{scalar(w.XMax), scalar(w.YMax), scalar(w.OffsetMax)}
	for k := range lo {
		if lo[k] == nil || hi[k] == nil || lo[k].Cmp(hi[k]) >= 0 {
			return false, nil
		}
	}
	x, z, x1, z1 := scalar(w.Side0.X), scalar(w.Side0.Z), scalar(w.Side1.X), scalar(w.Side1.Z)
	if x == nil || z == nil || x1 == nil || z1 == nil || x.Cmp(x1) == 0 {
		return false, nil
	}
	slope := divide(minus(z1, z), minus(x1, x))
	amp := plus(scalar(1), new(big.Rat).Abs(slope))
	for i, t := range tris {
		if err := meter.SpendWork(ctx, 1); err != nil {
			return false, err
		}
		// Authenticated sources may meet the window anywhere on their facets.
		if allowed[groups[i]] {
			continue
		}
		minX, minY, maxX, maxY := math.Inf(1), math.Inf(1), math.Inf(-1), math.Inf(-1)
		errMax := 0.
		for _, id := range t {
			if id < 0 || id >= len(verts) {
				return false, nil
			}
			v, r := verts[id], beta[id]
			if proofbound.IsNonFinite(v.X) || proofbound.IsNonFinite(v.Y) || proofbound.IsNonFinite(v.Z) || proofbound.IsNonFinite(r) || r < 0 {
				return false, nil
			}
			minX, minY = math.Min(minX, v.X), math.Min(minY, v.Y)
			maxX, maxY = math.Max(maxX, v.X), math.Max(maxY, v.Y)
			errMax = math.Max(errMax, r)
		}
		if math.Nextafter(maxX+errMax, math.Inf(1)) < w.XMin || math.Nextafter(minX-errMax, math.Inf(-1)) > w.XMax ||
			math.Nextafter(maxY+errMax, math.Inf(1)) < w.YMin || math.Nextafter(minY-errMax, math.Inf(-1)) > w.YMax {
			continue
		}
		if err := meter.ReserveBytes(ctx, 65536); err != nil {
			return false, err
		}
		if err := meter.SpendWork(ctx, 64); err != nil {
			return false, err
		}
		var a, b [3]*big.Rat
		var radius big.Rat
		for j, id := range t {
			if id < 0 || id >= len(verts) {
				return false, nil
			}
			v := lift(verts[id])
			r := scalar(beta[id])
			if r == nil || r.Sign() < 0 || v[0] == nil || v[1] == nil || v[2] == nil {
				return false, nil
			}
			if r.Cmp(&radius) > 0 {
				radius.Set(r)
			}
			v[2] = minus(minus(v[2], z), times(slope, minus(v[0], x)))
			for k := range v {
				if j == 0 || a[k].Cmp(v[k]) > 0 {
					a[k] = v[k]
				}
				if j == 0 || b[k].Cmp(v[k]) < 0 {
					b[k] = v[k]
				}
			}
		}
		separate := false
		for k := range a {
			r := &radius
			if k == 2 {
				r = times(r, amp)
			}
			if plus(b[k], r).Cmp(lo[k]) < 0 || minus(a[k], r).Cmp(hi[k]) > 0 {
				separate = true
				break
			}
		}
		if !separate {
			if err := meter.ReserveBundle(ctx, 64, 1, 16384); err != nil {
				return false, err
			}
			// Along x +/- z the complete linear window reaches its extreme at a corner.
			sz := 1
			if slope.Sign() < 0 {
				sz = -1
			}
			zEnd := plus(z, times(slope, minus(hi[0], x)))
			zOff := hi[2]
			if sz < 0 {
				zOff = lo[2]
			}
			maximum := plus(hi[0], signed(plus(zEnd, zOff), sz))
			radiusProjection := times(&radius, scalar(proofbound.RatSqrtUp(scalar(2))))
			outside := true
			for _, j := range t {
				v := verts[j]
				outside = outside && minus(plus(scalar(v.X), signed(scalar(v.Z), sz)), radiusProjection).Cmp(maximum) > 0
			}
			if outside {
				continue
			}

			return false, nil
		}
	}
	return true, nil
}
