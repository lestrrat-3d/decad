package sweeppath

import (
	"math/big"

	"github.com/lestrrat-3d/decad/internal/motionbound"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/radiusmemo"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
)

// NodeBoundsCorners returns the eight corners of node's built bounds at its
// current placement, each coordinate pushed outward by the bounds' error.
// node must be non-nil.
func NodeBoundsCorners(node *topo.Body) ([8]proofarith.DyV3, bool) {
	box := node.Bounds
	return CheckedBoundsCorners(box.Min, box.Max, box.Bound)
}

// RotationalSweepRadius is §4.1's ρ: an upper bound, rounded to a float, on
// the largest distance of the From-mapped inflated bounds corners from the
// axis line through center, the root of max |(corner − center) × axis|² /
// |axis|². Every squared cross product shares the one denominator |axis|², so
// the largest is chosen by its numerator, which is exact dyadic arithmetic
// for the dyadic corners, pivot and axis every caller holds; one division
// then gives the same rational the per-corner quotients' maximum is. A body's
// readings are memoized on its node (internal/radiusmemo), since a step reads
// each path's radius both for its swept box and for its sweep. node must be
// non-nil.
func RotationalSweepRadius(node *topo.Body, from r3.Transform, center r3.Vec,
	axis motionbound.RatVec) (*big.Rat, bool) {
	var dyAxis proofarith.DyV3
	for k := range 3 {
		component, ok := proofarith.DyOfRat(axis[k])
		if !ok {
			corners, ok := NodeBoundsCorners(node)
			if !ok {
				return nil, false
			}
			return motionbound.SweepRadiusRational(corners[:], from, center, axis)
		}
		dyAxis[k] = component
	}
	if !motionbound.MemosOn() {
		corners, ok := NodeBoundsCorners(node)
		if !ok {
			return nil, false
		}
		return motionbound.SweepRadiusDyadic(corners[:], from, center, dyAxis)
	}
	key := radiusmemo.NewRadiusKey(from, center, dyAxis)
	if radius, ok, hit := node.SweepRadii.Load(key); hit {
		if !ok {
			return nil, false
		}
		return proofarith.FloatRat(radius), true
	}
	corners, ok := NodeBoundsCorners(node)
	if !ok {
		node.SweepRadii.Store(key, 0, false)
		return nil, false
	}
	radius, ok := motionbound.SweepRadiusDyadic(corners[:], from, center, dyAxis)
	held := 0.0
	if ok {
		held, _ = radius.Float64()
	}
	node.SweepRadii.Store(key, held, ok)
	return radius, ok
}
