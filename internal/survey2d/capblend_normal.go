package survey2d

import (
	"math/big"

	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// WindowReachesDirection decides, over arcs each shorter than a half turn,
// whether the direction (dx, dy) lies inside the window the given cosines and
// sines bound. It returns the proven answer first and the possible one second,
// and never collapses the two: a straddling cross product leaves the direction
// possible but unproven, which is exactly the case an extreme may only widen an
// enclosure with rather than fix an end of it.
//
// A zero direction — the constant form, a = b = 0 — makes both cross products
// exactly zero and so reads as proven inside, which is right: every azimuth
// attains the constant.
func WindowReachesDirection(coss, sins []proofbound.RatInterval, dx, dy *big.Rat) (bool, bool) {
	sure, maybe := false, false
	for j := 0; j+1 < len(coss); j++ {
		// The cross product of the arc's start with the direction, then of the
		// direction with the arc's end: both non-negative places it between them.
		from := proofbound.IntervalSub(proofbound.IntervalScale(coss[j], dy), proofbound.IntervalScale(sins[j], dx))
		to := proofbound.IntervalSub(proofbound.IntervalScale(sins[j+1], dx), proofbound.IntervalScale(coss[j+1], dy))
		if from.Lo.Sign() >= 0 && to.Lo.Sign() >= 0 {
			sure = true
		}
		if from.Hi.Sign() >= 0 && to.Hi.Sign() >= 0 {
			maybe = true
		}
	}
	return sure, maybe
}

// PlacedFrameMap is a prism payload's plane-local to world map, evaluated in
// EXACT arithmetic on the held frame and placement numbers alone.
// prismPayload.point rounds that map twice — once through the frame, once
// through the placement — and this is the map those roundings approximate,
// which is the map the payload DENOTES: a placement re-evaluates the record and
// stores its own coordinates, so the held numbers ARE what they denote (the same
// rule normal_bound.go states).
type PlacedFrameMap struct {
	Origin, Du, Dv, Dn IvVec3
}

// point is the exact image of a plane-local (u, v) at height z.
func (m PlacedFrameMap) Point(u, v, z *big.Rat) IvVec3 {
	return IvVec3Add(m.Origin, IvVec3Add(
		IvVec3Mul(m.Du, proofbound.PointInterval(u)),
		IvVec3Add(IvVec3Mul(m.Dv, proofbound.PointInterval(v)), IvVec3Mul(m.Dn, proofbound.PointInterval(z))),
	))
}

func RatMin(a, b *big.Rat) *big.Rat {
	if a.Cmp(b) <= 0 {
		return a
	}
	return b
}

func RatMax(a, b *big.Rat) *big.Rat {
	if a.Cmp(b) >= 0 {
		return a
	}
	return b
}
