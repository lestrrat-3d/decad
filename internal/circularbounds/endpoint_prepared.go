package circularbounds

import (
	"math"
	"math/big"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/units"
)

// PreparedEndpoint keeps a recorded circular segment's parameter-independent
// exact bounds for repeated station readings. Its fields are read only after
// construction; At may be called concurrently.
type PreparedEndpoint struct {
	circle    bool
	cu, cv    *big.Rat
	radius    proofbound.RatInterval
	a0, sweep proofbound.RatInterval
}

// PrepareEndpoint builds the exact bounds shared by every endpoint reading.
// A false result means the record cannot state a finite circular endpoint.
func PrepareEndpoint(segment CurveSegment) (PreparedEndpoint, bool) {
	switch seg := segment.(type) {
	case CircleSeg:
		radius, err := seg.Radius.In(units.Millimeter)
		if err != nil {
			return PreparedEndpoint{}, false
		}
		r := proofarith.FloatRat(radius)
		cu, cv := proofarith.FloatRat(seg.Center.U), proofarith.FloatRat(seg.Center.V)
		if r == nil || cu == nil || cv == nil {
			return PreparedEndpoint{}, false
		}
		return PreparedEndpoint{circle: true, cu: cu, cv: cv, radius: proofbound.PointInterval(r)}, true
	case ArcSeg:
		cu, cv := proofarith.FloatRat(seg.Center.U), proofarith.FloatRat(seg.Center.V)
		if cu == nil || cv == nil {
			return PreparedEndpoint{}, false
		}
		dx0 := exactCoordinateDelta(seg.Start.U, seg.Center.U)
		dy0 := exactCoordinateDelta(seg.Start.V, seg.Center.V)
		dx1 := exactCoordinateDelta(seg.End.U, seg.Center.U)
		dy1 := exactCoordinateDelta(seg.End.V, seg.Center.V)
		r2 := new(big.Rat).Add(new(big.Rat).Mul(dx0, dx0), new(big.Rat).Mul(dy0, dy0))
		rLo := proofarith.FloatRat(proofbound.RatSqrtDown(r2))
		rHi := proofarith.FloatRat(proofbound.RatSqrtUp(r2))
		if rLo == nil || rHi == nil {
			return PreparedEndpoint{}, false
		}
		heldDY0 := seg.Start.V - seg.Center.V
		heldDY1 := seg.End.V - seg.Center.V
		a0 := proofbound.Atan2Interval(dy0, dx0, heldDY0 == 0 && math.Signbit(heldDY0))
		a1 := proofbound.Atan2Interval(dy1, dx1, heldDY1 == 0 && math.Signbit(heldDY1))
		sweep := proofbound.IntervalSub(a1, a0)
		heldA0 := math.Atan2(heldDY0, seg.Start.U-seg.Center.U)
		heldA1 := math.Atan2(heldDY1, seg.End.U-seg.Center.U)
		if heldA1-heldA0 <= 0 {
			sweep = proofbound.IntervalAdd(sweep, proofbound.TwoPiInterval())
		}
		return PreparedEndpoint{
			cu: cu, cv: cv, radius: proofbound.Interval(rLo, rHi), a0: a0, sweep: sweep,
		}, true
	default:
		return PreparedEndpoint{}, false
	}
}

// At encloses the segment's denoted point at an exact rational parameter.
// The parameter must not be nil.
func (p PreparedEndpoint) At(t *big.Rat) (proofbound.RatInterval, proofbound.RatInterval, bool) {
	if p.circle {
		if p.radius.Lo.Sign() == 0 {
			return proofbound.PointInterval(p.cu), proofbound.PointInterval(p.cv), true
		}
		sin, cos := quarterTurnSinCos(t)
		return proofbound.IntervalAdd(proofbound.PointInterval(p.cu), proofbound.IntervalScale(cos, p.radius.Lo)),
			proofbound.IntervalAdd(proofbound.PointInterval(p.cv), proofbound.IntervalScale(sin, p.radius.Lo)), true
	}
	sin, cos, ok := proofbound.RadSinCosSpan(proofbound.IntervalAdd(p.a0, proofbound.IntervalScale(p.sweep, t)))
	if !ok {
		return proofbound.RatInterval{}, proofbound.RatInterval{}, false
	}
	return proofbound.IntervalAdd(proofbound.PointInterval(p.cu), proofbound.IntervalMul(p.radius, cos)),
		proofbound.IntervalAdd(proofbound.PointInterval(p.cv), proofbound.IntervalMul(p.radius, sin)), true
}

// CanShareCircleTurn reports whether both endpoints use the same turn-based
// circle proof and need its trigonometric interval at interior stations.
func (p PreparedEndpoint) CanShareCircleTurn(other PreparedEndpoint) bool {
	return p.circle && other.circle && p.radius.Lo.Sign() != 0 && other.radius.Lo.Sign() != 0
}

// AtSharedCircleTurn encloses two circles at the same exact turn. The caller
// checks CanShareCircleTurn and the equality of their recorded parameters.
func (p PreparedEndpoint) AtSharedCircleTurn(other PreparedEndpoint, t *big.Rat) (
	u0, v0, u1, v1 proofbound.RatInterval,
) {
	return p.AtSharedCircleTurnWithMemo(other, t, nil)
}

// AtSharedCircleTurnWithMemo uses a caller-owned memo for repeated turns.
// The caller checks CanShareCircleTurn and equal recorded parameters. A nil
// memo keeps the independent evaluation path. The turn must not be nil, and
// the caller must not share a non-nil memo across concurrent calls.
func (p PreparedEndpoint) AtSharedCircleTurnWithMemo(other PreparedEndpoint, t *big.Rat,
	memo *proofbound.TurnTrigMemo,
) (proofbound.RatInterval, proofbound.RatInterval, proofbound.RatInterval, proofbound.RatInterval) {
	sin, cos := quarterTurnSinCosWithMemo(t, memo)
	u0 := proofbound.IntervalAdd(proofbound.PointInterval(p.cu), proofbound.IntervalScale(cos, p.radius.Lo))
	v0 := proofbound.IntervalAdd(proofbound.PointInterval(p.cv), proofbound.IntervalScale(sin, p.radius.Lo))
	u1 := proofbound.IntervalAdd(proofbound.PointInterval(other.cu), proofbound.IntervalScale(cos, other.radius.Lo))
	v1 := proofbound.IntervalAdd(proofbound.PointInterval(other.cv), proofbound.IntervalScale(sin, other.radius.Lo))
	return u0, v0, u1, v1
}
