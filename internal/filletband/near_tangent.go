package filletband

import (
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/survey2d"
)

// This file is agent-docs/loop-fillet-design.md §5.5: the terms an LF5′
// corner charges. The band is built as at an LF5 join, so every reading
// states the structure's own value and widens it by the corner's reach ρ
// (capband.NearTangentCorner, agent-docs/draft-design.md §9.1.1 Lemmas 1 and
// 1′). At every offset t of the sweep, the true corner points (the miter m(t)
// of a convex sliver, the connector arc of a reflex one) and both walks'
// radial feet f(t) lie within 2ρ of each other. Every number here is a
// rational rounded upward once.

// NearTangentMassAllowance bounds the change an LF5′ corner at plane-local
// (cornerU, cornerV), stated within cornerError, makes to the strip's volume
// and in-plane first moments. At offset t the strip pieces of Table CF and the
// true strip differ by a closed chain through the corner, both radial feet and
// the true corner points, whose enclosed area is at most t·ρ + 3ρ²; it lies
// within t + 2ρ of the corner. Integrated over every height of the quarter,
// t <= r, the volume changes by at most r_hi·ρ·(r_hi + 3ρ), and each in-plane
// moment by that times the coordinate's reach. The sign is unknown, so the
// caller widens both ends.
func NearTangentMassAllowance(cornerU, cornerV, cornerError, rho float64, radius proofbound.RatInterval) (
	volume, momentU, momentV *big.Rat, ok bool,
) {
	d, e := proofarith.FloatRat(rho), proofarith.FloatRat(cornerError)
	cu, cv := proofarith.FloatRat(cornerU), proofarith.FloatRat(cornerV)
	if d == nil || e == nil || cu == nil || cv == nil || d.Sign() < 0 || radius.Hi.Sign() <= 0 {
		return nil, nil, nil, false
	}
	volume = new(big.Rat).Add(radius.Hi, new(big.Rat).Mul(big.NewRat(3, 1), d))
	volume.Mul(volume, new(big.Rat).Mul(radius.Hi, d))
	reach := new(big.Rat).Add(e, new(big.Rat).Add(radius.Hi, new(big.Rat).Mul(big.NewRat(2, 1), d)))
	u := new(big.Rat).Add(new(big.Rat).Abs(cu), reach)
	v := new(big.Rat).Add(new(big.Rat).Abs(cv), reach)
	return volume, new(big.Rat).Mul(volume, u), new(big.Rat).Mul(volume, v), true
}

// WidenNearTangentExtent widens an extent's enclosure by the loop's largest
// local reach (Loop.LocalReach, agent-docs/loop-fillet-design.md §5.7) along
// the planar part of direction g: every true band point lies, in its own
// level, within it of a point of the stated patches, and each stated point
// within it of a true one. reach is nil for a loop with no LF5′ corner and no
// radial step.
func WidenNearTangentExtent(reach []float64, g [3]*big.Rat, old Interval) (Interval, error) {
	largest := 0.0
	for _, d := range reach {
		largest = math.Max(largest, d)
	}
	r := proofarith.FloatRat(largest)
	if r == nil {
		return Interval{}, fmt.Errorf(`%w: a near-tangent fillet extent has no corner reach`, decaderr.ErrNotFinite)
	}
	if r.Sign() == 0 {
		return old, nil
	}
	planar, ok := proofbound.SqrtInterval(proofbound.PointInterval(new(big.Rat).Add(
		new(big.Rat).Mul(g[0], g[0]), new(big.Rat).Mul(g[1], g[1]))))
	if !ok {
		return Interval{}, fmt.Errorf(`%w: a near-tangent fillet extent has no planar direction bound`, decaderr.ErrUnsupported)
	}
	return proofbound.IntervalWiden(old, new(big.Rat).Mul(planar.Hi, r)), nil
}

// NearTangentNormalTurn bounds how far the in-plane outward normal of the
// true band strays, near LF5′ corner k, from the normals of the two patches
// beside it: at a reflex sliver the horn's normals lie between the two walks'
// normals, at most 2·sin(θ/2) <= ρ/r_lo apart; at a convex sliver a toroidal
// patch's true window ends at m(t), at most ρ along a moved circle of radius
// at least its least moved radius, and a cylinder's normal does not change.
// radius is the fillet radius's span. ok is false where a moved radius is not
// proven positive or a number does not lift.
func NearTangentNormalTurn(walks []survey2d.SideWalk, k int, rho float64, radius proofbound.RatInterval) (*big.Rat, bool) {
	d := proofarith.FloatRat(rho)
	if d == nil || d.Sign() < 0 || radius.Lo.Sign() <= 0 {
		return nil, false
	}
	out := new(big.Rat).Quo(d, radius.Lo)
	n := len(walks)
	for _, w := range []survey2d.SideWalk{walks[(k+n-1)%n], walks[k]} {
		if !w.IsCircular() {
			continue
		}
		lo, ok := movedRadiusLower(w, walks[k].StartU, walks[k].StartV, radius.Hi)
		if !ok {
			return nil, false
		}
		if q := new(big.Rat).Quo(d, lo); q.Cmp(out) > 0 {
			out = q
		}
	}
	return out, true
}

// movedRadiusLower is the least radius circular walk w's carrier reaches
// over offsets up to reach, read about its recorded centre through the corner
// (u, v): |v − C| − reach for a walk that holds its material inside, |v − C|
// otherwise. ok is false where that is not positive.
func movedRadiusLower(w survey2d.SideWalk, u, v float64, reach *big.Rat) (*big.Rat, bool) {
	pu, pv := proofarith.FloatRat(u), proofarith.FloatRat(v)
	cu, cv := proofarith.FloatRat(w.CU), proofarith.FloatRat(w.CV)
	if pu == nil || pv == nil || cu == nil || cv == nil {
		return nil, false
	}
	du, dv := new(big.Rat).Sub(pu, cu), new(big.Rat).Sub(pv, cv)
	r, ok := proofbound.SqrtInterval(proofbound.PointInterval(new(big.Rat).Add(new(big.Rat).Mul(du, du), new(big.Rat).Mul(dv, dv))))
	if !ok {
		return nil, false
	}
	lo := new(big.Rat).Set(r.Lo)
	if w.Th1 > w.Th0 {
		lo.Sub(lo, reach)
	}
	return lo, lo.Sign() > 0
}
