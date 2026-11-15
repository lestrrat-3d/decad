package filletband

import (
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/capband"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
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

// NearTangentAreaAllowance bounds the area change an LF5′ corner makes to
// each pipe patch beside it: π·r_hi·ρ. At a convex sliver the patch's level
// curve at meridian angle φ ends at m(t) instead of its radial foot, along
// its own carrier: at most 2ρ along a line, an arc of chord at most ρ (under
// (π/2)·ρ long) on a circle, and the area element is that length times r·dφ.
// At a reflex sliver each patch keeps its radial foot exactly, and the horn
// between the two feet, θ·r²·(π/2 − 1) with θ <= π·ρ/(2r), is under
// (π/2)(π/2 − 1)·r·ρ.
func NearTangentAreaAllowance(rho float64, radius proofbound.RatInterval) (*big.Rat, bool) {
	d := proofarith.FloatRat(rho)
	if d == nil || d.Sign() < 0 || radius.Hi.Sign() <= 0 {
		return nil, false
	}
	out := new(big.Rat).Mul(proofbound.PiUpper, radius.Hi)
	return out.Mul(out, d), true
}

// NearTangentReach is twice the largest LF5′ reach of the loop, the distance
// within which every true corner point lies of each radial foot at every
// offset. It is zero for a loop with no LF5′ corner.
func NearTangentReach(rho []float64) float64 {
	largest := 0.0
	for _, d := range rho {
		largest = math.Max(largest, d)
	}
	return proofbound.ProductUpper(2, largest)
}

// WidenNearTangentExtent widens an extent's enclosure by the LF5′ corners'
// reach along the planar part of direction g: every true band point lies
// within NearTangentReach, in its own level, of a point of the patches the
// structure states, and each of those within it of a true one.
func WidenNearTangentExtent(rho []float64, g [3]*big.Rat, old Interval) (Interval, error) {
	reach := proofarith.FloatRat(NearTangentReach(rho))
	if reach == nil {
		return Interval{}, fmt.Errorf(`%w: a near-tangent fillet extent has no corner reach`, decaderr.ErrNotFinite)
	}
	if reach.Sign() == 0 {
		return old, nil
	}
	planar, ok := proofbound.SqrtInterval(proofbound.PointInterval(new(big.Rat).Add(
		new(big.Rat).Mul(g[0], g[0]), new(big.Rat).Mul(g[1], g[1]))))
	if !ok {
		return Interval{}, fmt.Errorf(`%w: a near-tangent fillet extent has no planar direction bound`, decaderr.ErrUnsupported)
	}
	return proofbound.IntervalWiden(old, new(big.Rat).Mul(planar.Hi, reach)), nil
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

// LoopNearTangentRho reads the LF5′ reach of every corner of a loop the offset
// joins resolve as G1: ρ where capband.NearTangentCorner admits a join that
// capband.JoinIsG1 does not prove exactly tangent, and zero at every other
// corner, including a G1 join the near-tangent rule refuses (the build's SF1
// refuses that loop). It returns nil for a loop with no LF5′ corner.
func LoopNearTangentRho(budget *proofbound.WorkBudget, loop sectionrecord.LoopRecord, walks []survey2d.SideWalk,
	r, rDelta, tol float64) ([]float64, error) {
	n := len(walks)
	if n == 0 || (n == 1 && walks[0].Closed) {
		return nil, nil
	}
	joins, err := OffsetJoins(budget, walks, r, 0, tol)
	if err != nil {
		return nil, err
	}
	var rho []float64
	for k, j := range joins {
		if err := survey2d.WallBudgetStep(budget); err != nil {
			return nil, err
		}
		if !j.G1 || j.Arc {
			continue
		}
		prevSeg, curSeg, err := cornerSegments(loop, walks[(k+n-1)%n], walks[k])
		if err != nil {
			return nil, err
		}
		if capband.JoinIsG1(prevSeg, curSeg) {
			continue
		}
		charge, why, _ := capband.NearTangentCorner(prevSeg, curSeg, r, r, rDelta)
		if why != "" {
			continue
		}
		if rho == nil {
			rho = make([]float64, n)
		}
		rho[k] = charge.Rho
	}
	return rho, nil
}
