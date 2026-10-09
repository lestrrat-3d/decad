package coilshell

import (
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/coil"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/r3"
)

// Centre is a circular segment's recorded centre as exact rationals.
func Centre(seg coil.Segment) (*big.Rat, *big.Rat) {
	var c sectionrecord.Point2
	switch s := seg.Record.(type) {
	case sectionrecord.ArcSeg:
		c = s.Center
	case sectionrecord.CircleSeg:
		c = s.Center
	}
	return new(big.Rat).SetFloat64(c.U), new(big.Rat).SetFloat64(c.V)
}

// RimCircle is one held rim circle: its centre and unit axis, and the
// Edge.curveBound of the circle the record denotes about it.
type RimCircle struct {
	Centre, Axis r3.Vec
	Bound        float64
	Bounded      bool
}

// RimCircles lifts a circular segment's rim circle, centre (cu, cv) and
// radius enclosed by radius, through the screw motion at θ = 0 and θ = Θ
// (§3, §5.3). A plane vector x turns to x + (x·e_r)·w with
// w = (cos Θ − 1)·e_r + σ·sin Θ·Side·N, which carries the centre, the plane's
// in-plane axes U and V, and turns the normal N = Side·e_t to
// cos Θ·N − Side·σ·sin Θ·e_r. Each held centre and axis is the float nearest
// its enclosure, and the held radius is heldRadius; rimCurveBound bounds
// the denoted circle against them.
func RimCircles(rec Record, cu, cv *big.Rat, radius coil.Iv, heldRadius float64) ([2]RimCircle, error) {
	var out [2]RimCircle
	zero := coil.Point(new(big.Rat))
	one := coil.Point(big.NewRat(1, 1))
	rho, _ := rec.Axis.Coords(cu, cv)
	erU, erV := rec.Axis.Radial()
	sinT, cosT := coil.TurnSinCos(rec.Turns)
	cm1 := proofbound.IntervalSub(cosT, one)
	slide := coil.Point(new(big.Rat).Mul(rec.Pitch, rec.Turns))
	x := proofbound.IntervalAdd(coil.Point(cu), proofbound.IntervalAdd(proofbound.IntervalMul(proofbound.IntervalMul(cm1, rho), erU), proofbound.IntervalMul(slide, rec.Axis.DU)))
	y := proofbound.IntervalAdd(coil.Point(cv), proofbound.IntervalAdd(proofbound.IntervalMul(proofbound.IntervalMul(cm1, rho), erV), proofbound.IntervalMul(slide, rec.Axis.DV)))
	tilt := big.NewRat(int64(rec.Sigma*rec.Axis.Side), 1)
	z := proofbound.IntervalScale(proofbound.IntervalMul(sinT, rho), tilt)
	held := func(p proofbound.IvVec3, what string) (r3.Vec, error) {
		var v [3]float64
		for k := range 3 {
			h, _, err := Held(p[k], what)
			if err != nil {
				return r3.Vec{}, err
			}
			v[k] = h
		}
		return r3.NewVec(v[0], v[1], v[2]), nil
	}
	turn := proofbound.IntervalScale(sinT, tilt)
	w := [3]coil.Iv{proofbound.IntervalMul(cm1, erU), proofbound.IntervalMul(cm1, erV), turn}
	turned := func(e [3]coil.Iv, along coil.Iv) [3]coil.Iv {
		return [3]coil.Iv{
			proofbound.IntervalAdd(e[0], proofbound.IntervalMul(along, w[0])),
			proofbound.IntervalAdd(e[1], proofbound.IntervalMul(along, w[1])),
			proofbound.IntervalAdd(e[2], proofbound.IntervalMul(along, w[2])),
		}
	}
	e1, e2 := [3]coil.Iv{one, zero, zero}, [3]coil.Iv{zero, one, zero}
	frames := [2]struct {
		centre, normal proofbound.IvVec3
		a, b           [3]coil.Iv
	}{
		{rec.World.Point(coil.Point(cu), coil.Point(cv), zero), rec.World.Vector(zero, zero, one), e1, e2},
		{
			rec.World.Point(x, y, z),
			rec.World.Vector(proofbound.IntervalNeg(proofbound.IntervalMul(turn, erU)), proofbound.IntervalNeg(proofbound.IntervalMul(turn, erV)), cosT),
			turned(e1, erU), turned(e2, erV),
		},
	}
	for i, f := range frames {
		c, err := held(f.centre, "rim centre")
		if err != nil {
			return out, err
		}
		a, err := held(f.normal, "rim axis")
		if err != nil {
			return out, err
		}
		n, ok := a.Normalize()
		if !ok {
			return out, fmt.Errorf(`%w: the coil rim's plane has no normal`, decaderr.ErrUnsupported)
		}
		u := rec.World.Vector(f.a[0], f.a[1], f.a[2])
		v := rec.World.Vector(f.b[0], f.b[1], f.b[2])
		bound, bounded := rimCurveBound(f.centre, u, v, radius, c, n, heldRadius)
		out[i] = RimCircle{Centre: c, Axis: n, Bound: bound, Bounded: bounded}
	}
	return out, nil
}

// rimCurveBound is Edge.curveBound for a coil rim: how far the denoted
// circle {centre + r·(cos α·u + sin α·v)}, r in radius and u, v the images of
// two orthonormal plane vectors under the denoted map, lies from the held
// circle (heldCentre, the unit heldAxis, heldRadius). With D = centre −
// heldCentre and q = r·(cos α·u + sin α·v), a denoted point sits at
// D + q from the held centre. Its axial part is at most
// ax = |A·D| + r·(|A·u| + |A·v|). |q|² = r²·(1 + x) with
// |x| ≤ e = max(||u|² − 1|, ||v|² − 1|) + |u·v|, so ||q| − r| ≤ r·e for e ≤ 1,
// and its radial part sits within |D| + r·e + |r − heldRadius| + ax of
// heldRadius, since dropping the axial part shortens a vector by at most ax.
// The sum 2·ax + |D| + r·e + |r − heldRadius| is rounded up once; it answers
// false where e passes 1 or the bound is not below half the held radius.
func rimCurveBound(centre, u, v proofbound.IvVec3, radius coil.Iv, heldCentre, heldAxis r3.Vec, heldRadius float64) (float64, bool) {
	hc, ok1 := proofbound.IvVec3Of(heldCentre)
	ha, ok2 := proofbound.IvVec3Of(heldAxis)
	if !ok1 || !ok2 {
		return math.Inf(1), false
	}
	d := proofbound.IvVec3Sub(centre, hc)
	one := coil.Point(big.NewRat(1, 1))
	defect := new(big.Rat).Add(
		new(big.Rat).Set(proofbound.RatMax(proofbound.IntervalAbsUpper(proofbound.IntervalSub(proofbound.IvVec3NormSq(u), one)),
			proofbound.IntervalAbsUpper(proofbound.IntervalSub(proofbound.IvVec3NormSq(v), one)))),
		proofbound.IntervalAbsUpper(proofbound.IvVec3Dot(u, v)),
	)
	if defect.Cmp(big.NewRat(1, 1)) > 0 {
		return math.Inf(1), false
	}
	// A is the held axis over its own exact length, read from below.
	norm, ok := proofbound.SqrtFixed(proofbound.IvVec3NormSq(ha).Lo)
	if !ok || norm.Lo.Sign() <= 0 {
		return math.Inf(1), false
	}
	r := radius.Hi
	ax := new(big.Rat).Add(proofbound.IntervalAbsUpper(proofbound.IvVec3Dot(ha, u)), proofbound.IntervalAbsUpper(proofbound.IvVec3Dot(ha, v)))
	ax.Mul(ax, r)
	ax.Add(ax, proofbound.IntervalAbsUpper(proofbound.IvVec3Dot(ha, d)))
	ax.Quo(ax, norm.Lo)
	dLen, ok := proofbound.SqrtFixed(proofbound.IvVec3NormSq(d).Hi)
	if !ok {
		return math.Inf(1), false
	}
	held := new(big.Rat).SetFloat64(heldRadius)
	gap := proofbound.RatMax(new(big.Rat).Abs(new(big.Rat).Sub(radius.Lo, held)), new(big.Rat).Abs(new(big.Rat).Sub(radius.Hi, held)))
	total := new(big.Rat).Mul(ax, big.NewRat(2, 1))
	total.Add(total, dLen.Hi)
	total.Add(total, new(big.Rat).Mul(r, defect))
	total.Add(total, gap)
	bound := proofbound.RatFloatUp(total)
	if proofbound.IsNonFinite(bound) || !(proofbound.ProductUpper(2, bound) < heldRadius) {
		return math.Inf(1), false
	}
	return bound, true
}

// Tangent is segment i's exact walk tangent at its walk start (atStart)
// or its walk end: a line's run, or an arc's radius turned a quarter turn in
// the walk's sense.
func Tangent(rec Record, i int, atStart bool) (*big.Rat, *big.Rat) {
	seg := rec.Profile.Segments[i]
	if !seg.IsArc() {
		v, w := seg.First, rec.Profile.Segments[rec.Profile.Next(i)].First
		return new(big.Rat).Sub(rec.U[w], rec.U[v]), new(big.Rat).Sub(rec.V[w], rec.V[v])
	}
	cu, cv := Centre(seg)
	v := seg.First
	if !atStart {
		v = rec.Profile.Segments[rec.Profile.Next(i)].First
	}
	du, dv := new(big.Rat).Sub(rec.U[v], cu), new(big.Rat).Sub(rec.V[v], cv)
	if seg.Reversed {
		return dv, du.Neg(du)
	}
	return dv.Neg(dv), du
}
