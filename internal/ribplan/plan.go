// Package ribplan computes the footprint of a rib from the line that names it
// (agent-docs/rib-design.md). It holds plain sketch-plane arithmetic and
// touches no body, document or sketch.
package ribplan

import (
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/r3"
)

// Footprint returns the four corners, in sketch-plane coordinates and in
// counter-clockwise order, of the rectangle that is the line (u0, v0)–(u1, v1)
// thickened by thickness across its width, centred on the line, and lengthened
// by overrun beyond each end. thickness is positive and overrun non-negative,
// both in millimetres. A line of zero length names no direction and is
// ErrDegenerate.
func Footprint(u0, v0, u1, v1, thickness, overrun float64) ([4][2]float64, error) {
	var corners [4][2]float64
	if !positive(thickness) {
		return corners, fmt.Errorf(`%w: the rib thickness must be positive and finite`, decaderr.ErrDegenerate)
	}
	if overrun < 0 || math.IsInf(overrun, 0) || math.IsNaN(overrun) {
		return corners, fmt.Errorf(`%w: the rib overrun must be non-negative and finite`, decaderr.ErrDegenerate)
	}
	du, dv := u1-u0, v1-v0
	length := math.Hypot(du, dv)
	if !positive(length) {
		return corners, fmt.Errorf(`%w: the rib line has no length`, decaderr.ErrDegenerate)
	}
	tu, tv := du/length, dv/length
	half := thickness / 2
	nu, nv := -tv*half, tu*half
	au, av := u0-tu*overrun, v0-tv*overrun
	bu, bv := u1+tu*overrun, v1+tv*overrun
	corners = [4][2]float64{
		{au - nu, av - nv},
		{bu - nu, bv - nv},
		{bu + nu, bv + nv},
		{au + nu, av + nv},
	}
	return corners, nil
}

// Band is an arc thickened into an annular sector: the region between the
// radii Inner and Outer about (CentreU, CentreV), counter-clockwise from the
// angle Start to the angle End, both in radians.
type Band struct {
	CentreU, CentreV float64
	Inner, Outer     float64
	Start, End       float64
}

// arcTolerance is the relative difference allowed between the two radii an arc's
// end points imply.
const arcTolerance = 1e-9

// ArcBand returns the band of the arc about (cu, cv) that runs counter-clockwise
// from (su, sv) to (eu, ev), thickened by thickness across its width and centred
// on the arc, and lengthened along the arc by overrun beyond each end. thickness
// is positive and overrun non-negative, both in millimetres. The arc's end points
// must lie on one circle about its centre, a positive radius, and the thickened
// band must keep a positive inner radius and not wrap past a full turn once
// lengthened; any other arc is ErrDegenerate.
func ArcBand(cu, cv, su, sv, eu, ev, thickness, overrun float64) (Band, error) {
	if !positive(thickness) {
		return Band{}, fmt.Errorf(`%w: the rib thickness must be positive and finite`, decaderr.ErrDegenerate)
	}
	if overrun < 0 || math.IsInf(overrun, 0) || math.IsNaN(overrun) {
		return Band{}, fmt.Errorf(`%w: the rib overrun must be non-negative and finite`, decaderr.ErrDegenerate)
	}
	rs, re := math.Hypot(su-cu, sv-cv), math.Hypot(eu-cu, ev-cv)
	if !positive(rs) || !positive(re) || math.Abs(rs-re) > arcTolerance*math.Max(rs, re) {
		return Band{}, fmt.Errorf(`%w: the arc's end points do not lie on one circle about its centre`, decaderr.ErrDegenerate)
	}
	radius := (rs + re) / 2
	inner := radius - thickness/2
	if !positive(inner) {
		return Band{}, fmt.Errorf(`%w: a %g mm rib is too thick for an arc of radius %g mm: its inner edge reaches the centre`,
			decaderr.ErrDegenerate, thickness, radius)
	}
	start := math.Atan2(sv-cv, su-cu)
	sweep := math.Mod(math.Atan2(ev-cv, eu-cu)-start, 2*math.Pi)
	if sweep <= 0 {
		sweep += 2 * math.Pi
	}
	extra := overrun / radius
	if sweep+2*extra >= 2*math.Pi {
		return Band{}, fmt.Errorf(`%w: the rib would wrap a full turn of its arc`, decaderr.ErrDegenerate)
	}
	return Band{CentreU: cu, CentreV: cv, Inner: inner, Outer: radius + thickness/2,
		Start: start - extra, End: start + sweep + extra}, nil
}

// Point returns the point at the angle a on the circle of radius r about the
// band's centre.
func (b Band) Point(r, a float64) (u, v float64) {
	return b.CentreU + r*math.Cos(a), b.CentreV + r*math.Sin(a)
}

// Disjoint reports whether the boxes [aLo, aHi] and [bLo, bHi] share no point
// once each is grown by slack on every side. A true answer is certain; a false
// answer proves nothing.
func Disjoint(aLo, aHi, bLo, bHi r3.Vec, slack float64) bool {
	return aHi.X+slack < bLo.X-slack || bHi.X+slack < aLo.X-slack ||
		aHi.Y+slack < bLo.Y-slack || bHi.Y+slack < aLo.Y-slack ||
		aHi.Z+slack < bLo.Z-slack || bHi.Z+slack < aLo.Z-slack
}

// AddsNothing reports whether a volume reading of after, value ± bound, is
// certifiably no larger than a reading of before. A true answer is certain; a
// false answer proves nothing.
func AddsNothing(before, beforeBound, after, afterBound float64) bool {
	return after+afterBound <= before-beforeBound
}

func positive(x float64) bool { return x > 0 && !math.IsInf(x, 0) && !math.IsNaN(x) }
