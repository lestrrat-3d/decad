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
