package offset2d

import (
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
)

// PrismUnionFilletQuarterArc confirms that a rectangular corner rewrite's
// connector is the exact inward quadrant with representable centre and feet.
func PrismUnionFilletQuarterArc(profile momentinput.Profile, corner sectionrecord.Point2, radius float64, blend *Blend) bool {
	arc, ok := blend.Connector.(sectionrecord.ArcSeg)
	if !ok || arc.TStart != 0 || arc.TEnd != 1 || arc.Start != blend.FA || arc.End != blend.FB ||
		blend.CutbackA != radius || blend.CutbackB != radius {
		return false
	}
	rect, ok := momentinput.AxisRectangle(profile)
	if !ok {
		return false
	}
	xDir, yDir := 1, 1
	if corner.U == rect.U1 {
		xDir = -1
	}
	if corner.V == rect.V1 {
		yDir = -1
	}
	xInner, xOK := momentinput.PrismUnionFilletExactShift(corner.U, radius, xDir)
	yInner, yOK := momentinput.PrismUnionFilletExactShift(corner.V, radius, yDir)
	if !xOK || !yOK || arc.Center != (sectionrecord.Point2{U: xInner, V: yInner}) {
		return false
	}
	xFoot := sectionrecord.Point2{U: xInner, V: corner.V}
	yFoot := sectionrecord.Point2{U: corner.U, V: yInner}
	if xDir == yDir {
		return blend.FA == yFoot && blend.FB == xFoot
	}
	return blend.FA == xFoot && blend.FB == yFoot
}

// PrismUnionChamferChord confirms that the exact directed feet and whole
// line connector remove the right triangle at a rectangular source corner.
func PrismUnionChamferChord(profile momentinput.Profile, corner sectionrecord.Point2,
	setback float64, blend *Blend) bool {
	line, ok := blend.Connector.(sectionrecord.LineSeg)
	if !ok || line.TStart != 0 || line.TEnd != 1 || line.Start != blend.FA || line.End != blend.FB ||
		blend.CutbackA != setback || blend.CutbackB != setback {
		return false
	}
	rect, ok := momentinput.AxisRectangle(profile)
	if !ok {
		return false
	}
	xDir, yDir := 1, 1
	if corner.U == rect.U1 {
		xDir = -1
	}
	if corner.V == rect.V1 {
		yDir = -1
	}
	xInner, xOK := momentinput.PrismUnionFilletExactShift(corner.U, setback, xDir)
	yInner, yOK := momentinput.PrismUnionFilletExactShift(corner.V, setback, yDir)
	if !xOK || !yOK {
		return false
	}
	xFoot := sectionrecord.Point2{U: xInner, V: corner.V}
	yFoot := sectionrecord.Point2{U: corner.U, V: yInner}
	if xDir == yDir {
		return blend.FA == yFoot && blend.FB == xFoot
	}
	return blend.FA == xFoot && blend.FB == yFoot
}
