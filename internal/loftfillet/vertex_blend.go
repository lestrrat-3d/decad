package loftfillet

import (
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/offset2d"
	"github.com/lestrrat-3d/decad/internal/sectionaudit"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/survey2d"
)

// VertexCorner is one corner where a fillet consumes a line that runs from a
// circle fragment to a fit spline. CircleArrives is true for the section order
// circle, line, fit spline, and false for fit spline, line, circle. Seed is
// offset2d.Fillet's blend at the circle/line corner, tangent to the circle and
// to the line's extension past its far end.
type VertexCorner struct {
	Circle        sectionrecord.CircleSeg
	CircleWalk    survey2d.SegmentWalk
	Line          survey2d.SegmentWalk
	Fit           survey2d.SegmentWalk
	Seed          *offset2d.Blend
	CircleArrives bool
}

// VertexBlend is the fillet at a VertexCorner whose rolling ball rests on the
// vertex V where the consumed line meets the fit spline
// (agent-docs/loft-design.md §17). The ball is tangent to the root circle and
// passes through V; its connector runs from the circle foot to V, and the fit
// spline stays whole.
//
// It reports false when the fit spline leaves V into that ball: the ball then
// meets the fit spline beyond V, and the fit-circle route applies. It refuses
// when the fillet's kink at V, the angle between the connector's and the fit
// spline's travel tangents, is not smaller than the turn the line made into the
// fit spline at V. The connector is a float proposal: the caller trims the
// circle by its foot and audits the rewritten section.
func VertexBlend(c VertexCorner, radius float64) (*offset2d.Blend, bool, error) {
	refuse := func(reason string) (*offset2d.Blend, bool, error) {
		return nil, false, fmt.Errorf(`%w: %s`, decaderr.ErrUnsupported, reason)
	}
	if c.Seed == nil || !finiteValue(radius) || radius <= 0 || c.CircleWalk.Kind != survey2d.WalkCircular ||
		c.Line.Kind != survey2d.WalkLine || c.Fit.Kind != survey2d.WalkFreeform {
		return refuse("the loft vertex fillet has invalid corner inputs")
	}
	seedArc, ok := c.Seed.Connector.(sectionrecord.ArcSeg)
	if !ok {
		return refuse("the loft vertex fillet has no seed arc")
	}
	ou, ov, rootR := c.CircleWalk.CU, c.CircleWalk.CV, c.CircleWalk.Radius
	// The vertex, the fit spline's travel tangent there, and the line's travel
	// tangent there, all in section order.
	vu, vv := c.Fit.StartU, c.Fit.StartV
	fitU, fitV := c.Fit.TanInU, c.Fit.TanInV
	lineU, lineV := c.Line.TanOutU, c.Line.TanOutV
	if !c.CircleArrives {
		vu, vv = c.Fit.EndU, c.Fit.EndV
		fitU, fitV = c.Fit.TanOutU, c.Fit.TanOutV
		lineU, lineV = c.Line.TanInU, c.Line.TanInV
	}
	if !finiteValue(ou, ov, rootR, vu, vv, fitU, fitV, lineU, lineV, seedArc.Center.U, seedArc.Center.V) || rootR <= 0 {
		return refuse("the loft vertex fillet has a non-finite corner")
	}
	// The ball's centre lies on the root circle's offset by the fillet radius,
	// on the side the seed ball takes, and at the fillet radius from V.
	rho := rootR + radius
	if math.Hypot(seedArc.Center.U-ou, seedArc.Center.V-ov) < rootR {
		rho = rootR - radius
	}
	du, dv := vu-ou, vv-ov
	d := math.Hypot(du, dv)
	if !finiteValue(rho, d) || rho <= sectionaudit.Tolerance || d <= math.Abs(rho-radius) || d >= rho+radius {
		return refuse("the loft vertex fillet ball cannot touch the root circle and the vertex")
	}
	along := (rho*rho - radius*radius + d*d) / (2 * d)
	height := math.Sqrt(rho*rho - along*along)
	baseU, baseV := ou+along*du/d, ov+along*dv/d
	centers := [2]sectionrecord.Point2{
		{U: baseU - height*dv/d, V: baseV + height*du/d},
		{U: baseU + height*dv/d, V: baseV - height*du/d},
	}
	center := centers[0]
	if math.Hypot(centers[1].U-seedArc.Center.U, centers[1].V-seedArc.Center.V) <
		math.Hypot(centers[0].U-seedArc.Center.U, centers[0].V-seedArc.Center.V) {
		center = centers[1]
	}
	// The fit spline must leave V away from the ball's centre. Otherwise it
	// enters the ball, which then rests on the fit spline rather than on V.
	ru, rv := vu-center.U, vv-center.V
	outward := ru*fitU + rv*fitV
	if !c.CircleArrives {
		outward = -outward
	}
	if !finiteValue(outward) || outward <= 0 {
		return nil, false, nil
	}
	cd := math.Hypot(center.U-ou, center.V-ov)
	foot := sectionrecord.Point2{U: ou + rootR*(center.U-ou)/cd, V: ov + rootR*(center.V-ov)/cd}
	// The connector continues the circle's travel at the foot.
	circleSign := math.Copysign(1, c.Circle.TEnd-c.Circle.TStart)
	circleTanU, circleTanV := -circleSign*(foot.V-ov), circleSign*(foot.U-ou)
	ccw := -(foot.V-center.V)*circleTanU+(foot.U-center.U)*circleTanV > 0
	sense := -1.0
	if ccw {
		sense = 1
	}
	arcU, arcV := -sense*rv/radius, sense*ru/radius
	kink := angleBetween(arcU, arcV, fitU, fitV)
	turn := angleBetween(lineU, lineV, fitU, fitV)
	if !finiteValue(kink, turn, foot.U, foot.V, center.U, center.V) || kink >= turn {
		return refuse(fmt.Sprintf("the loft vertex fillet leaves a %.3g degree kink at the line's far end, "+
			"not less than the line's own %.3g degree turn there", kink*180/math.Pi, turn*180/math.Pi))
	}
	vertex := sectionrecord.Point2{U: vu, V: vv}
	blend := &offset2d.Blend{FA: foot, FB: vertex}
	if !c.CircleArrives {
		blend.FA, blend.FB = vertex, foot
	}
	blend.Connector = offset2d.ArcSegment(center, blend.FA, blend.FB, ccw)
	return blend, true, nil
}

// angleBetween is the unsigned angle between two nonzero directions.
func angleBetween(au, av, bu, bv float64) float64 {
	return math.Atan2(math.Abs(au*bv-av*bu), au*bu+av*bv)
}
