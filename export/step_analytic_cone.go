package export

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/step"
	"github.com/lestrrat-3d/step/ap214"
	"github.com/lestrrat-3d/units"
)

// supportsAnalyticConeWall reports whether a cone face is a full wall: two
// one-circle loops, neither at the apex, whose start vertices lie on one meridian
// of the cone, so a straight seam between them runs along a generator. A cone
// that reaches its apex, or has a partial wall, is written faceted.
func supportsAnalyticConeWall(loops []*decad.Loop, cone decad.Cone) bool {
	if len(loops) != 2 {
		return false
	}
	for _, loop := range loops {
		if !supportsAnalyticCircleLoop(loop) {
			return false
		}
	}
	start := loops[0].CoEdges()[0].Start().Position().Value
	end := loops[1].CoEdges()[0].Start().Position().Value
	a, okA := radialAbout(cone.Origin, cone.Axis, start)
	b, okB := radialAbout(cone.Origin, cone.Axis, end)
	if !okA || !okB {
		return false
	}
	return a.Cross(b) == (r3.Vec{}) && a.Dot(b) > 0
}

// addConeFace writes a full conical wall as a CONICAL_SURFACE bounded by its two
// circles and a seam along the generator between their start vertices, the way
// addCylinderFace writes a cylinder.
func (b *analyticSTEPBuilder) addConeFace(ctx context.Context, face *decad.Face, cone decad.Cone) (step.Reference, error) {
	loops := face.Loops()
	first := loops[0].CoEdges()[0]
	second := loops[1].CoEdges()[0]
	start := first.Start().Position().Value
	end := second.Start().Position().Value
	reference, ok := radialAbout(cone.Origin, cone.Axis, start)
	if !ok {
		return 0, fmt.Errorf("%w: cone has no radial reference", decad.ErrDegenerate)
	}
	radius, err := cone.Radius.In(units.Millimeter)
	if err != nil {
		return 0, err
	}
	halfAngle, err := cone.HalfAngle.In(units.Radian)
	if err != nil {
		return 0, err
	}
	surface := b.add(step.Entity{Name: "CONICAL_SURFACE", Parameters: []step.Value{
		step.String(""), b.addPlacement(cone.Origin, cone.Axis, reference), step.Real(radius), step.Real(halfAngle),
	}})
	normal, err := face.NormalAt(start)
	if err != nil {
		return 0, err
	}
	sameSense := normal.Value.Dot(reference) > 0
	firstEdge, err := b.addEdge(first.Edge())
	if err != nil {
		return 0, err
	}
	secondEdge, err := b.addEdge(second.Edge())
	if err != nil {
		return 0, err
	}
	if err := ctx.Err(); err != nil {
		return 0, err
	}
	seamStart := b.addVertex(first.Start())
	seamEnd := b.addVertex(second.Start())
	seamDirection, ok := end.Sub(start).Normalize()
	if !ok {
		return 0, fmt.Errorf("%w: cone seam has no direction", decad.ErrDegenerate)
	}
	seamVector := b.add(ap214.Vector(0, "", b.addDirection(seamDirection), 1))
	seamLine := b.add(ap214.Line(0, "", b.points[first.Start()], seamVector))
	seam := b.add(ap214.EdgeCurve(0, "", seamStart, seamEnd, seamLine, true))
	oriented := []step.Reference{
		b.add(ap214.OrientedEdge(0, "", firstEdge, first.IsForward())),
		b.add(ap214.OrientedEdge(0, "", seam, true)),
		b.add(ap214.OrientedEdge(0, "", secondEdge, second.IsForward())),
		b.add(ap214.OrientedEdge(0, "", seam, false)),
	}
	edgeLoop := b.add(ap214.EdgeLoop(0, "", oriented...))
	bound := b.add(ap214.FaceOuterBound(0, "", edgeLoop, true))
	return b.add(ap214.AdvancedFace(0, "", surface, sameSense, bound)), nil
}
