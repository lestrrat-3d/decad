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

// supportsAnalyticCone reports whether a cone face is a full wall
// (supportsAnalyticConeWall) or a patch (supportsAnalyticConePatch). A cone
// of one circle loop, which reaches its apex without a vertex there, is
// written faceted.
func supportsAnalyticCone(loops []*decad.Loop, cone decad.Cone) bool {
	if len(loops) == 1 {
		return supportsAnalyticConePatch(loops[0], cone)
	}
	return supportsAnalyticConeWall(loops, cone)
}

// supportsAnalyticConePatch admits a cone face of one loop of at least three
// edges on a cone whose Radius is exactly zero, so its Origin is the apex:
// each edge an Arc3 about the cone's axis (its Axis the cone's or its
// negation and its Center on the axis, exactly, with both vertices in its
// plane, arcInItsPlane) or a Line3 along a generator (an exactly zero cross
// product of its two ends taken from the apex), with both kinds present and
// every vertex Exact (loopVerticesExact). A
// chamfer's corner patch is such a loop: two arcs and two generators at a
// convex corner, or one arc and two generators meeting at the apex vertex at
// a reflex one.
func supportsAnalyticConePatch(loop *decad.Loop, cone decad.Cone) bool {
	coedges := loop.CoEdges()
	if len(coedges) < 3 || cone.Radius.Mag() != 0 || !loopVerticesExact(loop) {
		return false
	}
	arcs, lines := 0, 0
	for _, ce := range coedges {
		edge := ce.Edge()
		switch c := edge.Curve().(type) {
		case decad.Arc3:
			if !axesAlign(c.Axis, cone.Axis) || c.Center.Sub(cone.Origin).Cross(cone.Axis) != (r3.Vec{}) ||
				!arcInItsPlane(edge, c) {
				return false
			}
			arcs++
		case decad.Line3:
			a := edge.Start().Position().Value.Sub(cone.Origin)
			b := edge.End().Position().Value.Sub(cone.Origin)
			if a == b || a.Cross(b) != (r3.Vec{}) {
				return false
			}
			lines++
		default:
			return false
		}
	}
	return arcs > 0 && lines > 0
}

// supportsAnalyticConeWall reports whether a cone face is a full wall: two
// one-circle loops, neither at the apex, whose start vertices lie on one meridian
// of the cone, so a straight seam between them runs along a generator.
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

// conicalSurface writes the cone as a CONICAL_SURFACE placed at its Origin
// about its Axis, with reference the radial direction there, its Radius at
// Origin and its HalfAngle. The STEP surface's normal has a positive
// component along the radial direction at every point off the axis.
func (b *analyticSTEPBuilder) conicalSurface(cone decad.Cone, reference r3.Vec) (step.Reference, error) {
	radius, err := cone.Radius.In(units.Millimeter)
	if err != nil {
		return 0, err
	}
	halfAngle, err := cone.HalfAngle.In(units.Radian)
	if err != nil {
		return 0, err
	}
	return b.add(step.Entity{Name: "CONICAL_SURFACE", Parameters: []step.Value{
		step.String(""), b.addPlacement(cone.Origin, cone.Axis, reference), step.Real(radius), step.Real(halfAngle),
	}}), nil
}

// addConePatchFace writes a cone patch (supportsAnalyticConePatch). Its sense
// is its outward normal against the radial direction at its first vertex off
// the axis, the apex being the only vertex on it, and its loop is walked as
// patchWalks decided.
func (b *analyticSTEPBuilder) addConePatchFace(ctx context.Context, face *decad.Face, cone decad.Cone) (step.Reference, error) {
	for _, ce := range face.Loops()[0].CoEdges() {
		p := ce.Start().Position().Value
		reference, ok := radialAbout(cone.Origin, cone.Axis, p)
		if !ok {
			continue
		}
		surface, err := b.conicalSurface(cone, reference)
		if err != nil {
			return 0, err
		}
		normal, err := face.NormalAt(p)
		if err != nil {
			return 0, err
		}
		return b.addWalkedLoopFace(ctx, face, surface, normal.Value.Dot(reference) > 0)
	}
	return 0, fmt.Errorf("%w: cone patch has no vertex off its axis", decad.ErrDegenerate)
}

// addConeFace writes a cone patch (addConePatchFace), or a full conical wall as
// a CONICAL_SURFACE bounded by its two circles and a seam along the generator
// between their start vertices, the way addCylinderFace writes a cylinder.
func (b *analyticSTEPBuilder) addConeFace(ctx context.Context, face *decad.Face, cone decad.Cone) (step.Reference, error) {
	loops := face.Loops()
	if len(loops) == 1 {
		return b.addConePatchFace(ctx, face, cone)
	}
	first := loops[0].CoEdges()[0]
	second := loops[1].CoEdges()[0]
	start := first.Start().Position().Value
	end := second.Start().Position().Value
	reference, ok := radialAbout(cone.Origin, cone.Axis, start)
	if !ok {
		return 0, fmt.Errorf("%w: cone has no radial reference", decad.ErrDegenerate)
	}
	surface, err := b.conicalSurface(cone, reference)
	if err != nil {
		return 0, err
	}
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
