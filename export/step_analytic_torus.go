package export

import (
	"context"
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/step"
	"github.com/lestrrat-3d/step/ap214"
	"github.com/lestrrat-3d/units"
)

// torusRadii is a torus's major and minor radius in millimetres; both must
// be positive. A minor radius above the major (a spindle torus) is admitted:
// the loop fillet of a round corner whose radius is under twice the fillet's
// lies on its outer quarter, where the tube never meets the axis.
func torusRadii(torus decad.Torus) (float64, float64, bool) {
	major, err := torus.Major.In(units.Millimeter)
	if err != nil || major <= 0 {
		return 0, 0, false
	}
	minor, err := torus.Minor.In(units.Millimeter)
	if err != nil || minor <= 0 {
		return 0, 0, false
	}
	return major, minor, true
}

// torusAzimuth is the unit direction from the torus axis to p.
func torusAzimuth(torus decad.Torus, p r3.Vec) (r3.Vec, bool) {
	return radialAbout(torus.Center, torus.Axis, p)
}

// supportsAnalyticTorus selects a torus face the writer states exactly: a
// whole turn (two one-circle loops, see supportsAnalyticWholeTorus), a patch
// of one loop of at least four Arc3 edges (supportsAnalyticTorusPatch), or a
// horn torus's patch of three, closing on the axis at a vertex
// (supportsAnalyticHornPatch).
func supportsAnalyticTorus(loops []*decad.Loop, torus decad.Torus) bool {
	if _, _, ok := torusRadii(torus); !ok {
		return false
	}
	switch len(loops) {
	case 1:
		if isHornPatch(loops[0]) {
			return supportsAnalyticHornPatch(loops[0], torus)
		}
		return supportsAnalyticTorusPatch(loops[0], torus)
	case 2:
		return supportsAnalyticWholeTorus(loops, torus)
	}
	return false
}

// supportsAnalyticWholeTorus admits two one-circle loops whose start
// vertices lie on one azimuth about the axis (an exactly zero cross product,
// the same side), so a synthetic seam, a meridian circle of the tube through
// both, joins them. That seam changes STEP topology, not the body's geometry.
func supportsAnalyticWholeTorus(loops []*decad.Loop, torus decad.Torus) bool {
	for _, loop := range loops {
		if !supportsAnalyticCircleLoop(loop) {
			return false
		}
	}
	a, ok := torusAzimuth(torus, loops[0].CoEdges()[0].Start().Position().Value)
	if !ok {
		return false
	}
	b, ok := torusAzimuth(torus, loops[1].CoEdges()[0].Start().Position().Value)
	if !ok || a.Cross(b) != (r3.Vec{}) || a.Dot(b) <= 0 {
		return false
	}
	_, _, ok = torusSeam(torus, loops)
	return ok
}

// torusSeam is a whole-turn torus's seam: the tube circle through both loops'
// start vertices, as its placement axis and unit reference direction. The
// reference points from the tube's centre to the first loop's start, and the
// axis turns it counter-clockwise onto the second's, the short way round.
func torusSeam(torus decad.Torus, loops []*decad.Loop) (axis, reference r3.Vec, ok bool) {
	major, _, ok := torusRadii(torus)
	if !ok {
		return r3.Vec{}, r3.Vec{}, false
	}
	p1 := loops[0].CoEdges()[0].Start().Position().Value
	p2 := loops[1].CoEdges()[0].Start().Position().Value
	azimuth, ok := torusAzimuth(torus, p1)
	if !ok {
		return r3.Vec{}, r3.Vec{}, false
	}
	tube := torus.Center.Add(azimuth.Scale(major))
	u, w := p1.Sub(tube), p2.Sub(tube)
	axis, ok = u.Cross(w).Normalize()
	if !ok {
		return r3.Vec{}, r3.Vec{}, false
	}
	reference, ok = u.Normalize()
	return axis, reference, ok
}

// torusArcIsParallel reports whether an arc runs about the torus's own axis
// (its Axis the torus's or its negation, exactly): a parallel of the torus.
func torusArcIsParallel(arc decad.Arc3, torus decad.Torus) bool {
	return axesAlign(arc.Axis, torus.Axis)
}

// torusArcIsMeridian reports whether an arc's plane contains the torus axis
// (its Axis exactly perpendicular to the torus's): a meridian of the tube.
func torusArcIsMeridian(arc decad.Arc3, torus decad.Torus) bool {
	return arc.Axis.Dot(torus.Axis) == 0
}

// torusPatchArc classifies one arc edge of a torus patch: whether it is a
// parallel (else a meridian), and whether the patch admits it at all. An arc
// that is neither, or whose vertices are not both in its own plane
// (arcInItsPlane), is refused: the file would state a CIRCLE the edge does
// not run on. A meridian whose Radius is not the torus's Minor, exactly, is
// refused too: the tag would not hold the tube the meridian runs round, and
// the file would state a torus through vertices off it.
func torusPatchArc(edge *decad.Edge, arc decad.Arc3, torus decad.Torus) (bool, bool) {
	if !arcInItsPlane(edge, arc) {
		return false, false
	}
	switch {
	case torusArcIsParallel(arc, torus):
		return true, true
	case torusArcIsMeridian(arc, torus):
		return false, meridianRadiusIsMinor(arc, torus)
	}
	return false, false
}

// meridianRadiusIsMinor reports whether a meridian arc's Radius equals the
// torus's Minor, both in millimetres, exactly. It only refuses.
func meridianRadiusIsMinor(arc decad.Arc3, torus decad.Torus) bool {
	r, err := arc.Radius.In(units.Millimeter)
	if err != nil {
		return false
	}
	minor, err := torus.Minor.In(units.Millimeter)
	return err == nil && r == minor
}

// supportsAnalyticTorusPatch admits one loop of at least four Arc3 edges,
// each a parallel of the torus or a meridian of its tube with both vertices
// in its own plane (torusPatchArc), with both kinds present: the pipe patch
// of a round corner, bounded by its top and side contours and the two
// meridians it shares with its neighbours.
func supportsAnalyticTorusPatch(loop *decad.Loop, torus decad.Torus) bool {
	coedges := loop.CoEdges()
	if len(coedges) < 4 {
		return false
	}
	parallels, meridians := 0, 0
	for _, ce := range coedges {
		arc, ok := ce.Edge().Curve().(decad.Arc3)
		if !ok {
			return false
		}
		parallel, ok := torusPatchArc(ce.Edge(), arc, torus)
		switch {
		case !ok:
			return false
		case parallel:
			parallels++
		default:
			meridians++
		}
	}
	return parallels > 0 && meridians > 0
}

// isHornPatch reports whether a torus face's one loop has three edges, the
// shape a horn torus's patch takes (supportsAnalyticHornPatch). Such a patch
// is walked by the edge-use rule (patchWalks), since the azimuth its (θ, φ)
// area reads has no value at its apex vertex on the axis.
func isHornPatch(loop *decad.Loop) bool {
	return len(loop.CoEdges()) == 3
}

// supportsAnalyticHornPatch admits a horn torus's patch: a torus whose Minor
// equals its Major, exactly, so its tube touches its axis at its Center, and
// one loop of three Arc3 edges, each holding both its vertices in its own
// plane (torusPatchArc), every vertex Exact (loopVerticesExact), exactly one
// vertex at the torus's Center (the apex), every edge at the apex a meridian
// of the tube, and exactly one parallel. A pocket mouth fillet's reflex corner
// (loop fillet LF6) is such a patch: its connector arc and the two meridians
// that run down from the arc's ends to the apex. Every leg only refuses.
func supportsAnalyticHornPatch(loop *decad.Loop, torus decad.Torus) bool {
	major, minor, ok := torusRadii(torus)
	coedges := loop.CoEdges()
	if !ok || major != minor || len(coedges) != 3 || !loopVerticesExact(loop) {
		return false
	}
	apexes, parallels := 0, 0
	for _, ce := range coedges {
		arc, ok := ce.Edge().Curve().(decad.Arc3)
		if !ok {
			return false
		}
		parallel, ok := torusPatchArc(ce.Edge(), arc, torus)
		if !ok {
			return false
		}
		if parallel {
			parallels++
		}
		start, end := ce.Start().Position().Value == torus.Center, ce.End().Position().Value == torus.Center
		if start {
			apexes++
		}
		if parallel && (start || end) {
			return false
		}
	}
	return apexes == 1 && parallels == 1
}

// torusOffAxisVertex is the first vertex a torus face's first loop walks that
// lies off the torus axis, and its azimuth. A horn patch's apex lies on the
// axis, where the azimuth has no value and the surface no normal.
func torusOffAxisVertex(torus decad.Torus, loop *decad.Loop) (r3.Vec, r3.Vec, bool) {
	for _, ce := range loop.CoEdges() {
		p := ce.Start().Position().Value
		if azimuth, ok := torusAzimuth(torus, p); ok {
			return p, azimuth, true
		}
	}
	return r3.Vec{}, r3.Vec{}, false
}

// torusTubeAngle is p's angle about the tube's centre circle, from the
// outward radial direction toward the torus axis, in (−π, π].
func torusTubeAngle(torus decad.Torus, major float64, p r3.Vec) float64 {
	v := p.Sub(torus.Center)
	h := v.Dot(torus.Axis)
	rho := v.Sub(torus.Axis.Scale(h)).Len()
	return math.Atan2(h, rho-major)
}

// wrapAngle folds an angle into (−π, π].
func wrapAngle(a float64) float64 {
	for a > math.Pi {
		a -= 2 * math.Pi
	}
	for a <= -math.Pi {
		a += 2 * math.Pi
	}
	return a
}

// torusNormalAt is the torus's own normal at p, pointing away from the
// tube's centre circle. The torus parameterised by (θ, φ), θ counter-clockwise
// about the axis and φ from the outward radial direction toward the axis, has
// ∂θ × ∂φ along it.
func torusNormalAt(torus decad.Torus, major float64, p r3.Vec) (r3.Vec, bool) {
	azimuth, ok := torusAzimuth(torus, p)
	if !ok {
		return r3.Vec{}, false
	}
	return p.Sub(torus.Center.Add(azimuth.Scale(major))).Normalize()
}

// torusPatchArea is a torus patch loop's signed area in the (θ, φ) parameter
// plane. A parallel advances θ by its sweep, signed by whether the coedge
// walks it counter-clockwise about the torus axis; a meridian holds θ and
// advances φ by the signed angle between its end vertices about the tube. A
// loop counter-clockwise in (θ, φ) runs counter-clockwise about the torus's
// own normal. Each coedge is taken as the straight step in the plane, exact
// for these edges since each holds one coordinate.
func torusPatchArea(loop *decad.Loop, torus decad.Torus, major float64) (float64, bool) {
	theta, phi, area := 0.0, 0.0, 0.0
	for _, ce := range loop.CoEdges() {
		arc, ok := ce.Edge().Curve().(decad.Arc3)
		if !ok {
			return 0, false
		}
		nextTheta, nextPhi := theta, phi
		if torusArcIsParallel(arc, torus) {
			sweep, ok := arcSweep(ce.Edge(), arc)
			if !ok {
				return 0, false
			}
			if ce.IsForward() == (arc.Axis.Dot(torus.Axis) > 0) {
				nextTheta += sweep
			} else {
				nextTheta -= sweep
			}
		} else {
			from := torusTubeAngle(torus, major, ce.Start().Position().Value)
			to := torusTubeAngle(torus, major, ce.End().Position().Value)
			nextPhi += wrapAngle(to - from)
		}
		area += (theta*nextPhi - nextTheta*phi) / 2
		theta, phi = nextTheta, nextPhi
	}
	return area, true
}

// torusFaceSense is a torus face's STEP face sense: whether its outward
// normal at p agrees with the torus's own.
func torusFaceSense(face *decad.Face, torus decad.Torus, major float64, p r3.Vec) (bool, error) {
	own, ok := torusNormalAt(torus, major, p)
	if !ok {
		return false, fmt.Errorf("%w: torus has no normal at a boundary vertex", decad.ErrDegenerate)
	}
	normal, err := face.NormalAt(p)
	if err != nil {
		return false, err
	}
	return normal.Value.Dot(own) > 0, nil
}

// addTorusFace writes a torus face: a whole turn with a synthetic seam, a
// patch walked by its (θ, φ) area, or a horn patch walked as patchWalks
// decided (supportsAnalyticTorus). The placement's reference and the face
// sense are read at the loop's first vertex off the axis
// (torusOffAxisVertex).
func (b *analyticSTEPBuilder) addTorusFace(ctx context.Context, face *decad.Face, torus decad.Torus) (step.Reference, error) {
	major, minor, ok := torusRadii(torus)
	if !ok {
		return 0, fmt.Errorf("%w: torus has no positive radii", decad.ErrDegenerate)
	}
	loops := face.Loops()
	start, reference, ok := torusOffAxisVertex(torus, loops[0])
	if !ok {
		return 0, fmt.Errorf("%w: torus has no azimuth reference", decad.ErrDegenerate)
	}
	surface := b.add(ap214.ToroidalSurface(0, "", b.addPlacement(torus.Center, torus.Axis, reference),
		step.Real(major), step.Real(minor)))
	sameSense, err := torusFaceSense(face, torus, major, start)
	if err != nil {
		return 0, err
	}
	if len(loops) == 1 && isHornPatch(loops[0]) {
		return b.addWalkedLoopFace(ctx, face, surface, sameSense)
	}
	if len(loops) == 1 {
		return b.addTorusPatchFace(ctx, torus, major, loops[0], surface, sameSense)
	}
	seamAxis, seamReference, ok := torusSeam(torus, loops)
	if !ok {
		return 0, fmt.Errorf("%w: torus has no seam", decad.ErrDegenerate)
	}
	first, second := loops[0].CoEdges()[0], loops[1].CoEdges()[0]
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
	tube := torus.Center.Add(reference.Scale(major))
	seamCircle := b.add(ap214.Circle(0, "", b.addPlacement(tube, seamAxis, seamReference), step.Real(minor)))
	seam := b.add(ap214.EdgeCurve(0, "", seamStart, seamEnd, seamCircle, true))
	oriented := []step.Reference{
		b.add(ap214.OrientedEdge(0, "", firstEdge, first.IsForward())),
		b.add(ap214.OrientedEdge(0, "", seam, true)),
		b.add(ap214.OrientedEdge(0, "", secondEdge, second.IsForward())),
		b.add(ap214.OrientedEdge(0, "", seam, false)),
	}
	bound := b.add(ap214.FaceOuterBound(0, "", b.add(ap214.EdgeLoop(0, "", oriented...)), true))
	return b.add(ap214.AdvancedFace(0, "", surface, sameSense, bound)), nil
}

// torusPatchReversed is whether a torus patch's loop is written against its
// source walk so that it runs counter-clockwise about the STEP face normal:
// when its (θ, φ) orientation (torusPatchArea) disagrees with the face sense.
func torusPatchReversed(torus decad.Torus, major float64, loop *decad.Loop, sameSense bool) (bool, error) {
	area, ok := torusPatchArea(loop, torus, major)
	if !ok || area == 0 {
		return false, fmt.Errorf("%w: a torus patch loop has no orientation", decad.ErrDegenerate)
	}
	return (area > 0) != sameSense, nil
}

// addTorusPatchFace writes a torus patch bounded by one loop
// (supportsAnalyticTorusPatch), walked as torusPatchReversed decides.
func (b *analyticSTEPBuilder) addTorusPatchFace(
	ctx context.Context, torus decad.Torus, major float64, loop *decad.Loop, surface step.Reference, sameSense bool,
) (step.Reference, error) {
	reverse, err := torusPatchReversed(torus, major, loop, sameSense)
	if err != nil {
		return 0, err
	}
	coedges := loop.CoEdges()
	oriented := make([]step.Reference, 0, len(coedges))
	for i := range coedges {
		if err := ctx.Err(); err != nil {
			return 0, err
		}
		ce := coedges[i]
		if reverse {
			ce = coedges[len(coedges)-1-i]
		}
		edge, err := b.addEdge(ce.Edge())
		if err != nil {
			return 0, err
		}
		oriented = append(oriented, b.add(ap214.OrientedEdge(0, "", edge, ce.IsForward() != reverse)))
	}
	bound := b.add(ap214.FaceOuterBound(0, "", b.add(ap214.EdgeLoop(0, "", oriented...)), true))
	return b.add(ap214.AdvancedFace(0, "", surface, sameSense, bound)), nil
}
