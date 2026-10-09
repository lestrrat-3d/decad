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

// supportsAnalyticSTEP selects the complete face set before writing any faces.
// A plane's loops are a single full circle or a chain of lines and arcs; a
// cylinder is a full wall (two one-circle loops whose starts align along its
// axis) or a partial wall (one loop of lines along its axis closed by arcs
// about it or by ellipses); a torus is a whole turn (two one-circle loops
// whose starts share an azimuth) or a patch of parallel and meridian arcs
// (supportsAnalyticTorus). An unsupported edge or wall sends the entire body
// through the faceted writer, so one file never mixes two unrelated boundary
// constructions.
func supportsAnalyticSTEP(ctx context.Context, body *decad.Body) (bool, error) {
	for _, face := range body.Faces() {
		if err := ctx.Err(); err != nil {
			return false, err
		}
		loops := face.Loops()
		switch surface := face.Surface().(type) {
		case decad.Plane:
			if len(loops) == 0 || !loops[0].IsOuter() {
				return false, nil
			}
			for i, loop := range loops {
				if loop.IsOuter() != (i == 0) || !supportsAnalyticPlanarLoop(loop) {
					return false, nil
				}
			}
		case decad.Cylinder:
			if len(loops) == 1 {
				if !supportsAnalyticPartialWall(loops[0], surface) {
					return false, nil
				}
				continue
			}
			if len(loops) != 2 {
				return false, nil
			}
			for _, loop := range loops {
				if !supportsAnalyticCircleLoop(loop) {
					return false, nil
				}
			}
			start := loops[0].CoEdges()[0].Start().Position().Value
			end := loops[1].CoEdges()[0].Start().Position().Value
			seam := end.Sub(start)
			if seam == (r3.Vec{}) || seam.Cross(surface.Axis) != (r3.Vec{}) {
				return false, nil
			}
		case decad.Torus:
			if !supportsAnalyticTorus(loops, surface) {
				return false, nil
			}
		default:
			return false, nil
		}
	}
	return true, nil
}

func supportsAnalyticPlanarLoop(loop *decad.Loop) bool {
	if supportsAnalyticCircleLoop(loop) {
		return true
	}
	coedges := loop.CoEdges()
	arcs := 0
	for _, ce := range coedges {
		switch ce.Edge().Curve().(type) {
		case decad.Line3:
		case decad.Arc3:
			arcs++
		default:
			return false
		}
	}
	// Two lines cannot close a region; a line and an arc (a D) can.
	return len(coedges) >= 3 || (len(coedges) == 2 && arcs > 0)
}

// axesAlign reports whether two unit axes are equal or exact negations.
func axesAlign(a, b r3.Vec) bool {
	return a == b || a == b.Scale(-1)
}

// supportsAnalyticPartialWall admits a cylinder face bounded by one loop of
// at least four edges, each an Arc3 about the cylinder's own axis (its Axis
// equal to the cylinder's or its negation, exactly), an Ellipse3 or a Line3
// along the axis (an exactly zero cross product with it), with lines and at
// least one arc or ellipse present: the wall a prism sweeps from an arc,
// whose side lines may be split into several edges where neighbouring faces
// put vertices on them, or a loop fillet's straight-walk patch, whose two
// ends are the meridian arcs or mitre ellipses it shares with its neighbours.
func supportsAnalyticPartialWall(loop *decad.Loop, cylinder decad.Cylinder) bool {
	coedges := loop.CoEdges()
	if len(coedges) < 4 {
		return false
	}
	curved, lines := 0, 0
	for _, ce := range coedges {
		edge := ce.Edge()
		switch c := edge.Curve().(type) {
		case decad.Ellipse3:
			curved++
		case decad.Arc3:
			if !axesAlign(c.Axis, cylinder.Axis) {
				return false
			}
			curved++
		case decad.Line3:
			run := edge.End().Position().Value.Sub(edge.Start().Position().Value)
			if run == (r3.Vec{}) || run.Cross(cylinder.Axis) != (r3.Vec{}) {
				return false
			}
			lines++
		default:
			return false
		}
	}
	return curved > 0 && lines > 0
}

func supportsAnalyticCircleLoop(loop *decad.Loop) bool {
	coedges := loop.CoEdges()
	if len(coedges) != 1 {
		return false
	}
	edge := coedges[0].Edge()
	if edge.Start() != edge.End() {
		return false
	}
	_, ok := edge.Curve().(decad.Circle3)
	return ok
}

type analyticSTEPBuilder struct {
	*fileBuilder
	points   map[*decad.Vertex]step.Reference
	vertices map[*decad.Vertex]step.Reference
	edges    map[*decad.Edge]step.Reference
}

func (b *fileBuilder) addAnalyticFaces(ctx context.Context, body *decad.Body) ([]step.Reference, error) {
	a := &analyticSTEPBuilder{
		fileBuilder: b,
		points:      make(map[*decad.Vertex]step.Reference),
		vertices:    make(map[*decad.Vertex]step.Reference),
		edges:       make(map[*decad.Edge]step.Reference),
	}
	faces := make([]step.Reference, 0, len(body.Faces()))
	for i, face := range body.Faces() {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		var ref step.Reference
		var err error
		switch surface := face.Surface().(type) {
		case decad.Plane:
			ref, err = a.addPlanarFace(ctx, face)
		case decad.Cylinder:
			ref, err = a.addCylinderFace(ctx, face, surface)
		case decad.Torus:
			ref, err = a.addTorusFace(ctx, face, surface)
		}
		if err != nil {
			return nil, fmt.Errorf("export: STEP analytic face %d: %w", i, err)
		}
		faces = append(faces, ref)
	}
	return faces, nil
}

func (b *analyticSTEPBuilder) addPoint(p r3.Vec) step.Reference {
	return b.add(ap214.CartesianPoint(0, "", real3(p)))
}

func (b *analyticSTEPBuilder) addDirection(v r3.Vec) step.Reference {
	return b.add(ap214.Direction(0, "", real3(v)))
}

func (b *analyticSTEPBuilder) addPlacement(origin, axis, reference r3.Vec) step.Reference {
	p := b.addPoint(origin)
	z := b.addDirection(axis)
	x := b.addDirection(reference)
	return b.add(ap214.Axis2Placement3D(0, "", p, z, x))
}

func (b *analyticSTEPBuilder) addVertex(v *decad.Vertex) step.Reference {
	if ref, ok := b.vertices[v]; ok {
		return ref
	}
	point := b.addPoint(v.Position().Value)
	ref := b.add(ap214.VertexPoint(0, "", point))
	b.points[v] = point
	b.vertices[v] = ref
	return ref
}

func (b *analyticSTEPBuilder) addEdge(edge *decad.Edge) (step.Reference, error) {
	if ref, ok := b.edges[edge]; ok {
		return ref, nil
	}
	start := b.addVertex(edge.Start())
	end := b.addVertex(edge.End())
	var curve step.Reference
	switch geometry := edge.Curve().(type) {
	case decad.Line3:
		origin := edge.Start().Position().Value
		tangent, ok := edge.End().Position().Value.Sub(origin).Normalize()
		if !ok {
			return 0, fmt.Errorf("%w: line edge has no direction", decad.ErrDegenerate)
		}
		vector := b.add(ap214.Vector(0, "", b.addDirection(tangent), 1))
		curve = b.add(ap214.Line(0, "", b.points[edge.Start()], vector))
	case decad.Circle3:
		radial, ok := edge.Start().Position().Value.Sub(geometry.Center).Normalize()
		if !ok {
			return 0, fmt.Errorf("%w: circle edge has no reference direction", decad.ErrDegenerate)
		}
		radius, err := geometry.Radius.In(units.Millimeter)
		if err != nil {
			return 0, err
		}
		placement := b.addPlacement(geometry.Center, geometry.Axis, radial)
		curve = b.add(ap214.Circle(0, "", placement, step.Real(radius)))
	case decad.Arc3:
		// An arc is its circle trimmed by the edge's own two vertices: the
		// circle's placement turns about the arc's Axis, which the arc sweeps
		// counter-clockwise from Start to End, so the edge runs with the
		// circle's own sense.
		radial, ok := edge.Start().Position().Value.Sub(geometry.Center).Normalize()
		if !ok {
			return 0, fmt.Errorf("%w: arc edge has no reference direction", decad.ErrDegenerate)
		}
		radius, err := geometry.Radius.In(units.Millimeter)
		if err != nil {
			return 0, err
		}
		placement := b.addPlacement(geometry.Center, geometry.Axis, radial)
		curve = b.add(ap214.Circle(0, "", placement, step.Real(radius)))
	case decad.Ellipse3:
		// An ellipse arc is its ellipse trimmed by the edge's own two
		// vertices. Ellipse3 is swept counter-clockwise about its Axis from
		// Major, as STEP's ELLIPSE is about its placement axis from its
		// reference direction, so the edge keeps the curve's sense.
		semiMajor, err := geometry.SemiMajor.In(units.Millimeter)
		if err != nil {
			return 0, err
		}
		semiMinor, err := geometry.SemiMinor.In(units.Millimeter)
		if err != nil {
			return 0, err
		}
		placement := b.addPlacement(geometry.Center, geometry.Axis, geometry.Major)
		curve = b.add(ap214.Ellipse(0, "", placement, step.Real(semiMajor), step.Real(semiMinor)))
	default:
		return 0, fmt.Errorf("%w: unsupported analytic edge %T", decad.ErrUnsupported, geometry)
	}
	ref := b.add(ap214.EdgeCurve(0, "", start, end, curve, true))
	b.edges[edge] = ref
	return ref, nil
}

// planarSTEPPlacement is a planar face's STEP placement from its outer loop:
// a full circle's own centre, axis and seam direction, or, for a chain of
// lines and arcs, areaLoopPlacement's plane normal turned to the loop's
// sense.
func planarSTEPPlacement(face *decad.Face, loop *decad.Loop) (r3.Vec, r3.Vec, r3.Vec, error) {
	coedges := loop.CoEdges()
	if supportsAnalyticCircleLoop(loop) {
		circle, ok := coedges[0].Edge().Curve().(decad.Circle3)
		if !ok {
			return r3.Vec{}, r3.Vec{}, r3.Vec{}, fmt.Errorf("%w: planar circle has no curve", decad.ErrDegenerate)
		}
		reference, ok := coedges[0].Start().Position().Value.Sub(circle.Center).Normalize()
		if !ok {
			return r3.Vec{}, r3.Vec{}, r3.Vec{}, fmt.Errorf("%w: circle has no radial direction", decad.ErrDegenerate)
		}
		return circle.Center, circle.Axis, reference, nil
	}
	return areaLoopPlacement(face, loop)
}

func (b *analyticSTEPBuilder) addPlanarFace(ctx context.Context, face *decad.Face) (step.Reference, error) {
	loops := face.Loops()
	origin, axis, reference, err := planarSTEPPlacement(face, loops[0])
	if err != nil {
		return 0, err
	}
	normal, err := face.NormalAt(loops[0].CoEdges()[0].Start().Position().Value)
	if err != nil {
		return 0, err
	}
	sameSense := normal.Value.Dot(axis) > 0
	surface := b.add(ap214.Plane(0, "", b.addPlacement(origin, axis, reference)))
	bounds := make([]step.Reference, 0, len(loops))
	for _, loop := range loops {
		if err := ctx.Err(); err != nil {
			return 0, err
		}
		coedges := loop.CoEdges()
		oriented := make([]step.Reference, 0, len(coedges))
		for i := range coedges {
			if err := ctx.Err(); err != nil {
				return 0, err
			}
			ce := coedges[i]
			if !sameSense {
				ce = coedges[len(coedges)-1-i]
			}
			edge, err := b.addEdge(ce.Edge())
			if err != nil {
				return 0, err
			}
			forward := ce.IsForward()
			if !sameSense {
				forward = !forward
			}
			oriented = append(oriented, b.add(ap214.OrientedEdge(0, "", edge, forward)))
		}
		edgeLoop := b.add(ap214.EdgeLoop(0, "", oriented...))
		if loop.IsOuter() {
			bounds = append(bounds, b.add(ap214.FaceOuterBound(0, "", edgeLoop, true)))
		} else {
			bounds = append(bounds, b.add(ap214.FaceBound(0, "", edgeLoop, true)))
		}
	}
	return b.add(ap214.AdvancedFace(0, "", surface, sameSense, bounds...)), nil
}

func (b *analyticSTEPBuilder) addCylinderFace(ctx context.Context, face *decad.Face, cylinder decad.Cylinder) (step.Reference, error) {
	loops := face.Loops()
	if len(loops) == 1 {
		return b.addPartialCylinderFace(ctx, face, cylinder, loops[0])
	}
	first := loops[0].CoEdges()[0]
	second := loops[1].CoEdges()[0]
	start := first.Start().Position().Value
	end := second.Start().Position().Value
	radial := start.Sub(cylinder.Origin)
	radial = radial.Sub(cylinder.Axis.Scale(radial.Dot(cylinder.Axis)))
	reference, ok := radial.Normalize()
	if !ok {
		return 0, fmt.Errorf("%w: cylinder has no radial reference", decad.ErrDegenerate)
	}
	radius, err := cylinder.Radius.In(units.Millimeter)
	if err != nil {
		return 0, err
	}
	surface := b.add(ap214.CylindricalSurface(0, "", b.addPlacement(cylinder.Origin, cylinder.Axis, reference),
		step.Real(radius)))
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
		return 0, fmt.Errorf("%w: cylinder seam has no direction", decad.ErrDegenerate)
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

// arcSweep is an arc edge's counter-clockwise sweep about its own Axis from
// its start vertex to its end vertex, in (0, 2π].
func arcSweep(edge *decad.Edge, arc decad.Arc3) (float64, bool) {
	u, ok := edge.Start().Position().Value.Sub(arc.Center).Normalize()
	if !ok {
		return 0, false
	}
	v := arc.Axis.Cross(u)
	rel := edge.End().Position().Value.Sub(arc.Center)
	sweep := math.Atan2(rel.Dot(v), rel.Dot(u))
	if sweep <= 0 {
		sweep += 2 * math.Pi
	}
	return sweep, true
}

// loopAreaAbout is a planar loop's signed area about the unit normal n: the
// shoelace sum over each coedge's chord in the plane's (u, v), plus, for an
// arc, the circular segment between its chord and itself, ½r²(φ − sin φ),
// counted positive when the coedge walks the arc counter-clockwise about n.
// Positive means the loop runs counter-clockwise about n. Only its sign is
// read, to orient the loop's STEP placement; the area of a face that closes
// a region is never near zero.
func loopAreaAbout(loop *decad.Loop, u, v, n r3.Vec) (float64, bool) {
	area := 0.0
	for _, ce := range loop.CoEdges() {
		a, b := ce.Start().Position().Value, ce.End().Position().Value
		area += (a.Dot(u)*b.Dot(v) - b.Dot(u)*a.Dot(v)) / 2
		arc, ok := ce.Edge().Curve().(decad.Arc3)
		if !ok {
			continue
		}
		sweep, ok := arcSweep(ce.Edge(), arc)
		if !ok {
			return 0, false
		}
		r, err := arc.Radius.In(units.Millimeter)
		if err != nil {
			return 0, false
		}
		segment := r * r * (sweep - math.Sin(sweep)) / 2
		if ce.IsForward() != (arc.Axis.Dot(n) > 0) {
			segment = -segment
		}
		area += segment
	}
	return area, true
}

// areaLoopPlacement is the plane placement of a face whose outer loop is a
// chain of lines and arcs: the face's own plane normal, turned so the loop
// runs counter-clockwise about it (loopAreaAbout), with the loop's first
// vertex as origin and the plane's own U as the reference direction. The
// sense is read off the whole loop, never off the turn at its first vertex,
// which a reflex corner reverses.
func areaLoopPlacement(face *decad.Face, loop *decad.Loop) (r3.Vec, r3.Vec, r3.Vec, error) {
	plane, ok := face.Surface().(decad.Plane)
	if !ok {
		return r3.Vec{}, r3.Vec{}, r3.Vec{}, fmt.Errorf("%w: a plane loop's face is not a plane", decad.ErrUnsupported)
	}
	u, v, n := plane.Frame.U(), plane.Frame.V(), plane.Frame.N()
	area, ok := loopAreaAbout(loop, u, v, n)
	if !ok || area == 0 {
		return r3.Vec{}, r3.Vec{}, r3.Vec{}, fmt.Errorf("%w: a plane loop has no orientation", decad.ErrDegenerate)
	}
	if area < 0 {
		n = n.Scale(-1)
	}
	return loop.CoEdges()[0].Start().Position().Value, n, u, nil
}

// partialWallArea is a partial cylinder wall loop's signed area in the
// cylinder's (θ, z) parameter plane, θ counter-clockwise about the axis from
// reference and z along it. A loop counter-clockwise in (θ, z) runs
// counter-clockwise about the outward radial direction, since ∂θ × ∂z is the
// radial direction. An arc advances θ by its sweep, signed by whether the
// coedge walks it counter-clockwise about the cylinder's axis; a line along
// the axis keeps θ and moves z. An ellipse advances θ by the signed angle
// between its end vertices' radial directions and is taken as the chord
// between them in the (θ, z) plane: only the sign of the area is read, and a
// patch's chord polygon is simple, so it has the sign of the curved loop.
func partialWallArea(loop *decad.Loop, cylinder decad.Cylinder) (float64, bool) {
	theta, area := 0.0, 0.0
	for _, ce := range loop.CoEdges() {
		z0 := ce.Start().Position().Value.Sub(cylinder.Origin).Dot(cylinder.Axis)
		z1 := ce.End().Position().Value.Sub(cylinder.Origin).Dot(cylinder.Axis)
		next := theta
		switch curve := ce.Edge().Curve().(type) {
		case decad.Arc3:
			sweep, ok := arcSweep(ce.Edge(), curve)
			if !ok {
				return 0, false
			}
			if ce.IsForward() == (curve.Axis.Dot(cylinder.Axis) > 0) {
				next += sweep
			} else {
				next -= sweep
			}
		case decad.Ellipse3:
			advance, ok := radialAdvance(cylinder, ce.Start().Position().Value, ce.End().Position().Value)
			if !ok {
				return 0, false
			}
			next += advance
		}
		area += (theta*z1 - next*z0) / 2
		theta = next
	}
	return area, true
}

// radialAdvance is the signed angle about the cylinder's axis from the radial
// direction of a to that of b, in (−π, π].
func radialAdvance(cylinder decad.Cylinder, a, b r3.Vec) (float64, bool) {
	ra, ok := radialAbout(cylinder.Origin, cylinder.Axis, a)
	if !ok {
		return 0, false
	}
	rb, ok := radialAbout(cylinder.Origin, cylinder.Axis, b)
	if !ok {
		return 0, false
	}
	return math.Atan2(cylinder.Axis.Dot(ra.Cross(rb)), ra.Dot(rb)), true
}

// radialAbout is the unit direction from the line (origin, axis) to p.
func radialAbout(origin, axis, p r3.Vec) (r3.Vec, bool) {
	v := p.Sub(origin)
	return v.Sub(axis.Scale(v.Dot(axis))).Normalize()
}

// partialWallSense is a partial cylinder face's STEP face sense (its outward
// normal against the radial reference at the loop's first vertex) and
// whether the loop must be walked in reverse to run counter-clockwise about
// the STEP face normal.
func partialWallSense(face *decad.Face, cylinder decad.Cylinder, loop *decad.Loop, reference r3.Vec) (bool, bool, error) {
	normal, err := face.NormalAt(loop.CoEdges()[0].Start().Position().Value)
	if err != nil {
		return false, false, err
	}
	sameSense := normal.Value.Dot(reference) > 0
	area, ok := partialWallArea(loop, cylinder)
	if !ok || area == 0 {
		return false, false, fmt.Errorf("%w: a partial cylinder loop has no orientation", decad.ErrDegenerate)
	}
	return sameSense, (area > 0) != sameSense, nil
}

// addPartialCylinderFace writes a cylinder face bounded by one loop
// (supportsAnalyticPartialWall). The face sense follows its outward normal
// against the radial direction, and the loop is walked counter-clockwise
// about the STEP face normal: reversed when its (θ, z) orientation
// (partialWallArea) disagrees with that sense.
func (b *analyticSTEPBuilder) addPartialCylinderFace(ctx context.Context, face *decad.Face, cylinder decad.Cylinder, loop *decad.Loop) (step.Reference, error) {
	coedges := loop.CoEdges()
	start := coedges[0].Start().Position().Value
	radial := start.Sub(cylinder.Origin)
	radial = radial.Sub(cylinder.Axis.Scale(radial.Dot(cylinder.Axis)))
	reference, ok := radial.Normalize()
	if !ok {
		return 0, fmt.Errorf("%w: cylinder has no radial reference", decad.ErrDegenerate)
	}
	radius, err := cylinder.Radius.In(units.Millimeter)
	if err != nil {
		return 0, err
	}
	surface := b.add(ap214.CylindricalSurface(0, "", b.addPlacement(cylinder.Origin, cylinder.Axis, reference),
		step.Real(radius)))
	sameSense, reverse, err := partialWallSense(face, cylinder, loop, reference)
	if err != nil {
		return 0, err
	}
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
