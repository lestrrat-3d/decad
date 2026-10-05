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

// supportsAnalyticSTEP selects the complete face set before writing any faces.
// An unsupported edge or partial cylinder sends the entire body through the
// faceted writer, so one file never mixes two unrelated boundary constructions.
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
	if len(coedges) < 3 {
		return false
	}
	for _, ce := range coedges {
		if _, ok := ce.Edge().Curve().(decad.Line3); !ok {
			return false
		}
	}
	return true
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
	default:
		return 0, fmt.Errorf("%w: unsupported analytic edge %T", decad.ErrUnsupported, geometry)
	}
	ref := b.add(ap214.EdgeCurve(0, "", start, end, curve, true))
	b.edges[edge] = ref
	return ref, nil
}

func planarSTEPPlacement(loop *decad.Loop) (r3.Vec, r3.Vec, r3.Vec, error) {
	coedges := loop.CoEdges()
	if supportsAnalyticCircleLoop(loop) {
		circle := coedges[0].Edge().Curve().(decad.Circle3)
		reference, ok := coedges[0].Start().Position().Value.Sub(circle.Center).Normalize()
		if !ok {
			return r3.Vec{}, r3.Vec{}, r3.Vec{}, fmt.Errorf("%w: circle has no radial direction", decad.ErrDegenerate)
		}
		return circle.Center, circle.Axis, reference, nil
	}
	origin := coedges[0].Start().Position().Value
	next := coedges[0].End().Position().Value
	reference, ok := next.Sub(origin).Normalize()
	if !ok {
		return r3.Vec{}, r3.Vec{}, r3.Vec{}, fmt.Errorf("%w: plane loop has a zero first edge", decad.ErrDegenerate)
	}
	for _, ce := range coedges[1:] {
		axis, ok := reference.Cross(ce.End().Position().Value.Sub(next)).Normalize()
		if ok {
			return origin, axis, reference, nil
		}
	}
	return r3.Vec{}, r3.Vec{}, r3.Vec{}, fmt.Errorf("%w: plane loop has no normal", decad.ErrDegenerate)
}

func (b *analyticSTEPBuilder) addPlanarFace(ctx context.Context, face *decad.Face) (step.Reference, error) {
	loops := face.Loops()
	origin, axis, reference, err := planarSTEPPlacement(loops[0])
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
