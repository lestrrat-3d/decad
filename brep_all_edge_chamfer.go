package decad

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/breprecord"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/r3"
)

// tryAllEdgeChamfer accepts only the complete straight-edge set of an exact
// axis-aligned box with one through-y circular bore. Other breps keep their
// existing route L result or refusal.
func tryAllEdgeChamfer(ctx context.Context, source *Body, bp brepPayload,
	edges []*Edge, distance, distanceDelta float64, asym *asymmetricChamfer) (*Body, bool, error) {
	if len(edges) != 12 || asym != nil || len(bp.Faces) != 7 || bp.Xform != r3.Identity() ||
		bp.SectionDelta() != 0 || bp.AxialDelta() != 0 || len(bp.LoopBands) != 0 ||
		bp.BossShell != nil || bp.PocketShell != nil {
		return nil, false, nil
	}
	if distanceDelta != 0 {
		return nil, true, fmt.Errorf(`%w: the all-edge chamfer needs an exact millimetre setback`, ErrUnsupported)
	}
	var p allEdgeChamferPayload
	p.Xform = r3.Identity()
	p.D = distance
	p.Box = [3][2]float64{{source.n.Bounds.Min.X, source.n.Bounds.Max.X},
		{source.n.Bounds.Min.Y, source.n.Bounds.Max.Y}, {source.n.Bounds.Min.Z, source.n.Bounds.Max.Z}}
	if source.n.Bounds.Bound.Base() != 0 || !p.AdmitBox() {
		return nil, false, nil
	}
	selected := map[*Edge]struct{}{}
	for _, e := range edges {
		if _, dup := selected[e]; dup || !p.OuterBoxEdge(edgeNode(e)) {
			return nil, false, nil
		}
		selected[e] = struct{}{}
	}
	planes := 0
	var boreFace *Face
	var boreSurface surfacegeom.Cylinder
	var seen [3][2]bool
	for _, f := range source.Faces() {
		switch surface := f.n.Surface.(type) {
		case surfacegeom.Plane:
			axis, side, ok := p.BoxPlane(faceNode(f), surface)
			if !ok || seen[axis][side] {
				return nil, false, nil
			}
			seen[axis][side] = true
			planes++
			if axis == 1 {
				if len(f.n.Loops) != 2 || len(f.n.Loops[1].CoEdges) != 1 {
					return nil, false, nil
				}
				circle, ok := f.n.Loops[1].CoEdges[0].Edge.Curve.(surfacegeom.Circle3)
				if !ok || circle.Center.Y != p.Box[1][side] ||
					circle.Axis != (r3.Vec{Y: 1}) && circle.Axis != (r3.Vec{Y: -1}) {
					return nil, false, nil
				}
				if p.Radius == 0 {
					p.Center, p.Radius = [2]float64{circle.Center.X, circle.Center.Z}, circle.Radius.Base()
				} else if p.Center != [2]float64{circle.Center.X, circle.Center.Z} || p.Radius != circle.Radius.Base() {
					return nil, false, nil
				}
			} else if len(f.n.Loops) != 1 {
				return nil, false, nil
			}
		case surfacegeom.Cylinder:
			if boreFace != nil {
				return nil, false, nil
			}
			boreFace, boreSurface = f, surface
		default:
			return nil, false, nil
		}
	}
	if planes != 6 || boreFace == nil || p.Radius <= 0 ||
		boreSurface.Axis != (r3.Vec{Y: 1}) && boreSurface.Axis != (r3.Vec{Y: -1}) ||
		boreSurface.Origin.X != p.Center[0] || boreSurface.Origin.Z != p.Center[1] ||
		boreSurface.Radius.Base() != p.Radius || !boreFace.n.Reversed || len(boreFace.n.Loops) != 2 {
		return nil, false, nil
	}
	var rims [2]bool
	for _, loop := range boreFace.n.Loops {
		if len(loop.CoEdges) != 1 {
			return nil, false, nil
		}
		edge := loop.CoEdges[0].Edge
		circle, ok := edge.Curve.(surfacegeom.Circle3)
		if !ok || !edge.CurveBounded || edge.CurveBound != 0 ||
			circle.Radius.Base() != p.Radius {
			return nil, false, nil
		}
		side := -1
		for s := range 2 {
			if circle.Center == r3.NewVec(p.Center[0], p.Box[1][s], p.Center[1]) {
				side = s
			}
		}
		if side < 0 || rims[side] {
			return nil, false, nil
		}
		rims[side] = true
	}
	for axis := range seen {
		if !seen[axis][0] || !seen[axis][1] {
			return nil, false, nil
		}
	}
	if !p.ExactCoordinates() {
		return nil, true, fmt.Errorf(`%w: an all-edge chamfer corner is not exactly representable`, ErrUnsupported)
	}
	if p.Center[0]-p.Radius <= p.Box[0][0]+distance ||
		p.Center[0]+p.Radius >= p.Box[0][1]-distance ||
		p.Center[1]-p.Radius <= p.Box[2][0]+distance ||
		p.Center[1]+p.Radius >= p.Box[2][1]-distance {
		return nil, true, fmt.Errorf(`%w: the bore reaches the all-edge chamfer`, ErrUnsupported)
	}
	body, err := bodyResult(breprecord.BuildAllEdgeChamfer(ctx, envOf(source.n.Doc, source.n.Doc.NextProducer), p))
	return body, true, err
}
