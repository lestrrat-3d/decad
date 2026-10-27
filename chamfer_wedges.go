package decad

import (
	"context"
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/orthoshell"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// maxWedgeEdges bounds the edges one wedge chamfer takes, which also bounds the distinct margins its tools need.
const maxWedgeEdges = 32

// wedge is one convex edge of an orthogonal solid in plain data: its ends a and b along an axis, the outward unit
// normals n1 and n2 of the two faces that meet at it, and the margins its tool reaches past both faces (m, the same for
// both so that the chamfer plane stays at 45 degrees) and past the ends (e0, e1), each distinct from every other tool's
// so that no two tools share a plane.
type wedge struct {
	a, b, n1, n2 r3.Vec
	m, e0, e1    float64
}

// chamferByWedges chamfers edges of the axis-square faceted solid b by d (agent-docs/orthogonal-boolean-design.md B17):
// the chamfer of a convex edge removes the triangular wedge between the two faces and the plane that cuts d along each,
// so the result is b less the union of those wedges, which at a corner where two or more selected edges meet is the
// solid within every chamfer plane, as a CAD chamfer of several edges is. Each wedge is cut by a prism a little longer
// than the edge and a little taller than the wedge, so that no face of the tool lies in a face of the solid, one Cut
// after another. handled is false, with no error, when the route does not apply: a receiver that is not an axis-square
// solid with exact vertices, an edge that is not a convex edge square to the axes between two such faces, or an edge
// whose neighbourhood holds material beyond the wedge, where a longer tool would cut more than the wedge, and the
// caller keeps its refusal.
func chamferByWedges(ctx context.Context, d *Document, b *Body, edges []*Edge, dmm float64) (result *Body, handled bool, err error) {
	payload, ok := orthogonalOperand(b)
	if !ok || len(edges) == 0 || len(edges) > maxWedgeEdges || !(dmm > 0) || math.IsInf(dmm, 0) {
		return nil, false, nil
	}
	wedges := make([]wedge, 0, len(edges))
	for k, e := range edges {
		w, ok := orthogonalWedge(e, dmm, k)
		if !ok {
			return nil, false, nil
		}
		if !wedgeFits(payload, w, dmm) {
			return nil, false, nil
		}
		wedges = append(wedges, w)
	}
	toolCtx, capture := privateConstructionContext(ctx)
	var tools []*Body
	defer func() { removeTools(tools) }()
	for _, w := range wedges {
		tool, err := buildWedgeTool(toolCtx, d, w, dmm)
		if err != nil {
			return nil, false, err
		}
		tools = append(tools, tool)
	}
	out, err := combineTools(ctx, d, OperationChamfer, b, meshbool.OpCut, tools, false, nil, capture)
	if err != nil {
		return nil, false, err
	}
	return out, true, nil
}

// wedgeFits reports whether the wedge's tool cuts the wedge and no more. The tool's box holds nothing of the solid beyond
// the two faces or past the ends, so the part of the solid in the box is the part in the box's quadrant behind both faces,
// and the block d deep behind both faces along the whole edge is solid, so the wedge in it is material to remove.
func wedgeFits(payload facetedPayload, w wedge, d float64) bool {
	length := w.b.Sub(w.a).Len()
	lo, hi := wedgeBox(w, d)
	verts, tris := axisBox(lo, hi)
	inBox, boundBox, err := orthoshell.OverlapVolume(payload.Verts, payload.Tris, verts, tris)
	if err != nil {
		return false
	}
	// The quadrant of the box behind both faces: from the edge d plus the margin back, along the edge only its length.
	back := w.a.Sub(w.n1.Scale(d + w.m)).Sub(w.n2.Scale(d + w.m))
	far := w.b
	qlo, qhi := orderedCorners(back, far)
	qv, qt := axisBox(qlo, qhi)
	inQuadrant, boundQ, err := orthoshell.OverlapVolume(payload.Verts, payload.Tris, qv, qt)
	if err != nil || math.Abs(inBox-inQuadrant) > boundBox+boundQ+1e-9*math.Max(1, inBox) {
		return false
	}
	// The d by d block behind both faces along the whole edge is wholly solid.
	block := w.a.Sub(w.n1.Scale(d)).Sub(w.n2.Scale(d))
	blo, bhi := orderedCorners(block, w.b)
	bv, bt := axisBox(blo, bhi)
	inBlock, boundB, err := orthoshell.OverlapVolume(payload.Verts, payload.Tris, bv, bt)
	return err == nil && math.Abs(inBlock-d*d*length) <= boundB+1e-9*math.Max(1, d*d*length)
}

// orderedCorners returns the componentwise least and greatest of two corners.
func orderedCorners(p, q r3.Vec) (lo, hi r3.Vec) {
	return r3.NewVec(math.Min(p.X, q.X), math.Min(p.Y, q.Y), math.Min(p.Z, q.Z)),
		r3.NewVec(math.Max(p.X, q.X), math.Max(p.Y, q.Y), math.Max(p.Z, q.Z))
}

// orthogonalWedge reads edge e, the k-th of the selection, as a wedge of a chamfer of distance d: a straight convex edge
// along a world axis between two faces whose outward normals are world axes square to it and to each other.
func orthogonalWedge(e *Edge, d float64, k int) (wedge, bool) {
	faces := e.Faces()
	if len(faces) != 2 || !e.IsConvex() {
		return wedge{}, false
	}
	a, b := e.Start().Position().Value, e.End().Position().Value
	dir := b.Sub(a)
	axis, _, ok := unitAxis(dir.Scale(1 / dir.Len()))
	if !ok || !(dir.Len() > 0) {
		return wedge{}, false
	}
	mid := a.Add(dir.Scale(0.5))
	var normals [2]r3.Vec
	for i, f := range faces {
		n, err := f.NormalAt(mid)
		if err != nil {
			return wedge{}, false
		}
		ax, sign, ok := unitAxis(n.Value)
		if !ok || ax == axis {
			return wedge{}, false
		}
		normals[i] = unitVec(ax, sign)
	}
	if normals[0].Dot(normals[1]) != 0 {
		return wedge{}, false
	}
	// Margins from half of d to d, distinct across the selection and across a tool's three, so a tool's faces are well
	// clear of the solid's and never in a plane of another tool's.
	margin := func(i int) float64 { return d * (0.5 + float64(i)/(2*3*maxWedgeEdges)) }
	return wedge{
		a: a, b: b, n1: normals[0], n2: normals[1],
		m: margin(3*k + 1), e0: margin(3*k + 2), e1: margin(3*k + 3),
	}, true
}

// unitVec is the unit vector along axis with the sign.
func unitVec(axis int, sign float64) r3.Vec {
	var c [3]float64
	c[axis] = sign
	return r3.NewVec(c[0], c[1], c[2])
}

// wedgeBox is the axis-aligned box that holds the wedge's tool: from d plus the margin behind each face to the margin
// beyond it, and from the margin before the edge's start to the margin past its end.
func wedgeBox(w wedge, d float64) (lo, hi r3.Vec) {
	dir := w.b.Sub(w.a)
	dir = dir.Scale(1 / dir.Len())
	p := w.a.Sub(dir.Scale(w.e0)).Add(w.n1.Scale(w.m)).Add(w.n2.Scale(w.m))
	q := w.b.Add(dir.Scale(w.e1)).Sub(w.n1.Scale(d + w.m)).Sub(w.n2.Scale(d + w.m))
	return orderedCorners(p, q)
}

// axisBox is the closed outward triangle mesh of the box from lo to hi.
func axisBox(lo, hi r3.Vec) ([]r3.Vec, [][3]int) {
	v := []r3.Vec{
		{X: lo.X, Y: lo.Y, Z: lo.Z}, {X: hi.X, Y: lo.Y, Z: lo.Z}, {X: hi.X, Y: hi.Y, Z: lo.Z}, {X: lo.X, Y: hi.Y, Z: lo.Z},
		{X: lo.X, Y: lo.Y, Z: hi.Z}, {X: hi.X, Y: lo.Y, Z: hi.Z}, {X: hi.X, Y: hi.Y, Z: hi.Z}, {X: lo.X, Y: hi.Y, Z: hi.Z},
	}
	var tris [][3]int
	for _, q := range [][4]int{{0, 3, 2, 1}, {4, 5, 6, 7}, {0, 1, 5, 4}, {3, 7, 6, 2}, {0, 4, 7, 3}, {1, 2, 6, 5}} {
		tris = append(tris, [3]int{q[0], q[1], q[2]}, [3]int{q[0], q[2], q[3]})
	}
	return v, tris
}

// buildWedgeTool extrudes the wedge's tool: the triangle in the plane square to the edge with legs along the two faces'
// normals, from m1 and m2 past the faces to the chamfer plane's reach d plus the margin behind them, swept along the
// edge from e0 before its start to e1 past its end.
func buildWedgeTool(ctx context.Context, d *Document, w wedge, dmm float64) (*Body, error) {
	dir := w.b.Sub(w.a)
	length := dir.Len()
	dir = dir.Scale(1 / length)
	origin := w.a.Sub(dir.Scale(w.e0))
	frame, err := r3.NewFrame(origin, w.n1, w.n2)
	if err != nil {
		return nil, fmt.Errorf(`%w: a chamfer wedge's plane is degenerate: %s`, ErrDegenerate, err)
	}
	world := sketch.NewWorld()
	pl, err := world.CreatePlaneFromFrame(frame)
	if err != nil {
		return nil, err
	}
	s, err := world.CreateSketch(pl)
	if err != nil {
		return nil, err
	}
	corners := [][2]float64{{w.m, w.m}, {w.m, -dmm - w.m}, {-dmm - w.m, w.m}}
	points := make([]*sketch.Point, len(corners))
	for i, c := range corners {
		points[i] = s.CreatePoint(c[0], c[1])
		s.Fix(points[i])
	}
	for i := range points {
		s.CreateLine(points[i], points[(i+1)%len(points)])
	}
	if _, err := s.Solve(ctx); err != nil {
		return nil, err
	}
	way := Along
	if frame.N().Dot(dir) < 0 {
		way = Against
	}
	return Extrude(ctx, d, s, s.Profiles()[0], Distance{D: units.Millimeters(length + w.e0 + w.e1), Dir: way})
}
