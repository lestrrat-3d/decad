package decad

import (
	"context"
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/meshbody"
	"github.com/lestrrat-3d/decad/internal/pointquery"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// MeshReport states what a [MeshBody]'s triangles say about it. The counts are
// read from the triangles alone; none of them depends on a tolerance.
type MeshReport struct {
	// Vertices and Triangles count the input.
	Vertices, Triangles int
	// DegenerateTriangles counts triangles of zero area, including those that
	// name a vertex twice. They add no area or volume and join no shell.
	DegenerateTriangles int
	// BoundaryEdges counts edges that exactly one triangle uses.
	BoundaryEdges int
	// NonManifoldEdges counts edges that three or more triangles use.
	NonManifoldEdges int
	// MisorientedEdges counts edges that exactly two triangles use in the same
	// direction.
	MisorientedEdges int
	// Shells counts connected groups of triangles joined across shared edges.
	Shells int
	// Closed reports a mesh whose every edge is used by exactly two triangles
	// in opposite directions.
	Closed bool
	// Inverted reports a closed mesh whose signed volume is negative, that is
	// one wound clockwise seen from outside.
	Inverted bool
}

// MeshBody is an imported triangle mesh (agent-docs/mesh-import-design.md). It
// NEVER claims to be a solid B-rep: its triangles are the whole of what it
// knows, it has no faces, edges or analytic surfaces, and no feature or Boolean
// takes it. It reads out what the triangles say: bounds, area, and the volume of
// a closed, consistently wound mesh, each as an exact or bounded measurement of
// the mesh itself. It is immutable.
type MeshBody struct {
	_         noCopy
	vertices  []r3.Vec
	triangles [][3]int
	report    MeshReport
	signed    *big.Rat
	areaLow   *big.Rat
	areaHigh  *big.Rat
	box       Box
}

// NewMeshBody builds a mesh body from vertices in millimetres and triangles as
// index triples into them. The triangles wind counter-clockwise seen from
// outside the body they enclose. vertices and triangles are copied.
//
// A nil context is ErrDegenerate. A mesh with no triangle is ErrDegenerate. A
// vertex that is not finite, or a triangle index outside vertices, is
// ErrUnsupported. NewMeshBody welds and repairs nothing: whatever the
// triangles say is what [MeshBody.Report] reports. A canceled context returns
// its error.
func NewMeshBody(ctx context.Context, vertices []r3.Vec, triangles [][3]int) (*MeshBody, error) {
	if ctx == nil {
		return nil, fmt.Errorf(`%w: a nil context cannot control a mesh read`, ErrDegenerate)
	}
	if len(triangles) == 0 {
		return nil, fmt.Errorf(`%w: a mesh body needs at least one triangle`, ErrDegenerate)
	}
	a, err := meshbody.Analyze(ctx, vertices, triangles)
	if err != nil {
		return nil, err
	}
	m := &MeshBody{
		vertices:  append([]r3.Vec(nil), vertices...),
		triangles: append([][3]int(nil), triangles...),
		signed:    a.SignedVolume,
		areaLow:   a.AreaLow,
		areaHigh:  a.AreaHigh,
	}
	m.report = MeshReport{
		Vertices: a.Vertices, Triangles: a.Triangles, DegenerateTriangles: a.DegenerateTriangles,
		BoundaryEdges: a.BoundaryEdges, NonManifoldEdges: a.NonManifoldEdges, MisorientedEdges: a.MisorientedEdges,
		Shells: a.Shells, Closed: a.Closed(),
	}
	m.report.Inverted = m.report.Closed && a.SignedVolume.Sign() < 0
	if len(vertices) > 0 {
		lo, hi := vertices[0], vertices[0]
		for _, v := range vertices[1:] {
			lo = r3.Vec{X: min(lo.X, v.X), Y: min(lo.Y, v.Y), Z: min(lo.Z, v.Z)}
			hi = r3.Vec{X: max(hi.X, v.X), Y: max(hi.Y, v.Y), Z: max(hi.Z, v.Z)}
		}
		m.box = Box{Min: lo, Max: hi, Exactness: Exact, Bound: units.Millimeters(0)}
	}
	return m, nil
}

// Vertices returns the vertex positions in millimetres. The slice is a copy.
func (m *MeshBody) Vertices() []r3.Vec { return append([]r3.Vec(nil), m.vertices...) }

// Triangles returns the triangles as index triples into Vertices. The slice is
// a copy.
func (m *MeshBody) Triangles() [][3]int { return append([][3]int(nil), m.triangles...) }

// Report returns what the triangles say about the mesh.
func (m *MeshBody) Report() MeshReport { return m.report }

// Bounds returns the axis-aligned box of the vertices. It is Exact: the
// vertices are the mesh. A vertex that no triangle uses still counts.
func (m *MeshBody) Bounds() Box { return m.box }

// Area returns the total area of the triangles. Each triangle's area is a
// square root, so the reading is Approximate with a proven bound, except where
// every root is exact.
func (m *MeshBody) Area() (Measurement, error) {
	value, bound, ok := meshbody.Interval(m.areaLow, m.areaHigh)
	if !ok {
		return Measurement{}, fmt.Errorf(`%w: the mesh area is not representable`, ErrNotFinite)
	}
	return Measurement{Value: units.SquareMillimeters(value), Bound: units.SquareMillimeters(bound),
		Exactness: exactnessOfBound(bound)}, nil
}

// Volume returns the volume the mesh encloses. It is ErrNotSolid unless the
// mesh is closed and wound counter-clockwise seen from outside: a mesh with a
// boundary edge, a non-manifold or misoriented edge, or a negative signed
// volume encloses no volume the triangles can state. The reading is the exact
// sum of signed tetrahedra, so it is Exact when that sum is representable and
// otherwise carries only the rounding of its float value.
func (m *MeshBody) Volume() (Measurement, error) {
	if !m.report.Closed {
		return Measurement{}, fmt.Errorf(`%w: the mesh is not closed and consistently wound`, ErrNotSolid)
	}
	if m.report.Inverted {
		return Measurement{}, fmt.Errorf(`%w: the mesh is wound inside out`, ErrNotSolid)
	}
	value, bound, ok := meshbody.Interval(m.signed, m.signed)
	if !ok {
		return Measurement{}, fmt.Errorf(`%w: the mesh volume is not representable`, ErrNotFinite)
	}
	return Measurement{Value: units.CubicMillimeters(value), Bound: units.CubicMillimeters(bound),
		Exactness: exactnessOfBound(bound)}, nil
}

// exactnessOfBound is Exact for a zero bound and Approximate otherwise.
func exactnessOfBound(bound float64) Exactness {
	if bound == 0 {
		return Exact
	}
	return Approximate
}

// held returns the mesh as the exact held mesh the point queries read: its
// triangles are the boundary, so every bound is zero. It refuses a mesh that
// is not closed, which encloses no region to locate a point against.
func (m *MeshBody) held(ctx context.Context, p r3.Vec) (topo.Mesh, error) {
	if ctx == nil || m == nil {
		return topo.Mesh{}, fmt.Errorf(`%w: a point query needs a mesh body and a context`, ErrDegenerate)
	}
	if err := ctx.Err(); err != nil {
		return topo.Mesh{}, err
	}
	if !proofbound.FiniteVec(p) {
		return topo.Mesh{}, fmt.Errorf(`%w: the query point must be finite`, ErrNotFinite)
	}
	if !m.report.Closed {
		return topo.Mesh{}, fmt.Errorf(`%w: the mesh is not closed and consistently wound`, ErrNotSolid)
	}
	return topo.Mesh{Vertices: m.vertices, Triangles: m.triangles, BoundaryOK: true, SymDiffOK: true}, nil
}

// LocatePoint classifies p against the region the closed mesh encloses
// (agent-docs/mesh-import-design.md §5). The answer is exact, with no
// tolerance: a ray crossing count over the triangles decides inside from
// outside, and a point on a triangle is PointOnBoundary. Where the mesh crosses
// itself the count decides, so a region covered twice reads as outside. The
// mesh need not wind counter-clockwise seen from outside: an inverted mesh
// encloses the same region. A mesh that is not closed is ErrNotSolid. A nil
// context or mesh is ErrDegenerate, and a point that is not finite is
// ErrNotFinite.
func (m *MeshBody) LocatePoint(ctx context.Context, p r3.Vec) (PointLocation, error) {
	held, err := m.held(ctx, p)
	if err != nil {
		return PointUndecided, err
	}
	result, err := pointquery.Locate(ctx, p, held.Vertices, held.Triangles, 0, 0)
	return PointLocation(result), err
}

// DistanceToPoint measures the minimum distance in millimetres from p to the
// region the closed mesh encloses: zero for a point inside or on it, and
// otherwise the distance to the nearest triangle. The reading is computed in
// exact arithmetic from the mesh itself, so it carries only the rounding of
// the float it is published as. Errors are LocatePoint's.
func (m *MeshBody) DistanceToPoint(ctx context.Context, p r3.Vec) (Measurement, error) {
	held, err := m.held(ctx, p)
	if err != nil {
		return Measurement{}, err
	}
	return distanceResult(pointquery.BodyDistance(ctx, p, &held))
}

// ClosestPoint finds the point of the closed mesh's region nearest p: p itself
// when it lies inside or on the mesh, and otherwise the nearest point of the
// nearest triangle, rounded to floats. Its OffSurface is that rounding alone.
// Errors are LocatePoint's.
func (m *MeshBody) ClosestPoint(ctx context.Context, p r3.Vec) (ClosestPoint, error) {
	held, err := m.held(ctx, p)
	if err != nil {
		return ClosestPoint{}, err
	}
	return closestResult(pointquery.BodyClosest(ctx, p, &held))
}
