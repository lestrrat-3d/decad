package decad

import (
	"context"
	"errors"
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/meshreduce"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/units"
)

// MeshReduceReport states what [ReduceMesh] changed.
type MeshReduceReport struct {
	// MergedVertices counts the vertices that shared a cell with an earlier one,
	// and DroppedTriangles the triangles that collapsed to fewer than three
	// cells or repeated another triangle.
	MergedVertices, DroppedTriangles int
	// MaxVertexMove is the greatest distance from an original vertex to the
	// vertex that replaced it. It bounds how far the vertices moved, and is not a
	// bound on how far the surface moved: a triangle can fold or pass through
	// others as its corners move.
	MaxVertexMove units.Value
	// SurfaceBound is set by [ReduceMeshWithin] and is the zero Value after [ReduceMesh]. It is a
	// proven bound on the distance between the input surface and the result, in both directions.
	SurfaceBound units.Value
}

// ReduceMesh thins m by vertex clustering and returns the result as a new
// [MeshBody] with a report (agent-docs/mesh-import-design.md §9). m is not
// changed.
//
// Space is cut into cubic cells of side cell aligned to the origin. All the
// vertices in a cell are replaced by their mean, and each triangle is rebuilt
// over the replacements: one whose corners share a cell, or that repeats another
// triangle's cells, is dropped. A cell larger than the mesh leaves nothing, and
// one smaller than the vertex spacing leaves the mesh as it was.
//
// The result need not be closed, manifold or free of self-intersection even when
// m is: read [MeshBody.Report] on it. Reducing shrinks convex corners, because
// the mean of a corner's cell lies inside it, and a coarse cell can thicken or
// pinch thin features away.
//
// ctx and m MUST NOT be nil, each otherwise ErrDegenerate. cell is a positive
// length: zero is ErrDegenerate, a negative value ErrNegativeMagnitude and a
// wrong unit kind ErrUnitKind. A reduction that leaves no triangle is
// ErrDegenerate. A canceled context returns its error.
func ReduceMesh(ctx context.Context, m *MeshBody, cell units.Value) (*MeshBody, MeshReduceReport, error) {
	if ctx == nil {
		return nil, MeshReduceReport{}, fmt.Errorf(`%w: a nil context cannot control a mesh reduction`, ErrDegenerate)
	}
	if m == nil {
		return nil, MeshReduceReport{}, fmt.Errorf(`%w: a mesh reduction needs a mesh body`, ErrDegenerate)
	}
	size, err := sectionrecord.MagnitudeIn(cell, units.Length, units.Millimeter, "the mesh reduction cell")
	if err != nil {
		return nil, MeshReduceReport{}, err
	}
	if size == 0 {
		return nil, MeshReduceReport{}, fmt.Errorf(`%w: the mesh reduction cell must be positive`, ErrDegenerate)
	}
	got, err := meshreduce.Reduce(ctx, m.vertices, m.triangles, size)
	if err != nil {
		return nil, MeshReduceReport{}, err
	}
	if len(got.Triangles) == 0 {
		return nil, MeshReduceReport{}, fmt.Errorf(`%w: a %g mm cell leaves no triangle of the mesh`, ErrDegenerate, size)
	}
	out, err := NewMeshBody(ctx, got.Vertices, got.Triangles)
	if err != nil {
		return nil, MeshReduceReport{}, err
	}
	return out, MeshReduceReport{
		MergedVertices: got.MergedVertices, DroppedTriangles: got.DroppedTriangles,
		MaxVertexMove: units.Millimeters(got.MaxMove),
	}, nil
}

// ReduceMeshWithin thins m by vertex clustering as [ReduceMesh] does, with the
// coarsest cell it can prove keeps the result within distance of m's surface, and
// returns the new [MeshBody] with a report whose SurfaceBound states the proven
// distance (agent-docs/mesh-import-design.md §12).
//
// The proof is the one the clustering gives: each vertex moves to its cell's mean,
// so the result is the image of m under the map that is affine on each triangle, and
// every point of m is within the greatest vertex move of its image. That image is
// the result wherever a triangle survived, and a triangle that collapsed to a
// segment or a point lies on a kept triangle's edge or corner, which the reduction
// checks; otherwise the cell is not used. The bound is the greatest move raised above
// the rounding of the distances. It holds in both directions, but says nothing
// of the result's topology: it need not be closed or free of self-intersection even
// when m is, as [ReduceMesh] says.
//
// ctx and m MUST NOT be nil, each otherwise ErrDegenerate. distance is a positive
// length: zero is ErrDegenerate, a negative value ErrNegativeMagnitude and a
// wrong unit kind ErrUnitKind. A distance no cell can prove, or one that leaves no
// triangle, is ErrUnsupported. A canceled context returns its error.
func ReduceMeshWithin(ctx context.Context, m *MeshBody, distance units.Value) (*MeshBody, MeshReduceReport, error) {
	if ctx == nil {
		return nil, MeshReduceReport{}, fmt.Errorf(`%w: a nil context cannot control a mesh reduction`, ErrDegenerate)
	}
	if m == nil {
		return nil, MeshReduceReport{}, fmt.Errorf(`%w: a mesh reduction needs a mesh body`, ErrDegenerate)
	}
	limit, err := sectionrecord.MagnitudeIn(distance, units.Length, units.Millimeter, "the mesh reduction distance")
	if err != nil {
		return nil, MeshReduceReport{}, err
	}
	if limit == 0 {
		return nil, MeshReduceReport{}, fmt.Errorf(`%w: the mesh reduction distance must be positive`, ErrDegenerate)
	}
	got, bound, ok, err := meshreduce.ReduceWithin(ctx, m.vertices, m.triangles, limit)
	if err != nil {
		return nil, MeshReduceReport{}, err
	}
	if !ok {
		return nil, MeshReduceReport{}, fmt.Errorf(`%w: no cell proves a reduction within %g mm of the mesh`, ErrUnsupported, limit)
	}
	out, err := NewMeshBody(ctx, got.Vertices, got.Triangles)
	if err != nil {
		return nil, MeshReduceReport{}, err
	}
	return out, MeshReduceReport{
		MergedVertices: got.MergedVertices, DroppedTriangles: got.DroppedTriangles,
		MaxVertexMove: units.Millimeters(got.MaxMove), SurfaceBound: units.Millimeters(bound),
	}, nil
}

// CollapseMesh thins the closed mesh m by edge collapse until it has at most triangles triangles, and returns the
// result as a new [MeshBody] with a report (agent-docs/mesh-import-design.md §13). m is not changed.
//
// The shortest edge is merged first, at its midpoint, then the next shortest, and so on. An edge is merged only when
// the surface stays a closed manifold (its two ends share just the two vertices opposite it), no triangle around it
// turns more than about 78 degrees, and at least four triangles remain, so the result is closed and wound as m is.
// Each merge removes two triangles, so the result has triangles or one fewer, or more when no remaining edge can be
// merged. The report's MergedVertices counts the merges, DroppedTriangles is twice that and MaxVertexMove is the
// greatest distance from a vertex of m to the vertex it ended at. SurfaceBound stays the zero Value: a collapse
// moves the surface by a distance the evaluator does not prove, and the result can pass through itself where a
// coarse mesh pinches a thin feature.
//
// ctx and m MUST NOT be nil, each otherwise ErrDegenerate, and triangles is at least 4 or ErrDegenerate. A mesh that
// is not closed, consistently wound and manifold is ErrUnsupported. A canceled context returns its error.
func CollapseMesh(ctx context.Context, m *MeshBody, triangles int) (*MeshBody, MeshReduceReport, error) {
	if ctx == nil {
		return nil, MeshReduceReport{}, fmt.Errorf(`%w: a nil context cannot control a mesh collapse`, ErrDegenerate)
	}
	if m == nil {
		return nil, MeshReduceReport{}, fmt.Errorf(`%w: a mesh collapse needs a mesh body`, ErrDegenerate)
	}
	if triangles < 4 {
		return nil, MeshReduceReport{}, fmt.Errorf(`%w: a closed mesh keeps at least 4 triangles, not %d`, ErrDegenerate, triangles)
	}
	got, err := meshreduce.Collapse(ctx, m.vertices, m.triangles, triangles)
	if err != nil {
		if errors.Is(err, meshreduce.ErrNotClosed) {
			return nil, MeshReduceReport{}, fmt.Errorf(`%w: %s`, ErrUnsupported, err)
		}
		return nil, MeshReduceReport{}, err
	}
	out, err := NewMeshBody(ctx, got.Vertices, got.Triangles)
	if err != nil {
		return nil, MeshReduceReport{}, err
	}
	return out, MeshReduceReport{
		MergedVertices: got.Collapses, DroppedTriangles: 2 * got.Collapses, MaxVertexMove: units.Millimeters(got.MaxMove),
	}, nil
}

// RemeshMesh rebuilds the closed mesh m from near-equilateral triangles with edges near edge, and returns the result
// as a new [MeshBody] (agent-docs/mesh-import-design.md §14). m is not changed.
//
// It makes eight passes of incremental isotropic remeshing: split the edges longer than 4/3 of edge, merge the edges
// shorter than 4/5 of it, flip edges toward six neighbours to a vertex, and slide each vertex toward its neighbours'
// centroid along the surface. Every step keeps the surface closed, manifold and wound as m is, and refuses a move
// that would turn a triangle by about 78 degrees or more. The vertices are not projected back onto m, so a convex
// surface shrinks a little in each pass and the result stands within a few per cent of m's volume, with no proven
// distance; a thin feature narrower than edge can pinch or pass through itself.
//
// ctx and m MUST NOT be nil, each otherwise ErrDegenerate. edge is a positive length: zero is ErrDegenerate, a
// negative value ErrNegativeMagnitude and a wrong unit kind ErrUnitKind. A mesh that is not closed, consistently
// wound and manifold is ErrUnsupported, and so is an edge so short that the result would pass a million triangles.
// A canceled context returns its error.
func RemeshMesh(ctx context.Context, m *MeshBody, edge units.Value) (*MeshBody, error) {
	if ctx == nil {
		return nil, fmt.Errorf(`%w: a nil context cannot control a remesh`, ErrDegenerate)
	}
	if m == nil {
		return nil, fmt.Errorf(`%w: a remesh needs a mesh body`, ErrDegenerate)
	}
	length, err := sectionrecord.MagnitudeIn(edge, units.Length, units.Millimeter, "the remesh edge length")
	if err != nil {
		return nil, err
	}
	if length == 0 {
		return nil, fmt.Errorf(`%w: the remesh edge length must be positive`, ErrDegenerate)
	}
	area, err := m.Area()
	if err != nil {
		return nil, err
	}
	if area.Value.Base()/(math.Sqrt(3)/4*length*length) > maxRemeshTriangles {
		return nil, fmt.Errorf(`%w: a %g mm edge would make over %d triangles`, ErrUnsupported, length, maxRemeshTriangles)
	}
	got, err := meshreduce.Remesh(ctx, m.vertices, m.triangles, length)
	if err != nil {
		if errors.Is(err, meshreduce.ErrNotClosed) {
			return nil, fmt.Errorf(`%w: %s`, ErrUnsupported, err)
		}
		return nil, err
	}
	return NewMeshBody(ctx, got.Vertices, got.Triangles)
}

// maxRemeshTriangles bounds the triangles a remesh may be asked to make.
const maxRemeshTriangles = 1_000_000
