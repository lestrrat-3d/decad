package decad

import (
	"context"
	"fmt"

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
