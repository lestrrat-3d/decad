package decad

import (
	"context"
	"errors"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/meshrefine"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/units"
)

// RefineMesh splits the long edges of m and returns the result as a new
// [MeshBody] (agent-docs/mesh-import-design.md §11). m is not changed.
//
// Every edge longer than maxEdge is cut at its midpoint, and each triangle is
// cut along its split edges, again and again until no edge is longer. Both
// triangles on a shared edge take the same new vertex, so a closed mesh stays
// closed, its winding is kept and its orientation readings stand. No vertex of m
// moves, so the mesh covers the same surface: its area and volume are those of m,
// to the exactness of the arithmetic, and the refinement adds triangles and no
// shape. Use it before a step that needs short edges, such as a finer
// tessellation to scale or reduce. A mesh with no long edge comes back as a copy.
//
// ctx and m MUST NOT be nil, each otherwise ErrDegenerate. maxEdge is a positive
// length: zero is ErrDegenerate, a negative value ErrNegativeMagnitude and a
// wrong unit kind ErrUnitKind. A refinement that needs over five million
// triangles, or whose edges are still long after 40 rounds, is ErrUnsupported. A
// canceled context returns its error.
func RefineMesh(ctx context.Context, m *MeshBody, maxEdge units.Value) (*MeshBody, error) {
	if ctx == nil {
		return nil, fmt.Errorf(`%w: a nil context cannot control a mesh refinement`, ErrDegenerate)
	}
	if m == nil {
		return nil, fmt.Errorf(`%w: a mesh refinement needs a mesh body`, ErrDegenerate)
	}
	limit, err := sectionrecord.MagnitudeIn(maxEdge, units.Length, units.Millimeter, "the longest edge")
	if err != nil {
		return nil, err
	}
	if limit == 0 {
		return nil, fmt.Errorf(`%w: the longest edge must be positive`, ErrDegenerate)
	}
	got, err := meshrefine.Refine(ctx, m.vertices, m.triangles, limit)
	if errors.Is(err, meshrefine.ErrTooLarge) {
		return nil, fmt.Errorf(`%w: %s`, ErrUnsupported, err)
	}
	if err != nil {
		return nil, err
	}
	return NewMeshBody(ctx, got.Vertices, got.Triangles)
}
