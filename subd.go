package decad

import (
	"context"
	"errors"
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/subd"
	"github.com/lestrrat-3d/r3"
)

// SubDCage is a polygon cage for [SubDMesh]: Faces list indices into Vertices,
// each face of three or more distinct vertices wound counter-clockwise seen from
// outside the shape. Vertices are in millimetres.
type SubDCage struct {
	Vertices []r3.Vec
	Faces    [][]int
}

// SubDMesh refines a polygon cage into its Catmull-Clark subdivision surface, a
// smooth shape drawn by moving the cage's vertices, as the Form workspace of a
// CAD system sculpts one, and returns it as a [MeshBody] (agent-docs/subd-design.md).
// Each of levels rounds replaces every n-gon by n quads and moves the vertices
// by the Catmull-Clark rules, so the mesh converges on the limit surface, which
// lies inside a convex cage. A closed cage gives a
// closed mesh whose [MeshBody.Volume] reads, and an open cage gives an open mesh
// whose boundary keeps to the limit of the cage's boundary curve.
//
// levels is from 1 to 8, and no round may make more than 1,000,000 quads,
// otherwise ErrDegenerate. The cage must be a manifold of polygons: an edge used by more than
// two faces, two faces that walk an edge the same way, a face of fewer than three
// distinct vertices or naming a vertex that is not there, an interior vertex in fewer
// than three faces, and a boundary vertex that does not touch exactly two boundary
// edges are ErrDegenerate. A vertex that is not finite is ErrNotFinite, and a nil
// context is ErrDegenerate.
func SubDMesh(ctx context.Context, cage SubDCage, levels int) (*MeshBody, error) {
	if ctx == nil {
		return nil, fmt.Errorf(`%w: a nil context cannot control a subdivision`, ErrDegenerate)
	}
	if levels < 1 {
		return nil, fmt.Errorf(`%w: a subdivision needs at least one level, not %d`, ErrDegenerate, levels)
	}
	for _, v := range cage.Vertices {
		if math.IsNaN(v.X+v.Y+v.Z) || math.IsInf(v.X+v.Y+v.Z, 0) {
			return nil, fmt.Errorf(`%w: a cage vertex is not finite`, ErrNotFinite)
		}
	}
	refined, err := subd.Subdivide(subd.Cage{Vertices: cage.Vertices, Faces: cage.Faces}, levels)
	if errors.Is(err, subd.ErrDegenerate) {
		return nil, fmt.Errorf("%w: %s", ErrDegenerate, err)
	}
	if err != nil {
		return nil, err
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	return NewMeshBody(ctx, refined.Vertices, refined.Triangles())
}
