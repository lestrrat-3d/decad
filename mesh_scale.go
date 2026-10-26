package decad

import (
	"context"
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// ScaleMesh stretches m by a separate factor along each world axis about anchor
// and returns the result as a new [MeshBody] (agent-docs/mesh-import-design.md
// §10). m is not changed.
//
// Every vertex p moves to anchor + (p − anchor) scaled by x, y and z along the
// axes. A uniform stretch is a similarity, and any other is not: it turns a
// sphere into an ellipsoid and tilts every face that is not square to an axis.
// The volume of a closed mesh is multiplied by |x·y·z|, and an area by no one
// factor.
//
// A negative factor mirrors the mesh across the plane through anchor square to
// that axis. An odd number of negative factors would turn the triangles inside
// out, so their corners are reordered to keep a closed mesh wound outward and
// its [MeshBody.Volume] positive.
//
// ctx and m MUST NOT be nil, each otherwise ErrDegenerate. Each factor is
// dimensionless, finite and not zero: a wrong unit kind is ErrUnitKind, a
// non-finite factor or anchor ErrNotFinite and a zero factor ErrDegenerate. A
// result whose vertices overflow is ErrNotFinite, and a triangle too large to
// bound its area ErrUnsupported. A canceled context returns its
// error.
func ScaleMesh(ctx context.Context, m *MeshBody, anchor r3.Vec, x, y, z units.Value) (*MeshBody, error) {
	if ctx == nil {
		return nil, fmt.Errorf(`%w: a nil context cannot control a mesh scale`, ErrDegenerate)
	}
	if m == nil {
		return nil, fmt.Errorf(`%w: a mesh scale needs a mesh body`, ErrDegenerate)
	}
	var s [3]float64
	for i, f := range []units.Value{x, y, z} {
		if f.Kind() != units.Dimensionless {
			return nil, fmt.Errorf(`%w: the mesh scale factor along %c must be dimensionless`, ErrUnitKind, "xyz"[i])
		}
		s[i] = f.Base()
		if math.IsNaN(s[i]) || math.IsInf(s[i], 0) {
			return nil, fmt.Errorf(`%w: the mesh scale factor along %c is not finite`, ErrNotFinite, "xyz"[i])
		}
		if s[i] == 0 {
			return nil, fmt.Errorf(`%w: the mesh scale factor along %c is zero`, ErrDegenerate, "xyz"[i])
		}
	}
	for _, c := range []float64{anchor.X, anchor.Y, anchor.Z} {
		if math.IsNaN(c) || math.IsInf(c, 0) {
			return nil, fmt.Errorf(`%w: the mesh scale anchor is not finite`, ErrNotFinite)
		}
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	vertices := make([]r3.Vec, len(m.vertices))
	for i, p := range m.vertices {
		vertices[i] = r3.Vec{
			X: anchor.X + (p.X-anchor.X)*s[0],
			Y: anchor.Y + (p.Y-anchor.Y)*s[1],
			Z: anchor.Z + (p.Z-anchor.Z)*s[2],
		}
	}
	for _, p := range vertices {
		if !proofbound.FiniteVec(p) {
			return nil, fmt.Errorf(`%w: the scaled mesh has a vertex past the largest float`, ErrNotFinite)
		}
	}
	triangles := append([][3]int(nil), m.triangles...)
	if s[0]*s[1]*s[2] < 0 {
		for i, t := range triangles {
			triangles[i] = [3]int{t[0], t[2], t[1]}
		}
	}
	return NewMeshBody(ctx, vertices, triangles)
}

// ScaleBodyMesh stretches a solid body by a separate factor along each world
// axis about anchor and returns the result as a [MeshBody]
// (agent-docs/mesh-import-design.md §10). The solid kernel has no per-axis
// scale: [ScaledAt] is uniform and [Transform] is rigid, and a stretched sphere
// is no sphere. So b is tessellated at chord tolerance tol, and the mesh is
// stretched as [ScaleMesh] stretches one. b is not changed.
//
// The result is a mesh of the stretched solid, not a [Body]: no feature takes it,
// but it reads, queries and exports as a mesh does. Its volume is that of the
// tessellation times |x·y·z|, so it holds the volume of b to the tessellation's
// own error, which tol bounds for a curved body and which is nil for a body of
// flat faces. opts are [Body.Tessellate]'s. ctx and b MUST NOT be nil; the other
// gates, with those of tol, are ScaleMesh's and Tessellate's, and a sheet or a
// body that will not tessellate is Tessellate's refusal.
func ScaleBodyMesh(ctx context.Context, b *Body, anchor r3.Vec, x, y, z, tol units.Value, opts ...TessellateOption) (*MeshBody, error) {
	if ctx == nil {
		return nil, fmt.Errorf(`%w: a nil context cannot control a mesh scale`, ErrDegenerate)
	}
	if b == nil {
		return nil, fmt.Errorf(`%w: a mesh scale needs a body`, ErrDegenerate)
	}
	mesh, err := b.Tessellate(ctx, tol, opts...)
	if err != nil {
		return nil, err
	}
	m, err := NewMeshBody(ctx, mesh.Vertices(), mesh.Triangles())
	if err != nil {
		return nil, err
	}
	return ScaleMesh(ctx, m, anchor, x, y, z)
}
