package decad

import (
	"context"
	"errors"
	"fmt"
	"io"

	"github.com/lestrrat-3d/decad/internal/igesbrep"
	"github.com/lestrrat-3d/decad/internal/igesread"
	"github.com/lestrrat-3d/decad/internal/meshparse"
	"github.com/lestrrat-3d/decad/internal/stepbrep"
	"github.com/lestrrat-3d/decad/internal/stepread"
	"github.com/lestrrat-go/option/v3"
)

// ErrMalformedMesh marks a mesh file that is not well-formed STL or OBJ: a bad
// header, a truncated record, a number that does not parse or an index that
// names no vertex.
var ErrMalformedMesh = meshparse.ErrMalformed

// ErrMeshTooLarge marks a mesh file with more triangles or bytes than the
// import limits allow. Raise a limit with [WithMaxMeshTriangles] or
// [WithMaxMeshBytes].
var ErrMeshTooLarge = meshparse.ErrTooLarge

// MeshImportOption configures [ReadSTL] and [ReadOBJ].
type MeshImportOption interface {
	option.Interface
	meshImportOption()
}

type meshImportOption struct{ option.Interface }

func (meshImportOption) meshImportOption() {}

type identMaxMeshTriangles struct{}

type identMaxMeshBytes struct{}

// WithMaxMeshTriangles sets the most triangles an import accepts. The default
// is 5,000,000. A file with more is [ErrMeshTooLarge], and n MUST be positive.
func WithMaxMeshTriangles(n int) MeshImportOption {
	return meshImportOption{option.New(identMaxMeshTriangles{}, n)}
}

// WithMaxMeshBytes sets the most bytes an import reads. The default is 512 MiB.
// A longer input is [ErrMeshTooLarge], and n MUST be positive.
func WithMaxMeshBytes(n int64) MeshImportOption {
	return meshImportOption{option.New(identMaxMeshBytes{}, n)}
}

func meshLimits(opts []MeshImportOption) (meshparse.Limits, error) {
	l := meshparse.Limits{Triangles: meshparse.DefaultMaxTriangles, Bytes: meshparse.DefaultMaxBytes}
	for _, o := range opts {
		if o == nil {
			return l, fmt.Errorf(`%w: a nil option names nothing to apply`, ErrDegenerate)
		}
		switch o.Ident().(type) {
		case identMaxMeshTriangles:
			n, ok := option.Get[int](o)
			if !ok || n <= 0 {
				return l, fmt.Errorf(`%w: the mesh triangle limit must be positive`, ErrDegenerate)
			}
			l.Triangles = n
		case identMaxMeshBytes:
			n, ok := option.Get[int64](o)
			if !ok || n <= 0 {
				return l, fmt.Errorf(`%w: the mesh byte limit must be positive`, ErrDegenerate)
			}
			l.Bytes = n
		}
	}
	return l, nil
}

// ReadSTL reads a binary or ASCII STL file into a [MeshBody]
// (agent-docs/mesh-import-design.md). A file is binary when its length is
// exactly 84 bytes plus 50 for each triangle its count names, and ASCII when it
// starts with "solid". Facet normals and attributes are ignored: a triangle's
// side comes from its vertex order. Coordinates are millimetres. Vertices with
// identical coordinates are welded and nothing else is repaired.
//
// ctx and r MUST NOT be nil, each otherwise ErrDegenerate. Malformed input is
// [ErrMalformedMesh] and input past a limit is [ErrMeshTooLarge]. A canceled
// context returns its error.
func ReadSTL(ctx context.Context, r io.Reader, opts ...MeshImportOption) (*MeshBody, error) {
	if ctx == nil || r == nil {
		return nil, fmt.Errorf(`%w: reading an STL file needs a context and a reader`, ErrDegenerate)
	}
	l, err := meshLimits(opts)
	if err != nil {
		return nil, err
	}
	mesh, err := meshparse.STL(ctx, r, l)
	if err != nil {
		return nil, err
	}
	return NewMeshBody(ctx, mesh.Vertices, mesh.Triangles)
}

// ReadOBJ reads a Wavefront OBJ file into a [MeshBody]
// (agent-docs/mesh-import-design.md). Only the "v" vertex and "f" face lines
// count. A face index is positive (from 1) or negative (back from the latest
// vertex), and any "/" part after the vertex is ignored. A face of more than
// three vertices is cut into a fan from its first vertex, which is right for a
// convex face and wrong for any other. Coordinates are millimetres. Vertices
// with identical coordinates are welded and nothing else is repaired.
//
// ctx and r MUST NOT be nil, each otherwise ErrDegenerate. Malformed input is
// [ErrMalformedMesh] and input past a limit is [ErrMeshTooLarge]. A canceled
// context returns its error.
func ReadOBJ(ctx context.Context, r io.Reader, opts ...MeshImportOption) (*MeshBody, error) {
	if ctx == nil || r == nil {
		return nil, fmt.Errorf(`%w: reading an OBJ file needs a context and a reader`, ErrDegenerate)
	}
	l, err := meshLimits(opts)
	if err != nil {
		return nil, err
	}
	mesh, err := meshparse.OBJ(ctx, r, l)
	if err != nil {
		return nil, err
	}
	return NewMeshBody(ctx, mesh.Vertices, mesh.Triangles)
}

// ReadThreeMF reads a 3MF package into a [MeshBody]
// (agent-docs/mesh-import-design.md §8). Every build item that is not a support
// is read: its mesh, or the meshes of its components, moved by the transforms on
// the item and its components and scaled from the model's unit to millimetres.
// Vertices with identical coordinates are welded and nothing else is repaired.
// Materials, colours, textures and other extensions are ignored.
//
// A transform must be rigid: a rotation, a translation or a reflection. A
// transform that scales or shears is [ErrUnsupported].
//
// ctx and r MUST NOT be nil, each otherwise ErrDegenerate. A package that is
// not a well-formed 3MF model is [ErrMalformedMesh] and input past a limit,
// including a part that expands past the byte limit, is [ErrMeshTooLarge]. A
// canceled context returns its error.
func ReadThreeMF(ctx context.Context, r io.Reader, opts ...MeshImportOption) (*MeshBody, error) {
	if ctx == nil || r == nil {
		return nil, fmt.Errorf(`%w: reading a 3MF package needs a context and a reader`, ErrDegenerate)
	}
	l, err := meshLimits(opts)
	if err != nil {
		return nil, err
	}
	mesh, err := meshparse.ThreeMF(ctx, r, l)
	if err != nil {
		return nil, err
	}
	return NewMeshBody(ctx, mesh.Vertices, mesh.Triangles)
}

// ReadIGES reads the planar-faced solids of an IGES file into a [MeshBody]
// (agent-docs/iges-import-design.md). It reads every manifold solid B-rep entity (186) with the shells (514),
// faces (510), loops (508), edge lists (504) and vertex lists (502) below it: each face on a plane
// (108, 190) is cut into triangles, a face with holes included, the vertices are welded by coordinate and
// each solid is wound outward, a void's shell into its cavity. Lengths are converted from the file's unit to
// millimetres.
//
// Only planes with straight line edges (110) are read. A face on any other surface, a curved edge, a vertex
// entry in a loop, an entity placed by a transformation matrix, a scaled model space, and a file that holds
// only surfaces and curves with no B-rep, are [ErrUnsupported]. A file that is not well-formed IGES, or whose
// entities do not make the B-rep they name (a loop that does not close, a shell whose faces are not
// two-sided), is [ErrMalformed], and input past a limit is [ErrMeshTooLarge]. The mesh is only the file's
// own faces: nothing is repaired, so [SolidFromMesh] decides whether it is a solid.
//
// ctx and r MUST NOT be nil, each otherwise ErrDegenerate. A canceled context returns its error.
func ReadIGES(ctx context.Context, r io.Reader, opts ...MeshImportOption) (*MeshBody, error) {
	if ctx == nil || r == nil {
		return nil, fmt.Errorf(`%w: reading an IGES file needs a context and a reader`, ErrDegenerate)
	}
	l, err := meshLimits(opts)
	if err != nil {
		return nil, err
	}
	data, err := io.ReadAll(io.LimitReader(r, l.Bytes+1))
	if err != nil {
		return nil, fmt.Errorf("IGES read failed: %w", err)
	}
	if int64(len(data)) > l.Bytes {
		return nil, fmt.Errorf("%w: more than %d bytes", ErrMeshTooLarge, l.Bytes)
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	model, err := igesread.Parse(data)
	if err != nil {
		return nil, fmt.Errorf("%w: %s", ErrMalformedMesh, err)
	}
	mesh, err := igesbrep.Read(model, l.Triangles)
	switch {
	case errors.Is(err, igesbrep.ErrTooLarge):
		return nil, fmt.Errorf("%w: %s", ErrMeshTooLarge, err)
	case errors.Is(err, igesbrep.ErrMalformed), errors.Is(err, igesread.ErrMalformed):
		return nil, fmt.Errorf("%w: %s", ErrMalformedMesh, err)
	case err != nil:
		return nil, err
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	return NewMeshBody(ctx, mesh.Vertices, mesh.Triangles)
}

// ReadSTEP reads the planar-, cylinder- and cone-faced solids of a STEP file into a [MeshBody]
// (agent-docs/step-import-design.md). It reads every MANIFOLD_SOLID_BREP and
// BREP_WITH_VOIDS: each face on a plane is cut into triangles, a face with holes
// included, each wall on a cylinder or a cone is cut into a ladder of triangles, arcs are
// cut into chords within 0.01 mm of their circle, and the vertices are welded
// by coordinate. A cavity's shell winds
// into the cavity, so the volume of a solid with one reads as the material
// alone. Lengths are converted from the file's unit to millimetres.
//
// Only planes, cylinders, cones and sphere caps, with straight and circular edges, are read. A
// face on a torus or free-form surface, a sphere face other than a cap with one circular bound, a cone at its apex, a wall with
// holes or an oblique boundary, or an edge that is an ellipse or a curve, is
// [ErrUnsupported]. A file that is
// not well-formed STEP, or whose entities do not make the B-rep they name, is
// [ErrMalformedMesh], and input past a limit is [ErrMeshTooLarge].
//
// ctx and r MUST NOT be nil, each otherwise ErrDegenerate. A canceled context
// returns its error.
func ReadSTEP(ctx context.Context, r io.Reader, opts ...MeshImportOption) (*MeshBody, error) {
	if ctx == nil || r == nil {
		return nil, fmt.Errorf(`%w: reading a STEP file needs a context and a reader`, ErrDegenerate)
	}
	l, err := meshLimits(opts)
	if err != nil {
		return nil, err
	}
	data, err := io.ReadAll(io.LimitReader(r, l.Bytes+1))
	if err != nil {
		return nil, fmt.Errorf("STEP read failed: %w", err)
	}
	if int64(len(data)) > l.Bytes {
		return nil, fmt.Errorf("%w: more than %d bytes", ErrMeshTooLarge, l.Bytes)
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	model, err := stepread.Parse(data)
	if err != nil {
		return nil, fmt.Errorf("%w: %s", ErrMalformedMesh, err)
	}
	mesh, err := stepbrep.Read(model, l.Triangles)
	switch {
	case errors.Is(err, stepbrep.ErrTooLarge):
		return nil, fmt.Errorf("%w: %s", ErrMeshTooLarge, err)
	case errors.Is(err, stepbrep.ErrMalformed):
		return nil, fmt.Errorf("%w: %s", ErrMalformedMesh, err)
	case err != nil:
		return nil, err
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	return NewMeshBody(ctx, mesh.Vertices, mesh.Triangles)
}
