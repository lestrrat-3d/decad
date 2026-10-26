package decad

import (
	"context"
	"fmt"
	"io"

	"github.com/lestrrat-3d/decad/internal/meshparse"
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
