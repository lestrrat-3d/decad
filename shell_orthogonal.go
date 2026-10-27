package decad

import (
	"context"
	"errors"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/brepmodify"
	"github.com/lestrrat-3d/decad/internal/orthoshell"
	"github.com/lestrrat-3d/decad/internal/topo"
)

// shellOrthogonal shells a faceted solid every one of whose faces squares to a world axis, such as a
// mesh solid made by SolidFromMesh or a mesh Boolean of axis-aligned boxes
// (agent-docs/orthogonal-shell-design.md). The cavity is the solid eroded by a cube of side 2t,
// computed on a grid of the solid's coordinates and those coordinates ± t, with the slab twice t
// thick over each removed face joined to the solid first so the cavity opens through it; the
// result is the body cut by that cavity, made a mesh solid. ok is false when the route does not
// apply, and the caller takes the receiver's own route: the receiver is not a faceted body, it
// holds a vertex with a bound, the sense is Outward, or a triangle is not square to an axis.
func shellOrthogonal(ctx context.Context, d *Document, b *Body, call brepmodify.ShellCall) (result *Body, ok bool, err error) {
	payload, isFaceted := b.n.Payload.(facetedPayload)
	if !isFaceted || !call.Inward || len(payload.Tris) == 0 {
		return nil, false, nil
	}
	for _, beta := range payload.VertexBound {
		if beta != 0 {
			return nil, false, nil
		}
	}
	removed := make([]bool, len(payload.Tris))
	if !call.NoOpenings {
		gone := map[*topo.Face]bool{}
		for _, f := range call.Removed {
			gone[f] = true
		}
		var faces []*topo.Face
		for _, l := range b.n.Lumps {
			for _, sh := range l.Shells {
				faces = append(faces, sh.Faces...)
			}
		}
		for i, f := range payload.FaceOf {
			if f >= 0 && f < len(faces) && gone[faces[f]] {
				removed[i] = true
			}
		}
	}
	return shellOrthogonalOnce(ctx, d, b, payload, removed, call.Tmm)
}

// shellOrthogonalOnce builds the walls of thickness t as a mesh and makes them the body that replaces b.
func shellOrthogonalOnce(ctx context.Context, d *Document, b *Body, payload facetedPayload, removed []bool, t float64) (*Body, bool, error) {
	wallVerts, wallTris, err := orthoshell.Walls(payload.Verts, payload.Tris, removed, t)
	switch {
	case errors.Is(err, orthoshell.ErrNotOrthogonal):
		return nil, false, nil
	case errors.Is(err, orthoshell.ErrDegenerate):
		return nil, true, fmt.Errorf("%w: %s", ErrDegenerate, err)
	case errors.Is(err, orthoshell.ErrUnsupported):
		return nil, true, fmt.Errorf("%w: %s", ErrUnsupported, err)
	case err != nil:
		return nil, true, err
	}
	mesh, err := NewMeshBody(ctx, wallVerts, wallTris)
	if err != nil {
		return nil, true, err
	}
	walls, err := solidFromMeshBody(ctx, d, mesh)
	if err != nil {
		return nil, true, fmt.Errorf("%w: the shell's walls are not a solid the evaluator can prove: %s", ErrUnsupported, err)
	}
	out, err := commitModifyResult(ctx, OperationShell, b, walls)
	return out, true, err
}
