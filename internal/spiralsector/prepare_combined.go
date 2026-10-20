package spiralsector

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/facetproof"
	"github.com/lestrrat-3d/decad/internal/massmoment"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/r3"
)

// OperandMesher meshes one Boolean operand at its own request tolerance, raised
// to a restating operand's held floor, and reports whether the operand
// restates a held mesh. The mesh carries every proof a VerifyAll tessellation
// can.
type OperandMesher interface {
	TessellateOperand(ctx context.Context, b *topo.Body, tolMM float64) (*topo.Mesh, bool, error)
}

// PrepareCombinedUnion transports the first join's certified temporary
// cap mesh to every retained copy. The source-to-proxy volume and boundary
// charges are summed before the ordinary mesh Boolean can consume them.
func PrepareCombinedUnion(ctx context.Context, op meshbool.OperationKind, mesher OperandMesher,
	blank, combined *topo.Body, original *topo.Mesh, pairTol float64) (*topo.Mesh, *facetedrecord.SpiralCombineSource, error) {
	p, ok := combined.Payload.(facetedrecord.Payload)
	if !ok || p.SpiralTools == nil || op != meshbool.OpUnion {
		return original, nil, nil
	}
	meta := p.SpiralTools.Source
	if meta == nil || len(meta.Tools) < 2 || len(meta.Tools) != len(meta.Moves) ||
		meta.First == nil || meta.First.Tooth == nil {
		return nil, nil, fmt.Errorf("%w: combined spiral source is incomplete", decaderr.ErrUnsupported)
	}
	first := meta.Tools[0]
	firstMesh, _, err := mesher.TessellateOperand(ctx, first, pairTol)
	if err != nil {
		return nil, nil, err
	}
	faceID := make(map[*topo.Face]int)
	for i, face := range topobuild.Faces(first) {
		faceID[face] = len(topobuild.Faces(blank)) + i
	}
	src, err := meshbool.SourceIDs(ctx, firstMesh.Source, faceID)
	if err != nil {
		return nil, nil, err
	}
	proxy, certified, err := OffsetToothMesh(ctx, blank, meta.Tools[0], firstMesh, src, meta.First)
	if err != nil || !certified {
		return nil, nil, err
	}
	placedParts, err := facetedrecord.PlacedProxyShells(ctx, meta.Copies, proxy)
	if err != nil {
		return nil, nil, err
	}
	next, err := CombinedProxyMesh(original, placedParts)
	if err != nil {
		return nil, nil, err
	}
	if err := massmoment.AuditMesh(ctx, next.Vertices, next.Triangles, true); err != nil {
		return nil, nil, err
	}
	return next, meta, nil
}

// CombinedProxyMesh copies the combined operand's mesh and gives its
// restated face bounds a private map before writing any entries.
func CombinedProxyMesh(original *topo.Mesh, placedParts []facetproof.PlacedMesh) (*topo.Mesh, error) {
	next := new(topo.Mesh)
	*next = *original
	next.Vertices = make([]r3.Vec, 0, len(original.Vertices))
	next.VertexBound = make([]float64, 0, len(original.Vertices))
	next.VolSymDiff, next.AreaSlack = 0, 0
	for _, placed := range placedParts {
		next.Vertices = append(next.Vertices, placed.Verts...)
		next.VertexBound = append(next.VertexBound, placed.VertexBound...)
		next.VolSymDiff = proofbound.AbsSumUpper(next.VolSymDiff, placed.VolSymDiff)
		next.AreaSlack = proofbound.AbsSumUpper(next.AreaSlack, placed.AreaSlack)
	}
	if len(next.Vertices) != len(original.Vertices) || len(next.VertexBound) != len(next.Vertices) ||
		proofbound.IsNonFinite(next.VolSymDiff) || proofbound.IsNonFinite(next.AreaSlack) {
		return nil, fmt.Errorf("%w: combined spiral proxy has no complete bound", decaderr.ErrUnsupported)
	}
	next.FaceBound = nil
	next.Bound = 0
	topobuild.SetVertexBounds(next, next.VertexBound)
	next.CoordBound = max(next.CoordBound, next.Bound)
	return next, nil
}
