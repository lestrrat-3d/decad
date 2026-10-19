package decad

import (
	"context"
	"errors"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/facetproof"
	"github.com/lestrrat-3d/decad/internal/massmoment"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/spiralsector"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
)

// spiralPrepareCombinedUnion transports the first join's certified temporary
// cap mesh to every retained copy. The source-to-proxy volume and boundary
// charges are summed before the ordinary mesh Boolean can consume them.
func spiralPrepareCombinedUnion(ctx context.Context, op meshbool.OperationKind,
	blank, combined *Body, original *Mesh, pairTol float64) (*Mesh, *spiralCombineSource, error) {
	p, ok := combined.n.Payload.(facetedPayload)
	if !ok || p.SpiralTools == nil || op != meshbool.OpUnion {
		return original, nil, nil
	}
	meta := p.SpiralTools.Source
	if meta == nil || len(meta.Tools) < 2 || len(meta.Tools) != len(meta.Moves) ||
		meta.First == nil || meta.First.Tooth == nil {
		return nil, nil, fmt.Errorf("%w: combined spiral source is incomplete", ErrUnsupported)
	}
	first := bodyHandle(meta.Tools[0])
	firstMesh, _, err := tessellateBooleanOperand(ctx, first, pairTol)
	if err != nil {
		return nil, nil, err
	}
	faceID := make(map[*topo.Face]int)
	for i, face := range first.Faces() {
		faceID[faceNode(face)] = len(blank.Faces()) + i
	}
	src, err := meshbool.SourceIDs(ctx, firstMesh.n.Source, faceID)
	if err != nil {
		return nil, nil, err
	}
	proxy, certified, err := spiralsector.OffsetToothMesh(ctx, bodyNode(blank), meta.Tools[0],
		&firstMesh.n, src, meta.First)
	if err != nil || !certified {
		return nil, nil, err
	}
	placedParts, err := placedProxyShells(ctx, meta.Copies, bornMesh(&Mesh{n: *proxy}))
	if err != nil {
		return nil, nil, err
	}
	next, err := spiralCombinedProxyMesh(original, placedParts)
	if err != nil {
		return nil, nil, err
	}
	if err := massmoment.AuditMesh(ctx, next.n.Vertices, next.n.Triangles, true); err != nil {
		return nil, nil, err
	}
	return next, meta, nil
}

// spiralCombinedProxyMesh copies the combined operand's mesh and gives its
// restated face bounds a private map before writing any entries.
func spiralCombinedProxyMesh(original *Mesh, placedParts []facetproof.PlacedMesh) (*Mesh, error) {
	next := bornMesh(&Mesh{n: original.n})
	next.n.Vertices = make([]r3.Vec, 0, len(original.n.Vertices))
	next.n.VertexBound = make([]float64, 0, len(original.n.Vertices))
	next.n.VolSymDiff, next.n.AreaSlack = 0, 0
	for _, placed := range placedParts {
		next.n.Vertices = append(next.n.Vertices, placed.Verts...)
		next.n.VertexBound = append(next.n.VertexBound, placed.VertexBound...)
		next.n.VolSymDiff = proofbound.AbsSumUpper(next.n.VolSymDiff, placed.VolSymDiff)
		next.n.AreaSlack = proofbound.AbsSumUpper(next.n.AreaSlack, placed.AreaSlack)
	}
	if len(next.n.Vertices) != len(original.n.Vertices) || len(next.n.VertexBound) != len(next.n.Vertices) ||
		proofbound.IsNonFinite(next.n.VolSymDiff) || proofbound.IsNonFinite(next.n.AreaSlack) {
		return nil, fmt.Errorf("%w: combined spiral proxy has no complete bound", ErrUnsupported)
	}
	next.n.FaceBound = nil
	next.n.Bound = 0
	next.setVertexBounds(next.n.VertexBound)
	next.n.CoordBound = max(next.n.CoordBound, next.n.Bound)
	return next, nil
}

func spiralCombineBody(ctx context.Context, d *Document, ref producerID,
	blank *Body, tools []*Body) (*Body, error) {
	source, err := spiralCombineInputs(ctx, blank, tools)
	if err != nil {
		return nil, err
	}
	combined, err := spiralCombinedToolBody(ctx, d, ref, source)
	if err != nil {
		return nil, err
	}
	eval, err := evaluateBoolean(ctx, meshbool.OpUnion, blank, combined)
	if err != nil {
		return nil, asBooleanError(meshbool.OpUnion, err)
	}
	return bodyResult(facetedrecord.BuildWithProof(ctx, envOf(documentNode(d), ref), eval.payload, eval.audit, measurementToInternal(eval.volume), eval.volumeRat))
}

// spiralCombinedToolBody builds the transient, unregistered disjoint union of
// all input tools that spiralsector.CombinedToolPayload assembles, then binds
// each composite face to the live tool face it copies.
func spiralCombinedToolBody(ctx context.Context, d *Document, ref producerID,
	source *spiralCombineSource) (*Body, error) {
	p, err := spiralsector.CombinedToolPayload(ctx, source)
	if err != nil {
		return nil, err
	}
	body, err := bodyResult(facetedrecord.Build(ctx, envOf(documentNode(d), ref), p))
	if err != nil {
		return nil, err
	}
	if err := spiralsector.BindCombinedOwners(bodyNode(body), p.SpiralTools); err != nil {
		return nil, err
	}
	return body, nil
}

// errSpiralCopyNotExact is the entrance refusal for a copied tooth that carries
// no exact rotation about the X axis through the origin. Its text appears in
// no later refusal, so a caller can tell the entrance gate from a failed proof.
var errSpiralCopyNotExact = errors.New("carries no exact X-axis rotation")

// spiralCombineInputs checks the first source's curved join, then consumes
// the neutral copy-lineage and shell-separation proof. Its X-axis gate belongs
// to the legacy inverse-seam proof, not the copy lineage.
func spiralCombineInputs(ctx context.Context, blank *Body, tools []*Body) (*spiralCombineSource, error) {
	if len(tools) < 2 || len(tools) > 31 {
		return nil, fmt.Errorf("%w: this spiral-tooth Union class accepts two to 31 tools", ErrUnsupported)
	}
	first, ok, err := spiralsector.CertifyFirstJoin(ctx, revolveRootCandidates(blank), bodyNode(tools[0]))
	if err != nil {
		return nil, err
	}
	if !ok {
		return nil, fmt.Errorf("%w: target and first tool have no spiral source certificate", ErrUnsupported)
	}
	copies, err := certifyFacetedCopies(ctx, tools)
	if err != nil {
		return nil, err
	}
	// The blank is invariant under an exact rotation about X, so the first
	// tooth's source facts hold for the copy. This includes a tangent cap:
	// rotating its plane preserves its exact contact with the heel cone and
	// every strict source gap. The copy proof above checks complete-tooth
	// separation. A float placement matrix is not orthogonal and would scale
	// the blank, so every copy must carry the exact record.
	for i, rec := range copies.Exact[1:] {
		if _, ok := spiralsector.AxisCopyMap(rec); !ok {
			return nil, fmt.Errorf("%w: copied tool %d %w; copy it with WithCopyRotation "+
				"about the X axis through the origin", ErrUnsupported, i+2, errSpiralCopyNotExact)
		}
	}
	for i, move := range copies.Moves[1:] {
		if !spiralsector.NearAxisRotation(move) {
			return nil, fmt.Errorf("%w: tool %d cannot use the legacy axial seam proof", ErrUnsupported, i+2)
		}
	}
	return &spiralCombineSource{First: first, Tools: bodyNodes(tools),
		Moves: copies.Moves, Copies: copies}, nil
}
