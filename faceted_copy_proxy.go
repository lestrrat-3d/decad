package decad

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/exactrotation"
	"github.com/lestrrat-3d/decad/internal/facetproof"
	"github.com/lestrrat-3d/decad/internal/massmoment"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/shellseparation"
)

// placedProxyShells places one source-certified proxy under every recorded
// copy motion and proves its held shells separate. The caller owns the proof
// that proxy's true boundary and volume represent the original source solid.
// This method checks the live source face of every proxy triangle; a mesh
// carrying unrelated faces cannot borrow the copy lineage.
func (set *facetedCopySet) placedProxyShells(ctx context.Context,
	proxy *Mesh) ([]facetproof.PlacedMesh, error) {
	if ctx == nil || set == nil || set.source == nil || proxy == nil ||
		len(set.tools) < 2 || len(set.moves) != len(set.tools) ||
		documentHandle(set.source.n.Doc).requireLive(set.source) != nil ||
		!proxy.n.BoundaryOK || !proxy.n.SymDiffOK {
		return nil, fmt.Errorf("%w: copied proxy lacks a live source proof", ErrUnsupported)
	}
	first, ok := set.source.n.Payload.(facetedPayload)
	if !ok || len(proxy.n.Vertices) != len(first.verts) ||
		len(proxy.n.VertexBound) != len(proxy.n.Vertices) ||
		len(proxy.n.Triangles) != len(first.tris) ||
		len(proxy.n.Source) != len(proxy.n.Triangles) ||
		len(first.faceOf) != len(first.tris) {
		return nil, fmt.Errorf("%w: copied proxy lost its source facets", ErrUnsupported)
	}
	faces := set.source.Faces()
	for i, tri := range proxy.n.Triangles {
		if i%256 == 0 {
			if err := ctx.Err(); err != nil {
				return nil, err
			}
		}
		faceIndex := first.faceOf[i]
		if tri != first.tris[i] || faceIndex < 0 || faceIndex >= len(faces) ||
			proxy.n.Source[i] != faceNode(faces[faceIndex]) {
			return nil, fmt.Errorf("%w: copied proxy changed source face ownership", ErrUnsupported)
		}
	}
	budget := proofbound.NewWorkBudget(ctx)
	placed := make([]facetproof.PlacedMesh, len(set.tools))
	placed[0] = facetproof.PlacedMesh{
		Verts: proxy.n.Vertices, Tris: proxy.n.Triangles,
		VertexBound: proxy.n.VertexBound,
		MeshBound:   facetproof.FacetBoundMax(proxy.n.Triangles, proxy.n.VertexBound),
		VolSymDiff:  proxy.n.VolSymDiff, AreaSlack: proxy.n.AreaSlack,
	}
	copies := make([]shellseparation.HeldCopy, len(set.tools))
	copies[0] = shellseparation.HeldCopy{Verts: proxy.n.Vertices, Tris: proxy.n.Triangles, Linked: true}
	for i := 1; i < len(set.tools); i++ {
		tool := set.tools[i]
		if tool == nil || tool.n.Doc != set.source.n.Doc ||
			documentHandle(set.source.n.Doc).requireLive(tool) != nil {
			return nil, fmt.Errorf("%w: copied proxy tool %d lost its live placement", ErrUnsupported, i+1)
		}
		part, ok := tool.n.Payload.(facetedPayload)
		if !ok || part.copyLineage == nil ||
			part.copyLineage.source != set.source ||
			part.copyLineage.sourceProducer != set.source.n.Origin.Producer ||
			part.copyLineage.sourcePlacement != first.xform ||
			part.copyLineage.targetPlacement != part.xform ||
			part.copyLineage.motion != set.moves[i] ||
			len(part.tris) != len(proxy.n.Triangles) {
			return nil, fmt.Errorf("%w: copied proxy tool %d changed its source", ErrUnsupported, i+1)
		}
		for j, triangle := range part.tris {
			want := proxy.n.Triangles[j]
			if set.moves[i].IsReflection() {
				want[1], want[2] = want[2], want[1]
			}
			if triangle != want {
				return nil, fmt.Errorf("%w: copied proxy tool %d changed its source facets", ErrUnsupported, i+1)
			}
		}
		var err error
		placed[i], err = set.placeProxy(ctx, budget, proxy, i, part.copyLineage.exact)
		if err != nil {
			return nil, err
		}
		copies[i] = shellseparation.HeldCopy{
			Verts: placed[i].Verts, Tris: placed[i].Tris, Linked: true,
		}
	}
	held, err := shellseparation.PlacedHeldShells(ctx, copies, set.moves)
	if err != nil {
		return nil, err
	}
	for _, axis := range []shellseparation.Axis{
		set.axis, shellseparation.AxisX, shellseparation.AxisY, shellseparation.AxisZ,
	} {
		if shellseparation.Separate(ctx, held, set.moves, nil,
			shellseparation.ProxyShell, axis, true) {
			return placed, nil
		}
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	return nil, fmt.Errorf("%w: copied proxy shells are not separate", ErrUnsupported)
}

// placeProxy places the proxy for copy i. An exact copy's true solid is its
// exact rotation of the source's, so the proxy pays the gap between that
// rotation and the float move instead of the float matrix's stretch.
func (set *facetedCopySet) placeProxy(ctx context.Context, budget *proofbound.WorkBudget, proxy *Mesh,
	i int, record *exactrotation.Rotation) (facetproof.PlacedMesh, error) {
	if record != set.exact[i] {
		return facetproof.PlacedMesh{}, fmt.Errorf("%w: copied proxy tool %d changed its exact rotation",
			ErrUnsupported, i+1)
	}
	if record != nil {
		linearGap, translationGap, ok := record.Gap(set.moves[i])
		if !ok {
			return facetproof.PlacedMesh{}, fmt.Errorf("%w: copied proxy tool %d has no finite rotation gap",
				ErrUnsupported, i+1)
		}
		return facetproof.PlaceMeshDenoting(ctx, budget, proxy.n.Vertices, proxy.n.Triangles,
			proxy.n.VertexBound, proxy.n.VolSymDiff, proxy.n.AreaSlack, set.moves[i],
			linearGap, translationGap)
	}
	linear, err := massmoment.PlacementRotation(set.moves[i])
	if err != nil {
		return facetproof.PlacedMesh{}, err
	}
	charge, err := massmoment.MapChargeOf(linear)
	if err != nil {
		return facetproof.PlacedMesh{}, err
	}
	return facetproof.PlaceMesh(ctx, budget, proxy.n.Vertices, proxy.n.Triangles,
		proxy.n.VertexBound, proxy.n.VolSymDiff, proxy.n.AreaSlack,
		charge.Stretch, charge.Volume, set.moves[i])
}
