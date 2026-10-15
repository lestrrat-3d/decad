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
	if ctx == nil || set == nil || set.Source == nil || proxy == nil ||
		len(set.Tools) < 2 || len(set.Moves) != len(set.Tools) ||
		documentHandle(set.Source.n.Doc).requireLive(set.Source) != nil ||
		!proxy.n.BoundaryOK || !proxy.n.SymDiffOK {
		return nil, fmt.Errorf("%w: copied proxy lacks a live source proof", ErrUnsupported)
	}
	first, ok := set.Source.n.Payload.(facetedPayload)
	if !ok || len(proxy.n.Vertices) != len(first.Verts) ||
		len(proxy.n.VertexBound) != len(proxy.n.Vertices) ||
		len(proxy.n.Triangles) != len(first.Tris) ||
		len(proxy.n.Source) != len(proxy.n.Triangles) ||
		len(first.FaceOf) != len(first.Tris) {
		return nil, fmt.Errorf("%w: copied proxy lost its source facets", ErrUnsupported)
	}
	faces := set.Source.Faces()
	for i, tri := range proxy.n.Triangles {
		if i%256 == 0 {
			if err := ctx.Err(); err != nil {
				return nil, err
			}
		}
		faceIndex := first.FaceOf[i]
		if tri != first.Tris[i] || faceIndex < 0 || faceIndex >= len(faces) ||
			proxy.n.Source[i] != faceNode(faces[faceIndex]) {
			return nil, fmt.Errorf("%w: copied proxy changed source face ownership", ErrUnsupported)
		}
	}
	budget := proofbound.NewWorkBudget(ctx)
	placed := make([]facetproof.PlacedMesh, len(set.Tools))
	placed[0] = facetproof.PlacedMesh{
		Verts: proxy.n.Vertices, Tris: proxy.n.Triangles,
		VertexBound: proxy.n.VertexBound,
		MeshBound:   facetproof.FacetBoundMax(proxy.n.Triangles, proxy.n.VertexBound),
		VolSymDiff:  proxy.n.VolSymDiff, AreaSlack: proxy.n.AreaSlack,
	}
	copies := make([]shellseparation.HeldCopy, len(set.Tools))
	copies[0] = shellseparation.HeldCopy{Verts: proxy.n.Vertices, Tris: proxy.n.Triangles, Linked: true}
	for i := 1; i < len(set.Tools); i++ {
		tool := set.Tools[i]
		if tool == nil || tool.n.Doc != set.Source.n.Doc ||
			documentHandle(set.Source.n.Doc).requireLive(tool) != nil {
			return nil, fmt.Errorf("%w: copied proxy tool %d lost its live placement", ErrUnsupported, i+1)
		}
		part, ok := tool.n.Payload.(facetedPayload)
		if !ok || part.CopyLineage == nil ||
			part.CopyLineage.Source != set.Source ||
			part.CopyLineage.SourceProducer != set.Source.n.Origin.Producer ||
			part.CopyLineage.SourcePlacement != first.Xform ||
			part.CopyLineage.TargetPlacement != part.Xform ||
			part.CopyLineage.Motion != set.Moves[i] ||
			len(part.Tris) != len(proxy.n.Triangles) {
			return nil, fmt.Errorf("%w: copied proxy tool %d changed its source", ErrUnsupported, i+1)
		}
		for j, triangle := range part.Tris {
			want := proxy.n.Triangles[j]
			if set.Moves[i].IsReflection() {
				want[1], want[2] = want[2], want[1]
			}
			if triangle != want {
				return nil, fmt.Errorf("%w: copied proxy tool %d changed its source facets", ErrUnsupported, i+1)
			}
		}
		var err error
		placed[i], err = set.placeProxy(ctx, budget, proxy, i, part.CopyLineage.Exact)
		if err != nil {
			return nil, err
		}
		copies[i] = shellseparation.HeldCopy{
			Verts: placed[i].Verts, Tris: placed[i].Tris, Linked: true,
		}
	}
	held, err := shellseparation.PlacedHeldShells(ctx, copies, set.Moves)
	if err != nil {
		return nil, err
	}
	for _, axis := range []shellseparation.Axis{
		set.Axis, shellseparation.AxisX, shellseparation.AxisY, shellseparation.AxisZ,
	} {
		if shellseparation.Separate(ctx, held, set.Moves, nil,
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
	if record != set.Exact[i] {
		return facetproof.PlacedMesh{}, fmt.Errorf("%w: copied proxy tool %d changed its exact rotation",
			ErrUnsupported, i+1)
	}
	if record != nil {
		linearGap, translationGap, ok := record.Gap(set.Moves[i])
		if !ok {
			return facetproof.PlacedMesh{}, fmt.Errorf("%w: copied proxy tool %d has no finite rotation gap",
				ErrUnsupported, i+1)
		}
		return facetproof.PlaceMeshDenoting(ctx, budget, proxy.n.Vertices, proxy.n.Triangles,
			proxy.n.VertexBound, proxy.n.VolSymDiff, proxy.n.AreaSlack, set.Moves[i],
			linearGap, translationGap)
	}
	linear, err := massmoment.PlacementRotation(set.Moves[i])
	if err != nil {
		return facetproof.PlacedMesh{}, err
	}
	charge, err := massmoment.MapChargeOf(linear)
	if err != nil {
		return facetproof.PlacedMesh{}, err
	}
	return facetproof.PlaceMesh(ctx, budget, proxy.n.Vertices, proxy.n.Triangles,
		proxy.n.VertexBound, proxy.n.VolSymDiff, proxy.n.AreaSlack,
		charge.Stretch, charge.Volume, set.Moves[i])
}
