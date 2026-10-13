package decad

import (
	"context"
	"fmt"

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
		set.source.doc.requireLive(set.source) != nil ||
		!proxy.boundaryOK || !proxy.symDiffOK {
		return nil, fmt.Errorf("%w: copied proxy lacks a live source proof", ErrUnsupported)
	}
	first, ok := set.source.payload.(facetedPayload)
	if !ok || len(proxy.vertices) != len(first.verts) ||
		len(proxy.vertexBound) != len(proxy.vertices) ||
		len(proxy.triangles) != len(first.tris) ||
		len(proxy.source) != len(proxy.triangles) ||
		len(first.faceOf) != len(first.tris) {
		return nil, fmt.Errorf("%w: copied proxy lost its source facets", ErrUnsupported)
	}
	faces := set.source.Faces()
	for i, tri := range proxy.triangles {
		if i%256 == 0 {
			if err := ctx.Err(); err != nil {
				return nil, err
			}
		}
		faceIndex := first.faceOf[i]
		if tri != first.tris[i] || faceIndex < 0 || faceIndex >= len(faces) ||
			proxy.source[i] != faces[faceIndex] {
			return nil, fmt.Errorf("%w: copied proxy changed source face ownership", ErrUnsupported)
		}
	}
	budget := proofbound.NewWorkBudget(ctx)
	placed := make([]facetproof.PlacedMesh, len(set.tools))
	placed[0] = facetproof.PlacedMesh{
		Verts: proxy.vertices, Tris: proxy.triangles,
		VertexBound: proxy.vertexBound,
		MeshBound:   facetproof.FacetBoundMax(proxy.triangles, proxy.vertexBound),
		VolSymDiff:  proxy.volSymDiff, AreaSlack: proxy.areaSlack,
	}
	copies := make([]shellseparation.HeldCopy, len(set.tools))
	copies[0] = shellseparation.HeldCopy{Verts: proxy.vertices, Tris: proxy.triangles, Linked: true}
	for i := 1; i < len(set.tools); i++ {
		tool := set.tools[i]
		if tool == nil || tool.doc != set.source.doc ||
			set.source.doc.requireLive(tool) != nil {
			return nil, fmt.Errorf("%w: copied proxy tool %d lost its live placement", ErrUnsupported, i+1)
		}
		part, ok := tool.payload.(facetedPayload)
		if !ok || part.copyLineage == nil ||
			part.copyLineage.source != set.source ||
			part.copyLineage.sourceProducer != set.source.origin.producer ||
			part.copyLineage.sourcePlacement != first.xform ||
			part.copyLineage.targetPlacement != part.xform ||
			part.copyLineage.motion != set.moves[i] ||
			len(part.tris) != len(proxy.triangles) {
			return nil, fmt.Errorf("%w: copied proxy tool %d changed its source", ErrUnsupported, i+1)
		}
		for j, triangle := range part.tris {
			want := proxy.triangles[j]
			if set.moves[i].IsReflection() {
				want[1], want[2] = want[2], want[1]
			}
			if triangle != want {
				return nil, fmt.Errorf("%w: copied proxy tool %d changed its source facets", ErrUnsupported, i+1)
			}
		}
		linear, err := massmoment.PlacementRotation(set.moves[i])
		if err != nil {
			return nil, err
		}
		charge, err := massmoment.MapChargeOf(linear)
		if err != nil {
			return nil, err
		}
		placed[i], err = facetproof.PlaceMesh(ctx, budget, proxy.vertices, proxy.triangles,
			proxy.vertexBound, proxy.volSymDiff, proxy.areaSlack,
			charge.Stretch, charge.Volume, set.moves[i])
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
		if shellseparation.Separate(ctx, held, set.moves,
			shellseparation.ProxyShell, axis, true) {
			return placed, nil
		}
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	return nil, fmt.Errorf("%w: copied proxy shells are not separate", ErrUnsupported)
}
