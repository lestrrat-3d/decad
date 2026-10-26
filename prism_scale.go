package decad

import (
	"context"
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/facetproof"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/prismcells"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/revolvemesh"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// prismScaleSource turns an authenticated line/circular prism into the
// certified faceted source consumed by ScaleMesh. The prism tessellator gives
// its shared-edge topology, source-face map and occupied-volume proof. A
// curved face becomes Faceted because this path has no scaled normal proof.
func prismScaleSource(ctx context.Context, b *Body, prism prismPayload, factor float64) (facetedPayload, error) {
	curved, eligible := prismcells.ScaleSourceProfile(prism.Profile)
	if prism.SurfaceResult || prism.SectionDelta != 0 || prism.Z0Delta != 0 || prism.Z1Delta != 0 ||
		!prism.Xform.IsValid() || !eligible {
		return facetedPayload{}, fmt.Errorf("%w: scale needs an exact line/circular solid prism", ErrUnsupported)
	}
	chord := 0.1
	if curved {
		// The source's curved-face displacement grows by factor. Reserve half
		// of the 0.1 mm target for its chord and half for construction rounding.
		// A subsequent similarity cannot refine this first held mesh.
		chord = math.Nextafter(0.05/math.Max(1, factor), 0)
		if chord <= 0 {
			return facetedPayload{}, fmt.Errorf("%w: the scale leaves no representable curved-source chord", ErrUnsupported)
		}
	}
	return certifiedScaleSource(ctx, b, chord)
}

// revolveScaleSource admits a solid Revolve, complete or partial, only after
// its own tessellation proves both the boundary and occupied volume and an
// its own contact audit covers the held mesh. The similarity
// then carries those proofs through the same faceted path as a prism.
func revolveScaleSource(ctx context.Context, b *Body, source revolvePayload, factor float64) (facetedPayload, error) {
	if source.SurfaceResult || !source.Xform.IsValid() || source.SectionDelta != 0 {
		return facetedPayload{}, fmt.Errorf("%w: scale needs an exact solid Revolve", ErrUnsupported)
	}
	chord := math.Nextafter(0.05/math.Max(1, factor), 0)
	if chord <= 0 {
		return facetedPayload{}, fmt.Errorf("%w: the scale leaves no representable Revolve chord", ErrUnsupported)
	}
	// The source's VerifyAll tessellation already ran the facet-pair contact
	// audit on the held mesh, so no second audit is added here.
	return certifiedScaleSource(ctx, b, chord)
}

// auditHeldScaleContact checks every held facet pair of a scale source for
// contact, so rounded tessellation coordinates cannot introduce an unproven
// contact before ScaleMesh moves them. It can only refuse.
func auditHeldScaleContact(ctx context.Context, fp facetedPayload) error {
	budget := proofbound.NewWorkBudget(ctx)
	data, err := revolvemesh.RequireRevolveFacetAreas(budget, fp.Verts, fp.Tris, 0)
	if err != nil {
		return err
	}
	return revolvemesh.RevolveContactAudit(budget, data, fp.Tris, 0)
}

// certifiedScaleSource transfers an authenticated source mesh to the
// similarity evaluator. Curved faces become Faceted; planar source tags and
// all face origins remain attached to their source facet groups.
func certifiedScaleSource(ctx context.Context, b *Body, chord float64) (facetedPayload, error) {
	mesh, err := tessellateContext(ctx, b, units.Millimeters(chord), VerifyAll)
	if err != nil {
		return facetedPayload{}, err
	}
	if !mesh.n.BoundaryOK || !mesh.n.SymDiffOK || len(mesh.n.Vertices) == 0 || len(mesh.n.Triangles) == 0 {
		return facetedPayload{}, fmt.Errorf("%w: the source mesh has no complete similarity proof", ErrUnsupported)
	}
	vertexBound, err := topobuild.VertexBounds(&mesh.n)
	if err != nil {
		return facetedPayload{}, err
	}
	faces := b.Faces()
	groups := make([]facetGroup, len(faces))
	faceID := make(map[*topo.Face]int, len(faces))
	for i, face := range faces {
		if err := ctx.Err(); err != nil {
			return facetedPayload{}, err
		}
		switch surface := face.n.Surface.(type) {
		case surfacegeom.Plane:
			groups[i] = facetGroup{Origins: face.originRefs(), Surface: surface, Planar: true}
		default:
			// The source mesh supplies every curved-face displacement. The
			// image keeps origins and publishes Faceted without a normal tag.
			groups[i] = facetGroup{Origins: face.originRefs()}
		}
		faceID[faceNode(face)] = i
	}
	source, err := meshbool.SourceIDs(ctx, mesh.n.Source, faceID)
	if err != nil {
		return facetedPayload{}, err
	}
	lo, hi := mesh.n.Vertices[0], mesh.n.Vertices[0]
	for _, vertex := range mesh.n.Vertices[1:] {
		lo = r3.NewVec(math.Min(lo.X, vertex.X), math.Min(lo.Y, vertex.Y), math.Min(lo.Z, vertex.Z))
		hi = r3.NewVec(math.Max(hi.X, vertex.X), math.Max(hi.Y, vertex.Y), math.Max(hi.Z, vertex.Z))
	}
	// The held box diagonal bounds every held pair. Each true boundary point
	// is within mesh.bound of the held boundary, so two such points add twice
	// that displacement to the centroid allowance's diameter.
	diameter := proofbound.AbsSumUpper(proofbound.DvLenUpper(proofbound.HeldDelta(hi, lo)),
		proofbound.ProductUpper(2, mesh.n.Bound))
	if proofbound.IsNonFinite(diameter) {
		return facetedPayload{}, fmt.Errorf("%w: the source has no finite diameter bound", ErrUnsupported)
	}
	return facetedPayload{
		SimilaritySource: true,
		ContactAudited:   true,
		Verts:            mesh.n.Vertices,
		VertexBound:      vertexBound,
		Tris:             mesh.n.Triangles,
		Src:              source,
		Groups:           groups,
		MeshBound:        facetproof.FacetBoundMax(mesh.n.Triangles, vertexBound),
		VolSymDiff:       mesh.n.VolSymDiff,
		AreaSlack:        mesh.n.AreaSlack,
		DPair:            diameter,
		Xform:            r3.Identity(),
	}, nil
}
