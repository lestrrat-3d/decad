package decad

import (
	"context"
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/facetproof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// straightPrismScaleSource turns an authenticated all-line prism into the
// certified faceted source consumed by ScaleMesh. The prism tessellator gives
// its shared-edge topology, source-face map and occupied-volume proof; no
// sampled geometry is promoted to a certificate here.
func straightPrismScaleSource(ctx context.Context, b *Body, prism prismPayload) (facetedPayload, error) {
	if prism.surfaceResult || prism.sectionDelta != 0 || prism.z0Delta != 0 || prism.z1Delta != 0 ||
		!prism.xform.IsValid() || !straightPrismProfile(prism.profile) {
		return facetedPayload{}, fmt.Errorf("%w: scale needs an exact all-line solid prism", ErrUnsupported)
	}
	mesh, err := tessellateContext(ctx, b, units.Millimeters(0.1), VerifyAll)
	if err != nil {
		return facetedPayload{}, err
	}
	if !mesh.boundaryOK || !mesh.symDiffOK || len(mesh.vertices) == 0 || len(mesh.triangles) == 0 {
		return facetedPayload{}, fmt.Errorf("%w: the prism mesh has no complete similarity proof", ErrUnsupported)
	}
	vertexBound, err := mesh.vertexBounds()
	if err != nil {
		return facetedPayload{}, err
	}
	faces := b.Faces()
	groups := make([]facetGroup, len(faces))
	faceID := make(map[*Face]int, len(faces))
	for i, face := range faces {
		if err := ctx.Err(); err != nil {
			return facetedPayload{}, err
		}
		plane, ok := face.Surface().(Plane)
		if !ok {
			return facetedPayload{}, fmt.Errorf("%w: an all-line prism has a nonplanar face", ErrUnsupported)
		}
		groups[i] = facetGroup{origins: face.Origins(), surface: plane, planar: true}
		faceID[face] = i
	}
	source, err := sourceIDs(ctx, mesh, faceID)
	if err != nil {
		return facetedPayload{}, err
	}
	lo, hi := mesh.vertices[0], mesh.vertices[0]
	for _, vertex := range mesh.vertices[1:] {
		lo = r3.NewVec(math.Min(lo.X, vertex.X), math.Min(lo.Y, vertex.Y), math.Min(lo.Z, vertex.Z))
		hi = r3.NewVec(math.Max(hi.X, vertex.X), math.Max(hi.Y, vertex.Y), math.Max(hi.Z, vertex.Z))
	}
	// The held box diagonal bounds every held pair. Each true boundary point
	// is within mesh.bound of the held boundary, so two such points add twice
	// that displacement to the centroid allowance's diameter.
	diameter := proofbound.AbsSumUpper(proofbound.DvLenUpper(proofbound.HeldDelta(hi, lo)),
		proofbound.ProductUpper(2, mesh.bound))
	if proofbound.IsNonFinite(diameter) {
		return facetedPayload{}, fmt.Errorf("%w: the prism has no finite diameter bound", ErrUnsupported)
	}
	return facetedPayload{
		straightPrismScale: true,
		contactAudited:     true,
		verts:              mesh.vertices,
		vertexBound:        vertexBound,
		tris:               mesh.triangles,
		src:                source,
		groups:             groups,
		meshBound:          facetproof.FacetBoundMax(mesh.triangles, vertexBound),
		volSymDiff:         mesh.volSymDiff,
		areaSlack:          mesh.areaSlack,
		dPair:              diameter,
		xform:              r3.Identity(),
	}, nil
}

func straightPrismProfile(profile profileRecord) bool {
	loops := append([]loopRecord{profile.Outer}, profile.Holes...)
	for _, loop := range loops {
		if len(loop.Segments) < 3 {
			return false
		}
		for _, segment := range loop.Segments {
			line, ok := segment.(lineSeg)
			if !ok || !((line.TStart == 0 && line.TEnd == 1) ||
				(line.TStart == 1 && line.TEnd == 0)) {
				return false
			}
		}
	}
	return true
}
