package decad

import (
	"context"
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/facetproof"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// prismScaleSource turns an authenticated line/circular prism into the
// certified faceted source consumed by ScaleMesh. The prism tessellator gives
// its shared-edge topology, source-face map and occupied-volume proof. A
// curved face becomes Faceted because this path has no scaled normal proof.
func prismScaleSource(ctx context.Context, b *Body, prism prismPayload, factor float64) (facetedPayload, error) {
	curved, eligible := prismScaleProfile(prism.profile)
	if prism.surfaceResult || prism.sectionDelta != 0 || prism.z0Delta != 0 || prism.z1Delta != 0 ||
		!prism.xform.IsValid() || !eligible {
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
	mesh, err := tessellateContext(ctx, b, units.Millimeters(chord), VerifyAll)
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
		switch surface := face.Surface().(type) {
		case Plane:
			groups[i] = facetGroup{origins: face.Origins(), surface: surface, planar: true}
		case Cylinder:
			// The source mesh supplies the curved-face displacement. Its group
			// keeps origins but publishes Faceted after the similarity.
			groups[i] = facetGroup{origins: face.Origins()}
		default:
			return facetedPayload{}, fmt.Errorf("%w: the prism has an unproved scaled surface", ErrUnsupported)
		}
		faceID[face] = i
	}
	source, err := meshbool.SourceIDs(ctx, mesh.source, faceID)
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
		prismScale:     true,
		contactAudited: true,
		verts:          mesh.vertices,
		vertexBound:    vertexBound,
		tris:           mesh.triangles,
		src:            source,
		groups:         groups,
		meshBound:      facetproof.FacetBoundMax(mesh.triangles, vertexBound),
		volSymDiff:     mesh.volSymDiff,
		areaSlack:      mesh.areaSlack,
		dPair:          diameter,
		xform:          r3.Identity(),
	}, nil
}

// prismScaleProfile admits only complete source edges. A partial Sketch edge
// needs its own source certificate even if its visible curve is circular.
func prismScaleProfile(profile profileRecord) (curved, eligible bool) {
	loops := append([]loopRecord{profile.Outer}, profile.Holes...)
	for _, loop := range loops {
		if len(loop.Segments) == 0 {
			return false, false
		}
		for _, segment := range loop.Segments {
			var start, end float64
			switch s := segment.(type) {
			case lineSeg:
				start, end = s.TStart, s.TEnd
			case circleSeg:
				if len(loop.Segments) != 1 {
					return false, false
				}
				start, end, curved = s.TStart, s.TEnd, true
			case arcSeg:
				start, end, curved = s.TStart, s.TEnd, true
			default:
				return false, false
			}
			if !((start == 0 && end == 1) || (start == 1 && end == 0)) {
				return false, false
			}
		}
	}
	return curved, true
}
