package decad

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/revolvemesh"
)

// stackedScaleSource converts only exact line-section stacks. The stack's
// own tessellator proves the recorded sections and their occupied volume;
// the additional facet audit proves the held mesh is embedded before its
// positive similarity reuses that mesh.
func stackedScaleSource(ctx context.Context, b *Body, source stackedPrismPayload) (facetedPayload, error) {
	if b.Kind() != BodySolid || !b.IsSolid() || !source.Xform.IsValid() ||
		source.SectionDelta != 0 || len(source.Slabs) == 0 {
		return facetedPayload{}, fmt.Errorf("%w: scale needs an exact line-section solid stack", ErrUnsupported)
	}
	for _, slab := range source.Slabs {
		if slab.Z0Delta != 0 || slab.Z1Delta != 0 {
			return facetedPayload{}, fmt.Errorf("%w: a stack level carries displacement", ErrUnsupported)
		}
		for _, region := range slab.Regions {
			if !scaleLineProfile(region) {
				return facetedPayload{}, fmt.Errorf("%w: a stack section is not line-only", ErrUnsupported)
			}
		}
	}
	for _, boundary := range source.Interfaces {
		for _, exposed := range append(append([]profileRecord(nil), boundary.LowerExposed...), boundary.UpperExposed...) {
			if !scaleLineProfile(exposed) {
				return facetedPayload{}, fmt.Errorf("%w: a stack interface is not line-only", ErrUnsupported)
			}
		}
	}
	return certifiedLineStackScaleSource(ctx, b)
}

// brepScaleSource takes only A1's recorded line-only stack with unmodified
// planar faces. A transformed source is accepted through its own certified
// mesh, which already includes placement error in each face bound.
func brepScaleSource(ctx context.Context, b *Body, source brepPayload) (facetedPayload, error) {
	if b.Kind() != BodySolid || !b.IsSolid() || !source.xform.IsValid() ||
		source.stack == nil || source.stack.delta != 0 || len(source.stack.slabs) == 0 ||
		len(source.loopBands) != 0 || source.bossShell != nil || source.pocketShell != nil {
		return facetedPayload{}, fmt.Errorf("%w: scale needs an exact line-section stack BRep", ErrUnsupported)
	}
	for _, slab := range source.stack.slabs {
		if slab.Z0Delta != 0 || slab.Z1Delta != 0 {
			return facetedPayload{}, fmt.Errorf("%w: a BRep stack level carries displacement", ErrUnsupported)
		}
		for _, region := range slab.Regions {
			if !scaleLineProfile(region) {
				return facetedPayload{}, fmt.Errorf("%w: a BRep stack section is not line-only", ErrUnsupported)
			}
		}
	}
	for _, face := range source.faces {
		if face.delta != 0 || face.z0Delta != 0 || face.z1Delta != 0 || face.blend != "" {
			return facetedPayload{}, fmt.Errorf("%w: a BRep face has no exact line-section proof", ErrUnsupported)
		}
		if face.planar() {
			if !scaleLineProfile(*face.region) {
				return facetedPayload{}, fmt.Errorf("%w: a BRep face is not line-only", ErrUnsupported)
			}
		} else if _, ok := face.wall.(lineSeg); !ok {
			return facetedPayload{}, fmt.Errorf("%w: a BRep wall is not line-only", ErrUnsupported)
		}
		for _, split := range append(append([]brepSplit(nil), face.side0...), face.side1...) {
			if split.ZDelta != 0 {
				return facetedPayload{}, fmt.Errorf("%w: a BRep wall split carries displacement", ErrUnsupported)
			}
		}
	}
	return certifiedLineStackScaleSource(ctx, b)
}

func scaleLineProfile(profile profileRecord) bool {
	for _, loop := range append([]loopRecord{profile.Outer}, profile.Holes...) {
		if len(loop.Segments) == 0 {
			return false
		}
		for _, segment := range loop.Segments {
			if _, ok := segment.(lineSeg); !ok {
				return false
			}
		}
	}
	return true
}

func certifiedLineStackScaleSource(ctx context.Context, b *Body) (facetedPayload, error) {
	fp, err := certifiedScaleSource(ctx, b, 0.1)
	if err != nil {
		return facetedPayload{}, err
	}
	// A stack/BRep's structural audit proves its analytic boundary. Check
	// every held facet pair as well, so rounded tessellation coordinates
	// cannot introduce an unproven contact before ScaleMesh moves them.
	budget := proofbound.NewWorkBudget(ctx)
	data, err := revolvemesh.RequireRevolveFacetAreas(budget, fp.verts, fp.tris, 0)
	if err != nil {
		return facetedPayload{}, err
	}
	if err := revolvemesh.RevolveContactAudit(budget, data, fp.tris, 0); err != nil {
		return facetedPayload{}, err
	}
	return fp, nil
}
