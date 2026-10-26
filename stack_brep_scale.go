package decad

import (
	"context"
	"fmt"
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
			if !region.AllLines() {
				return facetedPayload{}, fmt.Errorf("%w: a stack section is not line-only", ErrUnsupported)
			}
		}
	}
	for _, boundary := range source.Interfaces {
		for _, exposed := range append(append([]profileRecord(nil), boundary.LowerExposed...), boundary.UpperExposed...) {
			if !exposed.AllLines() {
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
	if b.Kind() != BodySolid || !b.IsSolid() || !source.Xform.IsValid() ||
		source.Stack == nil || source.Stack.Delta != 0 || len(source.Stack.Slabs) == 0 ||
		len(source.LoopBands) != 0 || source.BossShell != nil || source.PocketShell != nil {
		return facetedPayload{}, fmt.Errorf("%w: scale needs an exact line-section stack BRep", ErrUnsupported)
	}
	for _, slab := range source.Stack.Slabs {
		if slab.Z0Delta != 0 || slab.Z1Delta != 0 {
			return facetedPayload{}, fmt.Errorf("%w: a BRep stack level carries displacement", ErrUnsupported)
		}
		for _, region := range slab.Regions {
			if !region.AllLines() {
				return facetedPayload{}, fmt.Errorf("%w: a BRep stack section is not line-only", ErrUnsupported)
			}
		}
	}
	for _, face := range source.Faces {
		if face.Delta != 0 || face.Z0Delta != 0 || face.Z1Delta != 0 || face.Blend != "" {
			return facetedPayload{}, fmt.Errorf("%w: a BRep face has no exact line-section proof", ErrUnsupported)
		}
		if face.Planar() {
			if !face.Region.AllLines() {
				return facetedPayload{}, fmt.Errorf("%w: a BRep face is not line-only", ErrUnsupported)
			}
		} else if _, ok := face.Wall.(lineSeg); !ok {
			return facetedPayload{}, fmt.Errorf("%w: a BRep wall is not line-only", ErrUnsupported)
		}
		for _, split := range append(append([]brepSplit(nil), face.Side0...), face.Side1...) {
			if split.ZDelta != 0 {
				return facetedPayload{}, fmt.Errorf("%w: a BRep wall split carries displacement", ErrUnsupported)
			}
		}
	}
	return certifiedLineStackScaleSource(ctx, b)
}

func certifiedLineStackScaleSource(ctx context.Context, b *Body) (facetedPayload, error) {
	fp, err := certifiedScaleSource(ctx, b, 0.1)
	if err != nil {
		return facetedPayload{}, err
	}
	// A stack/BRep's structural audit proves its analytic boundary; the held
	// facet pairs are audited too.
	if err := auditHeldScaleContact(ctx, fp); err != nil {
		return facetedPayload{}, err
	}
	return fp, nil
}
