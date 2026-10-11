package decad

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// tryBevelPatternCombineJoin admits a single original point-loft tooth and
// authenticated rigid copies. It rebuilds the complete pattern against the
// original finite blank, avoiding the accumulating bounds of serial Unions.
func tryBevelPatternCombineJoin(ctx context.Context, d *Document, ref producerID,
	blankBody *Body, tools []*Body) (*Body, bool, error) {
	blank, ok := matchBevelBlankSection(blankBody, false)
	if !ok || len(tools) < 2 {
		return nil, false, nil
	}
	first, ok := tools[0].payload.(facetedPayload)
	if !ok || first.xform != r3.Identity() || first.pointCone == nil {
		return nil, false, nil
	}
	if len(tools) < 3 || len(tools) > 12 {
		return nil, true, fmt.Errorf("%w: straight CombineJoin accepts three to twelve teeth", ErrUnsupported)
	}
	record := first.pointCone
	if record.lower == nil || record.upper == nil || record.base.pointSection == nil ||
		record.base.pointSection.apex != (r3.Vec{}) || record.base.xform != r3.Identity() {
		return nil, true, fmt.Errorf("%w: straight tooth trim source is incomplete", ErrUnsupported)
	}
	teeth := make([]facetedPayload, 0, len(tools))
	teeth = append(teeth, first)
	for i, tool := range tools[1:] {
		part, ok := tool.payload.(facetedPayload)
		if !ok || part.bevelTooth == nil || part.bevelTooth.source != record ||
			part.xform != part.bevelTooth.motion || !spiralNearAxisRotation(part.xform) {
			return nil, true, fmt.Errorf("%w: tool %d is not a recorded axis rotation of the first tooth",
				ErrUnsupported, i+2)
		}
		teeth = append(teeth, part)
	}
	separate, err := bevelPatternSourcesSeparate(ctx, tools)
	if err != nil {
		return nil, true, err
	}
	if !separate {
		return nil, true, fmt.Errorf("%w: straight tooth source sectors are not proven disjoint", ErrUnsupported)
	}
	boundary, ok := certifyBevelToothBoundary(record.base.pointSection, blank, record.base)
	if !ok {
		return nil, true, fmt.Errorf("%w: straight tooth source boundary is not certified", ErrUnsupported)
	}
	body, err := buildBevelPatternJoin(ctx, d, ref, blankBody, blank, record, boundary, teeth, true)
	return body, true, err
}

// bevelPatternSourcesSeparate finds one exact YZ projection that separates
// each pair's complete source solids. Each interval expands the held vertex
// projection by its authenticated two-sided spatial mesh bound.
func bevelPatternSourcesSeparate(ctx context.Context, tools []*Body) (bool, error) {
	for i, first := range tools {
		if err := ctx.Err(); err != nil {
			return false, err
		}
		pi, ok := first.payload.(facetedPayload)
		if !ok {
			return false, nil
		}
		ci := pi.xform.ApplyDir(r3.Vec{Y: 1})
		if !proofbound.FiniteVec(ci) {
			return false, nil
		}
		for j := i + 1; j < len(tools); j++ {
			if err := ctx.Err(); err != nil {
				return false, err
			}
			pj, ok := tools[j].payload.(facetedPayload)
			if !ok {
				return false, nil
			}
			cj := pj.xform.ApplyDir(r3.Vec{Y: 1})
			if !proofbound.FiniteVec(cj) {
				return false, nil
			}
			ny, nz := lfSub(lfF(ci.Y), lfF(cj.Y)), lfSub(lfF(ci.Z), lfF(cj.Z))
			if ny.Sign() == 0 && nz.Sign() == 0 {
				return false, nil
			}
			norm := proofbound.RatSqrtUp(lfAdd(lfMul(ny, ny), lfMul(nz, nz)))
			if proofbound.IsNonFinite(norm) || norm <= 0 {
				return false, nil
			}
			loI, hiI, okI := facetedToolProjection(first, ny, nz, norm)
			loJ, hiJ, okJ := facetedToolProjection(tools[j], ny, nz, norm)
			if !okI || !okJ || (hiI.Cmp(loJ) >= 0 && hiJ.Cmp(loI) >= 0) {
				return false, nil
			}
		}
	}
	return true, nil
}
