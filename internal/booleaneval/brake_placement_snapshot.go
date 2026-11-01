package booleaneval

import (
	"context"
	"unsafe"

	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
)

type brakePlacementSource struct {
	ptr   *facetedrecord.SourcePatch
	value facetedrecord.SourcePatch
}

// BrakePlacementSnapshot binds the admitted result payload and its complete
// source chains to one replay budget. Unsupported nested evidence refuses.
type BrakePlacementSnapshot struct {
	payload candidatePayloadSnapshot
	sources []brakePlacementSource
	budget  *proofbound.ReplayBudget
}

func SnapshotBrakePlacement(ctx context.Context, p facetedrecord.Payload, b *proofbound.ReplayBudget) (*BrakePlacementSnapshot, error) {
	if b == nil {
		return nil, errBrakeEvalMissingPlacementBudget
	}
	if err := b.SpendWork(ctx, 32); err != nil {
		return nil, err
	}
	// This payload has no Union shrink record on this branch.
	if p.SmoothRectSection != nil || p.SmoothHomotheticSection != nil ||
		p.PrismPatternJoin != nil || p.ObliqueBoss != nil || p.PrismUnionFillet != nil || p.CrossCylinderBore != nil ||
		p.LoftCylinderJoin != nil || p.LoftCylinderBore != nil || p.LoftMirrorJoin != nil || p.LoftMirrorCopy != nil ||
		p.MirrorCapGroups != nil || p.LoftMirrorCylinderJoin != nil || p.LoftMirrorCylinderBore != nil ||
		p.PointSection != nil || p.PointCone != nil || p.PointPlane != nil || p.PointSimilarity != nil ||
		p.LoftFacesCap != nil || p.LoftFacesToe != nil || p.LoftFacesTrim != nil || p.CopyLineage != nil ||
		p.ExactCopyMoments != nil || p.RevolveLoftJoin != nil || p.RevolveLoftSector != nil || p.LowerSupport != nil {
		return nil, errBrakeEvalUnsupportedPlacementPayloadEvidence
	}
	count := 0
	for _, g := range p.Groups {
		if err := b.SpendWork(ctx, 1); err != nil {
			return nil, err
		}
		switch g.Surface.(type) {
		case nil, surfacegeom.Plane, surfacegeom.Cylinder:
		default:
			return nil, errBrakeEvalUnsupportedPlacementSurface
		}
		if g.Denoted != nil {
			return nil, errBrakeEvalUnsupportedPlacementDenotation
		}
		for node := g.Source; node != nil; node = node.Parent {
			if err := b.SpendWork(ctx, 1); err != nil {
				return nil, err
			}
			count++
			if count > 65536 {
				return nil, errBrakeEvalPlacementSourceChainLimit
			}
		}
	}
	if err := b.ReserveRecords(ctx, uint64(count)+1); err != nil {
		return nil, err
	}
	if err := b.ReserveArray(ctx, uint64(count), uint64(unsafe.Sizeof(brakePlacementSource{}))); err != nil {
		return nil, err
	}
	if err := b.ReserveArray(ctx, 1, uint64(unsafe.Sizeof(BrakePlacementSnapshot{}))); err != nil {
		return nil, err
	}
	sources := make([]brakePlacementSource, 0, count)
	for _, g := range p.Groups {
		for node := g.Source; node != nil; node = node.Parent {
			if err := b.SpendWork(ctx, 1); err != nil {
				return nil, err
			}
			sources = append(sources, brakePlacementSource{ptr: node, value: *node})
		}
	}
	snapshot, err := snapshotCandidatePayload(ctx, p, b)
	if err != nil {
		return nil, err
	}
	return &BrakePlacementSnapshot{payload: snapshot, sources: sources, budget: b}, nil
}

func (s *BrakePlacementSnapshot) Unchanged(ctx context.Context, p facetedrecord.Payload, b *proofbound.ReplayBudget) (bool, error) {
	if s == nil || b == nil || s.budget != b {
		return false, errBrakeEvalPlacementSnapshotBudgetChanged
	}
	if err := b.SpendWork(ctx, uint64(len(s.sources))+uint64(len(p.Groups))); err != nil {
		return false, err
	}
	index := 0
	for _, group := range p.Groups {
		for node := group.Source; node != nil; node = node.Parent {
			if index >= len(s.sources) || node != s.sources[index].ptr || *node != s.sources[index].value {
				return false, nil
			}
			index++
		}
	}
	if index != len(s.sources) {
		return false, nil
	}
	return s.payload.unchanged(ctx, p, b)
}
