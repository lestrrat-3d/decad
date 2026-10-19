package decad

import (
	"context"
	"errors"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/breprecord"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/modifyselect"
	"github.com/lestrrat-3d/decad/internal/offset2d"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
)

// chamferPrismUnion removes one triangular corner prism from an authenticated
// source of a faceted Union. The other source stays strictly outside the
// corner box, so the ordinary Boolean rebuild certifies the modified Union.
func (b *Body) tryChamferPrismUnion(ctx context.Context, edges []*Edge,
	rec *prismUnionFilletRecord, setback, setbackDelta float64) (*Body, bool, error) {
	if len(edges) != 1 || setbackDelta != 0 || !(setback > 0) || proofbound.IsNonFinite(setback) {
		return nil, false, nil
	}
	for _, pair := range [][2]*Body{{bodyHandle(rec.First), bodyHandle(rec.Second)}, {bodyHandle(rec.Second), bodyHandle(rec.First)}} {
		pp, ok := pair[0].n.Payload.(prismPayload)
		if !ok {
			continue
		}
		corner, band, sourceEdge, ok := prismUnionFilletCorner(pair[0], pp, edges[0], setback)
		if !ok || !prismUnionFilletSourceFaces(edges[0], sourceEdge) {
			continue
		}
		if !prismUnionFilletBandSeparate(pair[1], band) {
			return nil, true, fmt.Errorf("%w: another Union source occupies the selected chamfer corner", ErrUnsupported)
		}
		out, err := chamferPrismUnionSource(ctx, documentHandle(b.n.Doc), pp, sourceEdge, corner, setback)
		if err != nil {
			return nil, true, err
		}
		// The removed triangular prism lies inside the certified corner box,
		// which is strictly separate from the unchanged source's true body.
		joined, err := booleanBody(ctx, meshbool.OpUnion, out, pair[1], b.n.Doc.NextProducer)
		if err != nil {
			return nil, true, err
		}
		result, err := commitModifyResult(ctx, b, joined)
		return result, true, err
	}
	return nil, false, nil
}

func chamferPrismUnionSource(ctx context.Context, d *Document, pp prismPayload,
	selected *Edge, corner sectionrecord.Point2, setback float64) (*Body, error) {
	budget := proofbound.NewWorkBudget(ctx)
	loops, err := offset2d.ProfileCornerLoopsBudget(budget, pp.Profile)
	if err != nil {
		return nil, err
	}
	if len(loops) != 1 || len(loops[0].Walks) != 4 {
		return nil, fmt.Errorf("%w: the source rectangle has no four-corner walk", ErrUnsupported)
	}
	li, ci, found, err := modifyselect.MatchCorner(budget, pp, loops, edgeNode(selected))
	if err != nil {
		return nil, err
	}
	if !found || li != 0 || loops[0].Walks[ci].StartU != corner.U ||
		loops[0].Walks[ci].StartV != corner.V {
		return nil, fmt.Errorf("%w: the source edge does not match the authenticated corner", ErrUnsupported)
	}
	blend, err := computeChamfer(loops[0], ci, setback, setback)
	if err != nil {
		return nil, err
	}
	if !offset2d.PrismUnionChamferChord(pp.Profile, corner, setback, blend) {
		return nil, fmt.Errorf("%w: the rectangular corner has no exact inward chamfer chord", ErrUnsupported)
	}
	blends := []map[int]*cornerBlend{{ci: blend}}
	profile, connectors, err := offset2d.RewriteProfileBudget(budget, pp.Profile, loops, blends)
	if err != nil {
		return nil, err
	}
	if err := offset2d.AuditRewriteBudget(budget, pp.Profile, profile, loops, blends); err != nil {
		if errors.Is(err, context.Canceled) || errors.Is(err, context.DeadlineExceeded) {
			return nil, err
		}
		return nil, fmt.Errorf("%w: the rectangular source chamfer failed its section audit: %v", ErrUnsupported, err)
	}
	return bodyResult(prismrecord.Build(ctx, envOf(documentNode(d), d.nextProducerID()), prismrecord.Payload{
		Profile: profile, Frame: pp.Frame, Z0: pp.Z0, Z1: pp.Z1,
		Xform: pp.Xform, BlendSegs: connectors, BlendKind: string(breprecord.BandChamfer),
	}, freeform.NewFreeformWork()))
}
