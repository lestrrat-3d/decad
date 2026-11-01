package prismtess

import (
	"context"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/massmoment"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
)

// CircleCandidateReplay builds an unpublished candidate. Its caller must seal
// and recheck the source payload, history, and face construction records.
// The returned mesh withholds its boundary and occupied-volume proof flags.
func CircleCandidateReplay(ctx context.Context, body *topo.Body, p prismrecord.Payload, seg sectionrecord.CircleSeg,
	faces [3]*topo.Face, samples tessellation.ChordSamples[*topo.Face], lift float64, charge massmoment.MapCharge,
	budget *proofbound.ReplayBudget) (*topo.Mesh, error) {
	if budget == nil {
		return nil, decaderr.ErrUnsupported
	}
	if err := budget.SpendWork(ctx, 64); err != nil {
		return nil, err
	}
	if body == nil {
		return nil, decaderr.ErrUnsupported
	}
	for i, f := range faces {
		if f == nil || f.Body != body {
			return nil, decaderr.ErrUnsupported
		}
		token, ok := f.PrismConstruction()
		kind, loop, segment := topo.PrismFaceWall, 0, 0
		if i == 1 {
			kind, loop, segment = topo.PrismFaceCapStart, -1, -1
		}
		if i == 2 {
			kind, loop, segment = topo.PrismFaceCapEnd, -1, -1
		}
		if !ok || !token.Matches(f, body, body.Origin.Producer, kind, loop, segment) {
			return nil, decaderr.ErrUnsupported
		}
	}
	mesh, store, err := circleGeometryReplay(ctx, p, samples, faces, lift, budget)
	if err != nil {
		return nil, err
	}
	if err = topobuild.RequireSingleShellWindingReplay(ctx, mesh, body.Lumps, budget); err != nil {
		return nil, err
	}
	area, volume, err := circleMeasuresReplay(ctx, p, seg, samples, mesh, store, charge, budget)
	if err != nil {
		return nil, err
	}
	mesh.AreaSlack, mesh.VolSymDiff = area, volume
	return mesh, nil
}
