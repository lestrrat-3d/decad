package decad

import (
	"context"

	"github.com/lestrrat-3d/decad/internal/boolchain"
	"github.com/lestrrat-3d/decad/internal/booleaneval"
	"github.com/lestrrat-3d/decad/internal/loftrecord"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
)

type cheekCutSource struct {
	reader       rimFaceReader
	root         *upperRimProof
	target, tool *Body
}

func cheekCutCandidate(d *Document, target *Body) bool {
	d.n.LiveMu.RLock()
	defer d.n.LiveMu.RUnlock()
	cur := target
	for range 3 {
		if cur == nil {
			return false
		}
		if rec := cur.upperRimProof; rec != nil {
			return rec.alignedCheek != nil
		}
		op := d.producers[cur]
		if op == nil || op.kind != OperationCut || len(op.inputs) != 2 {
			return false
		}
		cur = op.inputs[0].Body
	}
	return false
}

func nativeCheekCutSource(ctx context.Context, target, tool *Body, m *proofbound.ReplayBudget) (*cheekCutSource, bool, error) {
	if target == nil || tool == nil || target.n.Doc == nil || target.n.Doc != tool.n.Doc || !directCheekLoft(tool) {
		return nil, false, nil
	}
	d := target.Document()
	h, ok, err := rimHistorySnapshot(ctx, d, m)
	if !ok || err != nil {
		return nil, false, err
	}
	history := pairedHistorySource{h}
	if !history.authenticated(tool) {
		return nil, false, nil
	}
	s := cheekCutSource{reader: rimFaceReader{history, m, map[*topo.Body][]*topo.Face{}}, target: target, tool: tool}
	cur := target
	for range 3 {
		if !history.authenticated(cur) {
			return nil, false, nil
		}
		if rec := cur.upperRimProof; rec != nil {
			if rec.result != cur || rec.alignedCheek == nil || rec.stock == nil || rec.cone == nil {
				return nil, false, nil
			}
			// The same sealed circular construction must own both the cone and its
			// consumed stock, even following two unrelated previous subtractions.
			check, ok, err := s.reader.outerAncestor(ctx, cur, rec.cone, true)
			if !ok || check != rec || err != nil {
				return nil, false, err
			}
			s.root = rec
			return &s, true, nil
		}
		k, inputs, ok := history.BooleanProducer(&cur.n)
		if !ok || k != meshbool.OpCut || !boolchain.WarningsMoveNoGeometry(&cur.n) {
			return nil, false, nil
		}
		cutter := bodyHandle(inputs[1])
		if !directCheekLoft(cutter) {
			return nil, false, nil
		}
		cur = bodyHandle(inputs[0])
	}
	return nil, false, nil
}

func directCheekLoft(b *Body) bool {
	if b == nil {
		return false
	}
	p, ok := b.n.Payload.(loftrecord.Payload)
	return ok && b.n.Solid && p.SourceIdentity != nil && p.SourceIdentity.Origin == b.n.Origin.Producer && p.AuthenticatedReconstruction &&
		!p.ThroughBoreCut && !p.SurfaceResult && p.Xform == r3.Identity() && p.ConstructionDelta == 0 && p.JoinDelta == 0 && p.JoinPlaneGap == 0 && p.MatchesDirectSections(&b.n)
}

// The evaluator invokes this provider only after its ordinary coarse-contact
// refusal. Successful ordinary Cuts never pay for source reconstruction.
func withNativeCutBounds(ctx context.Context, a, b *Body) context.Context {
	return booleaneval.WithWholeCutBounds(ctx, &a.n, &b.n, proofbound.NewStrictReplayBudget(), func(ctx context.Context, stock, tool *topo.Mesh, m *proofbound.ReplayBudget) (map[int]meshbool.CutFacetBound, bool, error) {
		source, ok, err := nativeCheekCutSource(ctx, a, b, m)
		if err != nil || !ok {
			return nil, false, err
		}
		return source.contactBounds(ctx, stock, tool, m)
	})
}
