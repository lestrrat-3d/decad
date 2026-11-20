package decad

import (
	"context"
	"slices"

	"github.com/lestrrat-3d/decad/internal/boolchain"
	"github.com/lestrrat-3d/decad/internal/loftrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/rimextension"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
)

type rimFaceReader struct {
	history pairedHistorySource
	meter   *proofbound.ReplayBudget
	faces   map[*topo.Body][]*topo.Face
}

func (r rimFaceReader) faceList(ctx context.Context, b *topo.Body) ([]*topo.Face, error) {
	if fs, ok := r.faces[b]; ok {
		return fs, nil
	}
	n := 0
	for _, l := range b.Lumps {
		if l == nil {
			return nil, nil
		}
		if err := r.meter.SpendWork(ctx, uint64(1+len(l.Shells))); err != nil {
			return nil, err
		}
		for _, shell := range l.Shells {
			if shell == nil || len(shell.Faces) > 65536-n {
				return nil, nil
			}
			n += len(shell.Faces)
		}
	}
	if err := r.meter.ReserveBundle(ctx, uint64(n), uint64(n), uint64(n)*32); err != nil {
		return nil, err
	}
	fs := topobuild.Faces(b)
	r.faces[b] = fs
	return fs, nil
}

func (r rimFaceReader) bind(ctx context.Context, stock *Body, final *topo.Face, tool bool) (rimextension.BoundFace, bool, error) {
	var none rimextension.BoundFace
	cur, f := stock, final
	cutTools := 0
	for range 64 {
		if e := r.meter.SpendWork(ctx, uint64(8*(len(r.history.h.operations)+1))); e != nil {
			return none, false, e
		}
		if f == nil || f.Body != &cur.n || !r.history.authenticated(cur) {
			return none, false, nil
		}
		op := r.history.h.producers[cur]
		switch op.kind {
		case OperationExtrude, OperationLoft:
			if len(op.inputs) != 0 || tool != (cutTools == 1) {
				return none, false, nil
			}
			if op.kind == OperationExtrude {
				ok, err := cur.n.ReplayPrismBuiltFace(ctx, f, r.meter)
				if err != nil || !ok {
					return none, false, err
				}
			} else {
				p, ok := cur.n.Payload.(loftrecord.Payload)
				if !ok || !p.AuthenticatedReconstruction || p.SourceIdentity == nil || p.SourceIdentity.Origin != cur.n.Origin.Producer || !p.MatchesDirectSections(&cur.n) {
					return none, false, nil
				}
			}
			return rimextension.BoundFace{Body: &cur.n, Native: f, Final: final}, true, nil
		case OperationCut, OperationUnion:
			_, ins, ok := r.history.BooleanProducer(&cur.n)
			p := f.Source
			if !ok || p == nil || p.Blend || len(p.Members) != 0 || p.Operand < 0 || p.Operand >= len(ins) {
				return none, false, nil
			}
			if p.Operand == 1 && op.kind == OperationCut {
				cutTools++
				if cutTools > 1 {
					return none, false, nil
				}
			}
			if p.Operand == 1 && !boolchain.WarningsMoveNoGeometry(&cur.n) {
				return none, false, nil
			}
			b := ins[p.Operand]
			if p.Producer != b.Origin.Producer {
				return none, false, nil
			}
			fs, e := r.faceList(ctx, b)
			if e != nil {
				return none, false, e
			}
			if p.Ordinal < 0 || p.Ordinal >= len(fs) || fs[p.Ordinal].Source != p.Parent {
				return none, false, nil
			}
			f, cur = fs[p.Ordinal], bodyHandle(b)
		case OperationChamfer, OperationFillet:
			if len(op.inputs) != 1 || op.inputs[0].Use != InputConsumed || op.inputs[0].Body == nil || f.Source == nil {
				return none, false, nil
			}
			b := op.inputs[0].Body
			if !slices.Contains(r.history.h.consumers[b], op) {
				return none, false, nil
			}
			fs, e := r.faceList(ctx, &b.n)
			if e != nil {
				return none, false, e
			}
			if err := r.meter.SpendWork(ctx, uint64(len(fs))); err != nil {
				return none, false, err
			}
			var previous *topo.Face
			for _, v := range fs {
				if v.Source == f.Source {
					if previous != nil {
						return none, false, nil
					}
					previous = v
				}
			}
			if previous == nil || previous.Surface != f.Surface || previous.Reversed != f.Reversed || previous.RetainedTag != f.RetainedTag {
				return none, false, nil
			}
			cur, f = b, previous
		default:
			return none, false, nil
		}
	}
	return none, false, nil
}
