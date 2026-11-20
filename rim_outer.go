package decad

import (
	"context"
	"slices"

	"github.com/lestrrat-3d/decad/internal/brepmodify"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/rimextension"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
)

// upperRimProof is attached only to the result of the native circular rim
// evaluator. A later shoulder replays that operation's consumed stock and
// follows the selected wall through the intervening modifications.
type upperRimProof struct {
	stock, result          *Body
	pad                    *topo.Body
	wall, sourceWall, cone *topo.Face
	shift                  r3.Vec
	geometry               rimextension.OuterAdmission
}

func (r rimFaceReader) outerAncestor(ctx context.Context, body *Body, final *topo.Face, cone bool) (*upperRimProof, bool, error) {
	cur, f := body, final
	for range 32 {
		if err := r.meter.SpendWork(ctx, uint64(len(r.history.h.operations)+1)); err != nil {
			return nil, false, err
		}
		if f == nil || f.Body != &cur.n || !r.history.authenticated(cur) {
			return nil, false, nil
		}
		op := r.history.h.producers[cur]
		if rec := cur.upperRimProof; rec != nil {
			if op.kind != OperationChamfer || rec.result != cur || len(op.inputs) != 1 || op.inputs[0].Body != rec.stock || op.inputs[0].Use != InputConsumed || !slices.Contains(r.history.h.consumers[rec.stock], op) {
				return nil, false, nil
			}
			if cone {
				if f != rec.cone {
					return nil, false, nil
				}
			} else if f.Source != rec.sourceWall.Source || f.Surface != rec.sourceWall.Surface || f.Reversed != rec.sourceWall.Reversed {
				return nil, false, nil
			}
			return rec, true, nil
		}
		if len(op.inputs) == 0 || op.inputs[0].Body == nil || op.inputs[0].Use != InputConsumed {
			return nil, false, nil
		}
		b := op.inputs[0].Body
		fs, err := r.faceList(ctx, &b.n)
		if err != nil {
			return nil, false, err
		}
		switch op.kind {
		case OperationCut:
			_, ins, ok := r.history.BooleanProducer(&cur.n)
			p := f.Source
			if !ok || ins[0] != &b.n || p == nil || p.Blend || len(p.Members) != 0 || p.Operand != 0 || p.Producer != b.n.Origin.Producer || p.Ordinal < 0 || p.Ordinal >= len(fs) || fs[p.Ordinal].Source != p.Parent {
				return nil, false, nil
			}
			f = fs[p.Ordinal]
		case OperationChamfer:
			if len(op.inputs) != 1 || !slices.Contains(r.history.h.consumers[b], op) || f.Source == nil {
				return nil, false, nil
			}
			var prev *topo.Face
			if err := r.meter.SpendWork(ctx, uint64(len(fs))); err != nil {
				return nil, false, err
			}
			for _, x := range fs {
				if x.Source == f.Source {
					if prev != nil {
						return nil, false, nil
					}
					prev = x
				}
			}
			if prev == nil || prev.Reversed != f.Reversed || prev.RetainedTag != f.RetainedTag {
				return nil, false, nil
			}
			_, previousFaceted := prev.Surface.(surfacegeom.Faceted)
			_, currentFaceted := f.Surface.(surfacegeom.Faceted)
			restatedCone := cone && previousFaceted && currentFaceted && !f.RetainedTag
			if prev.Surface != f.Surface && !restatedCone {
				return nil, false, nil
			}
			f = prev
		default:
			return nil, false, nil
		}
		cur = b
	}
	return nil, false, nil
}

func extendOuterRim(ctx context.Context, d *Document, b *Body, e *topo.Edge, setback float64) (brepmodify.Result, bool, error) {
	var none brepmodify.Result
	if _, suppressed := ctx.Value(operationHistorySuppressedKey{}).(*operationHistoryCapture); suppressed {
		return none, false, nil
	}
	m := proofbound.NewStrictReplayBudget()
	for _, wing := range []int{1, -1} {
		c, ok, err := rimextension.IdentifyOuter(ctx, &b.n, e, wing, m)
		if err != nil {
			return none, false, err
		}
		if !ok || c.Section.Stop.X >= c.Section.End.X {
			continue
		}
		h, ok, err := rimHistorySnapshot(ctx, d, m)
		if !ok || err != nil {
			return none, false, err
		}
		r := rimFaceReader{pairedHistorySource{h}, m, map[*topo.Body][]*topo.Face{}}
		rec, ok, err := r.outerAncestor(ctx, b, c.Boss, false)
		if !ok || err != nil {
			return none, false, err
		}
		a := rec.geometry
		a.Stock = &b.n
		a.Wing = wing
		a.Section = c.Section
		a.Setback = setback
		a.Boss = rimextension.BoundFace{Final: c.Boss, Body: rec.pad, Native: rec.wall, Translation: rec.shift}
		fs, err := r.faceList(ctx, &b.n)
		if err != nil {
			return none, false, err
		}
		if err := r.meter.SpendWork(ctx, 32*uint64(len(fs))); err != nil {
			return none, false, err
		}
		for _, f := range fs {
			p := f.Source
			for depth := 0; p != nil && depth < 32; depth++ {
				if p.Blend && p.Producer == rec.result.n.Origin.Producer && p.Ordinal == 0 {
					if a.Cone != nil {
						return none, false, nil
					}
					lineage, valid, err := r.outerAncestor(ctx, b, f, true)
					if err != nil {
						return none, false, err
					}
					if !valid || lineage != rec {
						return none, false, nil
					}
					a.Cone = f
					break
				}
				if p.Blend || len(p.Members) != 0 || p.Operand != 0 {
					break
				}
				p = p.Parent
			}
		}
		if a.Cone == nil {
			return none, false, nil
		}
		for _, p := range []struct {
			f      *topo.Face
			target *rimextension.BoundFace
		}{{c.Cap, &a.Cap}, {c.Side, &a.Side}, {c.Round, &a.Round}} {
			*p.target, ok, err = r.bind(ctx, b, p.f, false)
			if !ok || err != nil {
				return none, false, err
			}
		}
		for _, f := range c.Stops {
			v, ok, err := r.bind(ctx, b, f, true)
			if !ok || err != nil {
				return none, false, err
			}
			a.Stop = append(a.Stop, v)
		}
		w, ok, err := rimextension.AdmitOuterWindow(ctx, a, m)
		if !ok || err != nil {
			return none, false, err
		}
		env := envOf(b.n.Doc, b.n.Doc.NextProducer)
		out, ok, err := rimextension.PrepareOuter(ctx, env, a, c, w, m)
		if !ok || err != nil {
			return none, false, err
		}
		return publishPreparedRim(ctx, env, b, out, m)
	}
	return none, false, nil
}
