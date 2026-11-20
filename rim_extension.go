package decad

import (
	"context"

	"github.com/lestrrat-3d/decad/internal/brepmodify"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"

	"github.com/lestrrat-3d/decad/internal/rimextension"
	"github.com/lestrrat-3d/decad/internal/topo"
)

func extendStraightRim(ctx context.Context, d *Document, b *Body, e *topo.Edge, setback float64) (brepmodify.Result, bool, error) {
	var none brepmodify.Result
	if _, suppressed := ctx.Value(operationHistorySuppressedKey{}).(*operationHistoryCapture); suppressed {
		return none, false, nil
	}
	m := proofbound.NewStrictReplayBudget()
	c, ok, err := rimextension.Identify(ctx, &b.n, e, m)
	if !ok || err != nil {
		return none, false, err
	}
	h, ok, err := rimHistorySnapshot(ctx, d, m)
	if !ok || err != nil {
		return none, false, err
	}
	r := rimFaceReader{pairedHistorySource{h}, m, map[*topo.Body][]*topo.Face{}}
	a := rimextension.Admission{Stock: &b.n, Section: c.Section, Setback: setback, Allowed: c.Allowed}
	for _, pair := range []struct {
		final *topo.Face
		bound *rimextension.BoundFace
	}{{c.Cap, &a.Cap}, {c.Side, &a.Side}, {c.Boss, &a.Boss}, {c.Round, &a.Round}} {
		if pair.final.TagDeparture != 0 || pair.final.ExtentDelta != 0 || pair.final.AxialDelta != 0 {
			return none, false, nil
		}
		*pair.bound, ok, err = r.bind(ctx, b, pair.final, false)
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
	w, ok, err := rimextension.AdmitWindow(ctx, a, m)
	if !ok || err != nil {
		return none, false, err
	}
	env := envOf(b.n.Doc, b.n.Doc.NextProducer)
	result, ok, err := rimextension.Prepare(ctx, env, a, c, w, m)
	if !ok || err != nil {
		return none, false, err
	}
	return publishPreparedRim(ctx, env, b, result, m)
}

func publishPreparedRim(ctx context.Context, env topo.Env, b *Body, result rimextension.Prepared, m *proofbound.ReplayBudget) (brepmodify.Result, bool, error) {
	var none brepmodify.Result
	old, ok := b.n.Payload.(facetedrecord.Payload)
	if !ok {
		return none, false, nil
	}
	p := result.Payload
	local, ok, err := rimLocalTriangles(ctx, old, p, result.Min, result.Max, m)
	if !ok || err != nil {
		return none, false, err
	}
	local, err = rimextension.ChangedNeighbors(ctx, p.Verts, local, result.Previous, m)
	if err != nil {
		return none, false, err
	}
	return publishRim(ctx, env, p, local, m)
}
