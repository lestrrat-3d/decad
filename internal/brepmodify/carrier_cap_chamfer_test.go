package brepmodify_test

import (
	"context"
	"fmt"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/brepmodify"
	"github.com/lestrrat-3d/decad/internal/capblend"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// The cap-loop route of agent-docs/carrier-record-design.md §8.1. The fixture
// is a 20 mm square with one 4 mm square hole, 10 mm tall, recorded as a
// displaced prism: one loop carries entries of capLoopEntry and the other is
// exact. A Chamfer of the exact loop's cap edges builds a DisplacedPayload.

const capLoopEntry = 1e-13

func capLoopSquare(side float64) sectionrecord.LoopRecord {
	return sectionrecord.LoopRecord{Segments: capLoopPoly(
		sectionrecord.Point2{U: 0, V: 0}, sectionrecord.Point2{U: side, V: 0},
		sectionrecord.Point2{U: side, V: side}, sectionrecord.Point2{U: 0, V: side})}
}

func capLoopHole(u, v, s float64) sectionrecord.LoopRecord {
	return sectionrecord.LoopRecord{Segments: capLoopPoly(
		sectionrecord.Point2{U: u, V: v}, sectionrecord.Point2{U: u, V: v + s},
		sectionrecord.Point2{U: u + s, V: v + s}, sectionrecord.Point2{U: u + s, V: v})}
}

func capLoopPoly(pts ...sectionrecord.Point2) []sectionrecord.CurveSegment {
	segs := make([]sectionrecord.CurveSegment, len(pts))
	for i := range pts {
		segs[i] = sectionrecord.LineSeg{Start: pts[i], End: pts[(i+1)%len(pts)], TStart: 0, TEnd: 1}
	}
	return segs
}

// capLoopReceiver records the square with its hole; entries[l] is the entry of
// every segment and junction of loop l.
func capLoopReceiver(t *testing.T, delta float64, entries ...float64) (*topo.Body, topo.Env) {
	t.Helper()
	profile := momentinput.Profile{Outer: capLoopSquare(20), Holes: []sectionrecord.LoopRecord{capLoopHole(8, 8, 4)}}
	cr := prismrecord.ZeroCarrierRecord(profile)
	cr.SectionDelta = delta
	for l, e := range entries {
		for j := range cr.Segment[l] {
			cr.Segment[l][j], cr.Junction[l][j] = e, e
		}
	}
	return buildReceiver(t, prismrecord.Payload{Profile: profile, Frame: carrierFrame(t), Z1: 10, Xform: r3.Identity(),
		SectionDelta: delta, Carriers: &cr})
}

// capLoopEdges are the body's edges on the z = z plane whose both ends lie
// inside the box [lo, hi]² (the hole's rim) or, with outer, on the square's
// boundary.
func capLoopEdges(t *testing.T, b *topo.Body, z float64, outer bool) []*topo.Edge {
	t.Helper()
	var out []*topo.Edge
	for _, e := range topobuild.Edges(b) {
		if _, ok := e.Curve.(surfacegeom.Line3); !ok || e.Start == nil || e.End == nil {
			continue
		}
		a, c := e.Start.Position, e.End.Position
		if math.Abs(a.Z-z) > 1e-9 || math.Abs(c.Z-z) > 1e-9 {
			continue
		}
		onOuter := func(p r3.Vec) bool {
			return math.Abs(p.X) < 1e-9 || math.Abs(p.X-20) < 1e-9 || math.Abs(p.Y) < 1e-9 || math.Abs(p.Y-20) < 1e-9
		}
		if onOuter(a) && onOuter(c) == outer || !outer && !onOuter(a) && !onOuter(c) {
			out = append(out, e)
		}
	}
	return out
}

func capLoopCall(t *testing.T, b *topo.Body, env topo.Env, edges []*topo.Edge, d float64, asym *brepmodify.Asymmetric) (brepmodify.Result, error) {
	t.Helper()
	return brepmodify.Chamfer(t.Context(), env, b, brepmodify.ChamferCall{Sel: "rims", Edges: edges, D: d, Asym: asym})
}

// answer is what a Chamfer answered: the payload class it built or its error.
func answer(res brepmodify.Result, err error) string {
	if err != nil {
		return "error: " + err.Error()
	}
	return fmt.Sprintf("%T", res.Body.Payload)
}

func bothCaps(t *testing.T, b *topo.Body, outer bool) []*topo.Edge {
	return append(capLoopEdges(t, b, 0, outer), capLoopEdges(t, b, 10, outer)...)
}

func TestCarrierCapLoopChamferBuilds(t *testing.T) {
	t.Run("an exact hole loop on both caps", func(t *testing.T) {
		body, env := capLoopReceiver(t, 1e-12, capLoopEntry, 0)
		edges := bothCaps(t, body, false)
		require.Len(t, edges, 8)
		res, err := capLoopCall(t, body, env, edges, 0.5, nil)
		require.NoError(t, err)
		dp, ok := res.Body.Payload.(capblend.DisplacedPayload)
		require.True(t, ok, answer(res, err))
		require.Equal(t, []float64{capLoopEntry, 0}, dp.LoopDelta)
		require.Equal(t, map[int]bool{1: true}, dp.Base.StartLoops)
		require.Equal(t, map[int]bool{1: true}, dp.Base.EndLoops)
		require.Equal(t, 0.5, dp.Base.Start.Dc)
	})
	t.Run("an exact outer loop with a displaced hole", func(t *testing.T) {
		// The control of the SX6 case below: the same receiver and selection
		// build at a setback the outer loop survives.
		body, env := capLoopReceiver(t, 1e-12, 0, capLoopEntry)
		res, err := capLoopCall(t, body, env, bothCaps(t, body, true), 0.5, nil)
		require.NoError(t, err)
		dp, ok := res.Body.Payload.(capblend.DisplacedPayload)
		require.True(t, ok, answer(res, err))
		require.Equal(t, []float64{0, capLoopEntry}, dp.LoopDelta)
	})
	t.Run("an asymmetric chamfer", func(t *testing.T) {
		body, env := capLoopReceiver(t, 1e-12, capLoopEntry, 0)
		edges := bothCaps(t, body, false)
		caps := brepmodify.CapsOf(body)
		refs := map[*topo.Edge]*topo.Face{}
		for _, e := range edges {
			for _, f := range e.Faces {
				if f == caps.Start || f == caps.End {
					refs[e] = f
				}
			}
		}
		asym := &brepmodify.Asymmetric{Body: body, Refs: refs, D: 0.5, Other: 0.3}
		res, err := capLoopCall(t, body, env, edges, 0.5, asym)
		require.NoError(t, err)
		dp, ok := res.Body.Payload.(capblend.DisplacedPayload)
		require.True(t, ok, answer(res, err))
		require.Equal(t, 0.5, dp.Base.Start.Dc)
		require.Equal(t, 0.3, dp.Base.Start.Ds)
	})
}

// Each of these is a selection or refusal the route does not own: it answers
// matched = false with no error, so Chamfer continues to RequireExactSection
// and the faceted view as it did before the route existed. The faceted view
// itself needs the root package's mesher; apitest compares those answers.
func TestCarrierCapLoopChamferDeclines(t *testing.T) {
	type tc struct {
		name  string
		build func(t *testing.T) (*topo.Body, topo.Env, []*topo.Edge, float64)
	}
	cases := []tc{
		{"a selected loop with a nonzero entry", func(t *testing.T) (*topo.Body, topo.Env, []*topo.Edge, float64) {
			body, env := capLoopReceiver(t, 1e-12, capLoopEntry, capLoopEntry)
			return body, env, bothCaps(t, body, false), 0.5
		}},
		{"a partial selection", func(t *testing.T) (*topo.Body, topo.Env, []*topo.Edge, float64) {
			body, env := capLoopReceiver(t, 1e-12, capLoopEntry, 0)
			return body, env, capLoopEdges(t, body, 10, false)[:2], 0.5
		}},
		{"a mixed selection", func(t *testing.T) (*topo.Body, topo.Env, []*topo.Edge, float64) {
			body, env := capLoopReceiver(t, 1e-12, capLoopEntry, 0)
			return body, env, append(capLoopEdges(t, body, 10, false), capLoopEdges(t, body, 10, true)[:1]...), 0.5
		}},
		{"a transfer failure (T1)", func(t *testing.T) (*topo.Body, topo.Env, []*topo.Edge, float64) {
			body, env := capLoopReceiver(t, 1e-6, 1e-6, 0)
			return body, env, bothCaps(t, body, false), 0.5
		}},
		{"an SX6 drop", func(t *testing.T) (*topo.Body, topo.Env, []*topo.Edge, float64) {
			body, env := capLoopReceiver(t, 1e-12, 0, capLoopEntry)
			return body, env, bothCaps(t, body, true), 10.5
		}},
	}
	for _, c := range cases {
		t.Run(c.name, func(t *testing.T) {
			body, env, edges, d := c.build(t)
			require.NotEmpty(t, edges)
			cr, valid, err := prismrecord.CarrierRecordOf(proofbound.NewWorkBudget(t.Context()), body)
			require.NoError(t, err)
			require.True(t, valid)
			_, matched, err := brepmodify.CarrierCapLoopChamfer(t.Context(), env, body.Payload.(prismrecord.Payload),
				brepmodify.CapsOf(body), cr, "rims", edges, d, 0, nil)
			require.NoError(t, err)
			require.False(t, matched)
		})
	}
	t.Run("a placed copy carries no record", func(t *testing.T) {
		body, env := capLoopReceiver(t, 1e-12, capLoopEntry, 0)
		move, err := r3.Translation(r3.NewVec(1, 2, 3))
		require.NoError(t, err)
		placed, err := body.Payload.(prismrecord.Payload).Placed(t.Context(), env, move)
		require.NoError(t, err)
		_, valid, err := prismrecord.CarrierRecordOf(proofbound.NewWorkBudget(t.Context()), placed)
		require.NoError(t, err)
		require.False(t, valid, "Chamfer reads no record, so the route is skipped")
	})
	t.Run("the route switched off builds no displaced payload", func(t *testing.T) {
		body, env := capLoopReceiver(t, 1e-12, capLoopEntry, 0)
		defer brepmodify.DisableCapLoopRoute()()
		cr, _, err := prismrecord.CarrierRecordOf(proofbound.NewWorkBudget(t.Context()), body)
		require.NoError(t, err)
		_, matched, err := brepmodify.CarrierCapLoopChamfer(t.Context(), env, body.Payload.(prismrecord.Payload),
			brepmodify.CapsOf(body), cr, "rims", bothCaps(t, body, false), 0.5, 0, nil)
		require.NoError(t, err)
		require.False(t, matched)
	})
}

func TestCarrierCapLoopChamferReturnsCancellation(t *testing.T) {
	body, env := capLoopReceiver(t, 1e-12, capLoopEntry, 0)
	edges := bothCaps(t, body, false)
	pp := body.Payload.(prismrecord.Payload)
	cr, valid, err := prismrecord.CarrierRecordOf(proofbound.NewWorkBudget(t.Context()), body)
	require.NoError(t, err)
	require.True(t, valid)
	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	_, matched, err := brepmodify.CarrierCapLoopChamfer(ctx, env, pp, brepmodify.CapsOf(body), cr, "rims", edges, 0.5, 0, nil)
	require.ErrorIs(t, err, context.Canceled)
	require.False(t, matched)
}
