package brepmodify_test

import (
	"context"
	"errors"
	"fmt"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/brepmodify"
	"github.com/lestrrat-3d/decad/internal/capblend"
	"github.com/lestrrat-3d/decad/internal/decaderr"
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
	return capLoopReceiverAt(t, delta, 8, entries...)
}

// capLoopReceiverAt is capLoopReceiver with the hole's left side at u.
func capLoopReceiverAt(t *testing.T, delta, u float64, entries ...float64) (*topo.Body, topo.Env) {
	t.Helper()
	profile := momentinput.Profile{Outer: capLoopSquare(20), Holes: []sectionrecord.LoopRecord{capLoopHole(u, 8, 4)}}
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
	// floor is the contact floor F of the 20 mm square, 1e-9 of its diagonal.
	floor := 1e-9 * 20 * math.Sqrt2
	type tc struct {
		name   string
		row    string // the transfer row capblend names, when the refusal is one
		dDelta float64
		build  func(t *testing.T) (*topo.Body, topo.Env, []*topo.Edge, float64)
	}
	cases := []tc{
		{"a selected loop with a nonzero entry", "", 0, func(t *testing.T) (*topo.Body, topo.Env, []*topo.Edge, float64) {
			body, env := capLoopReceiver(t, 1e-12, capLoopEntry, capLoopEntry)
			return body, env, bothCaps(t, body, false), 0.5
		}},
		{"a partial selection", "", 0, func(t *testing.T) (*topo.Body, topo.Env, []*topo.Edge, float64) {
			body, env := capLoopReceiver(t, 1e-12, capLoopEntry, 0)
			return body, env, capLoopEdges(t, body, 10, false)[:2], 0.5
		}},
		{"a mixed selection", "", 0, func(t *testing.T) (*topo.Body, topo.Env, []*topo.Edge, float64) {
			body, env := capLoopReceiver(t, 1e-12, capLoopEntry, 0)
			return body, env, append(capLoopEdges(t, body, 10, false), capLoopEdges(t, body, 10, true)[:1]...), 0.5
		}},
		// T1: four times the largest entry (4e-6) is not below F (2.8e-8).
		{"a transfer failure (T1)", "T1)", 0, func(t *testing.T) (*topo.Body, topo.Env, []*topo.Edge, float64) {
			body, env := capLoopReceiver(t, 1e-6, 1e-6, 0)
			return body, env, bothCaps(t, body, false), 0.5
		}},
		// T2: the entry is tiny, so T1 holds, and the hole's chamfer contour
		// stands F + 5.2e-9 from the outer wall at x = 20 on the held setback.
		// The setback's 5e-9 conversion rounding is charged to the contour, so
		// no certified gap clears F. The offset audit passes at the held
		// setback and at the top of its span (a gap of F + 2e-10 there), so T2
		// alone refuses it. A near-end crossing never reaches T2 here: the
		// offset audit finds it first (prismcells' CertifiedContact test holds
		// that reading).
		{"a transfer failure (T2)", "T2)", 5e-9, func(t *testing.T) (*topo.Body, topo.Env, []*topo.Edge, float64) {
			body, env := capLoopReceiverAt(t, 1e-12, 20-4-0.5-floor-5.2e-9, capLoopEntry, 0)
			// The outer loop is the displaced one; the hole is the selection.
			return body, env, bothCaps(t, body, false), 0.5
		}},
		{"an SX6 drop", "", 0, func(t *testing.T) (*topo.Body, topo.Env, []*topo.Edge, float64) {
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
			pp := body.Payload.(prismrecord.Payload)
			_, matched, err := brepmodify.CarrierCapLoopChamfer(t.Context(), env, pp,
				brepmodify.CapsOf(body), cr, "rims", edges, d, c.dDelta, nil)
			require.NoError(t, err)
			require.False(t, matched)
			if c.row == "" {
				return
			}
			// The refusal is the named transfer row's, not another gate's.
			set := capblend.Setback{Dc: d, DcDelta: c.dDelta, Ds: d, DsDelta: c.dDelta}
			loopDelta := []float64{cr.LoopH(0), cr.LoopH(1)}
			_, err = capblend.BuildCheckedDisplaced(t.Context(), env, pp, set, set,
				map[int]bool{1: true}, map[int]bool{1: true}, loopDelta)
			require.ErrorIs(t, err, decaderr.ErrUnsupported)
			require.Contains(t, err.Error(), "§8.3, "+c.row)
			require.Contains(t, err.Error(), "carrier-record-design.md")
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

// refusingMesher stands for the root package's mesher: the faceted view the
// route's refusals fall through to meshes the receiver first, and this one
// ends that view with its own error, so a test reads "reached the faceted
// view" from the error and no body.
type refusingMesher struct{}

var errFacetedView = errors.New("faceted view reached")

func (refusingMesher) TessellateOperand(context.Context, *topo.Body, float64) (*topo.Mesh, bool, error) {
	return nil, false, errFacetedView
}

// TestCarrierCapLoopChamferFallsThrough compares Chamfer with the cap-loop
// route switched on and off. Where the route owns the call it answers a
// DisplacedPayload, and off the same call reaches today's faceted view
// (the stand-in mesher's error). Every call the route declines answers the
// same whether it is on or off: the fall-through is the route's own refusal,
// not a different answer.
func TestCarrierCapLoopChamferFallsThrough(t *testing.T) {
	floor := 1e-9 * 20 * math.Sqrt2
	type tc struct {
		name   string
		dDelta float64
		owned  bool
		end    string // where a declined call ends, on or off
		build  func(t *testing.T) (*topo.Body, topo.Env, []*topo.Edge, float64)
	}
	const refused, viewed = "the section it denotes only", "faceted view reached"
	cases := []tc{
		{"an exact hole loop on both caps", 0, true, "", func(t *testing.T) (*topo.Body, topo.Env, []*topo.Edge, float64) {
			body, env := capLoopReceiver(t, 1e-12, capLoopEntry, 0)
			return body, env, bothCaps(t, body, false), 0.5
		}},
		{"a nonzero entry on the selected loop", 0, false, refused, func(t *testing.T) (*topo.Body, topo.Env, []*topo.Edge, float64) {
			body, env := capLoopReceiver(t, 1e-12, capLoopEntry, capLoopEntry)
			return body, env, bothCaps(t, body, false), 0.5
		}},
		{"a T1 failure", 0, false, viewed, func(t *testing.T) (*topo.Body, topo.Env, []*topo.Edge, float64) {
			body, env := capLoopReceiver(t, 1e-6, 1e-6, 0)
			return body, env, bothCaps(t, body, false), 0.5
		}},
		{"a T2 failure", 5e-9, false, viewed, func(t *testing.T) (*topo.Body, topo.Env, []*topo.Edge, float64) {
			body, env := capLoopReceiverAt(t, 1e-12, 20-4-0.5-floor-5.2e-9, capLoopEntry, 0)
			return body, env, bothCaps(t, body, false), 0.5
		}},
		{"an SX6 drop", 0, false, viewed, func(t *testing.T) (*topo.Body, topo.Env, []*topo.Edge, float64) {
			body, env := capLoopReceiver(t, 1e-12, 0, capLoopEntry)
			return body, env, bothCaps(t, body, true), 10.5
		}},
	}
	for _, c := range cases {
		t.Run(c.name, func(t *testing.T) {
			body, env, edges, d := c.build(t)
			call := brepmodify.ChamferCall{Sel: "rims", Edges: edges, D: d, DDelta: c.dDelta,
				Hooks: brepmodify.Hooks{Mesher: refusingMesher{}}}
			on, onErr := brepmodify.Chamfer(t.Context(), env, body, call)
			restore := brepmodify.DisableCapLoopRoute()
			off, offErr := brepmodify.Chamfer(t.Context(), env, body, call)
			restore()
			if c.owned {
				require.NoError(t, onErr)
				_, ok := on.Body.Payload.(capblend.DisplacedPayload)
				require.True(t, ok, answer(on, onErr))
				require.ErrorIs(t, offErr, errFacetedView, "off, the call reaches the faceted view: %s", answer(off, offErr))
				return
			}
			require.Error(t, offErr)
			require.Contains(t, offErr.Error(), c.end, "off: %s", answer(off, offErr))
			require.Equal(t, answer(off, offErr), answer(on, onErr), "the route declines, so on answers as off does")
		})
	}
}
