package brepmodify_test

import (
	"errors"
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/circularbounds"

	"github.com/lestrrat-3d/decad/internal/brepmodify"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionaudit"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// The carrier route of agent-docs/carrier-record-design.md §7. The fixtures
// are a disc with one all-line tooth, drawn the way the pair Union records
// it: a trimmed CircleSeg and whole flanks whose junctions carry a few ulps
// of cut-parameter rounding. The tooth's two roots are its concave corners.

const (
	carrierR     = 20.0 // disc radius
	carrierTop   = 26.0 // tooth tip
	carrierHalf  = 2.0  // tooth half width
	carrierDelta = 1e-12
	carrierJunc  = 1e-13
)

// carrierTooth is the disc-and-tooth outline rotated by turn (a fraction of
// a full turn) about the origin, with its record's entries: ε on every
// carrier and δ on every junction.
func carrierTooth(turn float64) (momentinput.Profile, []sectionrecord.Point2) {
	x0 := math.Sqrt(carrierR*carrierR - carrierHalf*carrierHalf)
	t0 := math.Atan2(carrierHalf, x0) / (2 * math.Pi)
	c, s := math.Cos(2*math.Pi*turn), math.Sin(2*math.Pi*turn)
	rot := func(u, v float64) sectionrecord.Point2 {
		if turn == 0 {
			return sectionrecord.Point2{U: u, V: v}
		}
		return sectionrecord.Point2{U: c*u - s*v, V: s*u + c*v}
	}
	line := func(a, b sectionrecord.Point2) sectionrecord.CurveSegment {
		return sectionrecord.LineSeg{Start: a, End: b, TStart: 0, TEnd: 1}
	}
	d, tipD, tipU, u := rot(x0, -carrierHalf), rot(carrierTop, -carrierHalf), rot(carrierTop, carrierHalf), rot(x0, carrierHalf)
	// The unrotated disc is a trimmed CircleSeg, as case 1 records it. A
	// rotated copy is the arc its pattern instance records: an ArcSeg from the
	// tooth's upper root round to its lower one.
	var circle sectionrecord.CurveSegment = sectionrecord.CircleSeg{
		Radius: units.Millimeters(carrierR), CCW: true, TStart: t0, TEnd: 1 - t0}
	if turn != 0 {
		circle = sectionrecord.ArcSeg{Center: rot(0, 0), Start: u, End: d, TStart: 0, TEnd: 1}
	}
	profile := momentinput.Profile{Outer: sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{
		circle, line(d, tipD), line(tipD, tipU), line(tipU, u),
	}}}
	return profile, []sectionrecord.Point2{d, u}
}

func carrierFrame(t *testing.T) r3.Frame {
	t.Helper()
	f, err := r3.NewFrame(r3.Vec{}, r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	return f
}

// recordedReceiver is a displaced prism with a carrier record.
func recordedReceiver(t *testing.T, profile momentinput.Profile, entries func(*prismrecord.CarrierRecord)) prismrecord.Payload {
	t.Helper()
	cr := prismrecord.ZeroCarrierRecord(profile)
	cr.SectionDelta = carrierDelta
	entries(&cr)
	return prismrecord.Payload{Profile: profile, Frame: carrierFrame(t), Z1: 10, Xform: r3.Identity(),
		SectionDelta: carrierDelta, Carriers: &cr}
}

func junctionRecord(delta float64) func(*prismrecord.CarrierRecord) {
	return func(cr *prismrecord.CarrierRecord) {
		for l := range cr.Junction {
			for j := range cr.Junction[l] {
				cr.Junction[l][j] = delta
			}
		}
	}
}

func buildReceiver(t *testing.T, pp prismrecord.Payload) (*topo.Body, topo.Env) {
	t.Helper()
	env := displacedEnv()
	var b *topo.Body
	var err error
	if pp.Carriers != nil {
		b, err = prismrecord.BuildWithRecord(t.Context(), env, pp, *pp.Carriers)
	} else {
		b, err = prismrecord.Build(t.Context(), env, pp, freeform.NewFreeformWork())
	}
	require.NoError(t, err)
	env.Ref++
	return b, env
}

// verticalEdgesAt are the body's straight vertical edges whose section point
// is within 1e-6 of one of pts.
func verticalEdgesAt(t *testing.T, b *topo.Body, pts []sectionrecord.Point2) []*topo.Edge {
	t.Helper()
	var out []*topo.Edge
	for _, e := range topobuild.Edges(b) {
		if _, ok := e.Curve.(surfacegeom.Line3); !ok || e.Start == nil || e.End == nil {
			continue
		}
		a, c := e.Start.Position, e.End.Position
		if math.Abs(a.X-c.X) > 1e-9 || math.Abs(a.Y-c.Y) > 1e-9 {
			continue
		}
		for _, p := range pts {
			if math.Hypot(a.X-p.U, a.Y-p.V) < 1e-6 {
				out = append(out, e)
			}
		}
	}
	require.Len(t, out, len(pts))
	return out
}

func blendCarrier(t *testing.T, b *topo.Body, env topo.Env, edges []*topo.Edge, fillet bool, size float64) (brepmodify.Result, error) {
	t.Helper()
	if fillet {
		return brepmodify.Fillet(t.Context(), env, b, brepmodify.FilletCall{Sel: "roots", Edges: edges, R: size})
	}
	return brepmodify.Chamfer(t.Context(), env, b, brepmodify.ChamferCall{Sel: "roots", Edges: edges, D: size})
}

func payloadOf(t *testing.T, b *topo.Body) prismrecord.Payload {
	t.Helper()
	pp, ok := b.Payload.(prismrecord.Payload)
	require.True(t, ok, `the result is a prism`)
	return pp
}

func requireRefusal(t *testing.T, err error, row string) *brepmodify.CarrierRefusal {
	t.Helper()
	var refusal *brepmodify.CarrierRefusal
	require.ErrorAs(t, err, &refusal)
	require.ErrorIs(t, err, decaderr.ErrUnsupported)
	require.Equal(t, row, refusal.Row)
	require.Contains(t, err.Error(), `agent-docs/carrier-record-design.md`)
	return refusal
}

// TestCarrierFilletClaimD is §7.4's Claim D′: the two-root fillet of the
// tooth's roots on a receiver whose carriers are exact. It builds, matches
// the same filleted outline built at SectionDelta 0 within both bounds,
// records every connector arc with a zero entry and every foot with a tiny
// positive one (its residual r_f), and states a section displacement at
// least every loop's h.
func TestCarrierFilletClaimD(t *testing.T) {
	profile, roots := carrierTooth(0)
	recorded := recordedReceiver(t, profile, junctionRecord(carrierJunc))
	body, env := buildReceiver(t, recorded)
	edges := verticalEdgesAt(t, body, roots)
	res, err := blendCarrier(t, body, env, edges, true, 1)
	require.NoError(t, err)

	got := payloadOf(t, res.Body)
	require.Positive(t, got.SectionDelta)
	require.Less(t, got.SectionDelta, 1e-11)
	cr, valid, err := got.CarrierRecord(proofbound.NewWorkBudget(t.Context()))
	require.NoError(t, err)
	require.True(t, valid, `the result's record is valid`)
	require.GreaterOrEqual(t, got.SectionDelta, cr.HMax())

	// Two connector arcs, each a zero entry; the feet beside them are the
	// feet's residuals, tiny and positive.
	require.Len(t, got.BlendSegs, 1)
	require.Len(t, got.BlendSegs[0], 2)
	segs := got.Profile.Outer.Segments
	require.Len(t, segs, 6, `the loop gains one connector at each root`)
	for k := range got.BlendSegs[0] {
		_, isArc := segs[k].(sectionrecord.ArcSeg)
		require.True(t, isArc)
		require.Zero(t, cr.Segment[0][k], `Claim D′: the connector carries no displacement`)
		before := cr.Junction[0][(k+len(segs)-1)%len(segs)]
		after := cr.Junction[0][k]
		require.Positive(t, before+after, `a foot is charged its residual r_f`)
		require.Less(t, before, 1e-10)
		require.Less(t, after, 1e-10)
	}

	// The same outline built at SectionDelta 0 is the exact filleted part.
	plain := prismrecord.Payload{Profile: profile, Frame: carrierFrame(t), Z1: 10, Xform: r3.Identity()}
	pBody, pEnv := buildReceiver(t, plain)
	pRes, err := blendCarrier(t, pBody, pEnv, verticalEdgesAt(t, pBody, roots), true, 1)
	require.NoError(t, err)
	requireSameBody(t, res.Body, pRes.Body)
	require.Zero(t, payloadOf(t, pRes.Body).SectionDelta)
	v, bound := volumeOf(t, res.Body)
	pv, _ := volumeOf(t, pRes.Body)
	require.InDelta(t, pv, v, 1e-6)
	require.Positive(t, bound)

	// The section area carries the two corner fillets: each fills
	// (1 − π/4)·r² less the tooth-root wedge, so the outline holds more area
	// than the unfilleted one by a positive, bounded amount.
	plainV, _ := volumeOf(t, body)
	require.Greater(t, v, plainV-1e-6)
}

// TestCarrierChamferExactCorner is the chamfer of the same roots: the float
// chord at a corner whose carriers and junctions are exact carries only its
// feet's residuals.
func TestCarrierChamferExactCorner(t *testing.T) {
	profile, roots := carrierTooth(0)
	recorded := recordedReceiver(t, profile, func(cr *prismrecord.CarrierRecord) {})
	body, env := buildReceiver(t, recorded)
	res, err := blendCarrier(t, body, env, verticalEdgesAt(t, body, roots), false, 1)
	require.NoError(t, err)
	got := payloadOf(t, res.Body)
	cr, valid, err := got.CarrierRecord(proofbound.NewWorkBudget(t.Context()))
	require.NoError(t, err)
	require.True(t, valid)
	for k := range got.BlendSegs[0] {
		_, isLine := got.Profile.Outer.Segments[k].(sectionrecord.LineSeg)
		require.True(t, isLine)
		require.Zero(t, cr.Segment[0][k], `an all-zero chamfer corner takes the float chord`)
	}
	require.Positive(t, cr.Max())
	require.Less(t, cr.Max(), 1e-10)
}

// TestCarrierRotatedCopyCentre is §7.5's centre bound for a displaced
// carrier: a 17th-turn copy whose every entry is 1e-13 fillets, its centre
// displacement D is positive and below 1e-12 mm, and the result still
// matches the unrotated outline's volume.
func TestCarrierRotatedCopyCentre(t *testing.T) {
	profile, roots := carrierTooth(1.0 / 17)
	recorded := recordedReceiver(t, profile, func(cr *prismrecord.CarrierRecord) {
		for l := range cr.Segment {
			for j := range cr.Segment[l] {
				cr.Segment[l][j] = 1e-13
				cr.Junction[l][j] = 1e-13
			}
		}
	})
	recorded.SectionDelta = 1e-12
	recorded.Carriers.SectionDelta = 1e-12
	body, env := buildReceiver(t, recorded)
	res, err := blendCarrier(t, body, env, verticalEdgesAt(t, body, roots), true, 1)
	require.NoError(t, err)
	got := payloadOf(t, res.Body)
	cr, valid, err := got.CarrierRecord(proofbound.NewWorkBudget(t.Context()))
	require.NoError(t, err)
	require.True(t, valid)
	for k := range got.BlendSegs[0] {
		require.Positive(t, cr.Segment[0][k], `a displaced corner charges its centre displacement D`)
		require.Less(t, cr.Segment[0][k], 1e-12)
	}

	plain, plainRoots := carrierTooth(0)
	pBody, pEnv := buildReceiver(t, prismrecord.Payload{Profile: plain, Frame: carrierFrame(t), Z1: 10, Xform: r3.Identity()})
	pRes, err := blendCarrier(t, pBody, pEnv, verticalEdgesAt(t, pBody, plainRoots), true, 1)
	require.NoError(t, err)
	v, vb := volumeOf(t, res.Body)
	pv, pb := volumeOf(t, pRes.Body)
	require.LessOrEqual(t, math.Abs(v-pv), vb+pb+1e-9, `a rotated copy fillets to the same volume`)
}

// TestCarrierRefusalsFallThrough shows the dispatch of §7.1: a chamfer's Table
// CB refusal falls through to today's route, a fillet's is returned.
func TestCarrierRefusalsFallThrough(t *testing.T) {
	// Entries on a merged walk refuse CB0: two collinear lines with one
	// nonzero junction entry.
	p := func(u, v float64) sectionrecord.Point2 { return sectionrecord.Point2{U: u, V: v} }
	line := func(a, b sectionrecord.Point2) sectionrecord.CurveSegment {
		return sectionrecord.LineSeg{Start: a, End: b, TStart: 0, TEnd: 1}
	}
	profile := momentinput.Profile{Outer: sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{
		line(p(0, 0), p(5, 0)), line(p(5, 0), p(10, 0)), line(p(10, 0), p(10, 10)), line(p(10, 10), p(0, 10)), line(p(0, 10), p(0, 0)),
	}}}
	recorded := recordedReceiver(t, profile, func(cr *prismrecord.CarrierRecord) { cr.Junction[0][0] = carrierJunc })
	body, env := buildReceiver(t, recorded)
	edges := verticalEdgesAt(t, body, []sectionrecord.Point2{p(10, 0)})

	_, err := blendCarrier(t, body, env, edges, true, 1)
	requireRefusal(t, err, "CB0")
	requireGuard(t, "CB0", []string{"CB0"}, func() error {
		_, err := blendCarrier(t, body, env, edges, true, 1)
		return err
	})

	_, err = blendCarrier(t, body, env, edges, false, 1)
	require.Error(t, err)
	var refusal *brepmodify.CarrierRefusal
	require.False(t, errors.As(err, &refusal), `a chamfer's CB refusal falls through to RequireExactSection`)
	require.ErrorIs(t, err, decaderr.ErrUnsupported)
}

// polygonReceiver records an all-line section with every entry set to entry.
func polygonReceiver(t *testing.T, delta, entry float64, pts ...sectionrecord.Point2) prismrecord.Payload {
	t.Helper()
	var segs []sectionrecord.CurveSegment
	for i, a := range pts {
		segs = append(segs, sectionrecord.LineSeg{Start: a, End: pts[(i+1)%len(pts)], TStart: 0, TEnd: 1})
	}
	profile := momentinput.Profile{Outer: sectionrecord.LoopRecord{Segments: segs}}
	cr := prismrecord.UniformCarrierRecord(profile, entry)
	cr.SectionDelta = delta
	return prismrecord.Payload{Profile: profile, Frame: carrierFrame(t), Z1: 10, Xform: r3.Identity(),
		SectionDelta: delta, Carriers: &cr}
}

// requireGuard shows a Table CB row is the guard that refuses: with it on the
// blend is a refusal naming the row, and with it off the blend no longer
// is that refusal.
func requireGuard(t *testing.T, row string, off []string, build func() error) {
	t.Helper()
	err := build()
	requireRefusal(t, err, row)
	restore := brepmodify.DisableCarrierRows(off...)
	defer restore()
	err = build()
	if refusal, ok := errors.AsType[*brepmodify.CarrierRefusal](err); ok {
		require.NotEqual(t, row, refusal.Row, `row %s no longer refuses once disabled`, row)
	}
}

func square(l float64) []sectionrecord.Point2 {
	return []sectionrecord.Point2{{U: 0, V: 0}, {U: l, V: 0}, {U: l, V: l}, {U: 0, V: l}}
}

// TestCarrierCB1Turn: a corner whose turn is within the displacement of S4's
// threshold refuses, because the true corner may turn the other way.
func TestCarrierCB1Turn(t *testing.T) {
	turn := 1e-9 + 5e-14 // above S4's 1e-9, below tol + θ_A + θ_B + 2⁻⁴⁸
	pts := []sectionrecord.Point2{{U: 0, V: 0}, {U: 10, V: 0}, {U: 20, V: 10 * turn}, {U: 20, V: 10}, {U: 0, V: 10}}
	build := func() error {
		body, env := buildReceiver(t, polygonReceiver(t, carrierDelta, carrierJunc, pts...))
		_, err := blendCarrier(t, body, env, verticalEdgesAt(t, body, pts[1:2]), true, 1)
		return err
	}
	requireGuard(t, "CB1", []string{"CB1"}, build)
}

// TestCarrierCB7Floor: a displacement above half the section's contact floor
// refuses, since the audit's verdicts no longer transfer.
func TestCarrierCB7Floor(t *testing.T) {
	pts := square(10)
	build := func() error {
		body, env := buildReceiver(t, polygonReceiver(t, 3e-8, 1e-8, pts...))
		_, err := blendCarrier(t, body, env, verticalEdgesAt(t, body, pts[:1]), true, 1)
		return err
	}
	requireGuard(t, "CB7", []string{"CB7"}, build)
}

// TestCarrierCentreLemmaT is CB2: two circles whose offsets are nearly
// tangent cross where Lemma T cannot place the denoted crossing, and a
// healthy crossing gets a positive bounded D.
func TestCarrierCentreLemmaT(t *testing.T) {
	p := func(u, v float64) sectionrecord.Point2 { return sectionrecord.Point2{U: u, V: v} }
	arc := func(c, s, e sectionrecord.Point2) sectionrecord.CurveSegment {
		return sectionrecord.ArcSeg{Center: c, Start: s, End: e, TStart: 0, TEnd: 1}
	}
	// Unit circles about (0, 0) and (2, 0); at r = 0.5 their concave offsets
	// are circles of radius 1.5 that cross at (1, √1.25) at a wide angle.
	a, b := arc(p(0, 0), p(1, 0), p(0, 1)), arc(p(2, 0), p(1, 0), p(2, 1))
	d, ok := brepmodify.CarrierCentre(a, b, [2]float64{1e-13, 1e-13}, [2]bool{true, true}, -1,
		[2]float64{1, math.Sqrt(1.25)}, 0.5)
	require.True(t, ok)
	require.Positive(t, d)
	require.Less(t, d, 1e-9)

	// Moving the second centre to 3 − 2e-9 makes the two offsets nearly
	// externally tangent: they cross at an angle of about 7e-5, which the
	// ball Lemma T needs (about 1e-4 wide at ε = 1e-9) turns the tangents
	// through by more than the angle itself.
	e := arc(p(3-2e-9, 0), p(2-2e-9, 0), p(3-2e-9, 1))
	x := 1.5 - 1e-9
	_, ok = brepmodify.CarrierCentre(a, e, [2]float64{1e-9, 1e-9}, [2]bool{true, true}, -1,
		[2]float64{x, math.Sqrt(2.25 - x*x)}, 0.5)
	require.False(t, ok, `a near-tangent crossing has no Lemma T acceptance`)
}

// dFlatReceiver is a disc of radius 5 cut by a chord at x = 5 − 1.5e-9, its
// arc recorded as one whole ArcSeg with displacement eps.
func dFlatReceiver(t *testing.T, eps float64) (prismrecord.Payload, []sectionrecord.Point2) {
	t.Helper()
	const radius = 5.0
	c := radius - 1.5e-9
	y := math.Sqrt(radius*radius - c*c)
	top, bottom := sectionrecord.Point2{U: c, V: y}, sectionrecord.Point2{U: c, V: -y}
	profile := momentinput.Profile{Outer: sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{
		sectionrecord.ArcSeg{Start: top, End: bottom, TStart: 0, TEnd: 1},
		sectionrecord.LineSeg{Start: bottom, End: top, TStart: 0, TEnd: 1},
	}}}
	cr := prismrecord.ZeroCarrierRecord(profile)
	cr.SectionDelta = 1e-7
	cr.Segment[0][0] = eps
	return prismrecord.Payload{Profile: profile, Frame: carrierFrame(t), Z1: 10, Xform: r3.Identity(),
		SectionDelta: 1e-7, Carriers: &cr}, []sectionrecord.Point2{top, bottom}
}

// TestCarrierCB6InwardCircle: an inward circular carrier whose offset radius
// sits just above S5's tolerance but below the carrier's displacement
// refuses. Lemma T's ball (CB2) already needs the offset radius above twice
// ε + D, so CB6 is shown on its own with CB2 and CB3 off.
func TestCarrierCB6InwardCircle(t *testing.T) {
	recorded, corners := dFlatReceiver(t, 1e-8)
	build := func() error {
		body, env := buildReceiver(t, recorded)
		_, err := blendCarrier(t, body, env, verticalEdgesAt(t, body, corners[:1]), true, 5-2e-9)
		return err
	}
	restore := brepmodify.DisableCarrierRows("CB2", "CB3")
	err := build()
	restore()
	refusal := requireRefusal(t, err, "CB6")
	require.Contains(t, refusal.Reason, `offset radius`)
	requireGuard(t, "CB2", []string{"CB2"}, build)
}

// TestCarrierCB6ReadsCentreDisplacement: CB6 compares the offset radius R − r
// with tol + ε + D, so a radius that clears tol + ε alone still refuses once
// the centre's displacement D joins the right-hand side (§7.6).
func TestCarrierCB6ReadsCentreDisplacement(t *testing.T) {
	t.Parallel()
	const tol = sectionaudit.Tolerance
	circle := sectionrecord.CircleSeg{Radius: units.Millimeters(5), CCW: true, TEnd: 1}
	const eps, d = 1e-8, 3e-9
	r := 5 - 1.2e-8 // R − r = 1.2e-8 clears tol + ε = 1.1e-8, not tol + ε + D = 1.4e-8
	off, need, ok := brepmodify.CarrierCB6Margin(circle, eps, true, 1, r, d)
	require.True(t, ok)
	require.InDelta(t, 1.2e-8, off, 1e-14)
	require.InDelta(t, tol+eps+d, need, 1e-15)
	require.False(t, off > need, `D in the margin refuses this radius`)
	_, withoutD, ok := brepmodify.CarrierCB6Margin(circle, eps, true, 1, r, 0)
	require.True(t, ok)
	require.InDelta(t, tol+eps, withoutD, 1e-15)
	require.True(t, off > withoutD, `without D the radius would pass`)
	// An exact circle (ε = 0) beside a displaced carrier takes tol + D.
	_, need, ok = brepmodify.CarrierCB6Margin(circle, 0, true, 1, r, d)
	require.True(t, ok)
	require.InDelta(t, tol+d, need, 1e-15)
	// A radius at or above R leaves no positive offset radius.
	_, _, ok = brepmodify.CarrierCB6Margin(circle, eps, true, 1, 5, d)
	require.False(t, ok)
}

// TestCarrierCB7PairsKeptAndRewrittenLoops: CB7's certified contact test covers
// every pair of segments of the result, a kept (cloned) loop against a
// rewritten one included. The record is the result section itself: an outer
// square and a triangular hole whose tip stands g from the outer wall. The
// wall's loop and the hole's loop share no vertex, so only the pair test sees
// the thin web, which the float S7 inside the blend would also refuse at this
// size (both read the same floor).
func TestCarrierCB7PairsKeptAndRewrittenLoops(t *testing.T) {
	p := func(u, v float64) sectionrecord.Point2 { return sectionrecord.Point2{U: u, V: v} }
	web := func(g float64) prismrecord.CarrierRecord {
		outer := polygonReceiver(t, carrierDelta, carrierJunc, p(0, 0), p(10, 0), p(10, 10), p(0, 10))
		profile := outer.Profile
		// a clockwise triangular hole whose tip (10 − g, 5) faces the right wall
		hole := []sectionrecord.Point2{p(10-g, 5), p(5, 3), p(5, 7)}
		var segs []sectionrecord.CurveSegment
		for i, a := range hole {
			segs = append(segs, sectionrecord.LineSeg{Start: a, End: hole[(i+1)%len(hole)], TStart: 0, TEnd: 1})
		}
		profile.Holes = []sectionrecord.LoopRecord{{Segments: segs}}
		cr := prismrecord.UniformCarrierRecord(profile, carrierJunc)
		cr.SectionDelta = carrierDelta
		return cr
	}
	budget := func() *proofbound.WorkBudget { return proofbound.NewWorkBudget(t.Context()) }
	// F for this 10 × 10 section is about 1.4e-8.
	err := brepmodify.CarrierAuditTransfer(budget(), web(1e-9))
	refusal := requireRefusal(t, err, "CB7")
	require.Contains(t, refusal.Reason, `loop 0`)
	require.Contains(t, refusal.Reason, `loop 1`)
	require.Contains(t, refusal.Reason, `contact floor`)
	require.NoError(t, brepmodify.CarrierAuditTransfer(budget(), web(1e-3)))

	restore := brepmodify.DisableCarrierRows("CB7")
	defer restore()
	require.NoError(t, brepmodify.CarrierAuditTransfer(budget(), web(1e-9)), `CB7 off lets the thin web through`)
}

// TestCarrierCB3Roots: the chord's two offset roots lie almost equally far
// from the corner, so the displacement could swap which one is nearest.
func TestCarrierCB3Roots(t *testing.T) {
	recorded, corners := dFlatReceiver(t, 1e-8)
	recorded.Carriers.Junction[0][1] = 1e-12 // the corner's own displacement outweighs the roots' difference
	build := func() error {
		body, env := buildReceiver(t, recorded)
		_, err := blendCarrier(t, body, env, verticalEdgesAt(t, body, corners[:1]), true, 5-2e-9)
		return err
	}
	restore := brepmodify.DisableCarrierRows("CB2")
	err := build()
	restore()
	refusal := requireRefusal(t, err, "CB3")
	require.Contains(t, refusal.Reason, `fillet centres`)
	restore = brepmodify.DisableCarrierRows("CB2", "CB3")
	defer restore()
	err = build()
	if again, ok := errors.AsType[*brepmodify.CarrierRefusal](err); ok {
		require.NotEqual(t, "CB3", again.Row, `CB3 no longer refuses once disabled`)
	}
}

// TestCarrierCB4Sense: a fillet radius within the feet's displacement could
// flip the connector arc's walk sense.
func TestCarrierCB4Sense(t *testing.T) {
	pts := square(10)
	build := func() error {
		body, env := buildReceiver(t, polygonReceiver(t, carrierDelta, carrierJunc, pts...))
		_, err := blendCarrier(t, body, env, verticalEdgesAt(t, body, pts[1:2]), true, 2e-13)
		return err
	}
	requireGuard(t, "CB4", []string{"CB4"}, build)
}

// TestCarrierChamferDisplacedFeet: a chamfer of the rotated copy's roots
// states each foot from §7.5 (ρ_f + δ_J on a line, Claim C's transport on the
// circle) and a connector entry at least every foot's.
func TestCarrierChamferDisplacedFeet(t *testing.T) {
	profile, roots := carrierTooth(1.0 / 17)
	recorded := recordedReceiver(t, profile, func(cr *prismrecord.CarrierRecord) {
		for l := range cr.Segment {
			for j := range cr.Segment[l] {
				cr.Segment[l][j] = 1e-13
				cr.Junction[l][j] = 1e-13
			}
		}
	})
	recorded.SectionDelta = 1e-12
	recorded.Carriers.SectionDelta = 1e-12
	body, env := buildReceiver(t, recorded)
	res, err := blendCarrier(t, body, env, verticalEdgesAt(t, body, roots), false, 1)
	require.NoError(t, err)
	got := payloadOf(t, res.Body)
	cr, valid, err := got.CarrierRecord(proofbound.NewWorkBudget(t.Context()))
	require.NoError(t, err)
	require.True(t, valid)
	segs := got.Profile.Outer.Segments
	for k := range got.BlendSegs[0] {
		before := cr.Junction[0][(k+len(segs)-1)%len(segs)]
		after := cr.Junction[0][k]
		require.GreaterOrEqual(t, before, 1e-13, `a foot carries at least its junction's displacement`)
		require.GreaterOrEqual(t, after, 1e-13)
		require.Greater(t, cr.Segment[0][k], 1e-13, `the chord carries its feet's displacement`)
		require.LessOrEqual(t, cr.Segment[0][k], math.Max(before, after))
		require.Less(t, cr.Segment[0][k], 1e-10)
	}
}

// TestCarrierClockwiseArcEntries: a hole loop's clockwise arc is recorded
// with Start at the walk's end (offset2d.ArcSegment), so blending only its
// walk-start end puts ε = 2·ε_src + d at ArcSeg.Start, the untouched end, and
// the radial residual ρ on the blended end beside ArcSeg.End.
func TestCarrierClockwiseArcEntries(t *testing.T) {
	const radius, c = 5.0, 3.0
	y := math.Sqrt(radius*radius - c*c)
	top, bottom := sectionrecord.Point2{U: c, V: y}, sectionrecord.Point2{U: c, V: -y}
	outer := polygonReceiver(t, carrierDelta, 0, sectionrecord.Point2{U: -20, V: -20}, sectionrecord.Point2{U: 20, V: -20},
		sectionrecord.Point2{U: 20, V: 20}, sectionrecord.Point2{U: -20, V: 20})
	profile := outer.Profile
	profile.Holes = []sectionrecord.LoopRecord{{Segments: []sectionrecord.CurveSegment{
		sectionrecord.LineSeg{Start: top, End: bottom, TStart: 0, TEnd: 1},
		sectionrecord.ArcSeg{Start: top, End: bottom, TStart: 1, TEnd: 0},
	}}}
	cr := prismrecord.ZeroCarrierRecord(profile)
	cr.SectionDelta = carrierDelta
	junctionRecord(carrierJunc)(&cr)
	cr.Segment[1][1] = 1e-13
	recorded := prismrecord.Payload{Profile: profile, Frame: carrierFrame(t), Z1: 10, Xform: r3.Identity(),
		SectionDelta: carrierDelta, Carriers: &cr}
	body, env := buildReceiver(t, recorded)
	res, err := blendCarrier(t, body, env, verticalEdgesAt(t, body, []sectionrecord.Point2{bottom}), true, 0.5)
	require.NoError(t, err)
	got := payloadOf(t, res.Body)
	rec, valid, err := got.CarrierRecord(proofbound.NewWorkBudget(t.Context()))
	require.NoError(t, err)
	require.True(t, valid)
	found := false
	for k, seg := range got.Profile.Holes[0].Segments {
		arc, ok := seg.(sectionrecord.ArcSeg)
		if !ok || arc.TStart != 1 || arc.TEnd != 0 || arc.Start != top {
			continue
		}
		found = true
		// ArcSeg.Start is the untouched walk end: its displacement is the
		// junction's 1e-13 and ε = 2·1e-13 + that.
		require.InDelta(t, 3e-13, rec.Segment[1][k], 2e-14, `ε = 2·ε_src + d at ArcSeg.Start`)
	}
	require.True(t, found, `the clockwise arc survives with its untouched end as ArcSeg.Start`)
}

// TestCarrierCB7AreaEnclosure: CB7's area row admits only a loop whose signed
// area enclosure, with its junction gaps and displacement area, clears zero.
// A 10 x 1e-5 strip stands 1e6 from the area anchor, so a 1e-10 junction gap
// moves its area by about 1e-4 through the gap's lever arm. The float area
// of 1e-4 clears the displacement area alone but not the enclosure.
func TestCarrierCB7AreaEnclosure(t *testing.T) {
	const (
		at = 1e6
		w  = 1e-5
	)
	p := func(u, v float64) sectionrecord.Point2 { return sectionrecord.Point2{U: at + u, V: at + v} }
	line := func(a, b sectionrecord.Point2) sectionrecord.CurveSegment {
		return sectionrecord.LineSeg{Start: a, End: b, TStart: 0, TEnd: 1}
	}
	record := func(gap float64) prismrecord.CarrierRecord {
		segs := []sectionrecord.CurveSegment{
			line(p(0, 0), p(10, 0)), line(p(10+gap, 0), p(10, w)), line(p(10, w), p(0, w)), line(p(0, w), p(0, 0)),
		}
		profile := momentinput.Profile{Outer: sectionrecord.LoopRecord{Segments: segs}}
		cr := prismrecord.UniformCarrierRecord(profile, carrierJunc)
		cr.SectionDelta = carrierDelta
		return cr
	}
	budget := func() *proofbound.WorkBudget { return proofbound.NewWorkBudget(t.Context()) }
	require.NoError(t, brepmodify.CarrierAuditTransfer(budget(), record(0)))
	err := brepmodify.CarrierAuditTransfer(budget(), record(1e-10))
	refusal := requireRefusal(t, err, "CB7")
	require.Contains(t, refusal.Reason, `includes zero`)
	area, err := sectionaudit.LoopSignedArea(budget(), record(1e-10).Profile.Outer)
	require.NoError(t, err)
	require.Greater(t, math.Abs(area), 5e-5, `the float area alone would have admitted the loop`)
}

// TestCarrierProductDownF: the denominator R·(R−ε) of a chamfer foot's step
// term is rounded down, so the quotient rounded up never understates the
// exact step. A product the float multiply rounds up shows the difference.
func TestCarrierProductDownF(t *testing.T) {
	exact := func(a, b float64) *big.Rat {
		return new(big.Rat).Mul(new(big.Rat).SetFloat64(a), new(big.Rat).SetFloat64(b))
	}
	roundedUp := 0
	for i := range 200 {
		a := 20 + float64(i)*0.0137
		b := math.Nextafter(a, 0) - 1e-9*float64(i+1)
		got := brepmodify.ProductDownF(a, b)
		x := exact(a, b)
		require.LessOrEqual(t, new(big.Rat).SetFloat64(got).Cmp(x), 0, `a=%v b=%v: %v exceeds the exact product`, a, b, got)
		require.Greater(t, new(big.Rat).SetFloat64(math.Nextafter(got, math.Inf(1))).Cmp(x), 0, `a=%v b=%v: %v is not tight`, a, b, got)
		if new(big.Rat).SetFloat64(a*b).Cmp(x) > 0 {
			roundedUp++
			require.Less(t, got, a*b, `the float product is above the exact one, so the bound is below it`)
		}
	}
	require.NotZero(t, roundedUp, `no sampled product rounds up`)
	require.Zero(t, brepmodify.ProductDownF(math.Inf(1), 2))
}

// TestCarrierConnectorEntryReadsEmittedStart: the connector's ε is
// D + ||Start − ĉ| − r| with Start the emitted ArcSeg's own Start.
// offset2d.ArcSegment stores a clockwise arc reversed (Start = f̂B), and the
// reflex corner of an L-shaped section fillets through exactly such a
// connector. Moving f̂A by 1e-3 leaves the arc's Start alone, so the entry
// stays at D; reading f̂A would add the 1e-3.
func TestCarrierConnectorEntryReadsEmittedStart(t *testing.T) {
	p := func(u, v float64) sectionrecord.Point2 { return sectionrecord.Point2{U: u, V: v} }
	pts := []sectionrecord.Point2{p(0, 0), p(10, 0), p(10, 5), p(5, 5), p(5, 10), p(0, 10)}
	var segs []sectionrecord.CurveSegment
	for i, a := range pts {
		segs = append(segs, sectionrecord.LineSeg{Start: a, End: pts[(i+1)%len(pts)], TStart: 0, TEnd: 1})
	}
	profile := momentinput.Profile{Outer: sectionrecord.LoopRecord{Segments: segs}}
	const ci = 3 // the reflex corner (5, 5)
	budget := func() *proofbound.WorkBudget { return proofbound.NewWorkBudget(t.Context()) }
	conn, eps, err := brepmodify.CarrierFilletConnEps(budget(), profile, ci, 1e-9, 1e-10, 1, nil)
	require.NoError(t, err)
	arc, ok := conn.(sectionrecord.ArcSeg)
	require.True(t, ok)
	require.Equal(t, 1.0, arc.TStart, `the reflex corner's connector is clockwise, stored reversed`)
	require.Positive(t, eps)
	require.Less(t, eps, 1e-8)

	moved, epsMoved, err := brepmodify.CarrierFilletConnEps(budget(), profile, ci, 1e-9, 1e-10, 1, &[2]float64{1e-3, 0})
	require.NoError(t, err)
	require.Equal(t, conn, moved)
	require.Equal(t, eps, epsMoved, `the foot f̂A is not the connector's Start`)
}

// seamProfile is a unit square whose loop has an open seam of g between the
// end of its first segment and the start of its second, which is reversed
// when flip is set (then it walks End to Start).
func seamProfile(g float64, flip bool) momentinput.Profile {
	p := func(u, v float64) sectionrecord.Point2 { return sectionrecord.Point2{U: u, V: v} }
	second := sectionrecord.LineSeg{Start: p(1+g, 0), End: p(1, 1), TStart: 0, TEnd: 1}
	if flip {
		second = sectionrecord.LineSeg{Start: p(1, 1), End: p(1+g, 0), TStart: 1, TEnd: 0}
	}
	return momentinput.Profile{Outer: sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{
		sectionrecord.LineSeg{Start: p(0, 0), End: p(1, 0), TStart: 0, TEnd: 1},
		second,
		sectionrecord.LineSeg{Start: p(1, 1), End: p(0, 1), TStart: 0, TEnd: 1},
		sectionrecord.LineSeg{Start: p(0, 1), End: p(0, 0), TStart: 0, TEnd: 1},
	}}}
}

// TestCarrierCloseSeams: a seam snap touches only a rewritten loop, compares
// the read ends in walk order (a reversed segment's start is its End field)
// and states how far it moved what it moved.
func TestCarrierCloseSeams(t *testing.T) {
	const g = 1e-14
	t.Run("kept loop untouched", func(t *testing.T) {
		in := seamProfile(g, false)
		got, moves := brepmodify.CloseSeams(in, []bool{false})
		require.Equal(t, in, got)
		require.Equal(t, []brepmodify.SeamMove{{}, {}, {}, {}}, moves[0])
	})
	t.Run("rewritten loop snaps and charges", func(t *testing.T) {
		got, moves := brepmodify.CloseSeams(seamProfile(g, false), []bool{true})
		seg := got.Outer.Segments[1].(sectionrecord.LineSeg)
		require.Equal(t, 1.0, seg.Start.U)
		require.InDelta(t, g, moves[0][1].Read, 1e-16)
		require.InDelta(t, g, moves[0][1].Carrier, 1e-16)
		require.Zero(t, moves[0][0])
	})
	t.Run("reversed segment moves its End field", func(t *testing.T) {
		got, moves := brepmodify.CloseSeams(seamProfile(g, true), []bool{true})
		seg := got.Outer.Segments[1].(sectionrecord.LineSeg)
		require.Equal(t, 1.0, seg.End.U, `the walk starts at End`)
		require.Equal(t, 1.0, seg.Start.U)
		require.InDelta(t, g, moves[0][1].Read, 1e-16)
	})
	t.Run("a gap beyond rounding stays", func(t *testing.T) {
		in := seamProfile(1e-6, false)
		got, moves := brepmodify.CloseSeams(in, []bool{true})
		require.Equal(t, in, got)
		require.Zero(t, moves[0][1])
	})
}

// TestCarrierSeamChargeReachesEntries: the segment a seam snap moved carries
// the move in its own entry and its junction's entry.
func TestCarrierSeamChargeReachesEntries(t *testing.T) {
	const g = 1e-14
	prof := seamProfile(g, false)
	snapped, moves := brepmodify.CloseSeams(prof, []bool{true})
	cr := prismrecord.UniformCarrierRecord(prof, carrierJunc)
	cr.SectionDelta = carrierDelta
	got, err := brepmodify.CarrierKeptRecord(proofbound.NewWorkBudget(t.Context()), cr, prof, snapped, moves)
	require.NoError(t, err)
	require.Greater(t, got.Segment[0][1], cr.Segment[0][1]+0.9*g, `the moved segment's entry carries the move`)
	require.Greater(t, got.Junction[0][0], cr.Junction[0][0]+0.9*g, `the seam's junction entry carries the move`)
	require.InDelta(t, cr.Segment[0][2], got.Segment[0][2], 1e-27, `an untouched segment keeps its entry`)
}

// TestCarrierSeamChargesTheCornerLeg: a seam snap between two nearly
// parallel lines whose gap runs diagonally moves the junction from a path
// that closes through the lines' corner, about a thousand gaps away, to the
// snapped point; the junction entry carries that leg, not only the gap.
func TestCarrierSeamChargesTheCornerLeg(t *testing.T) {
	const g = 1e-14
	p := func(u, v float64) sectionrecord.Point2 { return sectionrecord.Point2{U: u, V: v} }
	prof := momentinput.Profile{Outer: sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{
		sectionrecord.LineSeg{Start: p(0, 0), End: p(1, 0), TStart: 0, TEnd: 1},
		sectionrecord.LineSeg{Start: p(1+g, g), End: p(2, 2e-3), TStart: 0, TEnd: 1},
		sectionrecord.LineSeg{Start: p(2, 2e-3), End: p(0, 1), TStart: 0, TEnd: 1},
		sectionrecord.LineSeg{Start: p(0, 1), End: p(0, 0), TStart: 0, TEnd: 1},
	}}}
	snapped, moves := brepmodify.CloseSeams(prof, []bool{true})
	require.NotZero(t, moves[0][1].Read, "the diagonal gap is snapped")
	cr := prismrecord.UniformCarrierRecord(prof, carrierJunc)
	cr.SectionDelta = carrierDelta
	got, err := brepmodify.CarrierKeptRecord(proofbound.NewWorkBudget(t.Context()), cr, prof, snapped, moves)
	require.NoError(t, err)
	// The corner of y = 0 with the line through (1+g, g) and (2, 2e-3).
	slope := (2e-3 - g) / (1 - g) // the legs stay within the evaluator's 1024-chord reach
	cornerU := 1 + g - g/slope
	leg := math.Abs(cornerU - 1)
	require.Greater(t, leg, 100*g)
	require.GreaterOrEqual(t, got.Junction[0][0], cr.Junction[0][0]+0.9*leg, "the junction entry carries the corner leg")
}

// TestCarrierCB5WidenedCutbacksRefuse: two corner blends that consume a wall
// by the S6 margin only once CB5 widens their cutbacks are a Table CB row's
// refusal, not an audit error. A chamfer's refusal falls through to
// RequireExactSection like every other CB row, and a fillet's is returned.
func TestCarrierCB5WidenedCutbacksRefuse(t *testing.T) {
	const size = 5 - 5e-7 // the bottom wall keeps 1e-6, above S6's 1e-8 margin and the contact floor
	pts := square(10)
	recorded := polygonReceiver(t, 3e-7, 3e-7, pts...) // each corner's displacement widens its cutback by about 6e-7
	body, env := buildReceiver(t, recorded)
	edges := verticalEdgesAt(t, body, pts[:2])
	for _, fillet := range []bool{true, false} {
		_, err := blendCarrier(t, body, env, edges, fillet, size)
		require.Error(t, err)
		var refusal *brepmodify.CarrierRefusal
		if fillet {
			refusal = requireRefusal(t, err, "CB5")
			require.Contains(t, refusal.Reason, `widens a cutback`)
			continue
		}
		require.False(t, errors.As(err, &refusal), `a chamfer's CB5 refusal falls through: %v`, err)
		require.Contains(t, err.Error(), `a receiver whose recorded section is the section it denotes only`, `RequireExactSection's refusal`)
		require.ErrorIs(t, err, decaderr.ErrUnsupported)
	}
	restore := brepmodify.DisableCarrierRows("CB5")
	defer restore()
	_, err := blendCarrier(t, body, env, edges, true, size)
	if refusal, ok := errors.AsType[*brepmodify.CarrierRefusal](err); ok {
		require.NotEqual(t, "CB5", refusal.Row)
	}
}

// TestCarrierArcReadEndEntry: a hole loop's clockwise arc whose End (the walk
// start) sits off Start's radius by more than rounding is blended at its Start
// end only. The untouched End side reads Start's radius on End's bearing, so
// its junction entry is at least the receiver's entry plus the exact distance
// between the receiver's and the emitted read ends, |R_emit − R_recv| where the
// centre and End are shared (agent-docs/carrier-record-design.md §7.7, "Arc
// read ends"). A walk-end allowance alone is zero there.
func TestCarrierArcReadEndEntry(t *testing.T) {
	const radius, c, off = 5.0, 3.0, 4e-15
	y := math.Sqrt(radius*radius - c*c)
	top, bottom := sectionrecord.Point2{U: c, V: y}, sectionrecord.Point2{U: c, V: -y - off}
	outer := polygonReceiver(t, carrierDelta, 0, sectionrecord.Point2{U: -20, V: -20}, sectionrecord.Point2{U: 20, V: -20},
		sectionrecord.Point2{U: 20, V: 20}, sectionrecord.Point2{U: -20, V: 20})
	profile := outer.Profile
	profile.Holes = []sectionrecord.LoopRecord{{Segments: []sectionrecord.CurveSegment{
		sectionrecord.LineSeg{Start: top, End: bottom, TStart: 0, TEnd: 1},
		sectionrecord.ArcSeg{Start: top, End: bottom, TStart: 1, TEnd: 0},
	}}}
	cr := prismrecord.ZeroCarrierRecord(profile)
	cr.SectionDelta = carrierDelta
	junctionRecord(carrierJunc)(&cr)
	recorded := prismrecord.Payload{Profile: profile, Frame: carrierFrame(t), Z1: 10, Xform: r3.Identity(),
		SectionDelta: carrierDelta, Carriers: &cr}
	body, env := buildReceiver(t, recorded)
	// The corner at top is the arc's Start end; bottom is its untouched End.
	res, err := blendCarrier(t, body, env, verticalEdgesAt(t, body, []sectionrecord.Point2{top}), true, 0.5)
	require.NoError(t, err)
	got := payloadOf(t, res.Body)
	rec, valid, err := got.CarrierRecord(proofbound.NewWorkBudget(t.Context()))
	require.NoError(t, err)
	require.True(t, valid)
	radiusOf := func(arc sectionrecord.ArcSeg) *big.Float {
		du := new(big.Float).SetPrec(300).Sub(new(big.Float).SetFloat64(arc.Start.U), new(big.Float).SetFloat64(arc.Center.U))
		dv := new(big.Float).SetPrec(300).Sub(new(big.Float).SetFloat64(arc.Start.V), new(big.Float).SetFloat64(arc.Center.V))
		sq := new(big.Float).SetPrec(300).Add(new(big.Float).SetPrec(300).Mul(du, du), new(big.Float).SetPrec(300).Mul(dv, dv))
		return new(big.Float).SetPrec(300).Sqrt(sq)
	}
	recvR := radiusOf(profile.Holes[0].Segments[1].(sectionrecord.ArcSeg))
	found := false
	for k, seg := range got.Profile.Holes[0].Segments {
		arc, ok := seg.(sectionrecord.ArcSeg)
		if !ok || arc.TStart != 1 || arc.TEnd != 0 || arc.End != bottom {
			continue
		}
		found = true
		gap, _ := new(big.Float).SetPrec(300).Sub(radiusOf(arc), recvR).Abs(new(big.Float).SetPrec(300).Sub(radiusOf(arc), recvR)).Float64()
		// The junction before the arc is the one at its End (walk start).
		n := len(got.Profile.Holes[0].Segments)
		entry := rec.Junction[1][(k+n-1)%n]
		require.GreaterOrEqual(t, entry, carrierJunc+gap, `the entry carries the exact read-end distance`)
		require.GreaterOrEqual(t, entry, carrierJunc+circularbounds.ArcRadialResidualUpper(arc),
			`and the new segment's own residual`)
	}
	require.True(t, found)
}
