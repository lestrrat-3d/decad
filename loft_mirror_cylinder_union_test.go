package decad

import (
	"context"
	"errors"
	"fmt"
	"math"
	"slices"
	"sync/atomic"
	"testing"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/loftcylinder"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// teethFixture is the mirrored Union's operands as the chamfer apitest builds
// them: a centered cylinder blank and the lower tooth pair with two copies
// turned about Z.
type teethFixture struct {
	d     *Document
	blank *Body
	pairs []*Body
}

func newTeethFixture(t *testing.T) teethFixture {
	t.Helper()
	d := New()
	world := sketch.NewWorld()
	middleFrame, err := r3.NewFrame(r3.Vec{Z: 1}, r3.Vec{X: 1}, r3.Vec{Y: 1})
	require.NoError(t, err)
	middlePlane, err := world.CreatePlaneFromFrame(middleFrame)
	require.NoError(t, err)
	bottomSketch, bottomProfile := teethSection(t, world, world.XY(), 0)
	middleSketch, middleProfile := teethSection(t, world, middlePlane, 0.03)
	lower, err := Loft(t.Context(), d, bottomSketch, bottomProfile, middleSketch, middleProfile)
	require.NoError(t, err)
	upper, err := MirroredCopy(t.Context(), lower, MirrorFrame{Frame: middleFrame})
	require.NoError(t, err)
	pair, err := Union(t.Context(), lower, upper)
	require.NoError(t, err)
	pairs := []*Body{pair}
	for i := 1; i <= 2; i++ {
		turn, err := r3.Rotation(r3.Vec{Z: 1}, units.Degrees(float64(45*i)))
		require.NoError(t, err)
		turned, err := Copy(t.Context(), pair, WithCopyPlacement(turn))
		require.NoError(t, err)
		pairs = append(pairs, turned)
	}
	ws := sketch.NewWorld()
	plane, err := ws.CreateOffsetPlane(ws.XY(), 0)
	require.NoError(t, err)
	s, err := ws.CreateSketch(plane)
	require.NoError(t, err)
	center := s.CreatePoint(0, 0)
	s.Fix(center)
	s.CreateCircle(center, 4)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	blank, err := Extrude(t.Context(), d, s, s.Profiles()[0], Distance{D: units.Millimeters(2), Dir: Along})
	require.NoError(t, err)
	return teethFixture{d: d, blank: blank, pairs: pairs}
}

func teethSection(t *testing.T, world *sketch.World, plane *sketch.Plane, turn float64) (*sketch.Sketch, *sketch.Profile) {
	t.Helper()
	s, err := world.CreateSketch(plane)
	require.NoError(t, err)
	center := s.CreatePoint(0, 0)
	s.Fix(center)
	polar := func(radius, angle float64) *sketch.Point {
		point := s.CreatePoint(radius*math.Cos(angle+turn), radius*math.Sin(angle+turn))
		s.Fix(point)
		return point
	}
	rightRoot, leftRoot := polar(4, -0.12), polar(4, 0.12)
	right := make([]*sketch.Point, 3)
	left := make([]*sketch.Point, 3)
	for i := range right {
		radius := 5 + float64(i)
		angle := 0.09 - 0.02*float64(i)
		right[i] = polar(radius, -angle)
		left[i] = polar(radius, angle)
	}
	s.CreateLine(rightRoot, right[0])
	_, err = s.CreateFitSpline(right...)
	require.NoError(t, err)
	s.CreateArc(center, right[len(right)-1], left[len(left)-1])
	_, err = s.CreateFitSpline(left...)
	require.NoError(t, err)
	s.CreateLine(left[0], leftRoot)
	s.CreateArc(center, rightRoot, leftRoot)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	profiles := s.Profiles()
	require.Len(t, profiles, 1)
	return s, profiles[0]
}

// pairsOutcome is what one call of loftMirrorCylinderPairs answers.
type pairsOutcome struct {
	body    *Body
	matched bool
	err     error
}

func (o pairsOutcome) requireSame(t *testing.T, want pairsOutcome) {
	t.Helper()
	require.Equal(t, want.matched, o.matched)
	if want.err != nil {
		require.EqualError(t, o.err, want.err.Error())
		if sentinel := errors.Unwrap(want.err); sentinel != nil {
			require.ErrorIs(t, o.err, sentinel)
		}
	} else {
		require.NoError(t, o.err)
	}
	require.Equal(t, want.body == nil, o.body == nil)
	if want.body == nil {
		return
	}
	wantPayload, ok := want.body.n.Payload.(facetedPayload)
	require.True(t, ok)
	gotPayload, ok := o.body.n.Payload.(facetedPayload)
	require.True(t, ok)
	require.Equal(t, wantPayload.Verts, gotPayload.Verts)
	require.Equal(t, wantPayload.Tris, gotPayload.Tris)
	require.Equal(t, want.body.n.Volume, o.body.n.Volume)
	require.Equal(t, want.body.n.Area, o.body.n.Area)
	require.Equal(t, want.body.n.Centroid, o.body.n.Centroid)
	require.Equal(t, want.body.n.Bounds, o.body.n.Bounds)
	require.Equal(t, wantPayload.VertexBound, gotPayload.VertexBound)
	require.Equal(t, wantPayload.MeshBound, gotPayload.MeshBound)
	// Every other payload field the old loop's result carries (provenance
	// groups src and MirrorCapGroups, source mapping, the proof fields, the
	// Join records) by deep comparison. The cylinder Join record holds the lower
	// half body, whose lazy caches differ by address, so it is split off and
	// compared field by field with the lower half's own payload.
	wantRecord, gotRecord := wantPayload.LoftMirrorCylinderJoin, gotPayload.LoftMirrorCylinderJoin
	require.NotNil(t, wantRecord)
	require.NotNil(t, gotRecord)
	wantPayload.LoftMirrorCylinderJoin, gotPayload.LoftMirrorCylinderJoin = nil, nil
	require.Equal(t, wantPayload, gotPayload)
	require.Same(t, wantRecord.Blank, gotRecord.Blank)
	require.Same(t, wantRecord.Source, gotRecord.Source)
	require.Equal(t, wantRecord.Pairs, gotRecord.Pairs)
	require.Equal(t, wantRecord.CapGroups, gotRecord.CapGroups)
	require.Equal(t, wantRecord.CapOutlines, gotRecord.CapOutlines)
	require.Equal(t, wantRecord.BridgeBound, gotRecord.BridgeBound)
	require.Equal(t, wantRecord.BridgeVolume, gotRecord.BridgeVolume)
	wantHalf := wantRecord.LowerHalf.Payload.(facetedPayload)
	gotHalf := gotRecord.LowerHalf.Payload.(facetedPayload)
	wantJoin, gotJoin := wantHalf.LoftCylinderJoin, gotHalf.LoftCylinderJoin
	require.NotNil(t, wantJoin)
	require.NotNil(t, gotJoin)
	wantHalf.LoftCylinderJoin, gotHalf.LoftCylinderJoin = nil, nil
	require.Equal(t, wantHalf, gotHalf)
	// The lower half's Join record holds the placed teeth, the part this
	// change computes on goroutines: every tooth body and its proof witness.
	// The half blank is evaluated afresh by each call.
	require.Equal(t, wantJoin.Blank.Payload, gotJoin.Blank.Payload)
	require.Equal(t, wantJoin.Blank.Volume, gotJoin.Blank.Volume)
	require.Equal(t, wantJoin.Blank.Bounds, gotJoin.Blank.Bounds)
	require.Equal(t, wantJoin.AxialBridge, gotJoin.AxialBridge)
	require.Equal(t, wantJoin.CapOutlines, gotJoin.CapOutlines)
	require.Equal(t, wantJoin.CapGroups, gotJoin.CapGroups)
	require.Equal(t, []float64{wantJoin.MinSideRadius, wantJoin.Radius, wantJoin.InnerInset, wantJoin.OuterExcess,
		wantJoin.RootCollar, wantJoin.Z0, wantJoin.Z1},
		[]float64{gotJoin.MinSideRadius, gotJoin.Radius, gotJoin.InnerInset, gotJoin.OuterExcess,
			gotJoin.RootCollar, gotJoin.Z0, gotJoin.Z1})
	for _, pair := range [][2][]*Body{{bodyHandles(wantJoin.Teeth), bodyHandles(gotJoin.Teeth)}, {bodyHandles(wantJoin.ProofTeeth), bodyHandles(gotJoin.ProofTeeth)}} {
		require.Len(t, pair[1], len(pair[0]))
		for i := range pair[0] {
			require.Equal(t, pair[0][i].n.Payload, pair[1][i].n.Payload, "tooth %d payload", i)
			require.Equal(t, pair[0][i].n.Volume, pair[1][i].n.Volume, "tooth %d volume", i)
			require.Equal(t, pair[0][i].n.Bounds, pair[1][i].n.Bounds, "tooth %d bounds", i)
		}
	}
	require.Equal(t, wantRecord.LowerHalf.Volume, gotRecord.LowerHalf.Volume)
	require.Equal(t, wantRecord.LowerHalf.Bounds, gotRecord.LowerHalf.Bounds)
	// The topology: every face's origins, area and surface kind, in order.
	wantFaces, gotFaces := want.body.Faces(), o.body.Faces()
	require.Len(t, gotFaces, len(wantFaces))
	for i := range wantFaces {
		require.Equal(t, wantFaces[i].Origins(), gotFaces[i].Origins(), "face %d origins", i)
		require.Equal(t, wantFaces[i].n.Area, gotFaces[i].n.Area, "face %d area", i)
		require.Equal(t, wantFaces[i].n.AreaBound, gotFaces[i].n.AreaBound, "face %d area bound", i)
		require.Equal(t, len(wantFaces[i].Loops()), len(gotFaces[i].Loops()), "face %d loops", i)
	}
	require.Equal(t, want.body.n.Origin, o.body.n.Origin)
}

// callOld answers as the pre-PR interleaved loop does.
func (f teethFixture) callOld(ctx context.Context, ref producerID, pairs []*Body) pairsOutcome {
	body, matched, err := oracleLoftMirrorCylinderPairs(ctx, f.d, ref, f.blank, pairs, nil)
	return pairsOutcome{body: body, matched: matched, err: err}
}

func (f teethFixture) call(ctx context.Context, ref producerID, pairs []*Body, workers int) pairsOutcome {
	body, matched, err := loftMirrorCylinderPairs(ctx, f.d, ref, f.blank, pairs, nil, workers)
	return pairsOutcome{body: body, matched: matched, err: err}
}

// pollCancelContext reports cancellation from its n-th Err call on and stays
// cancelled. Its counter is atomic because placement goroutines poll it.
type pollCancelContext struct {
	context.Context //nolint:containedctx // deterministic cancellation wrapper used only within one test call.
	cancelAt        int64
	calls           atomic.Int64
}

func (c *pollCancelContext) Err() error {
	if c.calls.Add(1) >= c.cancelAt {
		return context.Canceled
	}
	return c.Context.Err()
}

// TestLoftMirrorCylinderPairsParallelMatchesSerial runs the real mirrored
// Union with one placement goroutine and with four, on the whole pair list and
// on lists where a pair is refused in the middle, at the end, or twice, and
// requires the same bodies, flags and error text.
func TestLoftMirrorCylinderPairsParallelMatchesSerial(t *testing.T) {
	t.Parallel()
	f := newTeethFixture(t)
	ref := f.d.nextProducerID()
	replaced := func(at ...int) []*Body {
		pairs := append([]*Body(nil), f.pairs...)
		for _, i := range at {
			pairs[i] = f.blank // a prism: the pair check refuses it without an error
		}
		return pairs
	}
	t.Run("all pairs", func(t *testing.T) {
		serial := f.callOld(t.Context(), ref, f.pairs)
		require.NoError(t, serial.err)
		require.True(t, serial.matched)
		require.NotNil(t, serial.body)
		moved := 0
		for _, p := range f.pairs {
			if p.n.Payload.(facetedPayload).Xform != r3.Identity() {
				moved++
			}
		}
		require.Equal(t, 2, moved)
		f.call(t.Context(), ref, f.pairs, 4).requireSame(t, serial)
	})
	for name, pairs := range map[string][]*Body{
		"refused in the middle": replaced(1),
		"refused at the end":    replaced(2),
		"refused twice":         replaced(1, 2),
		"zero pairs":            nil,
	} {
		t.Run(name, func(t *testing.T) {
			serial := f.callOld(t.Context(), ref, pairs)
			f.call(t.Context(), ref, pairs, 4).requireSame(t, serial)
		})
	}
	t.Run("no authenticated placement", func(t *testing.T) {
		// A copy whose placement record names another source ends the Union
		// with an error naming that pair, whichever teeth are placed beside it.
		pairs := f.pairs
		good := pairs[2].n.Payload
		fp := good.(facetedPayload)
		fp.LoftMirrorCopy = &loftMirrorPlacementRecord{Source: &loftMirrorJoinRecord{}, Motion: fp.LoftMirrorCopy.Motion}
		pairs[2].n.Payload = fp
		t.Cleanup(func() { pairs[2].n.Payload = good })
		serial := f.callOld(t.Context(), ref, pairs)
		require.ErrorIs(t, serial.err, ErrUnsupported)
		require.ErrorContains(t, serial.err, "pair 3 has no authenticated axial placement")
		require.True(t, serial.matched)
		f.call(t.Context(), ref, pairs, 4).requireSame(t, serial)
	})
}

// TestLoftMirrorCylinderPlaceTeethCancelled cancels the context at polls spread
// over the real placements. A cancelled call returns the cancellation and no
// teeth, one cancelled after the last poll returns every tooth, and the
// parallel run polls about as often as the serial one. The poll count is not
// exact because the audit's own workers poll on a timer.
func TestLoftMirrorCylinderPlaceTeethCancelled(t *testing.T) {
	t.Parallel()
	f := newTeethFixture(t)
	ref := f.d.nextProducerID()
	record := f.pairs[0].n.Payload.(facetedPayload).LoftMirrorJoin
	lower := record.Lower.Payload.(loftPayload)
	checked := loftcylinder.CheckMirrorPairs(documentNode(f.d), bodyNodes(f.pairs), record, lower.Frame1.Origin().Z)
	run := func(cancelAt int64, workers int) (int64, []*Body, error) {
		ctx := &pollCancelContext{Context: t.Context(), cancelAt: cancelAt}
		teeth, err := loftcylinder.PlaceTeeth(checked.Stop, workers, func(i int) (*Body, error) {
			if checked.Motions[i] == r3.Identity() {
				return bodyHandle(record.Lower), nil
			}
			return placePayload(ctx, lower, f.d, ref, checked.Motions[i])
		})
		return ctx.calls.Load(), teeth, err
	}
	polls, teeth, err := run(math.MaxInt64, 1)
	require.NoError(t, err)
	require.Len(t, teeth, 3)
	require.Greater(t, polls, int64(100))
	parallelPolls, teeth, err := run(math.MaxInt64, 4)
	require.NoError(t, err)
	require.Len(t, teeth, 3)
	require.InDelta(t, polls, parallelPolls, float64(polls)/100)
	for _, n := range []int64{1, 2, polls / 3, polls * 2 / 3, polls * 9 / 10} {
		_, teeth, err := run(n, 4)
		require.ErrorIs(t, err, context.Canceled, "cancel at poll %d", n)
		require.Nil(t, teeth)
	}
	_, teeth, err = run(polls*12/10, 4)
	require.NoError(t, err)
	require.Len(t, teeth, 3)
}

// TestLoftMirrorCylinderPairsRefusalPlacesNothingPastIt compares the polls of
// a mirrored Union that refuses its last pair under four placement goroutines
// with those under one: the parallel call places no tooth the serial call would
// not reach.
func TestLoftMirrorCylinderPairsRefusalPlacesNothingPastIt(t *testing.T) {
	t.Parallel()
	f := newTeethFixture(t)
	ref := f.d.nextProducerID()
	pairs := append([]*Body(nil), f.pairs...)
	pairs[2] = f.blank
	count := func(workers int) (int64, pairsOutcome) {
		ctx := &pollCancelContext{Context: t.Context(), cancelAt: math.MaxInt64}
		if workers == 0 {
			out := f.callOld(ctx, ref, pairs)
			return ctx.calls.Load(), out
		}
		out := f.call(ctx, ref, pairs, workers)
		return ctx.calls.Load(), out
	}
	serialPolls, want := count(0)
	parallelPolls, got := count(4)
	got.requireSame(t, want)
	require.False(t, got.matched)
	require.Greater(t, serialPolls, int64(100))
	require.InDelta(t, serialPolls, parallelPolls, float64(serialPolls)/100)
	// The refusal ends the call right after the placements, so a cancellation
	// in the last tenth of the polls lands in a placement. It must surface, not
	// the refusal.
	// The old loop's answer is the oracle: pair 3's refusal must not replace
	// pair 2's placement error.
	cancelAt := serialPolls * 9 / 10
	oldOut := f.callOld(&pollCancelContext{Context: t.Context(), cancelAt: cancelAt}, ref, pairs)
	require.ErrorIs(t, oldOut.err, context.Canceled)
	require.Nil(t, oldOut.body)
	for _, workers := range []int{1, 4} {
		ctx := &pollCancelContext{Context: t.Context(), cancelAt: cancelAt}
		f.call(ctx, ref, pairs, workers).requireSame(t, oldOut)
	}
}

// TestLoftMirrorCylinderPlaceTeethMatchesSerialPlacement places the real lower
// tooth under every motion with one goroutine and with four, and with a
// cancelled context for the first, a middle, the last and two teeth.
func TestLoftMirrorCylinderPlaceTeethMatchesSerialPlacement(t *testing.T) {
	t.Parallel()
	f := newTeethFixture(t)
	ref := f.d.nextProducerID()
	record := f.pairs[0].n.Payload.(facetedPayload).LoftMirrorJoin
	lower := record.Lower.Payload.(loftPayload)
	checked := loftcylinder.CheckMirrorPairs(documentNode(f.d), bodyNodes(f.pairs), record, lower.Frame1.Origin().Z)
	require.Equal(t, len(f.pairs), checked.Stop)
	live, stopLive := context.WithCancel(t.Context())
	defer stopLive()
	cancelled, cancel := context.WithCancel(t.Context())
	cancel()
	placeWith := func(failing ...int) func(i int) (*Body, error) {
		return func(i int) (*Body, error) {
			ctx := live
			if slices.Contains(failing, i) {
				ctx = cancelled
			}
			if checked.Motions[i] == r3.Identity() {
				return bodyHandle(record.Lower), nil
			}
			return placePayload(ctx, lower, f.d, ref, checked.Motions[i])
		}
	}
	serial, err := loftcylinder.PlaceTeeth(checked.Stop, 1, placeWith())
	require.NoError(t, err)
	parallel, err := loftcylinder.PlaceTeeth(checked.Stop, 4, placeWith())
	require.NoError(t, err)
	require.Len(t, parallel, len(serial))
	for i := range serial {
		want, got := serial[i].n.Payload.(loftPayload), parallel[i].n.Payload.(loftPayload)
		if checked.Motions[i] == r3.Identity() {
			require.Same(t, serial[i], parallel[i])
			continue
		}
		require.NotEmpty(t, want.Verts)
		require.Equal(t, want.Verts, got.Verts)
		require.Equal(t, want.Tris, got.Tris)
		require.Equal(t, want.Proof, got.Proof)
		require.Equal(t, serial[i].n.Volume, parallel[i].n.Volume)
		require.Equal(t, serial[i].n.Bounds, parallel[i].n.Bounds)
	}
	for _, failing := range [][]int{{1}, {2}, {1, 2}} {
		for _, workers := range []int{1, 4} {
			teeth, err := loftcylinder.PlaceTeeth(checked.Stop, workers, placeWith(failing...))
			require.ErrorIs(t, err, context.Canceled)
			require.Nil(t, teeth)
		}
	}
}

// oracleLoftMirrorCylinderPairs is the interleaved loop as it was before the
// placements ran side by side, kept verbatim as the test oracle: each pair is
// checked and placed in pair order, and the first refusal or placement error
// ends the call. Test-only; the production function must answer as it does.
func oracleLoftMirrorCylinderPairs(ctx context.Context, d *Document, ref producerID,
	blank *Body, pairs []*Body, source *loftMirrorJoinRecord) (*Body, bool, error) {
	pp, blankOK := blank.n.Payload.(prismPayload)
	if !blankOK || !loftcylinder.MirrorBlank(pp) || len(pairs) == 0 || len(pairs) > 32 {
		return nil, false, nil
	}
	first, firstOK := pairs[0].n.Payload.(facetedPayload)
	if !firstOK || first.LoftMirrorJoin == nil || first.Xform != r3.Identity() ||
		!first.ContactAudited || source != nil && source != first.LoftMirrorJoin {
		return nil, false, nil
	}
	record := first.LoftMirrorJoin
	if record.Lower == nil || record.Upper == nil || record.StationCount < 3 ||
		record.Lower.Doc != documentNode(d) || record.Upper.Doc != documentNode(d) {
		return nil, false, nil
	}
	lower, lowOK := record.Lower.Payload.(loftPayload)
	upper, upOK := record.Upper.Payload.(loftPayload)
	if !lowOK || !upOK || !lower.AuthenticatedReconstruction ||
		!upper.AuthenticatedReconstruction || lower.SourceIdentity == nil ||
		lower.SourceIdentity != upper.SourceIdentity ||
		lower.Xform != r3.Identity() || !lower.MirrorAxialSource() {
		return nil, false, nil
	}
	reflection, err := r3.Reflection(lower.Frame1)
	if err != nil {
		return nil, true, fmt.Errorf("%w: mirrored cylinder source has no reflection: %s", ErrUnsupported, err)
	}
	if upper.Xform != reflection {
		return nil, false, nil
	}
	z0, middle, z2 := pp.Z0, lower.Frame1.Origin().Z, pp.Z1
	if lower.Frame0.Origin().Z != z0 || !(z0 < middle && middle < z2) ||
		!loftcylinder.MirrorLevels(z0, middle, z2) {
		return nil, false, nil
	}
	// The first-stage source relation proves upper=reflection(lower). The
	// blank has the same circle and exact caps over both half intervals.
	half := pp
	half.Z1 = middle
	halfBlank, err := bodyResult(prismrecord.Build(ctx, envOf(documentNode(d), ref), half, freeform.NewFreeformWork()))
	if err != nil {
		return nil, true, err
	}
	lowerTeeth := make([]*Body, len(pairs))
	upperBridges := make([]float64, len(pairs))
	var bridgeBound, bridgeVolume float64
	for i, pair := range pairs {
		payload, ok := pair.n.Payload.(facetedPayload)
		if !ok || !payload.ContactAudited || pair.n.Doc != documentNode(d) {
			return nil, false, nil
		}
		motion, ok := loftcylinder.MirrorPairMotion(payload, record)
		if !ok {
			return nil, true, fmt.Errorf("%w: mirrored cylinder pair %d has no authenticated axial placement", ErrUnsupported, i+1)
		}
		if !loftcylinder.AxialPlacement(motion) {
			shift, swept, ok := loftcylinder.MirrorUpperBridge(bodyNode(pair), motion, middle)
			if !ok {
				return nil, true, fmt.Errorf("%w: mirrored cylinder pair %d has no upper axial bridge", ErrUnsupported, i+1)
			}
			bridgeBound = max(bridgeBound, shift)
			bridgeVolume = proofbound.AbsSumUpper(bridgeVolume, swept)
			upperBridges[i] = shift
		}
		tooth := bodyHandle(record.Lower)
		if motion != r3.Identity() {
			tooth, err = placePayload(ctx, lower, d, ref, motion)
			if err != nil {
				return nil, true, err
			}
		}
		lowerTeeth[i] = tooth
	}
	lowerJoin, admitted, err := tryLoftCylinderUnionTeethWithUpperBridge(
		ctx, d, ref, halfBlank, lowerTeeth, upperBridges)
	if err != nil {
		return nil, true, err
	}
	if !admitted {
		return nil, true, fmt.Errorf("%w: mirrored cylinder's lower slab lacks a certified circular Join", ErrUnsupported)
	}
	result, err := bodyResult(loftcylinder.BuildMirrorJoin(ctx, envOf(documentNode(d), ref), bodyNode(blank),
		bodyNodes(pairs), record, bodyNode(lowerJoin), reflection, bridgeBound, bridgeVolume))
	return result, true, err
}
