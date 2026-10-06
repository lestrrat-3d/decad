package apitest_test

import (
	"context"
	"math"
	"math/big"
	"slices"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// The fixtures of docs/multibody-dynamics-design.md §13 PR 18: §10.4's
// ContactBand for a body whose held mesh stands for its true boundary up to a
// positive displacement δ. Two real producers supply δ, read back through
// Tessellate's Bound rather than pinned: a cap-loop chamfered block, whose δ
// is the float rounding of its offset contour (about 1e-15 mm), and a box
// Union a chorded disc, whose δ is the chord sagitta (about 4e-4 mm), wide
// enough to place held gaps and depths inside and outside it on the dyadic
// grid.
//
// Legs shown to fail (each deleted or zeroed in turn, fixture red, then
// restored):
//   - the band width 2δ: TestContactPairBandChamferedBlockOnFloor publishes a
//     zero Gap.Bound;
//   - the pose's stretch on δ: TestContactPairBandChamferedBlockOnFloor's
//     turned pair publishes exactly 2δ;
//   - δ on a held gap: TestContactPairBandChargesHeldGap's 2^-14 mm gap reads
//     Separated, and its 1 mm gap publishes a bound below δ;
//   - δ on the deep-vertex margin: TestContactPairBandChargesHeldGap's
//     2^-14 mm overlap reads Overlapping;
//   - δ on each manifold witness ball and the band on each Separation:
//     TestContactPairBandChamferedBlockOnFloor's points carry the clip's bare
//     balls and an exact zero separation;
//   - the exact-face rule for the band's normal: TestContactPairBandNeedsAnExactFace
//     publishes a held face's normal;
//   - the track's 2δ widening: TestSweepPairBandTrack publishes an exact
//     SweepPersistentTouch; in BandAt alone, its prefix band reads zero;
//   - δ on the track's witness balls: TestSweepPairBandFootStaysInsideFace's
//     points carry the bare pose deviation, zero at its dyadic poses;
//   - the track's exact-support rule: TestSweepPairBandTrack's stacked
//     blocks publish a track on a held face;
//   - δ on the track's face containment: TestSweepPairBandFootStaysInsideFace
//     publishes a track whose true foot may leave the floor;
//   - the transfer charge on the replay's clear check:
//     TestReplayTransferChargeIsTheBasisDifference in
//     contact_band_internal_test.go reads a zero charge on a rotating path.
//
// contact_band_internal_test.go shows the sweep's transfer legs on the
// transfer itself. The track replay's height check reads the held depth, as
// PR 13's does; it only refuses, and no sound fixture reaches it.

// chamferedBlock is a 12 mm cube on the floor plane z = 0 with its upper cap
// loop chamfered 0.1 mm, and the displacement its held mesh publishes.
func chamferedBlock(t *testing.T, doc *decad.Document) (*decad.Body, float64) {
	t.Helper()
	box := boxBody(t, doc, -6, -6, 6, 6, 12)
	block, err := box.Chamfer(t.Context(), capLoopEdges(box), units.Millimeters(.1))
	require.NoError(t, err)
	mesh, err := block.Tessellate(t.Context(), units.Millimeters(1))
	require.NoError(t, err)
	delta := mesh.Bound().Base()
	require.Positive(t, delta, "a 0.1 mm setback rounds its offset contour")
	return block, delta
}

// knobBody is a 10 mm box Union a radius-2 disc standing 2 mm proud of its
// top, moved 1/8 mm along X so no exact support proof survives, and its
// displacement: the disc's chord sagitta.
func knobBody(t *testing.T, doc *decad.Document) (*decad.Body, float64) {
	t.Helper()
	base := boxBody(t, doc, -5, -5, 5, 5, 10)
	disc := discBody(t, doc, 0, 2, 4)
	disc, err := disc.Placed(t.Context(), contactPose(t, r3.Vec{Z: 8}))
	require.NoError(t, err)
	union, err := decad.Union(t.Context(), base, disc)
	require.NoError(t, err)
	knob, err := union.Placed(t.Context(), contactPose(t, r3.Vec{X: .125}))
	require.NoError(t, err)
	mesh, err := knob.Tessellate(t.Context(), units.Millimeters(1))
	require.NoError(t, err)
	delta := mesh.Bound().Base()
	require.Greater(t, delta, math.Ldexp(1, -14), "the fixtures place gaps of 2^-14 mm inside the band")
	require.Less(t, delta, math.Ldexp(1, -10), "and gaps of 2^-10 mm outside it")
	return knob, delta
}

// octagonFloor is a convex prism floor, not a source box, whose top face is
// z = 8, with a straight rim at x = ±64 for |y| <= 32.
func octagonFloor(t *testing.T, doc *decad.Document) *decad.Body {
	t.Helper()
	return prismBody(t, doc, [][2]float64{{-64, -32}, {-32, -64}, {32, -64}, {64, -32},
		{64, 32}, {32, 64}, {-32, 64}, {-64, 32}}, 8)
}

func bandContactRequest() decad.ContactRequest {
	return decad.ContactRequest{PointResolution: units.Millimeters(1e-3), NormalResolution: units.Degrees(1)}
}

func TestContactPairBandChamferedBlockOnFloor(t *testing.T) {
	doc := decad.New()
	floor := boxBodyAtZ(t, doc, -100, -100, 100, 100, -10, 10)
	block, delta := chamferedBlock(t, doc)
	before := doc.Bodies()
	id := r3.Identity()
	for order := range 2 {
		a, b := floor, block
		if order == 1 {
			a, b = block, floor
		}
		report, err := doc.ContactPair(t.Context(), a, b, id, id, contactRequest())
		require.NoError(t, err)
		require.Equal(t, decad.ContactBand, report.Relation, "order %d reason=%v", order, report.Reason)
		require.Zero(t, report.Gap.Value.Base())
		// Doubling a float is exact, and the identity pose stretches nothing.
		require.Equal(t, 2*delta, report.Gap.Bound.Base())
		require.NotNil(t, report.Manifold, "reason=%v", report.Reason)
		require.Len(t, report.Manifold.Points, 4)
		normal := 1.0
		if order == 1 {
			normal = -1
		}
		for _, point := range report.Manifold.Points {
			onBlock, onFloor := point.OnB, point.OnA
			if order == 1 {
				onBlock, onFloor = point.OnA, point.OnB
			}
			// The clip's corners are the block's lower corners, exact floats.
			require.Equal(t, 6.0, math.Abs(onBlock.Value.X))
			require.Equal(t, 6.0, math.Abs(onBlock.Value.Y))
			require.Zero(t, onBlock.Value.Z)
			require.Equal(t, onBlock.Value, onFloor.Value)
			require.GreaterOrEqual(t, onBlock.Bound.Base(), delta, "a true corner lies within δ of the held one")
			require.Zero(t, onFloor.Bound.Base(), "the floor carries no displacement")
			require.Equal(t, r3.Vec{Z: normal}, point.Normal.Value)
			require.Zero(t, point.Separation.Value.Base())
			require.Equal(t, 2*delta, point.Separation.Bound.Base())
		}
	}

	// Turned together about Z the pair still touches exactly, held; the
	// float basis stretches lengths by a hair over one, and δ with them.
	turn := rotationPose(t, r3.Vec{Z: 1}, 37, r3.Vec{X: 3})
	turned, err := doc.ContactPair(t.Context(), floor, block, turn, turn, contactRequest())
	require.NoError(t, err)
	require.Equal(t, decad.ContactBand, turned.Relation)
	require.Greater(t, turned.Gap.Bound.Base(), 2*delta)
	require.Less(t, turned.Gap.Bound.Base(), 2*delta*(1+1e-9))
	require.Equal(t, before, doc.Bodies())
}

func TestContactPairBandCancels(t *testing.T) {
	doc := decad.New()
	floor := boxBodyAtZ(t, doc, -100, -100, 100, 100, -10, 10)
	block, _ := chamferedBlock(t, doc)
	id := r3.Identity()
	// The first query caches the block's held mesh and both convexity
	// certificates; the second counts the polls of a query that reads them
	// and publishes the charged manifold.
	var polls int32
	for range 2 {
		counting := newCancelAfterContext(t.Context(), math.MaxInt32)
		report, err := doc.ContactPair(counting, floor, block, id, id, contactRequest())
		require.NoError(t, err)
		require.NotNil(t, report.Manifold)
		polls = counting.calls.Load()
	}
	before := doc.Bodies()
	for limit := int32(1); limit <= polls; limit++ {
		canceling := newCancelAfterContext(t.Context(), limit)
		report, err := doc.ContactPair(canceling, floor, block, id, id, contactRequest())
		require.ErrorIs(t, err, context.Canceled, "limit=%d", limit)
		require.Nil(t, report)
	}
	require.Equal(t, before, doc.Bodies())
}

func TestContactPairBandNeedsAnExactFace(t *testing.T) {
	// One chamfered block stands on another's 11.8 mm top face. Both held
	// faces carry a displacement, so neither's normal is a true face normal.
	doc := decad.New()
	lower, delta := chamferedBlock(t, doc)
	upper, _ := chamferedBlock(t, doc)
	report, err := doc.ContactPair(t.Context(), lower, upper, r3.Identity(),
		contactPose(t, r3.Vec{Z: 12}), contactRequest())
	require.NoError(t, err)
	require.Equal(t, decad.ContactBand, report.Relation)
	require.Equal(t, 4*delta, report.Gap.Bound.Base())
	require.Nil(t, report.Manifold)
	require.Equal(t, decad.ContactNoNormalProof, report.Reason)
}

func TestContactPairBandChargesHeldGap(t *testing.T) {
	doc := decad.New()
	floor := octagonFloor(t, doc)
	knob, delta := knobBody(t, doc)
	at := func(z float64) *decad.ContactReport {
		t.Helper()
		report, err := doc.ContactPair(t.Context(), floor, knob, r3.Identity(),
			contactPose(t, r3.Vec{Z: 8 + z}), bandContactRequest())
		require.NoError(t, err)
		return report
	}

	touch := at(0)
	require.Equal(t, decad.ContactBand, touch.Relation)
	require.Equal(t, 2*delta, touch.Gap.Bound.Base())

	// A held gap inside δ may close on the true bodies. The exact floor hosts
	// the knob's four lower corners as its lifted set, read within b = δ, and
	// the gap carries δ on the held one (§10.4).
	near := at(math.Ldexp(1, -14))
	require.Equal(t, decad.ContactBand, near.Relation, "reason=%v", near.Reason)
	require.Zero(t, near.Gap.Value.Base())
	require.GreaterOrEqual(t, near.Gap.Bound.Base(), math.Ldexp(1, -14)+delta)
	require.NotNil(t, near.Manifold, "reason=%v", near.Reason)
	require.Len(t, near.Manifold.Points, 4)
	for _, point := range near.Manifold.Points {
		require.Equal(t, math.Ldexp(1, -14), point.Separation.Value.Base())
		require.GreaterOrEqual(t, point.Separation.Bound.Base(), delta)
		require.GreaterOrEqual(t, point.OnB.Bound.Base(), delta)
		require.Zero(t, point.OnA.Bound.Base())
		require.Equal(t, r3.Vec{Z: 1}, point.Normal.Value)
	}

	// A held gap past δ is a true gap with δ charged.
	far := at(1)
	require.Equal(t, decad.ContactSeparated, far.Relation)
	require.Equal(t, 1.0, far.Gap.Value.Base())
	require.GreaterOrEqual(t, far.Gap.Bound.Base(), delta)
	require.Positive(t, far.Gap.Value.Base()-far.Gap.Bound.Base())

	// A held overlap shallower than δ proves nothing; the knob is not convex,
	// so no penetration depth puts it in the band either.
	shallow := at(-math.Ldexp(1, -14))
	require.Equal(t, decad.ContactUndecided, shallow.Relation)
	deep := at(-1)
	require.Equal(t, decad.ContactOverlapping, deep.Relation)
}

func TestSweepPairBandImpact(t *testing.T) {
	// The block falls 1 mm at 1 mm/s and lands exactly at the end of the
	// second: the bracket's right sample is the held touch, a band.
	doc := decad.New()
	floor := boxBodyAtZ(t, doc, -100, -100, 100, 100, -10, 10)
	block, delta := chamferedBlock(t, doc)
	floorPath := sweepDrift(r3.Vec{}, 1)
	blockPath := sweepDrift(r3.Vec{Z: -1}, 1)
	blockPath.From = contactPose(t, r3.Vec{Z: 1})
	for order := range 2 {
		a, b := floor, block
		pathA, pathB := decad.PairPath(floorPath), decad.PairPath(blockPath)
		if order == 1 {
			a, b, pathA, pathB = block, floor, pathB, pathA
		}
		report, err := doc.SweepPair(t.Context(), a, b, pathA, pathB, tumbleRequest())
		require.NoError(t, err)
		require.Equal(t, decad.SweepImpactBracket, report.Outcome, "order %d cause=%v", order, report.Cause)
		require.Equal(t, 1.0, report.Bracket.To.Fraction.Base())
		require.Equal(t, 1-math.Ldexp(1, -20), report.Bracket.From.Fraction.Base())
		require.Equal(t, decad.ContactBand, report.Event.Relation)
		require.Equal(t, 2*delta, report.Event.Gap.Bound.Base())
		require.NotNil(t, report.Event.Manifold)

		poseA, poseB, err := report.CertifiedPosesAt(units.Seconds(.5))
		require.NoError(t, err)
		contact, err := doc.ContactPair(t.Context(), a, b, poseA, poseB, contactRequest())
		require.NoError(t, err)
		require.Equal(t, decad.ContactSeparated, contact.Relation)
		require.InDelta(t, .5, contact.Gap.Value.Base(), contact.Gap.Bound.Base())
	}
}

func TestSweepPairBandTrack(t *testing.T) {
	// The block rests on the floor while both slide along X at 16 mm/s far
	// from the origin, so the rounded poses deviate from the ideal path.
	doc := decad.New()
	floor := boxBodyAtZ(t, doc, -100, -100, 100, 100, -10, 10)
	block, delta := chamferedBlock(t, doc)
	floorPath := sweepDrift(r3.Vec{X: 16}, 1)
	floorPath.From = contactPose(t, r3.Vec{X: tumbleOffset})
	blockPath := sweepDrift(r3.Vec{X: 16}, 1)
	blockPath.From = contactPose(t, r3.Vec{X: tumbleOffset})
	for order := range 2 {
		a, b := floor, block
		pathA, pathB := decad.PairPath(floorPath), decad.PairPath(blockPath)
		if order == 1 {
			a, b, pathA, pathB = block, floor, pathB, pathA
		}
		report, err := doc.SweepPair(t.Context(), a, b, pathA, pathB, bandRequest())
		require.NoError(t, err)
		require.Equal(t, decad.SweepPersistentBand, report.Outcome, "order %d cause=%v", order, report.Cause)
		require.Equal(t, decad.ContactBand, report.InitialEvent.Relation)
		track := report.ContactTrack
		require.Equal(t, 1.0, track.End().Fraction.Base())
		// Nothing closes or turns, so the held band is zero and the published
		// one is 2δ.
		band := track.Band()
		require.NotNil(t, band)
		require.InDelta(t, 2*delta, band.Value.Base(), band.Bound.Base())
		prefix, err := track.BandAt(units.Scalar(.5))
		require.NoError(t, err)
		require.InDelta(t, 2*delta, prefix.Value.Base(), prefix.Bound.Base())
		manifold, err := track.ManifoldAt(units.Scalar(.5))
		require.NoError(t, err)
		require.Len(t, manifold.Points, 4)
		for _, point := range manifold.Points {
			onBlock := point.OnB
			if order == 1 {
				onBlock = point.OnA
			}
			want := r3.Vec{X: tumbleOffset + 8 + math.Copysign(6, onBlock.Value.X-tumbleOffset-8),
				Y: math.Copysign(6, onBlock.Value.Y)}
			require.InDelta(t, 0, onBlock.Value.Sub(want).Len(), onBlock.Bound.Base())
			require.GreaterOrEqual(t, onBlock.Bound.Base(), delta)
			require.GreaterOrEqual(t, point.Separation.Bound.Base(), 2*delta)
		}
		poseA, poseB, err := report.CertifiedPosesAt(units.Seconds(.75))
		require.NoError(t, err)
		contact, err := doc.ContactPair(t.Context(), a, b, poseA, poseB, contactRequest())
		require.NoError(t, err)
		require.Equal(t, decad.ContactBand, contact.Relation)
	}

	// A band start is an initial contact; no departure is published.
	req := bandRequest()
	req.StartPolicy = decad.StopAtInitialContact
	stop, err := doc.SweepPair(t.Context(), floor, block, floorPath, blockPath, req)
	require.NoError(t, err)
	require.Equal(t, decad.SweepInitiallyTouching, stop.Outcome)
	require.Equal(t, decad.ContactBand, stop.Event.Relation)
	req.StartPolicy = decad.ContinueSeparatingTouch
	separating, err := doc.SweepPair(t.Context(), floor, block, floorPath, blockPath, req)
	require.NoError(t, err)
	require.Equal(t, decad.SweepUndecided, separating.Outcome)
	require.Equal(t, decad.SweepDepartureUnproved, separating.Cause)

	// Two chamfered blocks resting on each other have no exact support face.
	upper, _ := chamferedBlock(t, doc)
	upperPath := sweepDrift(r3.Vec{X: 16}, 1)
	upperPath.From = contactPose(t, r3.Vec{X: tumbleOffset, Z: 12})
	stacked, err := doc.SweepPair(t.Context(), block, upper, blockPath, upperPath, bandRequest())
	require.NoError(t, err)
	require.Equal(t, decad.ContactBand, stacked.InitialEvent.Relation)
	require.Equal(t, decad.SweepUndecided, stacked.Outcome)
	require.Equal(t, decad.SweepContactTrackUnproved, stacked.Cause)
}

func TestSweepPairBandFootStaysInsideFace(t *testing.T) {
	// The knob rests on the octagon floor. Its held lower face sits strictly
	// inside the floor's top face, but a true corner may lie δ beyond a held
	// one, so a held corner 2^-16 mm from the rim cannot certify a track.
	doc := decad.New()
	floor := octagonFloor(t, doc)
	knob, delta := knobBody(t, doc)
	req := bandRequest()
	req.ContactRequest = bandContactRequest()
	rest := func(x float64) *decad.SweepReport {
		t.Helper()
		knobPath := sweepDrift(r3.Vec{}, 1)
		knobPath.From = contactPose(t, r3.Vec{X: x, Z: 8})
		report, err := doc.SweepPair(t.Context(), floor, knob, sweepDrift(r3.Vec{}, 1), knobPath, req)
		require.NoError(t, err)
		require.Equal(t, decad.ContactBand, report.InitialEvent.Relation)
		return report
	}
	inside := rest(0)
	require.Equal(t, decad.SweepPersistentBand, inside.Outcome, "cause=%v", inside.Cause)
	require.InDelta(t, 2*delta, inside.ContactTrack.Band().Value.Base(), inside.ContactTrack.Band().Bound.Base())
	manifold, err := inside.ContactTrack.ManifoldAt(units.Scalar(1))
	require.NoError(t, err)
	for _, point := range manifold.Points {
		require.GreaterOrEqual(t, point.OnB.Bound.Base(), delta)
	}

	// The knob's held x = 5.125 side lands 2^-16 mm inside the x = 64 rim.
	rim := rest(64 - 5.125 - math.Ldexp(1, -16))
	require.Equal(t, decad.SweepUndecided, rim.Outcome)
	require.Equal(t, decad.SweepContactTrackUnproved, rim.Cause)
}

func TestSweepPairBandReplaysBracketLeftEdge(t *testing.T) {
	// The knob falls 1 mm onto the octagon floor at 1 mm/s. Its held gap
	// enters the band at δ, so the bracket brackets that crossing, and the
	// left edge's true gap is no wider than one time step's travel.
	doc := decad.New()
	floor := octagonFloor(t, doc)
	knob, delta := knobBody(t, doc)
	knobPath := sweepDrift(r3.Vec{Z: -1}, 1)
	knobPath.From = contactPose(t, r3.Vec{Z: 9})
	req := tumbleRequest()
	req.ContactRequest = bandContactRequest()
	report, err := doc.SweepPair(t.Context(), floor, knob, sweepDrift(r3.Vec{}, 1), knobPath, req)
	require.NoError(t, err)
	require.Equal(t, decad.SweepImpactBracket, report.Outcome, "cause=%v", report.Cause)
	require.Equal(t, decad.ContactBand, report.Event.Relation)
	left := report.Bracket.From.Elapsed.Value.Base()
	step := math.Ldexp(1, -20)
	require.InDelta(t, 1-delta, left, 2*step)

	_, _, err = report.CertifiedPosesAt(units.Seconds(.5))
	require.NoError(t, err)
	// At the left edge the proven true gap is positive but far under δ. The
	// falling path only translates, so its rounded basis is the ideal one and
	// replay charges no transfer (§10.4): the gap need only exceed the pose
	// rounding, and the edge replays.
	_, _, err = report.CertifiedPosesAt(report.Bracket.From.Elapsed.Value)
	require.NoError(t, err)
}

// partsBinBottle is §2's revolved bottle: the full revolve about Z of a
// line-and-arc half-profile with a Ø16 mm base from z = 0 to 14, a
// quarter-circle shoulder to a Ø8 mm neck, and its top at z = 24.
func partsBinBottle(t *testing.T, doc *decad.Document) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XZ())
	require.NoError(t, err)
	a := s.CreatePoint(0, 0)
	b := s.CreatePoint(8, 0)
	c := s.CreatePoint(8, 14)
	center := s.CreatePoint(4, 14)
	d := s.CreatePoint(4, 18)
	e := s.CreatePoint(4, 24)
	f := s.CreatePoint(0, 24)
	for _, p := range []*sketch.Point{a, b, c, center, d, e, f} {
		s.Fix(p)
	}
	s.CreateLine(a, b)
	s.CreateLine(b, c)
	s.CreateArc(center, c, d)
	s.CreateLine(d, e)
	s.CreateLine(e, f)
	s.CreateLine(f, a)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	require.Len(t, s.Profiles(), 1)
	body, err := doc.Revolve(s, s.Profiles()[0], decad.SketchLine{
		Start: decad.Point2{U: 0, V: 0}, End: decad.Point2{U: 0, V: 1}}, decad.FullRevolution{})
	require.NoError(t, err)
	return body
}

// partsBinCup is §2's 24×16×16 mm box shelled 2 mm through its top.
func partsBinCup(t *testing.T, doc *decad.Document) *decad.Body {
	t.Helper()
	box := boxBody(t, doc, -12, -8, 12, 8, 16)
	cup, err := box.Shell(t.Context(), topCap(box), units.Millimeters(2))
	require.NoError(t, err)
	return cup
}

// partsBinLoft is §2's loft from a square with its edge midpoints pushed out
// at z = 0 to an octagon at z = 16.
func partsBinLoft(t *testing.T, doc *decad.Document) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	top, err := w.CreateOffsetPlane(w.XY(), 16)
	require.NoError(t, err)
	s0, p0 := meshPolygonSketch(t, w, w.XY(), [][2]float64{
		{8.5, 0}, {8, 8}, {0, 8.5}, {-8, 8}, {-8.5, 0}, {-8, -8}, {0, -8.5}, {8, -8}})
	s1, p1 := meshPolygonSketch(t, w, top, [][2]float64{
		{6, -2.5}, {6, 2.5}, {2.5, 6}, {-2.5, 6}, {-6, 2.5}, {-6, -2.5}, {-2.5, -6}, {2.5, -6}})
	loft, err := doc.Loft(t.Context(), s0, p0, s1, p1)
	require.NoError(t, err)
	return loft
}

// partsBinSweep is §2's straight 14 mm sweep of the parts-bin hexagon, which
// tessellates as the prism it reduced to.
func partsBinSweep(t *testing.T, doc *decad.Document) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	s, profile := meshPolygonSketch(t, w, w.XY(), sweepHexagonCorners)
	path, err := decad.NewPath(r3.Vec{}, decad.LineTo{End: r3.NewVec(0, 0, 14)})
	require.NoError(t, err)
	body, err := doc.Sweep(t.Context(), s, profile, path)
	require.NoError(t, err)
	return body
}

// requireGapHolds asserts that the true signed separation lies inside the
// report's published gap interval, compared exactly.
func requireGapHolds(t *testing.T, report *decad.ContactReport, truth *big.Float) {
	t.Helper()
	require.NotNil(t, report.Gap, "relation=%v reason=%v", report.Relation, report.Reason)
	value := new(big.Float).SetPrec(512).SetFloat64(report.Gap.Value.Base())
	bound := new(big.Float).SetPrec(512).SetFloat64(report.Gap.Bound.Base())
	low := new(big.Float).SetPrec(512).Sub(value, bound)
	high := new(big.Float).SetPrec(512).Add(value, bound)
	require.LessOrEqual(t, low.Cmp(truth), 0, "true separation %s below the gap [%s, %s]",
		truth.Text('g', 20), low.Text('g', 20), high.Text('g', 20))
	require.LessOrEqual(t, truth.Cmp(high), 0, "true separation %s above the gap [%s, %s]",
		truth.Text('g', 20), low.Text('g', 20), high.Text('g', 20))
}

// TestHeldMeshAdmitsEveryPayload is docs/multibody-dynamics-design.md §13
// PR 20b: every solid payload without an exact contact family is read off its
// VerifyAll tessellation, an all-planar body at a chord that chords nothing
// and a body with a curved face at the request's HeldChord, with δ the mesh's
// Bound charged on the relation (§10.4).
//
// Legs shown to fail (each deleted, fixture red, then restored):
//   - the δ reading: with the mesh's Bound replaced by zero, the bottle on its
//     side with a held facet δ/2 above the floor reads an exact Separated gap
//     of about δ/2, while the true base cylinder, evaluated in 512-bit
//     arithmetic, hangs its chord sagitta below that facet and pokes into the
//     floor (and the upright bottle's bound falls below the mesh's Bound);
//   - the snapshot cache's chord key: with every cached snapshot serving every
//     chord, the coarse query publishes the fine mesh's bound.
func TestHeldMeshAdmitsEveryPayload(t *testing.T) {
	const chord = .03
	doc := decad.New()
	floor := boxBodyAtZ(t, doc, -100, -100, 100, 100, -10, 10)
	bottle := partsBinBottle(t, doc)
	mesh, err := bottle.Tessellate(t.Context(), units.Millimeters(chord))
	require.NoError(t, err)
	delta := mesh.Bound().Base()
	require.Positive(t, delta, "the bottle's base and shoulder are chorded")
	req := contactRequest()
	req.HeldChord = units.Millimeters(chord)

	up := contactPose(t, r3.Vec{Z: 5})
	upright, err := doc.ContactPair(t.Context(), floor, bottle, r3.Identity(), up, req)
	require.NoError(t, err)
	require.Equal(t, decad.ContactSeparated, upright.Relation, "reason=%v", upright.Reason)
	requireGapHolds(t, upright, big.NewFloat(5))
	// The translation pose stretches δ by a hair over one, if at all.
	require.GreaterOrEqual(t, upright.Gap.Bound.Base(), delta)
	require.LessOrEqual(t, upright.Gap.Bound.Base(), .05)

	// The snapshot is cached per body and chord: a second query reads it and
	// publishes the same report bit for bit.
	again, err := doc.ContactPair(t.Context(), floor, bottle, r3.Identity(), up, req)
	require.NoError(t, err)
	require.Equal(t, upright, again)
	// A coarser chord reads its own, coarser mesh, and the finer chord then
	// reads the same report again.
	coarseMesh, err := bottle.Tessellate(t.Context(), units.Millimeters(.1))
	require.NoError(t, err)
	require.Greater(t, coarseMesh.Bound().Base(), .05)
	coarseReq := req
	coarseReq.HeldChord = units.Millimeters(.1)
	coarse, err := doc.ContactPair(t.Context(), floor, bottle, r3.Identity(), up, coarseReq)
	require.NoError(t, err)
	require.Equal(t, decad.ContactSeparated, coarse.Relation, "reason=%v", coarse.Reason)
	require.GreaterOrEqual(t, coarse.Gap.Bound.Base(), coarseMesh.Bound().Base())
	again, err = doc.ContactPair(t.Context(), floor, bottle, r3.Identity(), up, req)
	require.NoError(t, err)
	require.Equal(t, upright, again)

	// A zero chord admits no curved body.
	unchorded, err := doc.ContactPair(t.Context(), floor, bottle, r3.Identity(), up, contactRequest())
	require.NoError(t, err)
	require.Equal(t, decad.ContactUndecided, unchorded.Relation)
	require.Nil(t, unchorded.Gap)

	// The bottle on its side, turned about its axis so the facet between two
	// adjacent base-rim vertices is lowest. The pose sends the body's Z axis
	// to −Y exactly and its row for world Z is (sin φ, cos φ, 0).
	var rim []float64
	for _, v := range mesh.Vertices() {
		if v.Z == 0 && math.Abs(math.Hypot(v.X, v.Y)-8) < 1e-9 {
			rim = append(rim, math.Atan2(v.Y, v.X))
		}
	}
	require.Greater(t, len(rim), 8)
	slices.Sort(rim)
	mid := (rim[0] + rim[1]) / 2
	sinPhi, cosPhi := -math.Cos(mid), -math.Sin(mid)
	basis := r3.Basis{EX: r3.Vec{X: cosPhi, Z: sinPhi}, EY: r3.Vec{X: -sinPhi, Z: cosPhi}, EZ: r3.Vec{Y: -1}}
	turned, err := r3.FromBasis(basis, r3.Vec{})
	require.NoError(t, err)
	held := math.Inf(1)
	for _, v := range mesh.Vertices() {
		held = math.Min(held, turned.Apply(v).Z)
	}
	side, err := r3.FromBasis(basis, r3.Vec{Z: delta/2 - held})
	require.NoError(t, err)
	// The true body's lowest point: the base cylinder's radius 8 against the
	// pose's world-Z row, whose Z entry is exactly zero.
	b := side.Basis()
	require.Zero(t, b.EZ.Z)
	row := new(big.Float).SetPrec(512).Mul(big.NewFloat(b.EX.Z), big.NewFloat(b.EX.Z))
	row.Add(row, new(big.Float).SetPrec(512).Mul(big.NewFloat(b.EY.Z), big.NewFloat(b.EY.Z)))
	row.Sqrt(row)
	truth := new(big.Float).SetPrec(512).SetFloat64(side.Translation().Z)
	truth.Sub(truth, row.Mul(row, big.NewFloat(8)))
	require.Negative(t, truth.Sign(), "the true base hangs its sagitta below the held facet into the floor")
	lying, err := doc.ContactPair(t.Context(), floor, bottle, r3.Identity(), side, req)
	require.NoError(t, err)
	requireGapHolds(t, lying, truth)
	require.Equal(t, decad.ContactBand, lying.Relation, "reason=%v", lying.Reason)

	// Every exact payload reads exactly through its own mesh.
	exact := map[string]*decad.Body{"cup": partsBinCup(t, doc), "loft": partsBinLoft(t, doc),
		"sweep": partsBinSweep(t, doc)}
	for name, body := range exact {
		report, err := doc.ContactPair(t.Context(), floor, body, r3.Identity(), up, req)
		require.NoError(t, err, name)
		require.Equal(t, decad.ContactSeparated, report.Relation, "%s reason=%v", name, report.Reason)
		require.Equal(t, 5.0, report.Gap.Value.Base(), name)
		require.Zero(t, report.Gap.Bound.Base(), name)
	}
	onFloor, err := doc.ContactPair(t.Context(), floor, exact["cup"], r3.Identity(),
		contactPose(t, r3.Vec{X: 3, Y: -2}), req)
	require.NoError(t, err)
	require.Equal(t, decad.ContactTouching, onFloor.Relation, "reason=%v", onFloor.Reason)

	// The 2.1 mm chamfer's feet are not dyadic: its δ is the rounding of its
	// offset contour.
	box := boxBody(t, doc, -6, -6, 6, 6, 12)
	block, err := box.Chamfer(t.Context(), capLoopEdges(box), units.Millimeters(2.1))
	require.NoError(t, err)
	chamfered, err := doc.ContactPair(t.Context(), floor, block, r3.Identity(), up, req)
	require.NoError(t, err)
	require.Equal(t, decad.ContactSeparated, chamfered.Relation, "reason=%v", chamfered.Reason)
	requireGapHolds(t, chamfered, big.NewFloat(5))
	require.Positive(t, chamfered.Gap.Bound.Base())
	require.Less(t, chamfered.Gap.Bound.Base(), 1e-12)
}

// TestHeldChordValidation rejects a negative, non-finite or non-Length
// HeldChord at both entry points. The sweep turns the block about a skew
// axis, a path SweepPair reports undecided before any ContactPair call, so
// its own check is the one that refuses. Each entry point's check was shown
// to fail: deleted, its cases return no error.
func TestHeldChordValidation(t *testing.T) {
	doc := decad.New()
	floor := boxBodyAtZ(t, doc, -100, -100, 100, 100, -10, 10)
	block := boxBody(t, doc, -6, -6, 6, 6, 12)
	still := decad.PoseSegment{From: r3.Identity(), To: r3.Identity(), Duration: units.Seconds(1)}
	skew := decad.PoseSegment{From: contactPose(t, r3.Vec{Z: 20}),
		To: rotationPose(t, r3.Vec{X: 1, Y: 1, Z: 1}, 30, r3.Vec{Z: 20}), Duration: units.Seconds(1)}
	sweep := tumbleRequest()
	unsupported, err := doc.SweepPair(t.Context(), floor, block, still, skew, sweep)
	require.NoError(t, err)
	require.Equal(t, decad.SweepContactUnsupported, unsupported.Cause)
	for _, tc := range []struct {
		name  string
		chord units.Value
		want  error
	}{
		{"negative", units.Millimeters(-.03), decad.ErrDegenerate},
		{"non-finite", units.Millimeters(math.Inf(1)), decad.ErrNotFinite},
		{"NaN", units.Millimeters(math.NaN()), decad.ErrNotFinite},
		{"not a Length", units.Degrees(1), decad.ErrUnitKind},
	} {
		req := contactRequest()
		req.HeldChord = tc.chord
		_, err := doc.ContactPair(t.Context(), floor, block, r3.Identity(), contactPose(t, r3.Vec{Z: 5}), req)
		require.ErrorIs(t, err, tc.want, tc.name)
		sweep.HeldChord = tc.chord
		_, err = doc.SweepPair(t.Context(), floor, block, still, skew, sweep)
		require.ErrorIs(t, err, tc.want, tc.name)
	}
}

// The fixtures of docs/multibody-dynamics-design.md §13 PR 20c: §10.4's rows
// for a displaced pair's lifted set and deep overlap, each run in both body
// orders over the §2 tray. A held separated pair within the lifted band
// b = max(SupportBand, δ) over which the tray, the zero-δ body, hosts a
// lifted set publishes ContactBand with Gap [0 ± (g + δ)] and that set; a
// held overlap deeper than δ publishes §9.6's face-local patch read over the
// held vertices grown by M's δ, then the lifted set, every point charged.
//
// §9.6 needs M's convexity certificate (§9.2), which the 2.1 mm block's held
// mesh does not carry: its rounded feet leave a vertex an ulp in front of a
// facet plane. The overlap fixtures use a 2.3 mm block, whose held mesh does,
// and TestDisplacedSupportSetChamferedBlock records the 2.1 mm block's
// refusal.
//
// Legs shown to fail (each deleted in turn, fixture red, then restored):
//   - δ on the lifted band's gap: TestDisplacedSupportSetBottle's bottle on
//     its side, a chord of its base lowest at a held height δ/10, publishes
//     [0 ± δ/10], which excludes the true base's lowest point, evaluated in
//     512-bit arithmetic a sagitta below the chord (its upright fixture's
//     bound check goes red first);
//   - the zero-δ host gate: TestDisplacedSupportSetBottle's bottle on the
//     2.1 mm block reads a lifted band hosted by the bottle's held base, whose
//     Gap.Bound g + δ lies below the held band's 2δ (chargedManifold's
//     exact-face check still withholds the normal read off that held face);
//   - b read from δ: with b read as SupportBand alone (0.01 mm, below the
//     upright bottle's held gap of 0.8·δ), TestDisplacedSupportSetBottle's
//     upright bottle publishes the held band [−2δ, 2δ] with no lifted set;
//   - §9.6's growth by M's δ, both conditions' growth deleted together:
//     TestDisplacedPenetrationWallMargin's block, whose held side stands δ/2
//     from the wall while it is sunk in the floor, publishes the floor's
//     patch. Either growth alone still refuses it (condition 3's sunk hull
//     reaches within δ of the floor's rim at the wall's foot, and condition
//     4's grown column meets the wall), so each was deleted alone as well and
//     the fixture stayed green; internal/pair's TestHullInsideRegionMargin
//     shows each of condition 3's distance checks red on its own;
//   - each witness ball's δ and each Separation's δ: the published bounds
//     fall below δ in TestDisplacedSupportSetChamferedBlock and
//     TestDisplacedSupportSetBottle.
//
// No fixture puts a true point outside a bare ball or Separation: every
// shipped tessellation samples its vertices on the true surface within
// rounding. The charges follow Bound's contract, which places the true
// boundary within δ of the held mesh and says nothing of where a held vertex
// lies, rather than any producer's behavior, as PR 14b records its vertBound
// term.

// tiltedBlock is a 12 mm block over [−6, 6]×[y0, y0+12] with its top loop
// chamfered by setback, and its displacement.
func tiltedBlock(t *testing.T, doc *decad.Document, y0, setback float64) (*decad.Body, float64) {
	t.Helper()
	box := boxBody(t, doc, -6, y0, 6, y0+12, 12)
	block, err := box.Chamfer(t.Context(), capLoopEdges(box), units.Millimeters(setback))
	require.NoError(t, err)
	mesh, err := block.Tessellate(t.Context(), units.Millimeters(1))
	require.NoError(t, err)
	delta := mesh.Bound().Base()
	require.Positive(t, delta, "the chamfer's feet are not dyadic")
	return block, delta
}

// tiltAboutY turns a body about Y with sin θ = s, then moves it by at. A
// body point (x, y, 0) lands at height at.Z − s·x, exactly.
func tiltAboutY(t *testing.T, s float64, at r3.Vec) r3.Transform {
	t.Helper()
	c := math.Sqrt(1 - s*s)
	return basisPose(t, r3.Vec{X: c, Z: -s}, r3.Vec{Y: 1}, r3.Vec{X: s, Z: c}, at)
}

// components lists a vector's coordinates.
func components(v r3.Vec) [3]float64 { return [3]float64{v.X, v.Y, v.Z} }

// stagedRat is the exact image of a body point under a pose.
func stagedRat(pose r3.Transform, p r3.Vec) [3]*big.Rat {
	basis, at := pose.Basis(), components(pose.Translation())
	columns := [3][3]float64{components(basis.EX), components(basis.EY), components(basis.EZ)}
	local := components(p)
	var out [3]*big.Rat
	for axis := range 3 {
		sum := new(big.Rat).SetFloat64(at[axis])
		for k := range 3 {
			sum.Add(sum, new(big.Rat).Mul(new(big.Rat).SetFloat64(columns[k][axis]), new(big.Rat).SetFloat64(local[k])))
		}
		out[axis] = sum
	}
	return out
}

// withinBall reports whether the exact point lies inside the witness's ball.
func withinBall(witness decad.VecMeasurement, exact [3]*big.Rat) bool {
	distance := new(big.Rat)
	value := components(witness.Value)
	for axis := range 3 {
		d := new(big.Rat).Sub(new(big.Rat).SetFloat64(value[axis]), exact[axis])
		distance.Add(distance, d.Mul(d, d))
	}
	bound := new(big.Rat).SetFloat64(witness.Bound.Base())
	return distance.Cmp(bound.Mul(bound, bound)) <= 0
}

// requireBlockCorners checks that each published point's block witness holds
// the exact image of one distinct bottom corner of the block over
// [−6, 6]×[−6, 6], with the block's δ in its ball, that its Separation is
// that corner's exact height with at least δ in its bound, and that its
// normal is the floor's +z. It returns the matched corners' heights in point
// order.
func requireBlockCorners(t *testing.T, points []decad.ContactPoint, pose r3.Transform, delta float64) []float64 {
	t.Helper()
	corners := []r3.Vec{{X: -6, Y: -6}, {X: 6, Y: -6}, {X: -6, Y: 6}, {X: 6, Y: 6}}
	used := make([]bool, len(corners))
	var heights []float64
	for _, point := range points {
		match := -1
		for i, corner := range corners {
			if !used[i] && withinBall(point.OnB, stagedRat(pose, corner)) {
				match = i
				break
			}
		}
		require.GreaterOrEqual(t, match, 0, "the witness %v holds no bottom corner", point.OnB.Value)
		used[match] = true
		height := stagedRat(pose, corners[match])[2]
		exact, ok := height.Float64()
		require.True(t, ok, "the fixtures place every corner at a float height")
		require.Equal(t, exact, point.Separation.Value.Base())
		require.GreaterOrEqual(t, point.Separation.Bound.Base(), delta)
		require.GreaterOrEqual(t, point.OnB.Bound.Base(), delta, "a held corner is no point of the true block")
		require.Equal(t, r3.Vec{Z: 1}, point.Normal.Value)
		heights = append(heights, exact)
	}
	return heights
}

func TestDisplacedSupportSetChamferedBlock(t *testing.T) {
	const (
		s    = 1.0 / (1 << 24)
		band = 1.0 / (1 << 20)
	)
	doc := decad.New()
	tray := sceneTrayBody(t, doc)
	block, delta := tiltedBlock(t, doc, -6, 2.1)

	// Lifted 2⁻³⁰ mm: the x = 6 edge's corners stand at 2⁻³⁰ and the
	// x = −6 edge's 12·2⁻²⁴ higher, all four inside the 2⁻²⁰ mm band.
	lifted := tiltAboutY(t, s, r3.Vec{Z: math.Ldexp(1, -30) + 6*s})
	report := trayCubeTwoWays(t, doc, tray, block, lifted, supportBandRequest(band))
	require.Equal(t, decad.ContactBand, report.Relation, "reason=%v", report.Reason)
	require.Equal(t, decad.ContactNoReason, report.Reason)
	require.Zero(t, report.Gap.Value.Base())
	// [0 ± (g + δ)]: the held gap is the lowest corner's exact height, and δ
	// at the pose is the block's times a stretch a hair over one.
	require.GreaterOrEqual(t, report.Gap.Bound.Base(), math.Ldexp(1, -30)+delta)
	require.LessOrEqual(t, report.Gap.Bound.Base(), math.Ldexp(1, -30)+2*delta)
	require.NotNil(t, report.Manifold)
	require.Len(t, report.Manifold.Points, 4)
	heights := requireBlockCorners(t, report.Manifold.Points, lifted, delta)
	slices.Sort(heights)
	require.Equal(t, []float64{math.Ldexp(1, -30), math.Ldexp(1, -30), 769 * math.Ldexp(1, -30),
		769 * math.Ldexp(1, -30)}, heights)

	// Sunk 2⁻²⁰ mm, the x = 6 edge pokes through the floor, a true overlap
	// at a depth far past δ. §9.6 needs M's convexity certificate, which the
	// 2.1 mm block's held mesh does not carry: its rounded feet fold a face
	// pair the wrong way by an ulp. The pair is Overlapping with no manifold.
	sunk := tiltAboutY(t, s, r3.Vec{Z: -band + 6*s})
	report = trayCubeTwoWays(t, doc, tray, block, sunk, supportBandRequest(band))
	require.Equal(t, decad.ContactOverlapping, report.Relation, "reason=%v", report.Reason)
	require.Nil(t, report.Manifold)
	require.Equal(t, decad.ContactNonConvex, report.Reason)

	// A 2.3 mm block, whose held mesh is certified convex, sunk the same:
	// §9.6 publishes its two deep corners at −2⁻²⁰ ± δ, then the x = −6
	// edge's corners, at −2⁻²², as its lifted set.
	convex, convexDelta := tiltedBlock(t, doc, -6, 2.3)
	report = trayCubeTwoWays(t, doc, tray, convex, sunk, supportBandRequest(band))
	require.Equal(t, decad.ContactOverlapping, report.Relation, "reason=%v", report.Reason)
	require.NotNil(t, report.Manifold, "reason=%v", report.Reason)
	require.Len(t, report.Manifold.Points, 4)
	heights = requireBlockCorners(t, report.Manifold.Points, sunk, convexDelta)
	require.Equal(t, []float64{-band, -band, -band / 4, -band / 4}, heights)
	for _, point := range report.Manifold.Points[:2] {
		require.NotNil(t, point.FeatureB.Edge, "the deepest feature is the edge")
	}
	for _, point := range report.Manifold.Points[2:] {
		require.NotNil(t, point.FeatureB.Vertex)
	}
}

func TestDisplacedPenetrationWallMargin(t *testing.T) {
	const s = 1.0 / (1 << 24)
	doc := decad.New()
	// The tray moved so its +y wall's inner face is the plane y = 0, the
	// floor's top z = 0 beside it.
	tray := sceneTrayBody(t, doc)
	trayPose := contactPose(t, r3.Vec{Y: -80})
	// A 2.3 mm block over [−6, 6]×[−12, 0], whose held mesh carries the
	// convexity certificate §9.6 needs of M. Turned about Y its x = 6 edge
	// sinks 2⁻²⁰ mm into the floor and its x = −6 edge 2⁻²² mm, and its
	// y = 0 side stays parallel to the wall at the pose's y.
	block, delta := tiltedBlock(t, doc, -12, 2.3)
	clearing := func(y float64) *decad.ContactReport {
		t.Helper()
		pose := tiltAboutY(t, s, r3.Vec{Y: y, Z: -math.Ldexp(1, -20) + 6*s})
		return contactBothWays(t, doc, tray, block, trayPose, pose, supportBandRequest(math.Ldexp(1, -20)))
	}

	// Clear of the wall by far more than δ, the patch publishes the deep
	// edge and then the lifted x = −6 edge.
	apart := clearing(-math.Ldexp(1, -40))
	require.Equal(t, decad.ContactOverlapping, apart.Relation)
	require.NotNil(t, apart.Manifold, "reason=%v", apart.Reason)
	require.Len(t, apart.Manifold.Points, 4)
	for _, point := range apart.Manifold.Points {
		require.Equal(t, r3.Vec{Z: 1}, point.Normal.Value)
	}

	// The held side δ/2 from the wall: the true block may reach the wall, a
	// second face, so the held patch claims nothing.
	near := clearing(-delta / 2)
	require.Equal(t, decad.ContactOverlapping, near.Relation)
	require.Nil(t, near.Manifold)
	require.Equal(t, decad.ContactNoNormalProof, near.Reason)
}

func TestDisplacedSupportSetBottle(t *testing.T) {
	const chord = .03
	doc := decad.New()
	tray := sceneTrayBody(t, doc)
	bottle := partsBinBottle(t, doc)
	mesh, err := bottle.Tessellate(t.Context(), units.Millimeters(chord))
	require.NoError(t, err)
	delta := mesh.Bound().Base()
	require.Greater(t, delta, .01, "the band reads δ, above the 0.01 mm SupportBand")
	req := supportBandRequest(.01)
	req.HeldChord = units.Millimeters(chord)
	req.PointResolution = units.Millimeters(.1)

	// Upright with its held base 0.8·δ up, every base vertex is lifted.
	up := .8 * delta
	upright := trayCubeTwoWays(t, doc, tray, bottle, contactPose(t, r3.Vec{Z: up}), req)
	require.Equal(t, decad.ContactBand, upright.Relation, "reason=%v", upright.Reason)
	require.Equal(t, decad.ContactNoReason, upright.Reason)
	require.GreaterOrEqual(t, upright.Gap.Bound.Base(), up+delta)
	require.Less(t, upright.Gap.Bound.Base(), up+delta*(1+1e-9))
	var base []r3.Vec
	for _, v := range mesh.Vertices() {
		if v.Z == 0 {
			base = append(base, r3.Vec{X: v.X, Y: v.Y, Z: up})
		}
	}
	require.Greater(t, len(base), 8)
	require.NotNil(t, upright.Manifold)
	require.Len(t, upright.Manifold.Points, len(base))
	for _, point := range upright.Manifold.Points {
		require.Contains(t, base, point.OnB.Value)
		require.Equal(t, r3.Vec{X: point.OnB.Value.X, Y: point.OnB.Value.Y}, point.OnA.Value)
		require.Equal(t, up, point.Separation.Value.Base())
		require.GreaterOrEqual(t, point.Separation.Bound.Base(), delta)
		require.GreaterOrEqual(t, point.OnB.Bound.Base(), delta)
		require.Zero(t, point.OnA.Bound.Base())
		require.Equal(t, r3.Vec{Z: 1}, point.Normal.Value)
	}

	// On its side, turned about its axis by half a chord step so the facet
	// between two adjacent base-rim vertices is lowest at a held height of
	// δ/10. The true base hangs its sagitta below that facet.
	var rim []float64
	for _, v := range mesh.Vertices() {
		if v.Z == 0 && math.Abs(math.Hypot(v.X, v.Y)-8) < 1e-9 {
			rim = append(rim, math.Atan2(v.Y, v.X))
		}
	}
	slices.Sort(rim)
	mid := (rim[0] + rim[1]) / 2
	sinPhi, cosPhi := -math.Cos(mid), -math.Sin(mid)
	basis := r3.Basis{EX: r3.Vec{X: cosPhi, Z: sinPhi}, EY: r3.Vec{X: -sinPhi, Z: cosPhi}, EZ: r3.Vec{Y: -1}}
	turned, err := r3.FromBasis(basis, r3.Vec{})
	require.NoError(t, err)
	held := math.Inf(1)
	for _, v := range mesh.Vertices() {
		held = math.Min(held, turned.Apply(v).Z)
	}
	side, err := r3.FromBasis(basis, r3.Vec{Z: delta/10 - held})
	require.NoError(t, err)
	b := side.Basis()
	row := new(big.Float).SetPrec(512).Mul(big.NewFloat(b.EX.Z), big.NewFloat(b.EX.Z))
	row.Add(row, new(big.Float).SetPrec(512).Mul(big.NewFloat(b.EY.Z), big.NewFloat(b.EY.Z)))
	row.Sqrt(row)
	truth := new(big.Float).SetPrec(512).SetFloat64(side.Translation().Z)
	truth.Sub(truth, row.Mul(row, big.NewFloat(8)))
	require.Negative(t, truth.Cmp(big.NewFloat(-delta/10)), "the sagitta reaches past the held gap below zero")
	lying := trayCubeTwoWays(t, doc, tray, bottle, side, req)
	require.Equal(t, decad.ContactBand, lying.Relation, "reason=%v", lying.Reason)
	require.NotNil(t, lying.Manifold, "reason=%v", lying.Reason)
	requireGapHolds(t, lying, truth)

	// On the 2.1 mm block, whose held mesh carries its own δ, neither body has
	// an exact face to host a lifted set: the pair is the held band
	// [−2δ, 2δ] with no manifold.
	block, blockDelta := tiltedBlock(t, doc, -6, 2.1)
	sum := new(big.Rat).Add(new(big.Rat).SetFloat64(delta), new(big.Rat).SetFloat64(blockDelta))
	gap, _ := new(big.Rat).Mul(sum, big.NewRat(4, 5)).Float64()
	stacked := contactBothWays(t, doc, block, bottle, r3.Identity(), contactPose(t, r3.Vec{Z: 12 + gap}), req)
	require.Equal(t, decad.ContactBand, stacked.Relation, "reason=%v", stacked.Reason)
	require.Nil(t, stacked.Manifold)
	require.Equal(t, decad.ContactNoNormalProof, stacked.Reason)
	require.Zero(t, stacked.Gap.Value.Base())
	require.GreaterOrEqual(t, new(big.Rat).SetFloat64(stacked.Gap.Bound.Base()).Cmp(new(big.Rat).Mul(sum, big.NewRat(2, 1))), 0)
}
