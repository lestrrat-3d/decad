package decad

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/contactfaceted"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func boundedFacetedFloorFixture(t *testing.T) (*Document, *Body, *Body) {
	t.Helper()
	doc, _, union := facetedFloorSweepFixture(t)
	shift := facetedSweepPose(t, r3.Vec{X: .1})
	placed, err := Transform(t.Context(), union, shift)
	require.NoError(t, err)
	pp, ok := placed.n.Payload.(facetedPayload)
	require.True(t, ok)
	require.Positive(t, pp.MeshBound)
	floor := internalOffsetBox(t, doc, -20, -20, 20, 20, -100,
		Distance{D: units.Millimeters(10), Dir: Along})
	return doc, floor, placed
}

func TestContactPairBoundedFacetedFloorStrictClear(t *testing.T) {
	doc, floor, placed := boundedFacetedFloorFixture(t)
	boundaryBound := placed.n.Payload.(facetedPayload).MeshBound
	req := facetedSweepRequest(StopAtInitialContact).ContactRequest
	for _, bodies := range [][2]*Body{{floor, placed}, {placed, floor}} {
		contact, err := doc.ContactPair(t.Context(), bodies[0], bodies[1],
			r3.Identity(), r3.Identity(), req)
		require.NoError(t, err)
		require.Equal(t, ContactSeparated, contact.Relation, "reason=%v", contact.Reason)
		require.NotNil(t, contact.Gap)
		require.InDelta(t, 90, contact.Gap.Value.Base(), 1e-12)
		require.Zero(t, contact.Gap.Bound.Base())
		require.Equal(t, Exact, contact.Gap.Exactness)
		require.Greater(t, contact.Gap.Value.Base()-contact.Gap.Bound.Base(), 89.9)
		require.Nil(t, contact.Manifold)
	}
	require.Positive(t, boundaryBound)
	nearFloor := internalOffsetBox(t, doc, -20, -20, 20, 20, -10,
		Distance{D: units.Millimeters(10), Dir: Along})
	near, err := doc.ContactPair(t.Context(), nearFloor, placed,
		r3.Identity(), r3.Identity(), req)
	require.NoError(t, err)
	require.Equal(t, ContactTouching, near.Relation)
	require.Len(t, near.Manifold.Points, 4)
	shifted := facetedSweepPose(t, r3.Vec{X: 25})
	outside, err := doc.ContactPair(t.Context(), floor, placed,
		r3.Identity(), shifted, req)
	require.NoError(t, err)
	// The floor-support path refuses a footprint off the floor; the exact
	// planar relation reads the translated source mesh instead: the floor's
	// top edge to the union's lower edge, 5.1 mm across and 90 mm down.
	require.Equal(t, ContactSeparated, outside.Relation, "reason=%v", outside.Reason)
	// 1e-9 mm is slack for the float reference value, not a bound figure.
	require.InDelta(t, math.Hypot(5.1, 90), outside.Gap.Value.Base(), 1e-9)
	require.Nil(t, outside.Manifold)
}

func TestContactPairPlacedFacetedFloorExactTouch(t *testing.T) {
	doc, _, placed := boundedFacetedFloorFixture(t)
	pp := placed.n.Payload.(facetedPayload)
	require.Positive(t, pp.MeshBound)
	require.Positive(t, pp.VolSymDiff)
	floor := internalOffsetBox(t, doc, -20, -20, 20, 20, -10,
		Distance{D: units.Millimeters(10), Dir: Along})
	req := facetedSweepRequest(StopAtInitialContact).ContactRequest
	for _, reversed := range []bool{false, true} {
		a, b := floor, placed
		if reversed {
			a, b = placed, floor
		}
		contact, err := doc.ContactPair(t.Context(), a, b, r3.Identity(), r3.Identity(), req)
		require.NoError(t, err)
		require.Equal(t, ContactTouching, contact.Relation, "reason=%v", contact.Reason)
		require.NotNil(t, contact.Manifold)
		require.Len(t, contact.Manifold.Points, 4)
		for _, point := range contact.Manifold.Points {
			facetedFace := point.FaceB
			if reversed {
				facetedFace = point.FaceA
			}
			require.Contains(t, placed.Faces(), facetedFace)
			require.Zero(t, point.Separation.Value.Base())
			require.LessOrEqual(t, point.OnA.Bound.Base(), req.PointResolution.Base())
			require.LessOrEqual(t, point.OnB.Bound.Base(), req.PointResolution.Base())
		}
	}
}

func TestSweepPairBoundedFacetedFloorStrictClearReplay(t *testing.T) {
	doc, floor, placed := boundedFacetedFloorFixture(t)
	still := facetedSweepPath(r3.Identity(), r3.Identity())
	move := facetedSweepPath(r3.Identity(), facetedSweepPose(t, r3.Vec{Z: -.5}))
	req := facetedSweepRequest(StopAtInitialContact)
	for _, reversed := range []bool{false, true} {
		a, b, pa, pb := floor, placed, PairPath(still), PairPath(move)
		if reversed {
			a, b, pa, pb = placed, floor, move, still
		}
		sweep, err := doc.SweepPair(t.Context(), a, b, pa, pb, req)
		require.NoError(t, err)
		require.Equal(t, SweepClear, sweep.Outcome, "cause=%v", sweep.Cause)
		require.True(t, sweep.HasAffineReplayProof())
		require.Len(t, sweep.Samples, 2)
		for _, elapsed := range []float64{0, .3, 1} {
			poseA, poseB, err := sweep.CertifiedPosesAt(units.Seconds(elapsed))
			require.NoError(t, err)
			contact, err := doc.ContactPair(t.Context(), a, b, poseA, poseB, req.ContactRequest)
			require.NoError(t, err)
			require.Equal(t, ContactSeparated, contact.Relation)
		}
	}
	nearFloor := internalOffsetBox(t, doc, -20, -20, 20, 20, -10,
		Distance{D: units.Millimeters(10), Dir: Along})
	near, err := doc.SweepPair(t.Context(), nearFloor, placed, still, move, req)
	require.NoError(t, err)
	require.Equal(t, SweepInitiallyTouching, near.Outcome)
	require.False(t, near.HasAffineReplayProof())
	transverse := facetedSweepPath(r3.Identity(), facetedSweepPose(t, r3.Vec{X: 1}))
	lateral, err := doc.SweepPair(t.Context(), floor, placed, still, transverse, req)
	require.NoError(t, err)
	require.Equal(t, SweepUndecided, lateral.Outcome)
	require.False(t, lateral.HasAffineReplayProof())
	rotation := RigidDriftSegment{From: r3.Identity(), Center: r3.Vec{},
		LinearVelocity: QuantityVec{X: units.MillimetersPerSecond(0),
			Y: units.MillimetersPerSecond(0), Z: units.MillimetersPerSecond(0)},
		AngularVelocity: QuantityVec{X: units.RadiansPerSecond(0),
			Y: units.RadiansPerSecond(0), Z: units.RadiansPerSecond(1)},
		Duration: units.Seconds(1)}
	// The placed union is also an exact planar solid, so the spin takes the
	// general rotating sweep (agent-docs/multibody-dynamics-design.md §10.1), which
	// certifies the clear path but keeps no replay proof.
	rotating, err := doc.SweepPair(t.Context(), floor, placed, still, rotation, req)
	require.NoError(t, err)
	require.Equal(t, SweepClear, rotating.Outcome, "cause=%v", rotating.Cause)
	require.False(t, rotating.HasAffineReplayProof())
	for _, sample := range rotating.Samples {
		require.Equal(t, ContactSeparated, sample.Ideal.Relation)
		require.Greater(t, sample.Ideal.Gap.Value.Base()-sample.Ideal.Gap.Bound.Base(), 0.0)
	}
}

func TestSweepPairPlacedFacetedFloorImpactAndDeparture(t *testing.T) {
	doc, _, placed := boundedFacetedFloorFixture(t)
	floor := internalOffsetBox(t, doc, -20, -20, 20, 20, -10,
		Distance{D: units.Millimeters(10), Dir: Along})
	still := facetedSweepPath(r3.Identity(), r3.Identity())
	start := facetedSweepPose(t, r3.Vec{Z: 10})
	end := facetedSweepPose(t, r3.Vec{Z: -10})
	up := facetedSweepPose(t, r3.Vec{Z: 5})
	for _, reversed := range []bool{false, true} {
		a, b := floor, placed
		impactA, impactB := PairPath(still), PairPath(facetedSweepPath(start, end))
		departA, departB := PairPath(still), PairPath(facetedSweepPath(r3.Identity(), up))
		if reversed {
			a, b = placed, floor
			impactA, impactB = impactB, impactA
			departA, departB = departB, departA
		}
		impact, err := doc.SweepPair(t.Context(), a, b, impactA, impactB,
			facetedSweepRequest(StopAtInitialContact))
		require.NoError(t, err)
		require.Equal(t, SweepImpactBracket, impact.Outcome, "cause=%v", impact.Cause)
		require.True(t, impact.HasAffineReplayProof())
		require.InDelta(t, .5, impact.Event.At.Fraction.Base(), 1e-12)
		require.Equal(t, ContactTouching, impact.Event.Relation)
		require.Len(t, impact.Event.Manifold.Points, 4)
		for _, point := range impact.Event.Manifold.Points {
			face := point.FaceB
			if reversed {
				face = point.FaceA
			}
			require.Contains(t, placed.Faces(), face)
		}
		for _, at := range []float64{0, .25, .5} {
			poseA, poseB, replayErr := impact.CertifiedPosesAt(units.Seconds(at))
			require.NoError(t, replayErr)
			contact, contactErr := doc.ContactPair(t.Context(), a, b, poseA, poseB,
				facetedSweepRequest(StopAtInitialContact).ContactRequest)
			require.NoError(t, contactErr)
			if at < .5 {
				require.Equal(t, ContactSeparated, contact.Relation)
			} else {
				require.Equal(t, ContactTouching, contact.Relation)
			}
		}
		departure, err := doc.SweepPair(t.Context(), a, b, departA, departB,
			facetedSweepRequest(ContinueSeparatingTouch))
		require.NoError(t, err)
		require.Equal(t, SweepDepartedClear, departure.Outcome, "cause=%v", departure.Cause)
		require.True(t, departure.HasAffineReplayProof())
		for _, at := range []float64{0, .25, 1} {
			_, _, replayErr := departure.CertifiedPosesAt(units.Seconds(at))
			require.NoError(t, replayErr)
		}
	}
	persistent, err := doc.SweepPair(t.Context(), floor, placed, still, still,
		facetedSweepRequest(ContinueCertifiedTouch))
	require.NoError(t, err)
	require.Equal(t, SweepPersistentTouch, persistent.Outcome)
	require.True(t, persistent.HasAffineReplayProof())
}

func positiveBoundFacetedFixture(t *testing.T, reverse bool) (*Document, *Body, *Body) {
	t.Helper()
	doc := New()
	base := internalBoxBody(t, doc, -5, -5, 5, 5, 10)
	upper := internalDiscBody(t, doc, 2, 4)
	shift, err := r3.Translation(r3.Vec{Z: 8})
	require.NoError(t, err)
	upper, err = Transform(t.Context(), upper, shift)
	require.NoError(t, err)
	if reverse {
		base, upper = upper, base
	}
	union, err := Union(t.Context(), base, upper)
	require.NoError(t, err)
	pp, ok := union.n.Payload.(facetedPayload)
	require.True(t, ok)
	require.Positive(t, pp.MeshBound)
	require.Positive(t, pp.VolSymDiff)
	require.Empty(t, pp.ExactSourceVerts)
	floor := internalOffsetBox(t, doc, -20, -20, 20, 20, -10,
		Distance{D: units.Millimeters(10), Dir: Along})
	return doc, floor, union
}

func TestPositiveBoundFacetedUnionRetainsExactLowerFace(t *testing.T) {
	for _, reverse := range []bool{false, true} {
		t.Run(map[bool]string{false: "box first", true: "cap first"}[reverse], func(t *testing.T) {
			positiveBoundFacetedUnionRetainsExactLowerFace(t, reverse)
		})
	}
}

func positiveBoundFacetedUnionRetainsExactLowerFace(t *testing.T, reverse bool) {
	t.Helper()
	doc, floor, body := positiveBoundFacetedFixture(t, reverse)
	pp := body.n.Payload.(facetedPayload)
	require.NotNil(t, pp.LowerSupport)
	mass, err := body.MassProperties(t.Context(), units.KilogramsPerCubicMillimeter(.001))
	require.NoError(t, err)
	require.Positive(t, mass.Mass.Bound.Base())
	require.Positive(t, mass.Center.Bound.Base())
	contact, err := doc.ContactPair(t.Context(), floor, body, r3.Identity(), r3.Identity(),
		ContactRequest{PointResolution: units.Millimeters(1e-6), NormalResolution: units.Radians(1e-6)})
	require.NoError(t, err)
	require.Equal(t, ContactTouching, contact.Relation)
	require.NotNil(t, contact.Manifold)
	require.Len(t, contact.Manifold.Points, 4)
	for _, point := range contact.Manifold.Points {
		require.Contains(t, body.Faces(), point.FaceB)
		require.Equal(t, r3.Vec{Z: 1}, point.Normal.Value)
	}
	shift, err := r3.Translation(r3.Vec{X: .1})
	require.NoError(t, err)
	placed, err := Transform(t.Context(), body, shift)
	require.NoError(t, err)
	require.Nil(t, placed.n.Payload.(facetedPayload).LowerSupport)
	uncertain, err := doc.ContactPair(t.Context(), floor, placed, r3.Identity(), r3.Identity(),
		ContactRequest{PointResolution: units.Millimeters(1e-6), NormalResolution: units.Radians(1e-6)})
	require.NoError(t, err)
	// Without the exact support certificate the held mesh decides the pair,
	// and its mesh bound turns the held touch into the §10.4 band.
	require.Equal(t, ContactBand, uncertain.Relation)
	require.Equal(t, 2*placed.n.Payload.(facetedPayload).MeshBound, uncertain.Gap.Bound.Base())
}

func TestPositiveBoundFacetedUnionRefusesToolBelowLowerFace(t *testing.T) {
	doc := New()
	base := internalBoxBody(t, doc, -5, -5, 5, 5, 10)
	upper := internalDiscBody(t, doc, 2, 4)
	shift, err := r3.Translation(r3.Vec{Z: -1})
	require.NoError(t, err)
	upper, err = Transform(t.Context(), upper, shift)
	require.NoError(t, err)
	union, err := Union(t.Context(), base, upper)
	require.NoError(t, err)
	pp := union.n.Payload.(facetedPayload)
	require.Positive(t, pp.MeshBound)
	require.Nil(t, pp.LowerSupport)
	_, proven, err := contactfaceted.AxisSupportAtPose(t.Context(), &union.n, r3.Identity())
	require.NoError(t, err)
	require.False(t, proven)
}

// internalCrossingFramedBox extrudes the 10 mm box spanning x, y = 5..15 and
// z = 4..12, drawn on a plane whose origin is the box's own corner (5, 5, 4).
// The corner offset is in the plane, so the box re-expresses into an
// XY-sketched partner's frame as a translation that is not the identity, and
// A1's brep build declines the pair (internal/stackedunion/brep.go): its union with
// the 10 mm box at the origin crosses that box's outline and takes the mesh
// path, which is the faceted result these fixtures are about. Every
// coordinate is exact.
func internalCrossingFramedBox(t *testing.T, doc *Document) *Body {
	t.Helper()
	w := sketch.NewWorld()
	frame, err := r3.NewFrame(r3.NewVec(5, 5, 4), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	plane, err := w.CreatePlaneFromFrame(frame)
	require.NoError(t, err)
	s, err := w.CreateSketch(plane)
	require.NoError(t, err)
	rect := s.CreateRectangle(0, 0, 10, 10)
	s.Fix(rect.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	body, err := Extrude(t.Context(), doc, s, s.Profiles()[0], Distance{D: units.Millimeters(8), Dir: Along})
	require.NoError(t, err)
	return body
}

func facetedFloorSweepFixture(t *testing.T) (*Document, *Body, *Body) {
	t.Helper()
	doc := New()
	a := internalBoxBody(t, doc, 0, 0, 10, 10, 10)
	b := internalCrossingFramedBox(t, doc)
	union, err := Union(t.Context(), a, b)
	require.NoError(t, err)
	_, faceted := union.n.Payload.(facetedPayload)
	require.True(t, faceted)
	floor := internalOffsetBox(t, doc, -20, -20, 20, 20, -10,
		Distance{D: units.Millimeters(10), Dir: Along})
	return doc, floor, union
}

func facetedSweepRequest(policy SweepStartPolicy) SweepRequest {
	return SweepRequest{
		ContactRequest: ContactRequest{
			PointResolution:  units.Millimeters(1e-6),
			NormalResolution: units.Degrees(1),
		},
		TimeResolution: units.Seconds(1e-6), MaxPoseEvaluations: 8, StartPolicy: policy,
	}
}

func facetedSweepPose(t *testing.T, v r3.Vec) r3.Transform {
	t.Helper()
	pose, err := r3.Translation(v)
	require.NoError(t, err)
	return pose
}

func facetedSweepPath(from, to r3.Transform) PoseSegment {
	return PoseSegment{From: from, To: to, Duration: units.Seconds(1)}
}

func TestSweepPairRealFacetedUnionClearAndImpact(t *testing.T) {
	doc, floor, union := facetedFloorSweepFixture(t)
	before := doc.Bodies()
	still := facetedSweepPath(r3.Identity(), r3.Identity())
	start := facetedSweepPose(t, r3.Vec{Z: 10})
	contact, err := doc.ContactPair(t.Context(), floor, union, r3.Identity(), start,
		facetedSweepRequest(StopAtInitialContact).ContactRequest)
	require.NoError(t, err)
	require.Equal(t, ContactSeparated, contact.Relation)
	require.Equal(t, units.Millimeters(10), contact.Gap.Value)

	clearEnd := facetedSweepPose(t, r3.Vec{Z: 5})
	clearSweep, err := doc.SweepPair(t.Context(), floor, union, still,
		facetedSweepPath(start, clearEnd), facetedSweepRequest(StopAtInitialContact))
	require.NoError(t, err)
	require.Equal(t, SweepClear, clearSweep.Outcome, "cause=%v", clearSweep.Cause)
	require.True(t, clearSweep.HasAffineReplayProof())
	for _, elapsed := range []float64{0, 0.25, 0.5, 1} {
		poseA, poseB, err := clearSweep.CertifiedPosesAt(units.Seconds(elapsed))
		require.NoError(t, err)
		at, err := doc.ContactPair(t.Context(), floor, union, poseA, poseB,
			facetedSweepRequest(StopAtInitialContact).ContactRequest)
		require.NoError(t, err)
		require.Equal(t, ContactSeparated, at.Relation)
	}

	end := facetedSweepPose(t, r3.Vec{Z: -10})
	impact, err := doc.SweepPair(t.Context(), floor, union, still,
		facetedSweepPath(start, end), facetedSweepRequest(StopAtInitialContact))
	require.NoError(t, err)
	require.Equal(t, SweepImpactBracket, impact.Outcome, "cause=%v", impact.Cause)
	require.NotNil(t, impact.Bracket)
	require.Equal(t, 0.5, impact.Bracket.To.Fraction.Base())
	require.Equal(t, ContactSeparated, impact.Samples[0].Ideal.Relation)
	require.Equal(t, ContactTouching, impact.Event.Relation)
	require.Len(t, impact.Event.Manifold.Points, 4)
	proof, ok, err := contactfaceted.AxisSupportAtPose(t.Context(), &union.n, r3.Identity())
	require.NoError(t, err)
	require.True(t, ok)
	for _, point := range impact.Event.Manifold.Points {
		require.Same(t, faceHandle(proof.Face), point.FaceB)
		require.Equal(t, r3.Vec{Z: 1}, point.Normal.Value)
	}
	for _, elapsed := range []float64{0, 0.1, 0.25, 0.5} {
		poseA, poseB, err := impact.CertifiedPosesAt(units.Seconds(elapsed))
		require.NoError(t, err)
		at, err := doc.ContactPair(t.Context(), floor, union, poseA, poseB,
			facetedSweepRequest(StopAtInitialContact).ContactRequest)
		require.NoError(t, err)
		if elapsed == 0.5 {
			require.Equal(t, ContactTouching, at.Relation)
		} else {
			require.Equal(t, ContactSeparated, at.Relation)
		}
	}
	_, _, err = impact.CertifiedPosesAt(units.Seconds(0.75))
	require.ErrorIs(t, err, ErrUnsupported)
	require.Equal(t, before, doc.Bodies())
}

func TestSweepPairRealFacetedUnionTouchContinuations(t *testing.T) {
	doc, floor, union := facetedFloorSweepFixture(t)
	still := facetedSweepPath(r3.Identity(), r3.Identity())
	up := facetedSweepPose(t, r3.Vec{Z: 5})
	depart, err := doc.SweepPair(t.Context(), union, floor,
		facetedSweepPath(r3.Identity(), up), still,
		facetedSweepRequest(ContinueSeparatingTouch))
	require.NoError(t, err)
	require.Equal(t, SweepDepartedClear, depart.Outcome, "cause=%v", depart.Cause)
	require.NotNil(t, depart.Departure)
	require.Equal(t, units.Millimeters(5), depart.Departure.GapAtUntil.Value)
	for _, elapsed := range []float64{0, 0.25, 1} {
		poseA, poseB, err := depart.CertifiedPosesAt(units.Seconds(elapsed))
		require.NoError(t, err)
		at, err := doc.ContactPair(t.Context(), union, floor, poseA, poseB,
			facetedSweepRequest(ContinueSeparatingTouch).ContactRequest)
		require.NoError(t, err)
		if elapsed == 0 {
			require.Equal(t, ContactTouching, at.Relation)
		} else {
			require.Equal(t, ContactSeparated, at.Relation)
		}
	}

	shift := facetedSweepPose(t, r3.Vec{X: 0.1})
	track, err := doc.SweepPair(t.Context(), floor, union,
		facetedSweepPath(r3.Identity(), shift),
		facetedSweepPath(r3.Identity(), shift),
		facetedSweepRequest(ContinueCertifiedTouch))
	require.NoError(t, err)
	require.Equal(t, SweepPersistentTouch, track.Outcome, "cause=%v", track.Cause)
	require.NotNil(t, track.ContactTrack)
	for _, fraction := range []float64{0, 0.25, 0.5, 1} {
		manifold, err := track.ContactTrack.ManifoldAt(units.Scalar(fraction))
		require.NoError(t, err)
		require.Len(t, manifold.Points, 4)
		poseA, poseB, err := track.CertifiedPosesAt(units.Seconds(fraction))
		require.NoError(t, err)
		at, err := doc.ContactPair(t.Context(), floor, union, poseA, poseB,
			facetedSweepRequest(ContinueCertifiedTouch).ContactRequest)
		require.NoError(t, err)
		require.Equal(t, ContactTouching, at.Relation)
	}
}

func TestSweepPairFacetedFloorRefusesUnprovedPath(t *testing.T) {
	doc, floor, union := facetedFloorSweepFixture(t)
	still := facetedSweepPath(r3.Identity(), r3.Identity())
	start := facetedSweepPose(t, r3.Vec{Z: 10})
	for _, end := range []r3.Vec{{X: 1, Z: -10}, {Z: -7}} {
		pose := facetedSweepPose(t, end)
		report, err := doc.SweepPair(t.Context(), floor, union, still,
			facetedSweepPath(start, pose), facetedSweepRequest(StopAtInitialContact))
		require.NoError(t, err)
		require.Equal(t, SweepUndecided, report.Outcome, "end=%v", end)
		require.False(t, report.HasAffineReplayProof())
	}
	edgeFloor := internalOffsetBox(t, doc, 0, 0, 10, 10, -10,
		Distance{D: units.Millimeters(10), Dir: Along})
	edge, err := doc.SweepPair(t.Context(), edgeFloor, union, still, still,
		facetedSweepRequest(ContinueCertifiedTouch))
	require.NoError(t, err)
	require.Equal(t, SweepUndecided, edge.Outcome)
	frame, err := r3.NewFrame(r3.Vec{}, r3.Vec{Y: 1}, r3.Vec{Z: 1})
	require.NoError(t, err)
	reflection, err := r3.Reflection(frame)
	require.NoError(t, err)
	reflected, err := Transform(t.Context(), union, reflection)
	require.NoError(t, err)
	require.Positive(t, reflected.n.Payload.(facetedPayload).MeshBound)
	require.Empty(t, reflected.n.Payload.(facetedPayload).ExactSourceVerts)
	unproved, err := doc.SweepPair(t.Context(), floor, reflected, still,
		facetedSweepPath(start, facetedSweepPose(t, r3.Vec{Z: -10})),
		facetedSweepRequest(StopAtInitialContact))
	require.NoError(t, err)
	require.Equal(t, SweepUndecided, unproved.Outcome)
	require.False(t, unproved.HasAffineReplayProof())
	clearReport, err := doc.SweepPair(t.Context(), floor, reflected, still,
		facetedSweepPath(start, facetedSweepPose(t, r3.Vec{Z: 5})),
		facetedSweepRequest(StopAtInitialContact))
	require.NoError(t, err)
	require.Equal(t, SweepClear, clearReport.Outcome)
	require.True(t, clearReport.HasAffineReplayProof())
	_, _, err = clearReport.CertifiedPosesAt(units.Seconds(.5))
	require.NoError(t, err)
}
