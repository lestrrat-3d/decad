package decad

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func boundedFacetedFloorFixture(t *testing.T) (*Document, *Body, *Body) {
	t.Helper()
	doc, _, union := facetedFloorSweepFixture(t)
	shift := facetedSweepPose(t, r3.Vec{X: .1})
	placed, err := union.Placed(t.Context(), shift)
	require.NoError(t, err)
	pp, ok := placed.payload.(facetedPayload)
	require.True(t, ok)
	require.Positive(t, pp.meshBound)
	floor := internalOffsetBox(t, doc, -20, -20, 20, 20, -100,
		Distance{D: units.Millimeters(10), Dir: Along})
	return doc, floor, placed
}

func TestContactPairBoundedFacetedFloorStrictClear(t *testing.T) {
	doc, floor, placed := boundedFacetedFloorFixture(t)
	boundaryBound := placed.payload.(facetedPayload).meshBound
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
	pp := placed.payload.(facetedPayload)
	require.Positive(t, pp.meshBound)
	require.Positive(t, pp.volSymDiff)
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
	// general rotating sweep (docs/multibody-dynamics-design.md §10.1), which
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
