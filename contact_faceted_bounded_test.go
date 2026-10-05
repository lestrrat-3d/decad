package decad

import (
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
		require.GreaterOrEqual(t, contact.Gap.Bound.Base(), boundaryBound)
		require.Equal(t, Approximate, contact.Gap.Exactness)
		require.Greater(t, contact.Gap.Value.Base()-contact.Gap.Bound.Base(), 89.9)
		require.Nil(t, contact.Manifold)
	}
	nearFloor := internalOffsetBox(t, doc, -20, -20, 20, 20, -10,
		Distance{D: units.Millimeters(10), Dir: Along})
	near, err := doc.ContactPair(t.Context(), nearFloor, placed,
		r3.Identity(), r3.Identity(), req)
	require.NoError(t, err)
	require.Equal(t, ContactUndecided, near.Relation)
	shifted := facetedSweepPose(t, r3.Vec{X: 25})
	outside, err := doc.ContactPair(t.Context(), floor, placed,
		r3.Identity(), shifted, req)
	require.NoError(t, err)
	require.Equal(t, ContactUndecided, outside.Relation)
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
	require.Equal(t, SweepUndecided, near.Outcome)
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
	rotating, err := doc.SweepPair(t.Context(), floor, placed, still, rotation, req)
	require.NoError(t, err)
	require.Equal(t, SweepUndecided, rotating.Outcome)
	require.False(t, rotating.HasAffineReplayProof())
}
