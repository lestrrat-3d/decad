package decad

import (
	"context"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/compositesweep"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/revolveangle"
	"github.com/lestrrat-3d/decad/internal/topo"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestSweepAuditAcceptsSeparatedStraightSpans(t *testing.T) {
	frame, err := r3.NewFrame(r3.Vec{}, r3.Vec{X: 1}, r3.Vec{Y: 1})
	require.NoError(t, err)
	profile := unitSquareProfile()
	doc := bornDocument(&Document{})
	spans := make([]sweepAuditSpan, 3)
	for i := range spans {
		body, buildErr := bodyResult(prismrecord.Build(t.Context(), envOf(documentNode(doc), producerID(i+1)), prismrecord.Payload{
			Profile: profile,
			Frame:   frame,
			Z0:      float64(i),
			Z1:      float64(i + 1),
			Xform:   r3.Identity(),
		}, freeform.NewFreeformWork()))
		require.NoError(t, buildErr)
		spans[i] = sweepAuditSpan{
			body:     body,
			startCap: sweepAuditFaceWithRole(t, body, provenance.RoleCapStart),
			endCap:   sweepAuditFaceWithRole(t, body, provenance.RoleCapEnd),
		}
	}

	require.NoError(t, auditCompositeSweep(t.Context(), spans))
}

func TestSweepAuditRejectsUncertifiedAdjacentAndRemotePairs(t *testing.T) {
	frame, err := r3.NewFrame(r3.Vec{}, r3.Vec{X: 1}, r3.Vec{Y: 1})
	require.NoError(t, err)
	profile := unitSquareProfile()
	doc := bornDocument(&Document{})
	build := func(z0, z1 float64) sweepAuditSpan {
		body, buildErr := bodyResult(prismrecord.Build(t.Context(), envOf(documentNode(doc), 1), prismrecord.Payload{
			Profile: profile,
			Frame:   frame,
			Z0:      z0,
			Z1:      z1,
			Xform:   r3.Identity(),
		}, freeform.NewFreeformWork()))
		require.NoError(t, buildErr)
		return sweepAuditSpan{
			body:     body,
			startCap: sweepAuditFaceWithRole(t, body, provenance.RoleCapStart),
			endCap:   sweepAuditFaceWithRole(t, body, provenance.RoleCapEnd),
		}
	}

	adjacentOverlap := []sweepAuditSpan{build(0, 2), build(1, 3)}
	require.ErrorIs(t, auditCompositeSweep(t.Context(), adjacentOverlap), ErrUnsupported)

	remoteTouch := []sweepAuditSpan{build(0, 1), build(1, 2), build(2, 3)}
	remoteTouch[2].body.n.Bounds = remoteTouch[0].body.n.Bounds
	require.ErrorIs(t, auditCompositeSweep(t.Context(), remoteTouch), ErrUnsupported)
}

func TestSweepAuditBoxesRequireStrictBoundedSeparation(t *testing.T) {
	box := func(lower, upper, bound float64) compositesweep.AuditBox {
		return compositesweep.AuditBox{
			Min:   r3.Vec{X: lower},
			Max:   r3.Vec{X: upper},
			Bound: units.Millimeters(bound).Base(),
		}
	}
	require.True(t, compositesweep.BoxesStrictlySeparated(box(0, 1, 0), box(2, 3, 0)))
	require.False(t, compositesweep.BoxesStrictlySeparated(box(0, 1, 0), box(1, 2, 0)))
	require.False(t, compositesweep.BoxesStrictlySeparated(box(0, 1, 0.25), box(1.25, 2, 0)))
	require.True(t, compositesweep.BoxesStrictlySeparated(box(0, 1, 0.125), box(1.5, 2, 0.125)))
}

func TestSweepAuditEndpointSupportStopsAtHalfTurn(t *testing.T) {
	body := bornBody(&Body{n: topo.Body{
		Payload: revolvePayload{Den: revolveangle.Sweep{
			Phi0: revolveangle.Angle{Rad: new(big.Rat), Turn: new(big.Rat)},
			Phi1: revolveangle.Angle{Rad: new(big.Rat), Turn: big.NewRat(1, 2)},
		}},
	}})
	require.True(t, sweepAuditEndpointSupports(body))

	payload := body.n.Payload.(revolvePayload)
	payload.Den.Phi1.Turn = big.NewRat(3, 4)
	body.n.Payload = payload
	require.False(t, sweepAuditEndpointSupports(body))
}

func TestSweepAuditHonorsCanceledContext(t *testing.T) {
	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	require.ErrorIs(t, auditCompositeSweep(ctx, []sweepAuditSpan{{}, {}}), context.Canceled)
}

func sweepAuditFaceWithRole(t *testing.T, body *Body, role string) *Face {
	t.Helper()
	for _, face := range body.Faces() {
		for _, origin := range face.Origins() {
			if origin.Role == role {
				return face
			}
		}
	}
	t.Fatalf("body has no face with role %q", role)
	return nil
}
