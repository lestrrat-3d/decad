package decad

import (
	"context"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestSweepAuditAcceptsSeparatedStraightSpans(t *testing.T) {
	frame, err := r3.NewFrame(r3.Vec{}, r3.Vec{X: 1}, r3.Vec{Y: 1})
	require.NoError(t, err)
	profile := unitSquareProfile()
	doc := &Document{}
	spans := make([]sweepAuditSpan, 3)
	for i := range spans {
		body, buildErr := evalPrismContext(t.Context(), doc, producerID(i+1), prismPayload{
			profile: profile,
			frame:   frame,
			z0:      float64(i),
			z1:      float64(i + 1),
			xform:   r3.Identity(),
		}, newFreeformWork())
		require.NoError(t, buildErr)
		spans[i] = sweepAuditSpan{
			body:     body,
			startCap: sweepAuditFaceWithRole(t, body, roleCapStart),
			endCap:   sweepAuditFaceWithRole(t, body, roleCapEnd),
		}
	}

	require.NoError(t, auditCompositeSweep(t.Context(), spans))
}

func TestSweepAuditRejectsUncertifiedAdjacentAndRemotePairs(t *testing.T) {
	frame, err := r3.NewFrame(r3.Vec{}, r3.Vec{X: 1}, r3.Vec{Y: 1})
	require.NoError(t, err)
	profile := unitSquareProfile()
	doc := &Document{}
	build := func(z0, z1 float64) sweepAuditSpan {
		body, buildErr := evalPrismContext(t.Context(), doc, 1, prismPayload{
			profile: profile,
			frame:   frame,
			z0:      z0,
			z1:      z1,
			xform:   r3.Identity(),
		}, newFreeformWork())
		require.NoError(t, buildErr)
		return sweepAuditSpan{
			body:     body,
			startCap: sweepAuditFaceWithRole(t, body, roleCapStart),
			endCap:   sweepAuditFaceWithRole(t, body, roleCapEnd),
		}
	}

	adjacentOverlap := []sweepAuditSpan{build(0, 2), build(1, 3)}
	require.ErrorIs(t, auditCompositeSweep(t.Context(), adjacentOverlap), ErrUnsupported)

	remoteTouch := []sweepAuditSpan{build(0, 1), build(1, 2), build(2, 3)}
	remoteTouch[2].body.bounds = remoteTouch[0].body.bounds
	require.ErrorIs(t, auditCompositeSweep(t.Context(), remoteTouch), ErrUnsupported)
}

func TestSweepAuditBoxesRequireStrictBoundedSeparation(t *testing.T) {
	box := func(lower, upper, bound float64) Box {
		return Box{
			Min:   r3.Vec{X: lower},
			Max:   r3.Vec{X: upper},
			Bound: units.Millimeters(bound),
		}
	}
	require.True(t, sweepAuditBoxesStrictlySeparated(box(0, 1, 0), box(2, 3, 0)))
	require.False(t, sweepAuditBoxesStrictlySeparated(box(0, 1, 0), box(1, 2, 0)))
	require.False(t, sweepAuditBoxesStrictlySeparated(box(0, 1, 0.25), box(1.25, 2, 0)))
	require.True(t, sweepAuditBoxesStrictlySeparated(box(0, 1, 0.125), box(1.5, 2, 0.125)))
}

func TestSweepAuditEndpointSupportStopsAtHalfTurn(t *testing.T) {
	body := &Body{payload: revolvePayload{den: sweepDenotation{
		phi0: angleDenotation{rad: new(big.Rat), turn: new(big.Rat)},
		phi1: angleDenotation{rad: new(big.Rat), turn: big.NewRat(1, 2)},
	}}}
	require.True(t, sweepAuditEndpointSupports(body))

	payload := body.payload.(revolvePayload)
	payload.den.phi1.turn = big.NewRat(3, 4)
	body.payload = payload
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
