package decad

import (
	"context"
	"testing"

	"github.com/stretchr/testify/require"
)

func TestBodyGateDiameterCancellation(t *testing.T) {
	doc := New()
	// An analytic prism, not a faceted body: bodyGateDiameter builds its carrier
	// model, which is the path that must observe cancellation.
	body := internalBoxBody(t, doc, 0, 0, 10, 10, 5)

	// A live context returns the diameter exactly, with no error — the path a
	// valid caller observes, byte-identical to before the fix.
	d, ok, err := bodyGateDiameter(t.Context(), body)
	require.NoError(t, err)
	require.True(t, ok)
	require.Positive(t, d)

	// A cancelled context is observed during the build rather than after the
	// whole carrier model finishes (the non-cancellable newBodyGeom gap).
	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	_, _, err = bodyGateDiameter(ctx, body)
	require.ErrorIs(t, err, context.Canceled)
}
