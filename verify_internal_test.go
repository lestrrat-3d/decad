package decad

import (
	"context"
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/r3"
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

// requireCertifiedDiameter judges a published gate diameter against the EXACT
// squared distance of the pair it is meant to report, in math/big and never in
// float64: the published value's own square must not exceed that square (the
// certification docs/verification-design.md §3 requires), and the next float
// UP must exceed it (so the reading is the tightest float64 carrying that
// certification, not an arbitrary smaller number).
func requireCertifiedDiameter(t *testing.T, published float64, exactSquare *big.Rat) {
	t.Helper()
	square := new(big.Rat).SetFloat64(published)
	require.NotNil(t, square)
	square.Mul(square, square)
	require.LessOrEqual(t, square.Cmp(exactSquare), 0,
		`a published gate diameter must sit at or below the exact witness maximum`)

	up := new(big.Rat).SetFloat64(math.Nextafter(published, math.Inf(1)))
	require.NotNil(t, up)
	up.Mul(up, up)
	require.Positive(t, up.Cmp(exactSquare),
		`the certified reading must be the tightest float64 at or below the exact maximum`)
}

// The shared witness-maximum reader publishes a DOWNWARD-rounded pair maximum,
// so an ordinary analytic body read through bodyGateDiameter's exact carrier
// arm never reports a diameter above its own. A 6x6x7 box's farthest pair is
// sqrt(6² + 6² + 7²) = sqrt(121) = 11 exactly, a representable value the float
// norm of that same pair overshoots by an ulp.
func TestBodyGateDiameterExactCarrierArmNeverOverstates(t *testing.T) {
	doc := New()
	body := internalBoxBody(t, doc, 0, 0, 6, 6, 7)

	require.Greater(t, r3.NewVec(6, 6, 0).Sub(r3.NewVec(0, 0, 7)).Len(), 11.0,
		`the float norm of the winning pair lands above the exact distance, which is why the reader may not publish it`)

	d, ok, err := bodyGateDiameter(t.Context(), body)
	require.NoError(t, err)
	require.True(t, ok)
	require.Equal(t, 11.0, d, `sqrt(121) is representable, so the certified reading is 11 exactly`)
	requireCertifiedDiameter(t, d, big.NewRat(121, 1))
}
