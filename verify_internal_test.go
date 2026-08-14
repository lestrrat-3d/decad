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

// A prismPayload carrying BOTH displacements still earns a gate reference
// (docs/prism-boolean-design.md §12's "Verify's structural/tolerance gates"
// row): the exact carrier model refuses it, so gateWitnessPrism hands
// fallbackGateDiameter the body's own recorded section beside the displacement
// each witness carries. This test pins what that arm proves — the charge is
// twice the SUM of the section and axial displacements, and the value it is
// subtracted from is the shared reader's exact-rational pair distance, not the
// float scan's own norm.
//
// The same 6x6x7 box is the fixture, so the exact witness maximum is
// sqrt(121) = 11 while the float norm of that pair reads 11.000000000000002.
// Both displacements are 0.125, so twice their sum is 0.5 and the exact
// shrunken value is exactly 10.5 — every quantity here is representable, and
// the two routes land on opposite sides of it.
func TestBodyGateDiameterDisplacedPrismShrinksByTwiceTheSum(t *testing.T) {
	doc := New()
	pp, isPrism := internalBoxBody(t, doc, 0, 0, 6, 6, 7).payload.(prismPayload)
	require.True(t, isPrism)

	const delta = 0.125
	pp.sectionDelta = delta
	pp.z1Delta = delta
	require.InDelta(t, delta, pp.axialDelta(), 0)

	witness, displacement, ok := gateWitnessPrism(pp)
	require.True(t, ok, `a displaced prism earns a witness prism instead of losing its gate reference`)
	require.Zero(t, witness.sectionDelta, `the witness prism's own section is read as held, never as displaced`)
	require.Equal(t, absSumUpper(delta, pp.axialDelta()), displacement,
		`the charged displacement is the SUM of the section and axial terms, since they are perpendicular`)

	d, ok, err := bodyGateDiameter(t.Context(), &Body{payload: pp})
	require.NoError(t, err)
	require.True(t, ok, `Verify forms a tolerance-gate reference for a displaced prism`)

	require.Less(t, d, 10.5,
		`the reference sits at or below the exact 11 - 2*(sectionDelta + axialDelta), and downRound puts it strictly below`)
	require.InDelta(t, 10.5, d, 1e-12,
		`the shrink is the two displacements themselves, not a coarse envelope`)
	require.Less(t, d, 11-2*delta-delta,
		`a shrink of twice the section term PLUS the axial term would leave the reference above 10.625`)

	// The float scan's own norm would publish a reference at or above the exact
	// shrunken value, which is the loosening direction §3 forbids. The
	// published reading is proven below it.
	scan := r3.NewVec(6, 6, 0).Sub(r3.NewVec(0, 0, 7)).Len()
	require.Greater(t, scan, 11.0)
	require.GreaterOrEqual(t, downRound(scan-2*displacement), 10.5)
	require.Less(t, d, downRound(scan-2*displacement),
		`the shrink is applied to the exact-rational pair distance, never to the float scan's norm`)
}
