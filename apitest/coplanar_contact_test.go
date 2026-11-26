package apitest_test

import (
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// roundTrip turns b 20 degrees about the diagonal axis through the origin and
// back by two separate Transforms, so the float corners of its faces round
// (agent-docs/coplanar-contact-design.md §11.3). Its faces then read a positive
// bound where they meet another body, so a coplanar contact with it keeps the
// exact refusal and the gap-tolerance retry. A turn about a cardinal axis keeps
// the corners exact and does not.
func roundTrip(t *testing.T, b *decad.Body) *decad.Body {
	t.Helper()
	for _, deg := range []float64{20, -20} {
		turn, err := r3.Rotation(r3.Vec{X: 1, Y: 1, Z: 1}, units.Degrees(deg))
		require.NoError(t, err)
		b, err = decad.Transform(b, turn).Do(t.Context())
		require.NoError(t, err)
	}
	return b
}

// roundedBoxBodyAtZ is boxBodyAtZ passed through roundTrip.
func roundedBoxBodyAtZ(t *testing.T, doc *decad.Document, x0, y0, x1, y1, z0, h float64) *decad.Body {
	t.Helper()
	return roundTrip(t, boxBodyAtZ(t, doc, x0, y0, x1, y1, z0, h))
}

// requireRoundedContactRefusal asserts the refusal of a coplanar contact with a
// rounded face. The rounding moves the face off the exact plane by about 1e-15,
// so the refusal is the proximity gate's near-miss one (it cannot tell touching
// from crossing inside the face's bound), not the mesh pass's TouchCoplanar;
// the unit tests TestCoplanarGateInexactFace and
// TestPairCutsCoplanarRefusedWhenInexact pin that refusal for faces that stay
// exactly coplanar.
func requireRoundedContactRefusal(t *testing.T, err error) {
	t.Helper()
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.NotErrorIs(t, err, decad.ErrDegenerate)
	var be *decad.BooleanError
	require.ErrorAs(t, err, &be)
	require.Equal(t, decad.BooleanUnsupportedContact, be.Code)
}

// TestCoplanarContactRoundedFixtureKeepsRetry pins what the rounded fixture is
// for: a pair whose contact face reads a positive bound is refused by the exact
// classification and decided by the gap-tolerance retry, as before.
func TestCoplanarContactRoundedFixtureKeepsRetry(t *testing.T) {
	t.Parallel()
	t.Run("default document retries", func(t *testing.T) {
		doc := decad.New()
		a := boxBodyAtZ(t, doc, 0, 0, 10, 10, 0, 10)
		b := roundedBoxBodyAtZ(t, doc, 0.5, 0, 10.5, 10, 10, 10)
		joined, err := decad.Union(a, b).Do(t.Context())
		require.NoError(t, err)
		shifted := false
		for _, d := range joined.OperationDiagnostics() {
			shifted = shifted || d.Code == decad.OpDiagGapShiftApplied
		}
		require.True(t, shifted, "the rounded contact face takes the gap shift")
		requireVolumeWithin(t, joined, 2000)
	})
	t.Run("ExactOnly document refuses", func(t *testing.T) {
		doc := decad.New(decad.ExactOnly())
		a := boxBodyAtZ(t, doc, 0, 0, 10, 10, 0, 10)
		b := roundedBoxBodyAtZ(t, doc, 0.5, 0, 10.5, 10, 10, 10)
		_, err := decad.Union(a, b).Do(t.Context())
		requireRoundedContactRefusal(t, err)
	})
}
