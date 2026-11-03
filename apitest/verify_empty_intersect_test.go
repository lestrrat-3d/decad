package apitest_test

import (
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/decadtest"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// TestVerifyEmptyIntersectionProvesDisjoint pins
// agent-docs/interference-design.md §6's empty-intersection rule: a thin pin
// longer than the bore it passes through, clear of the bore's wall, has a box
// that meets the plate's and no analytic, coaxial or history proof, so the
// read-only intersection decides it, and its empty result passed the
// proximity gate unshifted. A pin into the wall overlaps instead.
//
// Leg shown to fail: leaving an empty intersection undecided keeps a pair
// diagnostic on the clear pin.
func TestVerifyEmptyIntersectionProvesDisjoint(t *testing.T) {
	t.Parallel()
	for _, tc := range []struct {
		name  string
		pinX  float64
		clear bool
	}{
		{"pin clear of the bore", 20, true},
		{"pin into the bore's wall", 22.5, false},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			doc := decad.New()
			bore := placedAlongZ(t, coaxialSection(t, doc, 20, 10, 30, 4), -5.001)
			plate, err := decad.Cut(t.Context(), slab(t, doc, 0, 0, 40, 20, 0, 10, r3.Identity()), bore)
			require.NoError(t, err)
			require.Equal(t, "decad.facetedPayload", decadtest.PayloadClass(plate))
			pin := placedAlongZ(t, coaxialSection(t, doc, tc.pinX, 10, 40, 2), -10)
			report, err := doc.Verify(t.Context())
			require.NoError(t, err)
			interferences, diagnostics := pairFindings(report, plate, pin)
			if tc.clear {
				require.Zero(t, interferences)
				require.Zero(t, diagnostics, "the empty intersection decides the pair")
				return
			}
			require.Positive(t, interferences+diagnostics, "a pin into the wall is never called disjoint")
		})
	}
}
