package apitest_test

import (
	"context"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// A rod crossing a semicircular prism makes a held target in
// this fixture. The final disk shares the prism's axis when centerX is zero.
// At centerX=2 it is tangent to the prism while still cutting through the rod.
func circularUnionPair(t *testing.T, doc *decad.Document, centerX float64) (*decad.Body, *decad.Body) {
	t.Helper()
	s, profile := semicircleSketchAt(t, -1, 2)
	half, err := decad.Extrude(doc, s, profile, decad.Distance{D: units.Millimeters(2), Dir: decad.Along}).Do(t.Context())
	require.NoError(t, err)
	rod := xCylinder(t, doc, -2, 4, .5, .5, .25)
	target, err := decad.Union(half, rod).Do(t.Context())
	require.NoError(t, err)
	return target, historyZCylinder(t, doc, centerX, 0, 1, -.5, 2)
}

func TestCircularUnionContact(t *testing.T) {
	t.Parallel()
	for _, tc := range []struct {
		name           string
		centerX        float64
		reverse, shift bool
	}{
		{"coaxial", 0, false, false}, {"coaxial reversed", 0, true, false},
		{"mixed tangent", 2, false, true}, {"mixed tangent reversed", 2, true, true},
		{"shallow overlap", 2 - .00001, false, true}, {"deep overlap", 2 - .125, false, false},
	} {
		t.Run(tc.name, func(t *testing.T) {
			doc := decad.New()
			if tc.shift && tc.reverse {
				// The shifted held operand needs its own chain allowance before this
				// control can observe the ordinary gap adjustment.
				doc = decad.New(decad.WithChainTolerance(units.Millimeters(.001)))
			}
			a, b := circularUnionPair(t, doc, tc.centerX)
			previous, err := a.Volume()
			require.NoError(t, err)
			if tc.reverse {
				a, b = b, a
			}
			joined, err := decad.Union(a, b).Do(t.Context())
			require.NoError(t, err)
			require.Len(t, joined.Lumps(), 1)
			if tc.centerX == 0 {
				// The disk adds its full section over half a unit of height and its
				// lower half-section over the next one and a half units.
				vol, e := joined.Volume()
				require.NoError(t, e)
				require.InDelta(t, 5*math.Pi/4, vol.Value.Base()-previous.Value.Base(), vol.Bound.Base()+previous.Bound.Base())
			}
			shifted := false
			for _, d := range joined.OperationDiagnostics() {
				if d.Code == decad.OpDiagGapShiftApplied {
					shifted = true
				}
			}
			require.Equal(t, tc.shift, shifted)
			_, err = joined.Tessellate(t.Context(), units.Millimeters(.125), decad.WithVerification(decad.VerifyAll))
			require.NoError(t, err)
		})
	}
}

func TestCircularUnionZeroGap(t *testing.T) {
	for _, tc := range []struct {
		name    string
		centerX float64
		reverse bool
	}{{"mixed tangent", 2, false}, {"mixed tangent reversed", 2, true}, {"shallow overlap", 2 - .00001, false}} {
		t.Run(tc.name, func(t *testing.T) {
			doc := decad.New(decad.ExactOnly())
			a, b := circularUnionPair(t, doc, tc.centerX)
			if tc.reverse {
				a, b = b, a
			}
			_, err := decad.Union(a, b).Do(t.Context())
			require.ErrorIs(t, err, decad.ErrUnsupported)
		})
	}
}

func TestCircularUnionCancellation(t *testing.T) {
	a, b := circularUnionPair(t, decad.New(), 0)
	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	_, err := decad.Union(a, b).Do(ctx)
	require.ErrorIs(t, err, context.Canceled)
}
