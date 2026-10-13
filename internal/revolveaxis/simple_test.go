package revolveaxis_test

import (
	"context"
	"errors"
	"testing"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/revolveaxis"
	"github.com/stretchr/testify/require"
)

func TestProvesSimple(t *testing.T) {
	ax := revolveaxis.Frame{AV: 0.5, DU: 1}
	type call struct{ gu, gv float64 }
	var calls []call
	scanOf := func(rawLo float64, err error) revolveaxis.ExtentScan {
		return func(_ context.Context, _ momentinput.Profile, gu, gv float64,
			_ *freeform.FreeformWork, _ *momentinput.ProfileWalks) (float64, float64, float64, error) {
			calls = append(calls, call{gu, gv})
			return rawLo, 9, 0, err
		}
	}
	forbidden := func(context.Context, momentinput.Profile, float64, float64,
		*freeform.FreeformWork, *momentinput.ProfileWalks) (float64, float64, float64, error) {
		t.Fatal("scan must not run")
		return 0, 0, 0, nil
	}
	prove := func(ctx context.Context, full, radialProof bool, sectionDelta float64,
		scan revolveaxis.ExtentScan) bool {
		return revolveaxis.ProvesSimple(ctx, momentinput.Profile{}, ax, full, radialProof, sectionDelta, scan)
	}

	require.False(t, prove(t.Context(), false, false, 0, forbidden))
	require.False(t, prove(t.Context(), true, false, 1, forbidden))
	canceled, cancel := context.WithCancel(t.Context())
	cancel()
	require.False(t, prove(canceled, true, false, 0, forbidden))
	require.True(t, prove(t.Context(), true, true, 0, forbidden))

	calls = nil
	require.True(t, prove(t.Context(), true, false, 0, scanOf(1, nil)))
	require.Equal(t, []call{{gu: 0, gv: 1}}, calls)

	// The axis anchor sits at v = 0.5, so a raw minimum of 0.25 is radius -0.25.
	require.False(t, prove(t.Context(), true, false, 0, scanOf(0.25, nil)))
	require.False(t, prove(t.Context(), true, false, 0, scanOf(1, errors.New("scan failed"))))
}
