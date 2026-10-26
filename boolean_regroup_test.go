package decad

import (
	"fmt"
	"testing"

	"github.com/stretchr/testify/require"

	"github.com/lestrrat-3d/decad/internal/meshbool"
)

func TestRegroupSurfacesInvariantBreaks(t *testing.T) {
	t.Parallel()
	for _, tc := range []struct {
		name string
		err  error
		want bool
	}{
		{"an evaluator failure surfaces", newBooleanError(meshbool.OpCut, BooleanEvaluatorFailure, ErrBooleanFailed, fmt.Errorf("broken")), true},
		{"an empty result is a miss", newBooleanError(meshbool.OpCut, BooleanEmpty, ErrBooleanFailed, fmt.Errorf("empty")), false},
		{"an unsupported contact is a miss", newBooleanError(meshbool.OpCut, BooleanUnsupportedContact, ErrUnsupported, fmt.Errorf("contact")), false},
		{"a bare boolean failure surfaces", fmt.Errorf("%w: raw", ErrBooleanFailed), true},
		{"a plain unsupported is a miss", fmt.Errorf("%w: staging", ErrUnsupported), false},
	} {
		require.Equal(t, tc.want, regroupSurfaces(tc.err), tc.name)
	}
}
