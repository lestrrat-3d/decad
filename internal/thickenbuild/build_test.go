package thickenbuild

import (
	"math"
	"strings"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/revolveaxis"
	"github.com/stretchr/testify/require"
)

// A resolved axis whose anchor or direction carries a proven bound reaches
// thickenRadialOf's second leg. No public axis constructor produces one
// alongside an exactly axis-parallel direction, so the gate is exercised
// where it lives.
func TestThickenRadialRefusesBoundedAxis(t *testing.T) {
	t.Parallel()
	for _, tc := range []struct {
		name string
		ax   revolveaxis.Frame
		want string
	}{
		{"bounded anchor", revolveaxis.Frame{DU: 0, DV: 1, AUBound: math.Ldexp(1, -40)},
			"not stated exactly in the sketch plane"},
		{"bounded direction", revolveaxis.Frame{DU: 0, DV: 1, DVBound: math.Ldexp(1, -40)},
			"not stated exactly in the sketch plane"},
		{"off a plane axis", revolveaxis.Frame{DU: 0.6, DV: 0.8},
			"not parallel to a recorded plane axis"},
	} {
		t.Run(tc.name, func(t *testing.T) {
			_, err := thickenRadialOf(tc.ax)
			require.ErrorIs(t, err, decaderr.ErrUnsupported)
			require.True(t, strings.Contains(err.Error(), tc.want), err.Error())
		})
	}
}
