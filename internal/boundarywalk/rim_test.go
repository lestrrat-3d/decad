package boundarywalk_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/stretchr/testify/require"
)

func TestRimConvexity(t *testing.T) {
	circular := func(th0, th1 float64) survey2d.SideWalk {
		return survey2d.SideWalk{Kind: survey2d.WalkCircular, Th0: th0, Th1: th1}
	}
	line := survey2d.SideWalk{Kind: survey2d.WalkLine}

	tests := []struct {
		name     string
		walk     survey2d.SideWalk
		holeLoop bool
		want     bool
	}{
		{"counter-clockwise circle", circular(0, 1), false, true},
		{"clockwise circle", circular(1, 0), false, false},
		{"equal angles", circular(1, 1), false, true},
		{"line in outer loop", line, false, true},
		{"line in hole loop", line, true, false},
	}
	for _, tc := range tests {
		t.Run(tc.name, func(t *testing.T) {
			got, err := boundarywalk.RimConvexity(t.Context(), tc.walk, tc.holeLoop, freeform.NewFreeformWork())
			require.NoError(t, err)
			require.Equal(t, tc.want, got)
		})
	}
}
