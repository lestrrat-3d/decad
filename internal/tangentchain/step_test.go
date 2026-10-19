package tangentchain

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/clearance"
	"github.com/stretchr/testify/require"
)

func TestStep(t *testing.T) {
	t.Parallel()
	yes, no, unknown := clearance.DegYes, clearance.DegNo, clearance.DegUnknown
	tests := []struct {
		name         string
		states       []clearance.DegState
		next, proven int
		undecided    int
	}{
		{name: "no candidates", next: -1},
		{name: "all disproven", states: []clearance.DegState{no, no}, next: -1},
		{name: "one proven", states: []clearance.DegState{no, yes, no}, next: 1, proven: 1},
		{name: "branch", states: []clearance.DegState{yes, no, yes}, next: 2, proven: 2},
		{name: "undecided", states: []clearance.DegState{no, unknown}, next: -1, undecided: 1},
		{name: "proven beside undecided", states: []clearance.DegState{yes, unknown}, next: 0, proven: 1, undecided: 1},
	}
	for _, tc := range tests {
		t.Run(tc.name, func(t *testing.T) {
			next, proven, undecided := Step(tc.states)
			require.Equal(t, tc.next, next)
			require.Equal(t, tc.proven, proven)
			require.Equal(t, tc.undecided, undecided)
		})
	}
}
