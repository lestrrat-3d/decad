package sectionaudit_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionaudit"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/stretchr/testify/require"
)

func TestContactFloorUsesArcExtremes(t *testing.T) {
	t.Parallel()
	for _, tc := range []struct {
		name string
		walk survey2d.SegmentWalk
		want float64
	}{
		{
			name: "semicircle",
			walk: survey2d.SegmentWalk{
				StartU: 10, EndU: -10, CU: 0, CV: 0,
				Kind: survey2d.WalkCircular, Radius: 10, Th0: 0, Th1: math.Pi,
			},
			want: sectionaudit.ContactEps * math.Hypot(20, 10),
		},
		{
			name: "whole circle",
			walk: survey2d.SegmentWalk{
				StartU: 5, EndU: 5, CU: 0, CV: 0,
				Kind: survey2d.WalkCircular, Radius: 5, Th0: 0, Th1: 2 * math.Pi,
			},
			want: sectionaudit.ContactEps * math.Hypot(10, 10),
		},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			got, err := sectionaudit.ContactFloor(proofbound.NewWorkBudget(t.Context()),
				[]sectionaudit.Entry{sectionaudit.NewEntry(0, 0, 1, tc.walk)})
			require.NoError(t, err)
			require.InEpsilon(t, tc.want, got, 1e-12)
		})
	}
}
