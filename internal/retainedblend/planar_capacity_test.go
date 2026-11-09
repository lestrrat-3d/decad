package retainedblend

import (
	"math"
	"testing"

	"github.com/stretchr/testify/require"
)

func TestAddCandidateCount(t *testing.T) {
	for _, tc := range []struct {
		name                string
		count, merged, hole int
		want                int
		ok                  bool
	}{
		{"empty", 0, 0, math.MaxInt, 0, true},
		{"ordinary", 7, 3, 4, 19, true},
		{"limit", 1, 1, math.MaxInt - 1, math.MaxInt, true},
		{"product overflow", 0, 2, math.MaxInt, 0, false},
		{"sum overflow", 2, 1, math.MaxInt - 1, 0, false},
	} {
		t.Run(tc.name, func(t *testing.T) {
			got, ok := addCandidateCount(tc.count, tc.merged, tc.hole)
			require.Equal(t, tc.ok, ok)
			require.Equal(t, tc.want, got)
		})
	}
}
