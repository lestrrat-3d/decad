package compositesweep_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/compositesweep"
	"github.com/stretchr/testify/require"
)

func TestRewriteSideRole(t *testing.T) {
	tests := []struct {
		role   string
		span   int
		want   string
		wantOK bool
	}{
		{"side(0,3)", 2, "side(2,0,3)", true},
		{"side(1,3)", 2, "side(2,1,3)", true},
		{"side(0,1,3)", 4, "side(4,1,3)", true},
		{"side(5,1,3)", 4, "side(4,5,1,3)", true},
		{"capStart", 2, "capStart", false},
		{"body", 0, "body", false},
	}
	for _, tc := range tests {
		got, ok := compositesweep.RewriteSideRole(tc.role, tc.span)
		require.Equal(t, tc.want, got, tc.role)
		require.Equal(t, tc.wantOK, ok, tc.role)
	}
}
