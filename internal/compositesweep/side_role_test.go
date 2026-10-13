package compositesweep_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/compositesweep"
	"github.com/stretchr/testify/require"
)

const (
	capStart = "capStart"
	capEnd   = "capEnd"
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
		{capStart, 2, capStart, false},
		{"body", 0, "body", false},
	}
	for _, tc := range tests {
		got, ok := compositesweep.RewriteSideRole(tc.role, tc.span)
		require.Equal(t, tc.want, got, tc.role)
		require.Equal(t, tc.wantOK, ok, tc.role)
	}
}

func TestSpanFaceRole(t *testing.T) {
	tests := []struct {
		role    string
		span    int
		reverse bool
		want    string
	}{
		{"side(0,3)", 2, false, "side(2,0,3)"},
		{"side(0,1,3)", 2, true, "side(2,0,1,3)"},
		{capStart, 1, false, capStart},
		{capStart, 1, true, capEnd},
		{capEnd, 1, true, capStart},
		{"other", 1, true, "other"},
	}
	for _, tc := range tests {
		got := compositesweep.SpanFaceRole(tc.role, tc.span, tc.reverse, capStart, capEnd)
		require.Equal(t, tc.want, got, tc.role)
	}
}

func TestSwapCapRole(t *testing.T) {
	require.Equal(t, capEnd, compositesweep.SwapCapRole(capStart, capStart, capEnd))
	require.Equal(t, capStart, compositesweep.SwapCapRole(capEnd, capStart, capEnd))
	require.Equal(t, "side(0,1)", compositesweep.SwapCapRole("side(0,1)", capStart, capEnd))
}
