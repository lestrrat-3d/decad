package loftmesh

import "testing"

func TestCommonSpanCountRespectsStationShare(t *testing.T) {
	for _, tc := range []struct {
		a, b, share, want int
		ok                bool
	}{
		{4, 5, 20, 20, true},
		{4, 6, 12, 12, true},
		{4, 5, 19, 0, false},
		{150, 151, 8192, 0, false},
	} {
		got, ok := commonSpanCount(tc.a, tc.b, tc.share)
		if got != tc.want || ok != tc.ok {
			t.Fatalf("commonSpanCount(%d, %d, %d) = (%d, %t), want (%d, %t)",
				tc.a, tc.b, tc.share, got, ok, tc.want, tc.ok)
		}
	}
}
