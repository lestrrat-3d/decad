package pairkernel_test

import (
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/clearance/pairkernel"
	"github.com/lestrrat-3d/decad/internal/pair"
	"github.com/lestrrat-3d/decad/internal/reportvocab"
	"github.com/stretchr/testify/require"
)

func TestDisjointResult(t *testing.T) {
	t.Parallel()

	const diam = 7.5
	tests := []struct {
		name   string
		gap    *pair.ScalarReading
		delta  *big.Rat
		ok     bool
		lo, hi float64
		exact  bool
	}{
		{"an exact gap", &pair.ScalarReading{ValueMM: 5}, new(big.Rat), true, 5, 5, true},
		{"a bounded gap", &pair.ScalarReading{ValueMM: 5, BoundMM: 1}, new(big.Rat), true, 4, 6, false},
		{"a displaced gap", &pair.ScalarReading{ValueMM: 5}, big.NewRat(1, 2), true, 4.5, 5.5, false},
		{"bound and displacement add", &pair.ScalarReading{ValueMM: 5, BoundMM: 1}, big.NewRat(1, 2), true, 3.5, 6.5, false},
		{"no gap", nil, new(big.Rat), false, 0, 0, false},
		{"a displacement that swallows the gap", &pair.ScalarReading{ValueMM: 1}, big.NewRat(1, 1), false, 0, 0, false},
		{"a bound that swallows the gap", &pair.ScalarReading{ValueMM: 1, BoundMM: 2}, new(big.Rat), false, 0, 0, false},
		{"an upper end past the largest float", &pair.ScalarReading{ValueMM: math.MaxFloat64}, big.NewRat(1, 1), false, 0, 0, false},
	}
	for _, tc := range tests {
		got, ok := pairkernel.DisjointResult(tc.gap, tc.delta, diam)
		require.Equal(t, tc.ok, ok, tc.name)
		if !tc.ok {
			require.Equal(t, pairkernel.Result{}, got, tc.name)
			continue
		}
		require.Equal(t, reportvocab.PairDisjoint, got.Verdict, tc.name)
		require.Equal(t, tc.lo, got.Lo, tc.name)
		require.Equal(t, tc.hi, got.Hi, tc.name)
		require.Equal(t, tc.exact, got.Exact, tc.name)
		require.Equal(t, diam, got.Diam, tc.name)
	}
}
