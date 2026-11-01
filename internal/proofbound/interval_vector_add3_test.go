package proofbound_test

import (
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/stretchr/testify/require"
)

func TestIvVec3Add3MatchesNested(t *testing.T) {
	types := []proofbound.RatInterval{
		proofbound.Interval(big.NewRat(0, 1), big.NewRat(0, 1)),
		proofbound.Interval(big.NewRat(5, 8), big.NewRat(5, 8)),
		proofbound.Interval(big.NewRat(-3, 2), big.NewRat(7, 3)),
		proofbound.Interval(big.NewRat(-7, 3), big.NewRat(-1, 4)),
		proofbound.Interval(big.NewRat(-2, 5), big.NewRat(3, 8)),
		proofbound.Interval(big.NewRat(1, 8), big.NewRat(5, 7)),
	}
	var a, b, c proofbound.IvVec3
	for coordinate := range a {
		a[coordinate] = types[coordinate]
		b[coordinate] = types[(coordinate+2)%len(types)]
		c[coordinate] = types[(coordinate+4)%len(types)]
	}
	inputs := []proofbound.IvVec3{a, b, c}
	before := cloneIvVec3s(inputs)

	got := proofbound.IvVec3Add3(a, b, c)
	want := proofbound.IvVec3Add(a, proofbound.IvVec3Add(b, c))
	for coordinate := range got {
		require.Zero(t, got[coordinate].Lo.Cmp(want[coordinate].Lo))
		require.Zero(t, got[coordinate].Hi.Cmp(want[coordinate].Hi))
		require.Equal(t, want[coordinate].Lo.RatString(), got[coordinate].Lo.RatString())
		require.Equal(t, want[coordinate].Hi.RatString(), got[coordinate].Hi.RatString())
	}

	for coordinate := range got {
		wantHi := new(big.Rat).Set(got[coordinate].Hi)
		got[coordinate].Lo.Add(got[coordinate].Lo, big.NewRat(11, 1))
		require.Zero(t, got[coordinate].Hi.Cmp(wantHi), "output endpoints alias")
		wantLo := new(big.Rat).Set(got[coordinate].Lo)
		got[coordinate].Hi.Sub(got[coordinate].Hi, big.NewRat(13, 1))
		require.Zero(t, got[coordinate].Lo.Cmp(wantLo), "output endpoints alias")
	}
	for i := range inputs {
		for coordinate := range inputs[i] {
			require.Zero(t, inputs[i][coordinate].Lo.Cmp(before[i][coordinate].Lo))
			require.Zero(t, inputs[i][coordinate].Hi.Cmp(before[i][coordinate].Hi))
		}
	}
}

func cloneIvVec3s(values []proofbound.IvVec3) []proofbound.IvVec3 {
	clones := make([]proofbound.IvVec3, len(values))
	for i, value := range values {
		for coordinate, interval := range value {
			clones[i][coordinate] = proofbound.Interval(interval.Lo, interval.Hi)
		}
	}
	return clones
}

var ivVec3Add3Sink proofbound.IvVec3

func BenchmarkIvVec3Add3(b *testing.B) {
	a := benchmarkIvVec3(-7, 11)
	bv := benchmarkIvVec3(17, 23)
	c := benchmarkIvVec3(-29, 37)
	b.Run("nested", func(b *testing.B) {
		b.ReportAllocs()
		for range b.N {
			ivVec3Add3Sink = proofbound.IvVec3Add(a, proofbound.IvVec3Add(bv, c))
		}
	})
	b.Run("add3", func(b *testing.B) {
		b.ReportAllocs()
		for range b.N {
			ivVec3Add3Sink = proofbound.IvVec3Add3(a, bv, c)
		}
	})
}

func benchmarkIvVec3(lo, hi int64) proofbound.IvVec3 {
	var out proofbound.IvVec3
	for coordinate := range out {
		out[coordinate] = proofbound.Interval(big.NewRat(lo+int64(coordinate), 7), big.NewRat(hi+int64(coordinate), 11))
	}
	return out
}
