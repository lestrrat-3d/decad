package prismplacement_test

import (
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/prismplacement"
	"github.com/stretchr/testify/require"
)

func TestDenotedLevel(t *testing.T) {
	t.Parallel()
	// fl(0.2)·fl(25.4) is the level a 0.2 in depth denotes; its held float
	// rounds it, and that rounding is the displacement.
	exact := new(big.Rat).Mul(new(big.Rat).SetFloat64(0.2), new(big.Rat).SetFloat64(25.4))
	held, _ := exact.Float64()
	gap := new(big.Rat).Sub(exact, new(big.Rat).SetFloat64(held))
	require.NotZero(t, gap.Sign(), "the fixture level is not a float")
	delta := 1e-15
	d := prismplacement.DenoteLevel(exact, held, delta)

	t.Run("recorded level", func(t *testing.T) {
		t.Parallel()
		got := prismplacement.DenotedLevel(held, delta, d)
		require.NotNil(t, got)
		require.Zero(t, got.Cmp(exact))
		got.Add(got, big.NewRat(1, 1))
		require.Zero(t, prismplacement.DenotedLevel(held, delta, d).Cmp(exact), "each read is a fresh value")
	})
	t.Run("undisplaced level denotes its float", func(t *testing.T) {
		t.Parallel()
		got := prismplacement.DenotedLevel(7, 0, prismplacement.LevelDenotation{})
		require.Zero(t, got.Cmp(big.NewRat(7, 1)))
	})
	t.Run("displaced level without a record", func(t *testing.T) {
		t.Parallel()
		require.Nil(t, prismplacement.DenotedLevel(held, delta, prismplacement.LevelDenotation{}))
	})
	t.Run("rewritten held level", func(t *testing.T) {
		t.Parallel()
		require.Nil(t, prismplacement.DenotedLevel(math.Nextafter(held, 0), delta, d))
	})
	t.Run("rewritten displacement", func(t *testing.T) {
		t.Parallel()
		require.Nil(t, prismplacement.DenotedLevel(held, 2*delta, d))
	})
	t.Run("claim farther than its displacement", func(t *testing.T) {
		t.Parallel()
		tight := prismplacement.DenoteLevel(exact, held, 1e-30)
		require.Equal(t, prismplacement.LevelDenotation{}, tight)
		require.Nil(t, prismplacement.DenotedLevel(held, 1e-30, tight))
	})
	t.Run("no exact level", func(t *testing.T) {
		t.Parallel()
		require.Equal(t, prismplacement.LevelDenotation{}, prismplacement.DenoteLevel(nil, held, delta))
	})
}

func TestSameDenotedLevel(t *testing.T) {
	exact := new(big.Rat).Mul(new(big.Rat).SetFloat64(0.2), new(big.Rat).SetFloat64(25.4))
	held, _ := exact.Float64()
	d1 := prismplacement.DenoteLevel(exact, held, 1e-15)
	d2 := prismplacement.DenoteLevel(exact, held, 2e-15)
	type level struct {
		held, delta float64
		denoted     prismplacement.LevelDenotation
	}
	valid := level{held, 1e-15, d1}
	zero := prismplacement.LevelDenotation{}
	cases := []struct {
		name string
		a, b level
		want bool
	}{
		{"undisplaced", level{7, 0, zero}, level{7, 0, zero}, true},
		{"different", level{7, 0, zero}, level{8, 0, zero}, false},
		{"different bounds", valid, level{held, 2e-15, d2}, true},
		{"different exact", valid, level{held, 0, zero}, false},
		{"unknown", valid, level{held, 1e-15, zero}, false},
		{"stale held", valid, level{math.Nextafter(held, 0), 1e-15, d1}, false},
		{"stale bound", valid, level{held, 2e-15, d1}, false},
		{"nan", valid, level{math.NaN(), 0, zero}, false},
		{"positive infinity", valid, level{math.Inf(1), 0, zero}, false},
		{"negative infinity", valid, level{math.Inf(-1), 0, zero}, false},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			for _, pair := range [][2]level{{tc.a, tc.b}, {tc.b, tc.a}} {
				a, b := pair[0], pair[1]
				require.Equal(t, tc.want, prismplacement.SameDenotedLevel(a.held, a.delta, a.denoted, b.held, b.delta, b.denoted))
			}
		})
	}
	require.Zero(t, prismplacement.DenotedLevel(held, 1e-15, d1).Cmp(exact))
	require.Zero(t, prismplacement.DenotedLevel(held, 2e-15, d2).Cmp(exact))
}
