package linkagebound_test

import (
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/linkagebound"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/stretchr/testify/require"
)

func rat(t *testing.T, s string) *big.Rat {
	t.Helper()
	r, ok := new(big.Rat).SetString(s)
	require.True(t, ok, s)
	return r
}

// upTo is a dependent's hull whose largest |value| is hi.
func upTo(t *testing.T, hi string) proofbound.RatInterval {
	t.Helper()
	return proofbound.IntervalOwned(rat(t, "2.05"), rat(t, hi))
}

// TestDependentReach is agent-docs/linkage-check-design.md §15.5's reach
// guard on scene 11's coupler near the crank's 90° end, the readings sketch's
// tightness-sized pieces give: the decomposition certified the whole drive
// as the cell [0, 1], whose hull sets m = 2.0905523; the cells [a, 1] for a =
// 1/2, 3/4, 7/8 and 15/16 have hulls reaching 2.0905423, 2.0905961, 2.0905351
// and 2.0905634, each enclosing the exact end value 2.0905268, two of them
// past m by their own last piece's overshoot.
//
// Legs seen to fail when deleted: the certified-stretch exemption (the 3/4
// and 15/16 cells are refused on a drive certified whole); holding a stretch
// outside the certified ones to m (the same hulls are admitted where the
// decomposition certified only [0, 1/2]).
func TestDependentReach(t *testing.T) {
	t.Parallel()
	m := "2.0905523"
	cells := []struct {
		a, hull string
		past    bool
	}{
		{"1/2", "2.0905423", false},
		{"3/4", "2.0905961", true},
		{"7/8", "2.0905351", false},
		{"15/16", "2.0905634", true},
	}
	end := upTo(t, "2.0905268")
	t.Run("a drive certified whole admits every cell's overshoot", func(t *testing.T) {
		t.Parallel()
		reach := linkagebound.DependentReach{
			Bound: []*big.Rat{rat(t, m)}, Certified: [][2]*big.Rat{{rat(t, "0"), rat(t, "1")}},
		}
		for _, c := range cells {
			h := upTo(t, c.hull)
			require.Equal(t, c.past, linkagebound.Magnitude(h).Cmp(reach.Bound[0]) > 0, `cell [%s, 1] against m`, c.a)
			require.LessOrEqual(t, linkagebound.Magnitude(end).Cmp(linkagebound.Magnitude(h)), 0, `cell [%s, 1] encloses the end value`, c.a)
			require.False(t, reach.StretchPast(0, h, rat(t, c.a), rat(t, "1")), `cell [%s, 1] lies inside the certified drive`, c.a)
		}
		require.False(t, reach.PointPast(0, end), `the end point lies within m`)
		require.True(t, reach.PointPast(0, upTo(t, "2.0905524")), `a point past m is refused even inside the certified drive`)
	})
	t.Run("a stretch the decomposition did not certify is held to m", func(t *testing.T) {
		t.Parallel()
		reach := linkagebound.DependentReach{
			Bound: []*big.Rat{rat(t, m)}, Certified: [][2]*big.Rat{{rat(t, "0"), rat(t, "1/2")}},
		}
		for _, c := range cells {
			require.Equal(t, c.past, reach.StretchPast(0, upTo(t, c.hull), rat(t, c.a), rat(t, "1")), `cell [%s, 1]`, c.a)
		}
		require.True(t, reach.StretchPast(0, upTo(t, "3"), rat(t, "1/4"), rat(t, "3/4")), `a stretch that leaves the certified one is held to m`)
		require.False(t, reach.StretchPast(0, upTo(t, "3"), rat(t, "1/4"), rat(t, "1/2")), `a stretch inside the certified one is not`)
		require.True(t, reach.StretchPast(0, upTo(t, "3"), rat(t, "5/8"), rat(t, "5/8")), `a point past the certified end is not covered`)
	})
	t.Run("coverage across certified stretches", func(t *testing.T) {
		t.Parallel()
		reach := linkagebound.DependentReach{Certified: [][2]*big.Rat{
			{rat(t, "0"), rat(t, "1/4")}, {rat(t, "1/4"), rat(t, "1/2")}, {rat(t, "3/4"), rat(t, "1")},
		}}
		require.True(t, reach.Covers(rat(t, "1/8"), rat(t, "1/2")), `adjacent stretches cover their union`)
		require.True(t, reach.Covers(rat(t, "1/2"), rat(t, "1/2")), `a shared end is covered`)
		require.True(t, reach.Covers(rat(t, "3/4"), rat(t, "1")))
		require.False(t, reach.Covers(rat(t, "3/8"), rat(t, "7/8")), `a gap breaks coverage`)
		require.False(t, reach.Covers(rat(t, "5/8"), rat(t, "5/8")), `a point in the gap is not covered`)
		require.False(t, linkagebound.DependentReach{}.Covers(rat(t, "0"), rat(t, "1")), `nothing certified covers nothing`)
	})
}
