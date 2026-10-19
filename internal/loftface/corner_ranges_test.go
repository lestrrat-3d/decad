package loftface

import (
	"math/big"
	"math/rand/v2"
	"testing"

	"github.com/stretchr/testify/require"
)

// randCornerMap draws a map whose denominator stays positive on the unit
// square: G0 in [4, 8] and GU, GV in [-1, 1] keep G above 2.
func randCornerMap(rng *rand.Rand) Map {
	vec := func() RatVec {
		return RatVec{randRatOperand(rng), randRatOperand(rng), randRatOperand(rng)}
	}
	return Map{
		Constant: vec(), Q0: vec(), QU: vec(), QV: vec(),
		G0: big.NewRat(int64(4+rng.IntN(5)), 1),
		GU: big.NewRat(int64(rng.IntN(5)-2), 2),
		GV: big.NewRat(int64(rng.IntN(5)-2), 2),
	}
}

// randCornerBox draws a source box u0 < u1, v0 < v1 inside [0, 1].
func randCornerBox(rng *rand.Rand) [4]*big.Rat {
	pair := func() (*big.Rat, *big.Rat) {
		a, b := int64(rng.IntN(16)), int64(rng.IntN(16))
		for a == b {
			b = int64(rng.IntN(16))
		}
		if a > b {
			a, b = b, a
		}
		return big.NewRat(a, 16), big.NewRat(b, 16)
	}
	u0, u1 := pair()
	v0, v1 := pair()
	return [4]*big.Rat{u0, u1, v0, v1}
}

func requireIntervalEqual(t *testing.T, want, got Interval) {
	t.Helper()
	requireRatEqual(t, want.Lo, got.Lo)
	requireRatEqual(t, want.Hi, got.Hi)
}

func TestCornerAxisRangesMatchScalarRange(t *testing.T) {
	rng := rand.New(rand.NewPCG(103, 107))
	for range 1000 {
		m := randCornerMap(rng)
		corners := BoxCorners(randCornerBox(rng))
		got := cornerAxisRanges(m, corners)
		arc := ArcCapRanges(m, corners)
		for axis := range got {
			var unit RatVec
			unit[0], unit[1], unit[2] = ratZero(), ratZero(), ratZero()
			unit[axis] = ratOne()
			want := ScalarRange(m, unit, corners)
			requireIntervalEqual(t, want, got[axis])
			requireIntervalEqual(t, want, arc[axis])
		}
	}
}

// legacyCircleOutsideMaps is CircleOutsideMaps as it was before the corner
// images were shared: three scalar ranges per map per band.
func legacyCircleOutsideMaps(maps []Map, box [4]*big.Rat,
	x, radius *big.Rat) bool {
	if len(maps) < 2 || x == nil || radius == nil || radius.Sign() <= 0 {
		return false
	}
	corners := BoxCorners(box)
	axis := [3]RatVec{{ratOne(), ratZero(), ratZero()},
		{ratZero(), ratOne(), ratZero()}, {ratZero(), ratZero(), ratOne()}}
	for _, mapped := range maps {
		if PositiveDenominator(mapped, box) != nil {
			return false
		}
	}
	radius2 := ratMul(radius, radius)
	for band := 0; band+1 < len(maps); band++ {
		var range3 [3]Interval
		for coordinate, direction := range axis {
			first := ScalarRange(maps[band], direction, corners)
			second := ScalarRange(maps[band+1], direction, corners)
			range3[coordinate] = IntervalHull(first, second)
		}
		if x.Cmp(range3[0].Lo) < 0 || x.Cmp(range3[0].Hi) > 0 {
			continue
		}
		minimum := ratAdd(IntervalSquareMin(range3[1]), IntervalSquareMin(range3[2]))
		maximum := ratAdd(IntervalSquareMax(range3[1]), IntervalSquareMax(range3[2]))
		if radius2.Cmp(minimum) >= 0 && radius2.Cmp(maximum) <= 0 {
			return false
		}
	}
	return true
}

func TestCircleOutsideMapsMatchesLegacy(t *testing.T) {
	rng := rand.New(rand.NewPCG(127, 131))
	var trues, falses int
	for range 1000 {
		maps := make([]Map, 2+rng.IntN(8))
		for i := range maps {
			maps[i] = randCornerMap(rng)
		}
		box := randCornerBox(rng)
		corners := BoxCorners(box)
		pick := cornerAxisRanges(maps[rng.IntN(len(maps))], corners)
		// x lies inside the X range of one map image so the X test passes
		// often; the radius spans 0.5 to 1.5 times the Y/Z extent of the
		// same image so that both outcomes occur.
		frac := big.NewRat(int64(rng.IntN(101)), 100)
		x := ratAdd(pick[0].Lo, ratMul(frac, ratSub(pick[0].Hi, pick[0].Lo)))
		if rng.IntN(4) == 0 {
			// Every fourth x lies beyond the X range of every map, so the
			// skip path (continue) is covered along with the radius test.
			top := pick[0].Hi
			for _, m := range maps {
				top = RatMax(top, cornerAxisRanges(m, corners)[0].Hi)
			}
			x = ratAdd(top, ratOne())
		}
		extent := 0.0
		for _, axis := range []int{1, 2} {
			for _, v := range []*big.Rat{pick[axis].Lo, pick[axis].Hi} {
				f, _ := ratAbs(v).Float64()
				extent = max(extent, f)
			}
		}
		radius := ratF(extent * (0.5 + rng.Float64()))

		want := legacyCircleOutsideMaps(maps, box, x, radius)
		require.Equal(t, want, CircleOutsideMaps(maps, box, x, radius))
		if want {
			trues++
		} else {
			falses++
		}
	}
	require.GreaterOrEqual(t, trues, 250, "too few true outcomes")
	require.GreaterOrEqual(t, falses, 250, "too few false outcomes")
}

func TestCircleOutsideMapsMatchesLegacyDegenerate(t *testing.T) {
	rng := rand.New(rand.NewPCG(127, 137))
	box := randCornerBox(rng)
	maps := []Map{randCornerMap(rng), randCornerMap(rng), randCornerMap(rng)}
	bad := randCornerMap(rng)
	bad.G0, bad.GU, bad.GV = ratZero(), ratZero(), ratZero()
	// Positive at one corner only: G = u - 1/2 over a box straddling 1/2.
	crossing := randCornerMap(rng)
	crossing.G0, crossing.GU, crossing.GV = big.NewRat(-1, 2), ratOne(), ratZero()
	straddle := [4]*big.Rat{big.NewRat(1, 4), big.NewRat(3, 4), big.NewRat(0, 1), big.NewRat(1, 1)}

	one, oneRadius := ratOne(), ratOne()
	cases := []struct {
		name   string
		maps   []Map
		box    [4]*big.Rat
		x, rad *big.Rat
	}{
		{"one map", maps[:1], box, one, oneRadius},
		{"no maps", nil, box, one, oneRadius},
		{"nil x", maps, box, nil, oneRadius},
		{"nil radius", maps, box, one, nil},
		{"zero radius", maps, box, one, ratZero()},
		{"negative radius", maps, box, one, big.NewRat(-1, 1)},
		{"zero denominator", []Map{maps[0], bad, maps[1]}, box, one, oneRadius},
		{"crossing denominator", []Map{maps[0], crossing}, straddle, one, oneRadius},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			require.Equal(t,
				legacyCircleOutsideMaps(tc.maps, tc.box, tc.x, tc.rad),
				CircleOutsideMaps(tc.maps, tc.box, tc.x, tc.rad))
		})
	}
}
