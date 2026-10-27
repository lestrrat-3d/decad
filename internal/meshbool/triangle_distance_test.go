package meshbool

import (
	"fmt"
	"math"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// exceedsPair builds one pair for TestTriTriDistanceExceedsMatchesDistance.
// Modes below 8 are trianglePair's (generic, zero-area, coincident, parallel,
// touching at a vertex or along an edge, integer grid, adversarial values
// with NaN and infinities). The rest are separated pairs: parallel facets
// whose corners project inside each other, a sliver against a facet, slightly
// tilted parallel facets, a generic pair moved apart, the same at large
// coordinates, at coordinates past 2^200, at a 1e-100 scale, and a facet whose
// corner sits just above a sliver's interior. The sliver's float normal is
// tilted enough that the corner's foot distance on it can fall well below the
// pair's true distance, which the gate must then still read.
func exceedsPair(rng *rand.Rand, mode int) ([3]r3.Vec, [3]r3.Vec) {
	if mode < 8 {
		return trianglePair(rng, mode)
	}
	ta := [3]r3.Vec{randVec(rng), randVec(rng), randVec(rng)}
	tb := [3]r3.Vec{randVec(rng), randVec(rng), randVec(rng)}
	apart := func(tb [3]r3.Vec) [3]r3.Vec {
		off := randVec(rng).Scale(1 + 4*rng.Float64())
		return [3]r3.Vec{tb[0].Add(off), tb[1].Add(off), tb[2].Add(off)}
	}
	shift := func(t [3]r3.Vec, by r3.Vec) [3]r3.Vec {
		return [3]r3.Vec{t[0].Add(by), t[1].Add(by), t[2].Add(by)}
	}
	switch mode {
	case 8: // parallel, tb a shrunk copy of ta moved along its normal
		n, _ := triNormal(ta).Normalize()
		c := ta[0].Add(ta[1]).Add(ta[2]).Scale(1.0 / 3)
		k := 0.2 + 0.7*rng.Float64()
		off := n.Scale(math.Pow(10, -6+6*rng.Float64()) * (2*float64(rng.IntN(2)) - 1))
		for i := range 3 {
			tb[i] = c.Add(ta[i].Sub(c).Scale(k)).Add(off)
		}
		if rng.IntN(2) == 0 {
			tb[0], tb[1] = tb[1], tb[0]
		}
	case 9: // a sliver: a corner a hair off the opposite edge
		ta[2] = ta[0].Add(ta[1].Sub(ta[0]).Scale(rng.Float64())).Add(randVec(rng).Scale(1e-9))
		if rng.IntN(2) == 0 {
			ta, tb = tb, ta
		}
		tb = apart(tb)
	case 10: // parallel facets with tb tilted by a small angle
		n, _ := triNormal(ta).Normalize()
		c := ta[0].Add(ta[1]).Add(ta[2]).Scale(1.0 / 3)
		tilt := randVec(rng).Scale(math.Pow(10, -8+6*rng.Float64()))
		gap := n.Scale(math.Pow(10, -4+4*rng.Float64()))
		for i := range 3 {
			d := ta[i].Sub(c).Scale(0.8)
			tb[i] = c.Add(d).Add(gap).Add(tilt.Scale(d.Len()))
		}
	case 11: // generic, moved apart
		tb = apart(tb)
	case 12: // moved apart at large coordinates
		tb = apart(tb)
		far := r3.Vec{X: 1, Y: -2, Z: 3}.Scale(math.Pow(10, 3+12*rng.Float64()))
		ta, tb = shift(ta, far), shift(tb, far)
	case 13: // past the 2^200 scale limit
		tb = apart(tb)
		for i := range 3 {
			ta[i], tb[i] = ta[i].Scale(0x1p205), tb[i].Scale(0x1p205)
		}
	case 14: // a 1e-100 scale, where the normals fall below 2^-300
		tb = apart(tb)
		for i := range 3 {
			ta[i], tb[i] = ta[i].Scale(1e-100), tb[i].Scale(1e-100)
		}
	case 15: // a corner of a facet just above a sliver's interior, parallel to it
		u, _ := randVec(rng).Normalize()
		v, _ := u.Cross(randVec(rng)).Normalize()
		const length, width = 4, 4e-10
		k, s := 0.3+0.6*rng.Float64(), 0.1+0.8*rng.Float64()
		tb = [3]r3.Vec{ta[0], ta[0].Add(u.Scale(length)), ta[0].Add(u.Scale(k * length)).Add(v.Scale(width))}
		q := ta[0].Add(u.Scale(s * k * length)).Add(v.Scale(width * s * rng.Float64()))
		p := q.Add(u.Cross(v).Scale(math.Pow(10, -7+3*rng.Float64())))
		ta = [3]r3.Vec{p, p.Add(u.Scale(2)).Add(v), p.Sub(u).Add(v.Scale(2))}
		if rng.IntN(2) == 0 {
			ta, tb = tb, ta
		}
	}
	return ta, tb
}

const exceedsPairModes = 16

// exceedsSlacks lists slacks on both sides of d, the distance the oracle
// returns, at relative steps from one ulp to a factor of two, and the
// slacks that leave the skip: zero, negative, NaN and infinite.
func exceedsSlacks(d float64) []float64 {
	out := []float64{0, math.Copysign(0, -1), -1, math.NaN(), math.Inf(1), math.Inf(-1),
		math.SmallestNonzeroFloat64, math.MaxFloat64}
	out = append(out, d, math.Nextafter(d, math.Inf(1)), math.Nextafter(d, math.Inf(-1)))
	for _, r := range []float64{1e-15, 1e-12, 1e-9, 1e-8, 1e-7, 1e-6, 1e-3, 0.5} {
		out = append(out, d*(1-r), d*(1+r))
	}
	return out
}

// TestTriTriDistanceExceedsMatchesDistance compares TriTriDistanceExceeds with
// the distance it answers for, over pairs that touch, cross, sit parallel,
// stay apart, are degenerate slivers, or carry large, tiny or non-finite
// coordinates, at slacks around each pair's distance. It also requires the
// sweep to reach every branch: both axes, one axis, none, and an axis that
// proves a slack within 1e-6 of the distance.
func TestTriTriDistanceExceedsMatchesDistance(t *testing.T) {
	t.Parallel()
	n := 60_000
	if raceDetector {
		n = 6_000
	}
	rng := rand.New(rand.NewPCG(11, 12))
	var both, onlyA, onlyB, neither, tight int
	for i := range n {
		mode := i % exceedsPairModes
		ta, tb := exceedsPair(rng, mode)
		d := oldTriTriDistance(ta, tb)
		for _, slack := range exceedsSlacks(d) {
			want := d > slack
			require.Equal(t, want, TriTriDistanceExceeds(ta, tb, slack),
				"mode %d slack %v distance %v: %v %v", mode, slack, d, ta, tb)
			m, ok := triPairScale(ta, tb)
			if !ok || !(slack >= 0 && slack <= math.MaxFloat64) {
				continue
			}
			thr := slack + (slack+m)*0x1p-26 + 0x1p-199
			clearA := axisGapExceeds(triNormal(ta), ta, tb, m, thr)
			clearB := axisGapExceeds(triNormal(tb), ta, tb, m, thr)
			switch {
			case clearA && clearB:
				both++
			case clearA:
				onlyA++
			case clearB:
				onlyB++
			default:
				neither++
			}
			if (clearA || clearB) && slack >= d*(1-1e-6) {
				tight++
			}
		}
	}
	label := fmt.Sprintf("both=%d onlyA=%d onlyB=%d neither=%d tight=%d", both, onlyA, onlyB, neither, tight)
	require.Positive(t, both, label)
	require.Positive(t, onlyA, label)
	require.Positive(t, onlyB, label)
	require.Positive(t, neither, label)
	require.Positive(t, tight, label)
}

// BenchmarkTriTriDistanceExceeds times the gate against the full distance on
// separated pairs, parallel and generic, at a slack far below their distance.
func BenchmarkTriTriDistanceExceeds(b *testing.B) {
	rng := rand.New(rand.NewPCG(13, 14))
	pairs := make([][2][3]r3.Vec, 1024)
	for i := range pairs {
		ta, tb := exceedsPair(rng, 8+3*(i%2))
		pairs[i] = [2][3]r3.Vec{ta, tb}
	}
	const slack = 1e-7
	b.Run("exceeds", func(b *testing.B) {
		far := 0
		for i := range b.N {
			p := pairs[i%len(pairs)]
			if TriTriDistanceExceeds(p[0], p[1], slack) {
				far++
			}
		}
		_ = far
	})
	b.Run("distance", func(b *testing.B) {
		far := 0
		for i := range b.N {
			p := pairs[i%len(pairs)]
			if TriTriDistance(p[0], p[1]) > slack {
				far++
			}
		}
		_ = far
	})
}
