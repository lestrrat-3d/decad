package meshbool

import (
	"fmt"
	"math"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/topo"
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

// axisProvesAbove reports whether one of TriTriDistanceExceeds's eighteen
// axes proves the float triangles farther apart than slack.
func axisProvesAbove(ta, tb [3]r3.Vec, slack float64) bool {
	m, ok := triPairScale(ta, tb)
	if !ok || !(slack >= 0 && slack <= math.MaxFloat64) {
		return false
	}
	thr := slack + (slack+m)*0x1p-26 + 0x1p-199
	return triGapAbove(ta, tb, slack) || closestEdgeAxisGapExceeds(ta, tb, m, thr)
}

// requireExceedsMatchesReference requires TriTriDistanceExceeds(ta, tb, slack)
// to equal TriTriDistance(ta, tb) > slack AND the exact distance > slack
// wherever refTriDistance2 decides the pair and both triangles are proper. A
// collapsed triangle may lose the exact proof, so there a far answer must
// only be true of the exact distance. Every far answer needs the float
// reading, and a non-finite corner is never far. The replay variant must give
// the same answer. It returns the answer.
func requireExceedsMatchesReference(t *testing.T, ta, tb [3]r3.Vec, slack float64, d2 refDist, decided, proper bool,
	format string, args ...any) bool {
	t.Helper()
	got := TriTriDistanceExceeds(ta, tb, slack)
	floatFar := TriTriDistance(ta, tb) > slack
	msg := fmt.Sprintf(format, args...)
	replayed, err := triTriDistanceExceedsReplay(t.Context(), ta, tb, slack, proofbound.NewReplayTrialBudget())
	require.NoError(t, err, msg)
	require.Equal(t, got, replayed, "the replay answers as TriTriDistanceExceeds: %s", msg)
	if got {
		require.True(t, floatFar, "a far answer needs the float reading: %s", msg)
	}
	switch {
	case proper:
		require.Equal(t, floatFar && refAbove(d2, slack), got, msg)
	case decided:
		if got {
			require.True(t, refAbove(d2, slack), "a far answer is far in exact arithmetic: %s", msg)
		}
	case !refFinite(ta, tb):
		require.False(t, got, "a non-finite corner is never far: %s", msg)
	}
	return got
}

// TestTriTriDistanceExceedsMatchesDistance compares TriTriDistanceExceeds with
// the float distance and an exact rational reference, over pairs that touch,
// cross, sit parallel, stay apart, are degenerate slivers, or carry large,
// tiny or non-finite coordinates, at slacks around each pair's float
// distance. It also requires the sweep to reach every branch: both normal
// axes, one, none, a normal axis that proves a slack within 1e-6 of the
// distance, the exact fallback, and a float reading the exact distance
// overturns.
func TestTriTriDistanceExceedsMatchesDistance(t *testing.T) {
	t.Parallel()
	n := 3_200
	if raceDetector {
		n = 480
	}
	rng := rand.New(rand.NewPCG(11, 12))
	var both, onlyA, onlyB, neither, tight, exact, overturned int
	for i := range n {
		mode := i % exceedsPairModes
		ta, tb := exceedsPair(rng, mode)
		d := TriTriDistance(ta, tb)
		d2, decided := refTriDistance2(ta, tb)
		proper := decided && refProper(ta, tb)
		for _, slack := range exceedsSlacks(d) {
			got := requireExceedsMatchesReference(t, ta, tb, slack, d2, decided, proper,
				"mode %d slack %v distance %v: %v %v", mode, slack, d, ta, tb)
			if d > slack && !got {
				overturned++
			}
			m, ok := triPairScale(ta, tb)
			if !ok || !(slack >= 0 && slack <= math.MaxFloat64) {
				continue
			}
			thr := slack + (slack+m)*0x1p-26 + 0x1p-199
			na, nb := triNormal(ta), triNormal(tb)
			clearA := axisGapExceeds(na, ta, tb, m, thr)
			clearB := axisGapExceeds(nb, ta, tb, m, thr)
			switch {
			case clearA && clearB:
				both++
			case clearA:
				onlyA++
			case clearB:
				onlyB++
			default:
				neither++
				if d > slack && !edgeAxesGapExceeds(ta, tb, na, nb, m, thr) && !closestEdgeAxisGapExceeds(ta, tb, m, thr) {
					exact++
				}
			}
			if (clearA || clearB) && slack >= d*(1-1e-6) {
				tight++
			}
		}
	}
	label := fmt.Sprintf("both=%d onlyA=%d onlyB=%d neither=%d tight=%d exact=%d overturned=%d",
		both, onlyA, onlyB, neither, tight, exact, overturned)
	require.Positive(t, both, label)
	require.Positive(t, onlyA, label)
	require.Positive(t, onlyB, label)
	require.Positive(t, neither, label)
	require.Positive(t, tight, label)
	require.Positive(t, exact, label)
	require.Positive(t, overturned, label)
}

// nearMissRotation is a random rotation, its rows applied to a point.
func nearMissRotation(rng *rand.Rand) [3]r3.Vec {
	q := [4]float64{rng.NormFloat64(), rng.NormFloat64(), rng.NormFloat64(), rng.NormFloat64()}
	n := math.Sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3])
	w, x, y, z := q[0]/n, q[1]/n, q[2]/n, q[3]/n
	return [3]r3.Vec{
		{X: 1 - 2*(y*y+z*z), Y: 2 * (x*y - z*w), Z: 2 * (x*z + y*w)},
		{X: 2 * (x*y + z*w), Y: 1 - 2*(x*x+z*z), Z: 2 * (y*z - x*w)},
		{X: 2 * (x*z - y*w), Y: 2 * (y*z + x*w), Z: 1 - 2*(x*x+y*y)},
	}
}

// nearMissKind picks the nearest features of a nearMissPair.
type nearMissKind string

const (
	// nearMissFoot puts a corner of b over a's interior.
	nearMissFoot nearMissKind = "foot"
	// nearMissEdge crosses two edges.
	nearMissEdge nearMissKind = "edge"
	// nearMissNearParallel crosses two edges theta from parallel.
	nearMissNearParallel nearMissKind = "nearpar"
	// nearMissSliver puts a corner of b over a sliver of apex height
	// scale*theta.
	nearMissSliver nearMissKind = "sliver"
	// nearMissSliverParallel crosses two slivers whose long edges are theta
	// from parallel.
	nearMissSliverParallel nearMissKind = "sliverpar"
)

// nearMissPair builds a disjoint pair whose coordinates reach about scale and
// whose nearest features, picked by kind, stand about gap apart. It builds
// the pair in a local frame where a lies at z <= 0 and b at z >= gap, then
// rotates and moves it.
func nearMissPair(rng *rand.Rand, kind nearMissKind, scale, gap, theta float64) ([3]r3.Vec, [3]r3.Vec) {
	l := scale / 2
	j := func() float64 { return rng.Float64()*2 - 1 }
	var a, b [3]r3.Vec
	switch kind {
	case nearMissFoot:
		a = [3]r3.Vec{{X: -l / 2, Y: -l / 3}, {X: l / 2, Y: -l / 3}, {X: l * 0.1 * j(), Y: 2 * l / 3}}
		x0, y0 := l*0.2*j(), l*0.1*j()
		b = [3]r3.Vec{{X: x0, Y: y0, Z: gap}, {X: x0 + l*0.3*j(), Y: y0 + l*0.3, Z: gap + l/2},
			{X: x0 + l*0.3, Y: y0 + l*0.3*j(), Z: gap + l/3}}
	case nearMissEdge, nearMissNearParallel:
		phi := 0.3 + 1.2*rng.Float64()
		if kind == nearMissNearParallel {
			phi = theta
		}
		cx := l * 0.2 * j()
		a = [3]r3.Vec{{X: -l / 2}, {X: l / 2}, {X: l * 0.2 * j(), Y: l * 0.3 * j(), Z: -l / 2}}
		dx, dy := math.Cos(phi)*l/2, math.Sin(phi)*l/2
		b = [3]r3.Vec{{X: cx - dx, Y: -dy, Z: gap}, {X: cx + dx, Y: dy, Z: gap},
			{X: l * 0.2 * j(), Y: l * 0.3 * j(), Z: gap + l/2}}
	case nearMissSliver:
		a = [3]r3.Vec{{X: -l / 2}, {X: l / 2}, {X: l * 0.4 * j(), Y: l * theta}}
		u, v := rng.Float64(), rng.Float64()
		if u+v > 1 {
			u, v = 1-u, 1-v
		}
		p := a[0].Scale(1 - u - v).Add(a[1].Scale(u)).Add(a[2].Scale(v))
		b = [3]r3.Vec{{X: p.X, Y: p.Y, Z: gap}, {X: p.X + l*0.3*j(), Y: p.Y + l*0.3, Z: gap + l/2},
			{X: p.X + l*0.3, Y: p.Y + l*0.3*j(), Z: gap + l/3}}
	case nearMissSliverParallel:
		a = [3]r3.Vec{{X: -l / 2}, {X: l / 2}, {X: l * 0.3 * j(), Y: -l * theta, Z: -l * theta}}
		dx, dy := math.Cos(theta)*l/2, math.Sin(theta)*l/2
		cx := l * 0.2 * j()
		b = [3]r3.Vec{{X: cx - dx, Y: -dy, Z: gap}, {X: cx + dx, Y: dy, Z: gap},
			{X: cx + l*0.3*j(), Y: l * theta, Z: gap + l*theta}}
	}
	r := nearMissRotation(rng)
	off := r3.Vec{X: (scale - l) * j(), Y: (scale - l) * j(), Z: (scale - l) * j()}
	for i := range 3 {
		a[i] = r3.Vec{X: r[0].Dot(a[i]) + off.X, Y: r[1].Dot(a[i]) + off.Y, Z: r[2].Dot(a[i]) + off.Z}
		b[i] = r3.Vec{X: r[0].Dot(b[i]) + off.X, Y: r[1].Dot(b[i]) + off.Y, Z: r[2].Dot(b[i]) + off.Z}
	}
	return a, b
}

// TestTriTriDistanceExceedsNearMissTables sweeps disjoint generic and sliver
// pairs, with edges up to 1e-10 rad from parallel, at coordinate scales of 1
// and 100 mm and gaps from 1e-9 to 1e-3 mm. At slacks on both sides of each
// pair's exact distance it requires TriTriDistanceExceeds to answer far
// exactly when the float reading and the exact distance both exceed the
// slack, so it never answers far at an exact distance within the slack. It
// requires the sweep to reach a float reading that the exact distance
// overturns, a far answer only the exact distance proves, and a far answer
// at half the distance.
func TestTriTriDistanceExceedsNearMissTables(t *testing.T) {
	t.Parallel()
	samples := 12
	if raceDetector {
		samples = 2
	}
	type config struct {
		kind  nearMissKind
		theta float64
	}
	configs := []config{{nearMissFoot, 0}, {nearMissEdge, 0}, {nearMissNearParallel, 1e-4},
		{nearMissNearParallel, 1e-6}, {nearMissNearParallel, 1e-8}, {nearMissNearParallel, 1e-10},
		{nearMissSliver, 1e-4}, {nearMissSliver, 1e-8}, {nearMissSliver, 1e-12}, {nearMissSliverParallel, 1e-6},
		{nearMissSliverParallel, 1e-8}}
	rng := rand.New(rand.NewPCG(21, 22))
	pairs, overturned, exactFar, farAtHalf := 0, 0, 0, 0
	for _, c := range configs {
		for _, scale := range []float64{1, 100} {
			for _, gap := range []float64{1e-9, 1e-6, 1e-3} {
				for range samples {
					ta, tb := nearMissPair(rng, c.kind, scale, gap, c.theta)
					d2, decided := refTriDistance2(ta, tb)
					if !decided || d2.n.Sign() == 0 || !refProper(ta, tb) {
						continue
					}
					pairs++
					s := refDistanceFloat(d2)
					for _, slack := range []float64{s, math.Nextafter(s, math.Inf(-1)), s * (1 + 1e-9), s * (1 - 1e-9),
						s * (1 + 1e-6), s * (1 - 1e-6), s / 2} {
						got := requireExceedsMatchesReference(t, ta, tb, slack, d2, true, true,
							"%s theta %v scale %v gap %v slack %v: %v %v", c.kind, c.theta, scale, gap, slack, ta, tb)
						if TriTriDistance(ta, tb) > slack && !got {
							overturned++
						}
						if got && !axisProvesAbove(ta, tb, slack) {
							exactFar++
						}
						if slack == s/2 && got {
							farAtHalf++
						}
					}
				}
			}
		}
	}
	label := fmt.Sprintf("pairs=%d overturned=%d exactFar=%d farAtHalf=%d", pairs, overturned, exactFar, farAtHalf)
	require.Positive(t, overturned, label)
	require.Positive(t, exactFar, label)
	require.Positive(t, farAtHalf, label)
}

// nearParallelTetra builds one closed tetrahedron as a single source face
// whose FaceBound and vertex bounds are delta.
func nearParallelTetra(t *testing.T, verts []r3.Vec, delta float64) (*topo.Mesh, *BoolMesh) {
	t.Helper()
	tris := [][3]int{{0, 1, 2}, {0, 3, 1}, {0, 2, 3}, {1, 3, 2}}
	face := &topo.Face{}
	bm, err := PrepContext(t.Context(), verts, tris, nil)
	require.NoError(t, err)
	beta := make([]float64, len(verts))
	for i := range beta {
		beta[i] = delta
	}
	bm.SetVertexBounds(beta)
	return &topo.Mesh{Vertices: verts, Triangles: tris, Source: []*topo.Face{face, face, face, face},
		FaceBound: map[*topo.Face]float64{face: delta}}, bm
}

// TestRefuseUndecidableProximityNearParallelEdges builds two tetrahedra about
// 90 mm from the origin whose nearest features are two edges 1e-8 rad from
// parallel. Each carries a bound of slack/2, so the gate's slack is the
// float at or just above their exact distance and the gate must refuse. The
// float facet distance overestimates the exact one by about 3e-11 mm, which
// the relative nudge does not cover.
func TestRefuseUndecidableProximityNearParallelEdges(t *testing.T) {
	a := []r3.Vec{
		{X: 45.34311794872238, Y: -64.82898893670323, Z: 49.89495566732255},
		{X: 81.05234456620133, Y: -60.26371102291974, Z: 84.59379386043001},
		{X: 59.10200992846318, Y: -86.09913185601087, Z: 74.55816534656037},
		{X: 78.3832874408551, Y: -76.88423118410093, Z: 53.50304855522049},
	}
	b := []r3.Vec{
		{X: 45.342840542173704, Y: -64.82804174691705, Z: 49.895116531172505},
		{X: 81.05206748100728, Y: -60.26276367955189, Z: 84.59395437336136},
		{X: 48.01189782819725, Y: -48.20752150894503, Z: 80.98586166092272},
		{X: 67.29317534058916, Y: -38.99262083703507, Z: 59.93074486958283},
	}
	const slack = 0.0010000000000024037
	tris := [][3]int{{0, 1, 2}, {0, 3, 1}, {0, 2, 3}, {1, 3, 2}}
	var nearest refDist
	floatMin := math.Inf(1)
	for _, ti := range tris {
		for _, tj := range tris {
			ta := [3]r3.Vec{a[ti[0]], a[ti[1]], a[ti[2]]}
			tb := [3]r3.Vec{b[tj[0]], b[tj[1]], b[tj[2]]}
			d2, ok := refTriDistance2(ta, tb)
			require.True(t, ok)
			require.Positive(t, d2.n.Sign(), "every facet pair is disjoint")
			if nearest.n == nil || d2.rat().Cmp(nearest.rat()) < 0 {
				nearest = d2
			}
			floatMin = math.Min(floatMin, TriTriDistance(ta, tb))
		}
	}
	require.False(t, refAbove(nearest, slack), "the exact facet distance is within the slack")
	require.Greater(t, floatMin, slack, "the float facet distance overestimates the exact one past the slack")

	ma, bmA := nearParallelTetra(t, a, slack/2)
	mb, bmB := nearParallelTetra(t, b, slack/2)
	err := RefuseUndecidableProximity(t.Context(), ma, mb, bmA, bmB, NewContactMemo(bmA, bmB), nil, nil)
	require.ErrorIs(t, err, decaderr.ErrUnsupported)
	require.ErrorContains(t, err, "held facets come within the chord tolerance")
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
