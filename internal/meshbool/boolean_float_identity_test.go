package meshbool

import (
	"fmt"
	"math"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// The functions below are the bodies TriTriDistance and its helpers had when they
// called math.Min, math.Max directly. They are the oracle: the comparison forms in
// boolean.go must return the same bits for every input.

func oldClamp01(num, den float64) float64 {
	if den <= 0 {
		return 0
	}
	return math.Max(0, math.Min(1, num/den))
}

func oldSegSegDist3(p0, p1, q0, q1 r3.Vec) float64 {
	u := p1.Sub(p0)
	v := q1.Sub(q0)
	w := p0.Sub(q0)
	a, b, c := u.Dot(u), u.Dot(v), v.Dot(v)
	d, e := u.Dot(w), v.Dot(w)
	den := a*c - b*b
	var s, t float64
	if den <= 0 {
		s, t = 0, oldClamp01(e, c)
	} else {
		s = oldClamp01(b*e-c*d, den)
		t = oldClamp01(a*e-b*d, den)
	}
	if tn := e + s*b; c > 0 {
		t = oldClamp01(tn, c)
	}
	if sn := -d + t*b; a > 0 {
		s = oldClamp01(sn, a)
	}
	return p0.Add(u.Scale(s)).Sub(q0.Add(v.Scale(t))).Len()
}

func oldPointTriDistance(p r3.Vec, t [3]r3.Vec) float64 {
	n := t[1].Sub(t[0]).Cross(t[2].Sub(t[0]))
	if nn := n.Dot(n); nn > 0 {
		h := n.Dot(p.Sub(t[0])) / nn
		foot := p.Sub(n.Scale(h))
		if PointInTriangle(foot, t, n) {
			return p.Sub(foot).Len()
		}
	}
	best := math.Inf(1)
	for i := range 3 {
		best = math.Min(best, oldSegSegDist3(p, p, t[i], t[(i+1)%3]))
	}
	return best
}

func oldTriTriDistance(ta, tb [3]r3.Vec) float64 {
	best := math.Inf(1)
	for i := range 3 {
		for j := range 3 {
			best = math.Min(best, oldSegSegDist3(ta[i], ta[(i+1)%3], tb[j], tb[(j+1)%3]))
		}
	}
	for i := range 3 {
		best = math.Min(best, oldPointTriDistance(ta[i], tb))
		best = math.Min(best, oldPointTriDistance(tb[i], ta))
	}
	if best <= 0 || best != best || best > math.MaxFloat64 || best < -math.MaxFloat64 {
		return 0
	}
	return best * (1 - 1e-12)
}

// adversarialFloats lists the values where math.Min, math.Max and the clamp change
// behaviour: both zeros, both infinities, NaNs with different payloads and signs,
// subnormals, the extreme finite values and the neighbours of each binade edge.
func adversarialFloats() []float64 {
	vals := []float64{
		0, math.Copysign(0, -1),
		math.Inf(1), math.Inf(-1),
		math.NaN(),
		math.Float64frombits(0x7ff8000000000005),
		math.Float64frombits(0xfff8000000000001),
		math.Float64frombits(0x7ff0000000000001),
		math.MaxFloat64, -math.MaxFloat64,
		math.SmallestNonzeroFloat64, -math.SmallestNonzeroFloat64,
		math.Float64frombits(0x000fffffffffffff), -math.Float64frombits(0x000fffffffffffff),
		math.Float64frombits(0x0010000000000000), -math.Float64frombits(0x0010000000000000),
		0.5, -0.5, 0.25, 3, -3,
	}
	for _, e := range []float64{1, 2, 4, 0.5, 1e-300, 1e300} {
		for _, s := range []float64{1, -1} {
			x := s * e
			vals = append(vals, x, math.Nextafter(x, math.Inf(1)), math.Nextafter(x, math.Inf(-1)))
		}
	}
	return vals
}

// requireSameBitsf requires got to carry the bits of want. Two NaNs match
// whatever their payloads: which payload math.Min and math.Max keep is
// architecture dependent (amd64 and arm64 differ), and no caller reads it. The
// distance callers compare the result (dist > slack, min, a non-finite check
// that returns 0), and a NaN fails every comparison whatever its payload.
// Every non-NaN result, including signed zeros and infinities, stays bit-exact.
func requireSameBitsf(t *testing.T, want, got float64, format string, args ...any) {
	t.Helper()
	if math.IsNaN(want) && math.IsNaN(got) {
		return
	}
	if math.Float64bits(want) == math.Float64bits(got) {
		return
	}
	require.FailNow(t, "bit mismatch", "%s: want %v (%#x) got %v (%#x)", fmt.Sprintf(format, args...),
		want, math.Float64bits(want), got, math.Float64bits(got))
}

func TestMinFloat64MatchesMathMin(t *testing.T) {
	vals := adversarialFloats()
	for _, x := range vals {
		for _, y := range vals {
			requireSameBitsf(t, math.Min(x, y), minFloat64(x, y), "min(%v, %v)", x, y)
		}
	}
	rng := rand.New(rand.NewPCG(1, 2))
	for range 1_000_000 {
		x, y := randomFloat(rng, vals), randomFloat(rng, vals)
		requireSameBitsf(t, math.Min(x, y), minFloat64(x, y), "min(%v, %v)", x, y)
	}
}

// TestMinFloat64NaNPayloads pins NaN operands of several payloads in both
// positions: a NaN against a number or another NaN is NaN, and a NaN against
// -Inf is -Inf, exactly as math.Min says.
func TestMinFloat64NaNPayloads(t *testing.T) {
	nans := []float64{
		math.Float64frombits(0x7ff8000000000001),
		math.Float64frombits(0x7ff8000000000005),
		math.Float64frombits(0xfff8000000000001),
		math.Float64frombits(0x7ff0000000000001),
	}
	others := []float64{0, math.Copysign(0, -1), 1, -1, math.Inf(1), math.Inf(-1), nans[0], nans[1]}
	for _, n := range nans {
		require.True(t, math.IsNaN(n))
		for _, o := range others {
			for _, got := range []float64{minFloat64(n, o), minFloat64(o, n)} {
				if math.IsInf(o, -1) {
					requireSameBitsf(t, o, got, "min(%#x, %v)", math.Float64bits(n), o)
					continue
				}
				require.True(t, math.IsNaN(got), "min(%#x, %v) = %v", math.Float64bits(n), o, got)
			}
			require.Equal(t, math.IsNaN(math.Min(n, o)), math.IsNaN(minFloat64(n, o)))
		}
	}
}

func TestClamp01MatchesMathForm(t *testing.T) {
	vals := adversarialFloats()
	for _, num := range vals {
		for _, den := range vals {
			requireSameBitsf(t, oldClamp01(num, den), Clamp01(num, den), "Clamp01(%v, %v)", num, den)
		}
	}
	rng := rand.New(rand.NewPCG(3, 4))
	for range 1_000_000 {
		num, den := randomFloat(rng, vals), randomFloat(rng, vals)
		requireSameBitsf(t, oldClamp01(num, den), Clamp01(num, den), "Clamp01(%v, %v)", num, den)
	}
	// The ratio itself lands on the clamp edges, which a pair from vals rarely does.
	for _, q := range vals {
		for _, den := range []float64{1, 2, 3, math.SmallestNonzeroFloat64} {
			num := q * den
			requireSameBitsf(t, oldClamp01(num, den), Clamp01(num, den), "Clamp01(%v, %v)", num, den)
		}
	}
}

// randomFloat draws from the adversarial list half the time, from a wide random
// range the other half.
func randomFloat(rng *rand.Rand, vals []float64) float64 {
	if rng.IntN(2) == 0 {
		return vals[rng.IntN(len(vals))]
	}
	return math.Float64frombits(rng.Uint64())
}

// gridVec draws coordinates from a handful of values so that ties, zeros and
// coincident vertices are common.
func gridVec(rng *rand.Rand, vals []float64) r3.Vec {
	pick := func() float64 { return vals[rng.IntN(len(vals))] }
	return r3.Vec{X: pick(), Y: pick(), Z: pick()}
}

func randVec(rng *rand.Rand) r3.Vec {
	const scale = 4
	return r3.Vec{
		X: (rng.Float64()*2 - 1) * scale,
		Y: (rng.Float64()*2 - 1) * scale,
		Z: (rng.Float64()*2 - 1) * scale,
	}
}

// trianglePair builds one pair of the kind named by mode: generic, zero-area,
// coincident, parallel, touching at a vertex, touching along an edge, integer
// grid, or a grid mixed with adversarial values.
func trianglePair(rng *rand.Rand, mode int) ([3]r3.Vec, [3]r3.Vec) {
	ta := [3]r3.Vec{randVec(rng), randVec(rng), randVec(rng)}
	tb := [3]r3.Vec{randVec(rng), randVec(rng), randVec(rng)}
	switch mode {
	case 1: // zero-area: collinear corners
		k := rng.Float64()
		ta[2] = ta[0].Add(ta[1].Sub(ta[0]).Scale(k))
		if rng.IntN(2) == 0 {
			tb[1] = tb[0]
		}
	case 2: // coincident, possibly reordered
		tb = ta
		if rng.IntN(2) == 0 {
			tb = [3]r3.Vec{ta[2], ta[0], ta[1]}
		}
	case 3: // parallel: tb is ta shifted along its normal
		n := ta[1].Sub(ta[0]).Cross(ta[2].Sub(ta[0]))
		off := n.Scale(rng.Float64() - 0.5)
		tb = [3]r3.Vec{ta[0].Add(off), ta[1].Add(off), ta[2].Add(off)}
		if rng.IntN(2) == 0 {
			tb[2] = tb[2].Add(ta[1].Sub(ta[0]).Scale(0.5))
		}
	case 4: // touching at one vertex
		tb[rng.IntN(3)] = ta[rng.IntN(3)]
	case 5: // touching along an edge
		tb[0], tb[1] = ta[0], ta[1]
	case 6: // integer grid
		g := []float64{0, 1, 2, -1, 0.5}
		for i := range 3 {
			ta[i], tb[i] = gridVec(rng, g), gridVec(rng, g)
		}
	case 7: // grid mixed with signed zeros, infinities, NaN, extremes
		g := adversarialFloats()
		for i := range 3 {
			ta[i], tb[i] = gridVec(rng, g), gridVec(rng, g)
		}
	}
	return ta, tb
}

func TestTriTriDistanceMatchesMathForm(t *testing.T) {
	rng := rand.New(rand.NewPCG(5, 6))
	for i := range 400_000 {
		mode := i % 8
		ta, tb := trianglePair(rng, mode)
		requireSameBitsf(t, oldTriTriDistance(ta, tb), TriTriDistance(ta, tb), "TriTriDistance mode %d %v %v", mode, ta, tb)
		for j := range 3 {
			requireSameBitsf(t, oldPointTriDistance(ta[j], tb), PointTriDistance(ta[j], tb),
				"PointTriDistance mode %d", mode)
			k := (j + 1) % 3
			requireSameBitsf(t, oldSegSegDist3(ta[j], ta[k], tb[j], tb[k]), SegSegDist3(ta[j], ta[k], tb[j], tb[k]),
				"SegSegDist3 mode %d", mode)
		}
	}
}

func BenchmarkTriTriDistance(b *testing.B) {
	rng := rand.New(rand.NewPCG(7, 8))
	pairs := make([][2][3]r3.Vec, 1024)
	for i := range pairs {
		ta, tb := trianglePair(rng, 0)
		pairs[i] = [2][3]r3.Vec{ta, tb}
	}
	b.Run("new", func(b *testing.B) {
		var sink float64
		for i := range b.N {
			p := pairs[i%len(pairs)]
			sink += TriTriDistance(p[0], p[1])
		}
		_ = sink
	})
	b.Run("old", func(b *testing.B) {
		var sink float64
		for i := range b.N {
			p := pairs[i%len(pairs)]
			sink += oldTriTriDistance(p[0], p[1])
		}
		_ = sink
	})
}

// triTriFoldPositions is the number of minimum folds in TriTriDistance: nine
// edge-edge distances, then six vertex-to-triangle distances in the order
// ta[0], tb[0], ta[1], tb[1], ta[2], tb[2].
const triTriFoldPositions = 15

// triTriDistanceDropNaNAt is TriTriDistance with the fold at position pos
// ignoring a NaN operand, the way `if x < best { best = x }` does.
func triTriDistanceDropNaNAt(ta, tb [3]r3.Vec, pos int) float64 {
	best := math.Inf(1)
	n := 0
	fold := func(x float64) {
		n++
		if n-1 == pos {
			if x < best {
				best = x
			}
			return
		}
		best = math.Min(best, x)
	}
	for i := range 3 {
		for j := range 3 {
			fold(oldSegSegDist3(ta[i], ta[(i+1)%3], tb[j], tb[(j+1)%3]))
		}
	}
	for i := range 3 {
		fold(oldPointTriDistance(ta[i], tb))
		fold(oldPointTriDistance(tb[i], ta))
	}
	if best <= 0 || best != best || best > math.MaxFloat64 || best < -math.MaxFloat64 {
		return 0
	}
	return best * (1 - 1e-12)
}

func rotateTri(t [3]r3.Vec, r int) [3]r3.Vec {
	return [3]r3.Vec{t[r%3], t[(r+1)%3], t[(r+2)%3]}
}

// TestTriTriDistanceNaNInOneFold pins one pair per fold position. In each pair
// exactly one fold value is NaN and the other fourteen are finite, so the true
// result is 0 (a NaN distance refuses) while a fold that drops NaN returns a
// positive number.
func TestTriTriDistanceNaNInOneFold(t *testing.T) {
	// The NaN comes from Inf/Inf in PointTriDistance(tb[2], ta): the foot
	// parameter divides an overflowed dot product by an overflowed norm.
	a := 1e140
	z := math.MaxFloat64 / (a * a)
	lo, hi := math.Nextafter(z, 0), math.Nextafter(z, math.Inf(1))
	point := [2][3]r3.Vec{
		{{}, {X: a}, {Y: a}},
		{{X: -1e20, Z: lo}, {X: -1e20, Z: lo}, {X: -1e20, Z: hi}},
	}
	// The NaN comes from an edge-edge distance whose edge vectors overflow.
	edge := [2][3]r3.Vec{
		{{X: 1e154, Y: 1e-140, Z: -1}, {X: 1e140, Y: -1e20, Z: 1e20}, {Y: 1e20, Z: 1.7976931348623156e+28}},
		{{Y: -1, Z: -1}, {Z: 1e-160}, {Y: 1e-160, Z: -1}},
	}
	// Rotating a triangle's corners moves the NaN to another fold; swapping the
	// triangles moves a vertex-to-triangle NaN between the ta and tb folds.
	cases := []struct {
		pos     int
		base    [2][3]r3.Vec
		swap    bool
		ra, rb  int
		comment string
	}{
		{0, edge, false, 0, 0, "edge ta0 tb0"},
		{1, edge, false, 0, 2, "edge ta0 tb1"},
		{2, edge, false, 0, 1, "edge ta0 tb2"},
		{3, edge, false, 2, 0, "edge ta1 tb0"},
		{4, edge, false, 2, 2, "edge ta1 tb1"},
		{5, edge, false, 2, 1, "edge ta1 tb2"},
		{6, edge, false, 1, 0, "edge ta2 tb0"},
		{7, edge, false, 1, 2, "edge ta2 tb1"},
		{8, edge, false, 1, 1, "edge ta2 tb2"},
		{9, point, true, 0, 2, "point ta0"},
		{10, point, false, 0, 2, "point tb0"},
		{11, point, true, 0, 1, "point ta1"},
		{12, point, false, 0, 1, "point tb1"},
		{13, point, true, 0, 0, "point ta2"},
		{14, point, false, 0, 0, "point tb2"},
	}
	require.Len(t, cases, triTriFoldPositions)
	for _, c := range cases {
		ta, tb := rotateTri(c.base[0], c.ra), rotateTri(c.base[1], c.rb)
		if c.swap {
			ta, tb = tb, ta
		}
		t.Run(c.comment, func(t *testing.T) {
			require.Zero(t, TriTriDistance(ta, tb))
			requireSameBitsf(t, oldTriTriDistance(ta, tb), TriTriDistance(ta, tb), "oracle")
			// Only the fold at c.pos drops the NaN, so the witness must tell
			// that fold apart from every other.
			for pos := range triTriFoldPositions {
				got := triTriDistanceDropNaNAt(ta, tb, pos)
				if pos == c.pos {
					require.Greater(t, got, 0.0, "dropping NaN at fold %d must change the result", pos)
					continue
				}
				require.Zero(t, got, "dropping NaN at fold %d must not change the result", pos)
			}
		})
	}
}
