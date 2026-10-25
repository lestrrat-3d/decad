package coneaxis_test

import (
	"math"
	"math/big"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/decad/internal/coneaxis"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/revolveaxis"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// testAxes are the design's fixtures (agent-docs/revolve-axis-design.md §7.1).
var testAxes = []r3.Vec{{X: 1, Y: 2}, {Y: 1}, {X: 3, Y: 4}, {X: 1, Z: 2}}

func rat(v float64) *big.Rat { return proofarith.FloatRat(v) }

func ratVec(v r3.Vec) [3]*big.Rat { return [3]*big.Rat{rat(v.X), rat(v.Y), rat(v.Z)} }

// oracleDot is p·q in big.Rat arithmetic, independent of Dyadic.
func oracleDot(p, q r3.Vec) *big.Rat {
	a, b := ratVec(p), ratVec(q)
	sum := new(big.Rat)
	for i := range a {
		sum.Add(sum, new(big.Rat).Mul(a[i], b[i]))
	}
	return sum
}

// oracleCrossSq is |p×q|² in big.Rat arithmetic, from Lagrange's identity
// |p|²|q|² − (p·q)², which shares no code with the component form.
func oracleCrossSq(p, q r3.Vec) *big.Rat {
	d := oracleDot(p, q)
	out := new(big.Rat).Mul(oracleDot(p, p), oracleDot(q, q))
	return out.Sub(out, new(big.Rat).Mul(d, d))
}

// dyadicPoint returns a point whose coordinates are random multiples of 2^-6
// in [-32, 32].
func dyadicPoint(rng *rand.Rand) r3.Vec {
	c := func() float64 { return float64(rng.IntN(4097)-2048) / 64 }
	return r3.Vec{X: c(), Y: c(), Z: c()}
}

func TestAxisReadingsMatchRationalOracle(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewPCG(1, 2))
	for _, v := range testAxes {
		axis, ok := coneaxis.New(v)
		require.True(t, ok)
		require.Equal(t, 0, oracleDot(v, v).Cmp(axis.NormSq().Rat()))
		for range 200 {
			p := dyadicPoint(rng)
			require.Equal(t, 0, oracleDot(p, v).Cmp(axis.Axial(p).Rat()), "axial %v about %v", p, v)
			require.Equal(t, 0, oracleCrossSq(p, v).Cmp(axis.RadialSq(p).Rat()), "radial %v about %v", p, v)
		}
	}
}

func TestNormBracketIsTight(t *testing.T) {
	t.Parallel()
	for _, v := range testAxes {
		axis, _ := coneaxis.New(v)
		q := oracleDot(v, v)
		lo, hi := axis.NormBracket()
		sq := func(f float64) *big.Rat { return new(big.Rat).Mul(rat(f), rat(f)) }
		require.LessOrEqual(t, sq(lo).Cmp(q), 0, "lo^2 <= A.A for %v", v)
		require.Greater(t, sq(math.Nextafter(lo, math.Inf(1))).Cmp(q), 0, "lo is the largest such float for %v", v)
		require.GreaterOrEqual(t, sq(hi).Cmp(q), 0, "hi^2 >= A.A for %v", v)
		require.Less(t, sq(math.Nextafter(hi, 0)).Cmp(q), 0, "hi is the smallest such float for %v", v)
	}
	// 3-4-5 is exact: the bracket collapses onto 5.
	axis, _ := coneaxis.New(r3.Vec{X: 3, Y: 4})
	lo, hi := axis.NormBracket()
	require.Equal(t, 5.0, lo)
	require.Equal(t, 5.0, hi)
	// (1, 2, 0) is irrational: the bracket spans exactly one step.
	axis, _ = coneaxis.New(r3.Vec{X: 1, Y: 2})
	lo, hi = axis.NormBracket()
	require.Less(t, lo, hi)
	require.Equal(t, math.Nextafter(lo, math.Inf(1)), hi)
}

// boxDistanceSq is the exact squared A-scaled distance σ² from the axis to the
// axis-aligned box of tri, found by brute force over the box's coordinate
// extremes: σ² = (p·N)² + p_n²(A·A) is minimised at a corner or on a face
// where p·N vanishes, so the minimum over the eight corners and the crossing
// points of p·N = 0 on the twelve edges is exact.
func boxDistanceSq(a, lo, hi r3.Vec) *big.Rat {
	var best *big.Rat
	corners := make([]r3.Vec, 0, 8)
	for _, x := range []float64{lo.X, hi.X} {
		for _, y := range []float64{lo.Y, hi.Y} {
			for _, z := range []float64{lo.Z, hi.Z} {
				corners = append(corners, r3.Vec{X: x, Y: y, Z: z})
			}
		}
	}
	consider := func(p r3.Vec) {
		s := oracleCrossSq(p, a)
		if best == nil || s.Cmp(best) < 0 {
			best = s
		}
	}
	for _, c := range corners {
		consider(c)
	}
	// Edge crossings: along each edge the function is a convex quadratic, so
	// sample its exact minimiser t = clamp(-b/(2c)) as a rational.
	for _, c := range corners {
		for axisIdx := range 3 {
			d := [3]float64{}
			d[axisIdx] = 1
			dir := r3.Vec{X: d[0], Y: d[1], Z: d[2]}
			// |(c + t·dir)×a|² = |c×a|² + 2t (c×a)·(dir×a) + t²|dir×a|²
			ca, da := c.Cross(a), dir.Cross(a)
			quad := oracleDot(da, da)
			if quad.Sign() == 0 {
				continue
			}
			tt := new(big.Rat).Quo(oracleDot(ca, da), quad)
			tt.Neg(tt)
			span := [3]float64{hi.X - lo.X, hi.Y - lo.Y, hi.Z - lo.Z}[axisIdx]
			if tt.Sign() < 0 || tt.Cmp(rat(span)) > 0 {
				continue
			}
			p := ratVec(c)
			p[axisIdx] = new(big.Rat).Add(p[axisIdx], tt)
			cs := crossRatSq(p, a)
			if best == nil || cs.Cmp(best) < 0 {
				best = cs
			}
		}
	}
	return best
}

func crossRatSq(p [3]*big.Rat, v r3.Vec) *big.Rat {
	q := ratVec(v)
	m := func(a, b, c, d *big.Rat) *big.Rat {
		return new(big.Rat).Sub(new(big.Rat).Mul(a, b), new(big.Rat).Mul(c, d))
	}
	cx, cy, cz := m(p[1], q[2], p[2], q[1]), m(p[2], q[0], p[0], q[2]), m(p[0], q[1], p[1], q[0])
	out := new(big.Rat).Mul(cx, cx)
	out.Add(out, new(big.Rat).Mul(cy, cy))
	return out.Add(out, new(big.Rat).Mul(cz, cz))
}

func TestRadialLowerBoundsTriangleAndKeepsBoxDistance(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewPCG(3, 4))
	for _, v := range testAxes {
		axis, _ := coneaxis.New(v)
		aa := oracleDot(v, v)
		for range 100 {
			var tri [3]r3.Vec
			// Keep the cell off the axis so the bound is positive for most draws.
			base := dyadicPoint(rng)
			for i := range tri {
				tri[i] = base.Add(r3.Vec{X: float64(rng.IntN(9)-4) / 32, Y: float64(rng.IntN(9)-4) / 32,
					Z: float64(rng.IntN(9)-4) / 32})
			}
			lower, ok := axis.RadialLower(tri)
			require.True(t, ok)
			// 1. It never exceeds σ at any rational point of the triangle.
			for _, w := range [][3]int64{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}, {1, 1, 0}, {1, 1, 1}, {1, 2, 3}, {3, 1, 1}} {
				sum := w[0] + w[1] + w[2]
				var q [3]*big.Rat
				for k := range q {
					q[k] = new(big.Rat)
					for i, p := range tri {
						c := [3]float64{p.X, p.Y, p.Z}[k]
						q[k].Add(q[k], new(big.Rat).Mul(big.NewRat(w[i], sum), rat(c)))
					}
				}
				sigmaSq := crossRatSq(q, v)
				require.True(t, proofbound.RatSquareAtMost(lower, sigmaSq), "lower %g exceeds sigma at %v of %v about %v",
					lower, w, tri, v)
			}
			// 2. It loses nothing against the exact distance from the cell's
			// coordinate box to the axis: the box minimum is a lower bound of
			// the triangle's, so lower is at least that distance.
			lo := r3.Vec{X: min(tri[0].X, tri[1].X, tri[2].X), Y: min(tri[0].Y, tri[1].Y, tri[2].Y),
				Z: min(tri[0].Z, tri[1].Z, tri[2].Z)}
			hi := r3.Vec{X: max(tri[0].X, tri[1].X, tri[2].X), Y: max(tri[0].Y, tri[1].Y, tri[2].Y),
				Z: max(tri[0].Z, tri[1].Z, tri[2].Z)}
			box := boxDistanceSq(v, lo, hi)
			floor := proofbound.RatSqrtDown(box)
			require.GreaterOrEqual(t, lower, floor, "lower %g is below the box distance %g for %v about %v",
				lower, floor, tri, v)
			// 3. And it is at most one rounding below the exact formula.
			require.LessOrEqual(t, lower, proofbound.RatSqrtUp(sigmaMinSq(v, aa, tri)))
		}
	}
}

// sigmaMinSq is the independent (p·N)² + p_n²(A·A) minimum over the triangle's
// vertex ranges, written without the package's span type.
func sigmaMinSq(a r3.Vec, aa *big.Rat, tri [3]r3.Vec) *big.Rat {
	n := 0
	for i, c := range [3]float64{a.X, a.Y, a.Z} {
		if c == 0 {
			n = i
			break
		}
	}
	en := [3]r3.Vec{{X: 1}, {Y: 1}, {Z: 1}}[n]
	na := a.Cross(en)
	minAbs := func(f func(p r3.Vec) *big.Rat) *big.Rat {
		lo, hi := f(tri[0]), f(tri[0])
		for _, p := range tri[1:] {
			v := f(p)
			if v.Cmp(lo) < 0 {
				lo = v
			}
			if v.Cmp(hi) > 0 {
				hi = v
			}
		}
		if lo.Sign() <= 0 && hi.Sign() >= 0 {
			return new(big.Rat)
		}
		la, ha := new(big.Rat).Abs(lo), new(big.Rat).Abs(hi)
		if la.Cmp(ha) < 0 {
			return la
		}
		return ha
	}
	inPlane := minAbs(func(p r3.Vec) *big.Rat { return oracleDot(p, na) })
	normal := minAbs(func(p r3.Vec) *big.Rat { return rat([3]float64{p.X, p.Y, p.Z}[n]) })
	out := new(big.Rat).Mul(inPlane, inPlane)
	return out.Add(out, new(big.Rat).Mul(aa, new(big.Rat).Mul(normal, normal)))
}

func TestRadialLowerCanonicalKeepsTheCoordinateRule(t *testing.T) {
	t.Parallel()
	tri := [3]r3.Vec{{X: 4, Y: 1, Z: -2}, {X: 4, Y: 3, Z: 5}, {X: 5, Y: 2, Z: 1}}
	got, ok := coneaxis.X().RadialLower(tri)
	require.True(t, ok)
	// min |y| = 1 over [1, 3] and the z range [-2, 5] contains 0: the rule is
	// max(1, 0) = 1, not the tighter sqrt(1² + 0²) form's equal value, so use a
	// cell where the two forms differ.
	require.Equal(t, 1.0, got)
	tri = [3]r3.Vec{{X: 4, Y: 3, Z: 4}, {X: 4, Y: 4, Z: 3}, {X: 5, Y: 3, Z: 3}}
	got, _ = coneaxis.X().RadialLower(tri)
	require.Equal(t, 3.0, got, "the canonical rule is the larger coordinate minimum, not their root-sum-square")
}

func TestRadialLowerRefusesAnAxisOutsideEveryCoordinatePlane(t *testing.T) {
	t.Parallel()
	axis, ok := coneaxis.New(r3.Vec{X: 1, Y: 1, Z: 1})
	require.True(t, ok)
	_, ok = axis.RadialLower([3]r3.Vec{{X: 1, Y: 2, Z: 3}, {X: 2, Y: 2, Z: 3}, {X: 1, Y: 3, Z: 3}})
	require.False(t, ok)
}

func TestNewRefusesDegenerateVectors(t *testing.T) {
	t.Parallel()
	for _, v := range []r3.Vec{{}, {X: math.NaN()}, {Y: math.Inf(1)}} {
		_, ok := coneaxis.New(v)
		require.False(t, ok, "%v", v)
	}
}

func mustFrame(t *testing.T, origin, u, v r3.Vec) r3.Frame {
	t.Helper()
	f, err := r3.NewFrame(origin, u, v)
	require.NoError(t, err)
	return f
}

func TestPredicates(t *testing.T) {
	t.Parallel()
	xy := mustFrame(t, r3.Vec{}, r3.Vec{X: 1}, r3.Vec{Y: 1})
	xz := mustFrame(t, r3.Vec{}, r3.Vec{X: 1}, r3.Vec{Z: 1})
	zx := mustFrame(t, r3.Vec{}, r3.Vec{Z: -1}, r3.Vec{X: 1})
	canonical := revolveaxis.Frame{DU: 1}

	t.Run("a canonical record takes the constant axis and never reads AxisVec", func(t *testing.T) {
		require.True(t, coneaxis.FrameCanonical(xy, canonical))
		axis, ok := coneaxis.OfRevolve(xy, canonical, r3.Vec{Y: 7})
		require.True(t, ok)
		require.True(t, axis.Canonical())
		require.Equal(t, coneaxis.X(), axis)
	})
	t.Run("an XZ record with AxisVec X fails the frame predicate and passes the axis predicate", func(t *testing.T) {
		require.False(t, coneaxis.FrameCanonical(xz, canonical))
		axis, ok := coneaxis.OfRevolve(xz, canonical, r3.Vec{X: 1})
		require.True(t, ok)
		require.True(t, axis.Canonical())
	})
	t.Run("a general record reads AxisVec", func(t *testing.T) {
		axis, ok := coneaxis.OfRevolve(xy, revolveaxis.Frame{DU: 0.6, DV: 0.8}, r3.Vec{X: 3, Y: 4})
		require.True(t, ok)
		require.False(t, axis.Canonical())
		require.Equal(t, r3.Vec{X: 3, Y: 4}, axis.Vec())
		axis, ok = coneaxis.OfRevolve(zx, canonical, r3.Vec{Z: -2})
		require.True(t, ok)
		require.Equal(t, r3.Vec{Z: -2}, axis.Vec())
	})
	t.Run("a parallel axis of another length is a different axis", func(t *testing.T) {
		a, _ := coneaxis.OfRevolve(xy, revolveaxis.Frame{DU: 0.6, DV: 0.8}, r3.Vec{X: 3, Y: 4})
		b, _ := coneaxis.OfRevolve(xy, revolveaxis.Frame{DU: 0.6, DV: 0.8}, r3.Vec{X: 6, Y: 8})
		require.NotEqual(t, a, b)
		require.NotEqual(t, coneaxis.X(), a)
	})
	for name, tc := range map[string]struct {
		frame  r3.Frame
		ax     revolveaxis.Frame
		vector r3.Vec
	}{
		"zero AxisVec":         {xz, canonical, r3.Vec{}},
		"non-finite AxisVec":   {xz, canonical, r3.Vec{X: math.NaN()}},
		"AxisVec off plane":    {xz, canonical, r3.Vec{X: 1, Y: 1}},
		"frame origin moved":   {mustFrame(t, r3.Vec{X: 1}, r3.Vec{X: 1}, r3.Vec{Z: 1}), canonical, r3.Vec{X: 1}},
		"anchor moved":         {xz, revolveaxis.Frame{AU: 1, DU: 1}, r3.Vec{X: 1}},
		"anchor bound":         {xz, revolveaxis.Frame{AVBound: 1e-300, DU: 1}, r3.Vec{X: 1}},
		"non-coordinate U":     {mustFrame(t, r3.Vec{}, r3.Vec{X: 1, Y: 1}.Scale(1/math.Sqrt2), r3.Vec{Z: 1}), canonical, r3.Vec{X: 1}},
		"canonical with bound": {xy, revolveaxis.Frame{DU: 1, DUBound: 1e-300}, r3.Vec{}},
	} {
		t.Run("refuses "+name, func(t *testing.T) {
			_, ok := coneaxis.OfRevolve(tc.frame, tc.ax, tc.vector)
			require.False(t, ok)
		})
	}
}

func TestPlaneScaledMatchesRationalOracle(t *testing.T) {
	t.Parallel()
	xy := mustFrame(t, r3.Vec{}, r3.Vec{X: 1}, r3.Vec{Y: 1})
	xz := mustFrame(t, r3.Vec{}, r3.Vec{X: 1}, r3.Vec{Z: 1})
	zxNeg := mustFrame(t, r3.Vec{}, r3.Vec{Z: -1}, r3.Vec{X: 1})
	rng := rand.New(rand.NewPCG(5, 6))
	for _, tc := range []struct {
		frame r3.Frame
		axis  r3.Vec
		// aU, aV are A read in the frame's axes.
		aU, aV float64
	}{
		{xy, r3.Vec{X: 1, Y: 2}, 1, 2},
		{xy, r3.Vec{X: 1}, 1, 0},
		{xz, r3.Vec{X: 1, Z: 2}, 1, 2},
		{xz, r3.Vec{Z: 1}, 0, 1},
		{zxNeg, r3.Vec{X: 2, Z: -1}, 1, 2},
	} {
		axis, ok := coneaxis.New(tc.axis)
		require.True(t, ok)
		for range 100 {
			u, v := float64(rng.IntN(1025)-512)/32, float64(rng.IntN(1025)-512)/32
			z, r := axis.PlaneScaled(tc.frame, u, v)
			wantZ := new(big.Rat).Add(new(big.Rat).Mul(rat(u), rat(tc.aU)), new(big.Rat).Mul(rat(v), rat(tc.aV)))
			wantR := new(big.Rat).Sub(new(big.Rat).Mul(rat(tc.aU), rat(v)), new(big.Rat).Mul(rat(tc.aV), rat(u)))
			require.Equal(t, 0, wantZ.Cmp(z.Rat()), "z of (%g, %g)", u, v)
			require.Equal(t, 0, wantR.Cmp(r.Rat()), "r of (%g, %g)", u, v)
		}
	}
	// A point on the axis line has r = 0 exactly.
	axis, _ := coneaxis.New(r3.Vec{X: 1, Y: 2})
	_, r := axis.PlaneScaled(xy, 1.5, 3)
	require.True(t, r.IsZero())
	_, r = axis.PlaneScaled(xy, math.Nextafter(1.5, 2), 3)
	require.NotZero(t, r.Sign())
}

func TestCanonicalIsExactlyPlusX(t *testing.T) {
	t.Parallel()
	for _, v := range []r3.Vec{{X: 2}, {X: -1}, {X: 1, Y: 0x1p-1000}, {X: math.Nextafter(1, 2)}, {Y: 1}} {
		axis, ok := coneaxis.New(v)
		require.True(t, ok)
		require.False(t, axis.Canonical(), "%v", v)
	}
	axis, ok := coneaxis.New(r3.Vec{X: 1})
	require.True(t, ok)
	require.True(t, axis.Canonical())
	require.Equal(t, coneaxis.X(), axis)
}

// RadialLowerOf over a point set bounds σ at every point of the set and
// equals RadialLower on a triangle's three vertices.
func TestRadialLowerOfBoundsEveryPoint(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewPCG(9, 10))
	for _, v := range testAxes {
		axis, _ := coneaxis.New(v)
		for range 50 {
			points := make([]r3.Vec, 2+rng.IntN(12))
			for i := range points {
				points[i] = dyadicPoint(rng)
			}
			lower, ok := axis.RadialLowerOf(points)
			require.True(t, ok)
			for _, p := range points {
				require.True(t, proofbound.RatSquareAtMost(lower, oracleCrossSq(p, v)), "%g exceeds σ(%v) about %v",
					lower, p, v)
			}
			tri := [3]r3.Vec{points[0], points[1], points[len(points)-1]}
			want, _ := axis.RadialLower(tri)
			got, _ := axis.RadialLowerOf(tri[:])
			require.Equal(t, want, got)
		}
	}
}

// ScaledDown and ScaledUp bracket q/|A| from the side §3.4 names. Each holds
// against the exact comparison of out²·(A·A) with q², read with the signs.
func TestScaledBracketsTheMillimetreValue(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewPCG(11, 12))
	// atMost reports whether out ≤ q/|A| exactly, and atLeast whether
	// out ≥ q/|A|.
	atMost := func(out float64, q, aa *big.Rat) bool {
		o := rat(out)
		cmp := new(big.Rat).Mul(new(big.Rat).Mul(o, o), aa).Cmp(new(big.Rat).Mul(q, q))
		if q.Sign() >= 0 {
			return o.Sign() <= 0 || cmp <= 0
		}
		return o.Sign() < 0 && cmp >= 0
	}
	atLeast := func(out float64, q, aa *big.Rat) bool {
		o := rat(out)
		cmp := new(big.Rat).Mul(new(big.Rat).Mul(o, o), aa).Cmp(new(big.Rat).Mul(q, q))
		if q.Sign() >= 0 {
			return o.Sign() >= 0 && cmp >= 0
		}
		return o.Sign() >= 0 || cmp <= 0
	}
	for _, v := range append(append([]r3.Vec(nil), testAxes...), r3.Vec{X: 2}, r3.Vec{Y: 0.5}) {
		axis, _ := coneaxis.New(v)
		aa := oracleDot(v, v)
		for i := range 200 {
			q := new(big.Rat).SetFrac64(int64(rng.IntN(20001)-10000), int64(1+rng.IntN(97)))
			if i == 0 {
				q = new(big.Rat)
			}
			down, ok := axis.ScaledDown(q)
			require.True(t, ok)
			up, ok := axis.ScaledUp(q)
			require.True(t, ok)
			require.True(t, atMost(down, q, aa), "ScaledDown(%s) = %g about %v", q, down, v)
			require.True(t, atLeast(up, q, aa), "ScaledUp(%s) = %g about %v", q, up, v)
		}
	}
	_, ok := coneaxis.Axis{}.ScaledDown(big.NewRat(1, 1))
	require.False(t, ok, "the zero axis has no positive norm bracket")
}

// oracleTriangleMinSq is the exact minimum of σ(p)² = |p|²(A·A) − (p·A)² over
// the closed triangle, in big.Rat arithmetic on the triangle's barycentric
// form p = a + s·(b − a) + t·(c − a). It minimises the quadratic in (s, t):
// the interior stationary point when the 2×2 system is regular and its
// solution lies in the triangle, and otherwise the three edges, each a
// clamped one-dimensional quadratic. A singular quadratic is constant along
// its null direction, so its minimum over the triangle is on an edge.
func oracleTriangleMinSq(tri [3]r3.Vec, axis r3.Vec) *big.Rat {
	sigma := func(p [3]*big.Rat) *big.Rat {
		a := ratVec(axis)
		pp, aa, pa := new(big.Rat), new(big.Rat), new(big.Rat)
		for i := range 3 {
			pp.Add(pp, new(big.Rat).Mul(p[i], p[i]))
			aa.Add(aa, new(big.Rat).Mul(a[i], a[i]))
			pa.Add(pa, new(big.Rat).Mul(p[i], a[i]))
		}
		return new(big.Rat).Sub(new(big.Rat).Mul(pp, aa), new(big.Rat).Mul(pa, pa))
	}
	at := func(s, t *big.Rat) [3]*big.Rat {
		a, b, c := ratVec(tri[0]), ratVec(tri[1]), ratVec(tri[2])
		var p [3]*big.Rat
		for i := range 3 {
			p[i] = new(big.Rat).Add(a[i], new(big.Rat).Add(
				new(big.Rat).Mul(s, new(big.Rat).Sub(b[i], a[i])), new(big.Rat).Mul(t, new(big.Rat).Sub(c[i], a[i]))))
		}
		return p
	}
	zero, one, half := new(big.Rat), big.NewRat(1, 1), big.NewRat(1, 2)
	// Q(s, t) = q0 + gs·s + gt·t + hss·s² + 2·hst·s·t + htt·t², read from
	// samples, which a quadratic determines exactly.
	q := func(s, t *big.Rat) *big.Rat { return sigma(at(s, t)) }
	q00, q10, q01, q11 := q(zero, zero), q(one, zero), q(zero, one), q(one, one)
	q20 := q(big.NewRat(2, 1), zero)
	q02 := q(zero, big.NewRat(2, 1))
	hss := new(big.Rat).Mul(half, new(big.Rat).Add(new(big.Rat).Sub(q20, new(big.Rat).Mul(big.NewRat(2, 1), q10)), q00))
	htt := new(big.Rat).Mul(half, new(big.Rat).Add(new(big.Rat).Sub(q02, new(big.Rat).Mul(big.NewRat(2, 1), q01)), q00))
	gs := new(big.Rat).Sub(new(big.Rat).Sub(q10, q00), hss)
	gt := new(big.Rat).Sub(new(big.Rat).Sub(q01, q00), htt)
	hst := new(big.Rat).Mul(half, new(big.Rat).Sub(new(big.Rat).Sub(new(big.Rat).Sub(q11, q00),
		new(big.Rat).Add(gs, gt)), new(big.Rat).Add(hss, htt)))
	best := (*big.Rat)(nil)
	consider := func(v *big.Rat) {
		if best == nil || v.Cmp(best) < 0 {
			best = v
		}
	}
	det := new(big.Rat).Sub(new(big.Rat).Mul(hss, htt), new(big.Rat).Mul(hst, hst))
	if det.Sign() > 0 {
		// ∇Q = 0: 2·hss·s + 2·hst·t = −gs, 2·hst·s + 2·htt·t = −gt.
		s := new(big.Rat).Quo(new(big.Rat).Sub(new(big.Rat).Mul(hst, gt), new(big.Rat).Mul(htt, gs)),
			new(big.Rat).Mul(big.NewRat(2, 1), det))
		t := new(big.Rat).Quo(new(big.Rat).Sub(new(big.Rat).Mul(hst, gs), new(big.Rat).Mul(hss, gt)),
			new(big.Rat).Mul(big.NewRat(2, 1), det))
		if s.Sign() >= 0 && t.Sign() >= 0 && new(big.Rat).Add(s, t).Cmp(one) <= 0 {
			consider(q(s, t))
		}
	}
	// Each edge is u ↦ Q(e0 + u·(e1 − e0)) on [0, 1], read from three samples.
	for _, edge := range [3][2][2]*big.Rat{{{zero, zero}, {one, zero}}, {{one, zero}, {zero, one}},
		{{zero, one}, {zero, zero}}} {
		point := func(u *big.Rat) *big.Rat {
			s := new(big.Rat).Add(edge[0][0], new(big.Rat).Mul(u, new(big.Rat).Sub(edge[1][0], edge[0][0])))
			t := new(big.Rat).Add(edge[0][1], new(big.Rat).Mul(u, new(big.Rat).Sub(edge[1][1], edge[0][1])))
			return q(s, t)
		}
		e0, e1, eh := point(zero), point(one), point(half)
		consider(e0)
		consider(e1)
		// e(u) = e0 + b·u + c·u² with c = 2(e0 + e1 − 2eh) and b = e1 − e0 − c.
		c := new(big.Rat).Mul(big.NewRat(2, 1), new(big.Rat).Sub(new(big.Rat).Add(e0, e1),
			new(big.Rat).Mul(big.NewRat(2, 1), eh)))
		if c.Sign() > 0 {
			b := new(big.Rat).Sub(new(big.Rat).Sub(e1, e0), c)
			u := new(big.Rat).Quo(new(big.Rat).Neg(b), new(big.Rat).Mul(big.NewRat(2, 1), c))
			if u.Sign() > 0 && u.Cmp(one) < 0 {
				consider(point(u))
			}
		}
	}
	return best
}

// TriangleBeyond decides exactly whether a triangle clears the axis by more
// than a distance. Random dyadic triangles near each fixture axis are
// compared with an independent big.Rat minimisation, at distances on, just
// below and just above the exact minimum, so a triangle the axis pierces, one
// whose nearest point is inside an edge and one whose nearest point is a
// vertex all occur (agent-docs/revolve-axis-design.md §9.5).
func TestTriangleBeyondMatchesRationalOracle(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewPCG(7, 11))
	coord := func() float64 { return float64(rng.IntN(1<<12)-(1<<11)) / (1 << 9) }
	kinds := map[string]int{}
	for _, a := range testAxes {
		axis, ok := coneaxis.New(a)
		require.True(t, ok)
		aa := oracleDot(a, a)
		for range 300 {
			var tri [3]r3.Vec
			for i := range tri {
				tri[i] = r3.Vec{X: coord(), Y: coord(), Z: coord()}
			}
			minSq := oracleTriangleMinSq(tri, a)
			vertexMin := (*big.Rat)(nil)
			for _, p := range tri {
				if v := oracleCrossSq(p, a); vertexMin == nil || v.Cmp(vertexMin) < 0 {
					vertexMin = v
				}
			}
			switch {
			case minSq.Sign() == 0:
				kinds["pierced"]++
			case minSq.Cmp(vertexMin) < 0:
				kinds["interior"]++
			default:
				kinds["vertex"]++
			}
			exact := new(big.Rat).Quo(minSq, aa)
			distance := proofbound.RatSqrtDown(exact)
			for _, d := range []float64{distance, math.Nextafter(distance, 0), math.Nextafter(distance, math.Inf(1)), 0} {
				dr := rat(d)
				want := exact.Cmp(new(big.Rat).Mul(dr, dr)) > 0
				require.Equal(t, want, axis.TriangleBeyond(tri, d), "axis %v triangle %v distance %g", a, tri, d)
			}
		}
	}
	for _, kind := range []string{"pierced", "interior", "vertex"} {
		require.Positive(t, kinds[kind], "no %s triangle drawn", kind)
	}
	require.False(t, coneaxis.X().TriangleBeyond([3]r3.Vec{{Y: 1}, {Y: 2}, {Y: math.NaN()}}, 0.5))
	require.False(t, coneaxis.X().TriangleBeyond([3]r3.Vec{{Y: 1}, {Y: 2}, {Y: 3}}, math.Inf(1)))
}
