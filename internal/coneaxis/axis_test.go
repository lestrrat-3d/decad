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
