package decad

import (
	"math"
	"math/big"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file tests bounds.go's perturbedTriangleAreaAllow, the one new
// helper docs/loft-design.md §12 PR 2a extracts from
// perturbedAreaUpperWithBudget's own per-facet term, rigidRoundAllow's
// own saturated-scale answer, which that PR's placement path is the first
// consumer to reach at an extreme coordinate, and boundedSqrt's own two
// arms: the exact answer a zero-bound operand keeps, and the outward step a
// genuinely bounded one still receives, and boundedFloatError, the bridge a
// producer crosses when it evaluates a quantity one way and proves it another.
// It also owns the outward-rounding primitives every other helper in that file
// is built on — productUpper and divUpper — where the rule under test is that
// a bound may vanish only when a term is genuinely ABSENT, never because
// float64 flushed it.

// TestRigidRoundAllowIsAlwaysAFiniteBound pins bounds.go's rigidRoundAllow
// answering a finite, positive bound at every magnitude, including the ones
// whose 2·maxInputAbs + maxTransAbs scale leaves the finite float64 range.
// NaN is the failure this guards: it is not a large bound but the absence of
// one, and a consumer's own `delta > 0` widening skips it silently, so a
// placement's volume term would vanish from the very measurements it is
// there to widen.
func TestRigidRoundAllowIsAlwaysAFiniteBound(t *testing.T) {
	// The largest scale whose ulp is still readable: 2·maxInputAbs +
	// maxTransAbs sits one binade below MaxFloat64, so nothing saturates and
	// the answer is the plain 16-ulp charge.
	unsaturated := rigidRoundAllow(math.Ldexp(1, 1021), 0)
	require.Equal(t, radius3D(16*math.Ldexp(1, 1022-52)), unsaturated)

	for _, tc := range []struct {
		name       string
		input      float64
		trans      float64
		mustExceed float64
	}{
		{"ordinary magnitudes", 100, 10, 0},
		// 2*x overflows for any coordinate above MaxFloat64/2 — the
		// reported reproduction's own magnitude.
		{"input scale overflows", 0.75 * math.MaxFloat64, 1, unsaturated},
		{"both at the ceiling", math.MaxFloat64, math.MaxFloat64, unsaturated},
		// ulpOf itself answers +Inf exactly at MaxFloat64, because its own
		// math.Nextafter step leaves the finite range.
		{"scale lands exactly on MaxFloat64", math.MaxFloat64 / 2, 0, unsaturated},
	} {
		t.Run(tc.name, func(t *testing.T) {
			got := rigidRoundAllow(tc.input, tc.trans)
			require.False(t, math.IsNaN(got), "a NaN bound is no bound at all")
			require.False(t, math.IsInf(got, 0), "the bound must stay finite")
			require.Greater(t, got, 0.0)
			require.GreaterOrEqual(t, got, tc.mustExceed,
				"a saturated scale must charge at least what the largest readable scale does")
		})
	}
}

// TestRigidRoundAllowEnclosesAPlacedCoordinate proves the saturated answer is
// a real bound and not merely a finite number: at a coordinate whose scale
// saturates, the helper's answer must still enclose the displacement an
// actual rigid motion commits on that coordinate.
func TestRigidRoundAllowEnclosesAPlacedCoordinate(t *testing.T) {
	x := 0.75 * math.MaxFloat64
	allow := rigidRoundAllow(x, 1)

	// A rotation about Z by 37 degrees, then a 1 mm translation: the placed
	// coordinates round inside their own products and sums, which is exactly
	// what allow speaks for.
	rot, err := r3.Rotation(r3.NewVec(0, 0, 1), units.Degrees(37))
	require.NoError(t, err)
	trans, err := r3.Translation(r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	move, err := rot.Then(trans)
	require.NoError(t, err)

	for _, p := range []r3.Vec{
		r3.NewVec(x, 0, 0),
		r3.NewVec(x/2, x/4, 0),
		r3.NewVec(-x/2, 0, x/8),
	} {
		got := move.Apply(p)
		require.True(t, finiteVec(got), "allow speaks only for a placed point the caller has proven finite")

		want := exactApply(move, p)
		dist := new(big.Float).SetPrec(400)
		for i, held := range []float64{got.X, got.Y, got.Z} {
			d := new(big.Float).SetPrec(400).Sub(big.NewFloat(held).SetPrec(400), want[i])
			dist.Add(dist, d.Mul(d, d))
		}
		require.Positive(t, dist.Sign(), "the fixture must actually commit rounding, or it proves nothing")
		require.LessOrEqual(t, dist.Cmp(new(big.Float).SetPrec(400).Mul(big.NewFloat(allow), big.NewFloat(allow))), 0,
			"the placed point %v sits further from its exact image than the %.17g bound allows", p, allow)
	}
}

// exactApply is move.Apply in arbitrary precision: the same three products,
// two sums and translation, with no rounding of its own.
func exactApply(move r3.Transform, p r3.Vec) [3]*big.Float {
	b := move.Basis()
	tr := move.Translation()

	term := func(basis, coord float64) *big.Float {
		return new(big.Float).SetPrec(400).Mul(big.NewFloat(basis).SetPrec(400), big.NewFloat(coord).SetPrec(400))
	}
	axis := func(bx, by, bz, t float64) *big.Float {
		sum := new(big.Float).SetPrec(400).Add(term(bx, p.X), term(by, p.Y))
		sum.Add(sum, term(bz, p.Z))
		return sum.Add(sum, big.NewFloat(t).SetPrec(400))
	}
	return [3]*big.Float{
		axis(b.EX.X, b.EY.X, b.EZ.X, tr.X),
		axis(b.EX.Y, b.EY.Y, b.EZ.Y, tr.Y),
		axis(b.EX.Z, b.EY.Z, b.EZ.Z, tr.Z),
	}
}

// TestWalkEndpointAllow pins docs/prism-boolean-design.md §7's δ_walk
// mechanism (task fu143): a positive OPERAND envelope — the magnitude of the
// values the walk's own arithmetic touches, which is what this helper's
// contract requires and never the endpoint that arithmetic produced — answers
// a bound at least the worst-case lerp2 rounding its own doc comment derives
// at that magnitude, a zero envelope answers zero, and a non-finite envelope
// answers +Inf rather than a number a caller's own `> 0` widening could skip.
//
// That the CALLER hands over the right envelope is walkChargeOf's own claim,
// proven against exact rational residuals over cancelling carriers in
// prism_boolean_internal_test.go's TestWalkChargeOfCoversLerpCancellation.
func TestWalkEndpointAllow(t *testing.T) {
	t.Run("zero envelope gives zero", func(t *testing.T) {
		require.Equal(t, 0.0, walkEndpointAllow(0))
	})

	t.Run("positive envelope exceeds the derived lerp2 rounding at that magnitude", func(t *testing.T) {
		for _, envelope := range []float64{12, 1, 1e-3, 1e6, 1e12} {
			got := walkEndpointAllow(envelope)
			require.Positive(t, got)
			require.False(t, math.IsInf(got, 0))
			// The helper's own derivation: lerp2's general arm rounds three
			// times — the difference, the product, the sum — for at most
			// 5·u·E per coordinate at operand magnitude E. The published
			// bound is a 3D radius over 16 ulps of 2E and must clear it.
			require.Greaterf(t, got, 5*unitRoundoff*envelope,
				"the bound at envelope %g must contain the derived per-coordinate worst case", envelope)
			require.Greaterf(t, got, ulpOf(envelope), "the bound at envelope %g must exceed one ulp there", envelope)
		}
	})

	t.Run("larger envelope gives a larger bound", func(t *testing.T) {
		require.Greater(t, walkEndpointAllow(1e6), walkEndpointAllow(12))
	})

	for _, tc := range []struct {
		name  string
		input float64
	}{
		{"+Inf", math.Inf(1)},
		{"NaN", math.NaN()},
	} {
		t.Run(tc.name+" envelope gives +Inf, never 0", func(t *testing.T) {
			got := walkEndpointAllow(tc.input)
			require.True(t, math.IsInf(got, 1), "an absent bound must never read as a small one")
		})
	}
}

// TestPerturbedTriangleAreaAllowEnclosesBruteForceSweep proves the
// extracted helper against a brute-force sweep of perturbed vertices
// (required test, §13): for every one of many random perturbations of up
// to delta per vertex, the resulting triangle's own area must lie within
// [heldArea - allow, heldArea + allow], at aspect ratios 1, 1e-3 and 1e-6 —
// the same slivers loft_moments_internal_test.go's area-bound tests use,
// since a thin triangle is exactly where a naive bound scaled off the held
// total stops enclosing anything.
func TestPerturbedTriangleAreaAllowEnclosesBruteForceSweep(t *testing.T) {
	triArea := func(a, b, c r3.Vec) float64 {
		return b.Sub(a).Cross(c.Sub(a)).Len() / 2
	}

	// Two orthonormal, non-axis-aligned directions, matching
	// loft_moments_internal_test.go's own sliver construction.
	dir := r3.NewVec(0.6, 0.8, 0)
	perp := r3.NewVec(-0.48, 0.36, 0.8)
	require.InDelta(t, 0, dir.Dot(perp), 1e-15)

	sliver := func(base, aspect float64) (r3.Vec, r3.Vec, r3.Vec) {
		origin := r3.NewVec(3, -5, 7)
		a := origin
		b := origin.Add(dir.Scale(base))
		c := origin.Add(dir.Scale(base * 0.37)).Add(perp.Scale(base * aspect))
		return a, b, c
	}

	rng := rand.New(rand.NewPCG(1, 2))
	randomUnit := func() r3.Vec {
		for {
			v := r3.NewVec(rng.Float64()*2-1, rng.Float64()*2-1, rng.Float64()*2-1)
			if l := v.Len(); l > 1e-9 {
				return v.Scale(1 / l)
			}
		}
	}

	for _, tc := range []struct {
		name   string
		aspect float64
		delta  float64
	}{
		{"aspect 1", 1, 1e-3},
		{"aspect 1e-3", 1e-3, 1e-3},
		{"aspect 1e-6", 1e-6, 1e-9},
	} {
		t.Run(tc.name, func(t *testing.T) {
			a, b, c := sliver(10, tc.aspect)
			held := triArea(a, b, c)
			allow := perturbedTriangleAreaAllow(a, b, c, tc.delta)
			require.Greater(t, allow, 0.0)

			for range 20000 {
				pa := a.Add(randomUnit().Scale(rng.Float64() * tc.delta))
				pb := b.Add(randomUnit().Scale(rng.Float64() * tc.delta))
				pc := c.Add(randomUnit().Scale(rng.Float64() * tc.delta))
				got := triArea(pa, pb, pc)
				require.LessOrEqualf(t, math.Abs(got-held), allow,
					"perturbed area %.17g must stay within %.17g of held %.17g", got, allow, held)
			}
		})
	}
}

// requireEnclosesSqrt proves a boundedSqrt answer's own interval contains the
// TRUE square root of q, by comparing the interval's exactly squared ends
// against q over the rationals — never against another float64 square root of
// the same operand, which would only compare one evaluation with itself.
func requireEnclosesSqrt(t *testing.T, q *big.Rat, got boundedScalar) {
	t.Helper()
	v, b := floatRat(got.value), floatRat(got.bound)
	require.NotNil(t, v)
	require.NotNil(t, b)
	lo := new(big.Rat).Sub(v, b)
	hi := new(big.Rat).Add(v, b)
	if lo.Sign() > 0 {
		require.LessOrEqual(t, new(big.Rat).Mul(lo, lo).Cmp(q), 0,
			`the interval's lower end sits above the true root`)
	}
	require.GreaterOrEqual(t, new(big.Rat).Mul(hi, hi).Cmp(q), 0,
		`the interval's upper end sits below the true root`)
}

// TestBoundedSqrtKeepsAZeroBoundOperandExact pins the arm the analytic surveys
// publish their exact readings through. A zero-bound operand's interval ends
// are its own held value — adding or subtracting exactly zero rounds nothing —
// so the rational brackets answer a zero bound precisely when that value is a
// perfect square of a float64, and a genuine directed-rounding bound when it is
// not.
func TestBoundedSqrtKeepsAZeroBoundOperandExact(t *testing.T) {
	for _, tc := range []struct {
		name string
		in   float64
		want float64
	}{
		{"a perfect square", 25, 5},
		{"zero", 0, 0},
		{"a power of four", 1024, 32},
		{"a fractional perfect square", 0.0625, 0.25},
	} {
		t.Run(tc.name, func(t *testing.T) {
			got := boundedSqrt(exactScalar(tc.in))
			require.Equal(t, tc.want, got.value)
			require.Equal(t, 0.0, got.bound)
		})
	}

	// boundedHypot (survey2d.go) reads two exact leaves through this same arm,
	// which is how a straight meridian's own tangent reaches it in
	// revolveMinRadius.
	h := boundedHypot(10, 0)
	require.Equal(t, 10.0, h.value)
	require.Equal(t, 0.0, h.bound)

	// An exact operand that is not a perfect square still reads bounded: no
	// float64 holds √2, so the brackets straddle it and the interval encloses
	// the truth.
	two := boundedSqrt(exactScalar(2))
	require.Greater(t, two.bound, 0.0)
	requireEnclosesSqrt(t, big.NewRat(2, 1), two)
}

// TestBoundedSqrtWidensABoundedOperand pins the other half: an operand that
// carries a bound keeps the outward step on both interval ends, because
// x.value ± x.bound is itself a rounded float operation. 1 ± 1e-17 is the case
// that proves the step load-bearing — both sums round straight back to 1.0, so
// without it the operand's own uncertainty would vanish and a perfect square
// would read exact while the true operand is not one.
func TestBoundedSqrtWidensABoundedOperand(t *testing.T) {
	got := boundedSqrt(measuredScalar(1, 1e-17))
	require.Equal(t, 1.0, got.value)
	require.Greater(t, got.bound, 0.0)

	one := big.NewRat(1, 1)
	tiny := floatRat(1e-17)
	requireEnclosesSqrt(t, new(big.Rat).Add(one, tiny), got)
	requireEnclosesSqrt(t, new(big.Rat).Sub(one, tiny), got)

	// A bound wide enough to survive the sum is enclosed on the same terms.
	wide := boundedSqrt(measuredScalar(4, 1e-6))
	require.Greater(t, wide.bound, 0.0)
	four := big.NewRat(4, 1)
	micro := floatRat(1e-6)
	requireEnclosesSqrt(t, new(big.Rat).Add(four, micro), wide)
	requireEnclosesSqrt(t, new(big.Rat).Sub(four, micro), wide)
}

// TestBoundedFloatErrorEnclosesEveryTruthTheScalarAdmits pins boundedFloatError's
// whole contract: the answer must bound |held − t| for EVERY t the bounded
// scalar admits, not merely the gap to its held centre. The truth is swept over
// the enclosure's own ends and interior at 200 bits, so a term dropped from the
// outward sum shows up as a case the answer fails to cover.
func TestBoundedFloatErrorEnclosesEveryTruthTheScalarAdmits(t *testing.T) {
	cases := []struct {
		name string
		bs   boundedScalar
		held float64
	}{
		{"exact scalar read back exactly", exactScalar(1.5), 1.5},
		{"exact scalar read one ulp off", exactScalar(1.5), math.Nextafter(1.5, math.Inf(1))},
		{"bounded scalar centred on the held value", measuredScalar(2, 1e-12), 2},
		{"bounded scalar the held value sits below", measuredScalar(2, 1e-12), 2 - 3e-13},
		{"bounded scalar the held value sits above", measuredScalar(2, 1e-12), 2 + 7e-13},
		{"held far from a tight enclosure", measuredScalar(1e6, 1e-9), 1e6 + 1e-6},
		{"negative operands", measuredScalar(-4.25, 1e-11), -4.25 - 4e-12},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			got := boundedFloatError(tc.bs, tc.held)
			require.False(t, isNonFinite(got), "a finite operand must never answer non-finite")
			require.GreaterOrEqual(t, got, 0.0)

			held := new(big.Float).SetPrec(200).SetFloat64(tc.held)
			centre := new(big.Float).SetPrec(200).SetFloat64(tc.bs.value)
			bound := new(big.Float).SetPrec(200).SetFloat64(tc.bs.bound)
			lo := new(big.Float).SetPrec(200).Sub(centre, bound)
			hi := new(big.Float).SetPrec(200).Add(centre, bound)
			answer := new(big.Float).SetPrec(200).SetFloat64(got)

			// Sweep the enclosure: both ends, the centre, and interior points.
			const steps = 64
			span := new(big.Float).SetPrec(200).Sub(hi, lo)
			for k := 0; k <= steps; k++ {
				frac := new(big.Float).SetPrec(200).Quo(
					new(big.Float).SetPrec(200).SetInt64(int64(k)),
					new(big.Float).SetPrec(200).SetInt64(int64(steps)),
				)
				truth := new(big.Float).SetPrec(200).Add(lo,
					new(big.Float).SetPrec(200).Mul(span, frac))
				gap := new(big.Float).SetPrec(200).Sub(held, truth)
				gap.Abs(gap)
				require.LessOrEqual(t, gap.Cmp(answer), 0,
					"the answer must bound the gap to every truth the scalar admits; k=%d", k)
			}
		})
	}
}

// TestBoundedFloatErrorRefusesANonFiniteOperand pins the rule that an absent
// bound reads as +Inf and never as a small one: answering 0 here would publish
// a saturated quantity as exactly known.
func TestBoundedFloatErrorRefusesANonFiniteOperand(t *testing.T) {
	inf := math.Inf(1)
	require.True(t, math.IsInf(boundedFloatError(measuredScalar(inf, 1), 1), 1))
	require.True(t, math.IsInf(boundedFloatError(measuredScalar(1, inf), 1), 1))
	require.True(t, math.IsInf(boundedFloatError(measuredScalar(1, 1), inf), 1))
	require.True(t, math.IsInf(boundedFloatError(measuredScalar(math.NaN(), 1), 1), 1))
}

// TestSnapToZeroAllowEnclosesTheOverwrittenCoordinate pins the composition a
// deliberate snap needs: the assigned zero's own interval must still reach the
// coordinate the assignment overwrote, wherever that coordinate's pre-snap
// bound had already put it.
func TestSnapToZeroAllowEnclosesTheOverwrittenCoordinate(t *testing.T) {
	for _, tc := range []struct {
		name       string
		bound      float64
		coordinate float64
	}{
		{"an exact pre-snap coordinate", 0, 5e-10},
		{"a bounded pre-snap coordinate", 1e-12, 5e-10},
		{"a coordinate under its own bound", 3e-12, 1e-12},
	} {
		t.Run(tc.name, func(t *testing.T) {
			got := snapToZeroAllow(tc.bound, math.Abs(tc.coordinate))
			// The truth is anywhere within the pre-snap bound of the
			// coordinate; the farthest it can sit from the assigned zero is
			// the sum of the two.
			require.GreaterOrEqual(t, got, tc.coordinate+tc.bound,
				`the assigned zero's interval must reach the coordinate it replaced`)
		})
	}

	// A coordinate the arithmetic already put exactly on zero discards
	// nothing, so its own exactness survives the assignment.
	require.Equal(t, 0.0, snapToZeroAllow(0, 0))
	require.Equal(t, 1e-12, snapToZeroAllow(1e-12, 0))

	// A magnitude no float states is the ABSENCE of a charge, never a zero
	// one: answering the caller's own bound would let the assignment vanish.
	require.True(t, math.IsInf(snapToZeroAllow(0, math.NaN()), 1))
	require.True(t, math.IsInf(snapToZeroAllow(0, math.Inf(1)), 1))
}

// TestOutwardRoundingNeverPublishesAFlushedZero pins the rule productUpper and
// divUpper carry for every bound in this package: two operands the caller has
// proven POSITIVE can never answer 0.
//
// float64 rounds a positive result to +0 as soon as that result falls below
// half the smallest subnormal, and upRound cannot undo it — 0 is not a
// representable neighbour away from a positive number, it IS the rounded
// answer. The consequence is not a slightly loose bound but a categorically
// false one: exactnessOf reads a zero bound as the CLAIM that the value it
// bounds is exactly representable, so every flushed product publishes an Exact
// measurement for a body whose error merely happens to be tiny.
//
// math.SmallestNonzeroFloat64 is the correct answer rather than a fudge.
// Rounding to +0 PROVES the exact result sits at or below half the smallest
// subnormal, so the smallest subnormal encloses it — and stays finite, which a
// +Inf refusal would not, and which the consumers that gate on a positive
// bound need.
func TestOutwardRoundingNeverPublishesAFlushedZero(t *testing.T) {
	const tiny = math.SmallestNonzeroFloat64

	t.Run("product", func(t *testing.T) {
		for _, tc := range []struct {
			name string
			a, b float64
		}{
			{"both factors far below the subnormal floor", 1e-200, 1e-200},
			{"a subnormal factor scaled down", 1e-320, 1e-10},
			{"the smallest subnormal halved", tiny, 0.5},
		} {
			t.Run(tc.name, func(t *testing.T) {
				require.Equal(t, 0.0, tc.a*tc.b, "the fixture must actually exercise a flush")
				require.Equal(t, tiny, productUpper(tc.a, tc.b))
				require.Equal(t, Approximate, exactnessOf(productUpper(tc.a, tc.b)))
			})
		}
	})

	t.Run("quotient", func(t *testing.T) {
		for _, tc := range []struct {
			name     string
			num, den float64
		}{
			{"a subnormal numerator over a large denominator", 1e-320, 1e10},
			{"the smallest subnormal quartered", tiny, 4},
		} {
			t.Run(tc.name, func(t *testing.T) {
				require.Equal(t, 0.0, tc.num/tc.den, "the fixture must actually exercise a flush")
				require.Equal(t, tiny, divUpper(tc.num, tc.den))
				require.Equal(t, Approximate, exactnessOf(divUpper(tc.num, tc.den)))
			})
		}
	})

	// An operand at or below zero is an ABSENT term, not a flushed one: it is
	// the only way a product or quotient legitimately vanishes, and the only
	// zero exactnessOf may read as a claim of exactness. Widening it would
	// convert every genuinely exact reading in the package into Approximate.
	require.Equal(t, 0.0, productUpper(0, 1e300))
	require.Equal(t, 0.0, productUpper(1e300, 0))
	require.Equal(t, 0.0, productUpper(-1, 5))
	require.Equal(t, 0.0, divUpper(0, 2))
	require.Equal(t, 0.0, divUpper(-1, 2))

	// An ordinary product and quotient still round OUTWARD.
	require.GreaterOrEqual(t, productUpper(3, 5), 15.0)
	require.GreaterOrEqual(t, divUpper(15, 5), 3.0)
	require.GreaterOrEqual(t, productUpper(0.1, 0.1), 0.01)
	require.GreaterOrEqual(t, divUpper(1, 3), 1.0/3.0)
}

// TestProductUpperRefusesRatherThanAnnihilatesARefusal pins the second half of
// the same family: +Inf is a REFUSAL, not a magnitude, so an absent factor may
// not cancel it away.
//
// The arithmetic identity Inf*0 = NaN is not what this guards — productUpper
// short-circuits on its own operand tests, so a +Inf paired with a 0 used to
// take the "absent term" arm and publish a PROVEN zero for a term whose scale
// nothing had bounded. cellChordCurveAreaAllow reaches it directly: its beta
// factor overflows to +Inf at a large rung while its energy sum vanishes at a
// small one, and the sharper arm then wins the final min with a bound of 0.
func TestProductUpperRefusesRatherThanAnnihilatesARefusal(t *testing.T) {
	for name, ab := range map[string][2]float64{
		"refusal first":  {math.Inf(1), 0},
		"refusal second": {0, math.Inf(1)},
		"both refused":   {math.Inf(1), math.Inf(1)},
		"refusal scaled": {math.Inf(1), 3},
	} {
		require.True(t, math.IsInf(productUpper(ab[0], ab[1]), 1), "%s must answer +Inf", name)
	}

	// A denominator that states no scale — zero, negative, or itself
	// overflowed — is a broken caller claim and answers +Inf, never a bound.
	for name, nd := range map[string][2]float64{
		"zero denominator":       {1, 0},
		"negative denominator":   {1, -1},
		"overflowed denominator": {1, math.Inf(1)},
		"NaN denominator":        {1, math.NaN()},
		"NaN numerator":          {math.NaN(), 1},
		"refused numerator":      {math.Inf(1), 2},
	} {
		require.True(t, math.IsInf(divUpper(nd[0], nd[1]), 1), "%s must answer +Inf", name)
	}
}
