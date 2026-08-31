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

// TestProductUpperRefusesRatherThanAnnihilatesARefusal pins the second half of
// the family TestOutwardRoundingNeverPublishesAFlushedZero
// (bounds_flush_internal_test.go) opens: +Inf is a REFUSAL, not a magnitude, so
// an absent factor may not cancel it away.
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

// This file tests ONE rule across bounds.go's outward-rounding primitives and
// every chorded-loft bound built on them: a bound may publish 0 only when the
// quantity it bounds is genuinely ABSENT, never because float64's own
// arithmetic flushed a positive result to +0.
//
// float64 rounds a positive product or quotient to +0 as soon as that result
// falls below half the smallest subnormal, and upRound cannot undo it — 0 is
// not a representable neighbour away from a positive number, it IS the rounded
// answer. The consequence is not a slightly loose bound but an UNSOUND one:
// the published interval [value−bound, value+bound] collapses to a point that
// excludes the truth, and exactnessOf reads the same 0 as the CLAIM that the
// value is exactly representable, so a flushed cell publishes an Exact
// measurement for a body whose error is merely tiny.
//
// math.SmallestNonzeroFloat64 is the correct replacement rather than a fudge:
// rounding to +0 PROVES the exact result sits at or below half the smallest
// subnormal, so the smallest subnormal encloses it — and stays FINITE, which a
// +Inf refusal would not, and which the consumers that gate on a positive
// bound (chordedBoundaryVolumeAllow's own wallAreaUpper > 0 branch) need.
//
// No leg below pins a float literal a platform's own FMA contraction could
// move: each asserts the SIGN of the published bound and the exactness class
// it lands in, and the primitive legs pin only the smallest subnormal itself,
// which a single multiply or divide reaches identically on every target.

// TestOutwardRoundingNeverPublishesAFlushedZero pins the rule productUpper and
// divUpper carry for every bound in this package: two operands the helper has
// itself PROVEN positive can never answer 0.
//
// An operand at or below zero is an ABSENT term, not a flushed one. It is the
// only way a product or quotient legitimately vanishes, and the only zero
// exactnessOf may read as a claim of exactness, so it must keep answering an
// honest 0 — widening it would convert every genuinely exact reading in the
// package into Approximate.
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
				require.Equal(t, 0.0, tc.a*tc.b, `the fixture must actually exercise a flush`)
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
			{"the smallest subnormal thirded", tiny, 3},
		} {
			t.Run(tc.name, func(t *testing.T) {
				require.Equal(t, 0.0, tc.num/tc.den, `the fixture must actually exercise a flush`)
				require.Equal(t, tiny, divUpper(tc.num, tc.den))
				require.Equal(t, Approximate, exactnessOf(divUpper(tc.num, tc.den)))
			})
		}
	})

	t.Run("an absent term still publishes an honest zero", func(t *testing.T) {
		require.Equal(t, 0.0, productUpper(0, 1e300))
		require.Equal(t, 0.0, productUpper(1e300, 0))
		require.Equal(t, 0.0, productUpper(-1, 5))
		require.Equal(t, 0.0, divUpper(0, 2))
		require.Equal(t, 0.0, divUpper(-1, 2))
		require.Equal(t, Exact, exactnessOf(productUpper(0, 1e300)))
	})

	t.Run("an ordinary reading still rounds outward", func(t *testing.T) {
		require.GreaterOrEqual(t, productUpper(3, 5), 15.0)
		require.GreaterOrEqual(t, productUpper(0.1, 0.1), 0.01)
		require.GreaterOrEqual(t, divUpper(15, 5), 3.0)
		require.GreaterOrEqual(t, divUpper(1, 3), 1.0/3.0)
	})

	t.Run("a divisor that states no scale refuses", func(t *testing.T) {
		require.True(t, math.IsInf(divUpper(1, 0), 1))
		require.True(t, math.IsInf(divUpper(1, -2), 1))
		require.True(t, math.IsInf(divUpper(1, math.Inf(1)), 1))
		require.True(t, math.IsInf(divUpper(math.NaN(), 2), 1))
		require.True(t, math.IsInf(divUpper(1, math.NaN()), 1))
		// A numerator that already refuses stays a refusal rather than
		// becoming a finite quotient.
		require.True(t, math.IsInf(divUpper(math.Inf(1), 3), 1))
	})
}

// TestChordedBoundsNeverPublishAFlushedZero drives every chorded-loft bound
// this package publishes at subnormal geometry, where each helper's own
// certified operands are POSITIVE but their float product or quotient
// underflows.
//
// Each fixture states a claim the helper admits — no refusal arm fires — and
// bounds a quantity that is genuinely positive, so the published 0 each leg
// used to return excluded the very displacement it was there to enclose.
func TestChordedBoundsNeverPublishAFlushedZero(t *testing.T) {
	const s = math.SmallestNonzeroFloat64

	// A degenerate-in-magnitude but non-degenerate-in-shape cell: the four
	// corners are distinct, so every certified span is positive.
	var (
		vLo = r3.Vec{X: 0, Y: 0, Z: 0}
		vHi = r3.Vec{X: s, Y: 0, Z: 0}
		wLo = r3.Vec{X: 0, Y: s, Z: 0}
		wHi = r3.Vec{X: s, Y: s, Z: 0}
		// The same cell sheared out of plane so the exact twist vector
		// T = vLo − vHi − wLo + wHi is (s,0,s) and the swept
		// determinant is nonzero.
		twHi = r3.Vec{X: 2 * s, Y: s, Z: s}
	)

	t.Run("cell chord-to-curve area", func(t *testing.T) {
		// The reviewer's own reproduction. Both certified spans and both eA
		// and eB are positive, so the ruled patch this bounds carries
		// positive area — a zero here claims the wall cell's whole
		// chord-to-curve homotopy sweeps no area at all.
		got := cellChordCurveAreaUpper(vLo, vHi, wLo, wHi, 4*s, 4*s, 0)
		require.Greater(t, got, 0.0,
			`a cell with four distinct corners bounds a positive area`)
		require.Equal(t, Approximate, exactnessOf(got))
	})

	t.Run("cell twist volume", func(t *testing.T) {
		// The exact swept determinant is nonzero, so the ruled patch and the
		// built triangle pair carry positive swept measure between them.
		require.Greater(t, cellTwistQuarterUpper(vLo, vHi, wLo, twHi), 0.0,
			`the fixture must carry a certified nonzero twist`)
		got := cellTwistVolumeAllow(vLo, vHi, wLo, twHi)
		require.Greater(t, got, 0.0,
			`a twisted cell bounds a positive ruled-to-triangle volume`)
		require.Equal(t, Approximate, exactnessOf(got))
	})

	t.Run("cap area volume", func(t *testing.T) {
		// Both operands pass the helper's own positivity gate, so |h|·|ΔArea|/3
		// is positive and a zero would republish a moved cap as an unmoved one.
		got := capAreaVolumeAllow(s, s)
		require.Greater(t, got, 0.0,
			`a positive plane offset over a positive area gap bounds a positive volume`)
		require.Equal(t, Approximate, exactnessOf(got))
	})

	t.Run("cap area volume whose product survives but whose third flushes", func(t *testing.T) {
		// The product itself is representable; only the division by 3
		// underflows, so this leg falsifies the DIVIDE independently of the
		// multiply the leg above covers.
		require.Greater(t, productUpper(s, 0.1), 0.0,
			`the fixture must carry a positive numerator into the divide`)
		require.Equal(t, 0.0, productUpper(s, 0.1)/3,
			`the fixture must actually exercise a flush at the divide`)
		got := capAreaVolumeAllow(s, 0.1)
		require.Greater(t, got, 0.0, `the third of a positive volume is positive`)
		require.Equal(t, Approximate, exactnessOf(got))
	})

	t.Run("wall chord-to-curve leg", func(t *testing.T) {
		// matchedDelta and wallAreaUpper both pass the helper's own > 0
		// branch gate, so the wall leg matchedDelta·wallAreaUpper is positive
		// and the composed total cannot be zero.
		got := chordedBoundaryVolumeAllow(s, s, 0, 0, 0)
		require.Greater(t, got, 0.0,
			`a positive displacement over a positive wall area bounds a positive volume`)
		require.Equal(t, Approximate, exactnessOf(got))
	})

	t.Run("seam correction", func(t *testing.T) {
		got := chordedBoundarySeamAllow(s, s, s)
		require.Greater(t, got, 0.0,
			`three positive operands bound a positive line-integral residue`)
		require.Equal(t, Approximate, exactnessOf(got))
	})

	t.Run("seam correction whose product survives but whose third flushes", func(t *testing.T) {
		// As with the cap: the numerator is representable and only the
		// division by 3 underflows.
		require.Equal(t, 0.0, productUpper(s, productUpper(0.1, 1))/3,
			`the fixture must actually exercise a flush at the divide`)
		got := chordedBoundarySeamAllow(s, 0.1, 1)
		require.Greater(t, got, 0.0, `the third of a positive residue is positive`)
		require.Equal(t, Approximate, exactnessOf(got))
	})

	t.Run("moment", func(t *testing.T) {
		// The volume leg is positive, and the widened radius is positive
		// through matchedDelta alone, so the first moment this bounds is
		// positive whatever the held coordinate envelope.
		got := chordedBoundaryMomentAllow(s, s, 0, 0, 0, 0, 0)
		require.Greater(t, got, 0.0,
			`a positive displaced volume at a positive radius bounds a positive moment`)
		require.Equal(t, Approximate, exactnessOf(got))
	})

	t.Run("a genuinely absent term still publishes an honest zero", func(t *testing.T) {
		// Every zero below comes from a term that is ABSENT, not flushed, and
		// each must survive the change: these are the only readings in the
		// family exactnessOf may keep calling Exact.
		require.Equal(t, 0.0, cellChordCurveAreaUpper(vLo, vLo, vLo, vLo, 0, 0, 0))
		require.Equal(t, 0.0, cellTwistVolumeAllow(vLo, vHi, wLo, wHi))
		require.Equal(t, 0.0, capAreaVolumeAllow(0, 1))
		require.Equal(t, 0.0, capAreaVolumeAllow(1, 0))
		require.Equal(t, 0.0, chordedBoundaryVolumeAllow(0, 1, 0, 0, 0))
		require.Equal(t, 0.0, chordedBoundarySeamAllow(0, 1, 1))
		require.Equal(t, 0.0, chordedBoundarySeamAllow(1, 0, 1))
		require.Equal(t, 0.0, chordedBoundaryMomentAllow(0, 1, 0, 0, 0, 0, 0))
	})
}

// This file is the falsifier for bounds.go's PROVEN-NORM rule: a bound whose
// factors are vector lengths must form every one of them exactly and round it
// OUTWARD (heldDelta into rvLenUpper), never read r3.Vec.Len.
//
// r3.Vec.Len is math.Hypot and r3.Vec.Sub is a float subtraction, both
// round-to-NEAREST, so neither is an upper bound on anything. The composed
// helpers upRound/productUpper/absSumUpper buy back about one ulp of the
// PRODUCT, which happens to hide the shortfall on a well-conditioned cell and
// cannot begin to cover a difference that CANCELS — there the float norm's own
// relative error is unbounded, and the fixtures below carry one where it
// reaches eleven percent.
//
// Every reference value here is computed at refPrec bits from the corners'
// own EXACT rational differences, with no rounding helper of the production
// kind in it, so it is an independent statement of the quantity rather than a
// copy of the code that publishes it.

// refPrec is the working precision of every reference value in this file. A
// float64 carries 53 bits, so a reference this wide is exact for the purposes
// of the comparisons below whatever the platform's own sqrt does.
const refPrec = 600

func refFloat(x float64) *big.Float { return new(big.Float).SetPrec(refPrec).SetFloat64(x) }

// refLen is the EXACT length of an exactly-represented vector, to refPrec bits.
func refLen(u ratV3) *big.Float {
	return new(big.Float).SetPrec(refPrec).Sqrt(new(big.Float).SetPrec(refPrec).SetRat(rvDot(u, u)))
}

func refAdd(a, b *big.Float) *big.Float { return new(big.Float).SetPrec(refPrec).Add(a, b) }
func refMul(a, b *big.Float) *big.Float { return new(big.Float).SetPrec(refPrec).Mul(a, b) }
func refQuo(a, b *big.Float) *big.Float { return new(big.Float).SetPrec(refPrec).Quo(a, b) }

func refMax(a, b *big.Float) *big.Float {
	if a.Cmp(b) >= 0 {
		return a
	}
	return b
}

func refMin(a, b *big.Float) *big.Float {
	if a.Cmp(b) <= 0 {
		return a
	}
	return b
}

// cellQuad is one wall cell's four held corners, the argument shape every
// bound in this file takes.
type cellQuad struct {
	name                   string
	vLo, vHi, wLo, wHi     r3.Vec
	floatShortfallExpected bool
}

// provenNormFixtures carries the cells the float-norm route provably
// understates. The first three are the reviewer's own corners, kept exactly as
// reported; the fourth is the planar case the bound must answer zero for.
func provenNormFixtures() []cellQuad {
	return []cellQuad{
		{
			name:                   "large-integer corners",
			vLo:                    r3.NewVec(770749, 887007, 339646),
			vHi:                    r3.NewVec(453885, 39861, 565228),
			wLo:                    r3.NewVec(547783, 4864, 319470),
			wHi:                    r3.NewVec(717918, 666858, 444888),
			floatShortfallExpected: true,
		},
		{
			name:                   "plain one-decimal millimetres",
			vLo:                    r3.NewVec(16.6, 65.1, 173.7),
			vHi:                    r3.NewVec(102.7, 169.5, 76.2),
			wLo:                    r3.NewVec(50.3, 70.1, 192.2),
			wHi:                    r3.NewVec(118.7, 187.3, 114.7),
			floatShortfallExpected: true,
		},
		{
			// The twist vector vLo−vHi−wLo+wHi cancels here, so the float
			// evaluation of |T| carries a relative error no ulp of the
			// published product can cover.
			name:                   "cancelling twist",
			vLo:                    r3.NewVec(-84.36003204767742, 181.4494626808933, 664.7106105946635),
			vHi:                    r3.NewVec(-773.0950741165469, -460.5310403373682, 631.9781962110208),
			wLo:                    r3.NewVec(-478.7808202679367, 446.1003348508805, 49.71601328095505),
			wHi:                    r3.NewVec(-1167.5158623368056, -195.88016816738144, 16.983598897312188),
			floatShortfallExpected: true,
		},
		{
			// Two sections offset along their shared normal alone: T is zero
			// exactly, so the whole product must vanish exactly.
			name: "planar cell",
			vLo:  r3.NewVec(0, 0, 0),
			vHi:  r3.NewVec(3, 4, 0),
			wLo:  r3.NewVec(0, 0, 7),
			wHi:  r3.NewVec(3, 4, 7),
		},
	}
}

// refTwistAreaProduct is |T|·(eA+eB) computed exactly from the cell's own
// corners, the quantity the linear fallback arm publishes.
func refTwistAreaProduct(c cellQuad) *big.Float {
	twist := rvSub(heldDelta(c.vLo, c.vHi), heldDelta(c.wLo, c.wHi))
	eA := refMax(refLen(heldDelta(c.vHi, c.vLo)), refLen(heldDelta(c.wHi, c.wLo)))
	eB := refMax(refLen(heldDelta(c.wLo, c.vLo)), refLen(heldDelta(c.wHi, c.vHi)))
	return refMul(refLen(twist), refAdd(eA, eB))
}

// floatTwistAreaProduct is the SAME product formed the way r3's own float
// norms form it. It exists only to show the fixtures above genuinely
// discriminate: a fixture on which this already encloses the exact product
// would prove nothing about the repair.
func floatTwistAreaProduct(c cellQuad) float64 {
	t := c.vLo.Sub(c.vHi).Sub(c.wLo).Add(c.wHi)
	eA := math.Max(c.vHi.Sub(c.vLo).Len(), c.wHi.Sub(c.wLo).Len())
	eB := math.Max(c.wLo.Sub(c.vLo).Len(), c.wHi.Sub(c.vHi).Len())
	return productUpper(t.Len(), absSumUpper(eA, eB))
}

// TestCellTwistAreaLinearArmEnclosesTheExactProduct pins the fallback arm on
// every review fixture.
func TestCellTwistAreaLinearArmEnclosesTheExactProduct(t *testing.T) {
	for _, c := range provenNormFixtures() {
		t.Run(c.name, func(t *testing.T) {
			corners := cellCornersOf(c.vLo, c.vHi, c.wLo, c.wHi)
			got := cellTwistAreaLinearFromSpans(corners.spans(), xtwistQuarterUpper(corners))
			want := refTwistAreaProduct(c)
			t.Logf("%s: published=%.17g exact=%s", c.name, got, want.Text('g', 25))
			require.GreaterOrEqual(t, refFloat(got).Cmp(want), 0,
				"the linear twist-area arm must enclose the exact |T|*(eA+eB)")

			if c.floatShortfallExpected {
				require.Less(t, refFloat(floatTwistAreaProduct(c)).Cmp(want), 0,
					"this fixture must discriminate: the float-norm route has to fall SHORT on it")
			}
			if want.Sign() == 0 {
				require.Equal(t, 0.0, got, "a planar cell charges nothing at all")
			}
		})
	}
}

// TestCellTwistAreaLinearArmEnclosesOrdinaryCells sweeps plain one-decimal-mm
// coordinates, the reachable-by-an-end-user range: no adversarial input is
// needed to drive the float-norm route below the exact product, so the swept
// enclosure has to hold everywhere rather than on the named fixtures alone.
func TestCellTwistAreaLinearArmEnclosesOrdinaryCells(t *testing.T) {
	rng := rand.New(rand.NewPCG(0x10f7c0de, 0x51de51de))
	coord := func() float64 { return math.Round(rng.Float64()*2000) / 10 }
	vec := func() r3.Vec { return r3.NewVec(coord(), coord(), coord()) }

	floatShort := 0
	const cases = 20000
	for range cases {
		c := cellQuad{vLo: vec(), vHi: vec(), wLo: vec(), wHi: vec()}
		want := refTwistAreaProduct(c)
		corners := cellCornersOf(c.vLo, c.vHi, c.wLo, c.wHi)
		got := cellTwistAreaLinearFromSpans(corners.spans(), xtwistQuarterUpper(corners))
		require.GreaterOrEqual(t, refFloat(got).Cmp(want), 0,
			"linear twist-area arm fell below the exact product for %+v", c)
		if refFloat(floatTwistAreaProduct(c)).Cmp(want) < 0 {
			floatShort++
		}
	}
	t.Logf("float-norm route fell short on %d of %d ordinary cells", floatShort, cases)
	require.Positive(t, floatShort,
		"the sweep must reach cells the float-norm route understates, or it proves nothing")
}

// refChordCurveAreaAllow states cellChordCurveAreaAllow's published value
// mathematically, at refPrec bits, from the cell's own EXACT corner
// differences: every norm is the true one and no term carries an outward
// nudge. Production composes the same terms through upRound/productUpper/
// absSumUpper, each of which only ever widens, and each term is monotone
// non-decreasing in every norm it reads — so the published value must sit at
// or above this reference, and a norm that reverts to r3.Vec.Len drops it
// below.
//
// nMin is read from production's own cellChordPatchNormalLower, which is
// already exact-rational: this reference falsifies the NORMS, not that helper.
func refChordCurveAreaAllow(c cellQuad, arcA, arcB, md, energyA, energyB float64) *big.Float {
	da, db := heldDelta(c.vHi, c.vLo), heldDelta(c.wHi, c.wLo)
	ca, cb := refLen(da), refLen(db)
	eB := refMax(refLen(heldDelta(c.wLo, c.vLo)), refLen(heldDelta(c.wHi, c.vHi)))
	cMax := refMax(ca, cb)
	mdF := refFloat(md)
	two := refFloat(2)

	dev := func(arcLen float64, chord *big.Float, energy float64) (*big.Float, *big.Float) {
		span := refAdd(refFloat(arcLen), chord)
		i, j := span, refMul(span, span)
		if !isNonFinite(energy) && energy >= 0 {
			e := refFloat(energy)
			i = refMin(i, new(big.Float).SetPrec(refPrec).Sqrt(e))
			j = refMin(j, e)
		}
		return i, j
	}
	ia, ja := dev(arcA, ca, energyA)
	ib, jb := dev(arcB, cb, energyB)
	iMax := refMax(ia, ib)
	beta := refAdd(eB, refMul(two, mdF))
	gamma := refMul(refMul(two, mdF), cMax)

	free := refAdd(refQuo(refMul(beta, refAdd(ia, ib)), two), gamma)
	nMin := cellChordPatchNormalLower(c.vLo, c.vHi, c.wLo, c.wHi)
	if nMin <= 0 {
		return free
	}
	nMinF := refFloat(nMin)
	twist := rvSub(heldDelta(c.vLo, c.vHi), heldDelta(c.wLo, c.wHi))
	pCrossT := refMax(refLen(rvCross(da, twist)), refLen(rvCross(db, twist)))
	oscW := refAdd(refLen(twist), refQuo(refMul(eB, pCrossT), nMinF))
	lin := refAdd(refMul(oscW, iMax), refMul(refMul(two, mdF), refAdd(cMax, iMax)))
	quad := refQuo(
		refAdd(refMul(refMul(beta, beta), refAdd(ja, jb)), refMul(two, refMul(gamma, gamma))),
		refMul(two, nMinF),
	)
	return refMin(free, refAdd(lin, quad))
}

// TestCellChordCurveAreaAllowEnclosesItsExactTerms holds the ruled leg to the
// same proven-norm rule as its twist sibling. The cancelling-twist fixture is
// the one that discriminates: there the oscillation term reads |T| directly,
// and a float |T| is short by a fraction of the term rather than an ulp of it.
func TestCellChordCurveAreaAllowEnclosesItsExactTerms(t *testing.T) {
	type row struct {
		name             string
		cell             cellQuad
		md               float64
		energyA, energyB float64
	}
	fixtures := provenNormFixtures()
	rows := []row{
		{"large-integer corners", fixtures[0], 1e3, math.Inf(1), math.Inf(1)},
		{"one-decimal millimetres", fixtures[1], 0.5, math.Inf(1), math.Inf(1)},
		{"cancelling twist, matched delta carried", fixtures[2], 2.5, 1e4, 1e4},
		{"cancelling twist, premise-free", fixtures[2], 2.5, math.Inf(1), math.Inf(1)},
		// The discriminating row. A matched delta of zero drops the term that
		// otherwise swamps everything, and an energy this small keeps the
		// quadratic term under the oscillation term, so the published value is
		// the oscillation term and reads |T| almost alone. The float |T| on
		// this cell is 0.888 of the exact one, so a reverted norm lands about
		// eleven percent below the reference.
		{"cancelling twist, oscillation carried", fixtures[2], 0, 1e-26, 1e-26},
		{"planar cell", fixtures[3], 0.25, math.Inf(1), math.Inf(1)},
	}
	for _, r := range rows {
		t.Run(r.name, func(t *testing.T) {
			c := r.cell
			// Both arc-length claims are proven upper bounds: an outward
			// chord length is itself one for a straight side, and the sweep
			// factor keeps every fixture's claim above the chord it subtends.
			arcA := upRound(1.25 * rvLenUpper(heldDelta(c.vHi, c.vLo)))
			arcB := upRound(1.25 * rvLenUpper(heldDelta(c.wHi, c.wLo)))
			got := cellChordCurveAreaAllow(c.vLo, c.vHi, c.wLo, c.wHi, arcA, arcB, r.md, r.energyA, r.energyB)
			want := refChordCurveAreaAllow(c, arcA, arcB, r.md, r.energyA, r.energyB)
			t.Logf("%s: published=%.17g exact=%s nMin=%.6e",
				r.name, got, want.Text('g', 25), cellChordPatchNormalLower(c.vLo, c.vHi, c.wLo, c.wHi))
			require.GreaterOrEqual(t, refFloat(got).Cmp(want), 0,
				"the published ruled leg must enclose its own exactly-stated terms")
		})
	}
}

// TestCellChordCurveAreaAllowAdmitsAnExactlyTightArcClaim pins the gate the
// outward chord would otherwise break: the arc-length claim is compared to the
// chord by EXACT rational comparison, so a caller whose claim equals the true
// chord is admitted rather than refused for the ulp the outward rounding adds.
func TestCellChordCurveAreaAllowAdmitsAnExactlyTightArcClaim(t *testing.T) {
	// A 3-4-5 side: the chord length is exactly 5, exactly representable, so
	// an arcLenUpper of 5 is exactly tight rather than short.
	vLo, vHi := r3.NewVec(0, 0, 0), r3.NewVec(3, 4, 0)
	wLo, wHi := r3.NewVec(0, 0, 12), r3.NewVec(3, 4, 12)
	require.Equal(t, 5.0, rvLenUpper(heldDelta(vHi, vLo)), "the fixture's chord must be exactly representable")
	require.Equal(t, 0.0, cellChordCurveAreaAllow(vLo, vHi, wLo, wHi, 5, 5, 0, 0, 0),
		"an exactly-tight arc claim on a straight, untwisted cell is admitted and charges nothing")
	require.True(t, math.IsInf(cellChordCurveAreaAllow(vLo, vHi, wLo, wHi, 4.999, 5, 0, 0, 0), 1),
		"an arc claim genuinely below its own chord is still refused")
}

// TestRvLenUpperEnclosesTheExactNorm is the primitive's own falsifier, and the
// direct statement of why r3.Vec.Len cannot stand in for it.
func TestRvLenUpperEnclosesTheExactNorm(t *testing.T) {
	cases := map[string][2]r3.Vec{
		"large integers":   {r3.NewVec(770749, 887007, 339646), r3.NewVec(453885, 39861, 565228)},
		"one-decimal mm":   {r3.NewVec(16.6, 65.1, 173.7), r3.NewVec(102.7, 169.5, 76.2)},
		"near cancelling":  {r3.NewVec(1, 1e-9, 0), r3.NewVec(1, 0, 1e-9)},
		"exactly zero":     {r3.NewVec(2, 3, 4), r3.NewVec(2, 3, 4)},
		"exact 3-4-5 side": {r3.NewVec(0, 0, 0), r3.NewVec(3, 4, 0)},
	}
	for name, pair := range cases {
		t.Run(name, func(t *testing.T) {
			d := heldDelta(pair[1], pair[0])
			got, want := rvLenUpper(d), refLen(d)
			require.GreaterOrEqual(t, refFloat(got).Cmp(want), 0, "rvLenUpper must enclose the exact norm")
			if want.Sign() == 0 {
				require.Equal(t, 0.0, got, "a zero difference has length exactly zero")
			}
		})
	}
}

// TestRatLenAtLeastDecidesExactly pins the gate primitive in both directions:
// it must admit a claim that is exactly tight and refuse one that is short by
// a single ulp, neither decided by a rounded norm.
func TestRatLenAtLeastDecidesExactly(t *testing.T) {
	d := heldDelta(r3.NewVec(3, 4, 0), r3.NewVec(0, 0, 0))
	require.True(t, ratLenAtLeast(5, d), "a claim equal to the exact norm is admitted")
	require.True(t, ratLenAtLeast(math.Nextafter(5, math.Inf(1)), d), "a claim above the exact norm is admitted")
	require.False(t, ratLenAtLeast(math.Nextafter(5, 0), d), "a claim one ulp short is refused")
	require.False(t, ratLenAtLeast(-1, d), "a negative claim is refused")
	require.False(t, ratLenAtLeast(math.NaN(), d), "a NaN claim is refused")
}
