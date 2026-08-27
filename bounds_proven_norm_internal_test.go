package decad

import (
	"math"
	"math/big"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

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
// corners — the quantity cellTwistAreaAllow's derivation publishes, stated
// independently of the code that publishes it.
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

// TestCellTwistAreaAllowEnclosesTheExactProduct is the assigned finding's own
// falsifier: the published twist leg must never fall below the exact
// |T|·(eA+eB) its derivation states, on any of the reviewer's corners.
func TestCellTwistAreaAllowEnclosesTheExactProduct(t *testing.T) {
	for _, c := range provenNormFixtures() {
		t.Run(c.name, func(t *testing.T) {
			got := cellTwistAreaAllow(c.vLo, c.vHi, c.wLo, c.wHi)
			want := refTwistAreaProduct(c)
			t.Logf("%s: published=%.17g exact=%s", c.name, got, want.Text('g', 25))
			require.GreaterOrEqual(t, refFloat(got).Cmp(want), 0,
				"the published twist leg must enclose the exact |T|*(eA+eB)")

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

// TestCellTwistAreaAllowEnclosesOrdinaryCells sweeps plain one-decimal-mm
// coordinates, the reachable-by-an-end-user range: no adversarial input is
// needed to drive the float-norm route below the exact product, so the swept
// enclosure has to hold everywhere rather than on the named fixtures alone.
func TestCellTwistAreaAllowEnclosesOrdinaryCells(t *testing.T) {
	rng := rand.New(rand.NewPCG(0x10f7c0de, 0x51de51de))
	coord := func() float64 { return math.Round(rng.Float64()*2000) / 10 }
	vec := func() r3.Vec { return r3.NewVec(coord(), coord(), coord()) }

	floatShort := 0
	const cases = 20000
	for range cases {
		c := cellQuad{vLo: vec(), vHi: vec(), wLo: vec(), wHi: vec()}
		want := refTwistAreaProduct(c)
		require.GreaterOrEqual(t, refFloat(cellTwistAreaAllow(c.vLo, c.vHi, c.wLo, c.wHi)).Cmp(want), 0,
			"published twist leg fell below the exact product for %+v", c)
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
