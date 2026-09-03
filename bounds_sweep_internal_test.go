package decad

import (
	"fmt"
	"math"
	"math/big"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestCellChordCurveAreaUpperEnclosesTheFlatTriangleCounterexample(t *testing.T) {
	vLo := r3.NewVec(0, 0, 0)
	vHi := r3.NewVec(1, 0, 0)
	for _, h := range []float64{0.1, 0.01, 0.001} {
		t.Run(fmt.Sprintf("h=%g", h), func(t *testing.T) {
			wLo := r3.NewVec(0, 1, h)
			wHi := r3.NewVec(0, 0, h)

			// Both sides are straight LineSegs (sectionDelta=0): each
			// side's own arc length equals its own chord length exactly,
			// claimed through cellSpanUpper because r3.Vec.Len of that same
			// chord is not itself a proven upper bound on it.
			arcLenA := cellSpanUpper(vHi, vLo)
			arcLenB := cellSpanUpper(wHi, wLo)

			patchArea := bilinearPatchAreaNumeric(vLo, vHi, wLo, wHi)
			heldTriangleArea := h / 2 // the two flat triangles' own combined area, degenerating toward 0 with h

			allow := cellChordCurveAreaUpper(vLo, vHi, wLo, wHi, arcLenA, arcLenB, 0)
			require.GreaterOrEqualf(t, allow, patchArea,
				"h=%g: cellChordCurveAreaUpper=%.6g must enclose the bilinear patch's own area %.6g, "+
					"even though the held triangle pair carries only %.6g",
				h, allow, patchArea, heldTriangleArea)
			t.Logf("h=%g: allow=%.6g patchArea=%.6g (ratio %.4g), heldTriangleArea=%.6g",
				h, allow, patchArea, allow/patchArea, heldTriangleArea)
		})
	}
}

// TestCellChordCurveAreaUpperEnclosesTheCrossedCellCounterexample pins F2's
// own counterexample: a CROSSED quad (the bottom and top sides run in
// perpendicular directions, each straddling the axis, so the ruled patch
// self-intersects rather than describing a simple twisted ribbon) — the
// shape an earlier cellRuledExcessUpper's subtraction step failed on even
// after its own 2x looseness was corrected.
func TestCellChordCurveAreaUpperEnclosesTheCrossedCellCounterexample(t *testing.T) {
	const h = 0.01
	vLo := r3.NewVec(-1, 0, 0)
	vHi := r3.NewVec(1, 0, 0)
	wLo := r3.NewVec(0, -1, h)
	wHi := r3.NewVec(0, 1, h)

	arcLenA := cellSpanUpper(vHi, vLo)
	arcLenB := cellSpanUpper(wHi, wLo)

	patchArea := bilinearPatchAreaNumeric(vLo, vHi, wLo, wHi)
	allow := cellChordCurveAreaUpper(vLo, vHi, wLo, wHi, arcLenA, arcLenB, 0)
	require.GreaterOrEqualf(t, allow, patchArea,
		"cellChordCurveAreaUpper=%.6g must enclose the crossed cell's own bilinear patch area %.6g",
		allow, patchArea)
	t.Logf("allow=%.6g patchArea=%.6g (ratio %.4g)", allow, patchArea, allow/patchArea)
}

// TestCellChordCurveAreaUpperReducesToTheTwistBoundAtZeroSectionDelta pins
// the cross-check the derivation's own doc comment claims: at
// sectionDelta=0, cellChordCurveAreaUpper's own eA (arc length, which
// equals chord length for a straight LineSeg pairing) and eB (corner
// separation) are EXACTLY the same eA, eB cellTwistVolumeAllow's own already
// -proven derivation uses for the SAME bilinear patch, so the two speak for
// the same eA*eB product on the same four corners. The check is against that
// product itself, not against cellTwistVolumeAllow's own float: both helpers
// certify their norms over the rationals (cellSpanUpper), so the two answers
// agree on the quantity while each rounds outward on its own.
func TestCellChordCurveAreaUpperReducesToTheTwistBoundAtZeroSectionDelta(t *testing.T) {
	vLo := r3.NewVec(0, 0, 0)
	vHi := r3.NewVec(1, 0, 0)
	wLo := r3.NewVec(0, 0, 1)
	wHi := r3.NewVec(0, 1, 1)

	arcLenA := cellSpanUpper(vHi, vLo)
	arcLenB := cellSpanUpper(wHi, wLo)
	eA := math.Max(arcLenA, arcLenB)
	eB := math.Max(cellSpanUpper(wLo, vLo), cellSpanUpper(wHi, vHi))
	want := eA * eB

	// Not an exact match: cellChordCurveAreaUpper's own eB folds a
	// productUpper(2, sectionDelta) term (0 at sectionDelta=0, but still an
	// upRound-outward-rounded absSumUpper step over eBBase) beside eBBase, so
	// the published answer can sit a representable float above this test's
	// own un-rounded eA*eB by construction — a platform-independent, one-ulp
	// -scale outward nudge, never a mismatch in the underlying quantity
	// (HOST PORTABILITY: never pin a bound to a literal this machine
	// measured).
	got := cellChordCurveAreaUpper(vLo, vHi, wLo, wHi, arcLenA, arcLenB, 0)
	require.InDelta(t, want, got, 1e-9)
	require.GreaterOrEqual(t, got, want, "the answer must round outward, never inward")
}

// TestCellChordCurveAreaUpperRefusesOnBrokenClaims pins the reject-only
// gates the derivation's own premises depend on: a non-finite operand, a
// negative arc length or sectionDelta claim, and an arc length claim
// SMALLER than its own chord (a chord can never exceed the arc it
// subtends) must all answer +Inf, never a finite number computed past a
// falsified premise.
func TestCellChordCurveAreaUpperRefusesOnBrokenClaims(t *testing.T) {
	vLo := r3.NewVec(0, 0, 0)
	vHi := r3.NewVec(1, 0, 0)
	wLo := r3.NewVec(0, 0, 1)
	wHi := r3.NewVec(0, 1, 1)
	chordA := cellSpanUpper(vHi, vLo)
	chordB := cellSpanUpper(wHi, wLo)

	require.True(t, math.IsInf(cellChordCurveAreaUpper(vLo, vHi, wLo, wHi, math.NaN(), chordB, 0), 1), "NaN arcLenA")
	require.True(t, math.IsInf(cellChordCurveAreaUpper(vLo, vHi, wLo, wHi, chordA, math.NaN(), 0), 1), "NaN arcLenB")
	require.True(t, math.IsInf(cellChordCurveAreaUpper(vLo, vHi, wLo, wHi, chordA, chordB, math.NaN()), 1), "NaN sectionDelta")
	require.True(t, math.IsInf(cellChordCurveAreaUpper(vLo, vHi, wLo, wHi, -1, chordB, 0), 1), "negative arcLenA")
	require.True(t, math.IsInf(cellChordCurveAreaUpper(vLo, vHi, wLo, wHi, chordA, -1, 0), 1), "negative arcLenB")
	require.True(t, math.IsInf(cellChordCurveAreaUpper(vLo, vHi, wLo, wHi, chordA, chordB, -1), 1), "negative sectionDelta")
	require.True(t, math.IsInf(cellChordCurveAreaUpper(vLo, vHi, wLo, wHi, chordA/2, chordB, 0), 1), "arcLenA below its own chord")
	require.True(t, math.IsInf(cellChordCurveAreaUpper(vLo, vHi, wLo, wHi, chordA, chordB/2, 0), 1), "arcLenB below its own chord")
	// +Inf arcLenA/arcLenB are legitimate (an unbounded caller claim), and
	// the answer must stay a genuine +Inf refusal rather than becoming NaN
	// through arithmetic.
	require.True(t, math.IsInf(cellChordCurveAreaUpper(vLo, vHi, wLo, wHi, math.Inf(1), chordB, 0), 1), "+Inf arcLenA")
}

// TestCellChordCurveAreaUpperRefusesNonFiniteCorners pins F5: an earlier
// version validated only the three scalar operands, so a NaN corner sailed
// straight through the range gate `arcLenUpperA < cellSpanUpper(vHi, vLo)`
// (NaN compares false against everything, so the gate never refuses) and
// propagated through eBBase's own math.Max into a silently-NaN answer for a
// cell whose own geometry is unstateable, rather than a refusing +Inf — an
// unchecked caller that widens its own bound by `answer > 0` treats NaN
// exactly like 0, since `NaN > 0` is false either way. Every one of the four corners, in
// isolation, must now trigger a genuine +Inf refusal.
func TestCellChordCurveAreaUpperRefusesNonFiniteCorners(t *testing.T) {
	vLo := r3.NewVec(0, 0, 0)
	vHi := r3.NewVec(1, 0, 0)
	wLo := r3.NewVec(0, 0, 1)
	wHi := r3.NewVec(0, 1, 1)
	nan := r3.NewVec(math.NaN(), 0, 0)
	chordA := cellSpanUpper(vHi, vLo)
	chordB := cellSpanUpper(wHi, wLo)

	require.True(t, math.IsInf(cellChordCurveAreaUpper(nan, vHi, wLo, wHi, chordA, chordB, 0), 1), "NaN vLo")
	require.True(t, math.IsInf(cellChordCurveAreaUpper(vLo, nan, wLo, wHi, chordA, chordB, 0), 1), "NaN vHi")
	require.True(t, math.IsInf(cellChordCurveAreaUpper(vLo, vHi, nan, wHi, chordA, chordB, 0), 1), "NaN wLo")
	require.True(t, math.IsInf(cellChordCurveAreaUpper(vLo, vHi, wLo, nan, chordA, chordB, 0), 1), "NaN wHi")
}

// ratSpanUpper and ratTwistQuarterUpper are the DIRECT big.Rat readings of
// |a−b| and |T|/4 — one exact rational operation per step, the most literal
// spelling of the two quantities there is. cellSpanUpper and
// cellTwistQuarterUpper carry the same exact values through the homogeneous
// INTEGER kernel instead, which normalises once at the end rather than at
// every step, and the two tests below pin that this is a change of
// representation only. They exist as REFERENCES for those tests and for
// nothing else; no bound reads them.
func ratSpanUpper(a, b r3.Vec) float64 {
	d := rvSub(ratVec(a), ratVec(b))
	return ratSqrtUp(rvDot(d, d))
}

func ratTwistQuarterUpper(vLo, vHi, wLo, wHi r3.Vec) float64 {
	t := rvSub(rvSub(ratVec(vLo), ratVec(vHi)), rvSub(ratVec(wLo), ratVec(wHi)))
	if rvIsZero(t) {
		return 0
	}
	return ratSqrtUp(new(big.Rat).Quo(rvDot(t, t), new(big.Rat).SetInt64(16)))
}

// TestCellExactReadingsMatchTheRationalReference pins that carrying the two
// certified cell quantities through the homogeneous integer kernel publishes
// EXACTLY what the direct big.Rat chain publishes — bit for bit, never within
// a tolerance. Nothing here is an approximation to be checked for closeness:
// both chains are exact, both feed ratSqrtUp, and a big.Rat is canonical, so
// equal VALUES are the same big.Rat and ratSqrtUp cannot tell which chain
// built it. A difference of any size would mean one of the chains is not
// carrying the value it claims.
//
// The table carries the cells where a representation change could most
// plausibly show: corners whose raw norms sit below their exact spans, a
// twist chain that cancels to exactly (0,0,0) in float, coordinates spread
// across the exponent range (a subnormal against a large one, where the two
// operands' own denominators differ by hundreds of bits), and exact zeros.
func TestCellExactReadingsMatchTheRationalReference(t *testing.T) {
	vecs := []r3.Vec{
		r3.NewVec(0, 0, 0),
		r3.NewVec(1, 0, 0),
		r3.NewVec(-0.18407194718000197, -0.9670493813481006, 0.11756527611707068),
		r3.NewVec(-0.2543362139571195, 0.17121027904800257, 0.6486272299368296),
		r3.NewVec(-0.329681753490781, 0.20068938566549477, 0.2784442506567102),
		r3.NewVec(-0.7983173553857771, 0.456370350439145, 0.09559367959010823),
		r3.NewVec(0.1, 0.2, 0.3),
		r3.NewVec(0.7, 1.1, 1.3),
		r3.NewVec(0.1, 0.2, 0.3).Add(r3.NewVec(0.7, 1.1, 1.3)),
		r3.NewVec(1e-300, 5e300, -3.5),
		r3.NewVec(math.SmallestNonzeroFloat64, math.MaxFloat64/4, 0),
		r3.NewVec(1e16, 1e-16, 1),
	}

	t.Run("spans", func(t *testing.T) {
		for i, a := range vecs {
			for j, b := range vecs {
				require.Equalf(t, ratSpanUpper(a, b), cellSpanUpper(a, b), "vecs[%d] to vecs[%d]", i, j)
			}
		}
		rng := rand.New(rand.NewPCG(41, 43))
		for range 500 {
			a := r3.NewVec(rng.Float64()*200-100, rng.Float64()*200-100, rng.Float64()*200-100)
			b := r3.NewVec(rng.Float64()*200-100, rng.Float64()*200-100, rng.Float64()*200-100)
			require.Equal(t, ratSpanUpper(a, b), cellSpanUpper(a, b))
		}
	})

	t.Run("twist quarter", func(t *testing.T) {
		for i := range vecs {
			vLo, vHi := vecs[i], vecs[(i+1)%len(vecs)]
			wLo, wHi := vecs[(i+2)%len(vecs)], vecs[(i+3)%len(vecs)]
			require.Equalf(t, ratTwistQuarterUpper(vLo, vHi, wLo, wHi), cellTwistQuarterUpper(vLo, vHi, wLo, wHi), "cell at vecs[%d]", i)
		}
		// The cancelling chain: wHi is BUILT as vHi+wLo, so the float chain
		// reads exactly (0,0,0) while the exact T is the addition's own
		// rounding residue.
		vHi, wLo := r3.NewVec(0.1, 0.2, 0.3), r3.NewVec(0.7, 1.1, 1.3)
		require.Equal(t,
			ratTwistQuarterUpper(r3.NewVec(0, 0, 0), vHi, wLo, vHi.Add(wLo)),
			cellTwistQuarterUpper(r3.NewVec(0, 0, 0), vHi, wLo, vHi.Add(wLo)))

		rng := rand.New(rand.NewPCG(47, 53))
		for range 500 {
			corner := func() r3.Vec {
				return r3.NewVec(rng.Float64()*200-100, rng.Float64()*200-100, rng.Float64()*200-100)
			}
			vLo, vHi, wLo, wHi := corner(), corner(), corner(), corner()
			require.Equal(t, ratTwistQuarterUpper(vLo, vHi, wLo, wHi), cellTwistQuarterUpper(vLo, vHi, wLo, wHi))
		}
	})
}

// TestCellAllowsOfMatchesThePerBoundHelpers pins cellAllowsOf's whole
// contract. That reader exists only to certify one cell's four spans and its
// |T|/4 endpoint ONCE for all three of the cell's bounds instead of once per
// bound, so each of its three fields must equal — BIT FOR BIT, never within a
// tolerance — what the corresponding helper publishes for the same cell. A
// difference of any size would mean the two entry points disagree about a
// published bound, which is what makes the sharing a change of WHICH
// computations happen rather than of what any of them returns.
//
// The table carries the cells where the two could most plausibly diverge:
// the raw-norm fixture whose corner separations read BELOW their exact norms
// under r3.Vec.Len (so the certified endpoint is what decides both the
// premise gate and eBBase); a cell whose twist chain CANCELS to exactly
// (0,0,0) in float while its exact T is nonzero (the case
// cellTwistQuarterUpper exists for); a wholly degenerate cell; a cell whose
// SCALAR claims are broken, where only the area reading may refuse; and a
// cell with a non-finite corner, where all three must. The randomized sweep
// then covers ordinary cells, with arc-length claims deliberately straddling
// their own chords so both the admitting and the refusing side of the premise
// gate are exercised.
func TestCellAllowsOfMatchesThePerBoundHelpers(t *testing.T) {
	check := func(t *testing.T, vLo, vHi, wLo, wHi r3.Vec, arcLenA, arcLenB, matchedDelta float64) {
		t.Helper()
		got := cellAllowsOf(vLo, vHi, wLo, wHi, arcLenA, arcLenB, matchedDelta)
		require.Equal(t, cellChordCurveAreaUpper(vLo, vHi, wLo, wHi, arcLenA, arcLenB, matchedDelta), got.chordCurveAreaUpper, "chordCurveAreaUpper")
		require.Equal(t, cellTwistVolumeAllow(vLo, vHi, wLo, wHi), got.twistVolumeAllow, "twistVolumeAllow")
		require.Equal(t, cellTwistOffsetUpper(vLo, vHi, wLo, wHi), got.twistOffsetUpper, "twistOffsetUpper")
	}

	// The float chain vLo−vHi−wLo+wHi cancels to exactly (0,0,0) here, since
	// wHi is BUILT as vHi+wLo, while the exact T is the addition's own
	// rounding residue, about (-2.78e-17, -5.55e-17, 5.55e-17).
	cancelVHi := r3.NewVec(0.1, 0.2, 0.3)
	cancelWLo := r3.NewVec(0.7, 1.1, 1.3)

	for _, tc := range []struct {
		name                         string
		vLo, vHi, wLo, wHi           r3.Vec
		arcLenA, arcLenB, matchedDel float64
	}{
		{
			name: "raw norms below the exact spans",
			vLo:  r3.NewVec(-0.18407194718000197, -0.9670493813481006, 0.11756527611707068),
			vHi:  r3.NewVec(-0.2543362139571195, 0.17121027904800257, 0.6486272299368296),
			wLo:  r3.NewVec(-0.329681753490781, 0.20068938566549477, 0.2784442506567102),
			wHi:  r3.NewVec(-0.7983173553857771, 0.456370350439145, 0.09559367959010823),
			// Dominates both sides' own certified chords, so the premise gate
			// admits and eA is exactly this claim.
			arcLenA: 1.5, arcLenB: 1.5, matchedDel: 0,
		},
		{
			name: "cancelling twist chain",
			vLo:  r3.NewVec(0, 0, 0), vHi: cancelVHi, wLo: cancelWLo, wHi: cancelVHi.Add(cancelWLo),
			arcLenA: 1, arcLenB: 1, matchedDel: 0.01,
		},
		{
			name: "wholly degenerate cell",
			vLo:  r3.NewVec(1, 2, 3), vHi: r3.NewVec(1, 2, 3), wLo: r3.NewVec(1, 2, 3), wHi: r3.NewVec(1, 2, 3),
			arcLenA: 0, arcLenB: 0, matchedDel: 0,
		},
		{
			name: "arc-length claim below its own chord",
			vLo:  r3.NewVec(0, 0, 0), vHi: r3.NewVec(1, 0, 0), wLo: r3.NewVec(0, 0, 1), wHi: r3.NewVec(0, 1, 1),
			arcLenA: 0.25, arcLenB: 2, matchedDel: 0,
		},
		{
			name: "broken scalar claim",
			vLo:  r3.NewVec(0, 0, 0), vHi: r3.NewVec(1, 0, 0), wLo: r3.NewVec(0, 0, 1), wHi: r3.NewVec(0, 1, 1),
			arcLenA: 2, arcLenB: 2, matchedDel: -1,
		},
		{
			name: "non-finite scalar claim",
			vLo:  r3.NewVec(0, 0, 0), vHi: r3.NewVec(1, 0, 0), wLo: r3.NewVec(0, 0, 1), wHi: r3.NewVec(0, 1, 1),
			arcLenA: math.Inf(1), arcLenB: 2, matchedDel: 0,
		},
		{
			name: "non-finite corner",
			vLo:  r3.NewVec(0, 0, 0), vHi: r3.NewVec(math.NaN(), 0, 0), wLo: r3.NewVec(0, 0, 1), wHi: r3.NewVec(0, 1, 1),
			arcLenA: 2, arcLenB: 2, matchedDel: 0,
		},
	} {
		t.Run(tc.name, func(t *testing.T) {
			check(t, tc.vLo, tc.vHi, tc.wLo, tc.wHi, tc.arcLenA, tc.arcLenB, tc.matchedDel)
		})
	}

	t.Run("randomized cells", func(t *testing.T) {
		rng := rand.New(rand.NewPCG(31, 37))
		coord := func() float64 { return rng.Float64()*4 - 2 }
		for range 300 {
			vLo := r3.NewVec(coord(), coord(), coord())
			vHi := r3.NewVec(coord(), coord(), coord())
			wLo := r3.NewVec(coord(), coord(), coord())
			wHi := r3.NewVec(coord(), coord(), coord())
			// Straddles each side's own certified chord, so roughly half the
			// rows are admitted by the premise gate and the rest refused.
			arcLenA := cellSpanUpper(vHi, vLo) * (0.5 + rng.Float64())
			arcLenB := cellSpanUpper(wHi, wLo) * (0.5 + rng.Float64())
			check(t, vLo, vHi, wLo, wHi, arcLenA, arcLenB, rng.Float64()*0.1)
		}
	})
}

// rawNormIsBelowExact reports whether r3.Vec.Len of a−b lands STRICTLY below
// the exact |a−b|, decided by exact rational comparison of the raw reading's
// own square against the exactly-rational squared norm (both operands are
// float64 corners, hence exact rationals, so the lift introduces no rounding
// of its own). It is what the two raw-norm regressions below use to prove
// their fixture actually exercises the channel they pin, rather than
// asserting it from a measured literal.
func rawNormIsBelowExact(a, b r3.Vec) bool {
	raw := ratOfFloat(a.Sub(b).Len())
	d := rvSub(ratVec(a), ratVec(b))
	return new(big.Rat).Mul(raw, raw).Cmp(rvDot(d, d)) < 0
}

// TestCellChordCurveAreaUpperEnclosesTheExactEdgeProduct pins the RAW-NORM
// channel on this helper's own eB term, the same channel
// TestCellTwistBoundsEncloseTheirExactTerms pins one helper over: r3.Vec.Len
// is nested math.Hypot, which carries no accuracy contract and sits several
// ulp BELOW the exact norm for a large share of vectors, and neither
// absSumUpper's nor productUpper's one-ulp outward nudge can recover a
// multi-ulp shortfall. At matchedDeltaUpper=0 the eB term is eBBase alone, so
// the published answer's own obligation is exactly eA·max(|wLo−vLo|,|wHi−vHi|)
// and an understated eBBase puts the answer BELOW it — the unsound direction,
// since chordedBoundaryVolumeAllow sums this reading over every wall cell for
// its wallAreaUpper. The comparison is over exact rationals in SQUARED form
// (the norms are irrational, their squares exactly rational), never against a
// float reference that shares the defect.
func TestCellChordCurveAreaUpperEnclosesTheExactEdgeProduct(t *testing.T) {
	vLo := r3.NewVec(-0.18407194718000197, -0.9670493813481006, 0.11756527611707068)
	vHi := r3.NewVec(-0.2543362139571195, 0.17121027904800257, 0.6486272299368296)
	wLo := r3.NewVec(-0.329681753490781, 0.20068938566549477, 0.2784442506567102)
	wHi := r3.NewVec(-0.7983173553857771, 0.456370350439145, 0.09559367959010823)

	// eBBase maximises over these two corner separations, and BOTH of them
	// read low under a raw norm, so no choice of maximum escapes the channel.
	require.True(t, rawNormIsBelowExact(wLo, vLo), "the fixture must exercise the raw-norm channel on |wLo−vLo|")
	require.True(t, rawNormIsBelowExact(wHi, vHi), "the fixture must exercise the raw-norm channel on |wHi−vHi|")

	// An exactly-representable arc-length claim above either side's own
	// certified chord, so the premise gate admits and eA is exactly this
	// value with no rounding of its own.
	const arcLenUpper = 1.5
	require.GreaterOrEqual(t, arcLenUpper, cellSpanUpper(vHi, vLo), "the fixture's arc-length claim must dominate side A's chord")
	require.GreaterOrEqual(t, arcLenUpper, cellSpanUpper(wHi, wLo), "the fixture's arc-length claim must dominate side B's chord")

	got := cellChordCurveAreaUpper(vLo, vHi, wLo, wHi, arcLenUpper, arcLenUpper, 0)
	require.False(t, math.IsInf(got, 1), "the fixture must be admitted, not refused")

	// exactCellTwistFactors' own second return IS eBBase's definition squared,
	// max(|wLo−vLo|², |wHi−vHi|²), taken from the float corners with no
	// rounding. got >= eA·eBBase holds iff got² >= eA²·eBBase².
	_, eB2 := exactCellTwistFactors(vLo, vHi, wLo, wHi)
	lhs := new(big.Rat).Mul(ratOfFloat(got), ratOfFloat(got))
	rhs := new(big.Rat).Mul(new(big.Rat).Mul(ratOfFloat(arcLenUpper), ratOfFloat(arcLenUpper)), eB2)
	require.GreaterOrEqual(t, lhs.Cmp(rhs), 0,
		"cellChordCurveAreaUpper = %.20g sits BELOW the exact eA·eBBase it claims to dominate", got)
}

// TestCellChordCurveAreaUpperRefusesARawNormArcLengthClaim pins the same
// raw-norm channel on the PREMISE gate rather than on the published product.
// The derivation's first bullet needs arcLenUpper to dominate its own side's
// chord, and a raw norm on the REFUSING side of that comparison rounds DOWN:
// it admits an arc-length claim that exact rational arithmetic proves sits
// below the true chord, and the helper then publishes a finite bound resting
// on a falsified premise. The certified endpoint can only over-refuse by an
// ulp, the reject-only-safe direction — pinned here by the certified pair
// still being admitted, so the gate is not simply refusing everything.
func TestCellChordCurveAreaUpperRefusesARawNormArcLengthClaim(t *testing.T) {
	vLo := r3.NewVec(-0.18407194718000197, -0.9670493813481006, 0.11756527611707068)
	vHi := r3.NewVec(-0.2543362139571195, 0.17121027904800257, 0.6486272299368296)
	wLo := r3.NewVec(-0.329681753490781, 0.20068938566549477, 0.2784442506567102)
	wHi := r3.NewVec(-0.7983173553857771, 0.456370350439145, 0.09559367959010823)

	require.True(t, rawNormIsBelowExact(vHi, vLo), "side A's raw norm must be provably below its own chord")
	require.True(t, rawNormIsBelowExact(wHi, wLo), "side B's raw norm must be provably below its own chord")

	rawA, rawB := vHi.Sub(vLo).Len(), wHi.Sub(wLo).Len()
	certA, certB := cellSpanUpper(vHi, vLo), cellSpanUpper(wHi, wLo)

	require.True(t, math.IsInf(cellChordCurveAreaUpper(vLo, vHi, wLo, wHi, rawA, certB, 0), 1),
		"a raw-norm arcLenUpperA is provably below side A's own chord and must be refused")
	require.True(t, math.IsInf(cellChordCurveAreaUpper(vLo, vHi, wLo, wHi, certA, rawB, 0), 1),
		"a raw-norm arcLenUpperB is provably below side B's own chord and must be refused")
	require.False(t, math.IsInf(cellChordCurveAreaUpper(vLo, vHi, wLo, wHi, certA, certB, 0), 1),
		"the certified chord endpoints are a sound claim and must still be admitted")
}

// TestCellChordCurveAreaUpperRefusesTheSagittaZigzag pins F1's own
// counterexample: cellChordCurveAreaUpper's matchedDeltaUpper obligation is
// a PARAMETER-MATCHED bound, never the loft evaluator's SET-distance sagitta
// (loftPayload.sectionDelta). Side A is straight (vLo=(0,0,0), vHi=(1,0,0),
// chord length 1). Side B's CHORD is also straight (wLo=(0,0,0.001),
// wHi=(1,0,0.001)), but the TRUE curve it chords is a 400-tooth zigzag of
// amplitude 0.001 packed into x in [0, 0.02] and straight for the rest —
// hugging its own chord within a sagitta of 0.001 (bounding BOTH sides'
// sagittas exactly) while its own arc length is 2.578, far more than its
// chord's 1.
//
// A caller who (wrongly, per the old broken contract this fixes) read the
// sagitta 0.001 as if it were matchedDeltaUpper would have published
// eA*eB=0.007734 against a true ruled-surface area of 0.2365 — a 30x
// violation, because max_s|b(s)-a(s)| under the constant-arc-length
// parametrization the homotopy actually uses is 0.5999 (200x the sagitta):
// packing almost all of side B's arc length into x in [0,0.02] decouples the
// zigzag's own arc-length-matched position from its chord position by
// nearly the full chord length, something a SET-distance sagitta says
// nothing about (cellChordCurveAreaUpper's own doc comment).
//
// No caller can derive a parameter-matched bound for this curve today (only
// a LINE or an ARC can, per that doc comment), so the only HONEST value to
// pass is +Inf — which is exactly what stops the 30x violation from ever
// reaching the derivation: refusal, not a shrunken bound.
func TestCellChordCurveAreaUpperRefusesTheSagittaZigzag(t *testing.T) {
	vLo := r3.NewVec(0, 0, 0)
	vHi := r3.NewVec(1, 0, 0)
	wLo := r3.NewVec(0, 0, 0.001)
	wHi := r3.NewVec(1, 0, 0.001)
	const arcLenUpperA, arcLenUpperB = 1.0, 2.578
	const sagitta = 0.001

	// The old (broken) contract's own answer, pinned here as the violation
	// this fix closes: a wrongly sagitta-fed reading published far less area
	// than the true ruled surface carries.
	oldBroken := cellChordCurveAreaUpper(vLo, vHi, wLo, wHi, arcLenUpperA, arcLenUpperB, sagitta)
	const trueRuledSurfaceArea = 0.2365
	require.Less(t, oldBroken, trueRuledSurfaceArea,
		"pinning the historical violation: the sagitta-fed reading %.6g must fall short of the true ruled-surface area %.6g", oldBroken, trueRuledSurfaceArea)

	// The fixed contract: no caller can honestly state a parameter-matched
	// bound for this curve, so the only value to pass is +Inf, and the
	// helper must publish +Inf right back — never the sagitta's own 0.007734.
	got := cellChordCurveAreaUpper(vLo, vHi, wLo, wHi, arcLenUpperA, arcLenUpperB, math.Inf(1))
	require.True(t, math.IsInf(got, 1), "an unstatable parameter-matched bound must refuse, not publish %.6g", got)
}

// ratUnitCirclePoints returns exact rational points (cos, sin) on the unit
// circle: ((1-t^2)/(1+t^2), 2t/(1+t^2)) is the standard rational
// parametrization, so every coordinate below is an exact big.Rat, no trig
// call and no rounding anywhere. Distinct t give distinct points, and t = 0
// gives (1, 0) — the collapsed half-angle a zero parameter offset needs.
func ratUnitCirclePoints(ts []int64) [][2]*big.Rat {
	one := big.NewRat(1, 1)
	pts := make([][2]*big.Rat, 0, len(ts))
	for _, n := range ts {
		t := big.NewRat(n, 7)
		t2 := new(big.Rat).Mul(t, t)
		den := new(big.Rat).Add(one, t2)
		cos := new(big.Rat).Quo(new(big.Rat).Sub(one, t2), den)
		sin := new(big.Rat).Quo(new(big.Rat).Add(t, t), den)
		pts = append(pts, [2]*big.Rat{cos, sin})
	}
	return pts
}

// TestArcMatchedDeltaEqualsSagitta pins Step 2's own carried-forward
// verification, and pins it as a PROOF rather than as a sample: for a
// CIRCULAR ARC under its own uniform-angle parametrization,
// sup_s |arc(s)-chord(s)| equals the chord's own TRUE sagitta
// 2r*sin^2(theta/4) EXACTLY, at every cell angle theta a build can produce.
// That is the one curve kind (besides a trivial straight LINE) where
// cellChordCurveAreaUpper's matchedDeltaUpper obligation and the loft
// evaluator's sagitta-only sectionDelta field coincide, so every caller
// leaning on the coincidence (bounds.go's own matchedDeltaUpper paragraph,
// loft_build.go's loftCircularCellStations) rests on what follows.
//
// THE DERIVATION. Put the chord's own midpoint on the x axis. An arc of
// radius r subtending theta = 2b, parametrized uniformly in angle by
// s in [0,1], is arc(u) = r*(cos(u*b), sin(u*b)) with u = 2s-1 in [-1,1],
// and the chord it subtends is chord(u) = r*(cos(b), u*sin(b)). Both scale
// linearly in r, so write D(u) = |arc(u)-chord(u)|^2 / r^2 and take r = 1.
// Expanding D through the half-angle substitutions cos b = 1-2*S^2,
// sin b = 2*S*C, cos(u*b) = 1-2*sa^2, sin(u*b) = 2*sa*ca:
//
//	D(u) - (1-cos b)^2 = 4*[ (u*S*C - sa*ca)^2 - sa^2*(2*S^2 - sa^2) ]   (1)
//
// with S = sin(b/2), C = cos(b/2), sa = sin(u*b/2), ca = cos(u*b/2). (1) is
// an ALGEBRAIC identity in five variables tied only by S^2+C^2 = 1 and
// sa^2+ca^2 = 1 — u is free of the two angles in it — and the first subtest
// verifies it over exact rationals rather than leaving it to the reader.
//
// r*(1-cos b) = 2r*sin^2(b/2) is the true sagitta (the half-angle identity,
// with b = theta/2), so the whole claim is that the bracket in (1) is never
// positive:
//
//	|u*S*C - sa*ca| <= sa*sqrt(2*S^2 - sa^2)                            (2)
//
// D is even in u, so u in [0,1] settles it. At u = 0 — the chord's own
// MIDPOINT, s = 1/2 — sa = 0 makes the bracket exactly zero, so (1) is an
// EQUALITY there and the supremum is ATTAINED, never merely approached: the
// separation at s = 1/2 is exactly r*(1-cos b), the sagitta itself. For
// u in (0,1] and a cell angle 2b with b in (0,2]:
//
//   - sa >= u*S and ca >= C — sin is concave on [0,pi], so sin(u*x) >= u*sin(x)
//     there, and cos decreases on [0,pi] — hence u*S*C <= sa*ca, and the left
//     side of (2) is the nonnegative sa*ca - u*S*C;
//   - sa <= u*b/2 (sin x <= x) gives u*S*C/sa >= 2*S*C/b = sin(b)/b, and
//     ca <= 1, so (sa*ca - u*S*C)/sa <= 1 - sin(b)/b;
//   - sa <= S (both half-angles lie in [0,1], where sin increases), so
//     sqrt(2*S^2 - sa^2) >= S and (2) follows from 1 - sin(b)/b <= sin(b/2);
//   - sin x >= x - x^3/6 for x >= 0 bounds both sides of that last one:
//     1 - sin(b)/b <= b^2/6 and sin(b/2) >= b/2 - b^3/48, so it holds
//     whenever b/6 + b^2/48 <= 1/2 — a single rational inequality in b,
//     increasing in b, which the second subtest settles at its own endpoint
//     b = 2 over exact rationals.
//
// b <= 2 covers every cell angle a build can produce, with room to spare:
// chordCount splits a walk into n = ceil(sweep/maxD) chords with
// maxD = 4*asin(sqrt(tol/(2r))) when tol < r and maxD = pi otherwise, and
// asin(sqrt(1/2)) = pi/4 caps the first form at pi too, so a cell angle
// sweep/n never exceeds maxD <= pi — that is 2b <= pi, b <= pi/2 < 2. The
// third subtest reads that ceiling back off chordCount itself.
//
// chordSagitta publishes a PROVEN UPPER bound (sin(x) <= x,
// docs/tessellation-design.md Sec 3) rather than the tight closed form, so
// what a circular cell carries is not the arc's exact matched-parameter
// deviation; it still encloses it,
// which the fourth subtest pins. That a circular cell's matchedDelta IS that
// published sagitta, cell for cell, is pinned in the evaluator itself by
// TestLoftCircularCellStationsMatchedDeltaIsItsSagitta
// (loft_stations_internal_test.go).
func TestArcMatchedDeltaEqualsSagitta(t *testing.T) {
	t.Run("the reduction to (1) is an exact identity", func(t *testing.T) {
		// Both sides of (1) have degree 4 in (C,S), degree 4 in (ca,sa) and
		// degree 2 in u, so their difference P vanishes identically once it
		// vanishes on this grid. Fixing the two circle points, P is a degree-2
		// polynomial in u killed by 4 distinct u; each of its u-coefficients
		// then has degree 4 in (C,S) and, vanishing at 9 > 2*4 points of the
		// unit circle, must vanish on the WHOLE circle (an irreducible conic
		// and a degree-4 curve sharing more than 8 points share a component);
		// running that step once per circle in either order extends it to the
		// whole product. Nothing here is sampled: the grid is a proof.
		ts := []int64{0, 1, 2, 3, 4, 5, 6, 8, 9}
		half := ratUnitCirclePoints(ts)
		one, two, four := big.NewRat(1, 1), big.NewRat(2, 1), big.NewRat(4, 1)
		for _, u := range []*big.Rat{big.NewRat(0, 1), big.NewRat(1, 3), big.NewRat(3, 4), one} {
			for _, hb := range half { // (C, S) = (cos(b/2), sin(b/2))
				cc, ss := hb[0], hb[1]
				// cos b = 1-2*S^2, sin b = 2*S*C.
				cb := new(big.Rat).Sub(one, new(big.Rat).Mul(two, new(big.Rat).Mul(ss, ss)))
				sb := new(big.Rat).Mul(two, new(big.Rat).Mul(ss, cc))
				for _, ha := range half { // (ca, sa) = (cos(u*b/2), sin(u*b/2))
					ca, sa := ha[0], ha[1]
					// cos(u*b) = 1-2*sa^2, sin(u*b) = 2*sa*ca.
					cua := new(big.Rat).Sub(one, new(big.Rat).Mul(two, new(big.Rat).Mul(sa, sa)))
					sua := new(big.Rat).Mul(two, new(big.Rat).Mul(sa, ca))

					// D(u) - (1-cos b)^2, straight off the two points:
					// arc = (cos(u*b), sin(u*b)), chord = (cos b, u*sin b).
					dx := new(big.Rat).Sub(cua, cb)
					dy := new(big.Rat).Sub(sua, new(big.Rat).Mul(u, sb))
					sag := new(big.Rat).Sub(one, cb)
					left := new(big.Rat).Add(new(big.Rat).Mul(dx, dx), new(big.Rat).Mul(dy, dy))
					left.Sub(left, new(big.Rat).Mul(sag, sag))

					// 4*[(u*S*C - sa*ca)^2 - sa^2*(2*S^2 - sa^2)].
					cross := new(big.Rat).Sub(
						new(big.Rat).Mul(u, new(big.Rat).Mul(ss, cc)),
						new(big.Rat).Mul(sa, ca),
					)
					rest := new(big.Rat).Sub(new(big.Rat).Mul(two, new(big.Rat).Mul(ss, ss)), new(big.Rat).Mul(sa, sa))
					right := new(big.Rat).Sub(
						new(big.Rat).Mul(cross, cross),
						new(big.Rat).Mul(new(big.Rat).Mul(sa, sa), rest),
					)
					right.Mul(four, right)

					require.Zerof(t, left.Cmp(right),
						"identity (1) failed at u=%s, (C,S)=(%s,%s), (ca,sa)=(%s,%s): %s vs %s",
						u.RatString(), cc.RatString(), ss.RatString(), ca.RatString(), sa.RatString(),
						left.RatString(), right.RatString())

					// u = 0 forces sa = 0 (the chord's own midpoint), where
					// the bracket is exactly zero: the sagitta is ATTAINED.
					if u.Sign() == 0 && sa.Sign() == 0 {
						require.Zerof(t, left.Sign(),
							"the midpoint separation must equal the sagitta exactly, not merely approach it; got %s", left.RatString())
					}
				}
			}
		}
	})

	t.Run("the closure holds over the whole proven range", func(t *testing.T) {
		// b/6 + b^2/48 <= 1/2 for every b in (0,2]. Both terms strictly
		// increase in b > 0, so the endpoint b = 2 settles the whole range,
		// and it settles it over exact rationals: 1/3 + 1/12 = 5/12 < 1/2.
		b := big.NewRat(2, 1)
		lhs := new(big.Rat).Add(
			new(big.Rat).Quo(b, big.NewRat(6, 1)),
			new(big.Rat).Quo(new(big.Rat).Mul(b, b), big.NewRat(48, 1)),
		)
		require.Equal(t, "5/12", lhs.RatString())
		require.Negativef(t, lhs.Cmp(big.NewRat(1, 2)),
			"the closure b/6 + b^2/48 <= 1/2 must hold at the range endpoint b=2; got %s", lhs.RatString())
	})

	t.Run("every production cell angle lands inside the proven range", func(t *testing.T) {
		// chordCount is the only thing that sets a circular cell's angle
		// (loftCircularCellStations calls it once per side and shares the
		// larger count), and its own maxD <= pi caps that angle at pi — half
		// the proven ceiling of 4 radians. Read back off chordCount itself,
		// over targets from coarser-than-the-radius (the maxD = pi arm) to
		// fine, on radii and sweeps a real section carries.
		for _, radius := range []float64{0.5, 5, 250} {
			for _, sweep := range []float64{0.05, math.Pi / 2, 2 * math.Pi} {
				for _, target := range []float64{1e3, 1, 1e-2, 1e-5} {
					w := circularWalk(0, 0, radius, 0, sweep, radius, sweep)
					w.closed = sweep >= 2*math.Pi
					n, _, err := chordCount(w, target, chordWalkMin(w))
					require.NoError(t, err)
					cell := sweep / float64(n)
					require.LessOrEqualf(t, cell, math.Pi,
						"radius=%v sweep=%v target=%v: a cell angle must never exceed pi; got %v over %d chords", radius, sweep, target, cell, n)
					require.Lessf(t, cell, 4.0,
						"radius=%v sweep=%v target=%v: a cell angle must stay inside the proven range 2b <= 4; got %v", radius, sweep, target, cell)
				}
			}
		}
	})

	t.Run("chordSagitta's published bound encloses the true sagitta", func(t *testing.T) {
		// The derivation above is about the TRUE quantity 2r*sin^2(theta/4);
		// what a cell actually carries is chordSagitta's proven upper bound
		// r*theta^2/(8n^2). This is the sin(x) <= x step, corroborated
		// numerically across the sweep range the evaluator admits.
		const radius = 7.0
		for _, sweepDeg := range []float64{5, 10, 30, 60, 90, 120, 150, 170, 180} {
			theta := sweepDeg * math.Pi / 180
			trueSagitta := 2 * radius * math.Sin(theta/4) * math.Sin(theta/4)
			bound := chordSagitta(radius, theta, 1)
			require.GreaterOrEqualf(t, bound, trueSagitta,
				"sweep=%g: chordSagitta's own proven bound %.10g must enclose the true sagitta %.10g", sweepDeg, bound, trueSagitta)
		}
	})
}

// TestCellChordCurveAreaUpperIsZeroForADegenerateCell pins the legitimate
// zero: both sides collapsed to a point (zero arc length on both) leaves
// nothing for a ruled surface to sweep.
func TestCellChordCurveAreaUpperIsZeroForADegenerateCell(t *testing.T) {
	p := r3.NewVec(1, 2, 3)
	require.Equal(t, 0.0, cellChordCurveAreaUpper(p, p, p, p, 0, 0, 0))
}

// TestCellTwistVolumeAllowIsZeroWithoutTwist pins the existing term's own
// baseline: a wall cell whose four corners already satisfy vHi-vLo ==
// wHi-wLo (the untwisted case every pre-existing loft fixture builds, where
// the two sections' own rules run parallel) has a zero twist vector and
// therefore contributes nothing.
func TestCellTwistVolumeAllowIsZeroWithoutTwist(t *testing.T) {
	vLo := r3.NewVec(1, 0, 0)
	vHi := r3.NewVec(0, 1, 0)
	wLo := vLo.Add(r3.NewVec(0, 0, 5))
	wHi := vHi.Add(r3.NewVec(0, 0, 5))
	require.Equal(t, 0.0, cellTwistVolumeAllow(vLo, vHi, wLo, wHi))
}

// TestCellTwistVolumeAllowMatchesTheSweptMeasure pins the exact determinant
// form against a hand-computed cell. Here a=(1,0,0), b=(0,0,1),
// T=(-1,1,0), so |det(a,T,b)|/12 is exactly 1/12.
func TestCellTwistVolumeAllowMatchesTheSweptMeasure(t *testing.T) {
	vLo := r3.NewVec(0, 0, 0)
	vHi := r3.NewVec(1, 0, 0)
	wLo := r3.NewVec(0, 0, 1)
	wHi := r3.NewVec(0, 1, 1)

	want := 1.0 / 12

	got := cellTwistVolumeAllow(vLo, vHi, wLo, wHi)
	require.InDelta(t, want, got, 1e-12)
	require.GreaterOrEqual(t, got, want, "the answer must round outward, never inward")
}

// TestCellTwistOffsetUpperMatchesPointwiseDeviation pins the exact |T|/4
// maximum against the same hand-computed cell as the swept-measure test.
func TestCellTwistOffsetUpperMatchesPointwiseDeviation(t *testing.T) {
	vLo := r3.NewVec(0, 0, 0)
	vHi := r3.NewVec(1, 0, 0)
	wLo := r3.NewVec(0, 0, 1)
	wHi := r3.NewVec(0, 1, 1)

	twist := vLo.Sub(vHi).Sub(wLo).Add(wHi)
	want := twist.Len() / 4

	got := cellTwistOffsetUpper(vLo, vHi, wLo, wHi)
	require.InDelta(t, want, got, 1e-15)
	require.GreaterOrEqual(t, got, want, "the answer must round outward, never inward")
}

// TestCellTwistOffsetUpperIsZeroWithoutTwist mirrors
// TestCellTwistVolumeAllowIsZeroWithoutTwist for the pointwise reading.
func TestCellTwistOffsetUpperIsZeroWithoutTwist(t *testing.T) {
	vLo := r3.NewVec(1, 0, 0)
	vHi := r3.NewVec(0, 1, 0)
	wLo := vLo.Add(r3.NewVec(0, 0, 5))
	wHi := vHi.Add(r3.NewVec(0, 0, 5))
	require.Equal(t, 0.0, cellTwistOffsetUpper(vLo, vHi, wLo, wHi))
}

// exactCellTwistFactors returns the exact rational squares of |T| and eB.
// The pointwise offset test uses |T|²; the linear area-arm test uses eB².
func exactCellTwistFactors(vLo, vHi, wLo, wHi r3.Vec) (*big.Rat, *big.Rat) {
	sub := func(a, b r3.Vec) [3]*big.Rat {
		return [3]*big.Rat{
			new(big.Rat).Sub(ratOfFloat(a.X), ratOfFloat(b.X)),
			new(big.Rat).Sub(ratOfFloat(a.Y), ratOfFloat(b.Y)),
			new(big.Rat).Sub(ratOfFloat(a.Z), ratOfFloat(b.Z)),
		}
	}
	norm2 := func(d [3]*big.Rat) *big.Rat {
		out := new(big.Rat)
		for _, c := range d {
			out.Add(out, new(big.Rat).Mul(c, c))
		}
		return out
	}
	larger := func(a, b *big.Rat) *big.Rat {
		if b.Cmp(a) > 0 {
			return b
		}
		return a
	}
	// T = vLo − vHi − wLo + wHi = (wHi − wLo) − (vHi − vLo).
	vSpan, wSpan := sub(vHi, vLo), sub(wHi, wLo)
	var twist [3]*big.Rat
	for i := range twist {
		twist[i] = new(big.Rat).Sub(wSpan[i], vSpan[i])
	}
	eB2 := larger(norm2(sub(wLo, vLo)), norm2(sub(wHi, vHi)))
	return norm2(twist), eB2
}

// exactCellTwistVolume returns |det(a,T,b)|/12 over the cell's exact float64
// coordinates, independently of cellTwistVolumeAllow's implementation.
func exactCellTwistVolume(vLo, vHi, wLo, wHi r3.Vec) *big.Rat {
	a := heldDelta(vHi, vLo)
	b := heldDelta(wLo, vLo)
	twist := rvSub(heldDelta(vLo, vHi), heldDelta(wLo, wHi))
	det := rvDot(a, rvCross(twist, b))
	det.Abs(det)
	return det.Quo(det, big.NewRat(12, 1))
}

// TestCellTwistBoundsEncloseTheirExactTerms pins the offset against |T|/4 and
// the volume allowance against |det(a,T,b)|/12 over the two channels that can
// drive a float evaluation below an exact result.
// Neither channel needs pathological geometry: both rows below are ordinary
// finite cells.
//
//   - the CANCELLATION channel. T = vLo − vHi − wLo + wHi is a cancelling
//     chain, so an ordinary parallelogram cell can drive the float-computed
//     chain to EXACTLY (0,0,0) while the exact T — that cell's own addition
//     rounding, here about (-2.8e-17,-5.6e-17,5.6e-17) — stays nonzero. A
//     zero published there is not a loose bound but a wrong one, so the
//     twistUpper <= 0 early return may only short-circuit once the
//     enclosure of |T| is ITSELF zero.
//   - the RAW-NORM channel. r3.Vec.Len is nested math.Hypot, which is not
//     correctly rounded and falls several ulp below the exact norm for a
//     large fraction of vectors, so eA and eB read raw understate the
//     product they feed and a single trailing one-ulp nudge cannot recover
//     the shortfall.
func TestCellTwistBoundsEncloseTheirExactTerms(t *testing.T) {
	for _, row := range []struct {
		name               string
		vLo, vHi, wLo, wHi r3.Vec
		floatChainCancels  bool
	}{
		{
			name:              "cancelling twist chain",
			vLo:               r3.NewVec(0, 0, 0),
			vHi:               r3.NewVec(0.1, 0.2, 0.3),
			wLo:               r3.NewVec(0.7, 1.1, 1.3),
			wHi:               r3.NewVec(0.1, 0.2, 0.3).Add(r3.NewVec(0.7, 1.1, 1.3)),
			floatChainCancels: true,
		},
		{
			name: "raw norms below the exact edge lengths",
			vLo:  r3.NewVec(-0.18407194718000197, -0.9670493813481006, 0.11756527611707068),
			vHi:  r3.NewVec(-0.2543362139571195, 0.17121027904800257, 0.6486272299368296),
			wLo:  r3.NewVec(-0.329681753490781, 0.20068938566549477, 0.2784442506567102),
			wHi:  r3.NewVec(-0.7983173553857771, 0.456370350439145, 0.09559367959010823),
		},
	} {
		t.Run(row.name, func(t *testing.T) {
			twist2, _ := exactCellTwistFactors(row.vLo, row.vHi, row.wLo, row.wHi)
			require.Equal(t, 1, twist2.Sign(), "the fixture must carry a genuinely nonzero exact twist")
			if row.floatChainCancels {
				require.Equal(t, r3.NewVec(0, 0, 0), row.vLo.Sub(row.vHi).Sub(row.wLo).Add(row.wHi),
					"this row exists to exercise the cancellation channel; a nonzero float chain stops exercising it")
			}
			// dominates reports 16·got² >= target, the squared form of
			// got >= sqrt(target)/4 exactCellTwistFactors' doc comment states.
			dominates := func(got float64, target *big.Rat) bool {
				lhs := new(big.Rat).Mul(ratOfFloat(got), ratOfFloat(got))
				lhs.Mul(lhs, new(big.Rat).SetInt64(16))
				return lhs.Cmp(target) >= 0
			}

			offset := cellTwistOffsetUpper(row.vLo, row.vHi, row.wLo, row.wHi)
			require.Positive(t, offset, "the pointwise bound must not vanish while the exact |T| is positive")
			require.True(t, dominates(offset, twist2),
				"cellTwistOffsetUpper = %.20g sits BELOW the exact |T|/4 it claims to dominate", offset)

			volume := cellTwistVolumeAllow(row.vLo, row.vHi, row.wLo, row.wHi)
			wantVolume := exactCellTwistVolume(row.vLo, row.vHi, row.wLo, row.wHi)
			require.GreaterOrEqual(t, ratOfFloat(volume).Cmp(wantVolume), 0,
				"cellTwistVolumeAllow = %.20g sits below the exact determinant measure", volume)
		})
	}
}

// TestCapAreaVolumeAllowIsExactForAPlanarFace pins the closed form's own
// derivation directly: for a cap whose true area exceeds its held polygon's
// area by a KNOWN exact amount, planeOffsetUpper * capAreaAllow / 3 is what
// the divergence-theorem identity Σvol6 = 2*h*Area gives, since
// |ΔVolume| = |h|*|ΔArea|/3.
func TestCapAreaVolumeAllowIsExactForAPlanarFace(t *testing.T) {
	const h, area = 4.0, 6.0
	want := h * area / 3
	got := capAreaVolumeAllow(h, area)
	require.InDelta(t, want, got, 1e-12)
	require.GreaterOrEqual(t, got, want, "the answer must round outward, never inward")
}

// TestCapAreaVolumeAllowIsZeroAtZeroOffsetOrZeroAreaGap pins the two
// legitimate zeros: a cap plane passing exactly through the accumulator's
// own anchor (offset 0, the ordinary case for the FIRST profile's own cap,
// docs/loft-design.md §8) contributes nothing whatever its own area gap,
// and a cap with a proven-zero area gap contributes nothing whatever its
// own plane offset.
func TestCapAreaVolumeAllowIsZeroAtZeroOffsetOrZeroAreaGap(t *testing.T) {
	require.Equal(t, 0.0, capAreaVolumeAllow(0, 6.0))
	require.Equal(t, 0.0, capAreaVolumeAllow(4.0, 0))
}

// TestCapAreaVolumeAllowRefusesOnBrokenClaims pins the reject-only gate: a
// non-finite or negative planeOffsetUpper or capAreaAllow must answer +Inf,
// never a finite number computed past a broken claim.
func TestCapAreaVolumeAllowRefusesOnBrokenClaims(t *testing.T) {
	require.True(t, math.IsInf(capAreaVolumeAllow(math.NaN(), 6.0), 1))
	require.True(t, math.IsInf(capAreaVolumeAllow(4.0, math.NaN()), 1))
	require.True(t, math.IsInf(capAreaVolumeAllow(math.Inf(1), 6.0), 1))
	require.True(t, math.IsInf(capAreaVolumeAllow(4.0, math.Inf(1)), 1))
	require.True(t, math.IsInf(capAreaVolumeAllow(-1, 6.0), 1))
	require.True(t, math.IsInf(capAreaVolumeAllow(4.0, -1), 1))
}

// TestChordedBoundaryVolumeAllowComposesAllFourLegs pins that
// chordedBoundaryVolumeAllow composes its wall chord-to-curve leg, its
// caller-supplied twist leg, its caller-supplied cap leg and its caller-
// supplied seam leg by absSumUpper, never by picking the largest of the four
// or dropping any: with only one leg positive at a time, the whole answer is
// exactly that leg; with all four positive, the answer is at least as large
// as any one leg alone.
func TestChordedBoundaryVolumeAllowComposesAllFourLegs(t *testing.T) {
	// absSumUpper rounds its outward-nudged sum away from an exact value by
	// construction (upRound's own contract), so single-leg cases are checked
	// as an enclosure — never pinned to a literal float this platform's own
	// rounding could move a ulp either way — rather than an exact match.
	twistOnly := chordedBoundaryVolumeAllow(0, 5.0, 3.5, 0, 0)
	require.GreaterOrEqual(t, twistOnly, 3.5)
	require.InDelta(t, 3.5, twistOnly, 1e-12)

	capOnly := chordedBoundaryVolumeAllow(0, 5.0, 0, 2.0, 0)
	require.GreaterOrEqual(t, capOnly, 2.0)
	require.InDelta(t, 2.0, capOnly, 1e-12)

	seamOnly := chordedBoundaryVolumeAllow(0, 5.0, 0, 0, 1.5)
	require.GreaterOrEqual(t, seamOnly, 1.5)
	require.InDelta(t, 1.5, seamOnly, 1e-12)

	require.Equal(t, 0.0, chordedBoundaryVolumeAllow(0, 5.0, 0, 0, 0))
	require.Equal(t, 0.0, chordedBoundaryVolumeAllow(0.01, 0, 0, 0, 0))

	all := chordedBoundaryVolumeAllow(0.01, 5.0, 3.5, 2.0, 1.5)
	wallOnly := chordedBoundaryVolumeAllow(0.01, 5.0, 0, 0, 0)
	require.GreaterOrEqual(t, all, wallOnly)
	require.GreaterOrEqual(t, all, twistOnly)
	require.GreaterOrEqual(t, all, capOnly)
	require.GreaterOrEqual(t, all, seamOnly)
}

// TestChordedBoundaryVolumeAllowRefusesOnBrokenClaims pins F6's own fix: an
// earlier version of this bound let a NaN wallAreaUpper compare false
// against `> 0` and silently vanish from the sum (rather than refusing),
// and let absSumUpper's internal math.Abs flip a negative broken
// twistVolumeUpper, capVolumeUpper or seamAllow positive instead of
// refusing. Every case here must answer +Inf, never a finite number computed
// past a broken claim.
func TestChordedBoundaryVolumeAllowRefusesOnBrokenClaims(t *testing.T) {
	require.True(t, math.IsInf(chordedBoundaryVolumeAllow(math.NaN(), 5.0, 3.5, 2.0, 1.5), 1), "NaN matchedDelta")
	require.True(t, math.IsInf(chordedBoundaryVolumeAllow(-1, 5.0, 3.5, 2.0, 1.5), 1), "negative matchedDelta")
	require.True(t, math.IsInf(chordedBoundaryVolumeAllow(1, math.NaN(), 3.5, 2.0, 1.5), 1), "matchedDelta>0 with NaN wallAreaUpper — F6's own scenario")
	require.True(t, math.IsInf(chordedBoundaryVolumeAllow(1, -1, 3.5, 2.0, 1.5), 1), "matchedDelta>0 with negative wallAreaUpper")
	require.True(t, math.IsInf(chordedBoundaryVolumeAllow(0.01, 5.0, math.NaN(), 2.0, 1.5), 1), "NaN twistVolumeUpper")
	require.True(t, math.IsInf(chordedBoundaryVolumeAllow(0.01, 5.0, -1, 2.0, 1.5), 1), "negative twistVolumeUpper — must refuse, never flip positive via absSumUpper")
	require.True(t, math.IsInf(chordedBoundaryVolumeAllow(0.01, 5.0, 3.5, math.NaN(), 1.5), 1), "NaN capVolumeUpper")
	require.True(t, math.IsInf(chordedBoundaryVolumeAllow(0.01, 5.0, 3.5, -1, 1.5), 1), "negative capVolumeUpper — must refuse, never flip positive via absSumUpper")
	require.True(t, math.IsInf(chordedBoundaryVolumeAllow(0.01, 5.0, 3.5, 2.0, math.NaN()), 1), "NaN seamAllow")
	require.True(t, math.IsInf(chordedBoundaryVolumeAllow(0.01, 5.0, 3.5, 2.0, -1), 1), "negative seamAllow — must refuse, never flip positive via absSumUpper")

	// matchedDelta==0 is a legitimate SKIP of the wall leg regardless of what
	// wallAreaUpper claims (the boundary provably does not move, so the area
	// it would move across is irrelevant) — never a refusal on its own.
	require.Equal(t, 0.0, chordedBoundaryVolumeAllow(0, math.Inf(1), 0, 0, 0))
}

// TestChordedBoundarySeamAllowRefusesOnBrokenClaims pins F2's own seam
// helper against the same reject-only convention: a non-finite or negative
// matchedDelta, posUpper or seamPerimeterUpper must answer +Inf, never a
// finite number computed past a broken claim, and the three legitimate
// zeros (any operand exactly 0) must publish exactly 0.
func TestChordedBoundarySeamAllowRefusesOnBrokenClaims(t *testing.T) {
	require.True(t, math.IsInf(chordedBoundarySeamAllow(math.NaN(), 5.0, 10.0), 1), "NaN matchedDelta")
	require.True(t, math.IsInf(chordedBoundarySeamAllow(-1, 5.0, 10.0), 1), "negative matchedDelta")
	require.True(t, math.IsInf(chordedBoundarySeamAllow(0.01, math.NaN(), 10.0), 1), "NaN posUpper")
	require.True(t, math.IsInf(chordedBoundarySeamAllow(0.01, -1, 10.0), 1), "negative posUpper")
	require.True(t, math.IsInf(chordedBoundarySeamAllow(0.01, 5.0, math.NaN()), 1), "NaN seamPerimeterUpper")
	require.True(t, math.IsInf(chordedBoundarySeamAllow(0.01, 5.0, -1), 1), "negative seamPerimeterUpper")

	require.Equal(t, 0.0, chordedBoundarySeamAllow(0, 5.0, 10.0))
	require.Equal(t, 0.0, chordedBoundarySeamAllow(0.01, 0, 10.0))
	require.Equal(t, 0.0, chordedBoundarySeamAllow(0.01, 5.0, 0))
}

// TestChordedBoundarySeamAllowScalesWithItsThreeOperands pins the closed
// form directly: matchedDelta*posUpper*seamPerimeterUpper/3, rounded
// outward.
func TestChordedBoundarySeamAllowScalesWithItsThreeOperands(t *testing.T) {
	const matchedDelta, posUpper, seamPerimeterUpper = 0.02, 12.0, 40.0
	want := matchedDelta * posUpper * seamPerimeterUpper / 3
	got := chordedBoundarySeamAllow(matchedDelta, posUpper, seamPerimeterUpper)
	require.InDelta(t, want, got, 1e-9)
	require.GreaterOrEqual(t, got, want, "the answer must round outward, never inward")
}

// TestChordedBoundaryMomentAllowComposesTheTwoSweptMeasures pins the two
// moment legs: the chord-to-curve wall measure at coordUpper+matchedDelta and
// the triangle-to-bilinear sweep at coordUpper. Planar cap and seam corrections
// are signed-volume terms and do not enter this measure.
func TestChordedBoundaryMomentAllowComposesTheTwoSweptMeasures(t *testing.T) {
	matchedDelta, wallAreaUpper, twistVolumeUpper, capVolumeUpper, seamAllow := 0.02, 7.5, 1.25, 0.5, 0.1
	maxTwistOffsetUpper, coordUpper := 0.3, 3.0

	wallMeasure := productUpper(matchedDelta, wallAreaUpper)
	want := absSumUpper(
		productUpper(wallMeasure, absSumUpper(coordUpper, matchedDelta)),
		productUpper(twistVolumeUpper, coordUpper),
	)

	got := chordedBoundaryMomentAllow(matchedDelta, wallAreaUpper, twistVolumeUpper, capVolumeUpper, seamAllow, maxTwistOffsetUpper, coordUpper)
	require.Equal(t, want, got)

	// A zero VOLUME is a legitimate zero: nothing is displaced, so there is
	// no moment for any radius to charge.
	require.Equal(t, 0.0, chordedBoundaryMomentAllow(0, wallAreaUpper, 0, 0, 0, 0, coordUpper))

	// A zero coordUpper still leaves the wall sweep's matchedDelta widening.
	zeroCoordWall := productUpper(wallMeasure, absSumUpper(0, matchedDelta))
	require.Equal(t, absSumUpper(zeroCoordWall, productUpper(twistVolumeUpper, 0)),
		chordedBoundaryMomentAllow(matchedDelta, wallAreaUpper, twistVolumeUpper, capVolumeUpper, seamAllow, maxTwistOffsetUpper, 0))

	// R == 0 — every widening leg zero as well — reaches 0 through
	// productUpper's own zero factor, not through a guard on coordUpper.
	require.Equal(t, 0.0, chordedBoundaryMomentAllow(0, wallAreaUpper, twistVolumeUpper, capVolumeUpper, seamAllow, 0, 0))
}

// TestChordedBoundaryMomentAllowWidensTheWallMeasure pins that the
// chord-to-curve sweep can extend matchedDelta beyond the held envelope. The
// twist sweep stays in the four corners' convex hull and needs no widening.
func TestChordedBoundaryMomentAllowWidensTheWallMeasure(t *testing.T) {
	const matchedDelta, wallAreaUpper, twistVolumeUpper, capVolumeUpper, seamAllow = 0.02, 7.5, 1.25, 0.5, 0.1
	const maxTwistOffsetUpper, coordUpper = 0.3, 3.0

	widenedAnswer := chordedBoundaryMomentAllow(matchedDelta, wallAreaUpper, twistVolumeUpper, capVolumeUpper, seamAllow, maxTwistOffsetUpper, coordUpper)
	wallMeasure := productUpper(matchedDelta, wallAreaUpper)
	unwidenedAnswer := absSumUpper(
		productUpper(wallMeasure, coordUpper),
		productUpper(twistVolumeUpper, coordUpper),
	)

	require.Greater(t, widenedAnswer, unwidenedAnswer,
		"the widened term must exceed the SAME product taken over the held coordUpper alone")

	// At coordUpper == 0 only the wall widening remains; the twist sweep is in
	// the held convex hull and therefore carries a zero coordinate radius.
	zeroCoordWidened := chordedBoundaryMomentAllow(matchedDelta, wallAreaUpper, twistVolumeUpper, capVolumeUpper, seamAllow, maxTwistOffsetUpper, 0)
	zeroCoordWall := productUpper(wallMeasure, absSumUpper(0, matchedDelta))
	require.Equal(t, absSumUpper(zeroCoordWall, productUpper(twistVolumeUpper, 0)), zeroCoordWidened)
}

// TestChordedBoundaryMomentAllowRefusesOnBrokenClaims pins the reject-only
// gate over every non-finite argument position: matchedDelta, wallAreaUpper,
// twistVolumeUpper, capVolumeUpper, seamAllow, maxTwistOffsetUpper and
// coordUpper must each, when NaN or a negative claim where negative is
// broken, produce a non-finite published bound, never a finite number
// silently computed past it. F6: a NEGATIVE coordUpper is one such broken
// claim — an earlier version of this bound guarded only isNonFinite(coordUpper)
// and let a negative claim return 0 instead of +Inf.
func TestChordedBoundaryMomentAllowRefusesOnBrokenClaims(t *testing.T) {
	const matchedDelta, wallAreaUpper, twistVolumeUpper, capVolumeUpper, seamAllow = 0.02, 7.5, 1.25, 0.5, 0.1
	const maxTwistOffsetUpper, coordUpper = 0.3, 3.0

	require.True(t, math.IsInf(chordedBoundaryMomentAllow(math.NaN(), wallAreaUpper, twistVolumeUpper, capVolumeUpper, seamAllow, maxTwistOffsetUpper, coordUpper), 1), "NaN matchedDelta")
	require.True(t, math.IsInf(chordedBoundaryMomentAllow(-1, wallAreaUpper, twistVolumeUpper, capVolumeUpper, seamAllow, maxTwistOffsetUpper, coordUpper), 1), "negative matchedDelta")
	require.True(t, math.IsInf(chordedBoundaryMomentAllow(matchedDelta, math.NaN(), twistVolumeUpper, capVolumeUpper, seamAllow, maxTwistOffsetUpper, coordUpper), 1), "NaN wallAreaUpper")
	require.True(t, math.IsInf(chordedBoundaryMomentAllow(matchedDelta, wallAreaUpper, math.NaN(), capVolumeUpper, seamAllow, maxTwistOffsetUpper, coordUpper), 1), "NaN twistVolumeUpper")
	require.True(t, math.IsInf(chordedBoundaryMomentAllow(matchedDelta, wallAreaUpper, twistVolumeUpper, math.NaN(), seamAllow, maxTwistOffsetUpper, coordUpper), 1), "NaN capVolumeUpper")
	require.True(t, math.IsInf(chordedBoundaryMomentAllow(matchedDelta, wallAreaUpper, twistVolumeUpper, capVolumeUpper, math.NaN(), maxTwistOffsetUpper, coordUpper), 1), "NaN seamAllow")
	require.True(t, math.IsInf(chordedBoundaryMomentAllow(matchedDelta, wallAreaUpper, twistVolumeUpper, capVolumeUpper, -1, maxTwistOffsetUpper, coordUpper), 1), "negative seamAllow")
	require.True(t, math.IsInf(chordedBoundaryMomentAllow(matchedDelta, wallAreaUpper, twistVolumeUpper, capVolumeUpper, seamAllow, math.NaN(), coordUpper), 1), "NaN maxTwistOffsetUpper")
	require.True(t, math.IsInf(chordedBoundaryMomentAllow(matchedDelta, wallAreaUpper, twistVolumeUpper, capVolumeUpper, seamAllow, -1, coordUpper), 1), "negative maxTwistOffsetUpper")
	require.True(t, math.IsInf(chordedBoundaryMomentAllow(matchedDelta, wallAreaUpper, twistVolumeUpper, capVolumeUpper, seamAllow, maxTwistOffsetUpper, math.NaN()), 1), "NaN coordUpper")
	require.True(t, math.IsInf(chordedBoundaryMomentAllow(matchedDelta, wallAreaUpper, twistVolumeUpper, capVolumeUpper, seamAllow, maxTwistOffsetUpper, -1), 1), "negative coordUpper — F6")
	require.True(t, math.IsInf(chordedBoundaryMomentAllow(matchedDelta, wallAreaUpper, twistVolumeUpper, capVolumeUpper, seamAllow, maxTwistOffsetUpper, math.Inf(1)), 1), "+Inf coordUpper")
}

// exactTwistAreaLower is a float64 at or below the exact value the linear
// arm names, |T|·(eA+eB), computed here from
// the cell's four float64 corners over big.Rat and a 300-bit big.Float root
// and then nudged one ulp toward zero. It shares no arithmetic with the
// helper under test: the helper's answer must dominate this number, and a
// helper that computed any part of the chain in float64 does not.
func exactTwistAreaLower(vLo, vHi, wLo, wHi r3.Vec) float64 {
	rat := func(v r3.Vec) [3]*big.Rat {
		return [3]*big.Rat{
			new(big.Rat).SetFloat64(v.X),
			new(big.Rat).SetFloat64(v.Y),
			new(big.Rat).SetFloat64(v.Z),
		}
	}
	sub := func(a, b [3]*big.Rat) [3]*big.Rat {
		var out [3]*big.Rat
		for i := range out {
			out[i] = new(big.Rat).Sub(a[i], b[i])
		}
		return out
	}
	norm := func(d [3]*big.Rat) *big.Float {
		q := new(big.Rat)
		for _, c := range d {
			q.Add(q, new(big.Rat).Mul(c, c))
		}
		return new(big.Float).SetPrec(300).Sqrt(new(big.Float).SetPrec(300).SetRat(q))
	}
	maxFloat := func(a, b *big.Float) *big.Float {
		if a.Cmp(b) >= 0 {
			return a
		}
		return b
	}
	rvLo, rvHi, rwLo, rwHi := rat(vLo), rat(vHi), rat(wLo), rat(wHi)

	twist := norm(sub(sub(rvLo, rvHi), sub(rwLo, rwHi)))
	eA := maxFloat(norm(sub(rvHi, rvLo)), norm(sub(rwHi, rwLo)))
	eB := maxFloat(norm(sub(rwLo, rvLo)), norm(sub(rwHi, rvHi)))
	prod := new(big.Float).SetPrec(300).Mul(twist, new(big.Float).SetPrec(300).Add(eA, eB))
	f, _ := prod.Float64()
	return math.Nextafter(f, math.Inf(-1))
}

// TestCellTwistAreaLinearArmDominatesTheExactProduct pins the fallback arm to
// the exact-arithmetic discipline used for every twist-vector reading.
//
// The first is CANCELLATION. T = vLo−vHi−wLo+wHi is a cancelling chain, so a
// float64 evaluation can answer exactly (0,0,0) for a cell whose exact T is
// nonzero, and this helper reads that zero as "planar, nothing to charge".
// The huge-scale row below is the sharpest form: the float chain cancels to
// (0,0,0) while the exact T is the unit vector (−1,0,0), so the deviation the
// helper claims to dominate is about 1e16 and a float64 chain publishes 0.
// The residue row is the ordinary form — a cell whose second rule was BUILT
// by adding a common offset, where the exact T is the addition's own rounding
// residue.
//
// The second is r3.Vec.Len, nested math.Hypot, which Go publishes no accuracy
// contract for and which sits several ulp BELOW the exact norm often enough
// that eA and eB taken that way are not upper bounds. The randomized sweep
// row cover it: every answer must dominate the independently computed exact
// product, never merely approach it.
func TestCellTwistAreaLinearArmDominatesTheExactProduct(t *testing.T) {
	residueVHi := r3.NewVec(0.1, 0.2, 0.3)
	residueWLo := r3.NewVec(0.7, 1.1, 1.3)

	for _, tc := range []struct {
		name               string
		vLo, vHi, wLo, wHi r3.Vec
		floatCancels       bool
	}{
		{
			// The float chain gives (1e16−1) − 1e16 + 0 = 0 in every
			// coordinate; the exact chain gives (−1, 0, 0).
			name:         "cancelling chain at a huge scale",
			vLo:          r3.NewVec(1e16, 0, 0),
			vHi:          r3.NewVec(1, 0, 0),
			wLo:          r3.NewVec(1e16, 1, 0),
			wHi:          r3.NewVec(0, 1, 0),
			floatCancels: true,
		},
		{
			// wHi is BUILT as vHi+wLo, so the float chain cancels to
			// exactly (0,0,0) while the exact T is that addition's own
			// rounding residue.
			name:         "cancelling chain over a rounding residue",
			vLo:          r3.NewVec(0, 0, 0),
			vHi:          residueVHi,
			wLo:          residueWLo,
			wHi:          residueVHi.Add(residueWLo),
			floatCancels: true,
		},
		{
			// An ordinary cell — nothing cancels — where the four
			// r3.Vec.Len readings sit far enough below their exact norms
			// that one up-round cannot recover the shortfall: taking the
			// spans that way publishes the float64 just below the exact
			// product.
			name: "libm norms shrink the product past one up-round",
			vLo:  r3.NewVec(-0.8675135336778659, 0.29838195716762295, -0.3981530225038339),
			vHi:  r3.NewVec(0.5107684458941033, -0.6206943280001098, -0.15068689686676673),
			wLo:  r3.NewVec(0.6646268586257917, 0.4551956430202062, -0.14964066133325593),
			wHi:  r3.NewVec(0.9089996001491185, -0.897793599718202, -0.08340000409501869),
		},
	} {
		t.Run(tc.name, func(t *testing.T) {
			if tc.floatCancels {
				require.Equal(t, r3.NewVec(0, 0, 0), tc.vLo.Sub(tc.vHi).Sub(tc.wLo).Add(tc.wHi),
					"fixture premise: the float64 twist chain cancels to zero here")
			}

			corners := cellCornersOf(tc.vLo, tc.vHi, tc.wLo, tc.wHi)
			got := cellTwistAreaLinearFromSpans(corners.spans(), xtwistQuarterUpper(corners))
			want := exactTwistAreaLower(tc.vLo, tc.vHi, tc.wLo, tc.wHi)
			require.Greater(t, want, 0.0, "fixture premise: the exact deviation is positive")
			require.GreaterOrEqual(t, got, want,
				"the published allowance must dominate the exact |T|(eA+eB) it claims to bound")
		})
	}

	t.Run("an exactly planar cell still answers zero", func(t *testing.T) {
		// T = (0,0,0) − (1,0,0) − (0,1,0) + (1,1,0) is exactly the zero
		// vector, not a cancelled one, so there is no deviation to charge.
		vLo, vHi := r3.NewVec(0, 0, 0), r3.NewVec(1, 0, 0)
		wLo, wHi := r3.NewVec(0, 1, 0), r3.NewVec(1, 1, 0)
		corners := cellCornersOf(vLo, vHi, wLo, wHi)
		require.Zero(t, cellTwistAreaLinearFromSpans(corners.spans(), xtwistQuarterUpper(corners)))
	})

	t.Run("a non-finite corner refuses", func(t *testing.T) {
		require.True(t, math.IsInf(cellTwistAreaAllow(
			r3.NewVec(math.NaN(), 0, 0), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0), r3.NewVec(1, 1, 1)), 1))
	})

	t.Run("randomized cells", func(t *testing.T) {
		rng := rand.New(rand.NewPCG(0x7e15, 0xa4ea))
		vec := func() r3.Vec {
			return r3.NewVec(rng.Float64()*2-1, rng.Float64()*2-1, rng.Float64()*2-1)
		}
		for i := range 400 {
			vLo, vHi, wLo, wHi := vec(), vec(), vec(), vec()
			corners := cellCornersOf(vLo, vHi, wLo, wHi)
			got := cellTwistAreaLinearFromSpans(corners.spans(), xtwistQuarterUpper(corners))
			require.GreaterOrEqual(t, got, exactTwistAreaLower(vLo, vHi, wLo, wHi), "cell %d", i)
		}
	})
}

// TestCellTwistAreaAllowEnclosesTheBilinearGap checks the published minimum
// against an independent dense integration of the ruled patch and the held
// triangle pair. The rows include the small-twist shape whose quadratic arm
// is much tighter than the linear fallback.
func TestCellTwistAreaAllowEnclosesTheBilinearGap(t *testing.T) {
	for _, tc := range []struct {
		name               string
		vLo, vHi, wLo, wHi r3.Vec
		sharp              bool
	}{
		{
			name: "small rotational twist",
			vLo:  r3.NewVec(9.5, 0, 0), vHi: r3.NewVec(9.49, 0.2, 0),
			wLo: r3.NewVec(9.49, 0.5, 5), wHi: r3.NewVec(9.47, 0.7, 5),
			sharp: true,
		},
		{
			name: "skew cell",
			vLo:  r3.NewVec(0, 0, 0), vHi: r3.NewVec(2, 0.5, 0),
			wLo: r3.NewVec(0.2, -0.1, 3), wHi: r3.NewVec(1.8, 1.2, 3.2),
		},
	} {
		t.Run(tc.name, func(t *testing.T) {
			patchArea := bilinearPatchAreaNumeric(tc.vLo, tc.vHi, tc.wLo, tc.wHi)
			heldArea := heldTrianglePairArea(tc.vLo, tc.vHi, tc.wLo, tc.wHi)
			gap := math.Abs(patchArea - heldArea)
			allow := cellTwistAreaAllow(tc.vLo, tc.vHi, tc.wLo, tc.wHi)
			require.LessOrEqual(t, gap, allow+1e-8,
				"the allowance must enclose the independently integrated area gap")
			if tc.sharp {
				corners := cellCornersOf(tc.vLo, tc.vHi, tc.wLo, tc.wHi)
				linear := cellTwistAreaLinearFromSpans(corners.spans(), xtwistQuarterUpper(corners))
				require.Less(t, allow, linear,
					"the cancellation-preserving arm must tighten a small rotational twist")
			}
		})
	}
}
