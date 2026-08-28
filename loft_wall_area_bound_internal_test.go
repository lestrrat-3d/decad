package decad

import (
	"fmt"
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file owns the fixtures for bounds.go's cellChordCurveAreaAllow — the
// RULED leg of one loft wall cell's own area gap, |Area(bilinear chord patch) −
// Area(ruled patch)| with BOTH patches pinned at the four corners that helper
// is handed — and for the two readings it is built on,
// uniformSpeedTangentEnergyUpper and cellChordPatchNormalLower. It also owns
// the fixture for the three-leg composition that leg is spent in, and for the
// STATION-SHIFT leg cellStationShiftAreaAllow that reaches from those held
// corners to the stations they denote:
// TestDisplacedStationCellNeedsTheStationShiftLeg.
//
// THE REFERENCE IS INTEGRATED DIRECTLY, NEVER BUILT THROUGH THE PRODUCTION
// PATH. ruledPatchAreaGap below integrates |X_s × X_r| over the unit square for
// BOTH patches at the SAME quadrature nodes and sums the pointwise difference,
// so the quadrature's own error largely cancels and the reference is a plain
// numerical integral of the two surfaces — it never calls Document.Loft, never
// touches loftMassAccumulator, and shares no code with the bound it checks.
// Resolution is justified by convergedRuledGap's own successive-refinement
// sweep, whose criterion reads the REFERENCE's own agreement between two
// consecutive resolutions and nothing else: zeroing any leg of the bound under
// test cannot move that gate, so a leg-zeroing always reaches the enclosure
// assertion itself rather than failing on a precondition.
//
// EVERY FIXTURE CARRIES TWIST. loftTwistSweepDegrees is 0, 5, 15 and 30
// degrees, and at every nonzero angle the two sections' chords are genuinely
// non-parallel, so the cell's own twist vector T = vLo−vHi−wLo+wHi is nonzero
// and the correspondence is a real rotation rather than an offset. The
// untwisted 0-degree row is kept as the baseline, never as a falsifier.
//
// FALSIFICATION LEDGER (verified by hand: zero the named term in the named
// source, rerun the named test, confirm RED, restore). Each entry quotes the
// ACTUAL failing assertion.
//
//   - L1, the CARRIED-OVER twist leg (cellTwistAreaAllow), zeroed in
//     loft_moments.go's own composition
//     (`absSumUpper(areaExcess, ruledLeg, 0*twistLeg, stationLeg)`):
//     TestLoftRotatedWedgeAreaBoundEnclosesDenotedSurface fails at 15 degrees —
//     `"0.5053145056934625" is not less than or equal to "0.07130694749809868"`
//     — on "the loft's own Area must enclose the denoted ruled surface at 15.0
//     deg of twist", the enclosure assertion itself.
//   - L2, the ruled leg (cellChordCurveAreaAllow), zeroed the same way
//     (`absSumUpper(areaExcess, 0*ruledLeg, twistLeg, stationLeg)`):
//     TestLoftTallThinArcWedgeAreaBoundEnclosesConvergedReference fails —
//     `"0.002446194848033656" is not less than or equal to
//     "0.000460843512606508"`;
//     TestLoftShearedArcWedgeAreaBoundEnclosesConvergedReference fails —
//     `"0.031970386054553046" is not less than or equal to
//     "0.0060826530347349314"`;
//     TestLoftRotatedWedgeAreaBoundEnclosesDenotedSurface fails at 0 degrees —
//     `"0.025836198167183966" is not less than or equal to
//     "0.004918800807284351"` — each on its own enclosure assertion.
//   - L3, the sharp arm's OSCILLATION term (bounds.go's oscW):
//     TestCellChordCurveAreaAllowComposesEveryTerm/oscillation-carried fails —
//     `Relative error is too high: 1e-13 (expected)` — on "the published bound
//     must be the four derivation terms composed as min(ceiling,
//     osc+md+quad)", and /ceiling-carried fails on the same assertion. NO
//     ENCLOSURE FIXTURE CAN BIND THIS TERM, and it is not redundant: it bounds
//     the mean-zero part of the surface-normal difference, which the derivation
//     cannot drop. Every geometry family searched — twisted arc pairs, tilted
//     pairs, coaxial frustum cells, parameter-shifted coaxial cells,
//     opposite-bulge cells, and each under a further rotation — either has the
//     matched-delta or quadratic term dominating it, or, where it does
//     dominate (up to 7.6x, measured), has the premise-free ceiling below the
//     whole sharp arm, so the published minimum never turns on it. Hence the
//     structural falsifier above, whose recomposition is written out
//     independently of the function under test.
//
//     The undiscriminability is MEASURED, so it can be rechecked: sweeping
//     30,240 cells across those families and keeping the 17,680 where
//     deleting oscTerm actually LOWERS the published minimum, the directly
//     integrated ruled-patch gap never reached 0.67 of the stripped bound —
//     worst case 0.6654, an opposite-bulge cell, at 1024^2 quadrature nodes.
//     So an enclosure assertion of the shape `gap <= allow` still passes with
//     the term deleted, and adding one would record coverage it does not
//     have. What deleting `productUpper(oscW, iMax)` from bounds.go's lin
//     does redden is four subtests across two tests:
//     TestCellChordCurveAreaAllowComposesEveryTerm/{oscillation-carried,
//     ceiling-carried} below, and
//     TestCellChordCurveAreaAllowEnclosesItsExactTerms/{cancelling twist,
//     oscillation carried; cancelling twist, matched delta carried} in
//     bounds_proven_norm_internal_test.go — whose refChordCurveAreaAllow
//     big.Float reference restates the published value mathematically and
//     never calls cellChordCurveAreaAllow, so those rows are not a
//     self-consistency check of the function against itself.
//   - L4, the sharp arm's MATCHED-DELTA term:
//     TestCellChordCurveAreaAllowEnclosesOppositeBulgeGap fails —
//     `"0.004686786475488334" is not less than or equal to
//     "0.00023754646338915357"` — on "the ruled leg must enclose the directly
//     integrated gap on a cell whose two sides bulge apart", the enclosure
//     assertion itself.
//   - L5, the sharp arm's QUADRATIC term:
//     TestCellChordCurveAreaAllowEnclosesTwistedRuledGap fails —
//     `"0.047904091377782294" is not less than or equal to
//     "0.03990633323647029"` — on "the ruled leg must enclose the directly
//     integrated chord-patch-to-ruled-patch area gap (twist 0.0 deg, cell sweep
//     0.400, height 12.0)".
//   - L6, the PREMISE-FREE ceiling:
//     TestCellChordCurveAreaAllowFallsBackOnADegenerateCell fails —
//     `"0.11616013542120403" is not less than or equal to "0"` — on "the
//     premise-free arm alone must enclose the gap where the sharp arm has no
//     area element", which is the only arm that cell has;
//     TestCellChordCurveAreaAllowEnclosesTwistedRuledGap and
//     ...EnclosesOppositeBulgeGap fail on their own enclosure assertions too.
//   - L7, uniformSpeedTangentEnergyUpper neutered to +Inf (the whole
//     tangent-deviation energy withdrawn):
//     TestCellChordCurveAreaAllowComposesEveryTerm fails on three rows —
//     `"0.8585262442152457" is not less than "0.03434351964087584"` and its
//     siblings — on "this row must be carried by the sharp arm": without the
//     energy every reading falls back to arcLen+chord and the sharp arm stops
//     being the tighter one at all.
//   - L8, the composition's THIRD leg, bounds.go's
//     cellStationShiftAreaAllow neutered to return 0:
//     TestDisplacedStationCellNeedsTheStationShiftLeg fails on all three rows
//     — `"0.019999497629115126" is not less than or equal to "0"`,
//     `"0.03999974896660774" is not less than or equal to "0"` and
//     `"0.002399004185227094" is not less than or equal to "0"` — on "the
//     station-shift leg must cover the directly integrated area step from the
//     held-corner ruled patch to the denoted one", the leg's own enclosure
//     assertion, and the three-leg assertion beneath it reddens on the same
//     rows. That fixture's own twoLegs assertion is the standing form of the
//     same falsification, so the entry stays red without needing the edit.
//
// The end-to-end twist fixtures live beside the older shear ones in
// loft_area_excess_fixture_internal_test.go's own family; the ones here are
// this file's own rotated-wedge pair.

// referenceUlpSlack is the relative window a float-evaluated REFERENCE in this
// file is compared against — a few ulps, covering that reference's own
// evaluation error and nothing else. It is never applied to a bound: an
// understated bound misses by orders of magnitude, never by an ulp (the
// package's own FMA-contraction rule: never pin a float bound to a locally
// measured literal).
const referenceUlpSlack = 8 * unitRoundoff

// loftTwistSweepDegrees is the rotation sweep every fixture in this file runs:
// an untwisted baseline plus three genuinely twisted rows.
var loftTwistSweepDegrees = []float64{0, 5, 15, 30}

// ruledArc is a CONSTANT-SPEED circular directrix — the same uniform-ANGLE
// parametrization loftCircularCellStations places its stations under, which is
// what makes uniformSpeedTangentEnergyUpper's own premise hold for it.
type ruledArc struct {
	centre r3.Vec
	radius float64
	u, v   r3.Vec
	t0, dt float64
}

func (a ruledArc) at(s float64) r3.Vec {
	sin, cos := math.Sincos(a.t0 + s*a.dt)
	return a.centre.Add(a.u.Scale(a.radius * cos)).Add(a.v.Scale(a.radius * sin))
}

func (a ruledArc) der(s float64) r3.Vec {
	sin, cos := math.Sincos(a.t0 + s*a.dt)
	return a.u.Scale(-a.radius * a.dt * sin).Add(a.v.Scale(a.radius * a.dt * cos))
}

// arcLen is the directrix's own exact arc length over the cell, and — the
// parametrization being uniform in angle — also its constant speed.
func (a ruledArc) arcLen() float64 { return math.Abs(a.radius * a.dt) }

// sagittaUpper is the arc's own maximum departure from its chord AT THE SAME
// PARAMETER, read through chordSagitta's own closed form R*sweep^2/8 rather
// than R*(1-cos(sweep/2)) — the identical shape the real build charges, and a
// strict upper bound on the true departure because 1-cos(x) <= x^2/2. Under the
// uniform-angle parametrization that departure IS the ordinary sagitta
// (loftCircularCellStations' own doc comment), and
// TestCellChordCurveAreaAllowSagittaIsParameterMatched checks it by sampling
// rather than assuming it.
func (a ruledArc) sagittaUpper() float64 {
	return chordSagitta(math.Abs(a.radius), math.Abs(a.dt), 1)
}

// ruledPatchAreaGap integrates Area(true ruled patch) − Area(bilinear chord
// patch) directly: both integrands are |X_s × X_r| over the unit square, and
// the two are subtracted AT THE SAME NODE before summing, so the midpoint
// rule's own truncation error cancels to the order the two surfaces agree.
// Nothing here reads the production evaluator.
func ruledPatchAreaGap(a, b ruledArc, n int) float64 {
	vLo, vHi := a.at(0), a.at(1)
	wLo, wHi := b.at(0), b.at(1)
	da, db := vHi.Sub(vLo), wHi.Sub(wLo)
	h := 1 / float64(n)
	sum := 0.0
	for i := range n {
		s := (float64(i) + 0.5) * h
		gTrue := b.at(s).Sub(a.at(s))
		gChord := wLo.Add(db.Scale(s)).Sub(vLo.Add(da.Scale(s)))
		aDer, bDer := a.der(s), b.der(s)
		for j := range n {
			r := (float64(j) + 0.5) * h
			xsTrue := aDer.Scale(1 - r).Add(bDer.Scale(r))
			xsChord := da.Scale(1 - r).Add(db.Scale(r))
			sum += xsTrue.Cross(gTrue).Len() - xsChord.Cross(gChord).Len()
		}
	}
	return sum * h * h
}

// convergedRuledGap sweeps ruledPatchAreaGap at increasing resolutions and
// returns the finest value once two SUCCESSIVE resolutions agree to within a
// FIXED relative tolerance OF THE REFERENCE ITSELF. The criterion never reads
// the bound under test, so zeroing a leg of that bound cannot tighten or
// loosen this gate.
func convergedRuledGap(t *testing.T, a, b ruledArc) float64 {
	t.Helper()
	const relTol = 1e-4
	prev, havePrev := 0.0, false
	for _, n := range []int{64, 128, 256, 512, 1024} {
		cur := ruledPatchAreaGap(a, b, n)
		if havePrev && math.Abs(cur-prev) <= relTol*math.Abs(cur) {
			return cur
		}
		prev, havePrev = cur, true
	}
	t.Fatalf("the directly integrated ruled-patch area gap did not converge to relative %.3g", relTol)
	return 0
}

// twistedArcCellPair builds one wall cell whose two sides are constant-speed
// circular arcs in parallel planes, the upper one the lower ROTATED about the
// world z axis by phi. The arc centre sits offCentre away from that rotation
// axis, so no point of either side is anywhere near the axis and the two wall
// fans never fold through each other: a genuine rotation of the correspondence,
// not an offset, and the cell's own twist vector is nonzero for every phi != 0.
func twistedArcCellPair(phi, offCentre, radius, height, t0, dt float64) (ruledArc, ruledArc) {
	rot := func(v r3.Vec) r3.Vec {
		sin, cos := math.Sincos(phi)
		return r3.NewVec(v.X*cos-v.Y*sin, v.X*sin+v.Y*cos, v.Z)
	}
	lo := ruledArc{
		centre: r3.NewVec(offCentre, 0, 0),
		radius: radius,
		u:      r3.NewVec(1, 0, 0),
		v:      r3.NewVec(0, 1, 0),
		t0:     t0, dt: dt,
	}
	hi := ruledArc{
		centre: rot(lo.centre).Add(r3.NewVec(0, 0, height)),
		radius: radius,
		u:      rot(lo.u),
		v:      rot(lo.v),
		t0:     t0, dt: dt,
	}
	return lo, hi
}

// cellAllowFor calls cellChordCurveAreaAllow with the obligations each side's
// own analytic geometry discharges: the exact arc length as the tangent bound,
// the parameter-matched sagitta as matchedDelta, and
// uniformSpeedTangentEnergyUpper over a chord rounded DOWN twice as the
// tangent-deviation energy — the same discharge perCellTangentEnergy's own
// circular arm makes in the real build.
func cellAllowFor(a, b ruledArc) float64 {
	vLo, vHi := a.at(0), a.at(1)
	wLo, wHi := b.at(0), b.at(1)
	arcA, arcB := upRound(a.arcLen()), upRound(b.arcLen())
	md := math.Max(a.sagittaUpper(), b.sagittaUpper())
	chordLo := func(c ruledArc) float64 {
		return downRound(downRound(2 * math.Abs(c.radius) * math.Sin(math.Abs(c.dt)/2)))
	}
	return cellChordCurveAreaAllow(vLo, vHi, wLo, wHi, arcA, arcB, md,
		uniformSpeedTangentEnergyUpper(arcA, chordLo(a)),
		uniformSpeedTangentEnergyUpper(arcB, chordLo(b)))
}

// TestCellChordCurveAreaAllowEnclosesTwistedRuledGap is the ruled leg's own
// primary falsifier: over a twist sweep and a range of per-cell sweeps, the
// published bound must enclose the DIRECTLY INTEGRATED gap between the true
// ruled patch and the bilinear chord patch. The reference shares no code with
// the bound; its resolution is settled by convergedRuledGap's own criterion.
func TestCellChordCurveAreaAllowEnclosesTwistedRuledGap(t *testing.T) {
	cellSweeps := []float64{0.4, 0.1, 0.025}
	for _, deg := range loftTwistSweepDegrees {
		phi := deg * math.Pi / 180
		for _, dt := range cellSweeps {
			for _, height := range []float64{1, 12} {
				lo, hi := twistedArcCellPair(phi, 4, 1.5, height, 0.3, dt)
				gap := math.Abs(convergedRuledGap(t, lo, hi))
				allow := cellAllowFor(lo, hi)
				t.Logf("twist=%4.1fdeg cellSweep=%.3f height=%4.1f: gap=%.6e allow=%.6e ratio=%.4f",
					deg, dt, height, gap, allow, gap/allow)
				require.LessOrEqual(t, gap, allow,
					"the ruled leg must enclose the directly integrated chord-patch-to-ruled-patch area gap (twist %.1f deg, cell sweep %.3f, height %.1f)",
					deg, dt, height)
			}
		}
	}
}

// TestCellChordCurveAreaAllowSurvivesAnUnderflowingScale is the ruled leg's own
// UNDERFLOW falsifier: at a scale where the products the bound forms internally
// round to +0, the published value must still enclose the cell's own gap rather
// than collapse to a proven zero.
//
// The cell is a pure translational sweep — one constant-speed arc of radius r
// and sweep dt, plus its copy displaced by sep along z — so both areas are
// closed form and the reference needs no integration. The ruled patch's own
// area element is |a'(s) x sep*zhat| = sep*|a'(s)|, the tangent being
// perpendicular to z, which integrates to sep*(r*dt); the bilinear chord patch
// replaces the arc by its chord and gives sep*(2*r*sin(dt/2)). The gap is
// therefore exactly sep*r*(dt - 2*sin(dt/2)), and this fixture forms the sep*r
// factor FIRST, so the reference itself never touches the extremes the bound is
// being asked to hold apart.
//
// A bound that published 0 here would not merely be loose: exactnessOf reads a
// zero bound as the claim that the area is exactly representable, so the body
// would report Exact while missing the whole gap — precisely what
// docs/loft-design.md §8's "Area is never Exact" forbids.
//
// The two separations and the two energy arms are four DIFFERENT flush sites,
// not one restated:
//
//   - a wide rung (sep = 1e200) leaves the premise-free arm at ordinary scale,
//     so only the PROVEN-energy arm flushes — inside
//     uniformSpeedTangentEnergyUpper's own (arcLen-chord)*(arcLen+chord)
//     product, which is quadratically smaller than the gap it serves;
//   - a unit rung (sep = 1) shrinks every term to the section's own scale, so
//     the premise-free square in tangentDeviationUpper flushes too and the
//     ABSENT-energy arm collapses as well. There the free arm survives at
//     8e-201 and the sharper arm is the one that flushes, so the published
//     minimum takes the flushed arm over a sound one — a bound that is wrong by
//     every order of magnitude it had, not by an ulp.
//
// A repair applied at any single one of those sites leaves the others
// publishing 0.
func TestCellChordCurveAreaAllowSurvivesAnUnderflowingScale(t *testing.T) {
	const radius, dt = 1e-200, 0.4
	for _, separation := range []float64{1e200, 1} {
		lo, hi := twistedArcCellPair(0, 0, radius, separation, 0.3, dt)
		gap := (separation * radius) * (dt - 2*math.Sin(dt/2))
		require.Positive(t, gap, "the reference must be a representable quantity worth missing")

		vLo, vHi, wLo, wHi := lo.at(0), lo.at(1), hi.at(0), hi.at(1)
		arcA, arcB := upRound(lo.arcLen()), upRound(hi.arcLen())
		md := math.Max(lo.sagittaUpper(), hi.sagittaUpper())
		chordLower := downRound(downRound(2 * radius * math.Sin(dt/2)))
		energy := uniformSpeedTangentEnergyUpper(arcA, chordLower)
		t.Run(fmt.Sprintf("sep=%g/energy", separation), func(t *testing.T) {
			require.Positive(t, energy,
				"this cell's arc genuinely exceeds its chord, so its own tangent-deviation energy is not zero")
		})

		for _, tc := range []struct {
			name   string
			energy float64
		}{
			{"proven energy", energy},
			{"absent energy", math.Inf(1)},
		} {
			t.Run(fmt.Sprintf("sep=%g/%s", separation, tc.name), func(t *testing.T) {
				allow := cellChordCurveAreaAllow(vLo, vHi, wLo, wHi, arcA, arcB, md, tc.energy, tc.energy)
				t.Logf("gap=%.9e allow=%.9e", gap, allow)
				require.LessOrEqual(t, gap, allow,
					"the ruled leg must enclose the underflowing cell's own chord-patch-to-ruled-patch gap")
				require.Equal(t, Approximate, exactnessOf(allow),
					"a bound whose intermediates flushed must never publish as Exact")
			})
		}
	}
}

// TestCellChordCurveAreaAllowSagittaIsParameterMatched pins the one premise
// cellAllowFor hands the bound that this file cannot read off the geometry
// directly: the analytic sagitta really is a bound on |curve(s) − chord(s)| at
// the SAME s, not merely a set distance. It is checked by sampling, so a
// fixture that quietly stopped satisfying it would be caught here rather than
// silently weakening every enclosure above.
func TestCellChordCurveAreaAllowSagittaIsParameterMatched(t *testing.T) {
	for _, dt := range []float64{0.4, 0.1, 0.025} {
		lo, _ := twistedArcCellPair(0, 4, 1.5, 1, 0.3, dt)
		chordStart, chord := lo.at(0), lo.at(1).Sub(lo.at(0))
		worst := 0.0
		for k := range 2001 {
			s := float64(k) / 2000
			worst = math.Max(worst, lo.at(s).Sub(chordStart.Add(chord.Scale(s))).Len())
		}
		require.LessOrEqual(t, worst, lo.sagittaUpper(),
			"the uniform-angle sagitta must dominate the parameter-matched departure at cell sweep %.3f", dt)
	}
}

// TestUniformSpeedTangentEnergyUpperEnclosesTheIntegratedEnergy checks the
// energy reading against the quantity it claims to bound, integrated directly:
// the mean square of |curve'(s) − chord| over the cell.
func TestUniformSpeedTangentEnergyUpperEnclosesTheIntegratedEnergy(t *testing.T) {
	for _, dt := range []float64{0.9, 0.4, 0.1, 0.025} {
		lo, _ := twistedArcCellPair(0, 4, 1.5, 1, 0.3, dt)
		chord := lo.at(1).Sub(lo.at(0))
		const n = 20000
		sum := 0.0
		for k := range n {
			s := (float64(k) + 0.5) / n
			d := lo.der(s).Sub(chord)
			sum += d.Dot(d)
		}
		integrated := sum / n
		published := uniformSpeedTangentEnergyUpper(upRound(lo.arcLen()),
			downRound(downRound(2*math.Abs(lo.radius)*math.Sin(math.Abs(dt)/2))))
		t.Logf("cellSweep=%.3f integrated energy=%.9e published=%.9e", dt, integrated, published)
		require.LessOrEqual(t, integrated, published,
			"the published tangent-deviation energy must enclose the integrated one at cell sweep %.3f", dt)
	}
}

// TestUniformSpeedTangentEnergyUpperRefusesBrokenClaims pins the +Inf answers:
// a non-finite or negative operand, or an arc-length claim below the chord it
// is supposed to subtend, is a broken caller claim and never a small bound.
func TestUniformSpeedTangentEnergyUpperRefusesBrokenClaims(t *testing.T) {
	for name, args := range map[string][2]float64{
		"non-finite arc": {math.Inf(1), 1},
		"NaN chord":      {2, math.NaN()},
		"negative arc":   {-1, 0},
		"negative chord": {2, -1},
		"chord past arc": {1, 2},
	} {
		require.True(t, math.IsInf(uniformSpeedTangentEnergyUpper(args[0], args[1]), 1),
			"%s must answer +Inf", name)
	}
	require.Equal(t, 0.0, uniformSpeedTangentEnergyUpper(0, 0),
		"a wholly degenerate cell has no energy to publish")
}

// TestCellChordPatchNormalLowerBoundsTheAreaElement checks the four-corner
// convex-combination reduction against the quantity it bounds: the minimum of
// |X_s × X_r| over the whole unit square of the bilinear chord patch, sampled
// densely. The sampled minimum is an upper bound on the true minimum, so a
// published value above it would be a proven falsification.
func TestCellChordPatchNormalLowerBoundsTheAreaElement(t *testing.T) {
	for _, deg := range loftTwistSweepDegrees {
		phi := deg * math.Pi / 180
		lo, hi := twistedArcCellPair(phi, 4, 1.5, 3, 0.3, 0.4)
		vLo, vHi, wLo, wHi := lo.at(0), lo.at(1), hi.at(0), hi.at(1)
		da, db := vHi.Sub(vLo), wHi.Sub(wLo)
		g0, g1 := wLo.Sub(vLo), wHi.Sub(vHi)
		sampled := math.Inf(1)
		const n = 200
		for i := range n + 1 {
			s := float64(i) / n
			g := g0.Scale(1 - s).Add(g1.Scale(s))
			for j := range n + 1 {
				r := float64(j) / n
				p := da.Scale(1 - r).Add(db.Scale(r))
				sampled = math.Min(sampled, p.Cross(g).Len())
			}
		}
		published := cellChordPatchNormalLower(vLo, vHi, wLo, wHi)
		t.Logf("twist=%4.1fdeg: published area-element lower bound=%.9g sampled minimum=%.9g", deg, published, sampled)
		require.Greater(t, published, 0.0, "this cell's own four corner normals do fit in one half space")
		// The slack is the SAMPLED reference's own evaluation error, not the
		// bound's: each sample is a float cross-product norm, good to a few
		// ulps, while the published value is exact-rational and rounded down
		// once. A real overstatement is orders of magnitude away from this
		// window, so it stays a falsifier.
		require.LessOrEqual(t, published, sampled*(1+referenceUlpSlack),
			"the published area-element lower bound must not exceed the sampled minimum at twist %.1f deg", deg)
	}
}

// TestCellChordPatchNormalLowerRefusesADegenerateCell pins the refusal arm: a
// cell whose four corner normals do NOT fit in one open half space has no
// positive area-element bound to publish, and this reduction answers 0 rather
// than a number its own premise does not support. cellChordCurveAreaAllow then
// falls back to its premise-free arm, which is what the next test checks.
func TestCellChordPatchNormalLowerRefusesADegenerateCell(t *testing.T) {
	// A cell whose rung reverses across the cell: wLo sits one way off the
	// section chord and wHi the other, so the patch folds through zero area
	// somewhere inside it.
	vLo := r3.NewVec(0, 0, 0)
	vHi := r3.NewVec(1, 0, 0)
	wLo := r3.NewVec(0, 1, 0)
	wHi := r3.NewVec(1, -1, 0)
	require.Equal(t, 0.0, cellChordPatchNormalLower(vLo, vHi, wLo, wHi),
		"a cell whose chord patch folds has no positive area-element lower bound")
	require.Equal(t, 0.0, cellChordPatchNormalLower(
		r3.NewVec(math.NaN(), 0, 0), vHi, wLo, wHi),
		"a non-finite corner is a refusal, never a NaN a comparison would drop")
}

// TestCellChordCurveAreaAllowFallsBackOnADegenerateCell is the premise-free
// arm's own falsifier: on the folded cell above the sharp arm has no area
// element to divide by, so the published bound is the premise-free ceiling
// alone — and it must still enclose the directly integrated gap.
func TestCellChordCurveAreaAllowFallsBackOnADegenerateCell(t *testing.T) {
	// Two coplanar arcs whose rung reverses direction across the cell.
	lo := ruledArc{centre: r3.NewVec(0, 0, 0), radius: 2, u: r3.NewVec(1, 0, 0), v: r3.NewVec(0, 1, 0), t0: 0.2, dt: 0.5}
	hi := ruledArc{centre: r3.NewVec(0, 0, 0), radius: 2, u: r3.NewVec(1, 0, 0), v: r3.NewVec(0, 1, 0), t0: 0.2 + 0.9, dt: -1.4}
	vLo, vHi, wLo, wHi := lo.at(0), lo.at(1), hi.at(0), hi.at(1)
	require.Equal(t, 0.0, cellChordPatchNormalLower(vLo, vHi, wLo, wHi),
		"this fixture must actually reach the premise-free arm")
	gap := math.Abs(convergedRuledGap(t, lo, hi))
	allow := cellAllowFor(lo, hi)
	t.Logf("degenerate-normal cell: gap=%.6e allow=%.6e", gap, allow)
	require.LessOrEqual(t, gap, allow,
		"the premise-free arm alone must enclose the gap where the sharp arm has no area element")
}

// TestCellChordCurveAreaAllowRefusesBrokenClaims pins the +Inf answers of the
// ruled leg itself: a broken caller claim must never publish a shrunken bound.
func TestCellChordCurveAreaAllowRefusesBrokenClaims(t *testing.T) {
	vLo, vHi := r3.NewVec(0, 0, 0), r3.NewVec(1, 0, 0)
	wLo, wHi := r3.NewVec(0, 0, 1), r3.NewVec(1, 0, 1)
	bad := math.NaN()
	cases := map[string]func() float64{
		"non-finite corner": func() float64 {
			return cellChordCurveAreaAllow(r3.NewVec(bad, 0, 0), vHi, wLo, wHi, 1, 1, 0, 0, 0)
		},
		"non-finite arc length": func() float64 {
			return cellChordCurveAreaAllow(vLo, vHi, wLo, wHi, math.Inf(1), 1, 0, 0, 0)
		},
		"negative matched delta": func() float64 {
			return cellChordCurveAreaAllow(vLo, vHi, wLo, wHi, 1, 1, -1, 0, 0)
		},
		"NaN energy": func() float64 {
			return cellChordCurveAreaAllow(vLo, vHi, wLo, wHi, 1, 1, 0, bad, 0)
		},
		"negative energy": func() float64 {
			return cellChordCurveAreaAllow(vLo, vHi, wLo, wHi, 1, 1, 0, 0, -1)
		},
		"arc length below its own chord": func() float64 {
			return cellChordCurveAreaAllow(vLo, vHi, wLo, wHi, 0.5, 1, 0, 0, 0)
		},
	}
	for name, run := range cases {
		require.True(t, math.IsInf(run(), 1), "%s must answer +Inf", name)
	}
	require.Equal(t, 0.0, cellChordCurveAreaAllow(vLo, vHi, wLo, wHi, 1, 1, 0, 0, 0),
		"an exactly straight cell has no ruled-versus-chord gap to charge")
}

// TestLoftCertifiedChordLowerIsALowerBound pins perCellTangentEnergy's own
// chord reading over a sweep of whole-circle records: the published value must
// never exceed the true cell chord 2*R*sin(sweep/(2m)), which is what keeps
// the energy it feeds from being understated.
func TestLoftCertifiedChordLowerIsALowerBound(t *testing.T) {
	const radius = 2.75
	for _, turn := range []float64{0.01, 0.25, 0.5, 1} {
		for _, m := range []int{1, 3, 64, 1024} {
			seg := CircleSeg{
				Center: Point2{U: 1.5, V: -0.5},
				Radius: units.Millimeters(radius),
				CCW:    true,
				TStart: 0,
				TEnd:   turn,
			}
			got := loftCertifiedChordLower(seg, m)
			sweep := 2 * math.Pi * turn
			want := 2 * radius * math.Sin(sweep/(2*float64(m)))
			// math.Sin is itself accurate only to about an ulp, so the
			// reference — not the bound — carries the slack; the certified
			// enclosure below it is a genuine lower bound on the true chord.
			require.LessOrEqual(t, got, want*(1+referenceUlpSlack),
				"the published chord lower bound must not exceed the true chord (turn %.4g, m=%d)", turn, m)
			if want > 1e-6 {
				// A whole turn at m = 1 is the one cell whose two stations
				// COINCIDE, so its true chord is 0 and the published bound is
				// too; every other row must carry a genuinely positive bound,
				// or the energy it feeds would collapse to the arc length.
				require.Greater(t, got, 0.0,
					"a cell of a positive chord must carry a positive chord bound (turn %.4g, m=%d)", turn, m)
			}
		}
	}
	zero := CircleSeg{Center: Point2{}, Radius: units.Millimeters(1), CCW: true, TStart: 0.25, TEnd: 0.25}
	require.Equal(t, 0.0, loftCertifiedChordLower(zero, 4), "a zero sweep has no chord to bound")
	require.Equal(t, 0.0, loftCertifiedChordLower(LineSeg{}, 4), "a record with no circular enclosure publishes the empty bound")
	require.Equal(t, 0.0, loftCertifiedChordLower(zero, 0), "a non-positive station count has no cell to bound")
}

// TestLoftCertifiedChordLowerRefusesTheHeldWalkFloats is the regression witness
// for the defect a certified reading repairs: a chord "lower bound" read off
// the walk's own held math.Hypot radius and math.Atan2 endpoint angles can land
// ABOVE the true chord, because a math.Atan2 endpoint's own error is amplified
// by 1/sweep on a short arc while the roundings applied to the held floats
// cover about one ulp. uniformSpeedTangentEnergyUpper DECREASES in its chord
// operand, so such an overstatement understates the energy — and
// cellChordCurveAreaAllow's sharp arm increases in the energy, so the published
// Area allowance comes out too small. A public caller reaches this with an
// unplaced loft over two untrimmed ArcSeg profiles at m = 1.
//
// The arc below is a Pythagorean rotation of its own start point about the
// origin: Start and End are EXACTLY equidistant from Center (asserted, not
// assumed), so the recorded arc's own t=1 point IS End and the true m=1 chord
// is exactly |End − Start|, which ratSqrtUp brackets from above with no
// trigonometry in the reference at all. A held-float reading of this record
// answers about 5592.1318893204816 against a true 5592.1318893050811 — roughly
// 1.5e-8 too large, some four orders of magnitude past this record's own ulp.
func TestLoftCertifiedChordLowerRefusesTheHeldWalkFloats(t *testing.T) {
	seg := ArcSeg{
		Center: Point2{U: 0, V: 0},
		Start:  Point2{U: 20724598.671875, V: 50331168.203125},
		End:    Point2{U: 20719427.640625, V: 50333297.140625},
		TStart: 0,
		TEnd:   1,
	}

	r2Start := new(big.Rat).Add(
		new(big.Rat).Mul(floatRat(seg.Start.U), floatRat(seg.Start.U)),
		new(big.Rat).Mul(floatRat(seg.Start.V), floatRat(seg.Start.V)),
	)
	r2End := new(big.Rat).Add(
		new(big.Rat).Mul(floatRat(seg.End.U), floatRat(seg.End.U)),
		new(big.Rat).Mul(floatRat(seg.End.V), floatRat(seg.End.V)),
	)
	require.Zero(t, r2Start.Cmp(r2End),
		"the fixture's two endpoints must be exactly equidistant from Center, or |End-Start| is not the true chord")

	du := new(big.Rat).Sub(floatRat(seg.End.U), floatRat(seg.Start.U))
	dv := new(big.Rat).Sub(floatRat(seg.End.V), floatRat(seg.Start.V))
	chord2 := new(big.Rat).Add(new(big.Rat).Mul(du, du), new(big.Rat).Mul(dv, dv))
	trueChordUpper := ratSqrtUp(chord2)

	got := loftCertifiedChordLower(seg, 1)
	require.Greater(t, got, 0.0, "the fixture must reach a real bound rather than the empty one")
	require.LessOrEqual(t, got, trueChordUpper,
		"the published chord lower bound must not exceed the true chord |End-Start|")

	// The energy that bound feeds must stay an UPPER bound on L^2-c^2: with
	// the chord overstated it fell BELOW the true energy on this record.
	w := segmentWalk{kind: walkCircular}
	arcUpper := perCellArcUpper(seg, w, 1)
	energy := perCellTangentEnergy(seg, w, 1)
	require.Greater(t, energy, 0.0, "a curved cell carries a positive tangent-deviation energy")
	require.GreaterOrEqual(t, energy, (arcUpper-trueChordUpper)*(arcUpper+trueChordUpper),
		"the published energy must dominate the L^2-c^2 reading its own arc-length bound and the true chord give")
}

// rotatedWedgeOffset, rotatedWedgeRadius and rotatedWedgeHeight fix this
// file's own end-to-end shape: a quarter-arc wedge whose apex sits
// rotatedWedgeOffset away from the rotation axis, so NO point of the profile is
// near that axis and a rotated pairing's two wall fans never fold through each
// other. It is deliberately wall-dominated (a tall, thin shape), so the wall
// term is load-bearing rather than masked by the caps.
const (
	rotatedWedgeOffset = 4.0
	rotatedWedgeRadius = 1.0
	rotatedWedgeHeight = 60.0
)

// rotatedWedgeSketch draws the wedge outline rotated by phi about the sketch
// origin: apex, one radial line out, the quarter arc, one radial line back.
func rotatedWedgeSketch(t *testing.T, w *sketch.World, plane *sketch.Plane, phi float64) (*sketch.Sketch, *sketch.Profile) {
	t.Helper()
	rot := func(x, y float64) (float64, float64) {
		sin, cos := math.Sincos(phi)
		return x*cos - y*sin, x*sin + y*cos
	}
	s, err := w.CreateSketch(plane)
	require.NoError(t, err)
	ax, ay := rot(rotatedWedgeOffset, 0)
	apex := s.CreatePoint(ax, ay)
	s.Fix(apex)
	pxx, pxy := rot(rotatedWedgeOffset+rotatedWedgeRadius, 0)
	pyx, pyy := rot(rotatedWedgeOffset, rotatedWedgeRadius)
	px := s.CreatePoint(pxx, pxy)
	py := s.CreatePoint(pyx, pyy)
	s.CreateLine(apex, px)
	s.CreateLine(py, apex)
	s.CreateArc(apex, px, py)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	profiles := s.Profiles()
	require.Len(t, profiles, 1)
	return s, profiles[0]
}

// rotatedWedgeRing is the wedge outline as m+2 plain 2D points — apex, then the
// m+1 arc stations — rotated by phi about the origin.
func rotatedWedgeRing(m int, phi float64) [][2]float64 {
	rot := func(x, y float64) [2]float64 {
		sin, cos := math.Sincos(phi)
		return [2]float64{x*cos - y*sin, x*sin + y*cos}
	}
	ring := make([][2]float64, 0, m+2)
	ring = append(ring, rot(rotatedWedgeOffset, 0))
	for k := 0; k <= m; k++ {
		th := (math.Pi / 2) * float64(k) / float64(m)
		ring = append(ring, rot(rotatedWedgeOffset+rotatedWedgeRadius*math.Cos(th), rotatedWedgeRadius*math.Sin(th)))
	}
	return ring
}

// denseRotatedWedgeArea sums the SAME Table B wall split and cap shoelace a
// hand-chorded loft of these two rings would publish, in plain float64 and with
// no sketch, Document.Loft or evaluator call of any kind — the independent
// reference the end-to-end tests below converge.
func denseRotatedWedgeArea(m int, phi float64) float64 {
	bot := rotatedWedgeRing(m, 0)
	top := rotatedWedgeRing(m, phi)
	n := len(bot)
	wall := 0.0
	for j := range n {
		jn := (j + 1) % n
		vLo := r3.NewVec(bot[j][0], bot[j][1], 0)
		vHi := r3.NewVec(bot[jn][0], bot[jn][1], 0)
		wLo := r3.NewVec(top[j][0], top[j][1], rotatedWedgeHeight)
		wHi := r3.NewVec(top[jn][0], top[jn][1], rotatedWedgeHeight)
		wall += triArea3(vLo, vHi, wHi) + triArea3(vLo, wHi, wLo)
	}
	return wall + shoelace2DAbs(bot) + shoelace2DAbs(top)
}

// convergedRotatedWedgeArea sweeps denseRotatedWedgeArea to convergence, judged
// entirely on the REFERENCE's own successive agreement at a fixed relative
// tolerance — never on the bound the caller is checking.
func convergedRotatedWedgeArea(t *testing.T, phi float64) float64 {
	t.Helper()
	const relTol = 1e-9
	prev, havePrev := 0.0, false
	for _, m := range []int{256, 512, 1024, 2048, 4096, 8192, 16384, 32768} {
		cur := denseRotatedWedgeArea(m, phi)
		if havePrev && math.Abs(cur-prev) <= relTol*math.Abs(cur) {
			t.Logf("dense rotated-wedge reference converged at m=%d: area=%.15g", m, cur)
			return cur
		}
		prev, havePrev = cur, true
	}
	t.Fatalf("the dense rotated-wedge area reference did not converge to relative %.3g", relTol)
	return 0
}

// TestLoftRotatedWedgeAreaBoundEnclosesConvergedReference is the end-to-end
// twist fixture: at every angle of loftTwistSweepDegrees the real
// Document.Loft/Body.Area() reading must enclose the converged dense reference.
// Every nonzero angle carries genuine twist — the two sections' chords are not
// parallel, so each cell's own T is nonzero — which is exactly what the
// arc-minus-chord length excess this leg replaced could not see.
func TestLoftRotatedWedgeAreaBoundEnclosesConvergedReference(t *testing.T) {
	for _, deg := range loftTwistSweepDegrees {
		phi := deg * math.Pi / 180
		w, base, top := wedgePlanesH(t, rotatedWedgeHeight)
		s0, p0 := rotatedWedgeSketch(t, w, base, 0)
		s1, p1 := rotatedWedgeSketch(t, w, top, phi)
		doc := New()
		body, err := doc.Loft(s0, p0, s1, p1)
		require.NoError(t, err, "the rotated wedge must build at %.1f deg", deg)
		area, err := body.Area()
		require.NoError(t, err)

		ref := convergedRotatedWedgeArea(t, phi)
		residual := math.Abs(area.Value.Base() - ref)
		t.Logf("rotated wedge %4.1fdeg: value=%.10g bound=%.6e ref=%.10g residual=%.6e",
			deg, area.Value.Base(), area.Bound.Base(), ref, residual)
		require.LessOrEqual(t, residual, area.Bound.Base(),
			"the loft's own Area must enclose the converged densely-chorded reference at %.1f deg of twist", deg)
	}
}

// A NOTE ON THE TWO REFERENCE KINDS. A densely chorded triangle sum converges
// to the denoted ruled surface only where the cells' own warp vanishes with the
// chord — true for an untwisted or purely translated pairing, false under a
// ROTATIONAL correspondence, where |T|/chord stays fixed as both shrink and the
// sum converges to a Schwarz-lantern limit strictly above the ruled area. So
// the rotated wedge is checked BOTH ways: against the dense sum, which pins the
// polyhedral reading, and against rotatedWedgeDenotedArea, the directly
// integrated ruled surface, which is the quantity the wall term is actually
// charged for and the one the twist leg's own falsifier turns on.
//
// A NOTE ON VERDICTS. The carried-over twist leg comes to roughly 2*sin(phi/2)
// times the wall area once summed over a loop, so a curved-section loft under
// real rotation cannot read Sound on Area at the default tolerance — measured
// on this fixture, it over-charges the true held-triangles-versus-bilinear gap
// by 23x at 30 degrees, 46x at 15 and 138x at 5. Tightening it is its own
// mechanism and its own change; nothing here depends on it.

// tiltedCellPair is twistedArcCellPair with the upper arc's own plane tilted
// out of the section plane by tilt radians about its u axis — the family where
// the sharp arm's oscillation term carries most of the bound.
func tiltedCellPair(phi, offCentre, radius, height, t0, dt, tilt float64) (ruledArc, ruledArc) {
	lo, hi := twistedArcCellPair(phi, offCentre, radius, height, t0, dt)
	sin, cos := math.Sincos(tilt)
	hi.v = hi.v.Scale(cos).Add(r3.NewVec(0, 0, 1).Scale(sin))
	return lo, hi
}

// oppositeBulgeCellPair reflects the lower arc about its OWN chord and lifts the
// result, so the two sides bulge in OPPOSITE directions: the family where
// f = (b − b0) − (a − a0) really does reach twice the matched delta, which is
// what the sharp arm's matched-delta term is charged for. skew slides the upper
// arc along x so the cell is not a mirror pair.
func oppositeBulgeCellPair(radius, height, t0, dt, skew float64) (ruledArc, ruledArc) {
	lo := ruledArc{centre: r3.NewVec(0, 0, 0), radius: radius, u: r3.NewVec(1, 0, 0), v: r3.NewVec(0, 1, 0), t0: t0, dt: dt}
	p0, p1 := lo.at(0), lo.at(1)
	axis := p1.Sub(p0)
	axis = axis.Scale(1 / axis.Len())
	refl := func(v r3.Vec) r3.Vec { return axis.Scale(2 * v.Dot(axis)).Sub(v) }
	hi := ruledArc{
		centre: p0.Add(refl(lo.centre.Sub(p0))).Add(r3.NewVec(skew, 0, height)),
		radius: radius,
		u:      refl(lo.u),
		v:      refl(lo.v),
		t0:     t0, dt: dt,
	}
	return lo, hi
}

// TestCellChordCurveAreaAllowEnclosesOppositeBulgeGap is the matched-delta
// term's own enclosure falsifier: on a cell whose two sides bulge in opposite
// directions the sharp arm's 2*md*(cMax+I) term carries most of the bound, and
// removing it drops the published value below the directly integrated gap.
func TestCellChordCurveAreaAllowEnclosesOppositeBulgeGap(t *testing.T) {
	lo, hi := oppositeBulgeCellPair(15, 0.01, 0.3, 0.05, 1)
	gap := math.Abs(convergedRuledGap(t, lo, hi))
	allow := cellAllowFor(lo, hi)
	t.Logf("opposite-bulge cell: gap=%.6e allow=%.6e ratio=%.4f", gap, allow, gap/allow)
	require.LessOrEqual(t, gap, allow,
		"the ruled leg must enclose the directly integrated gap on a cell whose two sides bulge apart")
}

// recomposeCellAllow rebuilds cellChordCurveAreaAllow's own published value from
// its four derivation terms, INDEPENDENTLY of that function, so a term dropped
// from the production composition shows up as a mismatch. It is the falsifier
// for the two terms no enclosure fixture can bind (see the table below).
func recomposeCellAllow(a, b ruledArc) (float64, float64, float64, float64, float64) {
	vLo, vHi := a.at(0), a.at(1)
	wLo, wHi := b.at(0), b.at(1)
	arcA, arcB := upRound(a.arcLen()), upRound(b.arcLen())
	md := math.Max(a.sagittaUpper(), b.sagittaUpper())
	chordLo := func(c ruledArc) float64 {
		return downRound(downRound(2 * math.Abs(c.radius) * math.Sin(math.Abs(c.dt)/2)))
	}
	energyA := uniformSpeedTangentEnergyUpper(arcA, chordLo(a))
	energyB := uniformSpeedTangentEnergyUpper(arcB, chordLo(b))

	da, db := heldDelta(vHi, vLo), heldDelta(wHi, wLo)
	ca, cb := rvLenUpper(da), rvLenUpper(db)
	eB := math.Max(rvLenUpper(heldDelta(wLo, vLo)), rvLenUpper(heldDelta(wHi, vHi)))
	cMax := math.Max(ca, cb)
	ia, ja := tangentDeviationUpper(arcA, ca, energyA)
	ib, jb := tangentDeviationUpper(arcB, cb, energyB)
	iMax := math.Max(ia, ib)
	beta := absSumUpper(eB, productUpper(2, md))
	gamma := productUpper(productUpper(2, md), cMax)

	free := absSumUpper(upRound(productUpper(beta, absSumUpper(ia, ib))/2), gamma)
	nMin := cellChordPatchNormalLower(vLo, vHi, wLo, wHi)
	if nMin <= 0 {
		return free, free, 0, 0, 0
	}
	twist := rvSub(heldDelta(vLo, vHi), heldDelta(wLo, wHi))
	pCrossT := math.Max(rvLenUpper(rvCross(da, twist)), rvLenUpper(rvCross(db, twist)))
	oscW := absSumUpper(rvLenUpper(twist), upRound(productUpper(eB, pCrossT)/nMin))
	oscTerm := productUpper(oscW, iMax)
	mdTerm := productUpper(productUpper(2, md), absSumUpper(cMax, iMax))
	quad := upRound(absSumUpper(
		productUpper(productUpper(beta, beta), absSumUpper(ja, jb)),
		productUpper(2, productUpper(gamma, gamma)),
	) / (2 * nMin))
	return math.Min(free, absSumUpper(absSumUpper(oscTerm, mdTerm), quad)), free, oscTerm, mdTerm, quad
}

// TestCellChordCurveAreaAllowComposesEveryTerm is the falsifier for the sharp
// arm's OSCILLATION term, which no enclosure fixture can bind: every geometry
// family searched — twisted arc pairs, tilted pairs, coaxial frustum cells,
// parameter-shifted coaxial cells, opposite-bulge cells, and each of those
// under a further rotation — either has the matched-delta or quadratic term
// dominating it, or has the premise-free ceiling below the whole sharp arm, so
// the published minimum never turns on it. It is NOT redundant: it bounds the
// mean-zero part of the surface-normal difference, a genuinely nonzero
// contribution the derivation cannot drop. So it is falsified structurally
// instead — this test rebuilds the published value from the four terms
// independently and compares, and each row names the term that carries it.
//
// L3 of this file's falsification ledger carries the measurement behind "no
// enclosure fixture can bind it" — the searched cell count, the worst gap-to-
// stripped-bound ratio, and the four subtests deleting the term does redden.
func TestCellChordCurveAreaAllowComposesEveryTerm(t *testing.T) {
	type row struct {
		name    string
		lo, hi  ruledArc
		carries string
	}
	oscLo, oscHi := tiltedCellPair(5*math.Pi/180, 4, 1.5, 0.10, 0.3, 0.02, 1.5)
	mdLo, mdHi := oppositeBulgeCellPair(15, 0.01, 0.3, 0.05, 1)
	quadLo, quadHi := twistedArcCellPair(0, 4, 0.05, 60, 0.3, 0.02)
	freeLo, freeHi := tiltedCellPair(15*math.Pi/180, 4, 12, 0.5, 0.3, 0.02, 1.0)
	for _, r := range []row{
		{"oscillation-carried", oscLo, oscHi, "osc"},
		{"matched-delta-carried", mdLo, mdHi, "md"},
		{"quadratic-carried", quadLo, quadHi, "quad"},
		{"ceiling-carried", freeLo, freeHi, "free"},
	} {
		t.Run(r.name, func(t *testing.T) {
			want, free, osc, md, quad := recomposeCellAllow(r.lo, r.hi)
			got := cellAllowFor(r.lo, r.hi)
			sharp := absSumUpper(absSumUpper(osc, md), quad)
			t.Logf("%s: published=%.9e free=%.9e osc=%.9e md=%.9e quad=%.9e", r.name, got, free, osc, md, quad)
			require.InEpsilon(t, want, got, 1e-13,
				"the published bound must be the four derivation terms composed as min(ceiling, osc+md+quad)")
			require.Greater(t, cellChordPatchNormalLower(r.lo.at(0), r.lo.at(1), r.hi.at(0), r.hi.at(1)), 0.0,
				"every row must reach the sharp arm, so the minimum below is a real choice")
			switch r.carries {
			case "free":
				require.Less(t, free, sharp, "this row must be carried by the premise-free ceiling")
			default:
				require.Less(t, sharp, free, "this row must be carried by the sharp arm")
				share := map[string]float64{"osc": osc, "md": md, "quad": quad}[r.carries]
				require.Greater(t, share, sharp/2,
					"the %s term must carry more than half of this row's sharp arm, or dropping it would not show", r.carries)
			}
		})
	}
}

// sideCurve is one directrix of a ruled patch, read as a position and a
// derivative under the SAME shared parameter the loft's own correspondence
// pairs the two sections by.
type sideCurve interface {
	at(s float64) r3.Vec
	der(s float64) r3.Vec
}

// ruledLine is the straight directrix a LineSeg pairing contributes, traversed
// linearly in the shared parameter exactly as the loft's own correspondence
// does.
type ruledLine struct{ p0, p1 r3.Vec }

func (l ruledLine) at(s float64) r3.Vec { return l.p0.Add(l.p1.Sub(l.p0).Scale(s)) }
func (l ruledLine) der(float64) r3.Vec  { return l.p1.Sub(l.p0) }

// ruledPatchArea integrates |X_s × X_r| over the unit square for the ruled
// patch X(s,r) = (1−r)*a(s) + r*b(s) — the surface the loft's construction
// DENOTES between two paired curves. It is a plain numerical integral: no
// sketch, no Document.Loft, no evaluator, and no chord chain of any kind.
//
// This is the reference a genuinely TWISTED circular pairing needs. A densely
// chorded triangle sum does NOT converge to it there: under a rotational
// correspondence each curved cell's own warp |T|/chord stays fixed as the
// chord count grows (both shrink together), so the triangle sum converges to a
// Schwarz-lantern limit strictly above the ruled surface's own area. That
// limit is what loft_area_excess_fixture_internal_test.go's own dense reference
// measures, and it is the right reference for an untwisted pairing, where the
// warp does vanish with the chord — but it understates the gap the curved wall
// term is charged for whenever the correspondence really rotates.
func ruledPatchArea(a, b sideCurve, panels int) float64 {
	// Five-point Gauss-Legendre per panel in each direction. The integrand is
	// analytic wherever the patch is non-degenerate, so this converges far
	// faster than a midpoint rule and reaches a reference accurate enough that
	// the fixture's residual is never in doubt.
	nodes := [5]float64{-0.9061798459386640, -0.5384693101056831, 0, 0.5384693101056831, 0.9061798459386640}
	weights := [5]float64{0.2369268850561891, 0.4786286704993665, 0.5688888888888889, 0.4786286704993665, 0.2369268850561891}
	h := 1 / float64(panels)
	sum := 0.0
	for pi := range panels {
		for si, sn := range nodes {
			s := (float64(pi) + 0.5*(sn+1)) * h
			g := b.at(s).Sub(a.at(s))
			aDer, bDer := a.der(s), b.der(s)
			inner := 0.0
			for pj := range panels {
				for ri, rn := range nodes {
					r := (float64(pj) + 0.5*(rn+1)) * h
					inner += weights[ri] * aDer.Scale(1-r).Add(bDer.Scale(r)).Cross(g).Len()
				}
			}
			sum += weights[si] * inner
		}
	}
	return sum * h * h / 4
}

// rotatedWedgeDenotedArea is the rotated wedge's own DENOTED area at the given
// panel count: the circular pair's ruled wall patch, the two LineSeg pairs'
// held triangle pairs, and the two exact circular-sector caps. Section 5 makes
// each LineSeg wall the held triangle pair itself, so only a CHORDED circular
// cell has a bilinear-to-triangle gap. The reference is independent of the
// production evaluator entirely.
func rotatedWedgeDenotedArea(phi float64, n int) float64 {
	rot := func(v r3.Vec) r3.Vec {
		sin, cos := math.Sincos(phi)
		return r3.NewVec(v.X*cos-v.Y*sin, v.X*sin+v.Y*cos, v.Z)
	}
	lift := func(v r3.Vec) r3.Vec { return rot(v).Add(r3.NewVec(0, 0, rotatedWedgeHeight)) }
	apex := r3.NewVec(rotatedWedgeOffset, 0, 0)
	px := r3.NewVec(rotatedWedgeOffset+rotatedWedgeRadius, 0, 0)
	py := r3.NewVec(rotatedWedgeOffset, rotatedWedgeRadius, 0)
	arcLo := ruledArc{
		centre: apex, radius: rotatedWedgeRadius,
		u: r3.NewVec(1, 0, 0), v: r3.NewVec(0, 1, 0),
		t0: 0, dt: math.Pi / 2,
	}
	arcHi := ruledArc{
		centre: lift(apex), radius: rotatedWedgeRadius,
		u: rot(r3.NewVec(1, 0, 0)), v: rot(r3.NewVec(0, 1, 0)),
		t0: 0, dt: math.Pi / 2,
	}
	heldLineArea := func(a0, a1, b0, b1 r3.Vec) float64 {
		return triArea3(a0, a1, b1) + triArea3(a0, b1, b0)
	}
	wall := heldLineArea(apex, px, lift(apex), lift(px)) +
		ruledPatchArea(arcLo, arcHi, n) +
		heldLineArea(py, apex, lift(py), lift(apex))
	caps := 2 * (math.Pi * rotatedWedgeRadius * rotatedWedgeRadius / 4)
	return wall + caps
}

// convergedRotatedWedgeDenotedArea sweeps the quadrature to convergence, judged
// on the REFERENCE's own successive agreement at a fixed relative tolerance and
// on nothing the bound under test can move.
func convergedRotatedWedgeDenotedArea(t *testing.T, phi float64) float64 {
	t.Helper()
	const relTol = 1e-10
	prev, havePrev := 0.0, false
	for _, n := range []int{2, 4, 8, 16, 32} {
		cur := rotatedWedgeDenotedArea(phi, n)
		if havePrev && math.Abs(cur-prev) <= relTol*math.Abs(cur) {
			t.Logf("denoted rotated-wedge reference converged at n=%d: area=%.15g", n, cur)
			return cur
		}
		prev, havePrev = cur, true
	}
	t.Fatalf("the directly integrated rotated-wedge denoted area did not converge to relative %.3g", relTol)
	return 0
}

// TestLoftRotatedWedgeAreaBoundEnclosesDenotedSurface is the end-to-end
// falsifier for BOTH legs of the curved wall term under real twist: the
// published Area must enclose the circular pair's DENOTED ruled surface, with
// each LineSeg pair read as the held triangle pair §5 defines. At every
// nonzero angle the circular cells' chords are genuinely non-parallel, so each
// curved cell's twist vector is nonzero and the twist leg is load-bearing here.
func TestLoftRotatedWedgeAreaBoundEnclosesDenotedSurface(t *testing.T) {
	for _, deg := range loftTwistSweepDegrees {
		phi := deg * math.Pi / 180
		w, base, top := wedgePlanesH(t, rotatedWedgeHeight)
		s0, p0 := rotatedWedgeSketch(t, w, base, 0)
		s1, p1 := rotatedWedgeSketch(t, w, top, phi)
		doc := New()
		body, err := doc.Loft(s0, p0, s1, p1)
		require.NoError(t, err, "the rotated wedge must build at %.1f deg", deg)
		area, err := body.Area()
		require.NoError(t, err)

		ref := convergedRotatedWedgeDenotedArea(t, phi)
		residual := math.Abs(area.Value.Base() - ref)
		t.Logf("rotated wedge %4.1fdeg denoted: value=%.10g bound=%.6e ref=%.10g residual=%.6e ratio=%.4f",
			deg, area.Value.Base(), area.Bound.Base(), ref, residual, residual/area.Bound.Base())
		require.LessOrEqual(t, residual, area.Bound.Base(),
			"the loft's own Area must enclose the denoted ruled surface at %.1f deg of twist", deg)
	}
}

// ruledDirectrix is one side of a ruled patch: a point and a tangent under the
// shared parameter s in [0,1]. ruledArc is one, and so is the shiftedArc below
// that carries a station displacement, which is what lets the composition
// fixture integrate the SAME patch area over the denoted stations and over the
// held corners with one integrator.
type ruledDirectrix interface {
	at(s float64) r3.Vec
	der(s float64) r3.Vec
}

// shiftedArc is a directrix displaced from the stations it denotes onto the
// corners a build HOLDS: a(s) = arc(s) + (1−s)·p + s·q, the affine correction
// cellChordCurveAreaAllow's own composition section writes, with p and q the
// two corner displacements. Its derivative is arc'(s) + (q−p) exactly, so the
// integrator below reads the held patch with no resampling of its own.
type shiftedArc struct {
	arc  ruledArc
	p, q r3.Vec
}

func (a shiftedArc) at(s float64) r3.Vec {
	return a.arc.at(s).Add(a.p.Scale(1 - s)).Add(a.q.Scale(s))
}

func (a shiftedArc) der(s float64) r3.Vec { return a.arc.der(s).Add(a.q.Sub(a.p)) }

// heldShiftFrom builds the shiftedArc whose two endpoints ARE the held corners
// lo and hi, from the denoted directrix they stand for.
func heldShiftFrom(arc ruledArc, lo, hi r3.Vec) shiftedArc {
	return shiftedArc{arc: arc, p: lo.Sub(arc.at(0)), q: hi.Sub(arc.at(1))}
}

// ruledPatchAreaAbs integrates |X_s × X_r| over the unit square for the ruled
// patch between two directrices, by the midpoint rule at n² nodes. It is the
// absolute-area twin of ruledPatchAreaGap above, needed because the
// composition fixture below compares the true surface against a HELD TRIANGLE
// pair rather than against another patch, so there is no second integrand to
// difference node by node.
func ruledPatchAreaAbs(a, b ruledDirectrix, n int) float64 {
	h := 1 / float64(n)
	sum := 0.0
	for i := range n {
		s := (float64(i) + 0.5) * h
		g := b.at(s).Sub(a.at(s))
		aDer, bDer := a.der(s), b.der(s)
		for j := range n {
			r := (float64(j) + 0.5) * h
			sum += aDer.Scale(1 - r).Add(bDer.Scale(r)).Cross(g).Len()
		}
	}
	return sum * h * h
}

// convergedRuledArea sweeps ruledPatchAreaAbs at increasing resolutions until
// two SUCCESSIVE resolutions agree to within tol ABSOLUTE. The caller passes a
// tolerance read from the geometry it is about to measure and never from the
// bound under test, so no leg of that bound can move this gate.
func convergedRuledArea(t *testing.T, a, b ruledDirectrix, tol float64) float64 {
	t.Helper()
	prev, havePrev := 0.0, false
	for _, n := range []int{64, 128, 256, 512, 1024} {
		cur := ruledPatchAreaAbs(a, b, n)
		if havePrev && math.Abs(cur-prev) <= tol {
			return cur
		}
		prev, havePrev = cur, true
	}
	t.Fatalf("the directly integrated ruled-patch area did not converge to absolute %.3g", tol)
	return 0
}

// pullStationsInward displaces a cell's four DENOTED stations into the HELD
// corners a build would hold, ADVERSARIALLY: both ends of each side are pulled
// INWARD along that side's own chord, so each held chord is shorter than the
// denoted one by exactly 2*delta. That is the direction that maximises the
// mean-zero violation cellChordCurveAreaAllow's own derivation is scoped away
// from — its two patches are pinned at the corners it is handed, and this is
// the displacement the THIRD leg has to carry.
func pullStationsInward(a, b ruledArc, delta float64) (r3.Vec, r3.Vec, r3.Vec, r3.Vec) {
	pull := func(lo, hi r3.Vec) (r3.Vec, r3.Vec) {
		u := hi.Sub(lo)
		u = u.Scale(1 / u.Len())
		return lo.Add(u.Scale(delta)), hi.Sub(u.Scale(delta))
	}
	vLo, vHi := pull(a.at(0), a.at(1))
	wLo, wHi := pull(b.at(0), b.at(1))
	return vLo, vHi, wLo, wHi
}

// heldTrianglePairArea is the area assembleLoft's own two wall triangles carry
// over one cell, in the Table B split loft_build.go builds: (vLo, vHi, wHi) and
// (vLo, wHi, wLo). It is the T_held term of the three-leg composition.
func heldTrianglePairArea(vLo, vHi, wLo, wHi r3.Vec) float64 {
	tri := func(a, b, c r3.Vec) float64 { return b.Sub(a).Cross(c.Sub(a)).Len() / 2 }
	return tri(vLo, vHi, wHi) + tri(vLo, wHi, wLo)
}

// TestDisplacedStationCellNeedsTheStationShiftLeg pins the THREE-LEG
// composition cellChordCurveAreaAllow's own doc comment states and area()
// spends: over a cell whose held corners are displaced from the stations they
// denote,
//
//	|A_true − T_held| <= ruled leg + twist leg + cellStationShiftAreaAllow,
//
// and the first two legs ALONE do not cover it. It asserts three things:
//
//   - the STEP the new leg is charged for — the area difference between the
//     ruled patch through the DENOTED stations and the one through the HELD
//     corners, both integrated directly — is covered by that leg alone. This is
//     the assertion the leg exists for, and it turns on the inequality rather
//     than on the whole composition's slack;
//   - the three legs together cover the whole gap to the held triangle pair;
//   - the first two legs alone do NOT, so the third stays load-bearing.
//
// The cells are UNTWISTED on purpose. Their four corners form a parallelogram,
// so the exact twist vector T is zero and cellTwistAreaAllow is a certified
// zero: what remains is the ruled leg against the station-shift leg, with no
// third term able to mask which one carries the gap. Every reference is a
// direct numerical integral of the surfaces themselves, converged well below
// the quantity it is differenced against, and shares no code with the bound.
func TestDisplacedStationCellNeedsTheStationShiftLeg(t *testing.T) {
	cases := []struct {
		name           string
		radius, height float64
		cells          int
		delta          float64
	}{
		{"r10 h100 over 256 cells", 10, 100, 256, 1e-4},
		{"r50 h200 over 512 cells", 50, 200, 512, 1e-4},
		{"r1.5 h12 over 63 cells", 1.5, 12, 63, 1e-4},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			dt := 2 * math.Pi / float64(tc.cells)
			lo, hi := twistedArcCellPair(0, 0, tc.radius, tc.height, 0.3, dt)
			vLo, vHi, wLo, wHi := pullStationsInward(lo, hi, tc.delta)

			arcA, arcB := upRound(lo.arcLen()), upRound(hi.arcLen())
			chordLo := func(a ruledArc) float64 {
				return downRound(downRound(2 * math.Abs(a.radius) * math.Sin(math.Abs(a.dt)/2)))
			}
			// The build's own per-cell obligations: chordCellDeltaUpper
			// composes the certified sagitta with the station displacement,
			// and perCellTangentEnergy's circular arm discharges the energy.
			md := chordCellDeltaUpper(math.Max(lo.sagittaUpper(), hi.sagittaUpper()), tc.delta)
			ruled := cellChordCurveAreaAllow(vLo, vHi, wLo, wHi, arcA, arcB, md,
				uniformSpeedTangentEnergyUpper(arcA, chordLo(lo)),
				uniformSpeedTangentEnergyUpper(arcB, chordLo(hi)))
			twist := cellTwistAreaAllow(vLo, vHi, wLo, wHi)
			require.Zero(t, twist, "an untwisted cell's four corners form a parallelogram, so the twist leg is a certified zero")
			station := cellStationShiftAreaAllow(vLo, vHi, wLo, wHi, arcA, arcB, md, tc.delta)

			// The step the station-shift leg is charged for, integrated
			// directly: the SAME patch area over the denoted directrices and
			// over the held ones, which differ by nothing but the affine
			// correction that lands each end on the corner the build holds.
			trueArea := convergedRuledArea(t, lo, hi, tc.delta*tc.delta)
			heldPatch := convergedRuledArea(t,
				heldShiftFrom(lo, vLo, vHi), heldShiftFrom(hi, wLo, wHi), tc.delta*tc.delta)
			surfaceStep := math.Abs(trueArea - heldPatch)

			held := heldTrianglePairArea(vLo, vHi, wLo, wHi)
			gap := math.Abs(trueArea - held)

			twoLegs := absSumUpper(ruled, twist)
			composed := absSumUpper(ruled, twist, station)
			t.Logf("%s: gap=%.6e surfaceStep=%.6e ruled=%.6e twist=%.6e station=%.6e twoLegs=%.6e composed=%.6e (step/station=%.4g gap/twoLegs=%.4g gap/composed=%.4g)",
				tc.name, gap, surfaceStep, ruled, twist, station, twoLegs, composed,
				surfaceStep/station, gap/twoLegs, gap/composed)

			require.LessOrEqual(t, surfaceStep, station,
				"the station-shift leg must cover the directly integrated area step from the held-corner ruled patch to the denoted one")
			require.Greater(t, gap, twoLegs,
				"the ruled and twist legs alone must NOT cover a displaced-station cell's gap — if they did, this fixture would stop pinning that the station-shift leg is load-bearing")
			require.LessOrEqual(t, gap, composed,
				"the three-leg composition must cover the directly integrated gap between the denoted ruled patch and the held triangle pair")
		})
	}
}
