package decad

import (
	"fmt"
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// This file tests bounds.go's chordedBoundaryVolumeAllow,
// chordedBoundaryMomentAllow, cellRuledExcessUpper and cellTwistVolumeAllow
// (docs/loft-design.md §5.2, the A10 plan's Part 2 Q4 and Part 4 R1
// fallback): the enclosure chordedBoundaryVolumeAllow proves between the
// HELD FLAT-TRIANGLE polyhedron assembleLoft actually builds and the true
// curved solid it approximates, over a table of radii, sweeps, heights,
// chord counts AND — the mechanism a previous version of this bound missed
// entirely — a TWIST angle between the two paired sections.
//
// A fixture with no twist cannot exercise cellTwistVolumeAllow: every wall
// cell degenerates to a planar quad, its own twist vector is exactly zero,
// and a bound that omits the twist term altogether still passes. So every
// table below carries a twist sweep, and the two rows the refuted version of
// this bound failed outright — 20 degrees of twist at 64 stations, and 90
// degrees at 256 stations — are asserted explicitly.

// twistedPieSliceMesh builds the watertight triangle mesh of a CHORDED
// circular-sector wedge whose top section is the bottom section's own arc
// ROTATED by twistRad about the Z axis before being straight-extruded to
// z=h: a center point, two straight radial sides, and n equal-angle arc
// chords per section. twistRad=0 reduces this to pieSliceChordMesh's own
// untwisted mesh (every wall cell's rule vertical, every twist vector zero).
func twistedPieSliceMesh(radius, sweepRad, twistRad, h float64, n int) (verts []r3.Vec, tris [][3]int) {
	const centerB, centerT = 0, 1
	arcPoint := func(i int, twist, z float64) r3.Vec {
		theta := twist + sweepRad*float64(i)/float64(n)
		return r3.NewVec(radius*math.Cos(theta), radius*math.Sin(theta), z)
	}
	verts = append(verts, r3.NewVec(0, 0, 0), r3.NewVec(0, 0, h))

	arcB := make([]int, n+1)
	arcT := make([]int, n+1)
	for i := range n + 1 {
		arcB[i] = len(verts)
		verts = append(verts, arcPoint(i, 0, 0))
		arcT[i] = len(verts)
		verts = append(verts, arcPoint(i, twistRad, h))
	}

	for i := range n {
		tris = append(tris, [3]int{centerB, arcB[i+1], arcB[i]})
		tris = append(tris, [3]int{centerT, arcT[i], arcT[i+1]})
		tris = append(tris, [3]int{arcB[i], arcB[i+1], arcT[i+1]})
		tris = append(tris, [3]int{arcB[i], arcT[i+1], arcT[i]})
	}
	tris = append(tris, [3]int{centerB, arcB[0], arcT[0]}, [3]int{centerB, arcT[0], centerT})
	tris = append(tris, [3]int{arcB[n], centerB, centerT}, [3]int{arcB[n], centerT, arcT[n]})

	return verts, tris
}

// twistedPieSliceTrueVolume is the CLOSED FORM for the true solid a twisted
// pie-slice loft denotes (docs/loft-design.md's own chord-to-curve homotopy,
// §5.2): the RULED surface between the bottom arc, at angle s over [0,
// sweepRad], and the top arc, at angle s+twistRad, linearly interpolated by
// height fraction f = z/h, plus the two flat sector caps and the two
// (excess-free, since they are single straight segments, not chorded) radial
// walls.
//
// By Cavalieri's principle, volume = h * integral over f of the
// cross-sectional area A(f). The two straight radial edges pass through the
// origin at every f (center-to-center is the z-axis exactly, degenerate in
// x/y), so they contribute nothing to the shoelace integral for A(f); the
// remaining arc-wall contribution reduces, using x*y' - y*x' = |X|^2 * dtheta
// (a standard identity for any planar curve X(s) in polar form), to a
// CONSTANT integrand in s because |[(1-f)+f*e^{i*twistRad}]|^2 does not
// depend on s (rotating a fixed-magnitude combination by e^{i*s} leaves its
// own magnitude alone):
//
//	A(f) = (radius^2 * sweepRad / 2) * ((1-f)^2 + f^2 + 2*f*(1-f)*cos(twistRad))
//
// Integrating f from 0 to 1 gives integral = (2 + cos(twistRad)) / 3, so
//
//	V = (radius^2 * sweepRad * h / 6) * (2 + cos(twistRad))
//
// which reduces at twistRad=0 to (1/2)*radius^2*sweepRad*h — the untwisted
// pie slice's own volume, the fixture TestChordedBoundaryVolumeAllowEnclosesTheMeasuredGap
// already pins.
func twistedPieSliceTrueVolume(radius, sweepRad, twistRad, h float64) float64 {
	return (radius * radius * sweepRad * h / 6) * (2 + math.Cos(twistRad))
}

// ratOfFloat lifts a float64 into an exact big.Rat leaf: every coordinate
// this test measures is itself a float64, hence an exact rational
// (clearance_poly.go's take-the-floats-exactly discipline), so no rounding
// is introduced by the lift.
func ratOfFloat(x float64) *big.Rat {
	r := new(big.Rat)
	r.SetFloat64(x)
	return r
}

// heldVolumeExact sums signed tetrahedra, anchored at the origin, over
// EXACTLY the triangle set a test mesh builds — the same two-triangle-per-
// wall-cell topology assembleLoft emits (loft_build.go) and
// loftMassAccumulator measures (loft_moments.go) — as an exact rational, so
// the "measured" gap this file's enclosure tests compare against is never
// itself subject to float summation slop.
func heldVolumeExact(verts []r3.Vec, tris [][3]int) float64 {
	vol6 := new(big.Rat)
	for _, tri := range tris {
		a, b, c := verts[tri[0]], verts[tri[1]], verts[tri[2]]
		ax, ay, az := ratOfFloat(a.X), ratOfFloat(a.Y), ratOfFloat(a.Z)
		bx, by, bz := ratOfFloat(b.X), ratOfFloat(b.Y), ratOfFloat(b.Z)
		cx, cy, cz := ratOfFloat(c.X), ratOfFloat(c.Y), ratOfFloat(c.Z)
		crossX := new(big.Rat).Sub(new(big.Rat).Mul(by, cz), new(big.Rat).Mul(bz, cy))
		crossY := new(big.Rat).Sub(new(big.Rat).Mul(bz, cx), new(big.Rat).Mul(bx, cz))
		crossZ := new(big.Rat).Sub(new(big.Rat).Mul(bx, cy), new(big.Rat).Mul(by, cx))
		dot := new(big.Rat).Add(new(big.Rat).Mul(ax, crossX), new(big.Rat).Mul(ay, crossY))
		dot.Add(dot, new(big.Rat).Mul(az, crossZ))
		vol6.Add(vol6, dot)
	}
	vol := new(big.Rat).Quo(vol6, big.NewRat(6, 1))
	f, _ := vol.Float64()
	return f
}

// chordedBoundaryAllowForTwistedPieSlice computes chordedBoundaryVolumeAllow's
// own two composed legs for one twisted-pie-slice row: the chord-to-curve
// leg (sectionDelta * areaUpper, areaUpper the mesh's perturbedAreaUpper PLUS
// cellRuledExcessUpper summed over the n curved arc cells — the two radial
// cells are single straight segments and carry zero excess) and the
// ruled-to-triangle (twist) leg, cellTwistVolumeAllow summed over every one
// of the n+2 wall cells (the n arc cells and the 2 radial ones), since a
// twisted top section gives every wall cell — including the two radial ones,
// whose "outer" corners rotate by twistRad between sections — a nonzero
// twist vector.
func chordedBoundaryAllowForTwistedPieSlice(radius, sweepRad, twistRad, h float64, n int) float64 {
	sectionDelta := chordSagitta(radius, sweepRad, n)
	verts, tris := twistedPieSliceMesh(radius, sweepRad, twistRad, h, n)
	areaUpper := perturbedAreaUpper(verts, tris, sectionDelta)

	dtheta := sweepRad / float64(n)
	chordLen := 2 * radius * math.Sin(dtheta/2)
	arcLen := radius * dtheta
	excess := math.Max(0, arcLen-chordLen)
	for range n {
		areaUpper = absSumUpper(areaUpper, cellRuledExcessUpper(h, excess, excess))
	}

	// Vertex layout from twistedPieSliceMesh: 0=centerB, 1=centerT, then per
	// i in [0, n]: arcB[i] at 2+2*i (bottom, twist=0), arcT[i] at 3+2*i (top,
	// twist=twistRad).
	arcB := func(i int) r3.Vec { return verts[2+2*i] }
	arcT := func(i int) r3.Vec { return verts[3+2*i] }
	centerB, centerT := verts[0], verts[1]

	twistVolumeUpper := 0.0
	for i := range n {
		twistVolumeUpper = absSumUpper(twistVolumeUpper, cellTwistVolumeAllow(arcB(i), arcB(i+1), arcT(i), arcT(i+1)))
	}
	twistVolumeUpper = absSumUpper(twistVolumeUpper, cellTwistVolumeAllow(centerB, arcB(0), centerT, arcT(0)))
	twistVolumeUpper = absSumUpper(twistVolumeUpper, cellTwistVolumeAllow(arcB(n), centerB, arcT(n), centerT))

	return chordedBoundaryVolumeAllow(sectionDelta, areaUpper, twistVolumeUpper)
}

// TestChordedBoundaryVolumeAllowEnclosesTheMeasuredGap is the A10 plan's
// required enclosure test, extended past a pure curvature sweep to carry a
// TWIST between the two paired sections in every row: the refuted version of
// this bound modelled the built wall as a bilinear RULED patch, when
// assembleLoft actually emits two FLAT TRIANGLES per cell, and a fixture
// with no twist cannot see that gap (every wall cell degenerates to a planar
// quad, whose ruled-patch and flat-triangle readings coincide). Two rows are
// the refutation's own counterexamples: 20 degrees of twist at 64 stations,
// and 90 degrees at 256 stations, where the pre-fix bound's own allow/gap
// ratio measured 0.53 and 0.059 respectively — well under the 1.0 an
// enclosure requires.
func TestChordedBoundaryVolumeAllowEnclosesTheMeasuredGap(t *testing.T) {
	radii := []float64{1, 5, 50}
	sweepsDeg := []float64{30, 90, 180, 270}
	heights := []float64{0.1, 10, 100}
	chordCounts := []int{8, 32, 64, 128, 256}
	twistsDeg := []float64{0, 5, 20, 45, 90}

	minRatio := math.Inf(1)
	var minRow string
	rows := 0

	for _, r := range radii {
		for _, sweepDeg := range sweepsDeg {
			sweepRad := sweepDeg * math.Pi / 180
			for _, h := range heights {
				for _, twistDeg := range twistsDeg {
					twistRad := twistDeg * math.Pi / 180
					for _, n := range chordCounts {
						rows++

						trueVolume := twistedPieSliceTrueVolume(r, sweepRad, twistRad, h)
						verts, tris := twistedPieSliceMesh(r, sweepRad, twistRad, h, n)
						heldVolume := heldVolumeExact(verts, tris)
						measuredGap := math.Abs(trueVolume - heldVolume)

						allow := chordedBoundaryAllowForTwistedPieSlice(r, sweepRad, twistRad, h, n)
						require.GreaterOrEqualf(t, allow, measuredGap,
							"r=%g sweep=%g h=%g twist=%g n=%d: allow must enclose the measured volume gap",
							r, sweepDeg, h, twistDeg, n)

						if measuredGap > 0 {
							ratio := allow / measuredGap
							if ratio < minRatio {
								minRatio = ratio
								minRow = fmt.Sprintf("r=%g sweep=%g h=%g twist=%g n=%d", r, sweepDeg, h, twistDeg, n)
							}
						}
					}
				}
			}
		}
	}

	require.Positive(t, rows)
	require.GreaterOrEqual(t, minRatio, 1.0, "the loosest row in the table: %s", minRow)
	t.Logf("worst-case allow/measuredGap ratio %.6g at %s (%d rows)", minRatio, minRow, rows)
}

// TestChordedBoundaryVolumeAllowEnclosesTheRefutedCounterexamples pins the
// two specific rows the audit measured against the pre-fix bound: 20 degrees
// of twist at 64 stations (pre-fix ratio 0.53) and 90 degrees at 256
// stations (pre-fix ratio 0.059). Both must now enclose with room to spare.
// A radius/sweep/height combination is fixed here (rather than swept) so
// this test states its own two rows directly, independent of whatever the
// broader sweep above happens to cover.
func TestChordedBoundaryVolumeAllowEnclosesTheRefutedCounterexamples(t *testing.T) {
	const radius, sweepDeg, h = 10.0, 120.0, 25.0
	sweepRad := sweepDeg * math.Pi / 180

	cases := []struct {
		twistDeg float64
		n        int
	}{
		{20, 64},
		{90, 256},
	}

	for _, tc := range cases {
		t.Run(fmt.Sprintf("twist=%gdeg_n=%d", tc.twistDeg, tc.n), func(t *testing.T) {
			twistRad := tc.twistDeg * math.Pi / 180

			trueVolume := twistedPieSliceTrueVolume(radius, sweepRad, twistRad, h)
			verts, tris := twistedPieSliceMesh(radius, sweepRad, twistRad, h, tc.n)
			heldVolume := heldVolumeExact(verts, tris)
			measuredGap := math.Abs(trueVolume - heldVolume)
			require.Greater(t, measuredGap, 0.0, "a twisted pairing must show a nonzero gap")

			allow := chordedBoundaryAllowForTwistedPieSlice(radius, sweepRad, twistRad, h, tc.n)
			require.GreaterOrEqual(t, allow, measuredGap,
				"twist=%g n=%d: allow=%.6g must enclose measuredGap=%.6g (ratio %.6g)",
				tc.twistDeg, tc.n, allow, measuredGap, allow/measuredGap)
			t.Logf("twist=%g n=%d: allow/measuredGap ratio %.6g", tc.twistDeg, tc.n, allow/measuredGap)
		})
	}
}

// TestChordedBoundaryVolumeAllowRatioDoesNotDegradeUnderRefinement is the
// refutation's own diagnostic, re-run against the fixed bound and pinned as
// an assertion rather than a log line: at FIXED twist, refining the chord
// count (more stations) must not push the allow/measuredGap ratio down
// toward 1. The refuted bound failed exactly this way — its own missing
// term shrank at O(1/n^2) while the true gap it needed to cover shrank only
// at O(1/n), so refining the mesh made the bound tighter than the truth
// faster than the truth itself tightened, and the ratio fell through 1.
func TestChordedBoundaryVolumeAllowRatioDoesNotDegradeUnderRefinement(t *testing.T) {
	const radius, sweepDeg, h = 10.0, 120.0, 25.0
	sweepRad := sweepDeg * math.Pi / 180
	chordCounts := []int{8, 32, 64, 128, 256}

	for _, twistDeg := range []float64{5, 20, 45, 90} {
		t.Run(fmt.Sprintf("twist=%gdeg", twistDeg), func(t *testing.T) {
			twistRad := twistDeg * math.Pi / 180

			var ratios []float64
			for _, n := range chordCounts {
				trueVolume := twistedPieSliceTrueVolume(radius, sweepRad, twistRad, h)
				verts, tris := twistedPieSliceMesh(radius, sweepRad, twistRad, h, n)
				heldVolume := heldVolumeExact(verts, tris)
				measuredGap := math.Abs(trueVolume - heldVolume)
				require.Greater(t, measuredGap, 0.0)

				allow := chordedBoundaryAllowForTwistedPieSlice(radius, sweepRad, twistRad, h, n)
				require.GreaterOrEqual(t, allow, measuredGap)
				ratios = append(ratios, allow/measuredGap)
			}

			t.Logf("twist=%g ratios across n=%v: %v", twistDeg, chordCounts, ratios)

			// The ratio may wobble a little station to station (both the
			// allowance and the measured gap are sums of many small
			// per-cell terms whose own relative weights shift as n grows),
			// but it must never trend down toward 1 as the mesh refines: the
			// finest station count's own ratio must stay within a modest
			// factor of the coarsest one, never collapse toward it from
			// above.
			require.GreaterOrEqual(t, ratios[len(ratios)-1], ratios[0]*0.5,
				"twist=%g: refining from n=%d to n=%d must not degrade the ratio toward 1 (got %.6g -> %.6g)",
				twistDeg, chordCounts[0], chordCounts[len(chordCounts)-1], ratios[0], ratios[len(ratios)-1])
		})
	}
}

// TestCellRuledExcessUpperRefusesOnANegativeExcess pins the fix to the
// soundness gap the audit found: a caller-supplied excess below zero is a
// BROKEN claim (a chord can never exceed the curve it subtends), and this
// helper must answer +Inf for it — the repository's convention for an
// underivable bound — never 0, which would silently understate a widening
// term instead of refusing to state one.
func TestCellRuledExcessUpperRefusesOnANegativeExcess(t *testing.T) {
	require.True(t, math.IsInf(cellRuledExcessUpper(10, -1, 0), 1),
		"a negative excess is a broken caller claim, never a shrinking term")
	require.True(t, math.IsInf(cellRuledExcessUpper(10, 0, -1), 1))
	require.True(t, math.IsInf(cellRuledExcessUpper(0, -1, 0), 1),
		"a zero rule length does not excuse a negative excess claim")
}

// TestCellRuledExcessUpperIsZeroWithoutCurvature pins the fallback term's own
// legitimate zero: a cell with no arc-minus-chord excess on either side (a
// straight LineSeg pairing, docs/loft-design.md's increment-1 case)
// contributes nothing, and a cell with no rule length has no area for a
// ruled surface to sweep, whatever its excess claims — a genuinely,
// unconditionally zero-area case, not a broken claim.
func TestCellRuledExcessUpperIsZeroWithoutCurvature(t *testing.T) {
	require.Equal(t, 0.0, cellRuledExcessUpper(10, 0, 0))
	require.Equal(t, 0.0, cellRuledExcessUpper(0, 1, 1), "no rule length is no excess to charge")
}

// TestCellRuledExcessUpperScalesWithRuleLengthAndExcess pins the closed form
// itself: ruleLengthUpper * (excess0 + excess1), outward-rounded.
func TestCellRuledExcessUpperScalesWithRuleLengthAndExcess(t *testing.T) {
	got := cellRuledExcessUpper(3, 0.1, 0.2)
	require.InDelta(t, 3*(0.1+0.2), got, 1e-15)
	require.GreaterOrEqual(t, got, 3*(0.1+0.2), "the answer must round outward, never inward")
}

// TestCellTwistVolumeAllowIsZeroWithoutTwist pins the new term's own
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

// TestCellTwistVolumeAllowScalesWithTheTwistVector pins the closed form
// against a hand-computed cell: a square cell twisted by rotating its top
// edge 90 degrees relative to its bottom edge, where the twist vector T =
// vLo-vHi-wLo+wHi, eA and eB (the two edge-length products the derivation's
// part (b) bounds the homotopy's own facet area by) are all exactly
// rational and can be checked by hand.
func TestCellTwistVolumeAllowScalesWithTheTwistVector(t *testing.T) {
	vLo := r3.NewVec(0, 0, 0)
	vHi := r3.NewVec(1, 0, 0)
	wLo := r3.NewVec(0, 0, 1)
	wHi := r3.NewVec(0, 1, 1)

	twist := vLo.Sub(vHi).Sub(wLo).Add(wHi) // (0,0,0)-(1,0,0)-(0,0,1)+(0,1,1) = (-1,1,0)
	require.InDelta(t, math.Sqrt2, twist.Len(), 1e-15)

	eA := math.Max(vHi.Sub(vLo).Len(), wHi.Sub(wLo).Len()) // max(1, 1) = 1
	eB := math.Max(wLo.Sub(vLo).Len(), wHi.Sub(vHi).Len()) // max(1, sqrt(2))
	want := (twist.Len() / 4) * eA * eB

	got := cellTwistVolumeAllow(vLo, vHi, wLo, wHi)
	require.InDelta(t, want, got, 1e-12)
	require.GreaterOrEqual(t, got, want, "the answer must round outward, never inward")
}

// TestChordedBoundaryVolumeAllowComposesBothLegs pins that
// chordedBoundaryVolumeAllow composes its chord-to-curve leg and its
// caller-supplied twist leg by absSumUpper, never by picking the larger of
// the two or dropping either: with a positive twist leg and a zero
// chord-to-curve leg (sectionDelta == 0), the whole answer is exactly the
// twist leg; with both positive, the answer is at least as large as either
// leg alone.
func TestChordedBoundaryVolumeAllowComposesBothLegs(t *testing.T) {
	// absSumUpper rounds its outward-nudged sum away from 3.5 by construction
	// (upRound's own contract), so this is checked as an enclosure — never
	// pinned to a literal float this platform's own rounding could move a ulp
	// either way — rather than an exact match.
	twistOnlyAnswer := chordedBoundaryVolumeAllow(0, 5.0, 3.5)
	require.GreaterOrEqual(t, twistOnlyAnswer, 3.5)
	require.InDelta(t, 3.5, twistOnlyAnswer, 1e-12)
	require.Equal(t, 0.0, chordedBoundaryVolumeAllow(0, 5.0, 0))
	require.Equal(t, 0.0, chordedBoundaryVolumeAllow(0.01, 0, 0))

	both := chordedBoundaryVolumeAllow(0.01, 5.0, 3.5)
	chordOnly := chordedBoundaryVolumeAllow(0.01, 5.0, 0)
	twistOnly := chordedBoundaryVolumeAllow(0, 5.0, 3.5)
	require.GreaterOrEqual(t, both, chordOnly)
	require.GreaterOrEqual(t, both, twistOnly)
}

// TestChordedBoundaryMomentAllowIsItsOwnThreeLineTwin pins
// chordedBoundaryMomentAllow's own closed form —
// chordedBoundaryVolumeAllow(sectionDelta, areaUpper, twistVolumeUpper)
// composed with coordUpper the same way sweptMomentAllow composes
// sweptVolumeAllow with it — computed here by calling
// chordedBoundaryVolumeAllow directly, never sweptVolumeAllow, so a caller
// that swapped the two internally would move this answer.
func TestChordedBoundaryMomentAllowIsItsOwnThreeLineTwin(t *testing.T) {
	sectionDelta, areaUpper, twistVolumeUpper, coordUpper := 0.02, 7.5, 1.25, 3.0
	want := productUpper(chordedBoundaryVolumeAllow(sectionDelta, areaUpper, twistVolumeUpper), coordUpper)
	got := chordedBoundaryMomentAllow(sectionDelta, areaUpper, twistVolumeUpper, coordUpper)
	require.Equal(t, want, got)

	require.Equal(t, 0.0, chordedBoundaryMomentAllow(0, areaUpper, 0, coordUpper))
	require.Equal(t, 0.0, chordedBoundaryMomentAllow(sectionDelta, areaUpper, twistVolumeUpper, 0))
}
