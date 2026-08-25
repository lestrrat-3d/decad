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
// chordedBoundaryMomentAllow, chordedBoundarySeamAllow, cellChordCurveAreaUpper,
// capAreaVolumeAllow, cellTwistOffsetUpper and cellTwistVolumeAllow
// (docs/loft-design.md §5 — the chord-chain subsection lands with the arc
// design change; the A10 plan's Part 2 Q4 and Part 4 R1 fallback): the
// enclosure chordedBoundaryVolumeAllow proves between the HELD FLAT-TRIANGLE
// polyhedron assembleLoft actually builds and the true curved solid it
// approximates, over a table of radii, sweeps, heights, chord counts AND —
// the mechanism a previous version of this bound missed entirely — a TWIST
// angle between the two paired sections.
//
// A fixture with no twist cannot exercise cellTwistVolumeAllow: every wall
// cell degenerates to a planar quad, its own twist vector is exactly zero,
// and a bound that omits the twist term altogether still passes. So every
// table below carries a twist sweep, and the two rows an earlier refutation
// of this bound failed outright — 20 degrees of twist at 64 stations, and 90
// degrees at 256 stations — are asserted explicitly.
//
// THREE independent audits refuted earlier versions of this bound. The
// third refutation's own findings (F1-F6) drive this file's structure
// beyond the sweep above and beyond the second refutation's own three
// bullets that follow it:
//
//   - the chord-to-curve leg must bound the AREA of the bilinear RULED PATCH
//     a wall cell's four chord corners span, and every surface between it
//     and the true curved wall — never a held-triangle-area-plus-excess
//     reading, which is not sign-definite (TestCellChordCurveAreaUpper*
//     pins a direct counterexample cell where the held triangle pair holds
//     almost no area while its own ruled patch already carries a third of a
//     square unit);
//   - a CAP's own area growth (polygon boundary replaced by the recorded
//     curve, its vertices fixed) needs a term of its own, closed exactly
//     through the cap's own fixed plane rather than a homotopy
//     (TestCapAreaVolumeAllow*);
//   - the fixture's own worst row must bind at a TWISTED station, never at
//     twist zero (the mechanism this whole file exists to prove), and a
//     refinement test must be driven by a quantity that ACTUALLY shrinks
//     with refinement — F4: an earlier refinement test's own arc/apex
//     SHARE was a fixture constant, sweep/(sweep+2), identical at every
//     station count, and a check against it was really testing the sweep
//     angle, not refinement
//     (TestChordedBoundaryVolumeAllowRatioDoesNotDegradeUnderRefinement).
//
// The third refutation (F1-F6, this file's own current structure):
//
//   - F1: cellChordCurveAreaUpper's own eB term silently upgraded a
//     SET-distance sagitta into a PARAMETER-MATCHED displacement — the two
//     coincide only for a LINE or an ARC (TestArcMatchedDeltaEqualsSagitta)
//     and can differ by the CHORD LENGTH for any other curve
//     (TestCellChordCurveAreaUpperRefusesTheSagittaZigzag);
//   - F2: chordedBoundaryVolumeAllow's own wall leg applied a closed-surface
//     flux identity to an OPEN patch, dropping the LINE-INTEGRAL boundary
//     term the by-parts identity commits when the wall's own r=0/r=1 seam
//     moves — chordedBoundarySeamAllow charges it explicitly, as a fourth
//     leg (TestChordedBoundarySeamAllow*, and the seam operands every
//     chordedBoundaryAllowForTwistedPieSlice/ringAllow row now composes);
//   - F3: the fixture was vacuous for the wall leg — deleting
//     cellChordCurveAreaUpper's own composed contribution never failed
//     anywhere in the 900-row sweep table. TestChordedBoundaryVolumeAllow*
//     LoadBearing and *JointlyLoadBearing pin exactly what deletion DOES
//     and does NOT fail, with the honest finding recorded in the latter's
//     own doc comment: F2's own (sound) seam leg subsumes the wall leg's
//     necessary share in this circular-arc family, so only wall+twist
//     TOGETHER could be shown load-bearing here, never wall alone;
//   - F4: see above (the second refutation's own third bullet, now fixed);
//   - F5: cellChordCurveAreaUpper validated its three scalar operands but
//     not its four r3.Vec corners, so a NaN vertex propagated to a silent
//     NaN answer rather than a refusing +Inf — just as dangerous as a
//     silent 0, since `NaN > 0` is false for every downstream consumer's
//     own widening check (TestCellChordCurveAreaUpperRefusesNonFiniteCorners);
//   - F6: chordedBoundaryMomentAllow guarded isNonFinite(coordUpper) but not
//     coordUpper<0 (TestChordedBoundaryMomentAllowRefusesOnBrokenClaims).

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
// pie-slice loft denotes (docs/loft-design.md's own chord-to-curve
// homotopy, §5 — the chord-chain subsection lands with the arc design
// change): the RULED surface between the bottom arc, at angle s over [0,
// sweepRad], and the top arc, at angle s+twistRad, linearly interpolated by
// height fraction f = z/h, plus the two flat sector caps (bounded by the
// TRUE arc, not the chorded polygon) and the two (excess-free, since they
// are single straight segments, not chorded) radial walls.
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

// bilinearPatchAreaNumeric estimates, by a fine midpoint Riemann sum, the
// area of the bilinear ruled patch X(s,r) = (1-r)*((1-s)*vLo + s*vHi) +
// r*((1-s)*wLo + s*wHi) over the unit square. It is a NUMERICAL REFERENCE
// this file's own regression tests compare cellChordCurveAreaUpper's
// published bound against, never a proof of its own: every cell this file
// feeds it is either flat or mildly curved, well within what a 400x400 grid
// resolves far past the margin these tests require.
func bilinearPatchAreaNumeric(vLo, vHi, wLo, wHi r3.Vec) float64 {
	const nGrid = 400
	const step = 1.0 / nGrid
	total := 0.0
	edgeA := vHi.Sub(vLo)
	edgeB := wHi.Sub(wLo)
	for i := range nGrid {
		s := (float64(i) + 0.5) * step
		a := vLo.Add(edgeA.Scale(s))
		b := wLo.Add(edgeB.Scale(s))
		rung := b.Sub(a)
		for j := range nGrid {
			r := (float64(j) + 0.5) * step
			ds := edgeA.Scale(1 - r).Add(edgeB.Scale(r))
			total += ds.Cross(rung).Len() * step * step
		}
	}
	return total
}

// chordedAllowBreakdown separates a twisted pie slice's own
// chordedBoundaryVolumeAllow composition into the REFINED chorded-arc wall
// cells (the n cells whose own geometry shrinks and multiplies as the
// station count n grows) and the two UNREFINED radial/apex cells
// (centerB-arcB(0)/centerT-arcT(0) and arcB(n)-centerB/arcT(n)-centerT,
// whose own four corners are fixed by radius, sweepRad and twistRad alone
// and never change with n) — the split an earlier refutation needed to show
// a "refinement does not degrade" test was actually driven by the refined
// cells, not by the two constant apex ones.
type chordedAllowBreakdown struct {
	sectionDelta        float64
	wallAreaArc         float64
	wallAreaApex        float64
	twistVolumeArc      float64
	twistVolumeApex     float64
	capVolumeUpper      float64
	seamPerimeterUpper  float64
	posUpper            float64
	seamAllow           float64
	maxTwistOffsetUpper float64
	allow               float64
}

// chordedBoundaryAllowForTwistedPieSlice computes chordedBoundaryVolumeAllow's
// own three composed legs for one twisted-pie-slice row, split by cell kind:
//
//   - wall chord-to-curve: cellChordCurveAreaUpper summed over every wall
//     cell (the n arc cells, each side's own TRUE arc length radius*dtheta
//     fed as arcLenUpper — never the height h a previous fixture fed as a
//     stand-in "rule length", a quantity this primitive no longer even
//     takes as a parameter, reading the cell's own corners directly
//     instead; and the 2 radial cells, each side's own arc length equal to
//     its own chord length exactly, since a radial wall is a straight
//     LineSeg) — multiplied by sectionDelta only once, inside
//     chordedBoundaryVolumeAllow itself;
//   - ruled-to-triangle (twist): cellTwistVolumeAllow summed over every one
//     of the n+2 wall cells, since a twisted top section gives every wall
//     cell — including the two radial ones, whose "outer" corners rotate by
//     twistRad between sections — a nonzero twist vector; and
//     cellTwistOffsetUpper's own MAXIMUM over every cell, the widening
//     chordedBoundaryMomentAllow's own coordUpper obligation needs (unused
//     by the volume-only tests below, carried for the moment tests);
//   - cap chord-to-curve: capAreaVolumeAllow for the top cap alone — the
//     bottom cap's own plane passes through this fixture's implicit anchor
//     (the world origin, matching heldVolumeExact's own unanchored
//     tetrahedron sum) exactly, so its plane offset is exactly 0 and it
//     contributes nothing, while the top cap sits at offset h;
//   - the seam leg: chordedBoundarySeamAllow over the SAME sectionDelta,
//     posUpper the exact distance from the origin to either loop's own true
//     arc (known exactly in this fixture), and seamPerimeterUpper the sum,
//     over every wall cell (arc and radial alike) and BOTH its own sides, of
//     that cell's own arc-length upper bound — the same quantities each
//     cellChordCurveAreaUpper call above already states.
func chordedBoundaryAllowForTwistedPieSlice(radius, sweepRad, twistRad, h float64, n int) chordedAllowBreakdown {
	sectionDelta := chordSagitta(radius, sweepRad, n)
	verts, _ := twistedPieSliceMesh(radius, sweepRad, twistRad, h, n)

	// Vertex layout from twistedPieSliceMesh: 0=centerB, 1=centerT, then per
	// i in [0, n]: arcB[i] at 2+2*i (bottom, twist=0), arcT[i] at 3+2*i (top,
	// twist=twistRad).
	arcB := func(i int) r3.Vec { return verts[2+2*i] }
	arcT := func(i int) r3.Vec { return verts[3+2*i] }
	centerB, centerT := verts[0], verts[1]

	var b chordedAllowBreakdown
	b.sectionDelta = sectionDelta

	dtheta := sweepRad / float64(n)
	arcLenPerArcCell := radius * dtheta

	for i := range n {
		vLo, vHi, wLo, wHi := arcB(i), arcB(i+1), arcT(i), arcT(i+1)
		b.wallAreaArc = absSumUpper(b.wallAreaArc, cellChordCurveAreaUpper(vLo, vHi, wLo, wHi, arcLenPerArcCell, arcLenPerArcCell, sectionDelta))
		b.twistVolumeArc = absSumUpper(b.twistVolumeArc, cellTwistVolumeAllow(vLo, vHi, wLo, wHi))
		b.maxTwistOffsetUpper = math.Max(b.maxTwistOffsetUpper, cellTwistOffsetUpper(vLo, vHi, wLo, wHi))
		// arcLenPerArcCell is BOTH sides' own arc-length upper bound for an
		// arc cell (an untwisted or twisted rotation does not change either
		// arc's own length), so this cell's own contribution to BOTH the
		// r=0 and r=1 loop's own seam perimeter is arcLenPerArcCell each.
		b.seamPerimeterUpper = absSumUpper(b.seamPerimeterUpper, arcLenPerArcCell, arcLenPerArcCell)
	}

	radialCells := [2][4]r3.Vec{
		{centerB, arcB(0), centerT, arcT(0)},
		{arcB(n), centerB, arcT(n), centerT},
	}
	for _, c := range radialCells {
		vLo, vHi, wLo, wHi := c[0], c[1], c[2], c[3]
		arcLenA := vHi.Sub(vLo).Len()
		arcLenB := wHi.Sub(wLo).Len()
		b.wallAreaApex = absSumUpper(b.wallAreaApex, cellChordCurveAreaUpper(vLo, vHi, wLo, wHi, arcLenA, arcLenB, sectionDelta))
		b.twistVolumeApex = absSumUpper(b.twistVolumeApex, cellTwistVolumeAllow(vLo, vHi, wLo, wHi))
		b.maxTwistOffsetUpper = math.Max(b.maxTwistOffsetUpper, cellTwistOffsetUpper(vLo, vHi, wLo, wHi))
		b.seamPerimeterUpper = absSumUpper(b.seamPerimeterUpper, arcLenA, arcLenB)
	}

	chordLen := 2 * radius * math.Sin(dtheta/2)
	perimeterUpper1 := float64(n)*chordLen + 2*radius
	capAreaAllow1 := sectionDisplacementArea(sectionDelta, n+2, perimeterUpper1)
	b.capVolumeUpper = capAreaVolumeAllow(h, capAreaAllow1)

	// posUpper is a PROVEN upper bound on the distance from the fixture's own
	// anchor (the world origin, matching heldVolumeExact's own unanchored
	// tetrahedron sum and centerB) to any point of either loop's TRUE curve:
	// the bottom loop's own true arc sits at exactly `radius` from the
	// origin, the top loop's own true arc at exactly sqrt(radius^2+h^2) —
	// known exactly in this fixture, nudged outward by one ulp so the
	// closed-form Hypot's own rounding can never understate it.
	b.posUpper = upRound(math.Nextafter(math.Hypot(radius, h), math.Inf(1)))
	b.seamAllow = chordedBoundarySeamAllow(sectionDelta, b.posUpper, b.seamPerimeterUpper)

	wallAreaUpper := absSumUpper(b.wallAreaArc, b.wallAreaApex)
	twistVolumeUpper := absSumUpper(b.twistVolumeArc, b.twistVolumeApex)
	b.allow = chordedBoundaryVolumeAllow(sectionDelta, wallAreaUpper, twistVolumeUpper, b.capVolumeUpper, b.seamAllow)
	return b
}

// TestChordedBoundaryVolumeAllowEnclosesTheMeasuredGap is the A10 plan's
// required enclosure test, extended past a pure curvature sweep to carry a
// TWIST between the two paired sections in every row: an earlier version of
// this bound modelled the built wall as a bilinear RULED patch, when
// assembleLoft actually emits two FLAT TRIANGLES per cell, and a fixture
// with no twist cannot see that gap (every wall cell degenerates to a
// planar quad, whose ruled-patch and flat-triangle readings coincide). Two
// rows are an earlier refutation's own counterexamples: 20 degrees of twist
// at 64 stations, and 90 degrees at 256 stations.
//
// The binding (minimum allow/measuredGap) row must be a TWISTED one: a
// fixture whose worst case lands at twist=0 is not exercising the mechanism
// this file exists to prove (cellTwistVolumeAllow returns exactly 0 there),
// and merely re-covers the untwisted enclosure an earlier, narrower fixture
// already pinned.
func TestChordedBoundaryVolumeAllowEnclosesTheMeasuredGap(t *testing.T) {
	radii := []float64{1, 5, 50}
	sweepsDeg := []float64{30, 90, 180, 270}
	heights := []float64{0.1, 10, 100}
	chordCounts := []int{8, 32, 64, 128, 256}
	twistsDeg := []float64{0, 5, 20, 45, 90}

	minRatio := math.Inf(1)
	var minRow string
	var minTwistDeg float64
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

						allow := chordedBoundaryAllowForTwistedPieSlice(r, sweepRad, twistRad, h, n).allow
						require.GreaterOrEqualf(t, allow, measuredGap,
							"r=%g sweep=%g h=%g twist=%g n=%d: allow must enclose the measured volume gap",
							r, sweepDeg, h, twistDeg, n)

						if measuredGap > 0 {
							ratio := allow / measuredGap
							if ratio < minRatio {
								minRatio = ratio
								minTwistDeg = twistDeg
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
	require.NotZero(t, minTwistDeg, "the binding row must be a TWISTED station, not twist=0 (row: %s) — otherwise this table never exercises cellTwistVolumeAllow at all", minRow)
	t.Logf("worst-case allow/measuredGap ratio %.6g at %s (%d rows)", minRatio, minRow, rows)
}

// TestChordedBoundaryVolumeAllowEnclosesTheRefutedCounterexamples pins the
// two specific rows an earlier audit measured against a pre-fix bound: 20
// degrees of twist at 64 stations and 90 degrees at 256 stations. Both must
// enclose with room to spare.
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

			allow := chordedBoundaryAllowForTwistedPieSlice(radius, sweepRad, twistRad, h, tc.n).allow
			require.GreaterOrEqual(t, allow, measuredGap,
				"twist=%g n=%d: allow=%.6g must enclose measuredGap=%.6g (ratio %.6g)",
				tc.twistDeg, tc.n, allow, measuredGap, allow/measuredGap)
			t.Logf("twist=%g n=%d: allow/measuredGap ratio %.6g", tc.twistDeg, tc.n, allow/measuredGap)
		})
	}
}

// TestChordedBoundaryVolumeAllowRatioDoesNotDegradeUnderRefinement is an
// earlier refutation's own diagnostic, re-run against the fixed primitives
// and pinned as an assertion: at FIXED twist, refining the chord count (more
// stations) must not push the allow/measuredGap ratio down toward 1.
//
// An earlier fixture's own refinement check was itself refuted twice. The
// second refutation's own "arc cells must drive the wall-area leg" guard
// checked arcShare := wallAreaArc/(wallAreaArc+wallAreaApex) > 0.5 — but that
// ratio (F4) is exactly sweep/(sweep+2) for THIS fixture's own geometry,
// identical at every n: the two apex cells' own four corners are fixed by
// radius, sweepRad and twistRad alone, so wallAreaApex never moves, and
// wallAreaArc's own total converges to a CONSTANT (the true wall's own area)
// as n grows rather than shrinking — so the ratio of two near-constants is
// itself near-constant, and the guard was really checking "sweep exceeds 2
// radians" (about a hard-coded 120 degrees) rather than anything about
// refinement — it would have READ AS PASSING at 120 degrees for a reason
// having nothing to do with whether the bound refines properly, and this
// test now runs the SAME guard at 90 degrees too (a sweep the old constant-
// share reading would have failed, per F4), to confirm the replacement
// actually measures refinement rather than sweep angle.
//
// This version checks something that ACTUALLY changes with refinement
// instead: the arc cells' own VOLUME contribution to the wall leg —
// sectionDelta(n) * wallAreaArc(n), the term the composed bound actually
// charges, not a share of an unrelated total — is O(1/n^2) (sectionDelta
// itself is; wallAreaArc converges to a constant as n grows), so refining
// from n=8 to n=256, a 32x refinement, must shrink it by close to 32^2=1024x.
// A guard requiring only 100x leaves ample host-portability slack while
// still failing outright on a fixture that is NOT actually refining (a
// constant-in-n quantity, as arcShare always was, would show a 1x "shrink").
func TestChordedBoundaryVolumeAllowRatioDoesNotDegradeUnderRefinement(t *testing.T) {
	const radius, h = 10.0, 25.0
	chordCounts := []int{8, 32, 64, 128, 256}

	for _, sweepDeg := range []float64{120, 90} {
		sweepRad := sweepDeg * math.Pi / 180
		for _, twistDeg := range []float64{5, 20, 45, 90} {
			t.Run(fmt.Sprintf("sweep=%gdeg/twist=%gdeg", sweepDeg, twistDeg), func(t *testing.T) {
				twistRad := twistDeg * math.Pi / 180

				var ratios []float64
				var arcOnlyVolumes []float64
				for _, n := range chordCounts {
					trueVolume := twistedPieSliceTrueVolume(radius, sweepRad, twistRad, h)
					verts, tris := twistedPieSliceMesh(radius, sweepRad, twistRad, h, n)
					heldVolume := heldVolumeExact(verts, tris)
					measuredGap := math.Abs(trueVolume - heldVolume)
					require.Greater(t, measuredGap, 0.0)

					b := chordedBoundaryAllowForTwistedPieSlice(radius, sweepRad, twistRad, h, n)
					require.GreaterOrEqual(t, b.allow, measuredGap)
					ratios = append(ratios, b.allow/measuredGap)
					arcOnlyVolumes = append(arcOnlyVolumes, b.sectionDelta*b.wallAreaArc)
					t.Logf("sweep=%g twist=%g n=%d: ratio=%.6g arcOnlyVolume=%.6g wallAreaArc=%.6g wallAreaApex=%.6g",
						sweepDeg, twistDeg, n, ratios[len(ratios)-1], arcOnlyVolumes[len(arcOnlyVolumes)-1], b.wallAreaArc, b.wallAreaApex)
				}

				// The REFINED arc cells' own volume contribution must actually
				// shrink as the mesh refines — a genuine refinement property,
				// unlike F4's constant arcShare — never merely track a fixed
				// geometric ratio that happens to exceed some threshold.
				first, last := arcOnlyVolumes[0], arcOnlyVolumes[len(arcOnlyVolumes)-1]
				require.Greaterf(t, first/last, 100.0,
					"sweep=%g twist=%g: the arc cells' own wall-leg volume must shrink by more than 100x from n=%d to n=%d (got %.6g -> %.6g, shrink %.4gx)",
					sweepDeg, twistDeg, chordCounts[0], chordCounts[len(chordCounts)-1], first, last, first/last)

				t.Logf("sweep=%g twist=%g ratios across n=%v: %v", sweepDeg, twistDeg, chordCounts, ratios)

				// The ratio may wobble a little station to station (both the
				// allowance and the measured gap are sums of many small
				// per-cell terms whose own relative weights shift as n grows),
				// but it must never trend down toward 1 as the mesh refines: the
				// finest station count's own ratio must stay within a modest
				// factor of the coarsest one, never collapse toward it from
				// above.
				require.GreaterOrEqual(t, ratios[len(ratios)-1], ratios[0]*0.5,
					"sweep=%g twist=%g: refining from n=%d to n=%d must not degrade the ratio toward 1 (got %.6g -> %.6g)",
					sweepDeg, twistDeg, chordCounts[0], chordCounts[len(chordCounts)-1], ratios[0], ratios[len(ratios)-1])
			})
		}
	}
}

// TestChordedBoundaryVolumeAllowTwistLegIsLoadBearing pins that the twist
// leg is genuinely necessary — not merely present — by DELETING it
// (twistVolumeUpper forced to 0, every other leg left intact) at the sweep
// table's own worst row for that deletion (r=1 sweep=30 h=100 twist=90
// n=256, found by scanning the same 900-row grid
// TestChordedBoundaryVolumeAllowEnclosesTheMeasuredGap already covers) and
// confirming the composed bound collapses far below the measured gap: F3's
// own finding was that deleting this leg drops the minimum ratio to 1.3e-5
// across the whole table, and this pins the concrete row and re-confirms it
// after F1/F2/F4's own fixes (the newly added seam leg can only ADD
// coverage, never rescue a row this leg alone was carrying).
func TestChordedBoundaryVolumeAllowTwistLegIsLoadBearing(t *testing.T) {
	const radius, sweepDeg, h, twistDeg, n = 1.0, 30.0, 100.0, 90.0, 256
	sweepRad := sweepDeg * math.Pi / 180
	twistRad := twistDeg * math.Pi / 180

	trueVolume := twistedPieSliceTrueVolume(radius, sweepRad, twistRad, h)
	verts, tris := twistedPieSliceMesh(radius, sweepRad, twistRad, h, n)
	heldVolume := heldVolumeExact(verts, tris)
	measuredGap := math.Abs(trueVolume - heldVolume)
	require.Greater(t, measuredGap, 0.0)

	b := chordedBoundaryAllowForTwistedPieSlice(radius, sweepRad, twistRad, h, n)
	require.GreaterOrEqual(t, b.allow, measuredGap, "the full four-leg bound must enclose this row")

	wallAreaUpper := absSumUpper(b.wallAreaArc, b.wallAreaApex)
	withoutTwist := chordedBoundaryVolumeAllow(b.sectionDelta, wallAreaUpper, 0, b.capVolumeUpper, b.seamAllow)
	require.Lessf(t, withoutTwist, measuredGap,
		"deleting the twist leg alone must fail to enclose the measured gap (got allow=%.6g < measuredGap=%.6g is required; ratio %.6g)",
		withoutTwist, measuredGap, withoutTwist/measuredGap)
	t.Logf("full ratio=%.6g, without-twist ratio=%.6g (must be < 1)", b.allow/measuredGap, withoutTwist/measuredGap)
}

// TestChordedBoundaryVolumeAllowCapLegIsLoadBearing pins that the cap leg is
// genuinely necessary at the COMPOSITION level. The geometric sweep table
// never drives this: a scan of the same 900-row grid found the cap leg's
// own deletion never drops the ratio below 3.0 anywhere in it, because
// capAreaVolumeAllow's own capAreaAllow input (sectionDisplacementArea) is
// generous enough, for every row in that circular-arc family, that the wall
// and seam legs already cover what the cap leg would have. This test proves
// the SAME fact TestCapAreaVolumeAllowIsExactForAPlanarFace already pins for
// capAreaVolumeAllow alone, one level up: with the wall, twist and seam legs
// all synthetically absent (0 — a legitimate state, e.g. an exact LineSeg
// wall pairing with no twist), the composed bound must still publish EXACTLY
// the cap leg's own known-exact volume displacement (h·area/3 = 8 for
// h=4, area=6), and deleting the cap leg alone must drop the composed
// answer to 0 — failing to cover a REAL, exactly-known volume displacement.
func TestChordedBoundaryVolumeAllowCapLegIsLoadBearing(t *testing.T) {
	const h, area = 4.0, 6.0
	const knownExactVolumeDisplacement = h * area / 3 // == 8, capAreaVolumeAllow's own exact identity

	capVolumeUpper := capAreaVolumeAllow(h, area)
	require.InDelta(t, knownExactVolumeDisplacement, capVolumeUpper, 1e-12)

	withCap := chordedBoundaryVolumeAllow(0, 0, 0, capVolumeUpper, 0)
	require.GreaterOrEqual(t, withCap, knownExactVolumeDisplacement)

	withoutCap := chordedBoundaryVolumeAllow(0, 0, 0, 0, 0)
	require.Lessf(t, withoutCap, knownExactVolumeDisplacement,
		"deleting the cap leg alone must fail to enclose the known-exact volume displacement %.6g (got %.6g)",
		knownExactVolumeDisplacement, withoutCap)
}

// ringMesh builds a CLOSED n-gon ring loft (a full 360-degree sweep, no
// apex/radial cells at all — every wall cell chords a genuine arc, and both
// caps are full n-gon disks) at radius, twisted by twistRad between its
// bottom and top section, straight-extruded to height h. It exists
// alongside twistedPieSliceMesh for exactly one purpose: the PIE-SLICE
// fixture's own two apex/radial cells make capAreaVolumeAllow's own
// capAreaAllow input carry needless slack (sectionDisplacementArea's
// delta-tube covers the apex cells' own two EXACT straight edges too, which
// never move at all), which is generous enough on its own to make the cap
// leg's own deletion never fail anywhere in the pie-slice sweep table. A
// closed ring has no such edges, so its own cap bound is tighter, and it is
// the fixture TestChordedBoundaryVolumeAllowWallAndTwistLegsAreJointlyLoadBearing
// uses to find a row where dropping the wall AND twist legs together still
// fails despite that tighter cap.
func ringMesh(radius, twistRad, h float64, n int) (verts []r3.Vec, tris [][3]int) {
	arcPoint := func(i int, twist, z float64) r3.Vec {
		theta := twist + 2*math.Pi*float64(i)/float64(n)
		return r3.NewVec(radius*math.Cos(theta), radius*math.Sin(theta), z)
	}
	arcB := make([]int, n)
	arcT := make([]int, n)
	for i := range n {
		arcB[i] = len(verts)
		verts = append(verts, arcPoint(i, 0, 0))
		arcT[i] = len(verts)
		verts = append(verts, arcPoint(i, twistRad, h))
	}
	for i := range n {
		jn := (i + 1) % n
		tris = append(tris, [3]int{arcB[i], arcB[jn], arcT[jn]})
		tris = append(tris, [3]int{arcB[i], arcT[jn], arcT[i]})
	}
	for i := 1; i < n-1; i++ {
		tris = append(tris, [3]int{arcB[0], arcB[i+1], arcB[i]})
	}
	for i := 1; i < n-1; i++ {
		tris = append(tris, [3]int{arcT[0], arcT[i], arcT[i+1]})
	}
	return verts, tris
}

// ringAllow computes chordedBoundaryVolumeAllow's own four operands for one
// ringMesh row, mirroring chordedBoundaryAllowForTwistedPieSlice's own
// construction one mesh family over: matchedDelta the ring's own sagitta
// (parameter-matched for an arc — TestArcMatchedDeltaEqualsSagitta),
// wallAreaUpper and twistVolumeUpper summed over every one of the n arc
// wall cells (there are no others), capVolumeUpper from BOTH caps' own
// TIGHT perimeter (n*chordLen, no radial-edge slack — bottom offset 0
// contributes nothing, matching the pie-slice fixture's own anchor
// convention), and seamAllow from posUpper = the exact distance from the
// origin anchor to either loop's own true arc.
func ringAllow(radius, twistRad, h float64, n int) (matchedDelta, wallAreaUpper, twistVolumeUpper, capVolumeUpper, seamAllow float64) {
	const sweepRad = 2 * math.Pi
	matchedDelta = chordSagitta(radius, sweepRad, n)
	arcPoint := func(i int, twist, z float64) r3.Vec {
		theta := twist + sweepRad*float64(i)/float64(n)
		return r3.NewVec(radius*math.Cos(theta), radius*math.Sin(theta), z)
	}
	arcLenPerCell := radius * sweepRad / float64(n)
	var seamPerimeterUpper float64
	for i := range n {
		jn := (i + 1) % n
		vLo, vHi := arcPoint(i, 0, 0), arcPoint(jn, 0, 0)
		wLo, wHi := arcPoint(i, twistRad, h), arcPoint(jn, twistRad, h)
		wallAreaUpper = absSumUpper(wallAreaUpper, cellChordCurveAreaUpper(vLo, vHi, wLo, wHi, arcLenPerCell, arcLenPerCell, matchedDelta))
		twistVolumeUpper = absSumUpper(twistVolumeUpper, cellTwistVolumeAllow(vLo, vHi, wLo, wHi))
		seamPerimeterUpper = absSumUpper(seamPerimeterUpper, arcLenPerCell, arcLenPerCell)
	}
	chordLen := 2 * radius * math.Sin(sweepRad/float64(n)/2)
	perimeterUpper := float64(n) * chordLen
	capAreaAllow := sectionDisplacementArea(matchedDelta, n, perimeterUpper)
	capVolumeUpper = capAreaVolumeAllow(h, capAreaAllow) // bottom cap offset 0 contributes nothing
	posUpper := upRound(math.Nextafter(math.Hypot(radius, h), math.Inf(1)))
	seamAllow = chordedBoundarySeamAllow(matchedDelta, posUpper, seamPerimeterUpper)
	return matchedDelta, wallAreaUpper, twistVolumeUpper, capVolumeUpper, seamAllow
}

// TestChordedBoundaryVolumeAllowWallAndTwistLegsAreJointlyLoadBearing
// documents what an extensive search — the pie-slice sweep table (900
// rows), the tighter ringMesh family above, varying radius, height, twist
// and chord count across both — could and could not show about the wall
// leg's own RAW chord-to-curve area/flux term (matchedDelta*wallAreaUpper)
// once F2's seam leg is present and sound: in EVERY row tried, deleting the
// wall leg alone (keeping twist, cap and seam) still encloses the measured
// gap, because the seam leg's own Cauchy-Schwarz bound is, by itself,
// already larger than the wall leg's own necessary share — confirmed
// analytically for the untwisted ring at the exact (no-slack) cap share:
// seamAllow alone exceeds the true 2/3-of-physical-ΔVolume residual leg (a)
// and leg (d) jointly cover, by a stable ~1.616x margin from n=6 to n=1024.
// That is a property of THIS derivation (a sound but non-tight seam bound
// subsumes leg (a) in a circular-arc family), not a fixture weakness left
// unexplored, so no amount of further fixture tuning within this family
// will make leg (a) alone fail.
//
// What DOES fail, and is pinned here as the concrete evidence Step 1 asks
// for: deleting the wall AND twist legs TOGETHER (both are 0 for the only
// pairing this evaluator admits today, an exact LineSeg wall) at radius=10
// h=25 twist=20deg n=32 drops the ring's own bound below the measured gap,
// so cap and seam alone are NOT a substitute for the mechanisms
// cellChordCurveAreaUpper and cellTwistVolumeAllow each independently prove
// (their own dedicated counterexample tests above — the flat-triangle,
// crossed-cell and twist-vector tests — pin that each is individually
// correct and necessary as an AREA/VOLUME bound in its own right, whatever
// this one composition's own redundancy happens to be).
func TestChordedBoundaryVolumeAllowWallAndTwistLegsAreJointlyLoadBearing(t *testing.T) {
	const radius, h, twistDeg, n = 10.0, 25.0, 20.0, 32
	twistRad := twistDeg * math.Pi / 180

	trueVolume := twistedPieSliceTrueVolume(radius, 2*math.Pi, twistRad, h)
	verts, tris := ringMesh(radius, twistRad, h, n)
	heldVolume := heldVolumeExact(verts, tris)
	measuredGap := math.Abs(trueVolume - heldVolume)
	require.Greater(t, measuredGap, 0.0)

	matchedDelta, wallAreaUpper, twistVolumeUpper, capVolumeUpper, seamAllow := ringAllow(radius, twistRad, h, n)
	full := chordedBoundaryVolumeAllow(matchedDelta, wallAreaUpper, twistVolumeUpper, capVolumeUpper, seamAllow)
	require.GreaterOrEqual(t, full, measuredGap, "the full four-leg bound must enclose this row")

	withoutWallAndTwist := chordedBoundaryVolumeAllow(matchedDelta, 0, 0, capVolumeUpper, seamAllow)
	require.Lessf(t, withoutWallAndTwist, measuredGap,
		"deleting the wall AND twist legs together must fail to enclose the measured gap (got allow=%.6g, measuredGap=%.6g, ratio %.6g)",
		withoutWallAndTwist, measuredGap, withoutWallAndTwist/measuredGap)
	t.Logf("full ratio=%.6g, without-wall-and-twist ratio=%.6g (must be < 1)", full/measuredGap, withoutWallAndTwist/measuredGap)
}

// PER-LEG DELETION-CHECK STATUS. Four legs compose chordedBoundaryVolumeAllow
// — wall (a), twist (b), cap (c), seam (d) — and every one now has a
// deletion check on record, so a reader can tell at a glance which the suite
// actually polices rather than merely carries:
//
//   - TWIST (b): SHOWN-TO-FAIL.
//     TestChordedBoundaryVolumeAllowTwistLegIsLoadBearing (r=1 sweep=30
//     h=100 twist=90 n=256): deleting twist alone drops the ratio to
//     1.3e-5 across the 900-row sweep table.
//   - CAP (c): SHOWN-TO-FAIL, at the COMPOSITION level (no row of the
//     geometric sweep table drives it — capAreaVolumeAllow's own
//     capAreaAllow input is generous enough there that wall+seam already
//     cover it).
//     TestChordedBoundaryVolumeAllowCapLegIsLoadBearing: with wall, twist
//     and seam legitimately 0 (an exact LineSeg wall pairing with no
//     twist, e.g. a straight prism over a chorded circular cap), deleting
//     cap alone drops the composed answer from the known-exact 8 to 0.
//   - WALL (a): PROVEN-REDUNDANT, not merely unshown-to-fail. See
//     chordedBoundaryVolumeAllow's own doc comment for the two-case
//     argument (T=0 / T≠0, no third case). Corroborated, never
//     substituted for, by
//     TestChordedBoundaryVolumeAllowWallAndTwistLegsAreJointlyLoadBearing
//     (wall's own deletion alone never fails anywhere an extensive
//     geometric search tried — only wall+twist TOGETHER fails) and by
//     TestChordedBoundaryVolumeAllowWallLegIsProvenRedundant below, which
//     pins that same "never fails alone" property as a running regression
//     over the full 900-row sweep table.
//   - SEAM (d): NOT SHOWN TO FAIL — an open question, recorded honestly
//     rather than forced into either of the other two categories.
//     TestChordedBoundaryVolumeAllowSeamLegDeletionSearch scans the same
//     900-row sweep table plus a ring-family grid and finds deleting seam
//     alone (wall, twist and cap all left intact) never drops the ratio
//     below roughly 2.5 anywhere tried. That is consistent with seam being
//     provably redundant the way wall is — structurally, seam is only ever
//     nonzero when the wall leg (a) is too (both gate on matchedDelta>0
//     and nonzero wall geometry), and leg (a)'s own flux term is 3x the
//     magnitude of the boundary term seam bounds (chordedBoundaryVolumeAllow's
//     own doc comment, the h·ΔArea vs h·ΔArea/3 split) — but no closed-form
//     redundancy proof was attempted here: doing so would be a NEW
//     derivation, which this pass's own scope rules out. Until either a
//     failing fixture turns up or that proof gets written, seam's own
//     constant is not policed by anything in this suite, exactly the
//     complaint this comment block exists to make impossible to miss.
func TestChordedBoundaryVolumeAllowWallLegIsProvenRedundant(t *testing.T) {
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
						trueVolume := twistedPieSliceTrueVolume(r, sweepRad, twistRad, h)
						verts, tris := twistedPieSliceMesh(r, sweepRad, twistRad, h, n)
						heldVolume := heldVolumeExact(verts, tris)
						measuredGap := math.Abs(trueVolume - heldVolume)
						if measuredGap <= 0 {
							continue
						}
						rows++

						b := chordedBoundaryAllowForTwistedPieSlice(r, sweepRad, twistRad, h, n)
						twistVolumeUpper := absSumUpper(b.twistVolumeArc, b.twistVolumeApex)
						withoutWall := chordedBoundaryVolumeAllow(b.sectionDelta, 0, twistVolumeUpper, b.capVolumeUpper, b.seamAllow)
						require.GreaterOrEqualf(t, withoutWall, measuredGap,
							"r=%g sweep=%g h=%g twist=%g n=%d: deleting the wall leg alone must still enclose the measured gap",
							r, sweepDeg, h, twistDeg, n)

						ratio := withoutWall / measuredGap
						if ratio < minRatio {
							minRatio = ratio
							minRow = fmt.Sprintf("r=%g sweep=%g h=%g twist=%g n=%d", r, sweepDeg, h, twistDeg, n)
						}
					}
				}
			}
		}
	}

	require.Positive(t, rows)
	t.Logf("without-wall worst-case ratio %.6g at %s (%d rows) — corroborates the PROVEN redundancy above, never a substitute for it", minRatio, minRow, rows)
}

// TestChordedBoundaryVolumeAllowSeamLegDeletionSearch is the seam leg's own
// deletion check, the gap this file's earlier F3 finding (about the wall
// leg) left inherited by the leg later added to answer it: zeroing seamAllow
// changed no minimum ratio in any shipped fixture, so nothing in the suite
// could catch a wrong constant in chordedBoundarySeamAllow. This test is
// that missing check — it does not manufacture a failure (deleting seam
// alone did not fail anywhere the search below tried, and no fixture is
// narrowed here to force one), so it stands as a documented NOT-SHOWN-TO-FAIL
// finding, not a SHOWN-TO-FAIL pin: see the per-leg status comment above for
// what would still need to happen before seam is either proven redundant or
// caught by a genuine failing row.
func TestChordedBoundaryVolumeAllowSeamLegDeletionSearch(t *testing.T) {
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
						trueVolume := twistedPieSliceTrueVolume(r, sweepRad, twistRad, h)
						verts, tris := twistedPieSliceMesh(r, sweepRad, twistRad, h, n)
						heldVolume := heldVolumeExact(verts, tris)
						measuredGap := math.Abs(trueVolume - heldVolume)
						if measuredGap <= 0 {
							continue
						}
						rows++

						b := chordedBoundaryAllowForTwistedPieSlice(r, sweepRad, twistRad, h, n)
						wallAreaUpper := absSumUpper(b.wallAreaArc, b.wallAreaApex)
						twistVolumeUpper := absSumUpper(b.twistVolumeArc, b.twistVolumeApex)
						withoutSeam := chordedBoundaryVolumeAllow(b.sectionDelta, wallAreaUpper, twistVolumeUpper, b.capVolumeUpper, 0)
						ratio := withoutSeam / measuredGap
						if ratio < minRatio {
							minRatio = ratio
							minRow = fmt.Sprintf("pie r=%g sweep=%g h=%g twist=%g n=%d", r, sweepDeg, h, twistDeg, n)
						}
					}
				}
			}
		}
	}

	// The ring family has no apex/radial cells at all, so it exercises the
	// wall/twist/cap/seam interplay without the pie slice's own unrefined
	// corners diluting it (ringAllow's own doc comment).
	ringRadii := []float64{1, 10, 50}
	ringHeights := []float64{0.1, 25, 1000}
	ringTwistsDeg := []float64{0, 5, 20, 90}
	ringN := []int{8, 32, 128, 256}
	for _, r := range ringRadii {
		for _, h := range ringHeights {
			for _, twistDeg := range ringTwistsDeg {
				twistRad := twistDeg * math.Pi / 180
				for _, n := range ringN {
					trueVolume := twistedPieSliceTrueVolume(r, 2*math.Pi, twistRad, h)
					verts, tris := ringMesh(r, twistRad, h, n)
					heldVolume := heldVolumeExact(verts, tris)
					measuredGap := math.Abs(trueVolume - heldVolume)
					if measuredGap <= 0 {
						continue
					}
					rows++

					matchedDelta, wallAreaUpper, twistVolumeUpper, capVolumeUpper, _ := ringAllow(r, twistRad, h, n)
					withoutSeam := chordedBoundaryVolumeAllow(matchedDelta, wallAreaUpper, twistVolumeUpper, capVolumeUpper, 0)
					ratio := withoutSeam / measuredGap
					if ratio < minRatio {
						minRatio = ratio
						minRow = fmt.Sprintf("ring r=%g h=%g twist=%g n=%d", r, h, twistDeg, n)
					}
				}
			}
		}
	}

	require.Positive(t, rows)
	require.GreaterOrEqualf(t, minRatio, 1.0,
		"deleting the seam leg alone dropped below the measured gap at %s — this IS a failing fixture: pin it as SHOWN-TO-FAIL and update the per-leg status comment above",
		minRow)
	t.Logf("without-seam worst-case ratio %.6g at %s (%d rows) — NOT shown to fail; not a proof of redundancy either (see the per-leg status comment above)", minRatio, minRow, rows)
}

// TestCellChordCurveAreaUpperEnclosesTheFlatTriangleCounterexample pins F1's
// own counterexample cell: vLo=(0,0,0) vHi=(1,0,0) wLo=(0,1,h) wHi=(0,0,h).
// The cell's HELD flat-triangle facet area is exactly h — vanishingly small
// at small h — while its own bilinear RULED patch already carries area
// 1/3 + O(h). A "held area plus excess" reading, as an earlier version of
// this bound used, cannot enclose that gap because there is no fixed held
// quantity for an excess to add to. cellChordCurveAreaUpper must publish an
// ABSOLUTE bound instead, which encloses the patch's own area directly, at
// every h.
func TestCellChordCurveAreaUpperEnclosesTheFlatTriangleCounterexample(t *testing.T) {
	vLo := r3.NewVec(0, 0, 0)
	vHi := r3.NewVec(1, 0, 0)
	for _, h := range []float64{0.1, 0.01, 0.001} {
		t.Run(fmt.Sprintf("h=%g", h), func(t *testing.T) {
			wLo := r3.NewVec(0, 1, h)
			wHi := r3.NewVec(0, 0, h)

			// Both sides are straight LineSegs (sectionDelta=0): each
			// side's own arc length equals its own chord length exactly.
			arcLenA := vHi.Sub(vLo).Len()
			arcLenB := wHi.Sub(wLo).Len()

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

	arcLenA := vHi.Sub(vLo).Len()
	arcLenB := wHi.Sub(wLo).Len()

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
// -proven derivation uses for the SAME bilinear patch, so the two must
// publish the identical eA*eB product for the same four corners.
func TestCellChordCurveAreaUpperReducesToTheTwistBoundAtZeroSectionDelta(t *testing.T) {
	vLo := r3.NewVec(0, 0, 0)
	vHi := r3.NewVec(1, 0, 0)
	wLo := r3.NewVec(0, 0, 1)
	wHi := r3.NewVec(0, 1, 1)

	arcLenA := vHi.Sub(vLo).Len()
	arcLenB := wHi.Sub(wLo).Len()
	eA := math.Max(arcLenA, arcLenB)
	eB := math.Max(wLo.Sub(vLo).Len(), wHi.Sub(vHi).Len())
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
	chordA := vHi.Sub(vLo).Len()
	chordB := wHi.Sub(wLo).Len()

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
// straight through `arcLenUpperA < vHi.Sub(vLo).Len()` (NaN compares false
// against everything, so the range gate never refuses) and propagated
// through eBBase's own math.Max into a silently-NaN answer for a cell whose
// own geometry is unstateable, rather than a refusing +Inf — an unchecked
// caller that widens its own bound by `answer > 0` treats NaN exactly like
// 0, since `NaN > 0` is false either way. Every one of the four corners, in
// isolation, must now trigger a genuine +Inf refusal.
func TestCellChordCurveAreaUpperRefusesNonFiniteCorners(t *testing.T) {
	vLo := r3.NewVec(0, 0, 0)
	vHi := r3.NewVec(1, 0, 0)
	wLo := r3.NewVec(0, 0, 1)
	wHi := r3.NewVec(0, 1, 1)
	nan := r3.NewVec(math.NaN(), 0, 0)
	chordA := vHi.Sub(vLo).Len()
	chordB := wHi.Sub(wLo).Len()

	require.True(t, math.IsInf(cellChordCurveAreaUpper(nan, vHi, wLo, wHi, chordA, chordB, 0), 1), "NaN vLo")
	require.True(t, math.IsInf(cellChordCurveAreaUpper(vLo, nan, wLo, wHi, chordA, chordB, 0), 1), "NaN vHi")
	require.True(t, math.IsInf(cellChordCurveAreaUpper(vLo, vHi, nan, wHi, chordA, chordB, 0), 1), "NaN wLo")
	require.True(t, math.IsInf(cellChordCurveAreaUpper(vLo, vHi, wLo, nan, chordA, chordB, 0), 1), "NaN wHi")
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

// TestArcMatchedDeltaEqualsSagitta pins Step 2's own carried-forward
// verification: for a CIRCULAR ARC under its own uniform-angle
// parametrization, sup_s|arc(s)-chord(s)| equals the chord's own TRUE
// sagitta EXACTLY — the one curve kind (besides a trivial straight LINE)
// where cellChordCurveAreaUpper's matchedDeltaUpper obligation and the
// loft evaluator's sagitta-only sectionDelta field coincide, over sweeps
// from 5 to 170 degrees. An arc of radius r subtending angle theta about
// its own centre has, at the uniform-angle parameter s in [0,1], true
// point r*(cos(s*theta), sin(s*theta)) and chord point (1-s)*(r,0) +
// s*(r*cos(theta), r*sin(theta)); the maximum separation over s is a
// standard result, r*(1-cos(theta/2)) BETWEEN the endpoints — the SAME
// value as 2r*sin(theta/4)^2 (2*sin(x/2)^2 = 1-cos(x) is the
// numerically-stable identity) — confirmed here by direct numerical
// maximisation over s rather than trusted algebraically. That TRUE
// closed-form value, trueSagitta below, is computed independently of
// chordSagitta: chordSagitta itself now publishes a PROVEN UPPER bound
// (sin(x)<=x, this file's own item 2) rather than the tight closed form,
// so it is no longer the arc's exact matched-parameter deviation — it
// still bounds it, which the second assertion below pins, but the
// coincidence this test's own name refers to is a fact about the TRUE
// mathematical quantities, not about chordSagitta's own numeric output.
func TestArcMatchedDeltaEqualsSagitta(t *testing.T) {
	const radius = 7.0
	for _, sweepDeg := range []float64{5, 10, 30, 60, 90, 120, 150, 170} {
		t.Run(fmt.Sprintf("sweep=%gdeg", sweepDeg), func(t *testing.T) {
			theta := sweepDeg * math.Pi / 180
			trueSagitta := 2 * radius * math.Sin(theta/4) * math.Sin(theta/4)

			chordStart := r3.NewVec(radius, 0, 0)
			chordEnd := r3.NewVec(radius*math.Cos(theta), radius*math.Sin(theta), 0)

			maxSep := 0.0
			const steps = 200000
			for i := 0; i <= steps; i++ {
				s := float64(i) / steps
				truePoint := r3.NewVec(radius*math.Cos(s*theta), radius*math.Sin(s*theta), 0)
				chordPoint := chordStart.Scale(1 - s).Add(chordEnd.Scale(s))
				sep := truePoint.Sub(chordPoint).Len()
				maxSep = math.Max(maxSep, sep)
			}

			// A fine but finite numerical maximisation converges toward the
			// true supremum from below, so this checks near-equality rather
			// than an exact match — HOST PORTABILITY: no literal from this
			// run is pinned, only a same-run comparison against the TRUE
			// closed form computed above.
			require.InDeltaf(t, trueSagitta, maxSep, trueSagitta*1e-4+1e-9,
				"sweep=%g: the arc-vs-chord max separation %.10g must match the true sagitta %.10g under the matched parametrization", sweepDeg, maxSep, trueSagitta)

			// chordSagitta's own PROVEN bound (sin(x)<=x, no longer the tight
			// closed form) must still enclose the true parameter-matched
			// deviation, so it remains valid to pass as matchedDeltaUpper for
			// an arc pairing even though it is no longer exactly equal to it.
			bound := chordSagitta(radius, theta, 1)
			require.GreaterOrEqualf(t, bound, trueSagitta,
				"sweep=%g: chordSagitta's own proven bound %.10g must enclose the true sagitta %.10g", sweepDeg, bound, trueSagitta)
		})
	}
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

// TestCellTwistOffsetUpperMatchesTheTwistVolumeAllowPointwiseTerm pins that
// cellTwistOffsetUpper is exactly cellTwistVolumeAllow's own internal |T|/4
// term, taken alone, for the SAME cell TestCellTwistVolumeAllowScalesWithTheTwistVector
// already hand-checks.
func TestCellTwistOffsetUpperMatchesTheTwistVolumeAllowPointwiseTerm(t *testing.T) {
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

// TestChordedBoundaryMomentAllowIsItsOwnWidenedTwin pins
// chordedBoundaryMomentAllow's own closed form —
// chordedBoundaryVolumeAllow(...) composed with coordUpper WIDENED by
// matchedDelta and maxTwistOffsetUpper via absSumUpper, the same pattern
// loft_moments.go:265's sweptMomentAllow call site widens by m.delta —
// computed here by calling chordedBoundaryVolumeAllow directly, never
// sweptMomentAllow, so a caller that swapped the two internally would move
// this answer.
func TestChordedBoundaryMomentAllowIsItsOwnWidenedTwin(t *testing.T) {
	matchedDelta, wallAreaUpper, twistVolumeUpper, capVolumeUpper, seamAllow := 0.02, 7.5, 1.25, 0.5, 0.1
	maxTwistOffsetUpper, coordUpper := 0.3, 3.0

	vol := chordedBoundaryVolumeAllow(matchedDelta, wallAreaUpper, twistVolumeUpper, capVolumeUpper, seamAllow)
	widened := absSumUpper(coordUpper, matchedDelta, maxTwistOffsetUpper)
	want := productUpper(vol, widened)

	got := chordedBoundaryMomentAllow(matchedDelta, wallAreaUpper, twistVolumeUpper, capVolumeUpper, seamAllow, maxTwistOffsetUpper, coordUpper)
	require.Equal(t, want, got)

	require.Equal(t, 0.0, chordedBoundaryMomentAllow(0, wallAreaUpper, 0, 0, 0, 0, coordUpper))
	require.Equal(t, 0.0, chordedBoundaryMomentAllow(matchedDelta, wallAreaUpper, twistVolumeUpper, capVolumeUpper, seamAllow, maxTwistOffsetUpper, 0))
}

// TestChordedBoundaryMomentAllowWidensPastTheHeldCoordEnvelope pins F7
// directly: the symmetric difference this term bounds the moment of extends
// OUTSIDE every held vertex, so a coordUpper read only over the HELD
// material (never widened) must publish a SMALLER answer than the widened
// term whenever matchedDelta or maxTwistOffsetUpper is positive — an
// earlier version of this bound charged the held envelope alone and so
// understated the obligation.
func TestChordedBoundaryMomentAllowWidensPastTheHeldCoordEnvelope(t *testing.T) {
	const matchedDelta, wallAreaUpper, twistVolumeUpper, capVolumeUpper, seamAllow = 0.02, 7.5, 1.25, 0.5, 0.1
	const maxTwistOffsetUpper, coordUpper = 0.3, 3.0

	widenedAnswer := chordedBoundaryMomentAllow(matchedDelta, wallAreaUpper, twistVolumeUpper, capVolumeUpper, seamAllow, maxTwistOffsetUpper, coordUpper)
	vol := chordedBoundaryVolumeAllow(matchedDelta, wallAreaUpper, twistVolumeUpper, capVolumeUpper, seamAllow)
	unwidenedAnswer := productUpper(vol, coordUpper)

	require.Greater(t, widenedAnswer, unwidenedAnswer,
		"the widened term must exceed the SAME product taken over the held coordUpper alone")
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
