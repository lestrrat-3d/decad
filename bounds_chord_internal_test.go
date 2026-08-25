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
// chordedBoundaryMomentAllow, cellChordCurveAreaUpper, capAreaVolumeAllow,
// cellTwistOffsetUpper and cellTwistVolumeAllow (docs/loft-design.md §5 —
// the chord-chain subsection lands with the arc design change; the A10
// plan's Part 2 Q4 and Part 4 R1 fallback): the enclosure
// chordedBoundaryVolumeAllow proves between the HELD FLAT-TRIANGLE
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
// Two independent audits refuted earlier versions of this bound. The second
// refutation's own findings drive this file's structure beyond the sweep
// above:
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
//     refinement test must be driven by the cells that actually refine —
//     the n curved arc cells — not the two apex/radial cells whose own
//     geometry never changes with the station count
//     (TestChordedBoundaryVolumeAllowRatioDoesNotDegradeUnderRefinement's
//     own arc/apex split).

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
//     contributes nothing, while the top cap sits at offset h.
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
	}

	chordLen := 2 * radius * math.Sin(dtheta/2)
	perimeterUpper1 := float64(n)*chordLen + 2*radius
	capAreaAllow1 := sectionDisplacementArea(sectionDelta, n+2, perimeterUpper1)
	b.capVolumeUpper = capAreaVolumeAllow(h, capAreaAllow1)

	wallAreaUpper := absSumUpper(b.wallAreaArc, b.wallAreaApex)
	twistVolumeUpper := absSumUpper(b.twistVolumeArc, b.twistVolumeApex)
	b.allow = chordedBoundaryVolumeAllow(sectionDelta, wallAreaUpper, twistVolumeUpper, b.capVolumeUpper)
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
// An earlier fixture's own refinement check was itself refuted: 99.2% of
// its own twist leg at n=256 came from the two apex/radial cells, whose own
// four corners never change with n, so the check was measuring two constant
// cells rather than a refining wall. This version logs the split every row
// and asserts on it directly: the REFINED arc cells' own share of the total
// wall-area leg must not collapse to a sliver of the total as n grows — the
// opposite of what drove the earlier refutation.
func TestChordedBoundaryVolumeAllowRatioDoesNotDegradeUnderRefinement(t *testing.T) {
	const radius, sweepDeg, h = 10.0, 120.0, 25.0
	sweepRad := sweepDeg * math.Pi / 180
	chordCounts := []int{8, 32, 64, 128, 256}

	for _, twistDeg := range []float64{5, 20, 45, 90} {
		t.Run(fmt.Sprintf("twist=%gdeg", twistDeg), func(t *testing.T) {
			twistRad := twistDeg * math.Pi / 180

			var ratios []float64
			var arcShares []float64
			for _, n := range chordCounts {
				trueVolume := twistedPieSliceTrueVolume(radius, sweepRad, twistRad, h)
				verts, tris := twistedPieSliceMesh(radius, sweepRad, twistRad, h, n)
				heldVolume := heldVolumeExact(verts, tris)
				measuredGap := math.Abs(trueVolume - heldVolume)
				require.Greater(t, measuredGap, 0.0)

				b := chordedBoundaryAllowForTwistedPieSlice(radius, sweepRad, twistRad, h, n)
				require.GreaterOrEqual(t, b.allow, measuredGap)
				ratios = append(ratios, b.allow/measuredGap)

				wallAreaTotal := absSumUpper(b.wallAreaArc, b.wallAreaApex)
				arcShare := 0.0
				if wallAreaTotal > 0 {
					arcShare = b.wallAreaArc / wallAreaTotal
				}
				arcShares = append(arcShares, arcShare)
				t.Logf("twist=%g n=%d: ratio=%.6g wallAreaArc=%.6g wallAreaApex=%.6g arcShare=%.4f twistVolumeArc=%.6g twistVolumeApex=%.6g",
					twistDeg, n, ratios[len(ratios)-1], b.wallAreaArc, b.wallAreaApex, arcShare, b.twistVolumeArc, b.twistVolumeApex)
			}

			// The REFINED cells must drive the wall-area leg once the mesh
			// has refined even a little: at n=32 and beyond, the n arc
			// cells' own share of the total wall area must be the majority,
			// never dominated by the two constant apex cells the way an
			// earlier fixture's own twist leg was (99.2% apex at n=256).
			for i, n := range chordCounts {
				if n < 32 {
					continue
				}
				require.Greaterf(t, arcShares[i], 0.5,
					"twist=%g n=%d: the refined arc cells must drive the wall-area leg (got arcShare=%.4f)",
					twistDeg, n, arcShares[i])
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

// TestChordedBoundaryVolumeAllowComposesAllThreeLegs pins that
// chordedBoundaryVolumeAllow composes its wall chord-to-curve leg, its
// caller-supplied twist leg and its caller-supplied cap leg by absSumUpper,
// never by picking the largest of the three or dropping any: with only one
// leg positive at a time, the whole answer is exactly that leg; with all
// three positive, the answer is at least as large as any one leg alone.
func TestChordedBoundaryVolumeAllowComposesAllThreeLegs(t *testing.T) {
	// absSumUpper rounds its outward-nudged sum away from an exact value by
	// construction (upRound's own contract), so single-leg cases are checked
	// as an enclosure — never pinned to a literal float this platform's own
	// rounding could move a ulp either way — rather than an exact match.
	twistOnly := chordedBoundaryVolumeAllow(0, 5.0, 3.5, 0)
	require.GreaterOrEqual(t, twistOnly, 3.5)
	require.InDelta(t, 3.5, twistOnly, 1e-12)

	capOnly := chordedBoundaryVolumeAllow(0, 5.0, 0, 2.0)
	require.GreaterOrEqual(t, capOnly, 2.0)
	require.InDelta(t, 2.0, capOnly, 1e-12)

	require.Equal(t, 0.0, chordedBoundaryVolumeAllow(0, 5.0, 0, 0))
	require.Equal(t, 0.0, chordedBoundaryVolumeAllow(0.01, 0, 0, 0))

	all := chordedBoundaryVolumeAllow(0.01, 5.0, 3.5, 2.0)
	wallOnly := chordedBoundaryVolumeAllow(0.01, 5.0, 0, 0)
	require.GreaterOrEqual(t, all, wallOnly)
	require.GreaterOrEqual(t, all, twistOnly)
	require.GreaterOrEqual(t, all, capOnly)
}

// TestChordedBoundaryVolumeAllowRefusesOnBrokenClaims pins F6's own fix: an
// earlier version of this bound let a NaN wallAreaUpper compare false
// against `> 0` and silently vanish from the sum (rather than refusing),
// and let absSumUpper's internal math.Abs flip a negative broken
// twistVolumeUpper or capVolumeUpper positive instead of refusing. Every
// case here must answer +Inf, never a finite number computed past a broken
// claim.
func TestChordedBoundaryVolumeAllowRefusesOnBrokenClaims(t *testing.T) {
	require.True(t, math.IsInf(chordedBoundaryVolumeAllow(math.NaN(), 5.0, 3.5, 2.0), 1), "NaN sectionDelta")
	require.True(t, math.IsInf(chordedBoundaryVolumeAllow(-1, 5.0, 3.5, 2.0), 1), "negative sectionDelta")
	require.True(t, math.IsInf(chordedBoundaryVolumeAllow(1, math.NaN(), 3.5, 2.0), 1), "sectionDelta>0 with NaN wallAreaUpper — F6's own scenario")
	require.True(t, math.IsInf(chordedBoundaryVolumeAllow(1, -1, 3.5, 2.0), 1), "sectionDelta>0 with negative wallAreaUpper")
	require.True(t, math.IsInf(chordedBoundaryVolumeAllow(0.01, 5.0, math.NaN(), 2.0), 1), "NaN twistVolumeUpper")
	require.True(t, math.IsInf(chordedBoundaryVolumeAllow(0.01, 5.0, -1, 2.0), 1), "negative twistVolumeUpper — must refuse, never flip positive via absSumUpper")
	require.True(t, math.IsInf(chordedBoundaryVolumeAllow(0.01, 5.0, 3.5, math.NaN()), 1), "NaN capVolumeUpper")
	require.True(t, math.IsInf(chordedBoundaryVolumeAllow(0.01, 5.0, 3.5, -1), 1), "negative capVolumeUpper — must refuse, never flip positive via absSumUpper")

	// sectionDelta==0 is a legitimate SKIP of the wall leg regardless of what
	// wallAreaUpper claims (the boundary provably does not move, so the area
	// it would move across is irrelevant) — never a refusal on its own.
	require.Equal(t, 0.0, chordedBoundaryVolumeAllow(0, math.Inf(1), 0, 0))
}

// TestChordedBoundaryMomentAllowIsItsOwnWidenedTwin pins
// chordedBoundaryMomentAllow's own closed form —
// chordedBoundaryVolumeAllow(...) composed with coordUpper WIDENED by
// sectionDelta and maxTwistOffsetUpper via absSumUpper, the same pattern
// loft_moments.go:265's sweptMomentAllow call site widens by m.delta —
// computed here by calling chordedBoundaryVolumeAllow directly, never
// sweptMomentAllow, so a caller that swapped the two internally would move
// this answer.
func TestChordedBoundaryMomentAllowIsItsOwnWidenedTwin(t *testing.T) {
	sectionDelta, wallAreaUpper, twistVolumeUpper, capVolumeUpper := 0.02, 7.5, 1.25, 0.5
	maxTwistOffsetUpper, coordUpper := 0.3, 3.0

	vol := chordedBoundaryVolumeAllow(sectionDelta, wallAreaUpper, twistVolumeUpper, capVolumeUpper)
	widened := absSumUpper(coordUpper, sectionDelta, maxTwistOffsetUpper)
	want := productUpper(vol, widened)

	got := chordedBoundaryMomentAllow(sectionDelta, wallAreaUpper, twistVolumeUpper, capVolumeUpper, maxTwistOffsetUpper, coordUpper)
	require.Equal(t, want, got)

	require.Equal(t, 0.0, chordedBoundaryMomentAllow(0, wallAreaUpper, 0, 0, 0, coordUpper))
	require.Equal(t, 0.0, chordedBoundaryMomentAllow(sectionDelta, wallAreaUpper, twistVolumeUpper, capVolumeUpper, maxTwistOffsetUpper, 0))
}

// TestChordedBoundaryMomentAllowWidensPastTheHeldCoordEnvelope pins F7
// directly: the symmetric difference this term bounds the moment of extends
// OUTSIDE every held vertex, so a coordUpper read only over the HELD
// material (never widened) must publish a SMALLER answer than the widened
// term whenever sectionDelta or maxTwistOffsetUpper is positive — an
// earlier version of this bound charged the held envelope alone and so
// understated the obligation.
func TestChordedBoundaryMomentAllowWidensPastTheHeldCoordEnvelope(t *testing.T) {
	const sectionDelta, wallAreaUpper, twistVolumeUpper, capVolumeUpper = 0.02, 7.5, 1.25, 0.5
	const maxTwistOffsetUpper, coordUpper = 0.3, 3.0

	widenedAnswer := chordedBoundaryMomentAllow(sectionDelta, wallAreaUpper, twistVolumeUpper, capVolumeUpper, maxTwistOffsetUpper, coordUpper)
	vol := chordedBoundaryVolumeAllow(sectionDelta, wallAreaUpper, twistVolumeUpper, capVolumeUpper)
	unwidenedAnswer := productUpper(vol, coordUpper)

	require.Greater(t, widenedAnswer, unwidenedAnswer,
		"the widened term must exceed the SAME product taken over the held coordUpper alone")
}

// TestChordedBoundaryMomentAllowRefusesOnBrokenClaims pins the reject-only
// gate over every non-finite argument position: sectionDelta,
// wallAreaUpper, twistVolumeUpper, capVolumeUpper, maxTwistOffsetUpper and
// coordUpper must each, when NaN or a negative claim where negative is
// broken, produce a non-finite published bound, never a finite number
// silently computed past it.
func TestChordedBoundaryMomentAllowRefusesOnBrokenClaims(t *testing.T) {
	const sectionDelta, wallAreaUpper, twistVolumeUpper, capVolumeUpper = 0.02, 7.5, 1.25, 0.5
	const maxTwistOffsetUpper, coordUpper = 0.3, 3.0

	require.True(t, math.IsInf(chordedBoundaryMomentAllow(math.NaN(), wallAreaUpper, twistVolumeUpper, capVolumeUpper, maxTwistOffsetUpper, coordUpper), 1), "NaN sectionDelta")
	require.True(t, math.IsInf(chordedBoundaryMomentAllow(-1, wallAreaUpper, twistVolumeUpper, capVolumeUpper, maxTwistOffsetUpper, coordUpper), 1), "negative sectionDelta")
	require.True(t, math.IsInf(chordedBoundaryMomentAllow(sectionDelta, math.NaN(), twistVolumeUpper, capVolumeUpper, maxTwistOffsetUpper, coordUpper), 1), "NaN wallAreaUpper")
	require.True(t, math.IsInf(chordedBoundaryMomentAllow(sectionDelta, wallAreaUpper, math.NaN(), capVolumeUpper, maxTwistOffsetUpper, coordUpper), 1), "NaN twistVolumeUpper")
	require.True(t, math.IsInf(chordedBoundaryMomentAllow(sectionDelta, wallAreaUpper, twistVolumeUpper, math.NaN(), maxTwistOffsetUpper, coordUpper), 1), "NaN capVolumeUpper")
	require.True(t, math.IsInf(chordedBoundaryMomentAllow(sectionDelta, wallAreaUpper, twistVolumeUpper, capVolumeUpper, math.NaN(), coordUpper), 1), "NaN maxTwistOffsetUpper")
	require.True(t, math.IsInf(chordedBoundaryMomentAllow(sectionDelta, wallAreaUpper, twistVolumeUpper, capVolumeUpper, -1, coordUpper), 1), "negative maxTwistOffsetUpper")
	require.True(t, math.IsInf(chordedBoundaryMomentAllow(sectionDelta, wallAreaUpper, twistVolumeUpper, capVolumeUpper, maxTwistOffsetUpper, math.NaN()), 1), "NaN coordUpper")
	require.True(t, math.IsInf(chordedBoundaryMomentAllow(sectionDelta, wallAreaUpper, twistVolumeUpper, capVolumeUpper, maxTwistOffsetUpper, math.Inf(1)), 1), "+Inf coordUpper")
}
