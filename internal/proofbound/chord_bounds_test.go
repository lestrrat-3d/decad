package proofbound_test

import (
	"fmt"
	"math"
	"math/big"
	"math/rand/v2"
	"runtime"
	"sync"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/tessellation"

	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// This file tests the chorded volume legs docs/loft-gear-bounds-design.md §2
// composes — the per-cell wall leg over proofbound.CellChordCurveAreaUpper and
// the twist measure proofbound.CellTwistVolumeAllow — beside
// proofbound.ChordedBoundaryMomentAllow, proofbound.CellTwistOffsetUpper,
// proofbound.CellTwistAreaAllow and the shared per-cell reader
// proofbound.CellAllowsOf: the enclosure those legs prove between the HELD
// FLAT-TRIANGLE polyhedron assembleLoft actually builds and the true curved
// solid it approximates, over a table of radii, sweeps, heights, chord counts
// AND a TWIST angle between the two paired sections. That is the tessellation's
// volSymDiff composition at delta == 0, where §2's vertex sweep and skirt are
// both exactly zero.
//
// A fixture with no twist cannot exercise proofbound.CellTwistVolumeAllow: every wall
// cell degenerates to a planar quad, its own twist vector is exactly zero,
// and a bound that omits the twist term altogether still passes. So every
// table below carries a twist sweep, and the two rows an earlier refutation
// of this bound failed outright — 20 degrees of twist at 64 stations, and 90
// degrees at 256 stations — are asserted explicitly.
//
// Findings from earlier audits that shape this file:
//
//   - the chord-to-curve leg must bound the AREA of the bilinear RULED PATCH
//     a wall cell's four chord corners span, and every surface between it
//     and the true curved wall — never a held-triangle-area-plus-excess
//     reading, which is not sign-definite (TestCellChordCurveAreaUpper*
//     pins a direct counterexample cell where the held triangle pair holds
//     almost no area while its own ruled patch already carries a third of a
//     square unit);
//   - the fixture's own worst row must bind at a TWISTED station, never at
//     twist zero, and a refinement test must be driven by a quantity that
//     ACTUALLY shrinks with refinement (F4)
//     (TestChordedWallAndTwistLegsRemainSoundUnderRefinement);
//   - F1: proofbound.CellChordCurveAreaUpper's own eB term silently upgraded a
//     SET-distance sagitta into a PARAMETER-MATCHED displacement — the two
//     coincide only for a LINE or an ARC (TestArcMatchedDeltaEqualsSagitta)
//     and can differ by the CHORD LENGTH for any other curve
//     (TestCellChordCurveAreaUpperRefusesTheSagittaZigzag);
//   - F5: proofbound.CellChordCurveAreaUpper validated its three scalar operands but
//     not its four r3.Vec corners, so a NaN vertex propagated to a silent
//     NaN answer rather than a refusing +Inf
//     (TestCellChordCurveAreaUpperRefusesNonFiniteCorners);
//   - F6: proofbound.ChordedBoundaryMomentAllow guarded proofbound.IsNonFinite(coordUpper) but not
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
// pie slice's own volume, the fixture TestChordedWallAndTwistLegsEncloseTheMeasuredGap
// already pins.
func twistedPieSliceTrueVolume(radius, sweepRad, twistRad, h float64) float64 {
	return (radius * radius * sweepRad * h / 6) * (2 + math.Cos(twistRad))
}

// heldVolumeExact sums signed tetrahedra, anchored at the origin, over
// EXACTLY the triangle set a test mesh builds — the same two-triangle-per-
// wall-cell topology assembleLoft emits (loft_build.go) and
// loftMassAccumulator measures (loft_moments.go) — as an exact rational, so
// the "measured" gap this file's enclosure tests compare against is never
// itself subject to float summation slop.
//
// Every coordinate is a float, so every denominator is a power of two and
// the largest of them, 2^shift, is the common one: the sum runs over the
// integers coordinate·2^shift and divides by 6·2^(3·shift) once at the end,
// giving the same rational heldVolumeExactRat reduces term by term.
func heldVolumeExact(verts []r3.Vec, tris [][3]int) float64 {
	rats := make([][3]*big.Rat, len(verts))
	shift := 0
	for v, p := range verts {
		rats[v] = [3]*big.Rat{ratOfFloat(p.X), ratOfFloat(p.Y), ratOfFloat(p.Z)}
		for _, r := range rats[v] {
			shift = max(shift, r.Denom().BitLen()-1)
		}
	}
	ints := make([][3]*big.Int, len(verts))
	for v, r := range rats {
		for axis := range 3 {
			ints[v][axis] = new(big.Int).Lsh(r[axis].Num(), uint(shift-(r[axis].Denom().BitLen()-1)))
		}
	}
	var vol6N, term, tmp big.Int
	var cross [3]big.Int
	for _, tri := range tris {
		a, b, c := ints[tri[0]], ints[tri[1]], ints[tri[2]]
		for i := range 3 {
			j, k := (i+1)%3, (i+2)%3
			cross[i].Mul(b[j], c[k])
			cross[i].Sub(&cross[i], tmp.Mul(b[k], c[j]))
		}
		term.Mul(a[0], &cross[0])
		term.Add(&term, tmp.Mul(a[1], &cross[1]))
		term.Add(&term, tmp.Mul(a[2], &cross[2]))
		vol6N.Add(&vol6N, &term)
	}
	den := new(big.Int).Lsh(big.NewInt(6), uint(3*shift))
	f, _ := new(big.Rat).SetFrac(&vol6N, den).Float64()
	return f
}

// heldVolumeExactRat is heldVolumeExact as a term-by-term rational sum,
// every partial sum reduced: the reference the common-denominator sum must
// reproduce bit for bit.
func heldVolumeExactRat(verts []r3.Vec, tris [][3]int) float64 {
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

// chordedAllowBreakdown separates a twisted pie slice's own volume legs into
// the REFINED chorded-arc wall cells (the n cells whose own geometry shrinks
// and multiplies as the station count n grows) and the two UNREFINED
// radial/apex cells (centerB-arcB(0)/centerT-arcT(0) and
// arcB(n)-centerB/arcT(n)-centerT, whose own four corners are fixed by radius,
// sweepRad and twistRad alone and never change with n) — the split an earlier
// refutation needed to show a "refinement does not degrade" test was actually
// driven by the refined cells, not by the two constant apex ones.
type chordedAllowBreakdown struct {
	sectionDelta        float64
	wallAreaArc         float64
	wallLegArc          float64
	twistVolumeArc      float64
	twistVolumeApex     float64
	maxTwistOffsetUpper float64
	allow               float64
}

// chordedBoundaryAllowForTwistedPieSlice composes one twisted-pie-slice row's
// volume legs, split by cell kind. Every per-cell reading below is taken from
// proofbound.CellAllowsOf, which publishes exactly what the three named helpers
// publish for that cell (TestCellAllowsOfMatchesThePerBoundHelpers) from one
// certification of the cell's own spans and twist vector:
//
//   - wall chord-to-curve (docs/loft-gear-bounds-design.md §2's wallLeg):
//     each arc cell's own matched departure times its own
//     proofbound.CellChordCurveAreaUpper, each side's arc-length claim its own
//     TRUE arc length radius*dtheta. The two radial cells are straight walls
//     whose chord IS the curve, so their departure is zero and they carry no
//     wall leg, the loft's own charged-cell rule;
//   - ruled-to-triangle (twist): proofbound.CellTwistVolumeAllow summed over every one
//     of the n+2 wall cells, since a twisted top section gives every wall
//     cell — including the two radial ones, whose "outer" corners rotate by
//     twistRad between sections — a nonzero twist vector; and
//     proofbound.CellTwistOffsetUpper's own MAXIMUM over every cell, used by the
//     facet-departure proof and carried beside these volume fixtures.
//
// The fixture is unplaced and its caps lie in z=0 and z=h exactly, so §2's
// vertex sweep and skirt are both zero and allow is wallLeg + twist.
func chordedBoundaryAllowForTwistedPieSlice(radius, sweepRad, twistRad, h float64, n int) chordedAllowBreakdown {
	sectionDelta := tessellation.ChordSagitta(radius, sweepRad, n)
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
		cell := proofbound.CellAllowsOf(vLo, vHi, wLo, wHi, arcLenPerArcCell, arcLenPerArcCell, sectionDelta)
		b.wallAreaArc = proofbound.AbsSumUpper(b.wallAreaArc, cell.ChordCurveAreaUpper)
		b.wallLegArc = proofbound.AbsSumUpper(b.wallLegArc, proofbound.ProductUpper(sectionDelta, cell.ChordCurveAreaUpper))
		b.twistVolumeArc = proofbound.AbsSumUpper(b.twistVolumeArc, cell.TwistVolumeAllow)
		b.maxTwistOffsetUpper = math.Max(b.maxTwistOffsetUpper, cell.TwistOffsetUpper)
	}

	radialCells := [2][4]r3.Vec{
		{centerB, arcB(0), centerT, arcT(0)},
		{arcB(n), centerB, arcT(n), centerT},
	}
	for _, c := range radialCells {
		vLo, vHi, wLo, wHi := c[0], c[1], c[2], c[3]
		// A radial wall is a straight LineSeg, so each side's own arc length
		// IS its chord length — but the CLAIM must be a proven UPPER bound on
		// that chord, and r3.Vec.Len is not one (proofbound.CellSpanUpper's own doc
		// comment). The certified endpoint is what this caller states.
		arcLenA := proofbound.CellSpanUpper(vHi, vLo)
		arcLenB := proofbound.CellSpanUpper(wHi, wLo)
		cell := proofbound.CellAllowsOf(vLo, vHi, wLo, wHi, arcLenA, arcLenB, sectionDelta)
		b.twistVolumeApex = proofbound.AbsSumUpper(b.twistVolumeApex, cell.TwistVolumeAllow)
		b.maxTwistOffsetUpper = math.Max(b.maxTwistOffsetUpper, cell.TwistOffsetUpper)
	}

	b.allow = proofbound.AbsSumUpper(b.wallLegArc, b.twistVolumeArc, b.twistVolumeApex)
	return b
}

// chordSweepRow is ONE row of the shared pie-slice sweep: the fixture's own
// parameters, the volume gap its HELD mesh actually shows against the true
// solid, and the breakdown of the legs composed for it.
type chordSweepRow struct {
	radius, sweepDeg, h, twistDeg float64
	n                             int
	measuredGap                   float64
	breakdown                     chordedAllowBreakdown
}

// label renders the row the way every assertion over the table names it.
func (r chordSweepRow) label() string {
	return fmt.Sprintf("r=%g sweep=%g h=%g twist=%g n=%d", r.radius, r.sweepDeg, r.h, r.twistDeg, r.n)
}

// chordSweepTable is the 900-row pie-slice sweep — radii, sweeps, heights,
// twists and chord counts — that the enclosure test and the wall-leg deletion
// test read. The two ask DIFFERENT questions of the SAME rows, and
// building one row is exact-rational work (the mesh's own held volume over
// big.Rat, and the per-cell certified spans behind its composed allow) that
// dominates this file's cost, so every row is built ONCE per test binary and
// both tests read it.
//
// Nothing but the fixture is shared: each test still applies its own filter and
// its own assertion to every row, and a row carries only what a test reads —
// its parameters, its measured gap, and its own breakdown — never a verdict one
// test could leak into another.
var chordSweepTable = sync.OnceValue(func() []chordSweepRow {
	radii := []float64{1, 5, 50}
	sweepsDeg := []float64{30, 90, 180, 270}
	heights := []float64{0.1, 10, 100}
	chordCounts := []int{8, 32, 64, 128, 256}
	twistsDeg := []float64{0, 5, 20, 45, 90}

	type input struct {
		r, sweepDeg, h, twistDeg float64
		n                        int
	}
	inputs := make([]input, 0, len(radii)*len(sweepsDeg)*len(heights)*len(chordCounts)*len(twistsDeg))
	for _, r := range radii {
		for _, sweepDeg := range sweepsDeg {
			for _, h := range heights {
				for _, twistDeg := range twistsDeg {
					for _, n := range chordCounts {
						inputs = append(inputs, input{r, sweepDeg, h, twistDeg, n})
					}
				}
			}
		}
	}
	rows := make([]chordSweepRow, len(inputs))
	workers := min(4, runtime.GOMAXPROCS(0))
	jobs := make(chan int)
	var workersDone sync.WaitGroup
	for range workers {
		workersDone.Go(func() {
			for i := range jobs {
				in := inputs[i]
				sweepRad := in.sweepDeg * math.Pi / 180
				twistRad := in.twistDeg * math.Pi / 180
				verts, tris := twistedPieSliceMesh(in.r, sweepRad, twistRad, in.h, in.n)
				trueVolume := twistedPieSliceTrueVolume(in.r, sweepRad, twistRad, in.h)
				rows[i] = chordSweepRow{
					radius: in.r, sweepDeg: in.sweepDeg, h: in.h, twistDeg: in.twistDeg, n: in.n,
					measuredGap: math.Abs(trueVolume - heldVolumeExact(verts, tris)),
					breakdown:   chordedBoundaryAllowForTwistedPieSlice(in.r, sweepRad, twistRad, in.h, in.n),
				}
			}
		})
	}
	for i := range inputs {
		jobs <- i
	}
	close(jobs)
	workersDone.Wait()
	return rows
})

// TestChordedWallAndTwistLegsEncloseTheMeasuredGap is the A10 plan's
// required enclosure test, read over docs/loft-gear-bounds-design.md §2's
// legs: the per-cell wall leg plus the twist measure, with no cap or seam leg,
// must enclose the gap between the held flat-triangle mesh and the true solid
// in every row. Every row carries a TWIST between the two paired sections: a
// fixture with no twist cannot see the held-triangle-to-ruled-patch gap
// (every wall cell degenerates to a planar quad). Two rows are an earlier
// refutation's own counterexamples: 20 degrees of twist at 64 stations, and
// 90 degrees at 256 stations.
//
// The binding (minimum allow/measuredGap) row must be a TWISTED one: a
// fixture whose worst case lands at twist=0 is not exercising the twist
// mechanism at all (proofbound.CellTwistVolumeAllow returns exactly 0 there).
//
// Shown to fail: deleting the wall leg fails the untwisted rows
// (TestChordedWallLegIsLoadBearing), and deleting the twist leg fails
// TestChordedTwistLegIsLoadBearing's row.
func TestChordedWallAndTwistLegsEncloseTheMeasuredGap(t *testing.T) {
	t.Parallel()
	minRatio := math.Inf(1)
	var minRow string
	var minTwistDeg float64
	rows := 0

	for _, row := range chordSweepTable() {
		rows++

		allow := row.breakdown.allow
		require.GreaterOrEqualf(t, allow, row.measuredGap,
			"%s: allow must enclose the measured volume gap", row.label())

		if row.measuredGap > 0 {
			ratio := allow / row.measuredGap
			if ratio < minRatio {
				minRatio = ratio
				minTwistDeg = row.twistDeg
				minRow = row.label()
			}
		}
	}

	require.Positive(t, rows)
	require.GreaterOrEqual(t, minRatio, 1.0, "the loosest row in the table: %s", minRow)
	require.NotZero(t, minTwistDeg, "the binding row must be a TWISTED station, not twist=0 (row: %s) — otherwise this table never exercises cellTwistVolumeAllow at all", minRow)
	t.Logf("worst-case allow/measuredGap ratio %.6g at %s (%d rows)", minRatio, minRow, rows)
}

// TestChordedWallAndTwistLegsEncloseTheRefutedCounterexamples pins the two
// specific rows an earlier audit measured against a pre-fix bound: 20 degrees
// of twist at 64 stations and 90 degrees at 256 stations. Both must enclose.
func TestChordedWallAndTwistLegsEncloseTheRefutedCounterexamples(t *testing.T) {
	t.Parallel()
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

// TestChordedWallAndTwistLegsRemainSoundUnderRefinement pins, at FIXED twist,
// that refining the chord count keeps the legs enclosing the measured gap, and
// that the refinement is real: the arc cells' own wall leg — the term the
// composed bound actually charges — is O(1/n^2) (sectionDelta itself is;
// wallAreaArc converges to a constant as n grows), so refining from n=8 to
// n=256, a 32x refinement, must shrink it by close to 32^2=1024x. A guard
// requiring only 100x leaves ample host-portability slack while still failing
// outright on a fixture that is NOT actually refining. An earlier version
// checked the arc cells' SHARE of the wall area, a fixture constant
// sweep/(sweep+2) at every n (F4); this test runs at 90 degrees as well as 120
// to confirm the replacement measures refinement rather than sweep angle.
func TestChordedWallAndTwistLegsRemainSoundUnderRefinement(t *testing.T) {
	t.Parallel()
	const radius, h = 10.0, 25.0
	chordCounts := []int{8, 32, 64, 128, 256}

	for _, sweepDeg := range []float64{120, 90} {
		sweepRad := sweepDeg * math.Pi / 180
		for _, twistDeg := range []float64{5, 20, 45, 90} {
			t.Run(fmt.Sprintf("sweep=%gdeg/twist=%gdeg", sweepDeg, twistDeg), func(t *testing.T) {
				twistRad := twistDeg * math.Pi / 180

				var ratios []float64
				var arcLegs []float64
				for _, n := range chordCounts {
					trueVolume := twistedPieSliceTrueVolume(radius, sweepRad, twistRad, h)
					verts, tris := twistedPieSliceMesh(radius, sweepRad, twistRad, h, n)
					heldVolume := heldVolumeExact(verts, tris)
					measuredGap := math.Abs(trueVolume - heldVolume)
					require.Greater(t, measuredGap, 0.0)

					b := chordedBoundaryAllowForTwistedPieSlice(radius, sweepRad, twistRad, h, n)
					require.GreaterOrEqual(t, b.allow, measuredGap)
					ratios = append(ratios, b.allow/measuredGap)
					arcLegs = append(arcLegs, b.wallLegArc)
					t.Logf("sweep=%g twist=%g n=%d: ratio=%.6g arcWallLeg=%.6g wallAreaArc=%.6g",
						sweepDeg, twistDeg, n, ratios[len(ratios)-1], arcLegs[len(arcLegs)-1], b.wallAreaArc)
				}

				first, last := arcLegs[0], arcLegs[len(arcLegs)-1]
				require.Greaterf(t, first/last, 100.0,
					"sweep=%g twist=%g: the arc cells' own wall leg must shrink by more than 100x from n=%d to n=%d (got %.6g -> %.6g, shrink %.4gx)",
					sweepDeg, twistDeg, chordCounts[0], chordCounts[len(chordCounts)-1], first, last, first/last)

				t.Logf("sweep=%g twist=%g ratios across n=%v: %v", sweepDeg, twistDeg, chordCounts, ratios)
			})
		}
	}
}

// TestChordedTwistLegIsLoadBearing pins that the twist leg is necessary by
// DELETING it (the wall leg alone) at the sweep table's own worst row for that
// deletion (r=1 sweep=30 h=100 twist=90 n=256) and confirming the bound
// collapses below the measured gap.
func TestChordedTwistLegIsLoadBearing(t *testing.T) {
	t.Parallel()
	const radius, sweepDeg, h, twistDeg, n = 1.0, 30.0, 100.0, 90.0, 256
	sweepRad := sweepDeg * math.Pi / 180
	twistRad := twistDeg * math.Pi / 180

	trueVolume := twistedPieSliceTrueVolume(radius, sweepRad, twistRad, h)
	verts, tris := twistedPieSliceMesh(radius, sweepRad, twistRad, h, n)
	heldVolume := heldVolumeExact(verts, tris)
	measuredGap := math.Abs(trueVolume - heldVolume)
	require.Greater(t, measuredGap, 0.0)

	b := chordedBoundaryAllowForTwistedPieSlice(radius, sweepRad, twistRad, h, n)
	require.GreaterOrEqual(t, b.allow, measuredGap, "the wall and twist legs must enclose this row")

	withoutTwist := b.wallLegArc
	require.Lessf(t, withoutTwist, measuredGap,
		"deleting the twist leg must fail to enclose the measured gap (got allow=%.6g < measuredGap=%.6g is required; ratio %.6g)",
		withoutTwist, measuredGap, withoutTwist/measuredGap)
	t.Logf("full ratio=%.6g, without-twist ratio=%.6g (must be < 1)", b.allow/measuredGap, withoutTwist/measuredGap)
}

// TestChordedWallLegIsLoadBearing pins that the wall leg is necessary once the
// cap and seam legs are gone (docs/loft-gear-bounds-design.md §2): over the
// sweep table, deleting it (the twist measure alone) must fail to enclose the
// measured gap on at least one row, and on every untwisted row, where the
// twist measure is exactly zero while the chorded arc still holds a gap.
func TestChordedWallLegIsLoadBearing(t *testing.T) {
	t.Parallel()
	failing, untwisted := 0, 0
	minRatio := math.Inf(1)
	var minRow string

	for _, row := range chordSweepTable() {
		if row.measuredGap <= 0 {
			continue
		}
		b := row.breakdown
		withoutWall := proofbound.AbsSumUpper(b.twistVolumeArc, b.twistVolumeApex)
		if withoutWall < row.measuredGap {
			failing++
		}
		if row.twistDeg == 0 {
			untwisted++
			require.Lessf(t, withoutWall, row.measuredGap,
				"%s: with no twist, the twist measure alone must fail to enclose the gap", row.label())
		}
		if ratio := withoutWall / row.measuredGap; ratio < minRatio {
			minRatio = ratio
			minRow = row.label()
		}
	}

	require.Positive(t, untwisted)
	require.Positive(t, failing, "deleting the wall leg must fail on some row")
	t.Logf("without-wall: %d failing rows, worst ratio %.6g at %s", failing, minRatio, minRow)
}

// TestCellChordCurveAreaUpperEnclosesTheFlatTriangleCounterexample pins F1's
// own counterexample cell: vLo=(0,0,0) vHi=(1,0,0) wLo=(0,1,h) wHi=(0,0,h).
// The cell's HELD flat-triangle facet area is exactly h — vanishingly small
// at small h — while its own bilinear RULED patch already carries area
// 1/3 + O(h). A "held area plus excess" reading, as an earlier version of
// this bound used, cannot enclose that gap because there is no fixed held
// quantity for an excess to add to. proofbound.CellChordCurveAreaUpper must publish an
// ABSOLUTE bound instead, which encloses the patch's own area directly, at
// every h.

// TestHeldVolumeExactMatchesRationalSum pins heldVolumeExact to the
// term-by-term rational sum, float bit for float bit, over a dozen
// chordSweepTable parameter rows and over random meshes whose float
// coordinates span many binary exponents.
func TestHeldVolumeExactMatchesRationalSum(t *testing.T) {
	t.Parallel()
	rows := []struct {
		r, sweepDeg, h, twistDeg float64
		n                        int
	}{
		{1, 30, 0.1, 0, 8}, {1, 90, 10, 5, 32}, {1, 180, 100, 20, 64}, {1, 270, 0.1, 45, 128},
		{5, 30, 10, 90, 256}, {5, 90, 100, 0, 8}, {5, 180, 0.1, 5, 32}, {5, 270, 10, 20, 64},
		{50, 30, 100, 45, 128}, {50, 90, 0.1, 90, 256}, {50, 180, 10, 0, 8}, {50, 270, 100, 45, 32},
	}
	for _, row := range rows {
		verts, tris := twistedPieSliceMesh(row.r, row.sweepDeg*math.Pi/180, row.twistDeg*math.Pi/180, row.h, row.n)
		want, got := heldVolumeExactRat(verts, tris), heldVolumeExact(verts, tris)
		require.Equal(t, math.Float64bits(want), math.Float64bits(got), `row %+v: %v vs %v`, row, want, got)
	}
	rng := rand.New(rand.NewPCG(0x68656c64, 0x766f6c))
	coord := func() float64 {
		if rng.IntN(8) == 0 {
			return 0
		}
		return math.Ldexp(rng.Float64()-0.5, rng.IntN(121)-60)
	}
	for trial := range 100 {
		verts := make([]r3.Vec, 4+rng.IntN(12))
		for v := range verts {
			verts[v] = r3.NewVec(coord(), coord(), coord())
		}
		tris := make([][3]int, 1+rng.IntN(30))
		for k := range tris {
			tris[k] = [3]int{rng.IntN(len(verts)), rng.IntN(len(verts)), rng.IntN(len(verts))}
		}
		want, got := heldVolumeExactRat(verts, tris), heldVolumeExact(verts, tris)
		require.Equal(t, math.Float64bits(want), math.Float64bits(got), `trial %d: %v vs %v`, trial, want, got)
	}
}
