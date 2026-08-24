package decad

import (
	"fmt"
	"math"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// This file tests bounds.go's chordedBoundaryVolumeAllow,
// chordedBoundaryMomentAllow and cellRuledExcessUpper (docs/loft-design.md
// §5.2, the A10 plan's Part 2 Q4 and Part 4 R1 fallback): the enclosure
// chordedBoundaryVolumeAllow proves between a hand-chorded polyhedron and
// the true curved solid it approximates, over a table of radii, sweeps,
// heights and chord counts.

// pieSliceChordMesh builds the watertight triangle mesh of a CHORDED
// circular-sector wedge — a center point, two straight radial sides, and n
// equal-angle arc chords — straight-extruded from z=0 to z=h. Both sections
// are identical (no taper), so every wall cell's own RULE (the segment from
// its section-0 point to the corresponding section-1 point at the same
// angle) is exactly vertical, of length h.
func pieSliceChordMesh(radius, sweepRad, h float64, n int) (verts []r3.Vec, tris [][3]int) {
	const centerB, centerT = 0, 1
	arcPoint := func(i int, z float64) r3.Vec {
		theta := sweepRad * float64(i) / float64(n)
		return r3.NewVec(radius*math.Cos(theta), radius*math.Sin(theta), z)
	}
	verts = append(verts, r3.NewVec(0, 0, 0), r3.NewVec(0, 0, h))

	arcB := make([]int, n+1)
	arcT := make([]int, n+1)
	for i := range n + 1 {
		arcB[i] = len(verts)
		verts = append(verts, arcPoint(i, 0))
		arcT[i] = len(verts)
		verts = append(verts, arcPoint(i, h))
	}

	for i := range n {
		// Bottom and top caps: a triangle fan from each center.
		tris = append(tris, [3]int{centerB, arcB[i+1], arcB[i]})
		tris = append(tris, [3]int{centerT, arcT[i], arcT[i+1]})
		// The arc-chord wall quad, split the same way tessellate.go's own
		// lateral quads are.
		tris = append(tris, [3]int{arcB[i], arcB[i+1], arcT[i+1]})
		tris = append(tris, [3]int{arcB[i], arcT[i+1], arcT[i]})
	}
	// The two straight radial wall quads.
	tris = append(tris, [3]int{centerB, arcB[0], arcT[0]}, [3]int{centerB, arcT[0], centerT})
	tris = append(tris, [3]int{arcB[n], centerB, centerT}, [3]int{arcB[n], centerT, arcT[n]})

	return verts, tris
}

// TestChordedBoundaryVolumeAllowEnclosesTheMeasuredGap is the A10 plan's
// required enclosure test: chordedBoundaryVolumeAllow, fed the fallback
// route's areaUpper (perturbedAreaUpper over the chorded mesh's sectionDelta
// PLUS, summed per arc wall cell, cellRuledExcessUpper's own arc-minus-chord
// excess), must never fall below the true measured volume gap between the
// hand-chorded polyhedron and the closed-form arc-swept solid.
//
// Both volumes come from closed forms, never from a signed-tetrahedron sum
// over the mesh: the TRUE solid is a circular sector extruded by h, volume
// (1/2)·r²·sweep·h; the CHORDED solid is the n-triangle chord-polygon fan
// extruded by h, volume (n/2)·r²·sin(sweep/n)·h — the standard closed-form
// prism volume of the chord polygon times the height. The gap between them
// is (n/2)·r²·(sweep/n − sin(sweep/n))·h, always non-negative since a chord
// never exceeds the arc it subtends.
func TestChordedBoundaryVolumeAllowEnclosesTheMeasuredGap(t *testing.T) {
	radii := []float64{1, 5, 50}
	sweepsDeg := []float64{30, 90, 180, 270}
	heights := []float64{0.1, 10, 100}
	chordCounts := []int{3, 8, 32}

	minRatio := math.Inf(1)
	var minRow string
	rows := 0

	for _, r := range radii {
		for _, sweepDeg := range sweepsDeg {
			sweepRad := sweepDeg * math.Pi / 180
			for _, h := range heights {
				for _, n := range chordCounts {
					rows++
					dtheta := sweepRad / float64(n)

					trueVolume := 0.5 * r * r * sweepRad * h
					chordVolume := 0.5 * r * r * float64(n) * math.Sin(dtheta) * h
					measuredGap := trueVolume - chordVolume
					require.Greater(t, measuredGap, 0.0,
						"the chord polygon must be strictly inscribed in the true sector")

					sectionDelta := chordSagitta(r, sweepRad, n)
					verts, tris := pieSliceChordMesh(r, sweepRad, h, n)
					areaUpper := perturbedAreaUpper(verts, tris, sectionDelta)

					chordLen := 2 * r * math.Sin(dtheta/2)
					arcLen := r * dtheta
					excess := arcLen - chordLen
					require.GreaterOrEqual(t, excess, 0.0, "an arc never runs shorter than its own chord")
					for range n {
						areaUpper = absSumUpper(areaUpper, cellRuledExcessUpper(h, excess, excess))
					}

					allow := chordedBoundaryVolumeAllow(sectionDelta, areaUpper)
					require.LessOrEqual(t, measuredGap, allow,
						"r=%g sweep=%g h=%g n=%d: allow must enclose the measured volume gap", r, sweepDeg, h, n)

					ratio := allow / measuredGap
					if ratio < minRatio {
						minRatio = ratio
						minRow = fmt.Sprintf("r=%g sweep=%g h=%g n=%d", r, sweepDeg, h, n)
					}
				}
			}
		}
	}

	require.Positive(t, rows)
	require.GreaterOrEqual(t, minRatio, 1.0, "the loosest row in the table: %s", minRow)
	t.Logf("worst-case allow/measuredGap ratio %.6g at %s (%d rows)", minRatio, minRow, rows)
}

// TestCellRuledExcessUpperIsZeroWithoutCurvature pins the fallback term's own
// baseline: a cell with no arc-minus-chord excess on either side (a straight
// LineSeg pairing, docs/loft-design.md's increment-1 case) contributes
// nothing, so the fallback's sum leaves areaUpper exactly
// perturbedAreaUpper's own answer.
func TestCellRuledExcessUpperIsZeroWithoutCurvature(t *testing.T) {
	require.Equal(t, 0.0, cellRuledExcessUpper(10, 0, 0))
	require.Equal(t, 0.0, cellRuledExcessUpper(0, 1, 1), "no rule length is no excess to charge")
	require.Equal(t, 0.0, cellRuledExcessUpper(10, -1, 0), "a negative excess is the caller's own error, never a shrinking term")
}

// TestCellRuledExcessUpperScalesWithRuleLengthAndExcess pins the closed form
// itself: ruleLengthUpper * (excess0 + excess1), outward-rounded.
func TestCellRuledExcessUpperScalesWithRuleLengthAndExcess(t *testing.T) {
	got := cellRuledExcessUpper(3, 0.1, 0.2)
	require.InDelta(t, 3*(0.1+0.2), got, 1e-15)
	require.GreaterOrEqual(t, got, 3*(0.1+0.2), "the answer must round outward, never inward")
}

// TestChordedBoundaryVolumeAllowMatchesSweptVolumeAllowsClosedForm pins that
// chordedBoundaryVolumeAllow carries the identical closed form
// sweptVolumeAllow does — sectionDelta * areaUpper, outward-rounded — even
// though it is its own helper with its own derivation for a different
// mechanism (Q4's twin decision, never an extension of sweptVolumeAllow in
// place).
func TestChordedBoundaryVolumeAllowMatchesSweptVolumeAllowsClosedForm(t *testing.T) {
	require.Equal(t, sweptVolumeAllow(0.01, 5.0), chordedBoundaryVolumeAllow(0.01, 5.0))
	require.Equal(t, 0.0, chordedBoundaryVolumeAllow(0, 5.0))
	require.Equal(t, 0.0, chordedBoundaryVolumeAllow(0.01, 0))
}

// TestChordedBoundaryMomentAllowIsItsOwnThreeLineTwin pins
// chordedBoundaryMomentAllow's own closed form —
// chordedBoundaryVolumeAllow(sectionDelta, areaUpper) composed with
// coordUpper the same way sweptMomentAllow composes sweptVolumeAllow with
// it — computed here by calling chordedBoundaryVolumeAllow directly, never
// sweptVolumeAllow, so a caller that swapped the two internally would move
// this answer for a delta/areaUpper pair where chordedBoundaryVolumeAllow
// and sweptVolumeAllow already agree bit for bit (both are
// upRound(delta*areaUpper)), then diverge once the two mechanisms' own
// derivations grow apart.
func TestChordedBoundaryMomentAllowIsItsOwnThreeLineTwin(t *testing.T) {
	sectionDelta, areaUpper, coordUpper := 0.02, 7.5, 3.0
	want := productUpper(chordedBoundaryVolumeAllow(sectionDelta, areaUpper), coordUpper)
	got := chordedBoundaryMomentAllow(sectionDelta, areaUpper, coordUpper)
	require.Equal(t, want, got)

	require.Equal(t, 0.0, chordedBoundaryMomentAllow(0, areaUpper, coordUpper))
	require.Equal(t, 0.0, chordedBoundaryMomentAllow(sectionDelta, areaUpper, 0))
}
