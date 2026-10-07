package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// TestVerifyJointBoxLoopDependentSlide pins a dependent slide's travel in the
// cell certificate (docs/linkage-check-design.md §16.3): its δ_j in τ_half and
// its half-span in the projection bound. The Scotch yoke of §15.10 — the
// block, listed, over [−4, 4] mm on the yoke, the yoke a dependent slide along
// X — stands beside a wall x ∈ [25.5, 60], y ∈ [−50, 50], z ∈ [1, 3] in the
// yoke's layer alone. The block sits at y = 24 + q, so the yoke's pin is at
// x = √(900 − (24 + q)²) and the yoke's block, x ∈ [16, 20] at the zero pose,
// reaches x = 2 + √(900 − (24 + q)²): the gap is
// 25.5 − 2 − √(900 − (24 + q)²), increasing in q, smallest, 23.5 − √500 ≈
// 1.14 mm, at q = −4.
//
// The box reads Sound, the reading encloses that minimum, every leaf's
// published yoke range holds the closed-form yoke values at its ends, and every
// clear leaf's bound sits at or below the gap at its lower end, the least over
// it.
//
// Measured: 23 centres into 12 leaves.
//
// Legs seen to fail when deleted: the dependent slide's term in τ_half, its
// half-span in the projection bound, and both (the root's bound rises above
// the gap at q = −4 and the reading stops beyond tolerance).
func TestVerifyJointBoxLoopDependentSlide(t *testing.T) {
	t.Parallel()
	lp := scotchYoke(t)
	boxBodyAtZ(t, lp.doc, 25.5, -50, 60, 50, 1, 2)
	gap := func(q float64) float64 {
		y := 24 + q
		return 25.5 - 2 - math.Sqrt(900-y*y)
	}
	report := verifyJointBox(t, lp.doc, lp.l, decad.JointBox{
		{Link: lp.links[1], Min: units.Millimeters(-4), Max: units.Millimeters(4)},
	})
	t.Logf("cells evaluated %d, leaves %d, status %s", report.CellsEvaluated, len(report.Cells), report.Status)
	require.Equal(t, decad.Sound, report.Status)
	requireBoxReadingEncloses(t, report, gap(-4))
	for _, cell := range report.Cells {
		require.Equal(t, decad.CellClear, cell.Outcome)
		lo, hi := millimetresOf(t, cell.Cell.Min[1]), millimetresOf(t, cell.Cell.Max[1])
		yokeLo, yokeHi := millimetresOf(t, cell.Cell.Min[0]), millimetresOf(t, cell.Cell.Max[0])
		atLo, atHi := lp.closed(1, lo)[0], lp.closed(1, hi)[0]
		require.LessOrEqual(t, yokeLo, math.Min(atLo, atHi)+1e-12, `the yoke's published range holds its closed form`)
		require.GreaterOrEqual(t, yokeHi, math.Max(atLo, atHi)-1e-12, `the yoke's published range holds its closed form`)
		require.LessOrEqual(t, cell.Clearance.Value.Mag(), gap(lo)+1e-9, `a leaf's bound never exceeds the gap's least value over it`)
	}
}
