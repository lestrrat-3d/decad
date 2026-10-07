package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// TestVerifyJointBoxSymmetricBody pins docs/linkage-check-design.md §5.2's
// symmetry rule in the cell form (§14.3): a disc of radius 5, z ∈ [−5, 5],
// spins over [0°, 90°] on a revolute about Z under a carriage, x, y ∈ [−5, 5],
// z ∈ [40, 50], that slides over [0, 20] mm along Y, beside a wall
// x ∈ [12, 22], y ∈ [−20, 40], z ∈ [−10, 10]. The disc's nearest point to
// the wall stays at x = 5 at every configuration, so the gap is 7 mm
// throughout the box.
//
//   - Spinning on its own axis the disc does not move under its joint, and
//     the slide keeps its x-extent, so the root's projection bound along X is
//     the exact 7 mm and the box reads Sound from its centre alone.
//   - Spinning about an axis 1e-9 mm off its own, the rule keeps the joint:
//     its box corner sweeps a circle of radius 5·√2, the root cannot certify,
//     and the box splits along the spin. Every clear leaf's bound sits at or
//     below the 7 mm gap.
//   - Tumbling about X through its centre, the disc's top reaches
//     y = 5·(cos θ + sin θ), so against a wall y ∈ [12, 22] the gap
//     12 − 5·(cos θ + sin θ) falls to 12 − 5·√2: the rule must not apply, and
//     every clear leaf's bound sits at or below the gap's minimum over it.
//
// Legs seen to fail when deleted: the rule in the cell form (the disc on its
// axis splits along the spin joint); the rule's centre test (the disc 1e-9
// mm off its axis certifies at the root); its direction test (the tumbling
// disc certifies its rest gap of 7 mm).
func TestVerifyJointBoxSymmetricBody(t *testing.T) {
	t.Parallel()
	build := func(t *testing.T, centre r3.Vec) (*decad.Document, *decad.Linkage, decad.JointBox) {
		t.Helper()
		doc := decad.New()
		carriage := boxBodyAtZ(t, doc, -5, -5, 5, 5, 40, 10)
		disc := discBodySymmetric(t, doc, 0, 5, 5)
		boxBodyAtZ(t, doc, 12, -20, 22, 40, -10, 20)
		l := decad.NewLinkage()
		slide, err := l.Ground().Prismatic(r3.NewVec(0, 1, 0), []*decad.Body{carriage})
		require.NoError(t, err)
		spin, err := slide.Revolute(centre, zAxis, []*decad.Body{disc})
		require.NoError(t, err)
		return doc, l, decad.JointBox{
			{Link: slide, Min: units.Millimeters(0), Max: units.Millimeters(20)},
			{Link: spin, Min: units.Degrees(0), Max: units.Degrees(90)},
		}
	}
	t.Run("on its own axis", func(t *testing.T) {
		t.Parallel()
		doc, l, box := build(t, r3.Vec{})
		report := verifyJointBox(t, doc, l, box)
		require.Equal(t, decad.Sound, report.Status)
		require.Equal(t, 1, report.CellsEvaluated, `the root certifies the whole box`)
		require.Len(t, report.Cells, 1)
		require.Equal(t, decad.CellClear, report.Cells[0].Outcome)
		require.Equal(t, 7.0, report.Cells[0].Clearance.Value.Mag(), `no travel: the exact 7 mm`)
		requireBoxReadingEncloses(t, report, 7)
	})
	t.Run("off its own axis", func(t *testing.T) {
		t.Parallel()
		doc, l, box := build(t, r3.NewVec(1e-9, 0, 0))
		report := verifyJointBox(t, doc, l, box, decad.WithResolution(units.Scalar(1.0/16)))
		t.Logf("cells evaluated %d, leaves %d, status %s", report.CellsEvaluated, len(report.Cells), report.Status)
		require.Greater(t, report.CellsEvaluated, 1, `the root cannot certify`)
		split := false
		for _, cell := range report.Cells {
			if degreesOf(t, cell.Cell.Max[1])-degreesOf(t, cell.Cell.Min[1]) < 90 {
				split = true
			}
			if cell.Outcome == decad.CellClear {
				require.LessOrEqual(t, cell.Clearance.Value.Mag(), 7.0+1e-9)
			}
		}
		require.True(t, split, `the box splits along the spin`)
	})
	t.Run("tumbling", func(t *testing.T) {
		t.Parallel()
		doc := decad.New()
		disc := discBodySymmetric(t, doc, 0, 5, 5)
		boxBodyAtZ(t, doc, -20, 12, 20, 22, -30, 60)
		l := decad.NewLinkage()
		tumble, err := l.Ground().Revolute(r3.Vec{}, r3.NewVec(1, 0, 0), []*decad.Body{disc})
		require.NoError(t, err)
		report := verifyJointBox(t, doc, l, decad.JointBox{{Link: tumble, Min: units.Degrees(0), Max: units.Degrees(90)}},
			decad.WithResolution(units.Scalar(1.0/16)))
		gap := func(th float64) float64 { return 12 - 5*(math.Cos(th)+math.Sin(th)) }
		cleared := 0
		for _, cell := range report.Cells {
			if cell.Outcome != decad.CellClear {
				continue
			}
			cleared++
			// g is least at 45° where the cell holds it, else at an end.
			lo, hi := radiansOf(t, cell.Cell.Min[0]), radiansOf(t, cell.Cell.Max[0])
			least := math.Min(gap(lo), gap(hi))
			if lo < math.Pi/4 && math.Pi/4 < hi {
				least = gap(math.Pi / 4)
			}
			require.LessOrEqual(t, cell.Clearance.Value.Mag(), least+1e-9, `a leaf's bound never exceeds the gap's minimum over it`)
		}
		require.Positive(t, cleared)
	})
}
