package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// TestVerifyJointBoxLoopProjection pins docs/linkage-check-design.md §5.8's
// expansion over a loop's dependent joint in the cell form (§16.3). Scene 7's
// crank-rocker carries a block x ∈ [96, 104], y ∈ [60, 66], z ∈ [40, 48] on
// its follower, in a layer of its own, under a ceiling x ∈ [50, 150],
// y ∈ [67.5, 77.5], z ∈ [39, 49] that slides down along (0, −1, 0) on a
// prismatic joint under the ground. The box is the crank over [0°, 90°] and
// the ceiling over [0, 1] mm. The follower turns by Δ = θ4(θ2) − θ4(0), so
// the gap is 67.5 − d − 66·cos Δ − 4·|sin Δ|, smallest,
// 66.5 − √4372 ≈ 0.379 mm, at d = 1 and Δ = −atan(4/66): twice along the
// crank, each a flat minimum along it.
//
// At the defaults the report reads Sound, the reading encloses the minimum,
// and every leaf's bound sits at or below the closed-form gap at each of its
// centre and corners.
//
// Measured: 137 centres into 69 leaves, against 6457 and Suspect under the
// travel bound alone.
//
// Legs seen to fail when deleted: the dependent's expansion (the pair takes
// the travel bound alone, and the box reads Suspect in 6457 centres); the
// hull of the cell in h (the centre's enclosure alone serves, and a leaf's
// bound exceeds the gap at a corner); and the far end of that hull (h a
// quarter of the hull's width, the same).
func TestVerifyJointBoxLoopProjection(t *testing.T) {
	t.Parallel()
	const g, r, f = rockerGround, rockerCrank, rockerFollower
	doc := decad.New()
	t4 := rockerTheta4(0)
	b := [2]float64{g + f*math.Cos(t4), f * math.Sin(t4)}
	crankBody := boxBodyAtZ(t, doc, 0, -4, r, 4, 0, 8)
	coupler := barBody(t, doc, [2]float64{r, 0}, b, 10)
	foll := barBody(t, doc, [2]float64{g, 0}, b, 20)
	block := boxBodyAtZ(t, doc, 96, 60, 104, 66, 40, 8)
	ceilingBody := boxBodyAtZ(t, doc, 50, 67.5, 150, 77.5, 39, 10)
	lk := decad.NewLinkage()
	z := r3.NewVec(0, 0, 1)
	crank, err := lk.Ground().Revolute(r3.Vec{}, z, []*decad.Body{crankBody})
	require.NoError(t, err)
	couplerLk, err := crank.Revolute(r3.NewVec(r, 0, 0), z, []*decad.Body{coupler})
	require.NoError(t, err)
	follow, err := lk.Ground().Revolute(r3.NewVec(g, 0, 0), z, []*decad.Body{foll, block})
	require.NoError(t, err)
	_, err = lk.Close(couplerLk, follow, r3.NewVec(b[0], b[1], 0), z)
	require.NoError(t, err)
	ceiling, err := lk.Ground().Prismatic(r3.NewVec(0, -1, 0), []*decad.Body{ceilingBody})
	require.NoError(t, err)

	gap := func(th2, d float64) float64 {
		delta := rockerTheta4(th2) - t4
		return 67.5 - d - 66*math.Cos(delta) - 4*math.Abs(math.Sin(delta))
	}
	minimum := 66.5 - math.Sqrt(66*66+4*4)
	report := verifyJointBox(t, doc, lk, decad.JointBox{
		{Link: crank, Min: units.Degrees(0), Max: units.Degrees(90)},
		{Link: ceiling, Min: units.Millimeters(0), Max: units.Millimeters(1)},
	})
	t.Logf("cells evaluated %d, leaves %d, status %s", report.CellsEvaluated, len(report.Cells), report.Status)
	require.Equal(t, decad.Sound, report.Status)
	requireBoxReadingEncloses(t, report, minimum)
	for _, cell := range report.Cells {
		require.Equal(t, decad.CellClear, cell.Outcome)
		thLo, thHi := radiansOf(t, cell.Cell.Min[0]), radiansOf(t, cell.Cell.Max[0])
		dLo, dHi := millimetresOf(t, cell.Cell.Min[3]), millimetresOf(t, cell.Cell.Max[3])
		least := gap((thLo+thHi)/2, (dLo+dHi)/2)
		for _, th := range []float64{thLo, thHi} {
			for _, d := range []float64{dLo, dHi} {
				least = math.Min(least, gap(th, d))
			}
		}
		require.LessOrEqual(t, cell.Clearance.Value.Mag(), least+1e-9, `a leaf's bound never exceeds the gap at its centre or corners`)
	}
}
