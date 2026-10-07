package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// TestVerifyJointBoxTiltedPendulum pins docs/linkage-check-design.md §5.8's
// hull points and directions in the cell form (§14.3): scene 13's box — the
// pendulum under a slide — carried by tiltToDiagonal. The carriage,
// x, y ∈ [−5, 5]·[−8, −2], z ∈ [120, 130], slides over [0, 30] mm along the
// tilt of X; the block, x ∈ [−5, 5], y ∈ [−50, −40], z ∈ [0, 10], swings over
// [0°, 90°] about the tilt of Z through the origin; the wall,
// x ∈ [−100, 100], y ∈ [20, 40], z ∈ [−10, 20], is met along the tilt of Y.
// Every distance is the untilted scene's, so the gap is
// g(θ) = 20 − 5·sin θ + 40·cos θ throughout, constant along the slide and
// smallest, 15, at 90°; but no coordinate direction meets the wall, and the
// bodies' boxes stand far outside them. Read at their own vertices and along
// the wall's face normal, the cell bound is second order again: the box
// reads Sound, the reading encloses 15, and every leaf's bound sits at or
// below g at its greater θ end.
//
// Measured: 23 centres into 12 leaves, as untilted, against 4607 under box
// corners and coordinate directions alone.
//
// Legs seen to fail when deleted: the hull bound (4607 centres); the face
// normals as candidate directions (the reading stops beyond tolerance,
// Suspect).
func TestVerifyJointBoxTiltedPendulum(t *testing.T) {
	t.Parallel()
	ts := newTiltedScene(t)
	doc := decad.New()
	carriage := ts.place(boxBodyAtZ(t, doc, -5, -8, 5, -2, 120, 10))
	block := ts.place(boxBodyAtZ(t, doc, -5, -50, 5, -40, 0, 10))
	ts.place(boxBodyAtZ(t, doc, -100, 20, 100, 40, -10, 30))
	l := decad.NewLinkage()
	slide, err := l.Ground().Prismatic(ts.tilt.ApplyDir(r3.NewVec(1, 0, 0)), []*decad.Body{carriage})
	require.NoError(t, err)
	swing, err := slide.Revolute(r3.Vec{}, ts.tilt.ApplyDir(zAxis), []*decad.Body{block})
	require.NoError(t, err)
	report := verifyJointBox(t, doc, l, decad.JointBox{
		{Link: slide, Min: units.Millimeters(0), Max: units.Millimeters(30)},
		{Link: swing, Min: units.Degrees(0), Max: units.Degrees(90)},
	})
	t.Logf("cells evaluated %d, leaves %d, status %s", report.CellsEvaluated, len(report.Cells), report.Status)
	require.Equal(t, decad.Sound, report.Status)
	require.Less(t, report.CellsEvaluated, 256, `the hull bound closes the reading as untilted`)
	requireBoxReadingEncloses(t, report, 15)
	g := func(th float64) float64 { return 20 - 5*math.Sin(th) + 40*math.Cos(th) }
	for _, cell := range report.Cells {
		require.Equal(t, decad.CellClear, cell.Outcome)
		require.LessOrEqual(t, cell.Clearance.Value.Mag(), g(radiansOf(t, cell.Cell.Max[1]))+1e-9, `a leaf's bound never exceeds g at its far end`)
	}
}

// TestVerifyJointBoxSlotKeepsItsBox pins the line-segment test of
// docs/linkage-check-design.md §5.8's hull points in the cell form: a slot,
// cap centres (0, 45) and (0, 85) with radius 5, z ∈ [0, 10], rocks over
// [−20°, 20°] about Z through the origin under a wall x ∈ [−100, 100],
// y ∈ [95, 105], z ∈ [−10, 20]. Its top cap reaches y = 85·cos θ + 5, so the
// gap 90 − 85·cos θ is smallest, 5 mm, upright — a flat minimum — while the
// hull of its segments' start points stops 5 mm lower there: an outer loop
// with an arc keeps its box corners. The box reads Sound, the reading
// encloses 5, and every leaf's bound sits at or below the gap's least value
// over the leaf. Measured: 563 centres into 282 leaves.
//
// Leg seen to fail when deleted: the line-segment test (an arc read at its
// start point; a leaf's bound near the minimum rises above the gap).
func TestVerifyJointBoxSlotKeepsItsBox(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	_, err = s.CreateSlot(0, 45, 0, 85, 5)
	require.NoError(t, err)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	slot, err := doc.Extrude(s, s.Profiles()[0], decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	require.NoError(t, err)
	boxBodyAtZ(t, doc, -100, 95, 100, 105, -10, 30)
	l := decad.NewLinkage()
	rock, err := l.Ground().Revolute(r3.Vec{}, zAxis, []*decad.Body{slot})
	require.NoError(t, err)
	report := verifyJointBox(t, doc, l, decad.JointBox{{Link: rock, Min: units.Degrees(-20), Max: units.Degrees(20)}})
	t.Logf("cells evaluated %d, leaves %d, status %s", report.CellsEvaluated, len(report.Cells), report.Status)
	gap := func(th float64) float64 { return 90 - 85*math.Cos(th) }
	requireBoxReadingEncloses(t, report, 5)
	for _, cell := range report.Cells {
		if cell.Outcome != decad.CellClear {
			continue
		}
		lo, hi := radiansOf(t, cell.Cell.Min[0]), radiansOf(t, cell.Cell.Max[0])
		least := math.Min(gap(lo), gap(hi))
		if lo < 0 && 0 < hi {
			least = gap(0)
		}
		require.LessOrEqual(t, cell.Clearance.Value.Mag(), least+1e-9, `a leaf's bound never exceeds the gap's least value over it`)
	}
}
