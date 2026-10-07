package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file holds the box tests of docs/linkage-check-design.md §5.8's cell
// form (§14.3, §14.8): scene 12, the three-joint box's flat minimum, and the
// boxes of scene 13's pendulum under a slide and scene 14's two arms. Each
// scene states its minimum in closed form, and every CellClear leaf's bound
// is asserted at or below a closed-form gap at a configuration of the leaf,
// which is at least the true minimum over it.

// requireBoxReadingEncloses asserts the whole-box Clearance reading holds the
// closed-form minimum gap inside its tolerance.
func requireBoxReadingEncloses(t *testing.T, report *decad.JointBoxReport, gap float64) {
	t.Helper()
	require.NotNil(t, report.Clearance)
	require.LessOrEqual(t, report.Clearance.Value.Mag()-report.Clearance.Bound.Mag(), gap)
	require.GreaterOrEqual(t, report.Clearance.Value.Mag()+report.Clearance.Bound.Mag(), gap)
	require.Equal(t, decad.ToleranceSatisfied, report.Clearance.Tolerance.State)
}

func radiansOf(t *testing.T, v units.Value) float64 {
	t.Helper()
	rad, err := v.In(units.Radian)
	require.NoError(t, err)
	return rad
}

// TestVerifyJointBoxThreeJointFlatMinimum is scene 12 of
// docs/linkage-check-design.md §14.8, the projection certificate's
// acceptance on a box: §10's three-joint arm with each joint over [0°, 90°],
// at the defaults. The wrist's leading corner (150, −10) sits at
// X = 50·(cos θ₁ + cos(θ₁+θ₂) + cos(θ₁+θ₂+θ₃)) + 10·sin(θ₁+θ₂+θ₃), whose
// largest value over the box, 100 + √2600, is reached at (0°, 0°,
// atan(1/5)) with the corner at the post's mid-height, so the box's minimum
// gap is 60 − 10·√26 ≈ 9.0098 mm: a flat minimum, where every first
// derivative of the gap vanishes. Where the corner faces the post's face
// (|Y| ≤ 10), 160 − X is its distance from the face, an upper bound on the
// gap.
//
// Measured: 419 centres into 210 leaves, where the travel bound alone
// exhausts the default budget of 16384 and reads Suspect (§14.7).
//
// Legs seen to fail when deleted: the projection bound (the budget runs out
// and the report reads Suspect); its first-order term (a leaf's bound exceeds
// the gap at one of its corners).
func TestVerifyJointBoxThreeJointFlatMinimum(t *testing.T) {
	t.Parallel()
	minimum := 60 - 10*math.Sqrt(26)
	require.InDelta(t, 9.0098, minimum, 1e-4)
	corner := func(t1, t2, t3 float64) (float64, float64) {
		a, b, c := t1, t1+t2, t1+t2+t3
		return 50*(math.Cos(a)+math.Cos(b)+math.Cos(c)) + 10*math.Sin(c),
			50*(math.Sin(a)+math.Sin(b)+math.Sin(c)) - 10*math.Cos(c)
	}
	x, y := corner(0, 0, math.Atan(0.2))
	require.InDelta(t, 100+math.Sqrt(2600), x, 1e-9)
	require.InDelta(t, 0, y, 1e-9)

	doc, l, drive := threeJointArm(t)
	box := make(decad.JointBox, len(drive))
	for n, sw := range drive {
		box[n] = decad.JointRange{Link: sw.Link, Min: units.Degrees(0), Max: units.Degrees(90)}
	}
	report := verifyJointBox(t, doc, l, box)
	t.Logf("cells evaluated %d, leaves %d, status %s", report.CellsEvaluated, len(report.Cells), report.Status)
	require.Equal(t, decad.Sound, report.Status)
	require.Empty(t, report.Diagnostics)
	require.Less(t, report.CellsEvaluated, 4096)
	requireBoxReadingEncloses(t, report, minimum)
	facing := 0
	for _, cell := range report.Cells {
		require.Equal(t, decad.CellClear, cell.Outcome)
		require.NotNil(t, cell.Clearance)
		bound := cell.Clearance.Value.Mag()
		upper := math.Inf(1)
		for _, row := range cell.Clearances {
			upper = math.Min(upper, row.Gap.Value.Mag()+row.Gap.Bound.Mag())
		}
		require.LessOrEqual(t, bound, upper, `a leaf's bound never exceeds a gap measured at its centre`)
		// The cell's minimum gap is at most 160 − X at any of its configurations
		// where the corner faces the post: its centre and its eight corners.
		configs := [][3]float64{{
			radiansOf(t, cell.Center.Values[0]), radiansOf(t, cell.Center.Values[1]), radiansOf(t, cell.Center.Values[2]),
		}}
		for n := range 8 {
			var q [3]float64
			for j := range 3 {
				end := cell.Cell.Min[j]
				if n&(1<<j) != 0 {
					end = cell.Cell.Max[j]
				}
				q[j] = radiansOf(t, end)
			}
			configs = append(configs, q)
		}
		for _, q := range configs {
			cx, cy := corner(q[0], q[1], q[2])
			if math.Abs(cy) <= 10 {
				facing++
				require.LessOrEqual(t, bound, 160-cx+1e-9, `a leaf's bound never exceeds the corner's gap at a configuration of it`)
			}
		}
	}
	require.Positive(t, facing)
}

// TestVerifyJointBoxPendulumUnderSlide is scene 13's box of
// docs/linkage-check-design.md §14.8, the prismatic-ancestor rule: the
// pendulum of §5.8 hung from a carriage, x ∈ [−5, 5], y ∈ [−8, −2],
// z ∈ [120, 130], on a prismatic joint along X under the ground, the box
// d ∈ [0, 30] mm × θ ∈ [0°, 90°]. The wall is wide, so the gap
// g(θ) = 20 − 5·sin θ + 40·cos θ is constant along d and falls with θ to its
// minimum 15 at 90°. The shallower joint is a prismatic, so B₁₁ = B₁₂ = 0 and
// the remainder carries no slide term: the slide has no share of the
// projection bound's defect and the reading never splits it. The carriage
// stands high above both: its swept box, grown by its 30 mm of travel, clears
// the wall along Z by 70 mm and the layer exclusion settles it against the
// block along Z at 110, above every gap of the block's, so each leaf's bound
// is the block's own against the wall. Near 0° the gap is concave, and a leaf there is clear only by a
// remainder of at least ½·|g”|·h² ≈ 20·h², which ½·ρ·h² ≈ 25·h² covers and
// half of it does not.
//
// Measured: 23 centres into 12 leaves.
//
// Legs seen to fail when deleted: B_ij's zero for a prismatic ancestor (the
// remainder then carries h_θ·h_d terms and the reading splits along d); the
// projection bound's own shares (the travel bound's rank the slide first, and
// the reading splits along d); the remainder, and half of it (a leaf's bound
// exceeds g at its far end).
func TestVerifyJointBoxPendulumUnderSlide(t *testing.T) {
	t.Parallel()
	build := func(t *testing.T) (*decad.Document, *decad.Linkage, decad.JointBox) {
		t.Helper()
		doc := decad.New()
		carriage := boxBodyAtZ(t, doc, -5, -8, 5, -2, 120, 10)
		block := boxBodyAtZ(t, doc, -5, -50, 5, -40, 0, 10)
		boxBodyAtZ(t, doc, -100, 20, 100, 40, -10, 30)
		l := decad.NewLinkage()
		slide, err := l.Ground().Prismatic(r3.NewVec(1, 0, 0), []*decad.Body{carriage})
		require.NoError(t, err)
		swing, err := slide.Revolute(r3.Vec{}, zAxis, []*decad.Body{block})
		require.NoError(t, err)
		return doc, l, decad.JointBox{
			{Link: slide, Min: units.Millimeters(0), Max: units.Millimeters(30)},
			{Link: swing, Min: units.Degrees(0), Max: units.Degrees(90)},
		}
	}
	g := func(th float64) float64 { return 20 - 5*math.Sin(th) + 40*math.Cos(th) }
	t.Run("the reading", func(t *testing.T) {
		t.Parallel()
		doc, l, box := build(t)
		report := verifyJointBox(t, doc, l, box)
		t.Logf("cells evaluated %d, leaves %d, status %s", report.CellsEvaluated, len(report.Cells), report.Status)
		require.Equal(t, decad.Sound, report.Status)
		requireBoxReadingEncloses(t, report, 15)
		for _, cell := range report.Cells {
			require.Equal(t, decad.CellClear, cell.Outcome)
			require.Equal(t, units.Millimeters(0), cell.Cell.Min[0], `the reading never splits the slide`)
			require.Equal(t, units.Millimeters(30), cell.Cell.Max[0], `the reading never splits the slide`)
			far := radiansOf(t, cell.Cell.Max[1])
			require.LessOrEqual(t, cell.Clearance.Value.Mag(), g(far)+1e-9, `a leaf's bound never exceeds g at its far end`)
		}
	})
	t.Run("the root alone", func(t *testing.T) {
		t.Parallel()
		doc, l, box := build(t)
		report := verifyJointBox(t, doc, l, box, decad.WithCellBudget(1))
		require.Len(t, report.Cells, 1)
		if c := report.Cells[0].Clearance; c != nil {
			require.Less(t, c.Value.Mag(), 15.0)
		}
	})
}

// rectGap is the distance between two rectangles in the plane, each given
// by its four corners in order: zero when one holds a corner of the other,
// else the least distance from a corner of one to an edge of the other.
func rectGap(p, q [4][2]float64) float64 {
	inside := func(x [2]float64, r [4][2]float64) bool {
		sign := 0.0
		for i := range 4 {
			a, b := r[i], r[(i+1)%4]
			c := (b[0]-a[0])*(x[1]-a[1]) - (b[1]-a[1])*(x[0]-a[0])
			if sign == 0 {
				sign = c
				continue
			}
			if c*sign < 0 {
				return false
			}
		}
		return true
	}
	seg := func(x, a, b [2]float64) float64 {
		dx, dy := b[0]-a[0], b[1]-a[1]
		s := ((x[0]-a[0])*dx + (x[1]-a[1])*dy) / (dx*dx + dy*dy)
		s = math.Max(0, math.Min(1, s))
		return math.Hypot(x[0]-a[0]-s*dx, x[1]-a[1]-s*dy)
	}
	best := math.Inf(1)
	for _, pair := range [][2][4][2]float64{{p, q}, {q, p}} {
		for _, x := range pair[0] {
			if inside(x, pair[1]) {
				return 0
			}
			for i := range 4 {
				best = math.Min(best, seg(x, pair[1][i], pair[1][(i+1)%4]))
			}
		}
	}
	return best
}

// TestVerifyJointBoxTwoArms is scene 14's box of docs/linkage-check-design.md
// §14.8, both bodies moving: §5.8's two arms, each turning over
// [−30°, 30°] about Z, A through the origin and B through (110, 0, 0). The
// gap has four flat minima of 110 − 10·√101 ≈ 9.501 mm at (±θ*, ±θ*) with
// tan θ* = 1/10, each with some corner pair at a shared height. Every leaf's
// bound is checked against the exact distance between the two arms'
// outlines at its centre.
//
// Measured: 283 centres into 142 leaves.
//
// Legs seen to fail when deleted: the partner's expansion (a leaf's bound
// exceeds the gap at its centre).
func TestVerifyJointBoxTwoArms(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	armA := boxBody(t, doc, 0, -5, 50, 5, 10)
	armB := boxBody(t, doc, 60, -5, 110, 5, 10)
	l := decad.NewLinkage()
	a, err := l.Ground().Revolute(r3.Vec{}, zAxis, []*decad.Body{armA})
	require.NoError(t, err)
	b, err := l.Ground().Revolute(r3.NewVec(110, 0, 0), zAxis, []*decad.Body{armB})
	require.NoError(t, err)
	minimum := 110 - 10*math.Sqrt(101)
	require.InDelta(t, 9.501, minimum, 1e-3)
	report := verifyJointBox(t, doc, l, decad.JointBox{
		{Link: a, Min: units.Degrees(-30), Max: units.Degrees(30)},
		{Link: b, Min: units.Degrees(-30), Max: units.Degrees(30)},
	})
	t.Logf("cells evaluated %d, leaves %d, status %s", report.CellsEvaluated, len(report.Cells), report.Status)
	require.Equal(t, decad.Sound, report.Status)
	require.Less(t, report.CellsEvaluated, 16384)
	requireBoxReadingEncloses(t, report, minimum)
	turn := func(corners [4][2]float64, cx, th float64) [4][2]float64 {
		var out [4][2]float64
		for n, p := range corners {
			dx := p[0] - cx
			out[n] = [2]float64{cx + dx*math.Cos(th) - p[1]*math.Sin(th), dx*math.Sin(th) + p[1]*math.Cos(th)}
		}
		return out
	}
	for _, cell := range report.Cells {
		require.Equal(t, decad.CellClear, cell.Outcome)
		ta, tb := radiansOf(t, cell.Center.Values[0]), radiansOf(t, cell.Center.Values[1])
		pa := turn([4][2]float64{{0, -5}, {50, -5}, {50, 5}, {0, 5}}, 0, ta)
		pb := turn([4][2]float64{{60, -5}, {110, -5}, {110, 5}, {60, 5}}, 110, tb)
		require.LessOrEqual(t, cell.Clearance.Value.Mag(), rectGap(pa, pb)+1e-9, `a leaf's bound never exceeds the gap at its centre`)
	}
}
