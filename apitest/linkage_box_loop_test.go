package apitest_test

import (
	"context"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file holds the B1 tests of docs/linkage-check-design.md §16.8: the
// joint box over a closed loop. Every closed form is scene 7's crank-rocker
// (ground 100, crank 30, coupler 80, follower 70), its follower's top corner
// at y_c(θ2) = 70·sin θ4 − 4·cos θ4, which rises from 67.0399 at θ2 = 0 to
// 69.3857 at the follower's extreme θ2* = 38.5727° and falls to 65.8629 at
// 90°. Bounds are asserted against geometric truths, never pinned to a
// measured literal: FMA contraction moves the last ulp between amd64 and
// arm64.

// rockerExtremeCrank is θ2*, the crank angle at the follower's extreme,
// bracketed to 1e-12 rad by bisection of dθ4/dθ2.
func rockerExtremeCrank() float64 {
	slope := func(th2 float64) float64 { return rockerTheta4(th2+1e-7) - rockerTheta4(th2-1e-7) }
	return bisectRoot(slope, 0.3, 1)
}

// rockerCornerY is y_c(θ2), the follower's top corner's height.
func rockerCornerY(th2 float64) float64 {
	_, y := rockerCorner(th2)
	return y
}

// rockerFollowerTurn is the follower's joint value at crank angle th2: its
// turn from the zero pose, θ4(θ2) − θ4(0).
func rockerFollowerTurn(th2 float64) float64 {
	return rockerTheta4(th2) - rockerTheta4(0)
}

// rockerTurnRange is the least and greatest follower turn over the crank
// range [lo, hi] (radians): at its ends, or at θ2* where the range holds it.
func rockerTurnRange(lo, hi float64) (float64, float64) {
	a, b := rockerFollowerTurn(lo), rockerFollowerTurn(hi)
	least, most := math.Min(a, b), math.Max(a, b)
	if star := rockerExtremeCrank(); lo < star && star < hi {
		least = math.Min(least, rockerFollowerTurn(star))
	}
	return least, most
}

// rockerNearest is the crank value of [lo, hi] nearest θ2*, where y_c is
// greatest over the range, and rockerFarthest the end farther from it, where
// y_c is least.
func rockerNearest(lo, hi float64) float64 {
	return math.Min(math.Max(rockerExtremeCrank(), lo), hi)
}

func rockerFarthest(lo, hi float64) float64 {
	star := rockerExtremeCrank()
	if math.Abs(lo-star) > math.Abs(hi-star) {
		return lo
	}
	return hi
}

// gateRocker is scene 11 of §16.8: scene 7's crank-rocker with no static
// body and link 4, the gate, a prism x ∈ [−50, 150], y ∈ [72, 82],
// z ∈ [19, 29] on a prismatic joint under the ground along (0, −1, 0), so
// its value d lowers its underside to y = 72 − d.
type gateRocker struct {
	fourBar
	gateBody *decad.Body
	gate     *decad.Link
}

func buildGateRocker(t *testing.T) gateRocker {
	t.Helper()
	return buildGateRockerSpan(t, -50, 150)
}

// buildGateRockerSpan is scene 11 with the gate spanning x ∈ [x0, x1].
func buildGateRockerSpan(t *testing.T, x0, x1 float64) gateRocker {
	t.Helper()
	fb, _ := buildRocker(t, false)
	g := gateRocker{fourBar: fb}
	g.gateBody = boxBodyAtZ(t, fb.doc, x0, 72, x1, 82, 19, 10)
	var err error
	g.gate, err = fb.linkage.Ground().Prismatic(r3.NewVec(0, -1, 0), []*decad.Body{g.gateBody})
	require.NoError(t, err)
	return g
}

func (g gateRocker) box(crankLo, crankHi, gateLo, gateHi float64) decad.JointBox {
	return decad.JointBox{
		{Link: g.crank, Min: units.Degrees(crankLo), Max: units.Degrees(crankHi)},
		{Link: g.gate, Min: units.Millimeters(gateLo), Max: units.Millimeters(gateHi)},
	}
}

// rockerFollowerRho is the ball reading of the follower about O4's axis
// (§5.2): its rest box's farthest corner, (71.9627, 67.0399), sits 72.6666 mm
// from it.
func rockerFollowerRho() float64 {
	t4 := rockerTheta4(0)
	bx, by := rockerGround+rockerFollower*math.Cos(t4), rockerFollower*math.Sin(t4)
	minX := bx - 4*math.Sin(t4)
	return math.Hypot(minX-rockerGround, by-4*math.Cos(t4))
}

// TestVerifyJointBoxLoopGate is scene 11 of docs/linkage-check-design.md
// §16.8, the acceptance target: the crank over [0°, 90°] and the gate over
// [0, 10] mm. The colliding region is {d > d*(θ2)}, d*(θ2) = 72 − y_c(θ2):
// 4.9601 at 0°, 2.6143 at θ2*, 6.1371 at 90°, a boundary that turns back in
// the crank. The (crank, gate) and (coupler, gate) pairs are settled by the
// layer exclusion along Z, so each centre evaluates (follower, gate) alone,
// and its τ_half is ρ·δ + Δd/2 with δ the follower's one-sided reach from the
// centre's enclosure across the cell's hull.
//
// The hull each cell publishes for the follower is at most 1.05 times the
// closed form's variation over the cell plus 1e-6 rad: sketch's piece rule
// allows 5% of a hull, and measured hulls run at most 1.0052 times the
// variation, 1.2e-4 rad over it. An undecided centre's δ is then at most
// 1.1·v, v the follower's farthest closed-form value from the centre's.
//
// Measured: 445 centres into 223 leaves (24 clear, 168 colliding, 31
// undecided); none blocks, since the gate's own allowance ½·Δd·8200 ≈ 2560
// mm³ exceeds every overlap at this floor.
//
// Legs seen to fail when deleted: the dependent's term in τ_half (a clear
// leaf's bound exceeds the true gap at its worst configuration).
func TestVerifyJointBoxLoopGate(t *testing.T) {
	t.Parallel()
	g := buildGateRocker(t)
	const res = 1.0 / 16
	report := verifyJointBox(t, g.doc, g.linkage, g.box(0, 90, 0, 10), decad.WithResolution(units.Scalar(res)))
	t.Logf("cells evaluated %d, leaves %d, status %s", report.CellsEvaluated, len(report.Cells), report.Status)

	require.Equal(t, decad.Interfering, report.Status)
	require.Empty(t, report.Against, `the gate is a link, and the scene has no static body`)
	require.Less(t, report.CellsEvaluated, 16384)
	for _, d := range report.Diagnostics {
		require.NotEqual(t, decad.DiagJointBoxBudgetExhausted, d.Code)
		require.NotNil(t, d.Cell, `a joint-box finding names its cell`)
	}

	rho := rockerFollowerRho()
	star := rockerExtremeCrank()
	worstSlack := 0.0
	area := 0.0
	type span struct{ thLo, thHi, dLo, dHi float64 }
	var spans []span
	outcomes := map[decad.CellOutcome]int{}
	for _, cell := range report.Cells {
		outcomes[cell.Outcome]++
		thLo, thHi := degreesOf(t, cell.Cell.Min[0])*math.Pi/180, degreesOf(t, cell.Cell.Max[0])*math.Pi/180
		dLo, dHi := millimetresOf(t, cell.Cell.Min[3]), millimetresOf(t, cell.Cell.Max[3])
		spans = append(spans, span{thLo, thHi, dLo, dHi})
		area += (thHi - thLo) / (math.Pi / 2) * (dHi - dLo) / 10
		thC, dC := degreesOf(t, cell.Center.Values[0])*math.Pi/180, millimetresOf(t, cell.Center.Values[3])
		require.InDelta(t, (thLo+thHi)/2, thC, 1e-12, `the centre is the cell's midpoint along the crank`)
		require.InDelta(t, (dLo+dHi)/2, dC, 1e-12, `the centre is the cell's midpoint along the gate`)

		// The follower's centre value is its closed-form turn, enclosed.
		require.Equal(t, units.Radian, cell.Center.Values[2].Unit())
		require.InDelta(t, rockerFollowerTurn(thC), cell.Center.Values[2].Mag(), 1e-9)
		require.Positive(t, cell.Center.Bounds[2].Mag())
		require.Less(t, cell.Center.Bounds[2].Mag(), 1e-9)
		require.Zero(t, cell.Center.Bounds[0].Mag())
		require.Zero(t, cell.Center.Bounds[3].Mag())

		// The cell publishes the follower's proven hull over its crank range.
		least, most := rockerTurnRange(thLo, thHi)
		hLo, hHi := cell.Cell.Min[2].Mag(), cell.Cell.Max[2].Mag()
		require.LessOrEqual(t, hLo, least+1e-12)
		require.GreaterOrEqual(t, hHi, most-1e-12)
		if variation := most - least; variation > 0 {
			worstSlack = math.Max(worstSlack, (hHi-hLo)/variation)
		}
		require.LessOrEqual(t, hHi-hLo, 1.05*(most-least)+1e-6, `the hull's slack stays inside sketch's piece rule`)

		switch cell.Outcome {
		case decad.CellClear:
			worst := rockerCornerY(rockerNearest(thLo, thHi))
			require.Less(t, worst, 72-dHi, `a clear cell holds no colliding configuration`)
			require.NotNil(t, cell.Clearance)
			require.LessOrEqual(t, cell.Clearance.Value.Mag(), 72-dHi-worst, `the cell's bound never exceeds its true minimum gap`)
		case decad.CellBlocked:
			require.Greater(t, rockerCornerY(rockerFarthest(thLo, thHi)), 72-dLo, `a blocked cell holds no clear configuration`)
		case decad.CellColliding:
			require.Greater(t, rockerCornerY(thC), 72-dC)
			require.LessOrEqual(t, (thHi-thLo)/(math.Pi/2), res+1e-12, `a colliding cell splits to the floor`)
			require.LessOrEqual(t, (dHi-dLo)/10, res+1e-12, `a colliding cell splits to the floor`)
		case decad.CellUndecided:
			require.LessOrEqual(t, (thHi-thLo)/(math.Pi/2), res+1e-12, `an undecided cell splits to the floor`)
			require.LessOrEqual(t, (dHi-dLo)/10, res+1e-12, `an undecided cell splits to the floor`)
			a, b := rockerFollowerTurn(thLo), rockerFollowerTurn(thHi)
			c := rockerFollowerTurn(thC)
			v := math.Max(math.Abs(a-c), math.Abs(b-c))
			if thLo < star && star < thHi {
				v = math.Max(v, math.Abs(rockerFollowerTurn(star)-c))
			}
			gap := 72 - dC - rockerCornerY(thC)
			require.LessOrEqual(t, gap, rho*(1.1*v+1e-6)+(dHi-dLo)/2, `an undecided centre lies within its cell's reach of the boundary`)
		default:
			require.Failf(t, `unexpected outcome`, `%s`, cell.Outcome)
		}
	}
	t.Logf("outcomes %v, worst hull slack ratio %.6f", outcomes, worstSlack)
	require.InDelta(t, 1, area, 1e-12, `the leaves cover the box`)
	for a := range spans {
		for b := a + 1; b < len(spans); b++ {
			th := math.Min(spans[a].thHi, spans[b].thHi) - math.Max(spans[a].thLo, spans[b].thLo)
			d := math.Min(spans[a].dHi, spans[b].dHi) - math.Max(spans[a].dLo, spans[b].dLo)
			require.Falsef(t, th > 1e-9 && d > 1e-9, `leaves %d and %d share interior`, a, b)
		}
	}
	require.Positive(t, outcomes[decad.CellClear])
	require.Positive(t, outcomes[decad.CellColliding])

	shallow := 0
	for _, hit := range report.Collisions {
		require.Same(t, g.foll, hit.A)
		require.Same(t, g.gateBody, hit.B)
		th, d := degreesOf(t, hit.Configuration.Values[0])*math.Pi/180, millimetresOf(t, hit.Configuration.Values[3])
		depth := rockerCornerY(th) - (72 - d)
		require.Positive(t, depth)
		require.Less(t, hit.Volume.Bound.Mag(), hit.Volume.Value.Mag())
		t4 := rockerTheta4(th)
		if depth < 8*math.Abs(math.Cos(t4)) {
			shallow++
			require.InDelta(t, 8*depth*depth/(2*math.Sin(t4)*(-math.Cos(t4))), hit.Volume.Value.Mag(), 1e-6)
		}
	}
	require.Positive(t, shallow, `some collision crosses the gate at the corner only`)
}

// TestVerifyJointBoxLoopOneAxis is §16.8's one-axis loop box: scene 7's
// crank-rocker and wall (y ∈ [68.5, 78.5]), the crank alone over [0°, 90°] at
// WithResolution(1/64). The follower's corner is inside the wall for
// θ2 ∈ (12.6250°, 66.7924°), the fractions s₁ = 0.140278 and s₂ = 0.742137.
// The corner's overlap is at most about 16 mm³ against the follower's area,
// so a colliding cell blocks only once its τ_half falls under about
// 7e-3 mm, near 1/5000 of the range: none blocks at this floor.
//
// Measured: 107 centres into 54 leaves.
//
// Legs seen to fail when deleted: the dependent's term in the blocked
// allowance (τ_half of (follower, wall) is then 0, every colliding centre
// blocks its cell, and the leaf holding s₁ reads CellBlocked though its lower
// part is clear).
func TestVerifyJointBoxLoopOneAxis(t *testing.T) {
	t.Parallel()
	fb, wall := buildRocker(t, true)
	const res = 1.0 / 64
	report := verifyJointBox(t, fb.doc, fb.linkage, decad.JointBox{{Link: fb.crank, Min: units.Degrees(0), Max: units.Degrees(90)}},
		decad.WithResolution(units.Scalar(res)))
	t.Logf("cells evaluated %d, leaves %d", report.CellsEvaluated, len(report.Cells))
	require.Equal(t, decad.Interfering, report.Status)
	s1, s2 := rockerWallHits()
	holds := func(lo, hi, s float64) bool { return lo <= s && s <= hi }
	for _, cell := range report.Cells {
		lo, hi := degreesOf(t, cell.Cell.Min[0])/90, degreesOf(t, cell.Cell.Max[0])/90
		c := degreesOf(t, cell.Center.Values[0]) / 90
		switch cell.Outcome {
		case decad.CellClear:
			require.True(t, hi <= s1 || lo >= s2, `a clear leaf lies outside the wall's stretch`)
		case decad.CellBlocked:
			require.True(t, lo > s1 && hi < s2, `a blocked leaf lies inside the wall's stretch`)
		case decad.CellColliding:
			require.True(t, c > s1 && c < s2, `a colliding centre lies inside the wall's stretch`)
		case decad.CellUndecided:
			require.LessOrEqual(t, hi-lo, res+1e-12, `an undecided leaf splits to the floor`)
		default:
			require.Failf(t, `unexpected outcome`, `%s`, cell.Outcome)
		}
		if holds(lo, hi, s1) || holds(lo, hi, s2) {
			require.Contains(t, []decad.CellOutcome{decad.CellUndecided, decad.CellColliding}, cell.Outcome, `a leaf holding a wall crossing decides nothing about it`)
		}
		th := c * math.Pi / 2
		require.InDelta(t, rockerFollowerTurn(th), cell.Center.Values[2].Mag(), 1e-9)
		require.Positive(t, cell.Center.Bounds[2].Mag())
		require.Less(t, cell.Center.Bounds[2].Mag(), 1e-9)
	}
	require.NotEmpty(t, report.Collisions)
	for _, hit := range report.Collisions {
		require.Same(t, fb.foll, hit.A)
		require.Same(t, wall, hit.B)
		th := degreesOf(t, hit.Configuration.Values[0]) * math.Pi / 180
		depth := rockerCornerY(th) - 68.5
		require.Positive(t, depth)
		require.Less(t, hit.Volume.Bound.Mag(), hit.Volume.Value.Mag())
		if t4 := rockerTheta4(th); depth < 8*math.Abs(math.Cos(t4)) {
			require.InDelta(t, 8*depth*depth/(2*math.Sin(t4)*(-math.Cos(t4))), hit.Volume.Value.Mag(), 1e-6)
		}
	}
}

// TestVerifyJointBoxLoopFold is §16.8's fold box: scene 9's non-Grashof
// four-bar (ground 100, crank 50, coupler 60, follower 50), the crank over
// [0°, 90°], no static body, at the defaults. The loop folds at
// s_fold = 0.974528, past which sketch certifies nothing: a cell whose hull
// holds the fold is gated, a centre past it is unbuildable, and a cell whose
// range meets no stretch the decomposition certified is stuck, published as
// it stands.
//
// Measured: 21 centres into 11 leaves.
//
// Legs seen to fail when deleted: the hull gate (the root's hull spans the
// fold, every pair is settled, and the root reads CellClear and the report
// Sound); the stuck rule (every leaf past the fold splits to the floor, 67
// centres against the 64 this test allows).
func TestVerifyJointBoxLoopFold(t *testing.T) {
	t.Parallel()
	sFold := math.Acos(0.04) / (math.Pi / 2)
	doc := decad.New()
	fb := buildFourBar(t, doc, foldCrank, foldCoupler, foldFollower)
	report := verifyJointBox(t, doc, fb.linkage, decad.JointBox{{Link: fb.crank, Min: units.Degrees(0), Max: units.Degrees(90)}})
	t.Logf("cells evaluated %d, leaves %d", report.CellsEvaluated, len(report.Cells))
	require.Equal(t, decad.Suspect, report.Status)
	require.Empty(t, report.Collisions)
	require.Less(t, report.CellsEvaluated, 64)
	const floor = 1.0 / 1024
	wide, holding := 0, 0
	for _, cell := range report.Cells {
		lo, hi := degreesOf(t, cell.Cell.Min[0])/90, degreesOf(t, cell.Cell.Max[0])/90
		if cell.Outcome == decad.CellClear {
			require.LessOrEqual(t, hi, sFold, `no clear leaf reaches past the fold`)
			continue
		}
		require.Equal(t, decad.CellUndecided, cell.Outcome)
		require.Len(t, cell.Diagnostics, 1)
		diag := cell.Diagnostics[0]
		require.Equal(t, decad.DiagMotionUndecidedInterval, diag.Code)
		require.NotNil(t, diag.Cell)
		require.Contains(t, diag.Message, `the loop closing links 1 and 2`)
		require.Contains(t, diag.Message, `not certified`)
		require.Zero(t, cell.Cell.Min[1], `a refused hull publishes no range for the dependent`)
		if lo <= sFold && sFold <= hi {
			holding++
			require.LessOrEqual(t, hi-lo, floor+1e-12, `the leaf holding the fold is at the floor`)
			continue
		}
		require.Greater(t, lo, sFold)
		require.Equal(t, decad.JointConfiguration{}, cell.Center, `a centre past the fold is unbuildable`)
		require.Empty(t, cell.Clearances)
		require.Empty(t, cell.Interferences)
		if hi-lo > floor+1e-12 {
			wide++
		}
	}
	require.Equal(t, 1, holding)
	require.Positive(t, wide, `a stuck leaf stands wider than the floor`)
}

// TestVerifyJointBoxLoopErrors is one subtest per row of
// docs/linkage-check-design.md §16.6.
func TestVerifyJointBoxLoopErrors(t *testing.T) {
	t.Parallel()
	z := r3.NewVec(0, 0, 1)
	t.Run("a box listing the crank and the follower", func(t *testing.T) {
		t.Parallel()
		fb, _ := buildRocker(t, false)
		_, err := fb.doc.VerifyJointBox(t.Context(), fb.linkage, decad.JointBox{
			{Link: fb.crank, Min: units.Degrees(0), Max: units.Degrees(90)},
			{Link: fb.follow, Min: units.Degrees(0), Max: units.Degrees(3)},
		})
		require.ErrorIs(t, err, decad.ErrDegenerate)
	})
	t.Run("a box listing the coupler", func(t *testing.T) {
		t.Parallel()
		fb, _ := buildRocker(t, false)
		_, err := fb.doc.VerifyJointBox(t.Context(), fb.linkage, decad.JointBox{{Link: fb.couplerLk, Min: units.Degrees(0), Max: units.Degrees(10)}})
		require.ErrorIs(t, err, decad.ErrUnsupported)
	})
	t.Run("the flat four-bar refuses its zero pose", func(t *testing.T) {
		t.Parallel()
		doc := decad.New()
		l := decad.NewLinkage()
		ck, err := l.Ground().Revolute(r3.Vec{}, z, []*decad.Body{boxBodyAtZ(t, doc, 0, -4, 30, 4, 0, 8)})
		require.NoError(t, err)
		cp, err := ck.Revolute(r3.NewVec(30, 0, 0), z, []*decad.Body{boxBodyAtZ(t, doc, 30, -4, 70, 4, 10, 8)})
		require.NoError(t, err)
		fl, err := l.Ground().Revolute(r3.NewVec(100, 0, 0), z, []*decad.Body{boxBodyAtZ(t, doc, 70, -4, 100, 4, 20, 8)})
		require.NoError(t, err)
		_, err = l.Close(cp, fl, r3.NewVec(70, 0, 0), z)
		require.NoError(t, err)
		_, err = doc.VerifyJointBox(t.Context(), l, decad.JointBox{{Link: ck, Min: units.Degrees(0), Max: units.Degrees(90)}})
		require.ErrorIs(t, err, decad.ErrUnsupported)
		require.ErrorIs(t, err, sketch.ErrUnderconstrained)
	})
	limited := func(t *testing.T, lo, hi float64) fourBar {
		t.Helper()
		doc := decad.New()
		fb := buildFourBarOpen(t, doc, z, decad.WithJointLimits(units.Degrees(lo), units.Degrees(hi)))
		var err error
		fb.loop, err = fb.linkage.Close(fb.couplerLk, fb.follow, r3.NewVec(fb.b[0], fb.b[1], 0), z)
		require.NoError(t, err)
		return fb
	}
	crankBox := func(fb fourBar) decad.JointBox {
		return decad.JointBox{{Link: fb.crank, Min: units.Degrees(0), Max: units.Degrees(90)}}
	}
	t.Run("follower limits the hull leaves", func(t *testing.T) {
		t.Parallel()
		fb := limited(t, -5, 5)
		_, err := fb.doc.VerifyJointBox(t.Context(), fb.linkage, crankBox(fb))
		require.ErrorIs(t, err, decad.ErrDegenerate)
		require.ErrorContains(t, err, `link 2's dependent joint`)
	})
	t.Run("follower limits the hull keeps", func(t *testing.T) {
		t.Parallel()
		fb := limited(t, -10, 5)
		verifyJointBox(t, fb.doc, fb.linkage, crankBox(fb), decad.WithResolution(units.Scalar(1.0/4)))
	})
	t.Run("a configuration of a looped linkage", func(t *testing.T) {
		t.Parallel()
		fb, _ := buildRocker(t, false)
		_, err := fb.linkage.Configuration([]units.Value{units.Degrees(0), units.Degrees(0), units.Degrees(0)})
		require.ErrorIs(t, err, decad.ErrUnsupported)
	})
}

// requireGateCentres asserts every leaf's centre reads the follower's closed
// form at its crank value, and that the held drive PoseAt poses at the
// centre's stated values agrees with it: two enclosures of one exact
// configuration, each within 1e-9 of the closed form.
func requireGateCentres(t *testing.T, g gateRocker, report *decad.JointBoxReport) {
	t.Helper()
	for n, cell := range report.Cells {
		crank, err := cell.Center.Values[0].In(units.Radian)
		require.NoError(t, err)
		require.InDeltaf(t, rockerFollowerTurn(crank), cell.Center.Values[2].Mag(), 1e-9, `leaf %d at crank %v`, n, crank)
		if n%7 != 0 {
			continue
		}
		held := decad.Drive{
			{Link: g.crank, From: cell.Center.Values[0], To: cell.Center.Values[0]},
			{Link: g.gate, From: cell.Center.Values[3], To: cell.Center.Values[3]},
		}
		pose, err := g.linkage.PoseAt(held, units.Scalar(0.5))
		require.NoError(t, err)
		require.InDelta(t, cell.Center.Values[2].Mag(), pose.Values[2].Mag(), 1e-9)
		require.InDelta(t, rockerFollowerTurn(crank), pose.Values[2].Mag(), 1e-9)
	}
}

// TestVerifyJointBoxLoopCrossingZero is §16.8's crossing standing test:
// scene 11 with the crank over [−30°, 60°], across 0 at the non-dyadic 1/3,
// and over [−30°, 1 rad], across 0 at an irrational fraction cut around a
// straddle (§15.8). Every centre reads the closed form, the stretch below 0
// on the mirrored scene, at WithResolution(1/4).
//
// Legs seen to fail when deleted: reading every cell on the scene's own side
// (a centre below 0 reads the follower's turn at the mirrored crank angle).
func TestVerifyJointBoxLoopCrossingZero(t *testing.T) {
	t.Parallel()
	for _, row := range []struct {
		name string
		max  units.Value
	}{
		{"to 60°", units.Degrees(60)},
		{"to 1 rad", units.Radians(1)},
	} {
		t.Run(row.name, func(t *testing.T) {
			t.Parallel()
			g := buildGateRocker(t)
			box := decad.JointBox{
				{Link: g.crank, Min: units.Degrees(-30), Max: row.max},
				{Link: g.gate, Min: units.Millimeters(0), Max: units.Millimeters(10)},
			}
			report := verifyJointBox(t, g.doc, g.linkage, box, decad.WithResolution(units.Scalar(1.0/4)))
			t.Logf("cells evaluated %d, leaves %d, status %s", report.CellsEvaluated, len(report.Cells), report.Status)
			below := 0
			for _, cell := range report.Cells {
				if cell.Center.Values[0].Mag() < 0 {
					below++
				}
			}
			require.Positive(t, below, `some centre lies below 0`)
			requireGateCentres(t, g, report)
		})
	}
}

// TestVerifyJointBoxLoopNonMutationAndDeterminism: two checks of scene 11
// leave the document as it was and return reports equal in every field.
func TestVerifyJointBoxLoopNonMutationAndDeterminism(t *testing.T) {
	t.Parallel()
	g := buildGateRocker(t)
	box := g.box(0, 90, 0, 10)
	first := verifyJointBox(t, g.doc, g.linkage, box, decad.WithResolution(units.Scalar(1.0/4)))
	second := verifyJointBox(t, g.doc, g.linkage, box, decad.WithResolution(units.Scalar(1.0/4)))
	require.Equal(t, first, second)
	requireGateCentres(t, g, first)
}

// TestVerifyJointBoxLoopCancellation: a context canceled at every depth of
// the check, inside a sketch.Enclose call among them, returns its error and no
// report, and the check finishes once the context outlasts it.
func TestVerifyJointBoxLoopCancellation(t *testing.T) {
	t.Parallel()
	g := buildGateRocker(t)
	box := g.box(0, 90, 0, 10)
	before := g.doc.Bodies()
	var completed *decad.JointBoxReport
	canceled := 0
	for limit := int32(1); limit <= 1<<24; limit += 1 + limit/2 {
		ctx := newCancelAfterContext(t.Context(), limit)
		report, err := g.doc.VerifyJointBox(ctx, g.linkage, box, decad.WithResolution(units.Scalar(1.0/2)))
		require.Equal(t, before, g.doc.Bodies())
		if err == nil {
			completed = report
			break
		}
		require.ErrorIs(t, err, context.Canceled)
		require.Nil(t, report)
		canceled++
	}
	require.NotNil(t, completed)
	require.Greater(t, canceled, 10)
}

// TestVerifyJointBoxLoopClearReading is §16.8's clear box: scene 11 with the
// gate over [0, 2] mm. d* is at least 2.6143 over the crank's range, so no
// configuration collides, and the minimum gap over the box is
// 72 − 2 − y_c(θ2*) = 0.6143 mm at (θ2*, 2), interior along the crank. The
// minimum is flat along the crank, but the gap falls linearly along the gate,
// so the reading meets the default tolerance by refining the gate axis.
//
// Measured: 305 centres into 153 leaves, the narrowest 1/2048 of a range.
//
// Legs seen to fail when deleted: the dependent's term in τ_half (a leaf's
// bound exceeds the true gap at its worst configuration); the reading floor
// past the verdict floor (no leaf is narrower than 1/1024).
func TestVerifyJointBoxLoopClearReading(t *testing.T) {
	t.Parallel()
	truth := 72 - 2 - rockerCornerY(rockerExtremeCrank())
	require.InDelta(t, 0.6143, truth, 1e-4)
	g := buildGateRocker(t)
	report := verifyJointBox(t, g.doc, g.linkage, g.box(0, 90, 0, 2))
	t.Logf("cells evaluated %d, leaves %d", report.CellsEvaluated, len(report.Cells))
	require.Equal(t, decad.Sound, report.Status)
	require.Empty(t, report.Diagnostics)
	require.Equal(t, units.Scalar(1.0/16384), report.ReadingResolution)
	require.Less(t, report.CellsEvaluated, 16384)
	narrowest := 1.0
	for _, cell := range report.Cells {
		require.Equal(t, decad.CellClear, cell.Outcome)
		thLo, thHi := degreesOf(t, cell.Cell.Min[0])*math.Pi/180, degreesOf(t, cell.Cell.Max[0])*math.Pi/180
		dLo, dHi := millimetresOf(t, cell.Cell.Min[3]), millimetresOf(t, cell.Cell.Max[3])
		narrowest = math.Min(narrowest, math.Min((thHi-thLo)/(math.Pi/2), (dHi-dLo)/2))
		worst := 72 - dHi - rockerCornerY(rockerNearest(thLo, thHi))
		require.NotNil(t, cell.Clearance)
		require.LessOrEqual(t, cell.Clearance.Value.Mag(), worst, `each bound is below the cell's true minimum gap`)
	}
	require.Less(t, narrowest, 1.0/1024-1e-9, `the reading refines past the verdict floor`)
	require.NotNil(t, report.Clearance)
	gap := report.Clearance.Measurement
	require.LessOrEqual(t, gap.Value.Mag()-gap.Bound.Mag(), truth)
	require.GreaterOrEqual(t, gap.Value.Mag()+gap.Bound.Mag(), truth)
	require.Equal(t, decad.ToleranceSatisfied, report.Clearance.Tolerance.State)
}

// TestVerifyJointBoxLoopClearMargins is the clear box's margins (§16.8) at a
// relative tolerance of 0.5, where the reading passes its gate on coarse
// cells and only a margin's own refinement decides it: 0.5 mm, below the true
// 0.6143 mm minimum, is met; 0.7 mm is disproven at some centre.
//
// Measured: 83 centres for 0.5 mm, 103 for 0.7 mm.
//
// Legs seen to fail when deleted: the margin's refinement (both margins read
// AssessmentUndecided).
func TestVerifyJointBoxLoopClearMargins(t *testing.T) {
	t.Parallel()
	loose := decad.WithMotionTolerance(units.Scalar(0.5))
	t.Run("a 0.5 mm margin is met", func(t *testing.T) {
		t.Parallel()
		g := buildGateRocker(t)
		report := verifyJointBox(t, g.doc, g.linkage, g.box(0, 90, 0, 2), loose, decad.WithMinClearance(units.Millimeters(0.5)))
		t.Logf("cells evaluated %d", report.CellsEvaluated)
		require.Equal(t, decad.AssessmentMet, report.Assessment)
		require.Equal(t, decad.Sound, report.Status)
	})
	t.Run("a 0.7 mm margin is violated", func(t *testing.T) {
		t.Parallel()
		g := buildGateRocker(t)
		report := verifyJointBox(t, g.doc, g.linkage, g.box(0, 90, 0, 2), loose, decad.WithMinClearance(units.Millimeters(0.7)))
		t.Logf("cells evaluated %d", report.CellsEvaluated)
		require.Equal(t, decad.AssessmentViolated, report.Assessment)
		require.Equal(t, decad.Violating, report.Status)
		violations := 0
		for _, d := range report.Diagnostics {
			if d.Code != decad.DiagMotionClearanceViolated {
				continue
			}
			violations++
			require.NotNil(t, d.Cell, `the violation names the cell whose centre proved it`)
			require.Less(t, d.Observed.Value.Mag()+d.Observed.Bound.Mag(), 0.7)
		}
		require.Positive(t, violations)
	})
}

// TestVerifyJointBoxLoopTurnBack is §16.8's turn-back cell: scene 7 without
// its wall, the crank over [0°, 81.857366°], where the follower returns to its
// zero-pose angle after dipping to −8.7632°, and a 0.8 mm pin in the
// follower's layer inside the follower's zero-pose bar — 60 mm along its axis
// from O4 and 3.5 mm off it along n4 = (−sin θ4, cos θ4) — so the pin is
// struck at both ends of the range. The root alone is evaluated: its centre,
// the crank at 40.9287°, turns the follower −8.7314° away from the pin, and
// the bar's near long edge then stands 8.0955 mm from the pin's nearest
// corner. The follower's hull over the range is [−8.7632°, 0], so
// δ = 8.7314° = 0.1524 rad and τ_half ≈ 72.67·0.1524 ≈ 11.1 mm exceeds that
// gap; half the hull's width would give 5.6 mm, which the gap exceeds.
//
// Legs seen to fail when deleted: δ as half the hull's width (the root reads
// CellClear across a struck pin); the dependent's term in τ_half (likewise).
func TestVerifyJointBoxLoopTurnBack(t *testing.T) {
	t.Parallel()
	fb, _ := buildRocker(t, false)
	t40 := rockerTheta4(0)
	ux, uy := math.Cos(t40), math.Sin(t40)
	cx, cy := rockerGround+60*ux-3.5*uy, 60*uy+3.5*ux
	pin := boxBodyAtZ(t, fb.doc, cx-0.4, cy-0.4, cx+0.4, cy+0.4, 23.6, 0.8)
	const top = 81.857366
	require.InDelta(t, 0, rockerFollowerTurn(top*math.Pi/180), 1e-7, `the follower returns to its zero-pose angle`)
	report := verifyJointBox(t, fb.doc, fb.linkage, decad.JointBox{{Link: fb.crank, Min: units.Degrees(0), Max: units.Degrees(top)}},
		decad.WithResolution(units.Scalar(1)))
	require.Equal(t, 1, report.CellsEvaluated)
	require.Len(t, report.Cells, 1)
	root := report.Cells[0]
	require.Equal(t, decad.CellUndecided, root.Outcome)
	require.Equal(t, decad.Suspect, report.Status)

	require.InDelta(t, top/2, degreesOf(t, root.Center.Values[0]), 1e-12)
	th := degreesOf(t, root.Center.Values[0]) * math.Pi / 180
	centre := rockerFollowerTurn(th)
	require.InDelta(t, -8.7314*math.Pi/180, centre, 1e-5)
	require.InDelta(t, centre, root.Center.Values[2].Mag(), 1e-9)
	dip := rockerFollowerTurn(rockerExtremeCrank())
	require.InDelta(t, -8.7632*math.Pi/180, dip, 1e-5)
	require.LessOrEqual(t, root.Cell.Min[2].Mag(), dip)
	require.GreaterOrEqual(t, root.Cell.Max[2].Mag(), 0.0)
	require.LessOrEqual(t, root.Cell.Max[2].Mag()-root.Cell.Min[2].Mag(), 1.05*(-dip)+1e-6)

	// The gap at the centre is the pin's nearest corner's offset from the
	// bar's axis, along the turned bar's normal, less the bar's half-width.
	t4 := rockerTheta4(th)
	nx, ny := -math.Sin(t4), math.Cos(t4)
	gap := math.Inf(1)
	for _, dx := range []float64{-0.4, 0.4} {
		for _, dy := range []float64{-0.4, 0.4} {
			gap = math.Min(gap, nx*(cx+dx-rockerGround)+ny*(cy+dy)-4)
		}
	}
	require.InDelta(t, 8.0955, gap, 1e-4)
	require.Len(t, root.Clearances, 1)
	row := root.Clearances[0]
	require.Same(t, fb.foll, row.A)
	require.Same(t, pin, row.B)
	require.InDelta(t, gap, row.Gap.Value.Mag(), 1e-9)
	require.Less(t, row.Gap.Bound.Mag(), 1e-9)
}

// TestVerifyJointBoxLoopHeld is §16.8's held loop: scene 11 with the crank
// held at 30° and the gate over [0, 10] mm at WithResolution(1/64). The loop
// is held, so its dependents stand still across every cell: the follower at
// θ4(30°) = 101.9717°, its turn −8.3285° from the zero pose, its corner at
// y_c = 69.3072, and the boundary at d_30 = 72 − y_c = 2.6928.
//
// Measured: 97 centres into 49 leaves.
//
// Legs seen to fail when deleted: reading a held driver's loop at the zero
// pose (the boundary moves to 4.9601, and a clear leaf reaches past d_30).
func TestVerifyJointBoxLoopHeld(t *testing.T) {
	t.Parallel()
	g := buildGateRocker(t)
	th := math.Pi / 6
	turn := rockerFollowerTurn(th)
	require.InDelta(t, -8.3285*math.Pi/180, turn, 1e-5)
	d30 := 72 - rockerCornerY(th)
	require.InDelta(t, 2.6928, d30, 1e-4)
	report := verifyJointBox(t, g.doc, g.linkage, g.box(30, 30, 0, 10), decad.WithResolution(units.Scalar(1.0/64)))
	t.Logf("cells evaluated %d, leaves %d", report.CellsEvaluated, len(report.Cells))
	require.Equal(t, decad.Interfering, report.Status)
	outcomes := map[decad.CellOutcome]int{}
	for _, cell := range report.Cells {
		outcomes[cell.Outcome]++
		dLo, dHi := millimetresOf(t, cell.Cell.Min[3]), millimetresOf(t, cell.Cell.Max[3])
		dC := millimetresOf(t, cell.Center.Values[3])
		require.InDelta(t, (dLo+dHi)/2, dC, 1e-12)
		require.Equal(t, units.Degrees(30), cell.Center.Values[0])
		switch cell.Outcome {
		case decad.CellClear:
			require.LessOrEqual(t, dHi, d30, `a clear leaf stops short of the boundary`)
			require.LessOrEqual(t, cell.Clearance.Value.Mag(), d30-dHi, `the bound never exceeds the true gap`)
		case decad.CellColliding, decad.CellBlocked:
			require.Greater(t, dC, d30)
		case decad.CellUndecided:
			require.LessOrEqual(t, dHi-dLo, 10.0/64+1e-12)
		default:
			require.Failf(t, `unexpected outcome`, `%s`, cell.Outcome)
		}
		require.InDelta(t, turn, cell.Center.Values[2].Mag(), 1e-9)
		require.Positive(t, cell.Center.Bounds[2].Mag())
		require.Less(t, cell.Center.Bounds[2].Mag(), 1e-9)
		require.InDelta(t, turn, cell.Cell.Min[2].Mag(), 1e-9)
		require.InDelta(t, turn, cell.Cell.Max[2].Mag(), 1e-9)
	}
	require.Positive(t, outcomes[decad.CellClear])
}

// TestVerifyJointBoxLoopBlocked is §16.8's blocked box: scene 11 with the
// gate cut to x ∈ [60, 120] (area 2600 mm²), the crank over [35°, 42°] and
// the gate over [8, 10] mm, at WithResolution(1/64). Over that crank range
// y_c stays within [69.3725, 69.3857], so the corner lies at least 5.37 mm
// past the gate's underside y = 72 − d ∈ [62, 64], beyond the bar's full
// width 8·|cos θ4| ≈ 1.61: the overlap is the trapezoidal prism
// 8·8·(δ − 4·|cos θ4|)/sin θ4 ≥ 298 mm³, and at the floor the allowance is
// about ½·(2/64)·2600 + ρ·δ_f·2368 ≈ 57 mm³, so the box resolves into blocked
// cells.
//
// Measured: 83 centres into 42 leaves.
//
// Legs seen to fail when deleted: the blocked certificate (no leaf blocks:
// every cell splits to the floor along both axes, 8191 centres, and reads
// CellColliding).
func TestVerifyJointBoxLoopBlocked(t *testing.T) {
	t.Parallel()
	g := buildGateRockerSpan(t, 60, 120)
	report := verifyJointBox(t, g.doc, g.linkage, g.box(35, 42, 8, 10), decad.WithResolution(units.Scalar(1.0/64)))
	t.Logf("cells evaluated %d, leaves %d", report.CellsEvaluated, len(report.Cells))
	require.Equal(t, decad.Interfering, report.Status)
	require.Less(t, report.CellsEvaluated, 512)
	for _, cell := range report.Cells {
		require.Equal(t, decad.CellBlocked, cell.Outcome)
		require.NotEmpty(t, cell.Interferences, `a blocked cell's centre collision is its witness`)
	}
	for _, hit := range report.Collisions {
		require.Same(t, g.foll, hit.A)
		require.Same(t, g.gateBody, hit.B)
		th, d := degreesOf(t, hit.Configuration.Values[0])*math.Pi/180, millimetresOf(t, hit.Configuration.Values[3])
		t4 := rockerTheta4(th)
		depth := rockerCornerY(th) - (72 - d)
		require.Greater(t, depth, 8*math.Abs(math.Cos(t4)), `both end corners have crossed`)
		require.Less(t, hit.Volume.Bound.Mag(), hit.Volume.Value.Mag())
		require.InDelta(t, 64*(depth-4*math.Abs(math.Cos(t4)))/math.Sin(t4), hit.Volume.Value.Mag(), 1e-6)
	}
}
