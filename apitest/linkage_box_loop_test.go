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
	fb, _ := buildRocker(t, false)
	g := gateRocker{fourBar: fb}
	g.gateBody = boxBodyAtZ(t, fb.doc, -50, 72, 150, 82, 19, 10)
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
