package apitest_test

import (
	"context"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file holds the PR 1 tests of docs/linkage-check-design.md §14.8:
// scene 6 (the crane's box), the budget, the one-joint boxes,
// Linkage.Configuration and the standing tests of VerifyJointBox. Bounds are
// asserted against geometric truths, never pinned to a measured literal: FMA
// contraction moves the last ulp between amd64 and arm64.

// craneBox is scene 6: scene 3's mast x, y ∈ [−5, 5], z ∈ [0, 38] on a
// revolute joint about Z through the origin and its boom x ∈ [10, 60],
// y ∈ [−5, 5], z ∈ [40, 50] on a prismatic joint along the mast's +X, the
// mast turning θ ∈ [0°, 80°] and the boom sliding d ∈ [0, 30] mm, against a
// wall x ∈ [−100, 150], y ∈ [wallY, wallY + 20], z ∈ [30, 100].
type craneBox struct {
	doc              *decad.Document
	mast, boom, wall *decad.Body
	linkage          *decad.Linkage
	turn, extend     *decad.Link
}

func buildCraneBox(t *testing.T, wallY float64) craneBox {
	t.Helper()
	return buildCraneBoxWithMast(t, wallY, 38)
}

// buildCraneBoxWithMast is scene 6 with the mast standing mastTop mm tall,
// 40 − mastTop mm under the boom.
func buildCraneBoxWithMast(t *testing.T, wallY, mastTop float64) craneBox {
	t.Helper()
	c := craneBox{doc: decad.New()}
	c.mast = boxBody(t, c.doc, -5, -5, 5, 5, mastTop)
	c.boom = boxBodyAtZ(t, c.doc, 10, -5, 60, 5, 40, 10)
	c.wall = boxBodyAtZ(t, c.doc, -100, wallY, 150, wallY+20, 30, 70)
	c.linkage = decad.NewLinkage()
	var err error
	c.turn, err = c.linkage.Ground().Revolute(r3.Vec{}, zAxis, []*decad.Body{c.mast})
	require.NoError(t, err)
	c.extend, err = c.turn.Prismatic(r3.NewVec(1, 0, 0), []*decad.Body{c.boom})
	require.NoError(t, err)
	return c
}

func (c craneBox) box() decad.JointBox {
	return decad.JointBox{
		{Link: c.turn, Min: units.Degrees(0), Max: units.Degrees(80)},
		{Link: c.extend, Min: units.Millimeters(0), Max: units.Millimeters(30)},
	}
}

// boomTipY is the height of the boom's +5 tip corner, (60 + d)·sin θ +
// 5·cos θ, at θ degrees and d millimetres. It increases in d, and in θ below
// 85.2°, so over scene 6's box a cell's (Max θ, Max d) corner is its highest
// and its (Min θ, Min d) corner its lowest; the boom's gap to a wall face
// y = Y above it is Y − boomTipY.
func boomTipY(thetaDeg, d float64) float64 {
	th := thetaDeg * math.Pi / 180
	return (60+d)*math.Sin(th) + 5*math.Cos(th)
}

func verifyJointBox(t *testing.T, doc *decad.Document, l *decad.Linkage, box decad.JointBox, opts ...decad.JointBoxOption) *decad.JointBoxReport {
	t.Helper()
	before := doc.Bodies()
	report, err := doc.VerifyJointBox(t.Context(), l, box, opts...)
	require.NoError(t, err)
	require.Equal(t, before, doc.Bodies(), `VerifyJointBox must leave the live body set and its order unchanged`)
	return report
}

func degreesOf(t *testing.T, v units.Value) float64 {
	t.Helper()
	deg, err := v.In(units.Degree)
	require.NoError(t, err)
	return deg
}

func millimetresOf(t *testing.T, v units.Value) float64 {
	t.Helper()
	mm, err := v.In(units.Millimeter)
	require.NoError(t, err)
	return mm
}

// craneCell is a scene 6 cell in degrees and millimetres, with its depth in
// the subdivision: a cell halved n times spans 2^−n of the box.
type craneCell struct {
	thLo, thHi, dLo, dHi float64
	depth                int
}

func readCraneCell(t *testing.T, cell decad.JointCell) craneCell {
	t.Helper()
	c := craneCell{
		thLo: degreesOf(t, cell.Min[0]), thHi: degreesOf(t, cell.Max[0]),
		dLo: millimetresOf(t, cell.Min[1]), dHi: millimetresOf(t, cell.Max[1]),
	}
	c.depth = int(math.Round(-math.Log2((c.thHi - c.thLo) / 80 * (c.dHi - c.dLo) / 30)))
	return c
}

// requireTiles asserts scene 6's leaves tile the box: their areas sum to the
// box's and no two share interior.
func requireTiles(t *testing.T, cells []craneCell) {
	t.Helper()
	area := 0.0
	for _, c := range cells {
		area += (c.thHi - c.thLo) / 80 * (c.dHi - c.dLo) / 30
	}
	require.InDelta(t, 1, area, 1e-12, `the leaves cover the box`)
	for a := range cells {
		for b := a + 1; b < len(cells); b++ {
			th := math.Min(cells[a].thHi, cells[b].thHi) - math.Max(cells[a].thLo, cells[b].thLo)
			d := math.Min(cells[a].dHi, cells[b].dHi) - math.Max(cells[a].dLo, cells[b].dLo)
			require.Falsef(t, th > 1e-9 && d > 1e-9, `leaves %d and %d share interior`, a, b)
		}
	}
}

// TestVerifyJointBoxCrane is scene 6 of docs/linkage-check-design.md §14.8,
// the acceptance target. The boom's tip corner reaches the wall's face
// y = 62 on the one curve d*(θ) = (62 − 5·cos θ)/sin θ − 60, from θ ≈ 40.29°
// at d = 30 to d ≈ 2.075 at θ = 80°; the colliding region lies above it. The
// (mast, boom) pair is settled by the layer exclusion along Z and the (mast,
// wall) pair by its swept box, so each centre evaluates (boom, wall) alone.
//
// The ball reading of the boom about the mast's axis is
// ρ = 35 + √675 + 30 ≈ 91 (§5.2), so a cell spanning Δθ radians and Δd mm
// has τ_half = ½·(ρ·Δθ + Δd), and an undecided cell's centre lies within that
// reach of the boundary.
//
// A colliding centre blocks its cell only once the cell's τ_half falls under
// about 0.9 mm, near 1/128 of the θ range, so at any floor down to 1/64 every
// colliding cell splits to the floor and none blocks. This runs at
// WithResolution(1/16): 239 centres evaluated, 120 leaves. At
// WithResolution(1/64) the same check evaluates 2953 centres into 1477
// leaves, about 15 s, 70 s under the race detector.
//
// Legs seen to fail when deleted: halving τ_half again, and dropping the
// prismatic joint's term from it (a clear leaf's bound exceeds the true gap
// at its worst corner); charging the blocked certificate a too-small area
// (a floor cell straddling the boundary reads blocked with a clear corner).
func TestVerifyJointBoxCrane(t *testing.T) {
	t.Parallel()
	c := buildCraneBox(t, 62)
	const res = 1.0 / 16
	report := verifyJointBox(t, c.doc, c.linkage, c.box(), decad.WithResolution(units.Scalar(res)))

	require.Equal(t, decad.Interfering, report.Status)
	require.Equal(t, []*decad.Body{c.wall}, report.Against)
	require.Equal(t, decad.JointBoxRequest{
		RelativeTolerance: units.Scalar(1e-3), Resolution: units.Scalar(res), CellBudget: 16384,
	}, report.Request)
	require.Equal(t, 2*len(report.Cells)-1, report.CellsEvaluated, `every split evaluates two centres`)
	require.Less(t, report.CellsEvaluated, 16384)

	rho := 35 + math.Sqrt(675) + 30
	cells := make([]craneCell, len(report.Cells))
	outcomes := map[decad.CellOutcome]int{}
	for n, cell := range report.Cells {
		cc := readCraneCell(t, cell.Cell)
		cells[n] = cc
		outcomes[cell.Outcome]++
		thC, dC := degreesOf(t, cell.Center.Values[0]), millimetresOf(t, cell.Center.Values[1])
		require.InDelta(t, (cc.thLo+cc.thHi)/2, thC, 1e-12, `the centre is the cell's midpoint`)
		require.InDelta(t, (cc.dLo+cc.dHi)/2, dC, 1e-12, `the centre is the cell's midpoint`)
		for _, row := range cell.Clearances {
			require.True(t, row.A == c.boom && row.B == c.wall, `only (boom, wall) is evaluated`)
		}
		for _, row := range cell.Interferences {
			require.True(t, row.A == c.boom && row.B == c.wall, `only (boom, wall) is evaluated`)
		}
		switch cell.Outcome {
		case decad.CellClear:
			worst := boomTipY(cc.thHi, cc.dHi)
			require.Less(t, worst, 62.0, `a clear cell holds no colliding configuration`)
			require.NotNil(t, cell.Clearance)
			require.Positive(t, cell.Clearance.Value.Mag())
			require.LessOrEqual(t, cell.Clearance.Value.Mag(), 62-worst, `the cell's bound never exceeds its true minimum gap`)
		case decad.CellBlocked:
			require.Greater(t, boomTipY(cc.thLo, cc.dLo), 62.0, `a blocked cell holds no clear configuration`)
			require.Nil(t, cell.Clearance)
			require.NotEmpty(t, cell.Interferences, `a blocked cell's centre collision is its witness`)
		case decad.CellColliding:
			require.Greater(t, boomTipY(thC, dC), 62.0)
			require.LessOrEqual(t, (cc.thHi-cc.thLo)/80, res, `a colliding cell splits to the floor`)
			require.LessOrEqual(t, (cc.dHi-cc.dLo)/30, res, `a colliding cell splits to the floor`)
			require.Nil(t, cell.Clearance)
		case decad.CellUndecided:
			require.LessOrEqual(t, (cc.thHi-cc.thLo)/80, res, `an undecided cell splits to the floor`)
			require.LessOrEqual(t, (cc.dHi-cc.dLo)/30, res, `an undecided cell splits to the floor`)
			tauHalf := (rho*(cc.thHi-cc.thLo)*math.Pi/180 + (cc.dHi - cc.dLo)) / 2
			require.LessOrEqual(t, math.Abs(62-boomTipY(thC, dC)), tauHalf+1e-3, `the boundary passes within the cell's reach`)
			require.Nil(t, cell.Clearance)
		default:
			require.Failf(t, `unexpected outcome`, `%s`, cell.Outcome)
		}
	}
	requireTiles(t, cells)
	require.Positive(t, outcomes[decad.CellClear])
	require.Positive(t, outcomes[decad.CellColliding])

	shallow := 0
	for _, hit := range report.Collisions {
		require.Same(t, c.boom, hit.A)
		require.Same(t, c.wall, hit.B)
		th, d := degreesOf(t, hit.Configuration.Values[0]), millimetresOf(t, hit.Configuration.Values[1])
		depth := boomTipY(th, d) - 62
		require.Positive(t, depth)
		require.Less(t, hit.Volume.Bound.Mag(), hit.Volume.Value.Mag())
		rad := th * math.Pi / 180
		if depth < 10*math.Cos(rad) && depth < 50*math.Sin(rad) && depth < 20 {
			// Only the tip corner has crossed: a triangular prism with legs
			// δ/sin θ and δ/cos θ over the boom's 10 mm height.
			shallow++
			require.InDelta(t, 10*depth*depth/(2*math.Sin(rad)*math.Cos(rad)), hit.Volume.Value.Mag(), 1e-6)
		}
	}
	require.Positive(t, shallow, `some collision crosses the wall at one corner only`)
	for _, d := range report.Diagnostics {
		require.NotEqual(t, decad.DiagJointBoxBudgetExhausted, d.Code)
		require.Nil(t, d.At, `a joint-box finding carries no scalar parameter`)
		require.NotNil(t, d.Cell, `a joint-box finding names its cell`)
	}
}

// TestVerifyJointBoxBlocked is §14.8's blocked box: scene 6's crane over
// θ ∈ [70°, 80°], d ∈ [25, 30] mm. The tip's lowest point over the box,
// y(70°, 25) ≈ 81.6, lies past the wall's face y = 62, so every configuration
// collides; near the centre the boom passes through the whole 20 mm wall,
// an overlap of about 100·20/sin θ ≈ 2071 mm³ against a boom area of
// 2200 mm². The blocked certificate closes once a cell's τ_half falls under
// about 0.9 mm, so the box resolves into blocked cells long before the
// default floor.
//
// Measured: 255 centres into 128 leaves.
//
// Legs seen to fail when deleted: the blocked certificate (the run splits to
// the floor, exhausts the budget and reads every leaf CellColliding).
func TestVerifyJointBoxBlocked(t *testing.T) {
	t.Parallel()
	c := buildCraneBox(t, 62)
	report := verifyJointBox(t, c.doc, c.linkage, decad.JointBox{
		{Link: c.turn, Min: units.Degrees(70), Max: units.Degrees(80)},
		{Link: c.extend, Min: units.Millimeters(25), Max: units.Millimeters(30)},
	})
	require.Equal(t, decad.Interfering, report.Status)
	require.Less(t, report.CellsEvaluated, 1024)
	require.Greater(t, report.CellsEvaluated, 1, `the root alone does not block`)
	for _, cell := range report.Cells {
		require.Equal(t, decad.CellBlocked, cell.Outcome)
		require.Nil(t, cell.Clearance)
		require.NotEmpty(t, cell.Interferences)
	}
	for _, d := range report.Diagnostics {
		require.NotEqual(t, decad.DiagJointBoxBudgetExhausted, d.Code)
		require.NotEqual(t, decad.DiagMotionUndecidedInterval, d.Code)
	}
}

// TestVerifyJointBoxBudget: scene 6 at the default resolution under a cell
// budget. A budget of 16 evaluates at most 16 centres, raises
// DiagJointBoxBudgetExhausted once, and leaves the box examined level by
// level: no leaf the budget held back is more than one level shallower than
// the deepest leaf. A budget of 1 evaluates the whole box's centre alone,
// (40°, 15 mm), where the tip stands at 75·sin 40° + 5·cos 40° ≈ 52.0, clear
// of the wall but not by the root's τ_half of about 79 mm.
//
// Legs seen to fail when deleted: the shallowest-first order (a depth-first
// split leaves held-back leaves far shallower than the deepest).
func TestVerifyJointBoxBudget(t *testing.T) {
	t.Parallel()
	t.Run("a budget of 16 stops the split", func(t *testing.T) {
		t.Parallel()
		c := buildCraneBox(t, 62)
		report := verifyJointBox(t, c.doc, c.linkage, c.box(), decad.WithCellBudget(16))
		require.Equal(t, 16, report.Request.CellBudget)
		require.LessOrEqual(t, report.CellsEvaluated, 16)
		require.Equal(t, decad.Interfering, report.Status)
		budget := 0
		for _, d := range report.Diagnostics {
			if d.Code == decad.DiagJointBoxBudgetExhausted {
				budget++
				require.Equal(t, decad.Suspect, d.Status)
				require.Nil(t, d.Cell)
				require.Nil(t, d.Pair)
			}
		}
		require.Equal(t, 1, budget)
		deepest := 0
		for _, cell := range report.Cells {
			deepest = max(deepest, readCraneCell(t, cell.Cell).depth)
		}
		held := 0
		for _, cell := range report.Cells {
			if cell.Outcome == decad.CellClear || cell.Outcome == decad.CellBlocked {
				continue
			}
			held++
			require.GreaterOrEqual(t, readCraneCell(t, cell.Cell).depth, deepest-1, `the box is examined level by level`)
		}
		require.Positive(t, held, `some leaf wider than the floor is held back`)
	})
	t.Run("a budget of 1 evaluates the root alone", func(t *testing.T) {
		t.Parallel()
		c := buildCraneBox(t, 62)
		report := verifyJointBox(t, c.doc, c.linkage, c.box(), decad.WithCellBudget(1))
		require.Equal(t, 1, report.CellsEvaluated)
		require.Len(t, report.Cells, 1)
		root := report.Cells[0]
		require.Equal(t, decad.CellUndecided, root.Outcome)
		require.Equal(t, []units.Value{units.Degrees(0), units.Millimeters(0)}, root.Cell.Min)
		require.Equal(t, []units.Value{units.Degrees(80), units.Millimeters(30)}, root.Cell.Max)
		require.Equal(t, []units.Value{units.Degrees(40), units.Millimeters(15)}, root.Center.Values)
		require.Len(t, root.Clearances, 1)
		require.InDelta(t, 62-boomTipY(40, 15), root.Clearances[0].Gap.Value.Mag(), 1e-3)
		require.Equal(t, decad.Suspect, report.Status)
	})
}

// TestVerifyJointBoxOneJoint covers one varying joint
// (docs/linkage-check-design.md §14.8).
//
//   - Scene 2's elbow as the box [0°, 180°], the shoulder unlisted: the
//     forearm's leading corner reaches the fence at φ* = acos(−19/25) −
//     atan(7/24) ≈ 123.20°, and stays in it to 180°. Every clear leaf ends at
//     or below φ*, every colliding leaf's centre lies past it, and the
//     declared (A, B) contact raises nothing.
//   - Scene 4b's blade held at 180° on its own joint and the hub's joint the
//     one varying joint over [0°, 90°]: the pin at radius 149 is found, every
//     witness within 0.41° of 90·31/64°.
//
// Legs seen to fail when deleted: the revolute ball's radius in ρ_{ik} (4b's
// cell around the pin certifies clear and no witness is found).
func TestVerifyJointBoxOneJoint(t *testing.T) {
	t.Parallel()
	t.Run("the stacked elbow", func(t *testing.T) {
		t.Parallel()
		e := buildStackedElbow(t, true)
		phiStar := (math.Acos(-19.0/25) - math.Atan(7.0/24)) * 180 / math.Pi
		report := verifyJointBox(t, e.doc, e.linkage, decad.JointBox{{Link: e.elbow, Min: units.Degrees(0), Max: units.Degrees(180)}},
			decad.WithResolution(units.Scalar(1.0/256)))
		require.Equal(t, decad.Interfering, report.Status)
		require.Equal(t, []decad.DiagnosticPair{{A: e.upper, B: e.forearm}}, report.JointContacts)
		colliding := 0
		for _, cell := range report.Cells {
			require.Equal(t, units.Radians(0), cell.Cell.Min[0], `the unlisted shoulder holds 0`)
			lo, hi := degreesOf(t, cell.Cell.Min[1]), degreesOf(t, cell.Cell.Max[1])
			switch cell.Outcome {
			case decad.CellClear:
				require.LessOrEqual(t, hi, phiStar)
			case decad.CellBlocked:
				colliding++
				require.Greater(t, lo, phiStar)
			case decad.CellColliding:
				colliding++
				require.Greater(t, (lo+hi)/2, phiStar)
			}
		}
		require.Positive(t, colliding)
		for _, hit := range report.Collisions {
			require.Same(t, e.fence, hit.A)
			require.Same(t, e.forearm, hit.B)
		}
		for _, d := range report.Diagnostics {
			if d.Pair != nil {
				named := (d.Pair.A == e.upper && d.Pair.B == e.forearm) || (d.Pair.A == e.forearm && d.Pair.B == e.upper)
				require.Falsef(t, named, `a declared joint contact raised %s`, d.Code)
			}
		}
	})
	t.Run("a held elbow's ball", func(t *testing.T) {
		t.Parallel()
		doc := decad.New()
		hub := boxBodyAtZ(t, doc, -5, -5, 5, 5, -10, 5)
		blade := boxBody(t, doc, 50, -0.5, 100, 0.5, 10)
		nearMissPin(t, doc, 149)
		l := decad.NewLinkage()
		swing, err := l.Ground().Revolute(r3.Vec{}, zAxis, []*decad.Body{hub})
		require.NoError(t, err)
		elbow, err := swing.Revolute(r3.NewVec(100, 0, 0), zAxis, []*decad.Body{blade})
		require.NoError(t, err)
		report := verifyJointBox(t, doc, l, decad.JointBox{
			{Link: swing, Min: units.Degrees(0), Max: units.Degrees(90)},
			{Link: elbow, Min: units.Degrees(180), Max: units.Degrees(180)},
		}, decad.WithResolution(units.Scalar(1.0/900)))
		require.Equal(t, decad.Interfering, report.Status)
		require.NotEmpty(t, report.Collisions)
		for _, hit := range report.Collisions {
			require.Same(t, blade, hit.A)
			require.Equal(t, units.Degrees(180), hit.Configuration.Values[1])
			require.InDelta(t, 90.0*31/64, degreesOf(t, hit.Configuration.Values[0]), 0.41)
		}
	})
}

// TestLinkageConfiguration pins Linkage.Configuration
// (docs/linkage-check-design.md §14.1): scene 1's forearm point (96, 0, 0) at
// shoulder 30° and elbow −30° is (48·cos 30°, 48·sin 30°, 0) + (48, 0, 0),
// PoseAt at s = 1/3 is Configuration of its own values bit for bit, and each
// refusal carries its sentinel.
func TestLinkageConfiguration(t *testing.T) {
	t.Parallel()
	a := buildFoldingArm(t, false)
	conf, err := a.linkage.Configuration([]units.Value{units.Degrees(30), units.Degrees(-30)})
	require.NoError(t, err)
	got := conf.Poses[1].Apply(r3.NewVec(96, 0, 0))
	want := r3.NewVec(48*math.Cos(math.Pi/6)+48, 48*math.Sin(math.Pi/6), 0)
	require.InDelta(t, want.X, got.X, 1e-12)
	require.InDelta(t, want.Y, got.Y, 1e-12)
	require.InDelta(t, want.Z, got.Z, 1e-12)

	pose, err := a.linkage.PoseAt(a.drive(), units.Scalar(1.0/3))
	require.NoError(t, err)
	same, err := a.linkage.Configuration(pose.Values)
	require.NoError(t, err)
	require.Equal(t, pose.Values, same.Values)
	require.Equal(t, pose.Poses, same.Poses, `PoseAt is Configuration of the drive's values`)

	limited := decad.NewLinkage()
	_, err = limited.Ground().Revolute(r3.Vec{}, zAxis, []*decad.Body{a.upper}, decad.WithJointLimits(units.Degrees(-10), units.Degrees(10)))
	require.NoError(t, err)
	far := decad.NewLinkage()
	_, err = far.Ground().Revolute(r3.NewVec(math.MaxFloat64, math.MaxFloat64, 0), zAxis, []*decad.Body{a.upper})
	require.NoError(t, err)
	var nilLinkage *decad.Linkage
	cases := []struct {
		name   string
		l      *decad.Linkage
		values []units.Value
		want   error
	}{
		{"a nil linkage", nilLinkage, nil, decad.ErrDegenerate},
		{"too few values", a.linkage, []units.Value{units.Degrees(0)}, decad.ErrDegenerate},
		{"too many values", a.linkage, []units.Value{units.Degrees(0), units.Degrees(0), units.Degrees(0)}, decad.ErrDegenerate},
		{"a wrong-kind value", a.linkage, []units.Value{units.Degrees(0), units.Millimeters(0)}, decad.ErrUnitKind},
		{"a non-finite value", a.linkage, []units.Value{units.Degrees(math.NaN()), units.Degrees(0)}, decad.ErrNotFinite},
		{"a value outside the limits", limited, []units.Value{units.Degrees(11)}, decad.ErrDegenerate},
		{"a pose r3 cannot represent", far, []units.Value{units.Degrees(90)}, decad.ErrNotFinite},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			_, err := tc.l.Configuration(tc.values)
			require.ErrorIs(t, err, tc.want)
		})
	}
	t.Run("a value at the limit poses", func(t *testing.T) {
		_, err := limited.Configuration([]units.Value{units.Degrees(10)})
		require.NoError(t, err)
	})
}

// TestJointBoxOptionSet pins §14.1's widened option set: every MotionOption
// is a JointBoxOption, and WithCellBudget is not a MotionOption, so passing
// it to VerifyMotion or VerifyLinkage does not compile.
func TestJointBoxOptionSet(t *testing.T) {
	t.Parallel()
	shared := []decad.JointBoxOption{
		decad.WithMotionTolerance(units.Scalar(1e-3)),
		decad.WithResolution(units.Scalar(1.0 / 8)),
		decad.WithMinClearance(units.Millimeters(1)),
	}
	require.Len(t, shared, 3)
	_, isMotion := any(decad.WithCellBudget(4)).(decad.MotionOption)
	require.False(t, isMotion, `the cell budget configures VerifyJointBox alone`)
}

// TestVerifyJointBoxErrors is one subtest per row of
// docs/linkage-check-design.md §14.6's table and per shared row of §8 the
// public API can reach, each asserting the sentinel, no report, and an
// unchanged document. The row for a body this evaluator did not build lives
// in linkage_internal_test.go.
func TestVerifyJointBoxErrors(t *testing.T) {
	t.Parallel()
	c := buildCraneBox(t, 62)
	doc := c.doc
	retired := boxBody(t, doc, 200, 200, 210, 210, 10)
	translated(t, retired, 0, 0, 100)
	foreign := boxBody(t, decad.New(), 0, 0, 10, 10, 10)
	withBody := func(b *decad.Body) *decad.Linkage {
		l := decad.NewLinkage()
		_, err := l.Ground().Prismatic(r3.NewVec(1, 0, 0), []*decad.Body{b})
		require.NoError(t, err)
		return l
	}
	slide := func(l *decad.Linkage) decad.JointBox {
		return decad.JointBox{{Link: l.Links()[0], Min: units.Millimeters(0), Max: units.Millimeters(1)}}
	}
	turn := func(l *decad.Linkage) decad.JointBox {
		return decad.JointBox{{Link: l.Links()[0], Min: units.Degrees(0), Max: units.Degrees(1)}}
	}
	retiredLinkage, foreignLinkage := withBody(retired), withBody(foreign)
	far := decad.NewLinkage()
	_, err := far.Ground().Revolute(r3.NewVec(math.MaxFloat64, math.MaxFloat64, 0), zAxis, []*decad.Body{c.mast})
	require.NoError(t, err)
	limited := decad.NewLinkage()
	_, err = limited.Ground().Revolute(r3.Vec{}, zAxis, []*decad.Body{c.mast}, decad.WithJointLimits(units.Degrees(-10), units.Degrees(10)))
	require.NoError(t, err)
	other := buildCraneBox(t, 62)
	canceled, cancel := context.WithCancel(t.Context())
	cancel()
	held := decad.JointBox{{Link: c.turn, Min: units.Degrees(30), Max: units.Degrees(30)}}
	contactWith := func(b *decad.Body) *decad.Linkage {
		l := decad.NewLinkage()
		_, err := l.Ground().Revolute(r3.Vec{}, zAxis, []*decad.Body{c.mast})
		require.NoError(t, err)
		require.NoError(t, l.DeclareJointContact(c.mast, b))
		return l
	}
	retiredContact, foreignContact := contactWith(retired), contactWith(foreign)
	opt := func(o decad.JointBoxOption) []decad.JointBoxOption { return []decad.JointBoxOption{o} }

	cases := []struct {
		name string
		ctx  context.Context //nolint:containedctx // a table of per-case contexts.
		l    *decad.Linkage
		box  decad.JointBox
		opts []decad.JointBoxOption
		want error
	}{
		{name: "a nil linkage", box: c.box(), want: decad.ErrDegenerate},
		{name: "a linkage with no link", l: decad.NewLinkage(), want: decad.ErrDegenerate},
		{name: "an empty box", l: c.linkage, want: decad.ErrDegenerate},
		{name: "a box with no varying joint", l: c.linkage, box: held, want: decad.ErrDegenerate},
		{name: "a box held across units", l: c.linkage, box: decad.JointBox{{Link: c.turn, Min: units.Degrees(0), Max: units.Radians(0)}}, want: decad.ErrDegenerate},
		{name: "a range with Min above Max", l: c.linkage, box: decad.JointBox{{Link: c.turn, Min: units.Degrees(10), Max: units.Degrees(0)}}, want: decad.ErrDegenerate},
		{name: "a link named twice", l: c.linkage, box: append(c.box(), c.box()[1]), want: decad.ErrDegenerate},
		{name: "a link of another linkage", l: c.linkage, box: decad.JointBox{{Link: other.turn, Min: units.Degrees(0), Max: units.Degrees(1)}}, want: decad.ErrDegenerate},
		{name: "a range outside the limits", l: limited, box: decad.JointBox{{Link: limited.Links()[0], Min: units.Degrees(0), Max: units.Degrees(20)}}, want: decad.ErrDegenerate},
		{name: "a zero cell budget", l: c.linkage, box: c.box(), opts: opt(decad.WithCellBudget(0)), want: decad.ErrDegenerate},
		{name: "a negative cell budget", l: c.linkage, box: c.box(), opts: opt(decad.WithCellBudget(-1)), want: decad.ErrDegenerate},
		{name: "a nil option", l: c.linkage, box: c.box(), opts: []decad.JointBoxOption{nil}, want: decad.ErrDegenerate},
		{name: "a retired link body", l: retiredLinkage, box: slide(retiredLinkage), want: decad.ErrRetiredBody},
		{name: "a foreign link body", l: foreignLinkage, box: slide(foreignLinkage), want: decad.ErrForeignBody},
		{name: "a retired declared contact", l: retiredContact, box: turn(retiredContact), want: decad.ErrRetiredBody},
		{name: "a foreign declared contact", l: foreignContact, box: turn(foreignContact), want: decad.ErrForeignBody},
		{name: "a wrong-kind range", l: c.linkage, box: decad.JointBox{{Link: c.turn, Min: units.Millimeters(0), Max: units.Millimeters(1)}}, want: decad.ErrUnitKind},
		{name: "a wrong-kind resolution", l: c.linkage, box: c.box(), opts: opt(decad.WithResolution(units.Degrees(1))), want: decad.ErrUnitKind},
		{name: "a wrong-kind tolerance", l: c.linkage, box: c.box(), opts: opt(decad.WithMotionTolerance(units.Millimeters(1))), want: decad.ErrUnitKind},
		{name: "a wrong-kind minimum", l: c.linkage, box: c.box(), opts: opt(decad.WithMinClearance(units.Degrees(1))), want: decad.ErrUnitKind},
		{name: "a non-finite range", l: c.linkage, box: decad.JointBox{{Link: c.turn, Min: units.Degrees(math.Inf(-1)), Max: units.Degrees(1)}}, want: decad.ErrNotFinite},
		{name: "a non-finite resolution", l: c.linkage, box: c.box(), opts: opt(decad.WithResolution(units.Scalar(math.NaN()))), want: decad.ErrNotFinite},
		{name: "a pose r3 cannot represent", l: far, box: decad.JointBox{{Link: far.Links()[0], Min: units.Degrees(0), Max: units.Degrees(90)}}, want: decad.ErrNotFinite},
		{name: "a negative resolution", l: c.linkage, box: c.box(), opts: opt(decad.WithResolution(units.Scalar(-1))), want: decad.ErrNegativeMagnitude},
		{name: "a zero resolution", l: c.linkage, box: c.box(), opts: opt(decad.WithResolution(units.Scalar(0))), want: decad.ErrNegativeMagnitude},
		{name: "a negative tolerance", l: c.linkage, box: c.box(), opts: opt(decad.WithMotionTolerance(units.Scalar(-1))), want: decad.ErrNegativeMagnitude},
		{name: "a negative minimum", l: c.linkage, box: c.box(), opts: opt(decad.WithMinClearance(units.Millimeters(-1))), want: decad.ErrNegativeMagnitude},
		{name: "a zero minimum", l: c.linkage, box: c.box(), opts: opt(decad.WithMinClearance(units.Millimeters(0))), want: decad.ErrDegenerate},
		{name: "validation wins over a canceled context", ctx: canceled, l: c.linkage, box: held, want: decad.ErrDegenerate},
		{name: "canceled after validation", ctx: canceled, l: c.linkage, box: c.box(), want: context.Canceled},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			ctx := tc.ctx
			if ctx == nil {
				ctx = t.Context()
			}
			before := doc.Bodies()
			report, err := doc.VerifyJointBox(ctx, tc.l, tc.box, tc.opts...)
			require.ErrorIs(t, err, tc.want)
			require.Nil(t, report)
			require.Equal(t, before, doc.Bodies())
		})
	}
	t.Run("nil document", func(t *testing.T) {
		var nilDoc *decad.Document
		report, err := nilDoc.VerifyJointBox(t.Context(), c.linkage, c.box())
		require.ErrorIs(t, err, decad.ErrDegenerate)
		require.Nil(t, report)
	})
	t.Run("nil context", func(t *testing.T) {
		var nilCtx context.Context
		report, err := doc.VerifyJointBox(nilCtx, c.linkage, c.box())
		require.ErrorIs(t, err, decad.ErrDegenerate)
		require.Nil(t, report)
	})
}

// TestVerifyJointBoxNonMutationAndDeterminism: the live body set and its
// order survive the call, and two calls on the same inputs return reports
// equal in every field.
func TestVerifyJointBoxNonMutationAndDeterminism(t *testing.T) {
	t.Parallel()
	c := buildCraneBox(t, 62)
	before := c.doc.Bodies()
	first := verifyJointBox(t, c.doc, c.linkage, c.box(), decad.WithResolution(units.Scalar(1.0/8)))
	second := verifyJointBox(t, c.doc, c.linkage, c.box(), decad.WithResolution(units.Scalar(1.0/8)))
	require.Equal(t, before, c.doc.Bodies())
	require.Equal(t, first, second)
}

// TestVerifyJointBoxCancellation: a context canceled at successively later
// checks after validation — at the root and on through the subdivision of the
// clear crane box — returns ctx.Err() and no report, and leaves the document
// unchanged.
func TestVerifyJointBoxCancellation(t *testing.T) {
	t.Parallel()
	c := buildCraneBox(t, 100)
	before := c.doc.Bodies()
	var completed *decad.JointBoxReport
	canceled := 0
	for limit := int32(1); limit <= 1<<24; limit += 1 + limit/2 {
		ctx := newCancelAfterContext(t.Context(), limit)
		report, err := c.doc.VerifyJointBox(ctx, c.linkage, c.box(), decad.WithResolution(units.Scalar(1.0/16)))
		require.Equal(t, before, c.doc.Bodies())
		if err == nil {
			completed = report
			break
		}
		require.ErrorIs(t, err, context.Canceled)
		require.Nil(t, report)
		canceled++
	}
	require.NotNil(t, completed, `the check finishes once the context outlasts it`)
	require.Greater(t, completed.CellsEvaluated, 2, `the probed checks reach into the subdivision`)
	require.Greater(t, canceled, 5)
}

// TestVerifyJointBoxPoseDeviationIsCharged: an arm turns about a pivot
// (cx, 0, 0) over [80°, 100°], whose centre is 90°, and a block rides past its
// tip on a prismatic joint the box holds at 0. A wall stands 10 mm beyond the
// block at 90°. The far pivot's pose rounds at its magnitude, and the block's
// row at the box's centre carries that rounding through the composition: its
// gap bound at cx = 1e6 exceeds the one at cx = 0, and both enclose 10 mm.
func TestVerifyJointBoxPoseDeviationIsCharged(t *testing.T) {
	t.Parallel()
	centreBound := func(t *testing.T, cx float64) float64 {
		t.Helper()
		doc := decad.New()
		arm := boxBody(t, doc, cx, -14, cx+48, 14, 10)
		block := boxBody(t, doc, cx+50, -4, cx+58, 4, 10)
		wall := boxBody(t, doc, cx-20, 68, cx+20, 78, 10)
		l := decad.NewLinkage()
		swing, err := l.Ground().Revolute(r3.NewVec(cx, 0, 0), zAxis, []*decad.Body{arm})
		require.NoError(t, err)
		ride, err := swing.Prismatic(r3.NewVec(1, 0, 0), []*decad.Body{block})
		require.NoError(t, err)
		report := verifyJointBox(t, doc, l, decad.JointBox{
			{Link: swing, Min: units.Degrees(80), Max: units.Degrees(100)},
			{Link: ride, Min: units.Millimeters(0), Max: units.Millimeters(0)},
		}, decad.WithCellBudget(1))
		require.Len(t, report.Cells, 1)
		require.Equal(t, units.Degrees(90), report.Cells[0].Center.Values[0])
		for _, row := range report.Cells[0].Clearances {
			if row.A != block || row.B != wall {
				continue
			}
			require.LessOrEqual(t, row.Gap.Value.Mag()-row.Gap.Bound.Mag(), 10.0)
			require.GreaterOrEqual(t, row.Gap.Value.Mag()+row.Gap.Bound.Mag(), 10.0)
			return row.Gap.Bound.Mag()
		}
		require.Fail(t, `the block's row against the wall is missing`)
		return 0
	}
	near := centreBound(t, 0)
	far := centreBound(t, 1e6)
	require.Greater(t, near, 0.0)
	require.Greater(t, far, near)
}

// TestVerifyJointBoxClearReading is §14.8's clear box: scene 6's crane with
// the wall moved to y ∈ [100, 120] and the mast cut to z ∈ [0, 20], so the
// layer exclusion's 20 mm between mast and boom is not the box's minimum.
// The tip's highest point over the box is its (80°, 30 mm) corner, so the
// minimum gap is 100 − 90·sin 80° − 5·cos 80° ≈ 10.499 mm, at the box's
// corner. At the defaults the whole-box reading
// refines past the verdict floor toward that corner, to the reading floor
// 1/16384 per axis, until it passes the tolerance gate; a 10 mm margin is
// proven and an 11 mm one disproven at a centre. A stated resolution is both
// floors, so at 1/64 the reading stays coarse and the report reads Suspect.
//
// Legs seen to fail when deleted: the reading's refinement (the defaults read
// Suspect, the reading beyond tolerance); the reading floor past the verdict
// floor (no leaf is narrower than 1/1024); a stated resolution setting the
// reading floor (a leaf narrower than 1/64); the margin's refinement (under a
// loose tolerance the 10 mm margin reads AssessmentUndecided).
func TestVerifyJointBoxClearReading(t *testing.T) {
	t.Parallel()
	truth := 100 - boomTipY(80, 30)
	narrowest := func(t *testing.T, report *decad.JointBoxReport) float64 {
		t.Helper()
		least := 1.0
		for _, cell := range report.Cells {
			cc := readCraneCell(t, cell.Cell)
			least = math.Min(least, (cc.thHi-cc.thLo)/80)
		}
		return least
	}
	t.Run("the defaults refine the reading to the gate", func(t *testing.T) {
		t.Parallel()
		c := buildCraneBoxWithMast(t, 100, 20)
		report := verifyJointBox(t, c.doc, c.linkage, c.box())
		require.Equal(t, decad.Sound, report.Status)
		require.Empty(t, report.Diagnostics)
		require.Equal(t, units.Scalar(1.0/1024), report.Request.Resolution)
		require.Equal(t, units.Scalar(1.0/16384), report.ReadingResolution)
		for _, cell := range report.Cells {
			require.Equal(t, decad.CellClear, cell.Outcome)
			cc := readCraneCell(t, cell.Cell)
			require.LessOrEqual(t, cell.Clearance.Value.Mag(), 100-boomTipY(cc.thHi, cc.dHi), `each bound is below the cell's true minimum`)
		}
		require.NotNil(t, report.Clearance)
		gap := report.Clearance.Measurement
		require.LessOrEqual(t, gap.Value.Mag()-gap.Bound.Mag(), truth)
		require.GreaterOrEqual(t, gap.Value.Mag()+gap.Bound.Mag(), truth)
		require.Equal(t, decad.ToleranceSatisfied, report.Clearance.Tolerance.State)
		require.Less(t, narrowest(t, report), 1.0/1024, `the reading refines past the verdict floor`)
		require.Less(t, report.CellsEvaluated, 1024)
	})
	t.Run("a 10 mm margin is met", func(t *testing.T) {
		t.Parallel()
		c := buildCraneBoxWithMast(t, 100, 20)
		report := verifyJointBox(t, c.doc, c.linkage, c.box(), decad.WithMinClearance(units.Millimeters(10)))
		require.Equal(t, decad.AssessmentMet, report.Assessment)
		require.Equal(t, decad.Sound, report.Status)
	})
	t.Run("a 10 mm margin refines past a loose reading", func(t *testing.T) {
		t.Parallel()
		// At a relative tolerance of 0.5 the reading passes its gate on coarse
		// cells, so only the margin's own refinement proves 10 mm.
		c := buildCraneBoxWithMast(t, 100, 20)
		report := verifyJointBox(t, c.doc, c.linkage, c.box(),
			decad.WithMinClearance(units.Millimeters(10)), decad.WithMotionTolerance(units.Scalar(0.5)))
		require.Equal(t, decad.AssessmentMet, report.Assessment)
		require.Equal(t, decad.Sound, report.Status)
	})
	t.Run("an 11 mm margin is violated", func(t *testing.T) {
		t.Parallel()
		c := buildCraneBoxWithMast(t, 100, 20)
		report := verifyJointBox(t, c.doc, c.linkage, c.box(), decad.WithMinClearance(units.Millimeters(11)))
		require.Equal(t, decad.AssessmentViolated, report.Assessment)
		require.Equal(t, decad.Violating, report.Status)
		violations := 0
		for _, d := range report.Diagnostics {
			if d.Code != decad.DiagMotionClearanceViolated {
				continue
			}
			violations++
			require.NotNil(t, d.Cell, `the violation names the cell whose centre proved it`)
			require.Less(t, d.Observed.Value.Mag()+d.Observed.Bound.Mag(), 11.0)
		}
		require.Positive(t, violations)
	})
	t.Run("a stated resolution is both floors", func(t *testing.T) {
		t.Parallel()
		c := buildCraneBoxWithMast(t, 100, 20)
		report := verifyJointBox(t, c.doc, c.linkage, c.box(), decad.WithResolution(units.Scalar(1.0/64)))
		require.Equal(t, units.Scalar(1.0/64), report.ReadingResolution)
		require.Equal(t, decad.Suspect, report.Status)
		require.NotNil(t, report.Clearance)
		require.NotEqual(t, decad.ToleranceSatisfied, report.Clearance.Tolerance.State)
		require.GreaterOrEqual(t, narrowest(t, report), 1.0/64)
	})
}
