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

// This file holds the PR 1 tests of docs/linkage-check-design.md §11: scene 1
// (the folding arm), the agreement of a one-link linkage with VerifyMotion,
// PoseAt's composition, and the standing tests. Bounds are asserted against
// geometric truths or as small, never pinned to a measured literal: FMA
// contraction moves the last ulp between amd64 and arm64. The internal tests
// of the chain bound sit in linkage_internal_test.go.
//
// Scene 1's upper arm is the motion arm, x ∈ [0, 48], y ∈ [−14, 14],
// z ∈ [0, 10], on a revolute joint about Z through the origin sweeping
// 0° → 90°. Its forearm, x ∈ [48, 96], y ∈ [−14, 14], z ∈ [12, 22], rides
// under it on a revolute joint about Z through (48, 0, 0) sweeping 0° → −90°.
// The two sweeps cancel in orientation: at s, with θ = 90°·s, the forearm
// keeps its zero-pose orientation and translates on the circle of radius 48
// about the origin, so its top face is the plane y = 48·sin θ + 14.

// foldingArm is scene 1's linkage, with the wall when asked for.
type foldingArm struct {
	doc             *decad.Document
	upper, forearm  *decad.Body
	wall            *decad.Body
	linkage         *decad.Linkage
	shoulder, elbow *decad.Link
}

func buildFoldingArm(t *testing.T, withWall bool) foldingArm {
	t.Helper()
	a := foldingArm{doc: decad.New()}
	a.upper = boxBody(t, a.doc, 0, -14, 48, 14, 10)
	a.forearm = boxBodyAtZ(t, a.doc, 48, -14, 96, 14, 12, 10)
	if withWall {
		// The wall reaches past every cap, so no pair shares a face plane.
		a.wall = boxBodyAtZ(t, a.doc, -100, 38, 150, 58, -10, 50)
	}
	a.linkage = decad.NewLinkage()
	var err error
	a.shoulder, err = a.linkage.Ground().Revolute(r3.Vec{}, r3.NewVec(0, 0, 1), []*decad.Body{a.upper})
	require.NoError(t, err)
	a.elbow, err = a.shoulder.Revolute(r3.NewVec(48, 0, 0), r3.NewVec(0, 0, 1), []*decad.Body{a.forearm})
	require.NoError(t, err)
	return a
}

func (a foldingArm) drive() decad.Drive {
	return decad.Drive{
		{Link: a.shoulder, From: units.Degrees(0), To: units.Degrees(90)},
		{Link: a.elbow, From: units.Degrees(0), To: units.Degrees(-90)},
	}
}

func verifyLinkage(t *testing.T, doc *decad.Document, l *decad.Linkage, d decad.Drive, opts ...decad.MotionOption) *decad.LinkageReport {
	t.Helper()
	before := doc.Bodies()
	report, err := doc.VerifyLinkage(t.Context(), l, d, opts...)
	require.NoError(t, err)
	require.Equal(t, before, doc.Bodies(), `VerifyLinkage must leave the live body set and its order unchanged`)
	return report
}

// requirePosesArePoseAt asserts every evaluated pose is the one Linkage.PoseAt
// builds at the same fraction: PoseAt is the one place a linkage pose is built.
func requirePosesArePoseAt(t *testing.T, report *decad.LinkageReport) {
	t.Helper()
	for _, p := range report.Poses {
		want, err := report.Linkage.PoseAt(report.Drive, p.Pose.At)
		require.NoError(t, err)
		require.Equal(t, want, p.Pose)
	}
}

// TestVerifyLinkageFoldingArm is scene 1 of docs/linkage-check-design.md §11.
//
// The forearm's top face y = 48·sin θ + 14 reaches the wall's face y = 38 at
// sin θ = 1/2: θ* = 30°, s* = 1/3. The upper arm's far corner (48, 14), at
// radius 50, reaches it at θ = asin(38/50) − atan(7/24) ≈ 33.2°, later. At
// WithResolution(1/256) the onset bisection brackets the first collision to
// the grid point 86/256, where the forearm overlaps the wall in the slab
// 48 × (48·sin θ − 24) × 10 mm³. The arms never meet: their relative motion
// is the elbow joint alone, about Z, so their z-extents [0, 10] and [12, 22]
// hold at every s and the layer exclusion (§5.7) settles the pair with the
// lower bound 2 mm, evaluating it at no pose.
//
// A post rigidly on the upper arm, x ∈ [20, 30], y ∈ [26, 36], z ∈ [0, 30],
// shares the forearm's layer, so that pair is evaluated; its relative motion
// is again the elbow alone, and its gap, 21-31 mm, certifies at Δs = 1/4.
//
// Legs seen to fail when deleted: the layer exclusion (the arms are evaluated
// and the wall-free drive refines to the floor and reads Suspect); and
// summing a link-link pair's τ over the joints strictly below the two links'
// lowest common ancestor only — leaving the shoulder in the post's pair at
// WithResolution(1/4) leaves every interval undecided.
func TestVerifyLinkageFoldingArm(t *testing.T) {
	t.Parallel()
	requireArmsUnevaluated := func(t *testing.T, a foldingArm, report *decad.LinkageReport) {
		t.Helper()
		for _, p := range report.Poses {
			for _, row := range p.Clearances {
				require.False(t, row.A == a.upper && row.B == a.forearm, `the layer exclusion settles the arms`)
			}
			for _, row := range p.Interferences {
				require.NotSame(t, a.forearm, row.B, `the arms never overlap`)
			}
		}
	}
	t.Run("the forearm reaches the wall at one third of the drive", func(t *testing.T) {
		t.Parallel()
		a := buildFoldingArm(t, true)
		report := verifyLinkage(t, a.doc, a.linkage, a.drive(), decad.WithResolution(units.Scalar(1.0/256)))
		sStar := 1.0 / 3
		sUpper := (math.Asin(38.0/50) - math.Atan(7.0/24)) / (math.Pi / 2)

		require.Equal(t, decad.Interfering, report.Status)
		require.Same(t, a.linkage, report.Linkage)
		require.Equal(t, a.drive(), report.Drive)
		require.Equal(t, []*decad.Link{a.shoulder, a.elbow}, report.Links)
		require.Equal(t, []*decad.Body{a.wall}, report.Against)
		require.Equal(t, units.Scalar(1.0/256), report.Request.Resolution)
		requirePosesArePoseAt(t, report)
		requireArmsUnevaluated(t, a, report)

		require.NotEmpty(t, report.Collisions)
		first := report.Collisions[0]
		require.Same(t, a.forearm, first.A)
		require.Same(t, a.wall, first.B)
		require.Equal(t, units.Scalar(86.0/256), first.At, `the first grid point above s* = 1/3`)
		theta := 90.0 * 86 / 256 * math.Pi / 180
		require.InDelta(t, 480*(48*math.Sin(theta)-24), first.Volume.Value.Mag(), 1e-3)
		require.Less(t, first.Volume.Bound.Mag(), first.Volume.Value.Mag())
		for _, c := range report.Collisions {
			require.Greater(t, c.At.Mag(), sStar)
			require.Same(t, a.wall, c.B, `the arms never meet`)
			if c.A == a.upper {
				require.Greater(t, c.At.Mag(), sUpper)
			}
		}
		for _, iv := range report.Intervals {
			if iv.Outcome == decad.IntervalClear {
				require.LessOrEqual(t, iv.To.Mag(), sStar)
			}
			if iv.From.Mag() <= sStar && sStar <= iv.To.Mag() {
				require.NotEqual(t, decad.IntervalClear, iv.Outcome, `the interval holding s* is never clear`)
			}
		}
		for _, p := range report.Poses {
			require.Len(t, p.Pose.Values, 2)
			require.InDelta(t, 90*p.Pose.At.Mag(), p.Pose.Values[0].Mag(), 1e-12)
			require.InDelta(t, -90*p.Pose.At.Mag(), p.Pose.Values[1].Mag(), 1e-12)
		}
	})
	t.Run("without the wall the drive is clear at the endpoints", func(t *testing.T) {
		t.Parallel()
		a := buildFoldingArm(t, false)
		report := verifyLinkage(t, a.doc, a.linkage, a.drive())
		require.Equal(t, decad.Sound, report.Status)
		require.True(t, report.Passed())
		require.Empty(t, report.Diagnostics)
		require.Empty(t, report.Against)
		require.Empty(t, report.Collisions)
		require.Equal(t, units.Scalar(1.0/1024), report.Request.Resolution)
		require.Len(t, report.Poses, 2, `the only pair is settled before any pose`)
		requireArmsUnevaluated(t, a, report)
		require.Len(t, report.Intervals, 1)
		iv := report.Intervals[0]
		require.Equal(t, decad.IntervalClear, iv.Outcome)
		require.NotNil(t, iv.Clearance)
		require.Equal(t, 2.0, iv.Clearance.Value.Mag(), `the layer gap is exact: 12 − 10 along Z`)
		require.Nil(t, report.Clearance, `a settled pair measures no upper bound`)
	})
	t.Run("a 1 mm margin between the arms is met", func(t *testing.T) {
		t.Parallel()
		a := buildFoldingArm(t, false)
		minimum := units.Millimeters(1)
		report := verifyLinkage(t, a.doc, a.linkage, a.drive(), decad.WithMinClearance(minimum))
		require.Equal(t, decad.AssessmentMet, report.Assessment)
		require.Equal(t, decad.Sound, report.Status)
		require.Equal(t, &minimum, report.Request.MinClearance)
	})
	t.Run("a 3 mm margin between the arms is undecided", func(t *testing.T) {
		t.Parallel()
		// The layer's proven 2 mm lower bound cannot meet 3 mm, and no pose
		// measures the arms to disprove it.
		a := buildFoldingArm(t, false)
		report := verifyLinkage(t, a.doc, a.linkage, a.drive(), decad.WithMinClearance(units.Millimeters(3)))
		require.Equal(t, decad.AssessmentUndecided, report.Assessment)
		require.Equal(t, decad.Suspect, report.Status)
		require.NotEmpty(t, report.Diagnostics)
		for _, d := range report.Diagnostics {
			require.Equal(t, decad.DiagMotionUndecidedClearance, d.Code)
		}
	})
	t.Run("a post in the forearm's layer certifies through the elbow alone", func(t *testing.T) {
		t.Parallel()
		doc := decad.New()
		upper := boxBody(t, doc, 0, -14, 48, 14, 10)
		post := boxBody(t, doc, 20, 26, 30, 36, 30)
		forearm := boxBodyAtZ(t, doc, 48, -14, 96, 14, 12, 10)
		l := decad.NewLinkage()
		shoulder, err := l.Ground().Revolute(r3.Vec{}, zAxis, []*decad.Body{upper, post})
		require.NoError(t, err)
		elbow, err := shoulder.Revolute(r3.NewVec(48, 0, 0), zAxis, []*decad.Body{forearm})
		require.NoError(t, err)
		report := verifyLinkage(t, doc, l, decad.Drive{
			{Link: shoulder, From: units.Degrees(0), To: units.Degrees(90)},
			{Link: elbow, From: units.Degrees(0), To: units.Degrees(-90)},
		}, decad.WithResolution(units.Scalar(1.0/4)))
		rows := 0
		for _, p := range report.Poses {
			for _, row := range p.Clearances {
				if row.A == post && row.B == forearm {
					rows++
				}
			}
		}
		require.Equal(t, len(report.Poses), rows, `the post's pair is evaluated at every pose`)
		for _, iv := range report.Intervals {
			require.Equal(t, decad.IntervalClear, iv.Outcome)
		}
	})
}

// TestLinkagePoseAt pins the composition Pose_k = J_k(q_k).Then(Pose_parent)
// on three joints: scene 1's arms and a slider on the forearm, a prismatic
// joint along +X sweeping 0 → 30 mm. At s = 1/3 the shoulder stands at 30°,
// the elbow at −30° and the slider at 10 mm. The forearm keeps its zero-pose
// orientation and rides the elbow's circle, so its far end (96, 0, 0) lands
// at (48·cos 30° + 48, 48·sin 30°, 0) and the elbow centre (48, 0, 0) at
// (48·cos 30°, 48·sin 30°, 0); the slider's point (100, 0, 0) slides to
// (110, 0, 0) and then rides the forearm's pose. Composing the parent's pose
// first, then the joint about its zero-pose axis, lands (96, 0, 0) elsewhere.
func TestLinkagePoseAt(t *testing.T) {
	t.Parallel()
	a := buildFoldingArm(t, false)
	pin := boxBody(t, a.doc, 100, -2, 104, 2, 4)
	slider, err := a.elbow.Prismatic(r3.NewVec(2, 0, 0), []*decad.Body{pin})
	require.NoError(t, err)
	drive := append(a.drive(), decad.JointSweep{Link: slider, From: units.Millimeters(0), To: units.Millimeters(30)})
	require.Equal(t, []*decad.Link{a.shoulder, a.elbow, slider}, a.linkage.Links())
	require.Same(t, a.elbow, slider.Parent())
	require.Equal(t, decad.PrismaticJoint{Dir: r3.NewVec(2, 0, 0)}, slider.Joint())
	require.Equal(t, decad.RevoluteJoint{Center: r3.NewVec(48, 0, 0), Axis: r3.NewVec(0, 0, 1)}, a.elbow.Joint())
	require.Equal(t, []*decad.Body{pin}, slider.Bodies())
	require.Nil(t, a.linkage.Ground().Parent())
	require.Nil(t, a.linkage.Ground().Joint())
	require.Empty(t, a.linkage.Ground().Bodies())

	pose, err := a.linkage.PoseAt(drive, units.Scalar(1.0/3))
	require.NoError(t, err)
	require.Equal(t, units.Scalar(1.0/3), pose.At)
	require.Len(t, pose.Values, 3)
	require.InDelta(t, 30, pose.Values[0].Mag(), 1e-12)
	require.Equal(t, units.Degree, pose.Values[0].Unit())
	require.InDelta(t, -30, pose.Values[1].Mag(), 1e-12)
	require.InDelta(t, 10, pose.Values[2].Mag(), 1e-12)
	require.Equal(t, units.Millimeter, pose.Values[2].Unit())
	c, s := math.Cos(math.Pi/6), math.Sin(math.Pi/6)
	requireNear := func(t *testing.T, want, got r3.Vec) {
		t.Helper()
		require.InDelta(t, want.X, got.X, 1e-12)
		require.InDelta(t, want.Y, got.Y, 1e-12)
		require.InDelta(t, want.Z, got.Z, 1e-12)
	}
	requireNear(t, r3.NewVec(48*c, 48*s, 0), pose.Poses[0].Apply(r3.NewVec(48, 0, 0)))
	requireNear(t, r3.NewVec(48*c, 48*s, 0), pose.Poses[1].Apply(r3.NewVec(48, 0, 0)))
	requireNear(t, r3.NewVec(48*c+48, 48*s, 0), pose.Poses[1].Apply(r3.NewVec(96, 0, 0)))
	requireNear(t, r3.NewVec(48*c+62, 48*s, 0), pose.Poses[2].Apply(r3.NewVec(100, 0, 0)))

	t.Run("the ends are the stated values and an unlisted joint holds 0", func(t *testing.T) {
		t.Parallel()
		end, err := a.linkage.PoseAt(a.drive(), units.Scalar(1))
		require.NoError(t, err)
		require.Equal(t, units.Degrees(90), end.Values[0])
		require.Equal(t, units.Degrees(-90), end.Values[1])
		require.Zero(t, end.Values[2].Mag())
		require.Equal(t, units.Length, end.Values[2].Kind())
		requireNear(t, r3.NewVec(52, 48, 0), end.Poses[2].Apply(r3.NewVec(100, 0, 0)))
		beyond, err := a.linkage.PoseAt(a.drive(), units.Scalar(2))
		require.NoError(t, err, `PoseAt takes no range`)
		require.InDelta(t, 180, beyond.Values[0].Mag(), 1e-12)
	})

	other := buildFoldingArm(t, false)
	refusals := []struct {
		name  string
		l     *decad.Linkage
		drive decad.Drive
		at    units.Value
		want  error
	}{
		{"nil linkage", nil, nil, units.Scalar(0), decad.ErrDegenerate},
		{"a sweep naming no link", a.linkage, decad.Drive{{From: units.Degrees(0), To: units.Degrees(1)}}, units.Scalar(0), decad.ErrDegenerate},
		{"a sweep naming the ground", a.linkage, decad.Drive{{Link: a.linkage.Ground(), From: units.Degrees(0), To: units.Degrees(1)}}, units.Scalar(0), decad.ErrDegenerate},
		{"a link of another linkage", a.linkage, decad.Drive{{Link: other.shoulder, From: units.Degrees(0), To: units.Degrees(1)}}, units.Scalar(0), decad.ErrDegenerate},
		{"a link named twice", a.linkage, append(a.drive(), a.drive()[0]), units.Scalar(0), decad.ErrDegenerate},
		{"a length on a revolute", a.linkage, decad.Drive{{Link: a.shoulder, From: units.Millimeters(0), To: units.Degrees(1)}}, units.Scalar(0), decad.ErrUnitKind},
		{"an angle on a prismatic", a.linkage, decad.Drive{{Link: slider, From: units.Millimeters(0), To: units.Degrees(1)}}, units.Scalar(0), decad.ErrUnitKind},
		{"a non-finite sweep", a.linkage, decad.Drive{{Link: a.shoulder, From: units.Degrees(0), To: units.Degrees(math.NaN())}}, units.Scalar(0), decad.ErrNotFinite},
		{"an angle as the fraction", a.linkage, a.drive(), units.Degrees(1), decad.ErrUnitKind},
		{"a non-finite fraction", a.linkage, a.drive(), units.Scalar(math.Inf(1)), decad.ErrNotFinite},
		{"sweeps with different numbers of Via values", a.linkage, decad.Drive{
			{Link: a.shoulder, From: units.Degrees(0), Via: []units.Value{units.Degrees(45)}, To: units.Degrees(90)},
			{Link: a.elbow, From: units.Degrees(0), To: units.Degrees(-90)},
		}, units.Scalar(0), decad.ErrDegenerate},
		{"a length as a Via value on a revolute", a.linkage, decad.Drive{{Link: a.shoulder, From: units.Degrees(0), Via: []units.Value{units.Millimeters(1)}, To: units.Degrees(1)}}, units.Scalar(0), decad.ErrUnitKind},
		{"a non-finite Via value", a.linkage, decad.Drive{{Link: a.shoulder, From: units.Degrees(0), Via: []units.Value{units.Degrees(math.NaN())}, To: units.Degrees(1)}}, units.Scalar(0), decad.ErrNotFinite},
	}
	for _, tc := range refusals {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			_, err := tc.l.PoseAt(tc.drive, tc.at)
			require.ErrorIs(t, err, tc.want)
		})
	}
}

// TestLinkageConstructorRefusals is one subtest per refusal of
// docs/linkage-check-design.md §2.1, in the order the constructors check them.
func TestLinkageConstructorRefusals(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	body := boxBody(t, doc, 0, 0, 10, 10, 10)
	free := boxBody(t, doc, 20, 0, 30, 10, 10)
	l := decad.NewLinkage()
	_, err := l.Ground().Prismatic(r3.NewVec(1, 0, 0), []*decad.Body{body})
	require.NoError(t, err)
	z := r3.NewVec(0, 0, 1)
	var nilLink *decad.Link
	cases := []struct {
		name string
		make func() (*decad.Link, error)
		want error
	}{
		{"nil parent", func() (*decad.Link, error) { return nilLink.Revolute(r3.Vec{}, z, []*decad.Body{free}) }, decad.ErrDegenerate},
		{"a link no linkage built", func() (*decad.Link, error) { return (&decad.Link{}).Prismatic(z, []*decad.Body{free}) }, decad.ErrDegenerate},
		{"no body", func() (*decad.Link, error) { return l.Ground().Revolute(r3.Vec{}, z, nil) }, decad.ErrDegenerate},
		{"a nil body", func() (*decad.Link, error) { return l.Ground().Revolute(r3.Vec{}, z, []*decad.Body{nil}) }, decad.ErrDegenerate},
		{"a body listed twice", func() (*decad.Link, error) { return l.Ground().Revolute(r3.Vec{}, z, []*decad.Body{free, free}) }, decad.ErrDegenerate},
		{"a body already in a link", func() (*decad.Link, error) { return l.Ground().Revolute(r3.Vec{}, z, []*decad.Body{body}) }, decad.ErrDegenerate},
		{"a non-finite center", func() (*decad.Link, error) {
			return l.Ground().Revolute(r3.NewVec(math.NaN(), 0, 0), z, []*decad.Body{free})
		}, decad.ErrNotFinite},
		{"a non-finite axis", func() (*decad.Link, error) {
			return l.Ground().Revolute(r3.Vec{}, r3.NewVec(0, 0, math.Inf(1)), []*decad.Body{free})
		}, decad.ErrNotFinite},
		{"a non-finite direction", func() (*decad.Link, error) {
			return l.Ground().Prismatic(r3.NewVec(math.Inf(-1), 0, 0), []*decad.Body{free})
		}, decad.ErrNotFinite},
		{"a zero axis", func() (*decad.Link, error) { return l.Ground().Revolute(r3.Vec{}, r3.Vec{}, []*decad.Body{free}) }, decad.ErrDegenerate},
		{"a zero direction", func() (*decad.Link, error) { return l.Ground().Prismatic(r3.Vec{}, []*decad.Body{free}) }, decad.ErrDegenerate},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			link, err := tc.make()
			require.ErrorIs(t, err, tc.want)
			require.Nil(t, link)
			require.Len(t, l.Links(), 1, `a refused link is never attached`)
		})
	}
}

// TestVerifyLinkageAgreesWithVerifyMotion is §11's agreement test: a one-link
// linkage on a revolute joint and the same body under the equivalent
// Revolute, on motion §9's fixtures 1 (a wall the arm hits), 2 (a wall it
// clears by 10 mm) and 4 (a stop it rests on). The one-link τ is
// ρ_{11}·|Δq_1|, which is MoverTravel's value, and the ideal pose is one
// MotionFrame.At, so at the endpoints alone, where no bound certifies the one
// interval, the two reports agree in every reading. Bisected, both take the
// larger of the travel bound and the projection bound
// (docs/linkage-check-design.md §5.8, docs/motion-check-design.md §5.2) over
// the same corner and hull readings, so they agree there too: the same
// collisions and Status, the same poses, the same interval outcomes, each
// interval's Clearance within 1e-9 — ρ_11 and ρ_max are read by two routes
// and may part in the last ulp — both readings enclosing 10 mm on fixture 2,
// and every IntervalClear interval's Clearance at or below the arm's
// closed-form gap at its ends — its corner (48, 14) at y = 48·sin θ + 14·cos θ
// below the wall's face, or its corner (0, −14) 14·(1 − cos θ) above the
// stop's.
//
// Leg seen to fail when deleted: the link's own revolute term ρ_{kk}·|Δq_k|
// in τ (fixture 2's swing certifies at the endpoints, so the interval
// outcomes part from VerifyMotion's).
func TestVerifyLinkageAgreesWithVerifyMotion(t *testing.T) {
	t.Parallel()
	corner := func(face float64) func(th float64) float64 {
		return func(th float64) float64 { return face - 48*math.Sin(th) - 14*math.Cos(th) }
	}
	fixtures := []struct {
		name   string
		static func(t *testing.T, doc *decad.Document)
		gap    func(th float64) float64
		// the floor each check is bisected to: VerifyMotion's default of
		// 90°/1024 for fixtures 2 and 4, for one floor serves its verdict
		// and reading alike, which the linkage states as 1/1024 so its
		// reading stops there too (§3)
		motionRes  []decad.MotionOption
		linkageRes decad.MotionOption
	}{
		{"a wall the arm hits", func(t *testing.T, doc *decad.Document) { boxBodyAtZ(t, doc, -100, 40, 100, 60, -10, 40) },
			corner(40), []decad.MotionOption{decad.WithResolution(units.Degrees(0.25))}, decad.WithResolution(units.Scalar(0.25 / 90))},
		{"a wall the arm clears", func(t *testing.T, doc *decad.Document) { boxBody(t, doc, -100, 60, 100, 80, 10) },
			corner(60), nil, decad.WithResolution(units.Scalar(1.0 / 1024))},
		{"a stop the arm rests on", func(t *testing.T, doc *decad.Document) { boxBody(t, doc, 0, -24, 48, -14, 10) },
			func(th float64) float64 { return 14 * (1 - math.Cos(th)) },
			nil, decad.WithResolution(units.Scalar(1.0 / 1024))},
	}
	both := func(t *testing.T, static func(t *testing.T, doc *decad.Document), motionRes []decad.MotionOption, linkageRes decad.MotionOption) (*decad.MotionReport, *decad.LinkageReport) {
		t.Helper()
		doc := decad.New()
		arm := motionArm(t, doc)
		static(t, doc)
		motion := verifyMotion(t, doc, []*decad.Body{arm}, armSwing(), motionRes...)
		l := decad.NewLinkage()
		link, err := l.Ground().Revolute(r3.Vec{}, r3.NewVec(0, 0, 1), []*decad.Body{arm})
		require.NoError(t, err)
		drive := decad.Drive{{Link: link, From: units.Degrees(0), To: units.Degrees(90)}}
		return motion, verifyLinkage(t, doc, l, drive, linkageRes)
	}
	requireSameCollisions := func(t *testing.T, motion *decad.MotionReport, linkage *decad.LinkageReport) {
		t.Helper()
		require.Len(t, linkage.Collisions, len(motion.Collisions))
		for k, c := range motion.Collisions {
			got := linkage.Collisions[k]
			require.InDelta(t, c.At.Mag(), 90*got.At.Mag(), 1e-9)
			require.Same(t, c.Moving, got.A)
			require.Same(t, c.Static, got.B)
			require.InDelta(t, c.Volume.Value.Mag(), got.Volume.Value.Mag(), 1e-9)
		}
	}
	for _, tc := range fixtures {
		t.Run(tc.name+" at the endpoints alone", func(t *testing.T) {
			t.Parallel()
			motion, linkage := both(t, tc.static, []decad.MotionOption{endpointsOnly(armSwing())}, decad.WithResolution(units.Scalar(1)))
			require.Equal(t, motion.Status, linkage.Status)
			require.Equal(t, motion.Against, linkage.Against)
			require.Len(t, linkage.Poses, len(motion.Poses))
			require.Len(t, linkage.Intervals, len(motion.Intervals))
			for k, iv := range motion.Intervals {
				got := linkage.Intervals[k]
				require.Equal(t, iv.Outcome, got.Outcome)
				require.InDelta(t, iv.From.Mag(), 90*got.From.Mag(), 1e-9)
				if iv.Clearance == nil {
					require.Nil(t, got.Clearance)
					continue
				}
				require.NotNil(t, got.Clearance)
				require.InDelta(t, iv.Clearance.Value.Mag(), got.Clearance.Value.Mag(), 1e-9)
			}
			requireSameCollisions(t, motion, linkage)
			for k, pose := range motion.Poses {
				linkPose := linkage.Poses[k]
				require.Len(t, linkPose.Clearances, len(pose.Clearances))
				for n, row := range pose.Clearances {
					require.Equal(t, row.Gap, linkPose.Clearances[n].Gap, `both paths place the arm by the same pose`)
				}
			}
		})
		t.Run(tc.name+" bisected", func(t *testing.T) {
			t.Parallel()
			motion, linkage := both(t, tc.static, tc.motionRes, tc.linkageRes)
			requireSameCollisions(t, motion, linkage)
			require.Equal(t, motion.Status, linkage.Status)
			require.Len(t, linkage.Poses, len(motion.Poses))
			require.Len(t, linkage.Intervals, len(motion.Intervals))
			for k, iv := range motion.Intervals {
				got := linkage.Intervals[k]
				require.Equal(t, iv.Outcome, got.Outcome)
				require.InDelta(t, iv.From.Mag(), 90*got.From.Mag(), 1e-9)
				if iv.Clearance == nil {
					require.Nil(t, got.Clearance)
					continue
				}
				require.NotNil(t, got.Clearance)
				require.InDelta(t, iv.Clearance.Value.Mag(), got.Clearance.Value.Mag(), 1e-9)
			}
			for _, iv := range linkage.Intervals {
				if iv.Outcome != decad.IntervalClear {
					continue
				}
				for _, s := range []float64{iv.From.Mag(), iv.To.Mag()} {
					require.LessOrEqual(t, iv.Clearance.Value.Mag(), tc.gap(s*math.Pi/2)+1e-9, "[%v, %v]", iv.From.Mag(), iv.To.Mag())
				}
			}
			if motion.Clearance == nil {
				require.Nil(t, linkage.Clearance)
				return
			}
			// Fixture 2: both projection bounds meet the gate at the floor.
			require.Equal(t, decad.Sound, motion.Status)
			for _, reading := range []*decad.ScalarReading{motion.Clearance, linkage.Clearance} {
				require.NotNil(t, reading)
				require.InDelta(t, 10, reading.Value.Mag(), 0.1)
				require.LessOrEqual(t, reading.Value.Mag()-reading.Bound.Mag(), 10.0)
				require.GreaterOrEqual(t, reading.Value.Mag()+reading.Bound.Mag(), 10.0)
			}
		})
	}
}

// TestVerifyLinkagePoseDeviationIsCharged: an arm swings 90° about a pivot
// (cx, 0, 0), and a block rides past its tip on a prismatic joint the drive
// leaves at 0, so the block's pose is its joint's identity composed onto the
// arm's. A wall stands 10 mm beyond the block at the end of the swing. The
// far pivot's pose rounds at its magnitude, and the block's row carries that
// rounding through the composition: its gap bound at cx = 1e6 exceeds the
// one at cx = 0, and both stay inside the tolerance gate.
func TestVerifyLinkagePoseDeviationIsCharged(t *testing.T) {
	t.Parallel()
	endBound := func(t *testing.T, cx float64) float64 {
		t.Helper()
		doc := decad.New()
		arm := boxBody(t, doc, cx, -14, cx+48, 14, 10)
		block := boxBody(t, doc, cx+50, -4, cx+58, 4, 10)
		// At 90° about (cx, 0, 0) the block spans y ∈ [50, 58]: the wall
		// stands 10 mm beyond it.
		wall := boxBody(t, doc, cx-20, 68, cx+20, 78, 10)
		l := decad.NewLinkage()
		swing, err := l.Ground().Revolute(r3.NewVec(cx, 0, 0), r3.NewVec(0, 0, 1), []*decad.Body{arm})
		require.NoError(t, err)
		_, err = swing.Prismatic(r3.NewVec(1, 0, 0), []*decad.Body{block})
		require.NoError(t, err)
		report := verifyLinkage(t, doc, l, decad.Drive{{Link: swing, From: units.Degrees(0), To: units.Degrees(90)}},
			decad.WithResolution(units.Scalar(1)))
		end := report.Poses[len(report.Poses)-1]
		for _, row := range end.Clearances {
			if row.A != block || row.B != wall {
				continue
			}
			require.LessOrEqual(t, row.Gap.Value.Mag()-row.Gap.Bound.Mag(), 10.0)
			require.GreaterOrEqual(t, row.Gap.Value.Mag()+row.Gap.Bound.Mag(), 10.0)
			require.Less(t, row.Gap.Bound.Mag(), 1e-3*10, `the bound passes the default gate at a 10 mm gap`)
			return row.Gap.Bound.Mag()
		}
		require.Fail(t, `the block's row against the wall is missing`)
		return 0
	}
	near := endBound(t, 0)
	far := endBound(t, 1e6)
	require.Greater(t, near, 0.0)
	require.Greater(t, far, near)
}

// TestVerifyLinkageErrors is one subtest per row of
// docs/linkage-check-design.md §8's table that PR 1's public API can reach,
// each asserting the sentinel, no report, and an unchanged document. The row
// for a body this evaluator did not build lives in linkage_internal_test.go.
func TestVerifyLinkageErrors(t *testing.T) {
	t.Parallel()
	a := buildFoldingArm(t, true)
	doc := a.doc
	retired := boxBody(t, doc, 200, 200, 210, 210, 10)
	translated(t, retired, 0, 0, 100)
	foreign := boxBody(t, decad.New(), 0, 0, 10, 10, 10)
	withBody := func(b *decad.Body) *decad.Linkage {
		l := decad.NewLinkage()
		_, err := l.Ground().Prismatic(r3.NewVec(1, 0, 0), []*decad.Body{b})
		require.NoError(t, err)
		return l
	}
	slide := func(l *decad.Linkage) decad.Drive {
		return decad.Drive{{Link: l.Links()[0], From: units.Millimeters(0), To: units.Millimeters(1)}}
	}
	retiredLinkage, foreignLinkage := withBody(retired), withBody(foreign)
	far := decad.NewLinkage()
	farLink, err := far.Ground().Revolute(r3.NewVec(math.MaxFloat64, math.MaxFloat64, 0), r3.NewVec(0, 0, 1), []*decad.Body{a.upper})
	require.NoError(t, err)
	other := buildFoldingArm(t, false)
	canceled, cancel := context.WithCancel(t.Context())
	cancel()
	held := decad.Drive{{Link: a.shoulder, From: units.Degrees(30), To: units.Degrees(30)}}
	contactWith := func(b *decad.Body) *decad.Linkage {
		l := decad.NewLinkage()
		_, err := l.Ground().Revolute(r3.Vec{}, zAxis, []*decad.Body{a.upper})
		require.NoError(t, err)
		require.NoError(t, l.DeclareJointContact(a.upper, b))
		return l
	}
	retiredContact, foreignContact := contactWith(retired), contactWith(foreign)
	opt := func(o decad.MotionOption) []decad.MotionOption { return []decad.MotionOption{o} }

	cases := []struct {
		name  string
		ctx   context.Context //nolint:containedctx // a table of per-case contexts.
		l     *decad.Linkage
		drive decad.Drive
		opts  []decad.MotionOption
		want  error
	}{
		{name: "nil linkage", drive: a.drive(), want: decad.ErrDegenerate},
		{name: "a linkage with no link", l: decad.NewLinkage(), want: decad.ErrDegenerate},
		{name: "an empty drive", l: a.linkage, want: decad.ErrDegenerate},
		{name: "a drive whose every sweep holds", l: a.linkage, drive: held, want: decad.ErrDegenerate},
		{name: "a drive whose sweep holds across units", l: a.linkage, drive: decad.Drive{{Link: a.shoulder, From: units.Degrees(0), To: units.Radians(0)}}, want: decad.ErrDegenerate},
		{name: "a link named twice", l: a.linkage, drive: append(a.drive(), a.drive()[1]), want: decad.ErrDegenerate},
		{name: "a link of another linkage", l: a.linkage, drive: decad.Drive{{Link: other.elbow, From: units.Degrees(0), To: units.Degrees(1)}}, want: decad.ErrDegenerate},
		{name: "a retired link body", l: retiredLinkage, drive: slide(retiredLinkage), want: decad.ErrRetiredBody},
		{name: "a foreign link body", l: foreignLinkage, drive: slide(foreignLinkage), want: decad.ErrForeignBody},
		{name: "a retired declared contact", l: retiredContact, drive: decad.Drive{{Link: retiredContact.Links()[0], From: units.Degrees(0), To: units.Degrees(1)}}, want: decad.ErrRetiredBody},
		{name: "a foreign declared contact", l: foreignContact, drive: decad.Drive{{Link: foreignContact.Links()[0], From: units.Degrees(0), To: units.Degrees(1)}}, want: decad.ErrForeignBody},
		{name: "a wrong-kind sweep", l: a.linkage, drive: decad.Drive{{Link: a.shoulder, From: units.Millimeters(0), To: units.Millimeters(1)}}, want: decad.ErrUnitKind},
		{name: "a wrong-kind resolution", l: a.linkage, drive: a.drive(), opts: opt(decad.WithResolution(units.Degrees(1))), want: decad.ErrUnitKind},
		{name: "a wrong-kind tolerance", l: a.linkage, drive: a.drive(), opts: opt(decad.WithMotionTolerance(units.Millimeters(1))), want: decad.ErrUnitKind},
		{name: "a wrong-kind minimum", l: a.linkage, drive: a.drive(), opts: opt(decad.WithMinClearance(units.Degrees(1))), want: decad.ErrUnitKind},
		{name: "a non-finite sweep", l: a.linkage, drive: decad.Drive{{Link: a.shoulder, From: units.Degrees(math.Inf(-1)), To: units.Degrees(1)}}, want: decad.ErrNotFinite},
		{name: "a non-finite resolution", l: a.linkage, drive: a.drive(), opts: opt(decad.WithResolution(units.Scalar(math.NaN()))), want: decad.ErrNotFinite},
		{name: "a pose r3 cannot represent", l: far, drive: decad.Drive{{Link: farLink, From: units.Degrees(0), To: units.Degrees(90)}}, want: decad.ErrNotFinite},
		{name: "a negative resolution", l: a.linkage, drive: a.drive(), opts: opt(decad.WithResolution(units.Scalar(-1))), want: decad.ErrNegativeMagnitude},
		{name: "a zero resolution", l: a.linkage, drive: a.drive(), opts: opt(decad.WithResolution(units.Scalar(0))), want: decad.ErrNegativeMagnitude},
		{name: "a negative tolerance", l: a.linkage, drive: a.drive(), opts: opt(decad.WithMotionTolerance(units.Scalar(-1))), want: decad.ErrNegativeMagnitude},
		{name: "a negative minimum", l: a.linkage, drive: a.drive(), opts: opt(decad.WithMinClearance(units.Millimeters(-1))), want: decad.ErrNegativeMagnitude},
		{name: "a zero minimum", l: a.linkage, drive: a.drive(), opts: opt(decad.WithMinClearance(units.Millimeters(0))), want: decad.ErrDegenerate},
		{name: "validation wins over a canceled context", ctx: canceled, l: a.linkage, drive: held, want: decad.ErrDegenerate},
		{name: "canceled after validation", ctx: canceled, l: a.linkage, drive: a.drive(), want: context.Canceled},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			ctx := tc.ctx
			if ctx == nil {
				ctx = t.Context()
			}
			before := doc.Bodies()
			report, err := doc.VerifyLinkage(ctx, tc.l, tc.drive, tc.opts...)
			require.ErrorIs(t, err, tc.want)
			require.Nil(t, report)
			require.Equal(t, before, doc.Bodies())
		})
	}
	t.Run("nil document", func(t *testing.T) {
		var nilDoc *decad.Document
		report, err := nilDoc.VerifyLinkage(t.Context(), a.linkage, a.drive())
		require.ErrorIs(t, err, decad.ErrDegenerate)
		require.Nil(t, report)
	})
}

// TestVerifyLinkageNonMutationAndDeterminism is §11's standing test, as
// motion §9 test 7: the live body set and its order survive the call, and two
// calls on the same inputs return reports equal in every field. The producer
// identity half lives in linkage_internal_test.go, which can read it.
func TestVerifyLinkageNonMutationAndDeterminism(t *testing.T) {
	t.Parallel()
	a := buildFoldingArm(t, true)
	before := a.doc.Bodies()
	first := verifyLinkage(t, a.doc, a.linkage, a.drive(), decad.WithResolution(units.Scalar(1.0/64)))
	second := verifyLinkage(t, a.doc, a.linkage, a.drive(), decad.WithResolution(units.Scalar(1.0/64)))
	require.Equal(t, before, a.doc.Bodies())
	require.Equal(t, first, second)
}

// TestVerifyLinkageCancellation: a context canceled at successively later
// checks after validation — through the endpoints and on through the
// bisection of scene 1 — returns ctx.Err() and no report, and leaves the
// document unchanged.
func TestVerifyLinkageCancellation(t *testing.T) {
	t.Parallel()
	a := buildFoldingArm(t, true)
	before := a.doc.Bodies()
	var completed *decad.LinkageReport
	canceled := 0
	for limit := int32(1); limit <= 1<<24; limit += 1 + limit/2 {
		ctx := newCancelAfterContext(t.Context(), limit)
		report, err := a.doc.VerifyLinkage(ctx, a.linkage, a.drive(), decad.WithResolution(units.Scalar(1.0/256)))
		require.Equal(t, before, a.doc.Bodies())
		if err == nil {
			completed = report
			break
		}
		require.ErrorIs(t, err, context.Canceled)
		require.Nil(t, report)
		canceled++
	}
	require.NotNil(t, completed, `the check finishes once the context outlasts it`)
	require.Greater(t, len(completed.Poses), 2, `the probed checks reach into bisection`)
	require.Greater(t, canceled, 10)
}

// TestVerifyLinkageSweptBoxExclusion: a far body changes nothing but the
// static set. Scene 1 with a block at x ∈ [500, 510] lists the block in
// Against and in no pose row, and the report's Status, poses and collisions
// match scene 1's own.
func TestVerifyLinkageSweptBoxExclusion(t *testing.T) {
	t.Parallel()
	plain := buildFoldingArm(t, true)
	want := verifyLinkage(t, plain.doc, plain.linkage, plain.drive(), decad.WithResolution(units.Scalar(1.0/256)))

	a := buildFoldingArm(t, true)
	far := boxBody(t, a.doc, 500, -14, 510, 14, 10)
	report := verifyLinkage(t, a.doc, a.linkage, a.drive(), decad.WithResolution(units.Scalar(1.0/256)))
	require.Equal(t, []*decad.Body{a.wall, far}, report.Against)
	for _, p := range report.Poses {
		for _, row := range p.Clearances {
			require.NotSame(t, far, row.B)
		}
		for _, row := range p.Interferences {
			require.NotSame(t, far, row.B)
		}
		for _, d := range p.Diagnostics {
			require.Nil(t, d.Pair)
		}
	}
	require.Equal(t, want.Status, report.Status)
	require.Len(t, report.Poses, len(want.Poses))
	require.Len(t, report.Collisions, len(want.Collisions))
	for k, c := range want.Collisions {
		require.Equal(t, c.At, report.Collisions[k].At)
		require.Equal(t, c.Volume, report.Collisions[k].Volume)
	}
}

// TestVerifyLinkageSweptBoxCoversAncestorTravel pins how far a link body's
// swept box reaches. The shoulder of scene 1 swings from 80° to 90° and the
// elbow holds 0, so the forearm turns rigidly with the upper arm about the
// origin. A wall at y ∈ [60, 80] lies 46 mm beyond the forearm's zero-pose
// box, past anything its own joint moves it, and past the shoulder's travel
// across [80°, 90°] alone — but the forearm overlaps it at both ends. The box
// must grow by the travel from the zero pose through every joint on the
// forearm's path, ρ_12·90°.
//
// Legs seen to fail when deleted: the reach measured from the zero pose
// rather than across [From, To] (the wall is excluded and the report reads
// Sound over two collisions), and the ancestor joints' terms of the reach.
func TestVerifyLinkageSweptBoxCoversAncestorTravel(t *testing.T) {
	t.Parallel()
	a := buildFoldingArm(t, false)
	wall := boxBodyAtZ(t, a.doc, -50, 60, 50, 80, -10, 50)
	report := verifyLinkage(t, a.doc, a.linkage, decad.Drive{{Link: a.shoulder, From: units.Degrees(80), To: units.Degrees(90)}})
	require.Equal(t, decad.Interfering, report.Status)
	require.Len(t, report.Collisions, 2)
	for _, c := range report.Collisions {
		require.Same(t, a.forearm, c.A)
		require.Same(t, wall, c.B)
	}
	require.Equal(t, units.Scalar(0), report.Collisions[0].At)
	require.Equal(t, units.Scalar(1), report.Collisions[1].At)
}

// TestLinkageJointLimits covers WithJointLimits (docs/linkage-check-design.md
// §2.1, §2.4): a joint reports its limits, the constructor refuses
// ill-formed ones, and a drive that takes a joint outside them — at either
// end of its sweep, or holding an unlisted joint at a 0 the limits exclude —
// is refused by PoseAt and VerifyLinkage alike, with the link named.
// Comparisons are exact, across units: a radian sweep end is judged against
// degree limits through π's enclosure.
func TestLinkageJointLimits(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	arm := boxBody(t, doc, 0, -14, 48, 14, 10)
	block := boxBodyAtZ(t, doc, 60, -5, 70, 5, 0, 10)
	l := decad.NewLinkage()
	swing, err := l.Ground().Revolute(r3.Vec{}, zAxis, []*decad.Body{arm}, decad.WithJointLimits(units.Degrees(-90), units.Degrees(90)))
	require.NoError(t, err)
	slide, err := swing.Prismatic(r3.NewVec(1, 0, 0), []*decad.Body{block}, decad.WithJointLimits(units.Millimeters(5), units.Millimeters(20)))
	require.NoError(t, err)
	require.Equal(t, &decad.JointLimits{Min: units.Degrees(-90), Max: units.Degrees(90)}, swing.Joint().(decad.RevoluteJoint).Limits)
	require.Equal(t, &decad.JointLimits{Min: units.Millimeters(5), Max: units.Millimeters(20)}, slide.Joint().(decad.PrismaticJoint).Limits)
	inside := decad.JointSweep{Link: slide, From: units.Millimeters(5), To: units.Centimeters(2)}
	insideVia := decad.JointSweep{Link: slide, From: units.Millimeters(5), Via: []units.Value{units.Millimeters(20)}, To: units.Millimeters(5)}

	t.Run("a drive inside every limit poses", func(t *testing.T) {
		t.Parallel()
		pose, err := l.PoseAt(decad.Drive{{Link: swing, From: units.Degrees(-90), To: units.Radians(1.5)}, inside}, units.Scalar(1))
		require.NoError(t, err)
		require.Equal(t, units.Radians(1.5), pose.Values[0])
	})
	t.Run("a drive whose every waypoint is inside every limit poses", func(t *testing.T) {
		t.Parallel()
		drive := decad.Drive{{Link: swing, From: units.Degrees(-90), Via: []units.Value{units.Degrees(80)}, To: units.Degrees(0)}, insideVia}
		pose, err := l.PoseAt(drive, units.Scalar(0.5))
		require.NoError(t, err)
		require.Equal(t, units.Degrees(80), pose.Values[0], `the fraction 1/2 is the drive's one interior waypoint`)
		require.Equal(t, units.Millimeters(20), pose.Values[1])
	})
	refused := []struct {
		name  string
		drive decad.Drive
		link  string
	}{
		{"a sweep end past Max", decad.Drive{{Link: swing, From: units.Degrees(0), To: units.Radians(1.6)}, inside}, "link 0"},
		{"a sweep end below Min", decad.Drive{{Link: swing, From: units.Degrees(-91), To: units.Degrees(0)}, inside}, "link 0"},
		{"an unlisted joint held at an excluded 0", decad.Drive{{Link: swing, From: units.Degrees(0), To: units.Degrees(10)}}, "link 1"},
		// Both ends sit inside the limits; only the interior waypoint leaves
		// them, and the joint's value there is the waypoint itself.
		{"a Via waypoint past Max", decad.Drive{{Link: swing, From: units.Degrees(0), Via: []units.Value{units.Degrees(100)}, To: units.Degrees(0)}, insideVia}, "link 0's joint to 100"},
		{"a Via waypoint below Min", decad.Drive{{Link: swing, From: units.Degrees(0), Via: []units.Value{units.Degrees(10)}, To: units.Degrees(10)}, {Link: slide, From: units.Millimeters(5), Via: []units.Value{units.Millimeters(4)}, To: units.Millimeters(5)}}, "link 1's joint to 4"},
	}
	for _, tc := range refused {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			_, err := l.PoseAt(tc.drive, units.Scalar(0))
			require.ErrorIs(t, err, decad.ErrDegenerate)
			require.ErrorContains(t, err, tc.link)
			report, err := doc.VerifyLinkage(t.Context(), l, tc.drive)
			require.ErrorIs(t, err, decad.ErrDegenerate)
			require.Nil(t, report)
		})
	}
	free := boxBody(t, doc, 200, 0, 210, 10, 10)
	constructor := []struct {
		name string
		opt  decad.JointOption
		want error
	}{
		{"a length on a revolute", decad.WithJointLimits(units.Millimeters(0), units.Degrees(1)), decad.ErrUnitKind},
		{"a non-finite bound", decad.WithJointLimits(units.Degrees(0), units.Degrees(math.Inf(1))), decad.ErrNotFinite},
		{"Min equal to Max", decad.WithJointLimits(units.Degrees(180), units.Degrees(180)), decad.ErrDegenerate},
		{"Min above Max across units", decad.WithJointLimits(units.Degrees(180), units.Radians(3)), decad.ErrDegenerate},
		{"a nil option", nil, decad.ErrDegenerate},
	}
	for _, tc := range constructor {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			other := decad.NewLinkage()
			link, err := other.Ground().Revolute(r3.Vec{}, zAxis, []*decad.Body{free}, tc.opt)
			require.ErrorIs(t, err, tc.want)
			require.Nil(t, link)
			require.Empty(t, other.Links())
		})
	}
}

// TestLinkageDeclareJointContactRefusals is one subtest per refusal of
// docs/linkage-check-design.md §2.2, and the declaration order JointContacts
// keeps.
func TestLinkageDeclareJointContactRefusals(t *testing.T) {
	t.Parallel()
	a := buildFoldingArm(t, true)
	pin := boxBody(t, a.doc, 100, 0, 104, 4, 4)
	_, err := a.elbow.Prismatic(r3.NewVec(1, 0, 0), []*decad.Body{pin})
	require.NoError(t, err)
	free := boxBody(t, a.doc, 300, 0, 310, 10, 10)
	require.NoError(t, a.linkage.DeclareJointContact(a.upper, a.forearm))
	require.NoError(t, a.linkage.DeclareJointContact(a.wall, a.upper))
	require.NoError(t, a.linkage.DeclareJointContact(a.forearm, pin))
	require.Equal(t, []decad.DiagnosticPair{{A: a.upper, B: a.forearm}, {A: a.wall, B: a.upper}, {A: a.forearm, B: pin}}, a.linkage.JointContacts())
	var nilLinkage *decad.Linkage
	cases := []struct {
		name string
		l    *decad.Linkage
		x, y *decad.Body
	}{
		{"nil linkage", nilLinkage, a.upper, a.forearm},
		{"a nil body", a.linkage, a.upper, nil},
		{"a body with itself", a.linkage, a.upper, a.upper},
		{"two bodies of no link", a.linkage, a.wall, free},
		{"declared twice", a.linkage, a.upper, a.forearm},
		{"declared twice in the other order", a.linkage, a.forearm, a.upper},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			require.ErrorIs(t, tc.l.DeclareJointContact(tc.x, tc.y), decad.ErrDegenerate)
			require.Len(t, a.linkage.JointContacts(), 3)
		})
	}
	t.Run("two bodies of one link", func(t *testing.T) {
		doc := decad.New()
		one, two := boxBody(t, doc, 0, 0, 10, 10, 10), boxBody(t, doc, 20, 0, 30, 10, 10)
		l := decad.NewLinkage()
		_, err := l.Ground().Revolute(r3.Vec{}, zAxis, []*decad.Body{one, two})
		require.NoError(t, err)
		require.ErrorIs(t, l.DeclareJointContact(one, two), decad.ErrDegenerate)
	})
}
