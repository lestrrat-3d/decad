package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file holds the drive tests of docs/linkage-check-design.md §5.8, the
// projection certificate: the three-joint arm's flat minimum, scene 13's
// pendulum and scene 14's two arms. Each scene states its minimum in closed
// form, and every IntervalClear interval's Clearance is asserted at or below
// the closed-form gap at its ends, which bounds the true minimum over the
// interval from above.

// requireReadingEncloses asserts the whole-drive Clearance reading holds the
// closed-form minimum gap.
func requireReadingEncloses(t *testing.T, report *decad.LinkageReport, gap float64) {
	t.Helper()
	require.NotNil(t, report.Clearance)
	require.LessOrEqual(t, report.Clearance.Value.Mag()-report.Clearance.Bound.Mag(), gap)
	require.GreaterOrEqual(t, report.Clearance.Value.Mag()+report.Clearance.Bound.Mag(), gap)
}

// requireIntervalsBelow asserts every IntervalClear interval's proven lower
// bound sits at or below gap(s) at each end s the predicate admits.
func requireIntervalsBelow(t *testing.T, report *decad.LinkageReport, gap func(s float64) float64, admit func(s float64) bool) {
	t.Helper()
	for _, iv := range report.Intervals {
		if iv.Outcome != decad.IntervalClear {
			continue
		}
		require.NotNil(t, iv.Clearance)
		for _, s := range []float64{iv.From.Mag(), iv.To.Mag()} {
			if admit != nil && !admit(s) {
				continue
			}
			require.LessOrEqual(t, iv.Clearance.Value.Mag(), gap(s)+1e-9, "[%v, %v] at s = %v", iv.From.Mag(), iv.To.Mag(), s)
		}
	}
}

// TestVerifyLinkageThreeJointFlatMinimum is the projection certificate's
// acceptance on §10's three-joint arm. The wrist's leading corner (150, −10)
// sits at X(θ) = 50·(cos θ + cos 2θ + cos 3θ) + 10·sin 3θ,
// Y(θ) = 50·(sin θ + sin 2θ + sin 3θ) − 10·cos 3θ for θ = 90°·s. X peaks where
// X'(θ) = 0, at θ* ≈ 2.44°, with the corner at Y ≈ 2.8 inside the post's face
// x = 160, so the drive's minimum gap is 160 − X(θ*) ≈ 9.360 mm, a flat
// minimum: the gap grows with the square of the turn while the travel bound
// shrinks with its first power. Where |Y| ≤ 10 the corner faces the post, and
// 160 − X is the corner's distance from it, an upper bound on the gap.
//
// At the defaults the projection bound closes the reading before the verdict
// floor: Sound, the reading enclosing the minimum inside the gate, no interval
// narrower than 1/1024. The segment term (§5.8) charges the corner's motion
// along the drive itself, near zero at the minimum though each joint alone
// moves it, so the bound is second order in the step. Measured: 16 poses,
// against 251 under the travel bound alone. The reading floor's own leg, a
// tolerance that refines past the verdict floor, is
// TestVerifyLinkageReadingFloor's.
//
// Leg seen to fail when deleted: the projection bound (the reading refines to
// 1/16384, some interval narrower than 1/1024).
func TestVerifyLinkageThreeJointFlatMinimum(t *testing.T) {
	t.Parallel()
	x := func(th float64) float64 { return 50*(math.Cos(th)+math.Cos(2*th)+math.Cos(3*th)) + 10*math.Sin(3*th) }
	y := func(th float64) float64 { return 50*(math.Sin(th)+math.Sin(2*th)+math.Sin(3*th)) - 10*math.Cos(3*th) }
	dx := func(th float64) float64 {
		return -50*(math.Sin(th)+2*math.Sin(2*th)+3*math.Sin(3*th)) + 30*math.Cos(3*th)
	}
	thStar := bisectRoot(dx, 1*math.Pi/180, 4*math.Pi/180)
	require.InDelta(t, 2.44, thStar*180/math.Pi, 0.01)
	require.InDelta(t, 2.8, y(thStar), 0.05)
	minimum := 160 - x(thStar)
	require.InDelta(t, 9.360, minimum, 1e-3)
	gap := func(s float64) float64 { return 160 - x(s*math.Pi/2) }
	facesPost := func(s float64) bool { return math.Abs(y(s*math.Pi/2)) <= 10 }

	doc, l, drive := threeJointArm(t)
	report := verifyLinkage(t, doc, l, drive)
	require.Equal(t, decad.Sound, report.Status)
	require.Empty(t, report.Diagnostics)
	require.Equal(t, decad.ToleranceSatisfied, report.Clearance.Tolerance.State)
	requireReadingEncloses(t, report, minimum)
	requireDriveReadingCovers(t, report)
	require.GreaterOrEqual(t, narrowest(report), 1.0/1024, `no interval is narrower than the verdict floor`)
	requireIntervalsBelow(t, report, gap, facesPost)
}

// signedPermutation is a rotation that maps the coordinate axes onto signed
// coordinate axes: the column for each source axis.
type signedPermutation [3]r3.Vec

func (p signedPermutation) apply(v r3.Vec) r3.Vec {
	return p[0].Scale(v.X).Add(p[1].Scale(v.Y)).Add(p[2].Scale(v.Z))
}

// box builds the image under p of the box [lo, hi].
func (p signedPermutation) box(t *testing.T, doc *decad.Document, lo, hi r3.Vec) *decad.Body {
	t.Helper()
	a, b := p.apply(lo), p.apply(hi)
	return boxBodyAtZ(t, doc, math.Min(a.X, b.X), math.Min(a.Y, b.Y), math.Max(a.X, b.X), math.Max(a.Y, b.Y),
		math.Min(a.Z, b.Z), math.Abs(a.Z-b.Z))
}

// pendulum is scene 13 of docs/linkage-check-design.md §5.8 carried by p: a
// block x ∈ [−5, 5], y ∈ [−50, −40], z ∈ [0, 10] hanging from a revolute
// joint about Z through the origin, swinging 0° → 90° toward a wall
// x ∈ [−100, 100], y ∈ [20, 40], z ∈ [−10, 20] above the pivot.
func pendulum(t *testing.T, p signedPermutation) (*decad.Document, *decad.Linkage, decad.Drive) {
	t.Helper()
	doc := decad.New()
	block := p.box(t, doc, r3.NewVec(-5, -50, 0), r3.NewVec(5, -40, 10))
	p.box(t, doc, r3.NewVec(-100, 20, -10), r3.NewVec(100, 40, 20))
	l := decad.NewLinkage()
	swing, err := l.Ground().Revolute(r3.Vec{}, p.apply(zAxis), []*decad.Body{block})
	require.NoError(t, err)
	return doc, l, decad.Drive{{Link: swing, From: units.Degrees(0), To: units.Degrees(90)}}
}

// TestVerifyLinkagePendulum is scene 13 of docs/linkage-check-design.md §5.8,
// the remainder's leg. The block's highest corner (5, −40) sits at
// y = 5·sin θ − 40·cos θ, so the gap g(θ) = 20 − 5·sin θ + 40·cos θ falls
// from 60 to 15, concave up to θ = atan 8: on a concave stretch the minimum
// over an interval is g at its far end, and the expansion from the near end
// without its remainder, g(a) − |g'(a)|·Δθ, exceeds it by about
// ½·|g”|·Δθ². ρ_11 = √(50² + 5² + 10²) ≈ 50.25 and |g”| ≤ 40, so
// Rem = ½·ρ_11·Δθ² covers it and half of it does not near 0°.
//
//   - At WithResolution(1/16) every IntervalClear interval's Clearance is at
//     or below g at its far end, the true minimum over it.
//   - At WithMotionTolerance(1e-5) the report reads Sound: the minimum, 15 mm
//     at the drive's end, is approached at a slope of 40 mm per radian, and
//     the travel bound sits 5.1 mm per radian of the last step below it,
//     which the gate admits only below the reading floor, while the
//     projection bound's defect is second order in the step.
//   - The same scene carried onto each of the six coordinate directions, the
//     wall on each side of the pivot along it, reads Sound at that tolerance.
//
// Legs seen to fail when deleted: the remainder, and half of it (an
// interval's bound exceeds g at its far end); the projection bound, and each
// direction of the six in turn (the reading stops at the reading floor beyond
// tolerance, Suspect).
func TestVerifyLinkagePendulum(t *testing.T) {
	t.Parallel()
	g := func(s float64) float64 {
		th := s * math.Pi / 2
		return 20 - 5*math.Sin(th) + 40*math.Cos(th)
	}
	identity := signedPermutation{r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0), r3.NewVec(0, 0, 1)}
	t.Run("every interval sits below the gap at its far end", func(t *testing.T) {
		t.Parallel()
		doc, l, drive := pendulum(t, identity)
		report := verifyLinkage(t, doc, l, drive, decad.WithResolution(units.Scalar(1.0/16)))
		certified := 0
		for _, iv := range report.Intervals {
			if iv.Outcome != decad.IntervalClear {
				continue
			}
			certified++
			require.LessOrEqual(t, iv.Clearance.Value.Mag(), g(iv.To.Mag()), "[%v, %v]", iv.From.Mag(), iv.To.Mag())
		}
		require.NotZero(t, certified)
	})
	for _, tc := range []struct {
		name string
		p    signedPermutation
	}{
		{"+Y", identity},
		{"−Y", signedPermutation{r3.NewVec(-1, 0, 0), r3.NewVec(0, -1, 0), r3.NewVec(0, 0, 1)}},
		{"+X", signedPermutation{r3.NewVec(0, -1, 0), r3.NewVec(1, 0, 0), r3.NewVec(0, 0, 1)}},
		{"−X", signedPermutation{r3.NewVec(0, 1, 0), r3.NewVec(-1, 0, 0), r3.NewVec(0, 0, 1)}},
		{"+Z", signedPermutation{r3.NewVec(1, 0, 0), r3.NewVec(0, 0, 1), r3.NewVec(0, -1, 0)}},
		{"−Z", signedPermutation{r3.NewVec(1, 0, 0), r3.NewVec(0, 0, -1), r3.NewVec(0, 1, 0)}},
	} {
		t.Run("toward "+tc.name+" the reading closes", func(t *testing.T) {
			t.Parallel()
			doc, l, drive := pendulum(t, tc.p)
			report := verifyLinkage(t, doc, l, drive, decad.WithMotionTolerance(units.Scalar(1e-5)))
			require.Equal(t, decad.Sound, report.Status)
			requireReadingEncloses(t, report, 15)
			requireIntervalsBelow(t, report, g, nil)
		})
	}
}

// TestVerifyLinkageTwoArms is scene 14 of docs/linkage-check-design.md §5.8,
// both bodies of the pair moving. Arm A, x ∈ [0, 50], y ∈ [−5, 5],
// z ∈ [0, 10], turns about Z through the origin −30° → 30°; arm B,
// x ∈ [60, 110], the same section, turns about Z through (110, 0, 0) under the
// ground 30° → −30°. The corners (50, −5) of A and (60, −5) of B share their
// height at every θ and their edges lean away from each other, so the gap is
// the corners' distance g(θ) = 110 − 100·cos θ − 10·|sin θ|, flat at
// tan θ = 1/10: g* = 110 − 10·√101 ≈ 9.501 mm at s ≈ 0.4048 and 0.5952. Their
// z-extents coincide, so the layer exclusion does not settle the pair.
//
// Legs seen to fail when deleted: the partner's expansion (its corners read
// at the interval's end alone; an interval's bound exceeds g at an end near
// a minimum).
func TestVerifyLinkageTwoArms(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	armA := boxBody(t, doc, 0, -5, 50, 5, 10)
	armB := boxBody(t, doc, 60, -5, 110, 5, 10)
	l := decad.NewLinkage()
	a, err := l.Ground().Revolute(r3.Vec{}, zAxis, []*decad.Body{armA})
	require.NoError(t, err)
	b, err := l.Ground().Revolute(r3.NewVec(110, 0, 0), zAxis, []*decad.Body{armB})
	require.NoError(t, err)
	drive := decad.Drive{
		{Link: a, From: units.Degrees(-30), To: units.Degrees(30)},
		{Link: b, From: units.Degrees(30), To: units.Degrees(-30)},
	}
	g := func(s float64) float64 {
		th := (-30 + 60*s) * math.Pi / 180
		return 110 - 100*math.Cos(th) - 10*math.Abs(math.Sin(th))
	}
	minimum := 110 - 10*math.Sqrt(101)
	require.InDelta(t, 9.501, minimum, 1e-3)
	report := verifyLinkage(t, doc, l, drive)
	require.Equal(t, decad.Sound, report.Status)
	requireReadingEncloses(t, report, minimum)
	require.GreaterOrEqual(t, narrowest(report), 1.0/1024)
	requireIntervalsBelow(t, report, g, nil)
	rows := 0
	for _, p := range report.Poses {
		for _, row := range p.Clearances {
			if row.A == armA && row.B == armB {
				rows++
			}
		}
	}
	require.Equal(t, len(report.Poses), rows, `the arms are evaluated at every pose`)
}

// TestVerifyLinkageOutAndBackPin pins where docs/linkage-check-design.md
// §5.8's segment term stops: a blade x ∈ [0, 50], y ∈ [−0.5, 0.5],
// z ∈ [0, 10] turns about Z through 0° → 10° → 0°, and a 0.8 mm pin at radius
// 49 and polar angle 9°, z ∈ [4.6, 5.4], sits in its path near the 10°
// waypoint at s = 1/2. Both ends of the drive stand at 0°, 6.77 mm below the
// pin, so the one interval of the endpoints alone holds the waypoint, and
// its joints leave one straight segment: the box form charges the blade's
// tip its 50 mm per radian over the 20° the joint travels, and the interval
// stays undecided. At WithResolution(1/8) the pose at the waypoint finds the
// collision.
//
// Leg seen to fail when deleted: the waypoint test of the segment term (the
// step from 0° to 0° is zero, the remainder alone is charged, and the
// interval certifies across the collision).
func TestVerifyLinkageOutAndBackPin(t *testing.T) {
	t.Parallel()
	build := func(t *testing.T) (*decad.Document, *decad.Linkage, decad.Drive, *decad.Body, *decad.Body) {
		t.Helper()
		doc := decad.New()
		blade := boxBody(t, doc, 0, -0.5, 50, 0.5, 10)
		alpha := 9 * math.Pi / 180
		cx, cy := 49*math.Cos(alpha), 49*math.Sin(alpha)
		pin := boxBodyAtZ(t, doc, cx-0.4, cy-0.4, cx+0.4, cy+0.4, 4.6, 0.8)
		l := decad.NewLinkage()
		turn, err := l.Ground().Revolute(r3.Vec{}, zAxis, []*decad.Body{blade})
		require.NoError(t, err)
		return doc, l, decad.Drive{{
			Link: turn, From: units.Degrees(0), Via: []units.Value{units.Degrees(10)}, To: units.Degrees(0),
		}}, blade, pin
	}
	t.Run("the endpoints alone never certify the waypoint's interval", func(t *testing.T) {
		t.Parallel()
		doc, l, drive, _, _ := build(t)
		report := verifyLinkage(t, doc, l, drive, decad.WithResolution(units.Scalar(1)))
		require.Equal(t, decad.Suspect, report.Status)
		require.Len(t, report.Poses, 2)
		require.Empty(t, report.Collisions)
		require.Equal(t, decad.IntervalUndecided, report.Intervals[0].Outcome)
	})
	t.Run("the pose at the waypoint finds the pin", func(t *testing.T) {
		t.Parallel()
		doc, l, drive, blade, pin := build(t)
		report := verifyLinkage(t, doc, l, drive, decad.WithResolution(units.Scalar(1.0/8)))
		require.Equal(t, decad.Interfering, report.Status)
		require.NotEmpty(t, report.Collisions)
		for _, hit := range report.Collisions {
			require.Same(t, blade, hit.A)
			require.Same(t, pin, hit.B)
		}
	})
}
