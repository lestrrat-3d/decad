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

// This file holds scene 8 of docs/linkage-check-design.md §11: a pin on one
// moving link in a bore on another, coaxial in the ideal poses and a few ulps
// off concentric in the float ones, which the clearance kernel's windowed
// nested cell (docs/clearance-design.md §4) measures at every pose.
//
// The pin turns about its own joint, so the pair is a constant pair
// (docs/linkage-check-design.md §5.9): it is evaluated once, on the two
// bodies as they stand, and every pose reads that one outcome.
//
// Legs seen red, each by breaking what it guards: dropping the symmetry test
// from the constant-pair rule leaves the pinned elbow's and the static post's
// rows Approximate with bounds near 1e-11, and the jammed pin's volume
// measured again at s = 1, a few ulps off the one at s = 0; and deleting the
// windowed face cell, or the edge tier's windowed reading of the bore's rims
// against the pin, leaves the pin under two stacked joints without its row
// from s = 1/4 on. The filled bore stays undecided even when the windowed
// cells admit a lower bound at or below the kernel's tolerance, since the
// pair's own lower bound must clear that tolerance too; the face-level band
// test is the leg that goes red there
// (internal/clearance/facepair/cells_test.go).

// holedBar extrudes the rectangle (x0, y0)-(x1, y1) with a circular hole of
// radius bore about (cx, cy) over z ∈ [z0, z0+h].
func holedBar(t *testing.T, doc *decad.Document, x0, y0, x1, y1, cx, cy, bore, z0, h float64) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	plane, err := w.CreateOffsetPlane(w.XY(), z0)
	require.NoError(t, err)
	s, err := w.CreateSketch(plane)
	require.NoError(t, err)
	rect := s.CreateRectangle(x0, y0, x1, y1)
	s.Fix(rect.A)
	c := s.CreatePoint(cx, cy)
	s.Fix(c)
	s.CreateCircle(c, bore)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	var profile *sketch.Profile
	for _, cand := range s.Profiles() {
		if cand.Valid && len(cand.Holes) == 1 {
			profile = cand
		}
	}
	require.NotNil(t, profile)
	body, err := doc.Extrude(s, profile, decad.Distance{D: units.Millimeters(h), Dir: decad.Along})
	require.NoError(t, err)
	return body
}

// pinPrism extrudes the circle of radius r about (cx, cy) over
// z ∈ [z0, z0+h].
func pinPrism(t *testing.T, doc *decad.Document, cx, cy, r, z0, h float64) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	plane, err := w.CreateOffsetPlane(w.XY(), z0)
	require.NoError(t, err)
	s, err := w.CreateSketch(plane)
	require.NoError(t, err)
	c := s.CreatePoint(cx, cy)
	s.Fix(c)
	s.CreateCircle(c, r)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	body, err := doc.Extrude(s, s.Profiles()[0], decad.Distance{D: units.Millimeters(h), Dir: decad.Along})
	require.NoError(t, err)
	return body
}

// pairRows returns, per evaluated pose, the Clearance rows naming the pair in
// either order.
func pairRows(report *decad.LinkageReport, a, b *decad.Body) [][]decad.Clearance {
	out := make([][]decad.Clearance, len(report.Poses))
	for i, p := range report.Poses {
		for _, c := range p.Clearances {
			if (c.A == a && c.B == b) || (c.A == b && c.B == a) {
				out[i] = append(out[i], c)
			}
		}
	}
	return out
}

// pinnedElbow is scene 8: the upper arm A, x ∈ [−14, 62], y ∈ [−14, 14],
// z ∈ [0, 8], holed by the circle of radius 5.5 about (48, 0), on the
// shoulder revolute about Z through the origin, 0° → 90°; the elbow pin P, a
// circle of radius r about (48, 0) over z ∈ [−1, 9], on the elbow revolute
// about Z through (48, 0, 0), 0° → −90°, declared against A; and scene 1's
// wall x ∈ [−100, 150], y ∈ [38, 58], z ∈ [−10, 40].
type pinnedElbow struct {
	doc        *decad.Document
	arm, pin   *decad.Body
	wall       *decad.Body
	linkage    *decad.Linkage
	drive      decad.Drive
	report     *decad.LinkageReport
	armOnset   float64 // A's far corner (62, 14) reaches the wall
	pinOnset   float64 // P's rim reaches the wall
	jammedArea float64 // the annulus a pin of radius r > 5.5 shares with the bore
}

func verifyPinnedElbow(t *testing.T, r float64) pinnedElbow {
	t.Helper()
	e := pinnedElbow{doc: decad.New()}
	e.arm = holedBar(t, e.doc, -14, -14, 62, 14, 48, 0, 5.5, 0, 8)
	e.pin = pinPrism(t, e.doc, 48, 0, r, -1, 10)
	e.wall = boxBodyAtZ(t, e.doc, -100, 38, 150, 58, -10, 50)
	e.linkage = decad.NewLinkage()
	shoulder, err := e.linkage.Ground().Revolute(r3.Vec{}, zAxis, []*decad.Body{e.arm})
	require.NoError(t, err)
	elbow, err := shoulder.Revolute(r3.NewVec(48, 0, 0), zAxis, []*decad.Body{e.pin})
	require.NoError(t, err)
	require.NoError(t, e.linkage.DeclareJointContact(e.arm, e.pin))
	e.drive = decad.Drive{
		{Link: shoulder, From: units.Degrees(0), To: units.Degrees(90)},
		{Link: elbow, From: units.Degrees(0), To: units.Degrees(-90)},
	}
	e.report, err = e.doc.VerifyLinkage(t.Context(), e.linkage, e.drive, decad.WithResolution(units.Scalar(1.0/256)))
	require.NoError(t, err)
	// θ = 90°·s. A's corner sits at radius √(62² + 14²) and polar angle
	// atan(14/62); P's centre rides the circle of radius 48.
	e.armOnset = (math.Asin(38/math.Hypot(62, 14)) - math.Atan2(14, 62)) / (math.Pi / 2)
	e.pinOnset = math.Asin((38-r)/48) / (math.Pi / 2)
	e.jammedArea = math.Pi * (r*r - 5.5*5.5)
	return e
}

// requireWallCollisions asserts every collision of the report lies on
// (A, wall) or (P, wall), each past its closed-form onset.
func (e pinnedElbow) requireWallCollisions(t *testing.T) {
	t.Helper()
	require.Equal(t, decad.Interfering, e.report.Status)
	for _, c := range e.report.Collisions {
		require.Same(t, e.wall, c.B)
		switch c.A {
		case e.arm:
			require.Greater(t, c.At.Mag(), e.armOnset)
		case e.pin:
			require.Greater(t, c.At.Mag(), e.pinOnset)
		default:
			t.Fatalf("a collision on %v", c.A)
		}
	}
}

// TestVerifyLinkagePinnedElbow is scene 8. A and P are coaxial in the ideal
// poses at every s, since the pin turns about its own axis and the shoulder
// carries the bore along with it, but A's bore anchor is the shoulder
// rotation's image of (48, 0, 0) while P's is the Then-composed
// elbow-then-shoulder pose's image of (48, 0, −1), and the two part by a few
// ulps at most parameters. A's far corner reaches the wall at
// θ = asin(38/√(62² + 14²)) − atan(14/62) ≈ 24.0°, and P when
// 48·sin θ + 5 = 38, θ ≈ 43.4°.
func TestVerifyLinkagePinnedElbow(t *testing.T) {
	t.Parallel()
	t.Run("concentric", func(t *testing.T) {
		t.Parallel()
		e := verifyPinnedElbow(t, 5)
		e.requireWallCollisions(t)
		require.Greater(t, e.report.Collisions[0].At.Mag(), e.armOnset)
		require.LessOrEqual(t, e.report.Collisions[0].At.Mag()-e.armOnset, 2.0/256)
		require.Same(t, e.arm, e.report.Collisions[0].A)
		pinHit := false
		for _, c := range e.report.Collisions {
			pinHit = pinHit || c.A == e.pin
		}
		require.True(t, pinHit, `P reaches the wall at an evaluated pose`)
		requireExactRows(t, e.report, e.arm, e.pin, 0.5)
		requireNoFindingNames(t, e.report, e.arm, e.pin)
	})
	t.Run("two stacked joints", func(t *testing.T) {
		t.Parallel()
		// The pin hangs from a wrist joint on the elbow's own axis, under an
		// elbow link that carries a block above both bars: the pin turns
		// 0° → −45° on each, so its relative path holds two moving joints,
		// the pair is not constant, and every pose measures the pin through
		// the kernel's windowed nested cell.
		doc := decad.New()
		arm := holedBar(t, doc, -14, -14, 62, 14, 48, 0, 5.5, 0, 8)
		pin := pinPrism(t, doc, 48, 0, 5, -1, 10)
		block := boxBodyAtZ(t, doc, 46, -2, 50, 2, 20, 2)
		boxBodyAtZ(t, doc, -100, 38, 150, 58, -10, 50)
		l := decad.NewLinkage()
		shoulder, err := l.Ground().Revolute(r3.Vec{}, zAxis, []*decad.Body{arm})
		require.NoError(t, err)
		elbow, err := shoulder.Revolute(r3.NewVec(48, 0, 0), zAxis, []*decad.Body{block})
		require.NoError(t, err)
		wrist, err := elbow.Revolute(r3.NewVec(48, 0, 0), zAxis, []*decad.Body{pin})
		require.NoError(t, err)
		require.NoError(t, l.DeclareJointContact(arm, pin))
		drive := decad.Drive{
			{Link: shoulder, From: units.Degrees(0), To: units.Degrees(90)},
			{Link: elbow, From: units.Degrees(0), To: units.Degrees(-45)},
			{Link: wrist, From: units.Degrees(0), To: units.Degrees(-45)},
		}
		report, err := doc.VerifyLinkage(t.Context(), l, drive, decad.WithResolution(units.Scalar(1.0/256)))
		require.NoError(t, err)
		require.NotEmpty(t, report.Poses)
		for i, rows := range pairRows(report, arm, pin) {
			at := report.Poses[i].Pose.At.Mag()
			require.Lenf(t, rows, 1, `the pin's row at s = %v`, at)
			require.InDeltaf(t, 0.5, rows[0].Gap.Value.Base(), 1e-9, `s = %v`, at)
			require.LessOrEqualf(t, rows[0].Gap.Bound.Base(), 1e-9, `s = %v`, at)
		}
		requireNoFindingNames(t, report, arm, pin)
	})
	t.Run("jammed", func(t *testing.T) {
		t.Parallel()
		// A pin of radius 5.6 shares the annulus π·(5.6² − 5.5²) with the bore
		// over the arm's 8 mm: the bore body's caps cross the pin's wall, and
		// that overlap is read ahead of the 0.1 mm the windowed cell proves
		// between the two cylinder faces.
		e := verifyPinnedElbow(t, 5.6)
		require.Equal(t, decad.Interfering, e.report.Status)
		want := 8 * e.jammedArea
		var once *decad.Measurement
		for _, p := range e.report.Poses {
			found := false
			for _, c := range e.report.Collisions {
				if c.At.Mag() != p.Pose.At.Mag() || c.A != e.arm || c.B != e.pin {
					continue
				}
				found = true
				require.InDeltaf(t, want, c.Volume.Value.Base(), c.Volume.Bound.Base(), `s = %v`, p.Pose.At.Mag())
				if once == nil {
					once = &c.Volume
				}
				require.Equalf(t, *once, c.Volume, `the volume is measured once and serves s = %v`, p.Pose.At.Mag())
			}
			require.Truef(t, found, `the jammed pin collides at s = %v`, p.Pose.At.Mag())
		}
	})
	t.Run("filled bore", func(t *testing.T) {
		t.Parallel()
		e := verifyPinnedElbow(t, 5.5)
		e.requireWallCollisions(t)
		for i, rows := range pairRows(e.report, e.arm, e.pin) {
			require.Emptyf(t, rows, `a pin that fills its bore publishes nothing at s = %v`, e.report.Poses[i].Pose.At.Mag())
		}
		requireNoFindingNames(t, e.report, e.arm, e.pin)
	})
}

// TestVerifyLinkagePinOnStaticPost is scene 8 without the elbow: P static, A
// on a shoulder revolute about Z through (48, 0, 0), the bore on the joint.
// The static post is symmetric about the joint, so the pair is constant and
// every pose reads the exact 0.5.
func TestVerifyLinkagePinOnStaticPost(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	arm := holedBar(t, doc, -14, -14, 62, 14, 48, 0, 5.5, 0, 8)
	post := pinPrism(t, doc, 48, 0, 5, -1, 10)
	boxBodyAtZ(t, doc, -100, 38, 150, 58, -10, 50)
	l := decad.NewLinkage()
	shoulder, err := l.Ground().Revolute(r3.NewVec(48, 0, 0), zAxis, []*decad.Body{arm})
	require.NoError(t, err)
	require.NoError(t, l.DeclareJointContact(arm, post))
	drive := decad.Drive{{Link: shoulder, From: units.Degrees(0), To: units.Degrees(90)}}
	report, err := doc.VerifyLinkage(t.Context(), l, drive, decad.WithResolution(units.Scalar(1.0/256)))
	require.NoError(t, err)
	require.NotEmpty(t, report.Poses)
	requireExactRows(t, report, arm, post, 0.5)
	requireNoFindingNames(t, report, arm, post)
}

// requireExactRows asserts every evaluated pose of the report carries one
// Clearance row for the pair, Exact at gap with a zero Bound.
func requireExactRows(t *testing.T, report *decad.LinkageReport, a, b *decad.Body, gap float64) {
	t.Helper()
	for i, rows := range pairRows(report, a, b) {
		at := report.Poses[i].Pose.At.Mag()
		require.Lenf(t, rows, 1, `the pair's row at s = %v`, at)
		require.Equalf(t, decad.Exact, rows[0].Gap.Exactness, `s = %v`, at)
		require.Equalf(t, gap, rows[0].Gap.Value.Base(), `s = %v`, at)
		require.Zerof(t, rows[0].Gap.Bound.Base(), `s = %v`, at)
	}
}
