package apitest_test

import (
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// requireDriveReadingCovers asserts the whole-drive reading is consistent with
// the poses that bound it: its interval reaches no higher than the smallest
// proven upper end of any gap row, and no lower than zero.
func requireDriveReadingCovers(t *testing.T, report *decad.LinkageReport) {
	t.Helper()
	require.NotNil(t, report.Clearance)
	lower := report.Clearance.Value.Mag() - report.Clearance.Bound.Mag()
	upper := report.Clearance.Value.Mag() + report.Clearance.Bound.Mag()
	require.Greater(t, lower, 0.0)
	smallestHi := upper
	for _, p := range report.Poses {
		for _, row := range p.Clearances {
			smallestHi = min(smallestHi, row.Gap.Value.Mag()+row.Gap.Bound.Mag())
		}
	}
	require.LessOrEqual(t, upper, smallestHi+1e-9, `the reading's upper end is a pose's own upper bound`)
}

// narrowest is the width of the narrowest evaluated interval.
func narrowest(report *decad.LinkageReport) float64 {
	w := 1.0
	for _, iv := range report.Intervals {
		w = min(w, iv.To.Mag()-iv.From.Mag())
	}
	return w
}

// TestVerifyLinkageReadingFloor pins docs/linkage-check-design.md §3's two
// floors.
//
//   - At the defaults the three-joint arm of §10 closes its reading before
//     the verdict floor (TestVerifyLinkageThreeJointFlatMinimum). Under a
//     tighter tolerance, WithMotionTolerance(1e-5), the reading refines past
//     the verdict floor 1/1024 around its one minimum (the wrist's corner
//     9.36 mm from a post) toward the reading floor 1/16384: Sound, with the
//     reading inside the gate, in 25 poses.
//   - Stated, WithResolution is both floors: at 1/64 the same arm stops its
//     reading at 1/64, Suspect with the reading beyond tolerance.
//   - A margin refines to the verdict floor only: scene 1's arms, settled by
//     the layer exclusion at a proven 2 mm, against a 3 mm minimum read
//     AssessmentUndecided with no interval narrower than 1/1024.
//   - A constant gap the layer rule cannot settle still stops: a disc of
//     radius 5 spinning 0° → 90° about an axis 1e-9 mm off its own, which
//     the symmetry rule therefore keeps, beside a wall in its layer, 7 mm
//     away, ties every interval for the smallest bound, so the reading
//     refines all of them. The travel is 5·√2·(π/2)·Δs, and the gate
//     admits 7e-3 mm at Δs = 1/512 but not at 1/256: 513 poses, Sound.
//
// Legs seen to fail when deleted: the reading floor (the arm stops at
// 1/1024 and reads Suspect); a stated resolution fixing the reading floor
// (the 1/64 arm refines on and reads Sound); and the margin's verdict floor
// (the 3 mm margin's intervals refine past 1/1024).
func TestVerifyLinkageReadingFloor(t *testing.T) {
	t.Parallel()
	t.Run("the three-joint arm refines its reading past the verdict floor", func(t *testing.T) {
		t.Parallel()
		doc, l, drive := threeJointArm(t)
		report := verifyLinkage(t, doc, l, drive, decad.WithMotionTolerance(units.Scalar(1e-5)))
		require.Equal(t, decad.Sound, report.Status)
		require.Empty(t, report.Diagnostics)
		require.Equal(t, units.Scalar(1.0/1024), report.Request.Resolution)
		require.Equal(t, units.Scalar(1.0/16384), report.ReadingResolution)
		require.Equal(t, decad.ToleranceSatisfied, report.Clearance.Tolerance.State)
		requireDriveReadingCovers(t, report)
		require.GreaterOrEqual(t, narrowest(report), 1.0/16384)
		require.Less(t, narrowest(report), 1.0/1024, `the reading refines past the verdict floor`)
		for _, iv := range report.Intervals {
			require.Equal(t, decad.IntervalClear, iv.Outcome)
		}
	})
	t.Run("a stated resolution is both floors", func(t *testing.T) {
		t.Parallel()
		doc, l, drive := threeJointArm(t)
		report := verifyLinkage(t, doc, l, drive, decad.WithResolution(units.Scalar(1.0/64)))
		require.Equal(t, units.Scalar(1.0/64), report.ReadingResolution)
		require.Equal(t, decad.Suspect, report.Status)
		require.Equal(t, decad.ToleranceExceeded, report.Clearance.Tolerance.State)
		require.GreaterOrEqual(t, narrowest(report), 1.0/64)
	})
	t.Run("a margin refines to the verdict floor", func(t *testing.T) {
		t.Parallel()
		a := buildFoldingArm(t, false)
		report := verifyLinkage(t, a.doc, a.linkage, a.drive(), decad.WithMinClearance(units.Millimeters(3)))
		require.Equal(t, decad.AssessmentUndecided, report.Assessment)
		require.Equal(t, decad.Suspect, report.Status)
		require.GreaterOrEqual(t, narrowest(report), 1.0/1024)
	})
	t.Run("a constant gap the layer rule cannot settle stops", func(t *testing.T) {
		t.Parallel()
		doc := decad.New()
		disc := discBodySymmetric(t, doc, 0, 5, 5)
		boxBodyAtZ(t, doc, 12, -20, 22, 20, -10, 20)
		l := decad.NewLinkage()
		// 1e-9 mm off the disc's own axis, so the symmetry rule (§5.2) keeps
		// the joint and the gap stays constant to that width.
		spin, err := l.Ground().Revolute(r3.NewVec(1e-9, 0, 0), zAxis, []*decad.Body{disc})
		require.NoError(t, err)
		report := verifyLinkage(t, doc, l, decad.Drive{{Link: spin, From: units.Degrees(0), To: units.Degrees(90)}})
		require.Equal(t, decad.Sound, report.Status)
		require.Len(t, report.Poses, 513)
		require.Equal(t, 1.0/512, narrowest(report))
		requireDriveReadingCovers(t, report)
		// The disc's centre swings 1e-9 mm toward the wall at the end.
		minimum := 7 - 1e-9
		require.LessOrEqual(t, report.Clearance.Value.Mag()-report.Clearance.Bound.Mag(), minimum)
		require.GreaterOrEqual(t, report.Clearance.Value.Mag()+report.Clearance.Bound.Mag(), minimum)
	})
}
