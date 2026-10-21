package report_test

import (
	"strconv"
	"testing"

	"github.com/lestrrat-3d/decad/report"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// refs maps an integer body or face reference to its decimal name.
func refs() report.RefLift[int, int, string, string] {
	name := func(n int) string { return strconv.Itoa(n) }
	return report.RefLift[int, int, string, string]{Body: name, Face: name}
}

type (
	intReport = report.ReportOf[int, int, report.JointCell]
	intBody   = report.BodyReportOf[int, int, report.JointCell]
	intDiag   = report.DiagnosticOf[int, report.JointCell]
)

func TestLiftVerifyReport(t *testing.T) {
	t.Parallel()
	lift := refs()
	var none *intReport
	require.Nil(t, report.LiftVerifyReport(&lift, none), "a nil report stays nil")

	wall := &report.WallRequest{Minimum: units.Millimeters(1)}
	cell := &report.JointCell{Min: []units.Value{units.Millimeters(1)}}
	pairDiag := intDiag{Code: report.DiagInterference, Status: report.Interfering,
		Pair: &report.DiagnosticPairOf[int]{A: 1, B: 2}, Message: "pair"}
	bodyDiag := intDiag{Code: report.DiagWallTooThin, Status: report.Violating, Body: 3, Cell: cell}
	in := &intReport{
		Request: report.VerifyRequest{Wall: wall},
		Bodies: []*intBody{{
			Body: 1, Status: report.Violating,
			Validity: report.ValidityResultOf[int, report.JointCell]{Outcome: report.ValidityValid,
				Diagnostics: []intDiag{bodyDiag}},
			Wall: report.WallResultOf[int, report.JointCell]{Request: wall, Outcome: report.ScalarMeasured,
				Diagnostics: []intDiag{bodyDiag}},
			Undercut: report.UndercutResultOf[int, int, report.JointCell]{Faces: []int{7, 8}},
			ConcaveRadius: report.ConcaveRadiusResultOf[int, report.JointCell]{Outcome: report.ScalarAbsent,
				Diagnostics: []intDiag{bodyDiag}},
			Diagnostics: []intDiag{bodyDiag},
		}, nil},
		Interferences: []report.InterferenceOf[int]{{A: 1, B: 2, Volume: report.Measurement{Value: units.CubicMillimeters(4)}}},
		Clearances:    []report.ClearanceOf[int]{},
		Diagnostics:   []intDiag{pairDiag},
		Status:        report.Interfering,
	}

	out := report.LiftVerifyReport(&lift, in)
	require.Equal(t, report.Interfering, out.Status)
	require.Same(t, wall, out.Request.Wall, "the request's pointers carry over")
	require.Len(t, out.Bodies, 2)
	require.Nil(t, out.Bodies[1], "an absent body row stays nil")
	row := out.Bodies[0]
	require.Equal(t, "1", row.Body)
	require.Same(t, wall, row.Wall.Request, "a body's wall request is the report's")
	require.Equal(t, []string{"7", "8"}, row.Undercut.Faces)
	require.Equal(t, "3", row.Validity.Diagnostics[0].Body)
	require.Equal(t, "3", row.Wall.Diagnostics[0].Body)
	require.Equal(t, "3", row.ConcaveRadius.Diagnostics[0].Body)
	require.Equal(t, "3", row.Diagnostics[0].Body)
	require.Same(t, cell, row.Diagnostics[0].Cell)
	require.Equal(t, report.ValidityValid, row.Validity.Outcome)
	require.Equal(t, report.ScalarMeasured, row.Wall.Outcome)
	require.Equal(t, report.ScalarAbsent, row.ConcaveRadius.Outcome)

	require.Equal(t, "1", out.Interferences[0].A)
	require.Equal(t, "2", out.Interferences[0].B)
	require.InDelta(t, 4, out.Interferences[0].Volume.Value.Base(), 0)
	require.NotNil(t, out.Clearances, "an empty clearance list stays empty, not nil")
	require.Empty(t, out.Clearances)
	require.Equal(t, "1", out.Diagnostics[0].Pair.A)
	require.Equal(t, "2", out.Diagnostics[0].Pair.B)
	require.Equal(t, "pair", out.Diagnostics[0].Message)

	in.Clearances = nil
	in.Interferences = nil
	empty := report.LiftVerifyReport(&lift, in)
	require.Nil(t, empty.Clearances, "a nil list stays nil")
	require.Nil(t, empty.Interferences)
}

func TestLiftMotionReport(t *testing.T) {
	t.Parallel()
	lift := refs()
	var none *report.MotionReportOf[int, any]
	require.Nil(t, report.LiftMotionReport(&lift, none, "m"), "a nil report stays nil")

	diag := intDiag{Code: report.DiagMotionCollision, Status: report.Interfering, Body: 4,
		Pair: &report.DiagnosticPairOf[int]{A: 1, B: 2}}
	in := &report.MotionReportOf[int, any]{
		Request: report.MotionRequest{Resolution: units.Scalar(0.5)},
		Moving:  []int{1, 3}, Against: []int{},
		Poses: []report.PoseResultOf[int]{{
			At:            units.Scalar(0.25),
			Interferences: []report.InterferenceOf[int]{{A: 1, B: 2}},
			Clearances:    []report.ClearanceOf[int]{{A: 1, B: 5}},
			Diagnostics:   []intDiag{diag},
		}},
		Intervals:   []report.MotionInterval{{Outcome: report.IntervalColliding}},
		Collisions:  []report.CollisionOf[int]{{Moving: 1, Static: 2, Volume: report.Measurement{Value: units.CubicMillimeters(3)}}},
		Clearance:   &report.ScalarReading{},
		Assessment:  report.AssessmentViolated,
		Diagnostics: []intDiag{diag},
		Status:      report.Interfering,
	}

	out := report.LiftMotionReport(&lift, in, "stated motion")
	require.Equal(t, "stated motion", out.Motion, "the stated motion is the caller's")
	require.Equal(t, []string{"1", "3"}, out.Moving)
	require.NotNil(t, out.Against, "an empty list stays empty")
	require.Empty(t, out.Against)
	require.Equal(t, in.Request, out.Request)
	require.Equal(t, report.IntervalColliding, out.Intervals[0].Outcome)
	require.Equal(t, "1", out.Poses[0].Interferences[0].A)
	require.Equal(t, "2", out.Poses[0].Interferences[0].B)
	require.Equal(t, "5", out.Poses[0].Clearances[0].B)
	require.Equal(t, "4", out.Poses[0].Diagnostics[0].Body)
	require.Equal(t, "2", out.Poses[0].Diagnostics[0].Pair.B)
	require.Equal(t, "1", out.Collisions[0].Moving)
	require.Equal(t, "2", out.Collisions[0].Static)
	require.InDelta(t, 3, out.Collisions[0].Volume.Value.Base(), 0)
	require.Same(t, in.Clearance, out.Clearance)
	require.Equal(t, report.AssessmentViolated, out.Assessment)
	require.Equal(t, "4", out.Diagnostics[0].Body)
	require.Equal(t, report.Interfering, out.Status)
	require.False(t, out.Passed())

	in.Moving, in.Poses, in.Collisions, in.Diagnostics = nil, nil, nil, nil
	empty := report.LiftMotionReport(&lift, in, "m")
	require.Nil(t, empty.Moving)
	require.Nil(t, empty.Poses)
	require.Nil(t, empty.Collisions)
	require.Nil(t, empty.Diagnostics)
}

func TestMotionReportPassed(t *testing.T) {
	t.Parallel()
	var none *report.MotionReportOf[int, any]
	require.False(t, none.Passed())
	require.False(t, (&report.MotionReportOf[int, any]{Status: report.Suspect}).Passed())
	require.True(t, (&report.MotionReportOf[int, any]{Status: report.Sound}).Passed())
}

func TestIntervalOutcomeString(t *testing.T) {
	t.Parallel()
	for outcome, want := range map[report.IntervalOutcome]string{
		report.IntervalNotEvaluated: "not_evaluated", report.IntervalClear: "clear",
		report.IntervalColliding: "colliding", report.IntervalUndecided: "undecided",
		report.IntervalOutcome(9): "interval_outcome(9)",
	} {
		require.Equal(t, want, outcome.String())
	}
}

func TestJointCellClone(t *testing.T) {
	t.Parallel()
	cell := report.JointCell{Min: []units.Value{units.Scalar(1)}, Max: []units.Value{units.Scalar(2)}}
	clone := cell.Clone()
	clone.Min[0], clone.Max[0] = units.Scalar(7), units.Scalar(8)
	require.InDelta(t, 1, cell.Min[0].Base(), 0)
	require.InDelta(t, 2, cell.Max[0].Base(), 0)
}
