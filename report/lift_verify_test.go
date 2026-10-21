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
