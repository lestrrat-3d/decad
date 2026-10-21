package report_test

import (
	"strconv"
	"testing"

	"github.com/lestrrat-3d/decad/report"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// refLift maps an integer reference to its decimal name, and zero to "".
func refLift() report.Lift[int, int, int, int, string, string, string, string] {
	name := func(n int) string {
		if n == 0 {
			return ""
		}
		return strconv.Itoa(n)
	}
	return report.Lift[int, int, int, int, string, string, string, string]{Body: name, Face: name, Edge: name, Vertex: name}
}

func TestLiftManifold(t *testing.T) {
	t.Parallel()
	lift := refLift()

	require.Nil(t, lift.Manifold(nil), "a nil manifold stays nil")

	empty := lift.Manifold(&report.ContactManifoldOf[int, int, int]{Points: []report.ContactPointOf[int, int, int]{}})
	require.NotNil(t, empty.Points, "an empty manifold stays empty, not nil")
	require.Empty(t, empty.Points)

	none := lift.Manifold(&report.ContactManifoldOf[int, int, int]{})
	require.NotNil(t, none)
	require.Nil(t, none.Points, "a manifold without points keeps nil points")

	point := report.ContactPointOf[int, int, int]{
		FaceA: 1, FaceB: 2,
		FeatureA: report.ContactFeatureOf[int, int, int]{Face: 1, Edge: 3},
		FeatureB: report.ContactFeatureOf[int, int, int]{Face: 2, Vertex: 4},
	}
	got := lift.Manifold(&report.ContactManifoldOf[int, int, int]{Points: []report.ContactPointOf[int, int, int]{point}})
	require.Len(t, got.Points, 1)
	require.Equal(t, "1", got.Points[0].FaceA)
	require.Equal(t, "2", got.Points[0].FaceB)
	require.Equal(t, report.ContactFeatureOf[string, string, string]{Face: "1", Edge: "3"}, got.Points[0].FeatureA)
	require.Equal(t, report.ContactFeatureOf[string, string, string]{Face: "2", Vertex: "4"}, got.Points[0].FeatureB)
	require.Empty(t, got.Points[0].FeatureA.Vertex, "a zero reference stays the zero reference")
}

func TestLiftReport(t *testing.T) {
	t.Parallel()
	lift := refLift()
	gap := report.Measurement{Value: units.Millimeters(2)}
	in := &report.ContactReportOf[int, int, int, int]{
		A: 7, B: 8, Relation: report.ContactSeparated, Reason: report.ContactNoGapProof, Gap: &gap,
	}

	out := lift.Report(in)
	require.Equal(t, "7", out.A)
	require.Equal(t, "8", out.B)
	require.Equal(t, report.ContactSeparated, out.Relation)
	require.Equal(t, report.ContactNoGapProof, out.Reason)
	require.Nil(t, out.Overlap, "an absent overlap stays nil")
	require.Nil(t, out.Manifold, "an absent manifold stays nil")

	require.NotNil(t, out.Gap)
	require.Equal(t, gap, *out.Gap)
	out.Gap.Value = units.Millimeters(9)
	require.InDelta(t, 2, in.Gap.Value.Base(), 0, "the lifted gap is a copy, so changing it leaves the source alone")
}
