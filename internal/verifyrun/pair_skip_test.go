package verifyrun

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/verifyoption"
	vocab "github.com/lestrrat-3d/decad/report"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// skipTestReports returns n solid body reports whose boxes all overlap, so
// every pair reaches pair work unless it is skipped.
func skipTestReports(n int) []*BodyReport {
	reports := make([]*BodyReport, n)
	for i := range reports {
		box := vocab.Box{Min: r3.NewVec(float64(i), 0, 0), Max: r3.NewVec(float64(i)+10, 10, 10), Bound: units.Millimeters(0)}
		reports[i] = &BodyReport{Body: &topo.Body{Kind: topo.BodySolid, Solid: true}, Bounds: vocab.BoundsReading{Box: box}}
	}
	return reports
}

// TestVerifyPairJobsLeaveSkippedPairsOut holds the pair partition to the
// skipped pairs: a skipped pair, named in either order and more than once,
// gets no job, so no proof, mesh or clearance work runs for it, while every
// other overlapping pair keeps its job in pair order. Without clearances, a
// skipped pair whose boxes are apart is absent as before; with them it is
// still skipped.
func TestVerifyPairJobsLeaveSkippedPairsOut(t *testing.T) {
	t.Parallel()
	reports := skipTestReports(4)
	body := func(i int) *topo.Body { return reports[i].Body }
	skip := newPairSkips([]verifyoption.SkippedPair{
		{A: body(2), B: body(0)}, {A: body(0), B: body(2)}, {A: body(1), B: body(3)},
	})
	for _, clearances := range []bool{false, true} {
		jobs, err := verifyPairJobs(t.Context(), reports, verifyConfig{Clearances: clearances}, skip)
		require.NoError(t, err)
		var got [][2]*topo.Body
		for _, job := range jobs {
			got = append(got, [2]*topo.Body{job.a, job.b})
		}
		want := [][2]*topo.Body{{body(0), body(1)}, {body(0), body(3)}, {body(1), body(2)}, {body(2), body(3)}}
		require.Equal(t, want, got, "clearances %v", clearances)
	}

	all, err := verifyPairJobs(t.Context(), reports, verifyConfig{}, nil)
	require.NoError(t, err)
	require.Len(t, all, 6, "premise: without skips every overlapping pair has a job")

	// A skipped pair whose boxes are apart: absent without clearances in any
	// case, absent with clearances only because it is skipped.
	far := skipTestReports(2)
	far[1].Bounds.Min, far[1].Bounds.Max = r3.NewVec(100, 0, 0), r3.NewVec(110, 10, 10)
	farSkip := newPairSkips([]verifyoption.SkippedPair{{A: far[0].Body, B: far[1].Body}})
	jobs, err := verifyPairJobs(t.Context(), far, verifyConfig{Clearances: true}, nil)
	require.NoError(t, err)
	require.Len(t, jobs, 1, "premise: a clearance request measures a box-apart pair")
	jobs, err = verifyPairJobs(t.Context(), far, verifyConfig{Clearances: true}, farSkip)
	require.NoError(t, err)
	require.Empty(t, jobs)
}

// TestPairSkipsInOrder holds Report.SkippedPairs to its contract: each
// skipped pair once, in the i < j order of the bodies with A the earlier one,
// pairs naming a body outside the list left out, and nil when nothing was
// skipped.
func TestPairSkipsInOrder(t *testing.T) {
	t.Parallel()
	reports := skipTestReports(4)
	bodies := make([]*topo.Body, len(reports))
	for i, r := range reports {
		bodies[i] = r.Body
	}
	outside := &topo.Body{Kind: topo.BodySolid}
	skip := newPairSkips([]verifyoption.SkippedPair{
		{A: bodies[3], B: bodies[1]}, {A: bodies[2], B: bodies[0]}, {A: bodies[1], B: bodies[3]}, {A: outside, B: bodies[0]},
	})
	require.Equal(t, []vocab.DiagnosticPairOf[*topo.Body]{
		{A: bodies[0], B: bodies[2]}, {A: bodies[1], B: bodies[3]},
	}, skip.inOrder(bodies))
	require.Nil(t, newPairSkips(nil).inOrder(bodies))
}
