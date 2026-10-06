package decad

import (
	"context"
	"errors"
	"math"
	"strconv"
	"testing"
	"time"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/stretchr/testify/require"
)

// pairJobsByIndex builds n jobs whose first operand identifies the job, so a
// prove function can tell which job it was handed.
func pairJobsByIndex(n int) ([]verifyPairJob, map[*Body]int) {
	jobs := make([]verifyPairJob, n)
	index := make(map[*Body]int, n)
	for i := range jobs {
		a := &Body{}
		jobs[i] = verifyPairJob{a: a}
		index[a] = i
	}
	return jobs, index
}

// TestRunVerifyPairsKeepsJobOrder finishes later jobs first and requires the
// outcomes back in job order, whatever order the workers complete in.
func TestRunVerifyPairsKeepsJobOrder(t *testing.T) {
	t.Parallel()
	const n = 24
	jobs, index := pairJobsByIndex(n)
	prove := func(_ context.Context, job verifyPairJob) (verifyPairOutcome, error) {
		i := index[job.a]
		time.Sleep(time.Duration(n-i) * time.Millisecond)
		return verifyPairOutcome{diagnostics: []Diagnostic{{Message: strconv.Itoa(i)}}, undecided: i%2 == 0}, nil
	}
	for _, workers := range []int{1, 2, 8, 64} {
		out, err := runVerifyPairs(t.Context(), jobs, workers, prove)
		require.NoError(t, err, `%d workers`, workers)
		require.Len(t, out, n)
		for i, o := range out {
			require.Equal(t, strconv.Itoa(i), o.diagnostics[0].Message, `%d workers, slot %d`, workers, i)
			require.Equal(t, i%2 == 0, o.undecided)
		}
	}
}

// TestRunVerifyPairsReturnsLowestFailingError fails two jobs, the later one
// first, and requires the error of the earlier one: the error a walk of the
// jobs in order stops on. Every job before it must still have run.
func TestRunVerifyPairsReturnsLowestFailingError(t *testing.T) {
	t.Parallel()
	const n = 20
	errEarly := errors.New("early job failed")
	errLate := errors.New("late job failed")
	for _, workers := range []int{1, 3, 8} {
		jobs, index := pairJobsByIndex(n)
		ran := make([]bool, n)
		prove := func(_ context.Context, job verifyPairJob) (verifyPairOutcome, error) {
			i := index[job.a]
			ran[i] = true
			switch i {
			case 5:
				time.Sleep(30 * time.Millisecond)
				return verifyPairOutcome{}, errEarly
			case 12:
				return verifyPairOutcome{}, errLate
			}
			return verifyPairOutcome{}, nil
		}
		out, err := runVerifyPairs(t.Context(), jobs, workers, prove)
		require.ErrorIs(t, err, errEarly, `%d workers`, workers)
		require.Nil(t, out)
		for i := range 5 {
			require.True(t, ran[i], `%d workers: job %d precedes the failure and must run`, workers, i)
		}
	}

	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	jobs, _ := pairJobsByIndex(n)
	_, err := runVerifyPairs(ctx, jobs, 4, func(context.Context, verifyPairJob) (verifyPairOutcome, error) {
		return verifyPairOutcome{}, nil
	})
	require.ErrorIs(t, err, context.Canceled)
}

// internalFrustumBody revolves the outline (0,0) (0,ra) (L,rb) (L,0) about
// the sketch's U axis and places U along a → b.
func internalFrustumBody(t *testing.T, doc *Document, a, b r3.Vec, ra, rb float64) *Body {
	t.Helper()
	l := b.Sub(a).Len()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	pts := [][2]float64{{0, 0}, {0, ra}, {l, rb}, {l, 0}}
	ps := make([]*sketch.Point, len(pts))
	for i, p := range pts {
		ps[i] = s.CreatePoint(p[0], p[1])
		s.Fix(ps[i])
	}
	for i := range ps {
		s.CreateLine(ps[i], ps[(i+1)%len(ps)])
	}
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	body, err := doc.Revolve(s, s.Profiles()[0], SketchLine{End: Point2{U: 1}}, FullRevolution{})
	require.NoError(t, err)
	d, ok := b.Sub(a).Normalize()
	require.True(t, ok)
	ref := r3.NewVec(0, 1, 0)
	if math.Abs(d.Y) > 0.9 {
		ref = r3.NewVec(1, 0, 0)
	}
	p, ok := d.Cross(ref).Normalize()
	require.True(t, ok)
	place, err := r3.FromBasis(r3.Basis{EX: d, EY: p.Cross(d), EZ: p}, a)
	require.NoError(t, err)
	body, err = body.Placed(t.Context(), place)
	require.NoError(t, err)
	return body
}

// TestVerifyPairWorkersMatchOneWorker verifies one document with one worker
// and with several, with and without clearances, and requires the two reports
// equal field for field. The document holds every pair path: overlapping
// revolves measured through the mesh intersection, an overlapping box pair
// measured analytically, disjoint boxes whose gap is measured, sheet against
// solid, and sheet against sheet.
func TestVerifyPairWorkersMatchOneWorker(t *testing.T) {
	t.Parallel()
	doc := New()
	internalFrustumBody(t, doc, r3.NewVec(0, 0, 0), r3.NewVec(0, 0, 6), 1.5, 1.2)
	internalFrustumBody(t, doc, r3.NewVec(-3, 0, 2), r3.NewVec(3, 0.4, 5), 1.2, 1.0)
	internalBoxBody(t, doc, 10, 0, 14, 4, 3)
	internalBoxBody(t, doc, 12, 2, 16, 6, 3)
	internalBoxBody(t, doc, 20, 0, 22, 2, 2)
	internalSheetBody(t, doc, 13, 1, 15, 3, 5)
	internalSheetBody(t, doc, 14, 2, 18, 5, 1)
	for _, opts := range [][]VerifyOption{nil, {WithClearances()}} {
		one, err := doc.Verify(withVerifyWorkers(t.Context(), 1), opts...)
		require.NoError(t, err)
		many, err := doc.Verify(withVerifyWorkers(t.Context(), 6), opts...)
		require.NoError(t, err)
		require.NotEmpty(t, one.Interferences)
		require.Equal(t, one, many)
	}
}
