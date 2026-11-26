package apitest_test

// CI: isolated-tests

import (
	"context"
	"sync"
	"testing"

	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"

	"github.com/lestrrat-3d/decad"
)

func TestIntersectionVolumeMatchesIntersect(t *testing.T) {
	t.Parallel()
	for _, tc := range intersectionVolumeCases() {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			docQ, aq, bq := tc.build(t)
			_, ai, bi := tc.build(t)
			queried, qerr := docQ.IntersectionVolume(t.Context(), aq, bq)
			got, ierr := decad.Intersect(t.Context(), ai, bi)
			requireOutcomeKind(t, tc.outcome, ierr)
			requireSameOutcome(t, queried, qerr, got, ierr)
		})
	}
}

func TestIntersectionVolumeCommitsNothing(t *testing.T) {
	t.Parallel()
	for _, tc := range intersectionVolumeCases() {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			doc, a, b := tc.build(t)
			twin, ta, tb := tc.build(t)
			bodies := doc.Bodies()
			ops := len(doc.OperationHistory().Operations())
			ca, err := doc.ContainerOf(a)
			require.NoError(t, err)
			cb, err := doc.ContainerOf(b)
			require.NoError(t, err)
			inA, inB := ca.Bodies(), cb.Bodies()

			_, _ = doc.IntersectionVolume(t.Context(), a, b)
			_, _ = doc.IntersectionVolume(t.Context(), b, a)

			require.Equal(t, bodies, doc.Bodies())
			require.Len(t, doc.OperationHistory().Operations(), ops)
			ca2, err := doc.ContainerOf(a)
			require.NoError(t, err)
			cb2, err := doc.ContainerOf(b)
			require.NoError(t, err)
			require.Same(t, ca, ca2)
			require.Same(t, cb, cb2)
			require.Equal(t, inA, ca.Bodies())
			require.Equal(t, inB, cb.Bodies())

			// The next operation numbers and builds as it does in a document that ran no query.
			got, err := decad.Intersect(t.Context(), a, b)
			want, werr := decad.Intersect(t.Context(), ta, tb)
			require.Equal(t, werr == nil, err == nil)
			if err != nil {
				return
			}
			gv, err := got.Volume()
			require.NoError(t, err)
			wv, err := want.Volume()
			require.NoError(t, err)
			requireSameMeasurement(t, wv, gv)
			require.Len(t, doc.OperationHistory().Operations(), ops+1)
			require.Len(t, twin.OperationHistory().Operations(), ops+1)
		})
	}
}

func TestIntersectionVolumeAcrossContainers(t *testing.T) {
	t.Parallel()
	for _, tc := range intersectionVolumeCases() {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			doc, a, b := tc.build(t)
			twin, ta, tb := tc.build(t)
			child, err := doc.Root().NewChild("tools")
			require.NoError(t, err)
			require.NoError(t, child.AddBody(b))
			queried, qerr := doc.IntersectionVolume(t.Context(), a, b)

			// Intersect refuses the split pair, and answers once the bodies share a container.
			_, err = decad.Intersect(t.Context(), a, b)
			require.ErrorIs(t, err, decad.ErrCrossContainer)
			tchild, err := twin.Root().NewChild("tools")
			require.NoError(t, err)
			require.NoError(t, tchild.AddBody(tb))
			require.NoError(t, twin.Root().AddBody(tb))
			got, ierr := decad.Intersect(t.Context(), ta, tb)
			requireSameOutcome(t, queried, qerr, got, ierr)
		})
	}
}

func TestIntersectionVolumeGates(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	a, b := boxBody(t, doc, 0, 0, 10, 10, 10), boxBody(t, doc, 5, 5, 15, 15, 10)
	other := decad.New()
	foreign := boxBody(t, other, 0, 0, 10, 10, 10)

	var nilCtx context.Context
	_, err := doc.IntersectionVolume(nilCtx, a, b)
	require.ErrorIs(t, err, decad.ErrDegenerate)
	_, err = doc.IntersectionVolume(t.Context(), nil, b)
	require.ErrorIs(t, err, decad.ErrDegenerate)
	_, err = doc.IntersectionVolume(t.Context(), a, nil)
	require.ErrorIs(t, err, decad.ErrDegenerate)
	_, err = doc.IntersectionVolume(t.Context(), a, a)
	require.ErrorIs(t, err, decad.ErrDegenerate)
	_, err = doc.IntersectionVolume(t.Context(), a, foreign)
	require.ErrorIs(t, err, decad.ErrForeignBody)
	_, err = doc.IntersectionVolume(t.Context(), foreign, b)
	require.ErrorIs(t, err, decad.ErrForeignBody)

	canceled, cancel := context.WithCancel(t.Context())
	cancel()
	_, err = doc.IntersectionVolume(canceled, a, b)
	require.Same(t, context.Canceled, err)

	_, err = decad.Intersect(t.Context(), a, b)
	require.NoError(t, err)
	_, err = doc.IntersectionVolume(t.Context(), a, b)
	require.ErrorIs(t, err, decad.ErrRetiredBody)
}

func TestIntersectionVolumeConcurrent(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	var pairs [][2]*decad.Body
	for i := range 8 {
		d := float64(i)
		pairs = append(pairs, [2]*decad.Body{
			boxBodyAtZ(t, doc, 0, 0, 10, 10, 0, 10),
			roundTrip(t, boxBodyAtZ(t, doc, 2+d/2, 3, 12+d, 13, 4, 10)),
		})
	}
	want := make([]decad.Measurement, len(pairs))
	for i, p := range pairs {
		m, err := doc.IntersectionVolume(t.Context(), p[0], p[1])
		require.NoError(t, err)
		want[i] = m
	}

	var wg sync.WaitGroup
	errs := make(chan error, 64)
	got := make([]decad.Measurement, len(pairs))
	for i, p := range pairs {
		wg.Go(func() {
			m, err := doc.IntersectionVolume(t.Context(), p[0], p[1])
			got[i] = m
			errs <- err
		})
	}
	wg.Go(func() {
		for range 4 {
			if _, err := doc.Verify(t.Context()); err != nil {
				errs <- err
				return
			}
		}
	})
	wg.Go(func() {
		for range 8 {
			for _, b := range doc.Bodies() {
				if _, err := b.Tessellate(t.Context(), units.Millimeters(1)); err != nil {
					errs <- err
					return
				}
			}
		}
	})
	wg.Wait()
	close(errs)
	for err := range errs {
		require.NoError(t, err)
	}
	for i := range pairs {
		requireSameMeasurement(t, want[i], got[i])
	}
}
