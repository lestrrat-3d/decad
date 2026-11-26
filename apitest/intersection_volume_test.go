package apitest_test

// CI: isolated-tests

import (
	"context"
	"fmt"
	"sync"
	"testing"

	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"

	"github.com/lestrrat-3d/decad"
)

func TestIntersectionVolumeMatchesIntersect(t *testing.T) {
	t.Parallel()
	for _, tc := range allIntersectionVolumeCases() {
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
	for _, tc := range allIntersectionVolumeCases() {
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

			// Intersect on the queried document succeeds or is refused as it is on the twin that ran no query,
			// answers the same volume, and adds one operation to the history. TestIntersectionVolumeLaterOperations
			// compares the later results in full: outcome, volume, bounds, faces, face origins and tessellation.
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
	_, want := decad.Intersect(t.Context(), nil, b)
	require.EqualError(t, err, want.Error())
	_, err = doc.IntersectionVolume(t.Context(), a, nil)
	require.ErrorIs(t, err, decad.ErrDegenerate)
	_, want = decad.Intersect(t.Context(), a, nil)
	require.EqualError(t, err, want.Error())
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

// TestIntersectionVolumeSharedOperandsConcurrent runs queries that share operands, and repeats one pair, beside
// Verify, Tessellate and the body readers, and requires every answer to equal the serial one.
func TestIntersectionVolumeSharedOperandsConcurrent(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	base := circlePrismAtZ(t, doc, 10, 10, 6, 0, 10)
	mesh := roundTrip(t, boxBodyAtZ(t, doc, 0, 0, 10, 10, 1, 10))
	tools := []*decad.Body{
		boxBodyAtZ(t, doc, 5, 5, 15, 15, 2, 10),
		circlePrismAtZ(t, doc, 14, 10, 3, -1, 12),
		roundTrip(t, boxBodyAtZ(t, doc, 4, 4, 14, 14, 3, 10)),
		boxBody(t, doc, 100, 100, 110, 110, 10),
		mesh,
	}
	var pairs [][2]*decad.Body
	for _, tool := range tools {
		pairs = append(pairs, [2]*decad.Body{base, tool}, [2]*decad.Body{tool, base})
	}
	pairs = append(pairs, [2]*decad.Body{mesh, tools[0]}, [2]*decad.Body{tools[2], mesh})
	want := make([]string, len(pairs))
	for i, p := range pairs {
		want[i] = outcomeKey(doc.IntersectionVolume(t.Context(), p[0], p[1]))
	}

	var wg sync.WaitGroup
	var mu sync.Mutex
	var bad []string
	check := func(label string, p [2]*decad.Body, want string) {
		if got := outcomeKey(doc.IntersectionVolume(t.Context(), p[0], p[1])); got != want {
			mu.Lock()
			bad = append(bad, label)
			mu.Unlock()
		}
	}
	for round := range 3 {
		for i, p := range pairs {
			wg.Go(func() { check(fmt.Sprintf("round %d pair %d", round, i), p, want[i]) })
		}
		// The same pair four more times at once.
		for range 4 {
			wg.Go(func() { check("repeated pair", pairs[0], want[0]) })
		}
	}
	errs := make(chan error, 4)
	wg.Go(func() {
		for range 2 {
			if _, err := doc.Verify(t.Context()); err != nil {
				errs <- err
				return
			}
		}
	})
	wg.Go(func() {
		for range 4 {
			for _, b := range doc.Bodies() {
				if _, err := b.Tessellate(t.Context(), units.Millimeters(0.5)); err != nil {
					errs <- err
					return
				}
				_, _ = b.Volume()
				_, _ = b.Bounds()
			}
		}
	})
	wg.Wait()
	close(errs)
	for err := range errs {
		require.NoError(t, err)
	}
	require.Empty(t, bad, "each concurrent query answers as the serial one did")
}

// TestIntersectionVolumeLaterOperations runs two queries per operand order, then Intersect, Union and Cut, and
// requires each later result to match the twin document that ran no query: outcome, volume bits, bounds, face
// count, tessellation bits and the operation history's length.
func TestIntersectionVolumeLaterOperations(t *testing.T) {
	t.Parallel()
	for _, tc := range intersectionVolumeCases() {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			doc, a, b := tc.build(t)
			twin, ta, tb := tc.build(t)
			for range 2 {
				_, _ = doc.IntersectionVolume(t.Context(), a, b)
				_, _ = doc.IntersectionVolume(t.Context(), b, a)
			}
			got := laterOperations(t, doc, a, b)
			want := laterOperations(t, twin, ta, tb)
			require.Equal(t, want, got)
		})
	}
}
