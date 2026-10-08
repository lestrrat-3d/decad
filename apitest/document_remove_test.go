package apitest_test

import (
	"sync"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// Document.Remove is retirement by the caller's own hand (core §6): the body
// leaves Bodies() and Verify, stays readable, and no operation takes it.

func TestDocumentRemoveTakesBodyOutOfTheModel(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	keep := boxBody(t, doc, 0, 0, 10, 10, 10)
	// A second cube sharing a 5 mm corner block with the first: Verify proves
	// the overlap, so the model fails until the intruder is removed.
	intruder := translated(t, boxBody(t, doc, 0, 0, 10, 10, 10), 5, 5, 5)

	report, err := doc.Verify(t.Context())
	require.NoError(t, err)
	require.False(t, report.Passed(), `two overlapping cubes must fail verification`)
	require.Len(t, report.Interferences, 1)

	volBefore, err := intruder.Volume()
	require.NoError(t, err)
	boxBefore, err := intruder.Bounds()
	require.NoError(t, err)
	planarBefore, err := decad.Faces(decad.Planar()).SelectFaces(intruder)
	require.NoError(t, err)
	originBefore := intruder.Origin()

	require.NoError(t, doc.Remove(intruder))
	require.Equal(t, []*decad.Body{keep}, doc.Bodies(), `the removed body leaves the live set, the other stays`)

	report, err = doc.Verify(t.Context())
	require.NoError(t, err)
	require.True(t, report.Passed(), `the model without the intruder is sound`)
	require.Equal(t, decad.Sound, report.Status)
	require.Len(t, report.Bodies, 1)
	require.Empty(t, report.Interferences)
	_, err = report.ForBody(intruder)
	require.ErrorIs(t, err, decad.ErrBodyReportNotFound, `Verify never reports on a removed body`)
	kept, err := report.ForBody(keep)
	require.NoError(t, err)
	require.Equal(t, decad.Sound, kept.Status)

	// A removed body is a retired body: its geometry, selectors and
	// provenance still answer, unchanged.
	volAfter, err := intruder.Volume()
	require.NoError(t, err)
	require.Equal(t, volBefore, volAfter)
	require.Equal(t, 1000.0, volumeMM(t, volAfter))
	boxAfter, err := intruder.Bounds()
	require.NoError(t, err)
	require.Equal(t, boxBefore, boxAfter)
	require.Equal(t, r3.NewVec(5, 5, 5), boxAfter.Min)
	require.Equal(t, r3.NewVec(15, 15, 15), boxAfter.Max)
	planarAfter, err := decad.Faces(decad.Planar()).SelectFaces(intruder)
	require.NoError(t, err)
	require.Len(t, planarAfter, len(planarBefore))
	require.Len(t, planarAfter, 6)
	require.Equal(t, originBefore, intruder.Origin())
	require.Same(t, doc, intruder.Document(), `removal changes membership, not ownership`)

	// ...and no operation takes it, so nothing returns it to the model.
	_, err = decad.Union(t.Context(), keep, intruder)
	require.ErrorIs(t, err, decad.ErrRetiredBody)
	_, err = intruder.Duplicate(t.Context())
	require.ErrorIs(t, err, decad.ErrRetiredBody)
	_, err = intruder.Placed(t.Context(), r3.Identity())
	require.ErrorIs(t, err, decad.ErrRetiredBody)
	require.Equal(t, []*decad.Body{keep}, doc.Bodies())
}

func TestDocumentRemoveRefusals(t *testing.T) {
	t.Parallel()

	t.Run("nil body", func(t *testing.T) {
		t.Parallel()
		doc := decad.New()
		live := boxBody(t, doc, 0, 0, 10, 10, 10)
		require.ErrorIs(t, doc.Remove(nil), decad.ErrDegenerate)
		require.Equal(t, []*decad.Body{live}, doc.Bodies())
	})

	t.Run("nil document", func(t *testing.T) {
		t.Parallel()
		var doc *decad.Document
		body := boxBody(t, decad.New(), 0, 0, 10, 10, 10)
		require.ErrorIs(t, doc.Remove(body), decad.ErrDegenerate)
	})

	t.Run("foreign body", func(t *testing.T) {
		t.Parallel()
		doc, other := decad.New(), decad.New()
		live := boxBody(t, doc, 0, 0, 10, 10, 10)
		foreign := boxBody(t, other, 0, 0, 10, 10, 10)
		require.ErrorIs(t, doc.Remove(foreign), decad.ErrForeignBody)
		require.Equal(t, []*decad.Body{live}, doc.Bodies())
		require.Equal(t, []*decad.Body{foreign}, other.Bodies(), `the owner keeps its body`)
	})

	t.Run("removed twice", func(t *testing.T) {
		t.Parallel()
		doc := decad.New()
		live := boxBody(t, doc, 0, 0, 10, 10, 10)
		gone := boxBody(t, doc, 20, 0, 30, 10, 10)
		require.NoError(t, doc.Remove(gone))
		require.ErrorIs(t, doc.Remove(gone), decad.ErrRetiredBody)
		require.Equal(t, []*decad.Body{live}, doc.Bodies())
	})

	t.Run("consumed by a feature", func(t *testing.T) {
		t.Parallel()
		doc := decad.New()
		source := boxBody(t, doc, 0, 0, 10, 10, 10)
		moved := translated(t, source, 20, 0, 0)
		require.ErrorIs(t, doc.Remove(source), decad.ErrRetiredBody, `Placed already retired its receiver`)
		require.Equal(t, []*decad.Body{moved}, doc.Bodies())
	})
}

// A body built from the removed one — a copy, and a boolean over that copy —
// keeps its geometry, its provenance and its place in the model.
func TestDocumentRemoveLeavesDerivedBodiesUnaffected(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	source := boxBody(t, doc, 0, 0, 10, 10, 10)
	shift, err := r3.Translation(r3.NewVec(30, 0, 0))
	require.NoError(t, err)
	instance, err := source.PlacedCopy(t.Context(), shift)
	require.NoError(t, err)
	partner := translated(t, boxBody(t, doc, 0, 0, 10, 10, 10), 35, 5, 5)
	merged, err := decad.Union(t.Context(), instance, partner)
	require.NoError(t, err)
	require.Equal(t, []*decad.Body{source, merged}, doc.Bodies())

	volBefore, err := merged.Volume()
	require.NoError(t, err)
	boxBefore, err := merged.Bounds()
	require.NoError(t, err)
	facesBefore := len(merged.Faces())
	originBefore := merged.Origin()

	require.NoError(t, doc.Remove(source))
	require.Equal(t, []*decad.Body{merged}, doc.Bodies())

	volAfter, err := merged.Volume()
	require.NoError(t, err)
	require.Equal(t, volBefore, volAfter)
	require.Equal(t, decad.Exact, volAfter.Exactness)
	require.Equal(t, 1875.0, volumeMM(t, volAfter), `1000 + 1000 - 5^3`)
	boxAfter, err := merged.Bounds()
	require.NoError(t, err)
	require.Equal(t, boxBefore, boxAfter)
	require.Equal(t, r3.NewVec(30, 0, 0), boxAfter.Min)
	require.Equal(t, r3.NewVec(45, 15, 15), boxAfter.Max)
	require.Len(t, merged.Faces(), facesBefore)
	require.Equal(t, originBefore, merged.Origin())

	report, err := doc.Verify(t.Context())
	require.NoError(t, err)
	require.True(t, report.Passed())
	require.Len(t, report.Bodies, 1)

	// The derived body is still an operand: removal touched only the source.
	far := boxBody(t, doc, 100, 0, 110, 10, 10)
	grown, err := decad.Union(t.Context(), merged, far)
	require.NoError(t, err)
	grownVol, err := grown.Volume()
	require.NoError(t, err)
	require.Equal(t, 2875.0, volumeMM(t, grownVol))
}

// Remove and Verify share the guarded live set: run under -race, any
// unguarded read or write of it fails this test. Every report must be one
// consistent snapshot — sound, and holding only the document's own bodies.
func TestDocumentRemoveConcurrentWithVerify(t *testing.T) {
	t.Parallel()
	const count = 6
	doc := decad.New()
	all := make(map[*decad.Body]struct{}, count)
	order := make([]*decad.Body, 0, count)
	for i := range count {
		x := float64(20 * i)
		b := boxBody(t, doc, x, 0, x+10, 10, 10)
		all[b] = struct{}{}
		order = append(order, b)
	}

	var wg sync.WaitGroup
	done := make(chan struct{})
	// ready opens once the Verify reader is running, so the removals below
	// always overlap at least one Verify call.
	ready := make(chan struct{})
	readerErrs := make(chan error, 2)
	type snapshot struct {
		passed bool
		bodies []*decad.Body
	}
	reports := make(chan snapshot, 1024)
	wg.Add(2)
	go func() {
		defer wg.Done()
		close(ready)
		// done is checked at the END of each iteration so every reader
		// publishes at least one snapshot, even when the removals finish
		// before the scheduler first runs it.
		for {
			r, err := doc.Verify(t.Context())
			if err != nil {
				readerErrs <- err
				return
			}
			s := snapshot{passed: r.Passed()}
			for _, br := range r.Bodies {
				s.bodies = append(s.bodies, br.Body)
			}
			select {
			case reports <- s:
			default:
			}
			select {
			case <-done:
				return
			default:
			}
		}
	}()
	go func() {
		defer wg.Done()
		// Same do-while shape as the Verify reader: done is checked last.
		for {
			select {
			case reports <- snapshot{passed: true, bodies: doc.Bodies()}:
			default:
			}
			select {
			case <-done:
				return
			default:
			}
		}
	}()

	// Remove all but the last body while both readers run.
	<-ready
	for _, b := range order[:count-1] {
		require.NoError(t, doc.Remove(b))
	}
	close(done)
	wg.Wait()
	close(readerErrs)
	close(reports)
	for err := range readerErrs {
		require.NoError(t, err)
	}
	seen := 0
	for s := range reports {
		seen++
		require.True(t, s.passed, `disjoint cubes always verify sound`)
		require.NotEmpty(t, s.bodies)
		require.LessOrEqual(t, len(s.bodies), count)
		for _, b := range s.bodies {
			require.Contains(t, all, b, `a snapshot holds only the document's own bodies`)
		}
	}
	require.Positive(t, seen)

	require.Equal(t, []*decad.Body{order[count-1]}, doc.Bodies())
	report, err := doc.Verify(t.Context())
	require.NoError(t, err)
	require.True(t, report.Passed())
	require.Len(t, report.Bodies, 1)
	require.Same(t, order[count-1], report.Bodies[0].Body)
}
