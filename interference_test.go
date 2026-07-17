package decad_test

import (
	"context"
	"encoding/json"
	"sync"
	"sync/atomic"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

type documentSnapshot struct {
	bodies []*decad.Body
	recipe []byte
}

type cancelAfterContext struct {
	context.Context //nolint:containedctx // deterministic cancellation wrapper used only within one test call.
	limit           int32
	calls           atomic.Int32
	done            chan struct{}
	once            sync.Once
}

func newCancelAfterContext(parent context.Context, limit int32) *cancelAfterContext {
	return &cancelAfterContext{Context: parent, limit: limit, done: make(chan struct{})}
}

func (c *cancelAfterContext) Done() <-chan struct{} { return c.done }

func (c *cancelAfterContext) Err() error {
	if c.calls.Add(1) >= c.limit {
		c.once.Do(func() { close(c.done) })
		return context.Canceled
	}
	return nil
}

func snapshotDocument(t *testing.T, doc *decad.Document) documentSnapshot {
	t.Helper()
	recipe, err := json.Marshal(doc.Recipe())
	require.NoError(t, err)
	return documentSnapshot{bodies: doc.Bodies(), recipe: recipe}
}

func requireDocumentUnchanged(t *testing.T, doc *decad.Document, before documentSnapshot) {
	t.Helper()
	require.Equal(t, before.bodies, doc.Bodies(), `Verify must preserve live body membership and order`)
	recipe, err := json.Marshal(doc.Recipe())
	require.NoError(t, err)
	require.Equal(t, before.recipe, recipe, `Verify must not append an intersection step`)
}

func TestVerifyOffsetBoxesReportBoundedInterference(t *testing.T) {
	doc := decad.New()
	a := boxBody(t, doc, 0, 0, 10, 10, 10)
	b := translated(t, boxBody(t, doc, 0, 0, 10, 10, 10), 5, 5, 5)
	before := snapshotDocument(t, doc)

	report, err := doc.Verify(t.Context(), decad.WithClearances())
	require.NoError(t, err)
	require.Equal(t, decad.Interfering, report.Status)
	require.False(t, report.Trustworthy())
	require.Len(t, report.Interferences, 1)
	require.Empty(t, report.Clearances, `an overlapping pair cannot also carry a clearance row`)
	row := report.Interferences[0]
	require.Same(t, a, row.A)
	require.Same(t, b, row.B)
	require.Equal(t, 125.0, row.Volume.Value.Base())
	require.Greater(t, row.Volume.Value.Base()-row.Volume.Bound.Base(), 0.0)
	requireDocumentUnchanged(t, doc, before)
}

func TestVerifyStrictContainmentReusesInnerVolume(t *testing.T) {
	doc := decad.New()
	outer := boxBody(t, doc, 0, 0, 10, 10, 10)
	inner := translated(t, boxBody(t, doc, 2, 2, 4, 4, 2), 0, 0, 2)
	want, err := inner.Volume()
	require.NoError(t, err)
	before := snapshotDocument(t, doc)

	report, err := doc.Verify(t.Context(), decad.WithClearances())
	require.NoError(t, err)
	require.Equal(t, decad.Interfering, report.Status)
	require.Len(t, report.Interferences, 1)
	require.Empty(t, report.Clearances)
	row := report.Interferences[0]
	require.Same(t, outer, row.A)
	require.Same(t, inner, row.B)
	require.Equal(t, want, row.Volume, `containment keeps the contained body's complete measurement`)
	requireDocumentUnchanged(t, doc, before)
}

func TestVerifyCoincidentAnalyticBodiesUseStableFirstVolume(t *testing.T) {
	doc := decad.New()
	a := boxBody(t, doc, 0, 0, 10, 10, 10)
	b := boxBody(t, doc, 0, 0, 10, 10, 10)
	want, err := a.Volume()
	require.NoError(t, err)

	report, err := doc.Verify(t.Context())
	require.NoError(t, err)
	require.Equal(t, decad.Interfering, report.Status)
	require.Len(t, report.Interferences, 1)
	row := report.Interferences[0]
	require.Same(t, a, row.A)
	require.Same(t, b, row.B)
	require.Equal(t, want, row.Volume)
}

func TestVerifyInterferenceRowsKeepDocumentPairOrder(t *testing.T) {
	doc := decad.New()
	a := boxBody(t, doc, 0, 0, 10, 10, 10)
	b := translated(t, boxBody(t, doc, 0, 0, 10, 10, 10), 3, 3, 3)
	c := translated(t, boxBody(t, doc, 0, 0, 10, 10, 10), 6, 6, 6)

	report, err := doc.Verify(t.Context(), decad.WithMinWallThickness(units.Millimeters(20)))
	require.NoError(t, err)
	require.Equal(t, decad.Interfering, report.Status, `interference outranks per-body violations`)
	require.Len(t, report.Interferences, 3)
	require.Same(t, a, report.Interferences[0].A)
	require.Same(t, b, report.Interferences[0].B)
	require.Same(t, a, report.Interferences[1].A)
	require.Same(t, c, report.Interferences[1].B)
	require.Same(t, b, report.Interferences[2].A)
	require.Same(t, c, report.Interferences[2].B)

	repeated, err := doc.Verify(t.Context(), decad.WithMinWallThickness(units.Millimeters(20)))
	require.NoError(t, err)
	require.Equal(t, report.Interferences, repeated.Interferences)
}

func TestVerifyTinyPositiveOverlapClearsItsBound(t *testing.T) {
	doc := decad.New()
	boxBody(t, doc, 0, 0, 10, 10, 10)
	translated(t, boxBody(t, doc, 0, 0, 10, 10, 10), 1, 1, 10-1e-8)

	report, err := doc.Verify(t.Context())
	require.NoError(t, err)
	require.Equal(t, decad.Interfering, report.Status)
	require.Len(t, report.Interferences, 1)
	volume := report.Interferences[0].Volume
	require.Greater(t, volume.Value.Base()-volume.Bound.Base(), 0.0)
}

func TestVerifyTouchingAndDisjointPairsEmitNoInterference(t *testing.T) {
	t.Run("touching", func(t *testing.T) {
		doc := decad.New()
		boxBody(t, doc, 0, 0, 10, 10, 10)
		translated(t, boxBody(t, doc, 0, 0, 10, 10, 10), 10, 0, 0)
		report, err := doc.Verify(t.Context(), decad.WithClearances())
		require.NoError(t, err)
		require.Empty(t, report.Interferences)
		require.Len(t, report.Clearances, 1)
		require.Zero(t, report.Clearances[0].Gap.Value.Base())
	})

	t.Run("disjoint", func(t *testing.T) {
		doc := decad.New()
		boxBody(t, doc, 0, 0, 10, 10, 10)
		translated(t, boxBody(t, doc, 0, 0, 10, 10, 10), 20, 0, 0)
		report, err := doc.Verify(t.Context())
		require.NoError(t, err)
		require.Equal(t, decad.Sound, report.Status)
		require.Empty(t, report.Interferences)
	})
}

func TestVerifyUnsupportedOverlapStaysSuspectAndReadOnly(t *testing.T) {
	doc := decad.New()
	ballBody(t, doc, 10)
	ball := ballBody(t, doc, 8)
	shift, err := r3.Translation(r3.NewVec(5, 0, 0))
	require.NoError(t, err)
	_, err = ball.Placed(shift)
	require.NoError(t, err)
	before := snapshotDocument(t, doc)

	report, err := doc.Verify(t.Context())
	require.NoError(t, err)
	require.Equal(t, decad.Suspect, report.Status)
	require.Empty(t, report.Interferences)
	requireDocumentUnchanged(t, doc, before)
}

func TestVerifyCoarseBooleanTessellationStaysSuspect(t *testing.T) {
	doc := decad.New()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(0, 0, 100, 60)
	s.Fix(rect.A)
	// The hole is valid and separated from the outline by 0.001 mm. The
	// pair-derived boolean chord tolerance is wider, so tessellation cannot
	// prove the two chorded loops stay separate at that resolution.
	s.CreateCircle(s.CreatePoint(89.999, 30), 10)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	var profile *sketch.Profile
	for _, p := range s.Profiles() {
		if len(p.Holes) == 1 {
			profile = p
			break
		}
	}
	require.NotNil(t, profile)
	_, err = doc.Extrude(s, profile, decad.Distance{D: units.Millimeters(8), Dir: decad.Along})
	require.NoError(t, err)
	boxBody(t, doc, -1, 1, 1, 2, 4)
	before := snapshotDocument(t, doc)

	report, err := doc.Verify(t.Context())
	require.NoError(t, err)
	require.Equal(t, decad.Suspect, report.Status)
	require.Empty(t, report.Interferences)
	requireDocumentUnchanged(t, doc, before)
}

func TestVerifyCancellationLeavesDocumentUnchanged(t *testing.T) {
	doc := decad.New()
	boxBody(t, doc, 0, 0, 10, 10, 10)
	translated(t, boxBody(t, doc, 0, 0, 10, 10, 10), 5, 5, 5)
	before := snapshotDocument(t, doc)
	ctx, cancel := context.WithCancel(t.Context())
	cancel()

	report, err := doc.Verify(ctx)
	require.ErrorIs(t, err, context.Canceled)
	require.Nil(t, report)
	requireDocumentUnchanged(t, doc, before)
}

func TestVerifyCancellationInsideReadOnlyIntersection(t *testing.T) {
	doc := decad.New()
	boxBody(t, doc, 0, 0, 10, 10, 10)
	translated(t, boxBody(t, doc, 0, 0, 10, 10, 10), 5, 5, 5)
	before := snapshotDocument(t, doc)
	// The first checks cover body and analytic-pair phases. Cancellation then
	// lands at a read-only boolean phase boundary, after operand preparation.
	ctx := newCancelAfterContext(t.Context(), 12)

	report, err := doc.Verify(ctx)
	require.ErrorIs(t, err, context.Canceled)
	require.Nil(t, report)
	requireDocumentUnchanged(t, doc, before)
}
