package decad_test

import (
	"context"
	"encoding/json"
	"math"
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

// ringBody revolves the axis-aligned rectangle u[u0,u1] v[v0,v1] a full turn
// about the u axis: a tube of outer radius v1, inner radius v0, length u1-u0.
// Each hole is a second rectangle, revolved into a closed toroidal cavity —
// one lump whose inner shell is a void.
func ringBody(t *testing.T, doc *decad.Document, u0, v0, u1, v1 float64, holes ...[4]float64) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(u0, v0, u1, v1)
	s.Fix(rect.A)
	for _, h := range holes {
		s.CreateRectangle(h[0], h[1], h[2], h[3])
	}
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	prof := s.Profiles()[0]
	for _, p := range s.Profiles() {
		if len(p.Holes) == len(holes) {
			prof = p
			break
		}
	}
	body, err := doc.Revolve(s, prof, uAxis, decad.FullRevolution{})
	require.NoError(t, err)
	return body
}

// A void shell of the outer body lying inside the inner body cuts the inner
// body's material in two: membership in the outer body is NOT constant across
// the inner lump, so the inner body's own witness proves nothing about the
// part the cavity carves away. The full-containment certificate must fail here
// (§4) — reusing the contained body's volume would report the whole of A where
// the true overlap is A minus B's cavity.
func TestVerifyOuterVoidInsideInnerBodyDeniesContainment(t *testing.T) {
	doc := decad.New()
	// B: a tube carrying a closed toroidal cavity at radii 7..13, u 2..8.
	b := ringBody(t, doc, 0, 5, 10, 15, [4]float64{2, 7, 8, 13})
	// A: a plain tube whose whole boundary sits 1 mm inside B's wall, and
	// which wholly contains B's cavity.
	a := ringBody(t, doc, 1, 6, 9, 13.5)

	// The fixture is the shape the certificate must handle: one lump, two
	// shells, the inner one a void. A single-lump gate does not see it.
	require.Len(t, b.Lumps(), 1)
	shells := b.Lumps()[0].Shells()
	require.Len(t, shells, 2)
	require.False(t, shells[0].IsVoid())
	require.True(t, shells[1].IsVoid(), `the revolved hole is a closed cavity`)
	require.Len(t, a.Lumps()[0].Shells(), 1)

	volA, err := a.Volume()
	require.NoError(t, err)
	require.InDelta(t, math.Pi*(13.5*13.5-6*6)*8, volA.Value.Base(), 1e-9)
	// The true overlap: A minus B's cavity, which lies wholly inside A.
	wantOverlap := math.Pi*(13.5*13.5-6*6)*8 - math.Pi*(13*13-7*7)*6
	before := snapshotDocument(t, doc)

	report, err := doc.Verify(t.Context())
	require.NoError(t, err)
	for _, row := range report.Interferences {
		if row.A != b || row.B != a {
			continue
		}
		// A row is admissible only if its proven interval really holds the
		// overlap. A's whole volume, marked Exact, does not.
		require.InDelta(t, wantOverlap, row.Volume.Value.Base(), row.Volume.Bound.Base(),
			`a reported overlap must bound the true overlap, not the contained body's whole volume`)
	}
	if len(report.Interferences) == 0 {
		require.Equal(t, decad.Suspect, report.Status,
			`an overlap this evaluator cannot bound reads Suspect, never a silent pass`)
	}
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

	// The two revolve balls overlap, but neither can be tessellated, so the
	// read-only intersect stages the pair (booleanExpectedStaging). Per
	// verification §1.1 a staged revolve operand is a DiagUnsupportedPair — a
	// capability limit — not a DiagUndecidedPair (an unresolved partition).
	requireDiagnosticInvariants(t, report)
	d, ok := findDiagnostic(report.Diagnostics, decad.DiagUnsupportedPair)
	require.True(t, ok, `a staged revolve operand emits DiagUnsupportedPair`)
	require.Equal(t, decad.Suspect, d.Status)
	require.Equal(t, decad.ReadingNone, d.Reading, `an unsupported pair names no reading`)
	require.Nil(t, d.Body)
	require.NotNil(t, d.Pair, `an unsupported pair names its pair`)

	_, undecided := findDiagnostic(report.Diagnostics, decad.DiagUndecidedPair)
	require.False(t, undecided, `a staged revolve operand is not an undecided partition`)
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
