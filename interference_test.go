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

// TestVerifyAdmittedCoplanarPrismPairResolvesAnalytically confirms
// docs/prism-boolean-design.md §14 PR4: an admitted coplanar, co-directional
// prism pair now resolves its interference volume through the same
// read-only analytic OpIntersect dispatch performBoolean uses
// (evaluateAnalyticIntersect), rather than always falling to the read-only
// mesh path. The container (0,0)-(20,20) height 10 and the nested prism
// (5,5)-(9,10) height 15 share the container's own coplanar base plane, so
// their overlap is exactly the nested prism's footprint (4x5 = 20 mm^2)
// over the z-interval the two heights share ([0,10], since the nested prism
// is taller and pokes through the container's cap): 20 * 10 = 200 mm^3. The
// nested prism's cap coincides with the container's own boundary, so the
// pair is not a strict full containment (clearance §4's strictly-positive
// clearance requirement fails) and previously reached the mesh path's
// coplanar-tangent refusal (unsupported_pair / unsupported_pair_contact;
// verify_diagnostics_test.go pins the diagnostic side of this same
// fixture).
func TestVerifyAdmittedCoplanarPrismPairResolvesAnalytically(t *testing.T) {
	doc := decad.New()
	container := boxBody(t, doc, 0, 0, 20, 20, 10)
	nested := boxBody(t, doc, 5, 5, 9, 10, 15)
	before := snapshotDocument(t, doc)

	report, err := doc.Verify(t.Context())
	require.NoError(t, err)
	require.Equal(t, decad.Interfering, report.Status)
	require.Len(t, report.Interferences, 1)
	row := report.Interferences[0]
	require.Same(t, container, row.A)
	require.Same(t, nested, row.B)
	require.InDelta(t, 200.0, row.Volume.Value.Base(), 1e-9)
	require.Equal(t, decad.Exact, row.Volume.Exactness)
	require.Zero(t, row.Volume.Bound.Base())
	requireDocumentUnchanged(t, doc, before)
}

// TestVerifyAdmittedDisplacedPrismPairCarriesItsOwnBound pins the analytic
// row's other arm: the row publishes the resolved intersection payload's own
// measurement, so an operand whose section carries a displacement
// (docs/prism-boolean-design.md §7) makes the row Approximate rather than
// Exact, and docs/interference-design.md §8's composed displacement is the
// bound it reports.
//
// Operand A is the analytic Union of a [0,10]² prism with a [2,8]² prism
// drawn 1e12 mm away and placed back over it: B lies strictly inside A, so
// the merged region is exactly the [0,10]² prism (1000 mm³), but the
// re-expression at that magnitude leaves the section a displacement of its
// own. Operand B is a [-1,11]² prism of height 15 sharing the same base
// plane, so the true overlap is the whole of the merged body. That shared
// plane is the coplanar contact the mesh classifier refuses (§5.2), so a
// published row can only have come from the analytic dispatch.
func TestVerifyAdmittedDisplacedPrismPairCarriesItsOwnBound(t *testing.T) {
	doc := decad.New()
	const shift = 1e12
	lo, hi := 2-shift, 8-shift
	merged, err := decad.Union(
		boxBody(t, doc, 0, 0, 10, 10, 10),
		placedFar(t, boxBody(t, doc, lo, 2, hi, 8, 10), shift),
	)
	require.NoError(t, err)
	require.False(t, anyFaceIsFaceted(merged), "the analytic reduction must own the union")
	own, err := merged.Volume()
	require.NoError(t, err)
	require.Equal(t, decad.Approximate, own.Exactness)
	require.Positive(t, own.Bound.Base(), "the displaced section must publish a bound of its own")

	outer := boxBody(t, doc, -1, -1, 11, 11, 15)
	before := snapshotDocument(t, doc)

	report, err := doc.Verify(t.Context())
	require.NoError(t, err)
	require.Equal(t, decad.Interfering, report.Status)
	require.Len(t, report.Interferences, 1)
	row := report.Interferences[0]
	require.Same(t, merged, row.A)
	require.Same(t, outer, row.B)
	// The row is the resolved intersection payload's own measurement
	// (docs/interference-design.md §6: "Copy the measurement unchanged into
	// the row"), not a republication of an operand's — only §4 containment
	// and §4.1 equality reuse an operand's measurement verbatim, and §4.1
	// withholds for a record carrying exactly this displacement. Here the
	// intersection is the nested operand A itself, so the two measurements
	// state the same value and exactness; the bound is A's own displacement
	// plus this resolution's own charge under an outward-rounded sum
	// (prism_boolean_nesting.go's sectionDelta), so it can only be at least
	// as wide as A's. A row that dropped the displacement charge would fall
	// below it.
	require.Equal(t, own.Value, row.Volume.Value, "the intersection of A with a body containing A is A's own volume")
	require.GreaterOrEqual(t, row.Volume.Bound.Base(), own.Bound.Base(),
		"the row's own bound composes A's displacement and cannot be tighter than A's")
	require.Equal(t, own.Exactness, row.Volume.Exactness)
	require.Equal(t, decad.Approximate, row.Volume.Exactness, "a displaced section cannot publish an Exact row")
	require.InDelta(t, 1000.0, row.Volume.Value.Base(), 1e-9)

	// The truth the interval must contain: the merged region is the union the
	// two recorded sections denote under their own exact placements, and the
	// taller [-1,11]² prism contains all of it.
	truth := ratFloat(trueBoxUnionVolume(
		[4]float64{0, 0, 10, 10},
		[4]float64{lo + shift, 2, hi + shift, 8},
		10,
	))
	value, bound := row.Volume.Value.Base(), row.Volume.Bound.Base()
	require.LessOrEqual(t, value-bound, truth, "the proven interval must contain the true overlap")
	require.GreaterOrEqual(t, value+bound, truth)
	require.Positive(t, value-bound, "§6's positive-volume gate still holds")

	// A bound this coarse cannot pass the tolerance gate, so the report says
	// so rather than presenting the row as trustworthy.
	require.False(t, report.Trustworthy())
	var overlap *decad.Diagnostic
	for i, d := range report.Diagnostics {
		if d.Code == decad.DiagMeasurementBeyondTolerance && d.Reading == decad.ReadingOverlapVolume {
			overlap = &report.Diagnostics[i]
		}
	}
	require.NotNil(t, overlap, "the overlap reading is judged against the tolerance like any other")
	require.Equal(t, decad.Suspect, overlap.Status)
	require.NotNil(t, overlap.Pair)
	require.NotNil(t, overlap.Observed)
	require.Equal(t, row.Volume, *overlap.Observed, "the diagnostic reports the row it judged")
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

	// The two revolve balls overlap and both tessellate; at the chord tolerance
	// this read-only check derives from the pair, the revolve mesh crosses its
	// facet ceiling, so the pair stages (booleanExpectedStaging). Per
	// verification §1.1 that is a DiagUnsupportedPairPayload — a capability
	// limit — not a contact, in-pipeline, or unresolved-partition diagnostic.
	requireDiagnosticInvariants(t, report)
	d, ok := findDiagnostic(report.Diagnostics, decad.DiagUnsupportedPairPayload)
	require.True(t, ok, `a staged revolve operand emits its cause-specific code`)
	require.Equal(t, decad.Suspect, d.Status)
	require.Equal(t, decad.ReadingNone, d.Reading, `an unsupported pair names no reading`)
	require.Nil(t, d.Body)
	require.NotNil(t, d.Pair, `an unsupported pair names its pair`)
	require.Contains(t, d.Message, `first operand`, `the message names the operand that failed tessellation`)
	require.Contains(t, d.Message, "tessellation refused at the chord tolerance", "the message names the real cause")

	_, undecided := findDiagnostic(report.Diagnostics, decad.DiagUndecidedPair)
	require.False(t, undecided, `a staged revolve operand is not an undecided partition`)
	legacy, broad := findDiagnostic(report.Diagnostics, decad.DiagUnsupportedPair)
	require.True(t, broad, `an overlapping revolve pair preserves the broad compatibility code`)
	require.Equal(t, decad.Suspect, legacy.Status)
	require.Equal(t, decad.ReadingNone, legacy.Reading)
	require.NotNil(t, legacy.Pair)
	_, contact := findDiagnostic(report.Diagnostics, decad.DiagUnsupportedPairContact)
	require.False(t, contact, `payload staging is not a contact refusal`)
	_, pipeline := findDiagnostic(report.Diagnostics, decad.DiagUnsupportedPairPipeline)
	require.False(t, pipeline, `payload staging is not an in-pipeline reach`)
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

// This file pins docs/prism-boolean-design.md §3.4's split-boundary reroute
// as it reaches Verify's interference reading (§4.5, shared with the
// crossing sub-case's own selection, prism_boolean_crossing.go's
// resolvePrismCrossingCells): a coplanar pair whose 2D outlines genuinely
// cross is measured only while none of the reroute's three independent
// causes fires — a section displacement on either operand, a walk charge on
// either operand, or a nonidentity re-expression between the two frames.
// Each cause is pinned on its own here, because each alone is enough to
// reroute the pair to the mesh path, whose own coplanar contact refusal
// (docs/interference-design.md §5.2) then leaves it undecided.
//
// crossingBoxPair builds box A [0,10]x[0,10] against box B [5,15]x[5,15],
// both 5 mm tall: a single 5x5 mm crossing overlap over the shared 5 mm
// height, 125 mm^3. seated draws B directly at its final coordinates; placed
// builds B at the origin footprint and moves it into place with Body.Placed,
// reaching the identical footprint through a nonidentity re-expression;
// placedBackToIdentity moves B twice, by a motion and its inverse, so the
// ACCUMULATED placement the re-expression actually reads is the identity
// again even though the last Placed call received a nonidentity transform.
//
// splitCellBody is the walk-charge half: the SAME [1,5]x[0,10] footprint a
// plain rectangle draws, but cut out of the larger rectangle [1,11]x[0,10] by
// a line through (5,-2)-(5,14), so its bottom and top walls are Partial
// fragments whose narrowed range makes buildPrismScene compute their shared
// corner instead of reading it off the record (§7's walk charge). Nothing is
// placed and nothing is displaced: the pair is fully seated and reroutes on
// the walk charge alone.

func crossingBoxPairSeated(t *testing.T, doc *decad.Document) (a, b *decad.Body) {
	t.Helper()
	a = boxBody(t, doc, 0, 0, 10, 10, 5)
	b = boxBody(t, doc, 5, 5, 15, 15, 5)
	return a, b
}

func crossingBoxPairPlaced(t *testing.T, doc *decad.Document) (a, b *decad.Body) {
	t.Helper()
	a = boxBody(t, doc, 0, 0, 10, 10, 5)
	origin := boxBody(t, doc, 0, 0, 10, 10, 5)
	b = translated(t, origin, 5, 5, 0)
	return a, b
}

func crossingBoxPairPlacedBackToIdentity(t *testing.T, doc *decad.Document) (a, b *decad.Body) {
	t.Helper()
	a = boxBody(t, doc, 0, 0, 10, 10, 5)
	seated := boxBody(t, doc, 5, 5, 15, 15, 5)
	b = translated(t, translated(t, seated, 3, 7, 2), -3, -7, -2)
	return a, b
}

// splitCellBody extrudes the LEFT cell of the rectangle [1,11]x[0,10] split by
// the fixed line (5,-2)-(5,14), h mm along its own sketch normal. The cell's
// footprint is exactly [1,5]x[0,10], the same rectangle boxBody draws
// directly, so a pair built from this body differs from the plain-rectangle
// pair in provenance alone.
func splitCellBody(t *testing.T, doc *decad.Document, h float64) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	r := s.CreateRectangle(1, 0, 11, 10)
	s.Fix(r.A)
	lo := s.CreatePoint(5, -2)
	hi := s.CreatePoint(5, 14)
	s.Fix(lo)
	s.Fix(hi)
	s.CreateLine(lo, hi)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)

	var left *sketch.Profile
	for _, p := range s.Profiles() {
		minU, maxU := math.Inf(1), math.Inf(-1)
		for _, e := range p.Outer {
			for _, pt := range e.Polyline {
				minU = math.Min(minU, pt[0])
				maxU = math.Max(maxU, pt[0])
			}
		}
		if minU == 1 && maxU <= 5.0000001 {
			left = p
		}
	}
	require.NotNil(t, left, "the split rectangle's left cell must exist")

	body, err := doc.Extrude(s, left, decad.Distance{D: units.Millimeters(h), Dir: decad.Along})
	require.NoError(t, err)
	return body
}

func TestVerifyCrossingPairSeatedResolvesPlacedStaysUndecided(t *testing.T) {
	t.Run("seated section resolves through the analytic reading", func(t *testing.T) {
		doc := decad.New()
		a, b := crossingBoxPairSeated(t, doc)
		before := snapshotDocument(t, doc)

		report, err := doc.Verify(t.Context())
		require.NoError(t, err)
		require.Equal(t, decad.Interfering, report.Status)
		require.Len(t, report.Interferences, 1)

		row := report.Interferences[0]
		require.Same(t, a, row.A)
		require.Same(t, b, row.B)
		require.InDelta(t, 125.0, row.Volume.Value.Base(), 1e-9)
		require.Greater(t, row.Volume.Value.Base()-row.Volume.Bound.Base(), 0.0,
			"§6's positive-volume gate must hold")

		_, contact := findDiagnostic(report.Diagnostics, decad.DiagUnsupportedPairContact)
		require.False(t, contact, "a seated pair must never fall back to the mesh path's coplanar refusal")
		requireDocumentUnchanged(t, doc, before)
	})

	t.Run("placed section stays undecided", func(t *testing.T) {
		doc := decad.New()
		crossingBoxPairPlaced(t, doc)
		before := snapshotDocument(t, doc)

		report, err := doc.Verify(t.Context())
		require.NoError(t, err)
		require.Empty(t, report.Interferences,
			"a nonidentity re-expression reroutes the pair before either analytic path can measure it")
		require.Equal(t, decad.Suspect, report.Status)

		d, ok := findDiagnostic(report.Diagnostics, decad.DiagUnsupportedPairContact)
		require.True(t, ok, "the rerouted pair reaches the mesh path's own staged coplanar contact")
		require.Equal(t, decad.Suspect, d.Status)
		require.NotNil(t, d.Pair)

		_, broad := findDiagnostic(report.Diagnostics, decad.DiagUnsupportedPair)
		require.True(t, broad, "the placed pair keeps the broad compatibility code the seated pair clears")
		requireDocumentUnchanged(t, doc, before)
	})

	// §3.4 reads the operands' ACCUMULATED placement, never the motion one
	// Placed call received: newPrismReexpression compares pa.frame/pa.xform
	// against pb.frame/pb.xform, and PlacedContext composes onto the transform
	// the receiver already carries. So a body moved by a motion and then by its
	// inverse is back at the partner's own placement and the pair is measured
	// again, even though the last Placed call took a nonidentity transform.
	t.Run("a placement composed back to the identity is measured again", func(t *testing.T) {
		doc := decad.New()
		a, b := crossingBoxPairPlacedBackToIdentity(t, doc)
		before := snapshotDocument(t, doc)

		report, err := doc.Verify(t.Context())
		require.NoError(t, err)
		require.Equal(t, decad.Interfering, report.Status)
		require.Len(t, report.Interferences, 1,
			"a nonidentity motion composing back to the partner's placement leaves the analytic reading available")

		row := report.Interferences[0]
		require.Same(t, a, row.A)
		require.Same(t, b, row.B)
		require.InDelta(t, 125.0, row.Volume.Value.Base(), 1e-9)

		_, contact := findDiagnostic(report.Diagnostics, decad.DiagUnsupportedPairContact)
		require.False(t, contact, "the pair must never reach the mesh path's coplanar refusal")
		requireDocumentUnchanged(t, doc, before)
	})
}

// TestVerifySeatedWalkChargedPairStaysUndecided pins §3.4's walk-charge cause
// on its own: both operands are drawn seated in their own sketch, so the
// re-expression is the identity and no placement is ever applied, yet the pair
// still reroutes because operand A's section was trimmed out of a larger sketch
// arrangement and its consumed walls carry a walk charge (§7's delta_walk).
// The control arm draws the IDENTICAL footprint as a plain rectangle — no
// trim, no walk charge — and the same overlap is measured, so the difference
// between the two arms is the walk charge and nothing else. That this exact
// fixture yields a positive delta_walk under an identity re-expression is
// pinned white-box by TestPrismUnionTrimmedSourceSplitBoundaryFallsBack
// (prism_boolean_internal_test.go), over the same split-cell section.
func TestVerifySeatedWalkChargedPairStaysUndecided(t *testing.T) {
	const h = 5.0
	// [1,5]x[0,10] against [4,6]x[3,7]: one crossing region [4,5]x[3,7],
	// 1 x 4 mm^2 over the shared 5 mm height.
	const wantVolume = 20.0

	t.Run("an untrimmed seated pair of the same footprint is measured", func(t *testing.T) {
		doc := decad.New()
		a := boxBody(t, doc, 1, 0, 5, 10, h)
		b := boxBody(t, doc, 4, 3, 6, 7, h)
		before := snapshotDocument(t, doc)

		report, err := doc.Verify(t.Context())
		require.NoError(t, err)
		require.Equal(t, decad.Interfering, report.Status)
		require.Len(t, report.Interferences, 1)

		row := report.Interferences[0]
		require.Same(t, a, row.A)
		require.Same(t, b, row.B)
		require.InDelta(t, wantVolume, row.Volume.Value.Base(), 1e-9)
		requireDocumentUnchanged(t, doc, before)
	})

	t.Run("the walk-charged seated pair stays undecided", func(t *testing.T) {
		doc := decad.New()
		splitCellBody(t, doc, h)
		boxBody(t, doc, 4, 3, 6, 7, h)
		before := snapshotDocument(t, doc)

		report, err := doc.Verify(t.Context())
		require.NoError(t, err)
		require.Empty(t, report.Interferences,
			"a seated pair still reroutes on either operand's own walk charge")
		require.Equal(t, decad.Suspect, report.Status)

		d, ok := findDiagnostic(report.Diagnostics, decad.DiagUnsupportedPairContact)
		require.True(t, ok, "the rerouted pair reaches the mesh path's own staged coplanar contact")
		require.Equal(t, decad.Suspect, d.Status)
		requireDocumentUnchanged(t, doc, before)
	})
}
