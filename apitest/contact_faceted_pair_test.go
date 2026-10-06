package apitest_test

import (
	"context"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// The fixtures of docs/multibody-dynamics-design.md §13 PR 10. Every
// coordinate is dyadic, and every pose is a float transform whose entries
// are read as exact dyadics, so each relation below is decided exactly.
//
// The kernel's certificate legs are shown to fail at the snapshot level in
// internal/pair/planar_test.go. At this level, each of these was shown to
// fail: removing the planar dispatch from ContactPair (every case reads
// Undecided); skipping the convexity lookup, or certifying every body convex
// (the hollow shell in the tray's corner loses ContactNonConvex); certifying
// no body convex (the hexagon's touch reads ContactNonConvex).

// placedBox is a source box over [x0, x1]×[y0, y1]×[z0, z0+h]: extruded from
// the XY plane and translated, which keeps its tessellation bound zero so
// a Union of such boxes is a zero-bound Boolean.
func placedBox(t *testing.T, doc *decad.Document, box [6]float64) *decad.Body {
	t.Helper()
	body := boxBody(t, doc, box[0], box[1], box[2], box[3], box[5])
	if box[4] == 0 {
		return body
	}
	placed, err := body.Placed(t.Context(), contactPose(t, r3.Vec{Z: box[4]}))
	require.NoError(t, err)
	return placed
}

// cutOf is the Boolean target minus tool over two placed boxes.
func cutOf(t *testing.T, doc *decad.Document, target, tool [6]float64) *decad.Body {
	t.Helper()
	body, err := decad.Cut(t.Context(), placedBox(t, doc, target), placedBox(t, doc, tool))
	require.NoError(t, err)
	return body
}

// trayBody is an open tray: the inside is [-35, 35]² above the floor's top
// face at z = 0, with walls rising to z = 30. Every crossing of the two
// operands' facets lands on a dyadic point, so the Boolean is exact
// (zero-bound), which §9 requires.
func trayBody(t *testing.T, doc *decad.Document) *decad.Body {
	t.Helper()
	return cutOf(t, doc, [6]float64{-40, -40, 40, 40, -10, 40}, [6]float64{-35, -35, 35, 35, 0, 40})
}

// hollowBody is a closed shell, [-20, 20]³ around the cavity [-15, 15]³:
// two shells, so its nesting reading takes a cast through both.
func hollowBody(t *testing.T, doc *decad.Document) *decad.Body {
	t.Helper()
	return cutOf(t, doc, [6]float64{-20, -20, 20, 20, -20, 40}, [6]float64{-15, -15, 15, 15, -15, 30})
}

// hexPrismBody is a hexagonal prism 20 mm across its flats and 12 mm tall,
// with one corner at the origin so that a rotation about the origin keeps
// that corner's exact position.
func hexPrismBody(t *testing.T, doc *decad.Document) *decad.Body {
	t.Helper()
	s, profile := polygonSketch(t, [][2]float64{
		{0, 0}, {5.75, -10}, {17.25, -10}, {23, 0}, {17.25, 10}, {5.75, 10},
	})
	body, err := doc.Extrude(s, profile, decad.Distance{D: units.Millimeters(12), Dir: decad.Along})
	require.NoError(t, err)
	return body
}

// hexPose tips the prism 37° about -y, which lifts every corner but the one
// at the origin, then puts that corner at the given point.
func hexPose(t *testing.T, at r3.Vec) r3.Transform {
	t.Helper()
	turn, err := r3.Rotation(r3.Vec{Y: -1}, units.Degrees(37))
	require.NoError(t, err)
	pose, err := r3.FromBasis(turn.Basis(), at)
	require.NoError(t, err)
	return pose
}

// contactBothOrders runs ContactPair in both orders and requires the same
// relation, gap and reason, and a manifold mirrored between them, as §9.5 and
// contact-geometry §5 state.
func contactBothOrders(t *testing.T, doc *decad.Document, a, b *decad.Body,
	poseA, poseB r3.Transform) *decad.ContactReport {
	t.Helper()
	forward := contactBothWays(t, doc, a, b, poseA, poseB, contactRequest())
	reversed, err := doc.ContactPair(t.Context(), b, a, poseB, poseA, contactRequest())
	require.NoError(t, err)
	require.Equal(t, forward.Gap, reversed.Gap)
	return forward
}

func TestContactPairExactPlanarHexagonOnTray(t *testing.T) {
	doc := decad.New()
	tray := trayBody(t, doc)
	hex := hexPrismBody(t, doc)
	before := doc.Bodies()
	id := r3.Identity()

	// The lowest corner 3 mm above the floor: the gap is that corner's
	// height, and every wall is farther away.
	gap := contactBothOrders(t, doc, tray, hex, id, hexPose(t, r3.Vec{X: -10, Z: 3}))
	require.Equal(t, decad.ContactSeparated, gap.Relation, "reason=%v", gap.Reason)
	require.NotNil(t, gap.Gap)
	require.LessOrEqual(t, gap.Gap.Value.Base()-gap.Gap.Bound.Base(), 3.0)
	require.GreaterOrEqual(t, gap.Gap.Value.Base()+gap.Gap.Bound.Base(), 3.0)
	// The enclosure may be a few ulps wide; 1e-12 mm is slack against
	// rounding differences between hosts, not a pinned figure.
	require.Less(t, gap.Gap.Bound.Base(), 1e-12)

	// The same corner on the floor: an exact vertex touch. The prism is
	// convex, so the floor's normal is the one admissible normal there
	// (§9.3), and the corner is the whole manifold.
	touch := contactBothOrders(t, doc, tray, hex, id, hexPose(t, r3.Vec{X: -10}))
	require.Equal(t, decad.ContactTouching, touch.Relation, "reason=%v", touch.Reason)
	require.Equal(t, decad.Exact, touch.Gap.Exactness)
	require.Zero(t, touch.Gap.Value.Base())
	require.Equal(t, decad.ContactNoReason, touch.Reason)
	require.NotNil(t, touch.Manifold)
	require.Len(t, touch.Manifold.Points, 1)
	require.Equal(t, r3.Vec{X: -10}, touch.Manifold.Points[0].OnA.Value)
	require.Equal(t, r3.Vec{Z: 1}, touch.Manifold.Points[0].Normal.Value)
	require.NotNil(t, touch.Manifold.Points[0].FeatureB.Vertex)

	// Half a millimetre lower, the corner's edges cross the floor's face
	// alone. The tray is not convex, so §9.3's patch does not apply, and the
	// face-local patch (§9.6) publishes the corner at depth with its foot on
	// the floor.
	crossing := contactBothOrders(t, doc, tray, hex, id, hexPose(t, r3.Vec{X: -10, Z: -0.5}))
	require.Equal(t, decad.ContactOverlapping, crossing.Relation, "reason=%v", crossing.Reason)
	require.Nil(t, crossing.Gap)
	require.NotNil(t, crossing.Manifold, "reason=%v", crossing.Reason)
	require.Len(t, crossing.Manifold.Points, 1)
	require.Equal(t, r3.Vec{X: -10}, crossing.Manifold.Points[0].OnA.Value)
	require.Equal(t, r3.Vec{X: -10, Z: -0.5}, crossing.Manifold.Points[0].OnB.Value)
	require.Equal(t, -0.5, crossing.Manifold.Points[0].Separation.Value.Base())
	require.Zero(t, crossing.Manifold.Points[0].Separation.Bound.Base())

	require.Equal(t, before, doc.Bodies(), "a contact query changes no document state")
}

func TestContactPairExactPlanarNesting(t *testing.T) {
	doc := decad.New()
	hollow := hollowBody(t, doc)
	id := r3.Identity()

	// Wholly inside the floor slab's material, no facet crossing anywhere:
	// only the nesting cast proves the overlap.
	inWall := placedBox(t, doc, [6]float64{-2, -2, 2, 2, -19, 2})
	nested := contactBothOrders(t, doc, hollow, inWall, id, id)
	require.Equal(t, decad.ContactOverlapping, nested.Relation, "reason=%v", nested.Reason)

	// Inside the cavity, the cast crosses both shells: separated from the
	// nearest cavity wall by 14 mm.
	inCavity := placedBox(t, doc, [6]float64{-1, -1, 1, 1, -1, 2})
	cavity := contactBothOrders(t, doc, hollow, inCavity, id, id)
	require.Equal(t, decad.ContactSeparated, cavity.Relation, "reason=%v", cavity.Reason)
	require.Equal(t, 14.0, cavity.Gap.Value.Base())
	require.Zero(t, cavity.Gap.Bound.Base())
}

func TestContactPairExactPlanarNonConvex(t *testing.T) {
	doc := decad.New()
	tray := trayBody(t, doc)
	hollow := hollowBody(t, doc)

	// The hollow shell resting on the tray floor against the wall x = -35:
	// opposed face patches on two faces with both sides non-convex, so no
	// single face holds the contact set and no manifold may follow (§9.2,
	// §10.5).
	cornered := contactBothOrders(t, doc, tray, hollow, r3.Identity(), contactPose(t, r3.Vec{X: -15, Z: 20}))
	require.Equal(t, decad.ContactTouching, cornered.Relation, "reason=%v", cornered.Reason)
	require.Nil(t, cornered.Manifold)
	require.Equal(t, decad.ContactNonConvex, cornered.Reason)

	// Away from the walls the floor alone holds the contact set, and the
	// shell's four bottom corners publish on it (§10.5).
	resting := contactBothOrders(t, doc, tray, hollow, r3.Identity(), contactPose(t, r3.Vec{Z: 20}))
	require.Equal(t, decad.ContactTouching, resting.Relation, "reason=%v", resting.Reason)
	requireManifoldAt(t, resting, []ratPoint{ratAt(-20, -20, 0), ratAt(20, -20, 0), ratAt(20, 20, 0),
		ratAt(-20, 20, 0)})

	lifted := contactBothOrders(t, doc, tray, hollow, r3.Identity(), contactPose(t, r3.Vec{Z: 22}))
	require.Equal(t, decad.ContactSeparated, lifted.Relation, "reason=%v", lifted.Reason)
	require.Equal(t, 2.0, lifted.Gap.Value.Base())
}

// TestExactPlanarPairAdmitsStitchedSolid reads mass_properties_mesh_test.go's
// stitched tetrahedron, four patches welded at the identity with every corner
// exact, through the exact pair (docs/multibody-dynamics-design.md §13 PR
// 14b), against a source-box floor whose top face is z = 0.
//
// Legs shown to fail (each deleted in turn, fixture red, then restored):
//   - the delta reading (planarStitchSolid returning a zero displacement):
//     the placed copy below reads an exact ContactTouching where §10.4
//     requires ContactBand;
//   - the face map (TestPlanarManifoldStitchedVertexTouch).
//
// The vertBound widening has no identity-placed fixture: only a
// certificate-welded stitch (docs/surface-design.md §6.2) carries a positive
// class bound at the identity. The term is the per-vertex bound Stitch
// stamps on each live Vertex, the one tessellate_stitch.go publishes per
// face, read back rather than proved twice; the placed copy's band below is
// asserted against it.
func TestExactPlanarPairAdmitsStitchedSolid(t *testing.T) {
	doc := decad.New()
	tetrahedron, a := stitchedTetrahedron(t, doc)
	for _, vertex := range tetrahedron.Vertices() {
		require.Zero(t, vertex.Position().Bound.Base(), "premise: an identity weld of exact corners is zero-bound")
	}
	floor := boxBodyAtZ(t, doc, -40, -40, 40, 40, -10, 10)
	id := r3.Identity()

	// Lifted 5 mm, the XY face is the nearest feature: the gap is exact.
	above := contactBothOrders(t, doc, floor, tetrahedron, id, contactPose(t, r3.Vec{Z: 5}))
	require.Equal(t, decad.ContactSeparated, above.Relation, "reason=%v", above.Reason)
	require.NotNil(t, above.Gap)
	require.Equal(t, 5.0, above.Gap.Value.Base())
	require.Zero(t, above.Gap.Bound.Base())

	// Resting on its XY face: an exact touch.
	resting := contactBothOrders(t, doc, floor, tetrahedron, id, id)
	require.Equal(t, decad.ContactTouching, resting.Relation, "reason=%v", resting.Reason)
	require.Equal(t, decad.Exact, resting.Gap.Exactness)
	require.Zero(t, resting.Gap.Value.Base())
	require.Equal(t, decad.ContactNoReason, resting.Reason)

	// The tray is not convex, so a manifold on its floor needs the
	// tetrahedron's own convexity certificate (§9.2); without it the touch
	// would name ContactNonConvex.
	tray := trayBody(t, doc)
	inTray := contactBothOrders(t, doc, tray, tetrahedron, id, id)
	require.Equal(t, decad.ContactTouching, inTray.Relation, "reason=%v", inTray.Reason)
	requireManifoldAt(t, inTray, []ratPoint{ratAt(a, 0, 0), ratAt(0, 0, 0), ratAt(0, a, 0)})

	// A placed copy carries the placement rounding as its displacement, so
	// the same held touch is a band (§10.4). Lifting by 1 mm and posing back
	// down moves no held corner: every coordinate below 8 keeps its ulp.
	lift, err := r3.Translation(r3.Vec{Z: 1})
	require.NoError(t, err)
	placed, err := tetrahedron.Placed(t.Context(), lift)
	require.NoError(t, err)
	var widest float64
	for _, vertex := range placed.Vertices() {
		widest = math.Max(widest, vertex.Position().Bound.Base())
	}
	require.Positive(t, widest, "premise: a placed stitch is not zero-bound")
	band := contactBothOrders(t, doc, floor, placed, id, contactPose(t, r3.Vec{Z: -1}))
	require.Equal(t, decad.ContactBand, band.Relation, "reason=%v", band.Reason)
	require.NotNil(t, band.Gap)
	require.Zero(t, band.Gap.Value.Base())
	require.GreaterOrEqual(t, band.Gap.Bound.Base(), 2*widest, "the band is twice the largest vertex bound")
}

func TestContactPairExactPlanarCancels(t *testing.T) {
	// A completed query is kept in the tray's report memo, so the polls are
	// counted in one document and the cancellations run in a second, built
	// the same way, that never completes the counted query before them. In
	// each, a first query at another touching pose caches both convexity
	// certificates, so the counted query reads them from the cache.
	pose := hexPose(t, r3.Vec{X: -10})
	warm := hexPose(t, r3.Vec{X: -12})
	setup := func() (*decad.Document, *decad.Body, *decad.Body) {
		doc := decad.New()
		tray := trayBody(t, doc)
		hex := hexPrismBody(t, doc)
		report, err := doc.ContactPair(t.Context(), tray, hex, r3.Identity(), warm, contactRequest())
		require.NoError(t, err)
		require.Equal(t, decad.ContactTouching, report.Relation)
		return doc, tray, hex
	}
	doc, tray, hex := setup()
	counting := newCancelAfterContext(t.Context(), math.MaxInt32)
	report, err := doc.ContactPair(counting, tray, hex, r3.Identity(), pose, contactRequest())
	require.NoError(t, err)
	require.Equal(t, decad.ContactTouching, report.Relation)
	polls := counting.calls.Load()
	require.Greater(t, polls, int32(8), "the planar kernel polls inside its loops")

	doc, tray, hex = setup()
	before := doc.Bodies()
	for _, limit := range []int32{3, polls / 2, polls} {
		canceling := newCancelAfterContext(t.Context(), limit)
		report, err := doc.ContactPair(canceling, tray, hex, r3.Identity(), pose, contactRequest())
		require.ErrorIs(t, err, context.Canceled, "limit=%d", limit)
		require.Nil(t, report)
	}
	require.Equal(t, before, doc.Bodies())
	// No canceled query entered the memo: the next one does the whole proof,
	// and only then does a repeat poll once, at entry, and read the memo.
	counting = newCancelAfterContext(t.Context(), math.MaxInt32)
	report, err = doc.ContactPair(counting, tray, hex, r3.Identity(), pose, contactRequest())
	require.NoError(t, err)
	require.Equal(t, decad.ContactTouching, report.Relation)
	require.Equal(t, polls, counting.calls.Load())
	counting = newCancelAfterContext(t.Context(), math.MaxInt32)
	_, err = doc.ContactPair(counting, tray, hex, r3.Identity(), pose, contactRequest())
	require.NoError(t, err)
	require.Equal(t, int32(1), counting.calls.Load())
}
