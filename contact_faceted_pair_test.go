package decad_test

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
// (the hollow-on-tray case loses ContactNonConvex); certifying no body
// convex (the hexagon's touch reads ContactNonConvex).

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
// relation, gap and reason, as §9.5 and contact-geometry §5 state.
func contactBothOrders(t *testing.T, doc *decad.Document, a, b *decad.Body,
	poseA, poseB r3.Transform) *decad.ContactReport {
	t.Helper()
	forward, err := doc.ContactPair(t.Context(), a, b, poseA, poseB, contactRequest())
	require.NoError(t, err)
	reversed, err := doc.ContactPair(t.Context(), b, a, poseB, poseA, contactRequest())
	require.NoError(t, err)
	require.Equal(t, forward.Relation, reversed.Relation, "reason=%v/%v", forward.Reason, reversed.Reason)
	require.Equal(t, forward.Reason, reversed.Reason)
	require.Equal(t, forward.Gap, reversed.Gap)
	require.Nil(t, forward.Manifold)
	require.Nil(t, reversed.Manifold)
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
	// convex, so the missing manifold is a missing normal proof (§9.3).
	touch := contactBothOrders(t, doc, tray, hex, id, hexPose(t, r3.Vec{X: -10}))
	require.Equal(t, decad.ContactTouching, touch.Relation, "reason=%v", touch.Reason)
	require.Equal(t, decad.Exact, touch.Gap.Exactness)
	require.Zero(t, touch.Gap.Value.Base())
	require.Equal(t, decad.ContactNoNormalProof, touch.Reason)

	// Half a millimetre lower, the corner's edges cross the floor's face.
	crossing := contactBothOrders(t, doc, tray, hex, id, hexPose(t, r3.Vec{X: -10, Z: -0.5}))
	require.Equal(t, decad.ContactOverlapping, crossing.Relation, "reason=%v", crossing.Reason)
	require.Nil(t, crossing.Gap)
	require.Equal(t, decad.ContactNoNormalProof, crossing.Reason)

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

	// The hollow shell resting on the tray floor: an opposed face patch with
	// both sides non-convex, so no manifold may follow (§9.2).
	resting := contactBothOrders(t, doc, tray, hollow, r3.Identity(), contactPose(t, r3.Vec{Z: 20}))
	require.Equal(t, decad.ContactTouching, resting.Relation, "reason=%v", resting.Reason)
	require.Equal(t, decad.ContactNonConvex, resting.Reason)

	lifted := contactBothOrders(t, doc, tray, hollow, r3.Identity(), contactPose(t, r3.Vec{Z: 22}))
	require.Equal(t, decad.ContactSeparated, lifted.Relation, "reason=%v", lifted.Reason)
	require.Equal(t, 2.0, lifted.Gap.Value.Base())
}

func TestContactPairExactPlanarCancels(t *testing.T) {
	doc := decad.New()
	tray := trayBody(t, doc)
	hex := hexPrismBody(t, doc)
	pose := hexPose(t, r3.Vec{X: -10})
	// The first query also caches both convexity certificates; the second
	// counts the polls of a query that reads them from the cache.
	var polls int32
	for range 2 {
		counting := newCancelAfterContext(t.Context(), math.MaxInt32)
		report, err := doc.ContactPair(counting, tray, hex, r3.Identity(), pose, contactRequest())
		require.NoError(t, err)
		require.Equal(t, decad.ContactTouching, report.Relation)
		polls = counting.calls.Load()
	}
	require.Greater(t, polls, int32(8), "the planar kernel polls inside its loops")
	before := doc.Bodies()
	for _, limit := range []int32{3, polls / 2, polls} {
		canceling := newCancelAfterContext(t.Context(), limit)
		report, err := doc.ContactPair(canceling, tray, hex, r3.Identity(), pose, contactRequest())
		require.ErrorIs(t, err, context.Canceled, "limit=%d", limit)
		require.Nil(t, report)
	}
	require.Equal(t, before, doc.Bodies())
}
