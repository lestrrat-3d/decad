package decad

import (
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func positiveBoundFacetedFixture(t *testing.T, reverse bool) (*Document, *Body, *Body) {
	t.Helper()
	doc := New()
	base := internalBoxBody(t, doc, -5, -5, 5, 5, 10)
	upper := internalDiscBody(t, doc, 2, 4)
	shift, err := r3.Translation(r3.Vec{Z: 8})
	require.NoError(t, err)
	upper, err = Transform(t.Context(), upper, shift)
	require.NoError(t, err)
	if reverse {
		base, upper = upper, base
	}
	union, err := Union(t.Context(), base, upper)
	require.NoError(t, err)
	pp, ok := union.n.Payload.(facetedPayload)
	require.True(t, ok)
	require.Positive(t, pp.MeshBound)
	require.Positive(t, pp.VolSymDiff)
	require.Empty(t, pp.ExactSourceVerts)
	floor := internalOffsetBox(t, doc, -20, -20, 20, 20, -10,
		Distance{D: units.Millimeters(10), Dir: Along})
	return doc, floor, union
}

func TestPositiveBoundFacetedUnionRetainsExactLowerFace(t *testing.T) {
	for _, reverse := range []bool{false, true} {
		t.Run(map[bool]string{false: "box first", true: "cap first"}[reverse], func(t *testing.T) {
			positiveBoundFacetedUnionRetainsExactLowerFace(t, reverse)
		})
	}
}

func positiveBoundFacetedUnionRetainsExactLowerFace(t *testing.T, reverse bool) {
	t.Helper()
	doc, floor, body := positiveBoundFacetedFixture(t, reverse)
	pp := body.n.Payload.(facetedPayload)
	require.NotNil(t, pp.LowerSupport)
	mass, err := body.MassProperties(t.Context(), units.KilogramsPerCubicMillimeter(.001))
	require.NoError(t, err)
	require.Positive(t, mass.Mass.Bound.Base())
	require.Positive(t, mass.Center.Bound.Base())
	contact, err := doc.ContactPair(t.Context(), floor, body, r3.Identity(), r3.Identity(),
		ContactRequest{PointResolution: units.Millimeters(1e-6), NormalResolution: units.Radians(1e-6)})
	require.NoError(t, err)
	require.Equal(t, ContactTouching, contact.Relation)
	require.NotNil(t, contact.Manifold)
	require.Len(t, contact.Manifold.Points, 4)
	for _, point := range contact.Manifold.Points {
		require.Contains(t, body.Faces(), point.FaceB)
		require.Equal(t, r3.Vec{Z: 1}, point.Normal.Value)
	}
	shift, err := r3.Translation(r3.Vec{X: .1})
	require.NoError(t, err)
	placed, err := Transform(t.Context(), body, shift)
	require.NoError(t, err)
	require.Nil(t, placed.n.Payload.(facetedPayload).LowerSupport)
	uncertain, err := doc.ContactPair(t.Context(), floor, placed, r3.Identity(), r3.Identity(),
		ContactRequest{PointResolution: units.Millimeters(1e-6), NormalResolution: units.Radians(1e-6)})
	require.NoError(t, err)
	// Without the exact support certificate the held mesh decides the pair,
	// and its mesh bound turns the held touch into the §10.4 band.
	require.Equal(t, ContactBand, uncertain.Relation)
	require.Equal(t, 2*placed.n.Payload.(facetedPayload).MeshBound, uncertain.Gap.Bound.Base())
}

func TestPositiveBoundFacetedUnionRefusesToolBelowLowerFace(t *testing.T) {
	doc := New()
	base := internalBoxBody(t, doc, -5, -5, 5, 5, 10)
	upper := internalDiscBody(t, doc, 2, 4)
	shift, err := r3.Translation(r3.Vec{Z: -1})
	require.NoError(t, err)
	upper, err = Transform(t.Context(), upper, shift)
	require.NoError(t, err)
	union, err := Union(t.Context(), base, upper)
	require.NoError(t, err)
	pp := union.n.Payload.(facetedPayload)
	require.Positive(t, pp.MeshBound)
	require.Nil(t, pp.LowerSupport)
	_, proven, err := sourceFacetedAxisSupport(t.Context(), union, r3.Identity(), 2, 0)
	require.NoError(t, err)
	require.False(t, proven)
}
