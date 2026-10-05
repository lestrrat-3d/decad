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
	upper, err = upper.Placed(t.Context(), shift)
	require.NoError(t, err)
	if reverse {
		base, upper = upper, base
	}
	union, err := Union(t.Context(), base, upper)
	require.NoError(t, err)
	pp, ok := union.payload.(facetedPayload)
	require.True(t, ok)
	require.Positive(t, pp.meshBound)
	require.Positive(t, pp.volSymDiff)
	require.Empty(t, pp.exactSourceVerts)
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
	pp := body.payload.(facetedPayload)
	require.NotNil(t, pp.lowerSupport)
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
	placed, err := body.Placed(t.Context(), shift)
	require.NoError(t, err)
	require.Nil(t, placed.payload.(facetedPayload).lowerSupport)
	uncertain, err := doc.ContactPair(t.Context(), floor, placed, r3.Identity(), r3.Identity(),
		ContactRequest{PointResolution: units.Millimeters(1e-6), NormalResolution: units.Radians(1e-6)})
	require.NoError(t, err)
	require.Equal(t, ContactUndecided, uncertain.Relation)
}

func TestPositiveBoundFacetedUnionRefusesToolBelowLowerFace(t *testing.T) {
	doc := New()
	base := internalBoxBody(t, doc, -5, -5, 5, 5, 10)
	upper := internalDiscBody(t, doc, 2, 4)
	shift, err := r3.Translation(r3.Vec{Z: -1})
	require.NoError(t, err)
	upper, err = upper.Placed(t.Context(), shift)
	require.NoError(t, err)
	union, err := Union(t.Context(), base, upper)
	require.NoError(t, err)
	pp := union.payload.(facetedPayload)
	require.Positive(t, pp.meshBound)
	require.Nil(t, pp.lowerSupport)
	_, proven, err := sourceFacetedAxisSupport(t.Context(), union, r3.Identity(), 2, 0)
	require.NoError(t, err)
	require.False(t, proven)
}
