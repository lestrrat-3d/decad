package decad_test

import (
	"context"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func facetedContextBody(t *testing.T) (*decad.Document, *decad.Body) {
	t.Helper()
	doc := decad.New()
	plate := boxBody(t, doc, 0, 0, 20, 20, 8)
	tool := translated(t, diskBody(t, doc, 14, 6, 2), 0, 0, -6)
	body, err := decad.Cut(plate, tool)
	require.NoError(t, err)
	return doc, body
}

func TestPlacementContextVariantsCancelFacetedRebuild(t *testing.T) {
	shift, err := r3.Translation(r3.NewVec(100, 0, 0))
	require.NoError(t, err)

	tests := []struct {
		name string
		run  func(context.Context, *decad.Body) (*decad.Body, error)
	}{
		{
			name: "Placed",
			run: func(ctx context.Context, body *decad.Body) (*decad.Body, error) {
				return body.PlacedContext(ctx, shift)
			},
		},
		{
			name: "Duplicate",
			run: func(ctx context.Context, body *decad.Body) (*decad.Body, error) {
				return body.DuplicateContext(ctx)
			},
		},
		{
			name: "PlacedCopy",
			run: func(ctx context.Context, body *decad.Body) (*decad.Body, error) {
				return body.PlacedCopyContext(ctx, shift)
			},
		},
	}

	for _, test := range tests {
		t.Run(test.name, func(t *testing.T) {
			doc, body := facetedContextBody(t)
			beforeBodies := doc.Bodies()
			beforeRecipe := doc.Recipe()
			ctx := newCancelAfterContext(t.Context(), 3)

			got, err := test.run(ctx, body)
			require.ErrorIs(t, err, context.Canceled)
			require.Nil(t, got)
			require.GreaterOrEqual(t, ctx.calls.Load(), int32(3),
				`cancellation must reach the faceted payload rebuild`)
			require.Equal(t, beforeBodies, doc.Bodies(),
				`a canceled rebuild must not change live bodies`)
			require.Equal(t, beforeRecipe, doc.Recipe(),
				`a canceled rebuild must not append a recipe step`)
		})
	}
}

func TestPlacementContextVariantsMatchCompatibilityWrappers(t *testing.T) {
	shift, err := r3.Translation(r3.NewVec(100, 0, 0))
	require.NoError(t, err)

	tests := []struct {
		name       string
		wrapper    func(*decad.Body) (*decad.Body, error)
		contextual func(*decad.Body) (*decad.Body, error)
	}{
		{
			name:    "Placed",
			wrapper: func(body *decad.Body) (*decad.Body, error) { return body.Placed(shift) },
			contextual: func(body *decad.Body) (*decad.Body, error) {
				return body.PlacedContext(t.Context(), shift)
			},
		},
		{
			name:    "Duplicate",
			wrapper: func(body *decad.Body) (*decad.Body, error) { return body.Duplicate() },
			contextual: func(body *decad.Body) (*decad.Body, error) {
				return body.DuplicateContext(t.Context())
			},
		},
		{
			name:    "PlacedCopy",
			wrapper: func(body *decad.Body) (*decad.Body, error) { return body.PlacedCopy(shift) },
			contextual: func(body *decad.Body) (*decad.Body, error) {
				return body.PlacedCopyContext(t.Context(), shift)
			},
		},
	}

	for _, test := range tests {
		t.Run(test.name, func(t *testing.T) {
			s, p := plateSketch(t)
			oldDoc := decad.New()
			oldSource, err := oldDoc.Extrude(s, p, decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
			require.NoError(t, err)
			newDoc := decad.New()
			newSource, err := newDoc.Extrude(s, p, decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
			require.NoError(t, err)

			oldBody, err := test.wrapper(oldSource)
			require.NoError(t, err)
			newBody, err := test.contextual(newSource)
			require.NoError(t, err)

			oldVolume, err := oldBody.Volume()
			require.NoError(t, err)
			newVolume, err := newBody.Volume()
			require.NoError(t, err)
			require.Equal(t, oldVolume, newVolume)
			oldCentroid, err := oldBody.Centroid()
			require.NoError(t, err)
			newCentroid, err := newBody.Centroid()
			require.NoError(t, err)
			require.Equal(t, oldCentroid, newCentroid)
			require.Equal(t, oldDoc.Recipe(), newDoc.Recipe())
			require.Len(t, newDoc.Bodies(), len(oldDoc.Bodies()))
		})
	}
}
