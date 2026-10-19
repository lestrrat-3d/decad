package facetedrecord_test

import (
	"context"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/surfacenormal"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// scaleSource is a certified similarity source: a tetrahedron with integer
// corners and no vertex rounding.
func scaleSource() facetedrecord.Payload {
	return facetedrecord.Payload{
		SimilaritySource: true,
		Xform:            r3.Identity(),
		Verts:            []r3.Vec{{}, {X: 2}, {Y: 2}, {Z: 2}},
		VertexBound:      []float64{0, 0, 0, 0},
		Tris:             [][3]int{{0, 2, 1}, {0, 1, 3}, {0, 3, 2}, {1, 2, 3}},
		DPair:            4,
	}
}

func TestScaleImage(t *testing.T) {
	t.Parallel()

	solid := &topo.Body{Kind: topo.BodySolid, Solid: true}
	anchor := r3.Vec{X: 1}

	t.Run("ScalesTheHeldMeshAboutTheAnchor", func(t *testing.T) {
		t.Parallel()
		frame, err := r3.NewFrame(r3.Vec{X: 3}, r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
		require.NoError(t, err)
		move, err := r3.Translation(r3.Vec{Y: 7})
		require.NoError(t, err)
		src := scaleSource()
		src.Xform = move
		src.Groups = []facetedrecord.Group{{Surface: surfacegeom.Plane{Frame: frame}}}
		src.LoftFacesPrimitive = true
		src.CopyLineage = &facetedrecord.FacetedCopyLineage{}
		src.PointPlaneSourceEligible = true

		got, err := facetedrecord.ScaleImage(t.Context(), solid, src, anchor, 2)
		require.NoError(t, err)
		require.Len(t, got.Verts, len(src.Verts))
		for i, v := range src.Verts {
			want := anchor.Add(v.Sub(anchor).Scale(2))
			require.InDelta(t, want.X, got.Verts[i].X, got.VertexBound[i]+1e-12)
			require.InDelta(t, want.Y, got.Verts[i].Y, got.VertexBound[i]+1e-12)
			require.InDelta(t, want.Z, got.Verts[i].Z, got.VertexBound[i]+1e-12)
		}
		require.GreaterOrEqual(t, got.DPair, 2*src.DPair)
		require.Equal(t, r3.Identity(), got.Xform)
		// Source roles do not survive the image.
		require.False(t, got.LoftFacesPrimitive)
		require.False(t, got.PointPlaneSourceEligible)
		require.Nil(t, got.CopyLineage)
		// The plane about the anchor moves to anchor + 2(origin - anchor) and
		// carries the unit-normal departure bound.
		plane, ok := got.Groups[0].Surface.(surfacegeom.Plane)
		require.True(t, ok)
		require.InDelta(t, 5, plane.Frame.Origin().X, 1e-12)
		require.Equal(t, 2.0, got.Groups[0].NormalBound)
		// The source payload is unchanged.
		require.False(t, src.Groups[0].NormalBound == 2)
		require.True(t, src.LoftFacesPrimitive)
	})

	const noSource = "no certified similarity source"

	refusals := []struct {
		name string
		body *topo.Body
		edit func(*facetedrecord.Payload)
		want string
	}{
		{name: "NoSimilarityRole", body: solid, want: noSource,
			edit: func(p *facetedrecord.Payload) { p.SimilaritySource = false }},
		{name: "InvalidPlacement", body: solid, want: noSource,
			edit: func(p *facetedrecord.Payload) { p.Xform = r3.Transform{} }},
		{name: "NoVertices", body: solid, want: noSource,
			edit: func(p *facetedrecord.Payload) { p.Verts, p.VertexBound = nil, nil }},
		{name: "VertexBoundCountDiffers", body: solid, want: noSource,
			edit: func(p *facetedrecord.Payload) { p.VertexBound = p.VertexBound[:3] }},
		{name: "BooleanRoleOnSheet", body: &topo.Body{Kind: topo.BodySheet, Solid: true}, want: noSource,
			edit: func(p *facetedrecord.Payload) { p.SimilaritySource, p.BooleanSimilaritySource = false, true }},
		{name: "BooleanRoleOnOpenSolid", body: &topo.Body{Kind: topo.BodySolid}, want: noSource,
			edit: func(p *facetedrecord.Payload) { p.SimilaritySource, p.BooleanSimilaritySource = false, true }},
		{name: "BooleanProofIncomplete", body: solid, want: "no complete proof",
			edit: func(p *facetedrecord.Payload) { p.SimilaritySource, p.BooleanSimilaritySource = false, true }},
		{name: "SlabSourceDiffersFromSplitSource", body: solid, want: "does not match its Split source",
			edit: func(p *facetedrecord.Payload) {
				p.PointSimilarity = &facetedrecord.PointPlaneSimilarityRecord{Slice: &facetedrecord.PointPlaneSliceRecord{}}
			}},
		{name: "DenotedSurface", body: solid, want: "denoted surface has no similarity proof",
			edit: func(p *facetedrecord.Payload) {
				p.Groups = []facetedrecord.Group{{Denoted: &surfacenormal.Revolved{}}}
			}},
		{name: "CurvedSurface", body: solid, want: "retained curved surface has no similarity proof",
			edit: func(p *facetedrecord.Payload) {
				p.Groups = []facetedrecord.Group{{Surface: surfacegeom.Cylinder{}}}
			}},
		{name: "DiameterOverflows", body: solid, want: "scaled source proof is not finite",
			edit: func(p *facetedrecord.Payload) { p.DPair = 1.7e308 }},
	}
	for _, tc := range refusals {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			src := scaleSource()
			tc.edit(&src)
			_, err := facetedrecord.ScaleImage(t.Context(), tc.body, src, anchor, 10)
			require.ErrorIs(t, err, decaderr.ErrUnsupported)
			require.ErrorContains(t, err, tc.want)
		})
	}

	t.Run("CanceledContext", func(t *testing.T) {
		t.Parallel()
		ctx, cancel := context.WithCancel(t.Context())
		cancel()
		_, err := facetedrecord.ScaleImage(ctx, solid, scaleSource(), anchor, 2)
		require.ErrorIs(t, err, context.Canceled)
	})
}
