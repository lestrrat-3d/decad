package facetedrecord_test

import (
	"context"
	"fmt"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/surfacenormal"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topo/topotest"
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
		src.ExactCopyMoments = &facetedrecord.ExactCopyMomentRecord{}
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
		require.Nil(t, got.ExactCopyMoments)
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
		{name: "UnmarkedProofIncomplete", body: solid, want: "faceted similarity source has no complete proof",
			edit: func(p *facetedrecord.Payload) { p.SimilaritySource = false }},
		{name: "UnmarkedSheet", body: &topo.Body{Kind: topo.BodySheet, Solid: true}, want: noSource,
			edit: func(p *facetedrecord.Payload) { p.SimilaritySource = false }},
		{name: "UnmarkedOpenSolid", body: &topo.Body{Kind: topo.BodySolid}, want: noSource,
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

	t.Run("UnretainedCurvedAndDenotedTagsReadFaceted", func(t *testing.T) {
		t.Parallel()
		src := scaleSource()
		src.Groups = []facetedrecord.Group{
			{Surface: surfacegeom.Cylinder{}, NormalBound: 0.5, Reversed: true},
			{Surface: surfacegeom.Plane{}, Denoted: &surfacenormal.Revolved{}, NormalBound: 0.25},
			{Surface: surfacegeom.NURBSSurface{}},
		}
		got, err := facetedrecord.ScaleImage(t.Context(), solid, src, anchor, 2)
		require.NoError(t, err)
		require.Len(t, got.Groups, len(src.Groups))
		for i, group := range got.Groups {
			require.Nil(t, group.Surface, "group %d keeps a tag", i)
			require.Nil(t, group.Denoted, "group %d keeps a denoted patch", i)
			require.Zero(t, group.NormalBound, "group %d keeps a tag departure", i)
			require.False(t, group.Reversed, "group %d keeps a tag orientation", i)
		}
		// The source payload is unchanged.
		require.NotNil(t, src.Groups[0].Surface)
		require.NotNil(t, src.Groups[1].Denoted)
	})

	t.Run("UnmarkedSolidPassesTheMeshAudit", func(t *testing.T) {
		t.Parallel()
		body, src := builtUnmarkedSource(t, false)
		src.PointPlaneCapDisplacement = 3
		got, err := facetedrecord.ScaleImage(t.Context(), body, src, anchor, 2)
		require.NoError(t, err)
		require.False(t, got.SimilaritySource)
		require.False(t, got.BooleanSimilaritySource)
		require.Zero(t, got.PointPlaneCapDisplacement)
		for i, v := range src.Verts {
			require.Equal(t, anchor.Add(v.Sub(anchor).Scale(2)), got.Verts[i])
		}
	})

	t.Run("UnmarkedSolidFailsTheCrossingAudit", func(t *testing.T) {
		t.Parallel()
		body, src := builtUnmarkedSource(t, true)
		_, err := facetedrecord.ScaleImage(t.Context(), body, src, anchor, 2)
		require.ErrorIs(t, err, decaderr.ErrUnsupported)
		require.ErrorContains(t, err, "failed its mesh audit")
		require.ErrorContains(t, err, "crossing audit failed")
	})

	t.Run("CanceledContext", func(t *testing.T) {
		t.Parallel()
		ctx, cancel := context.WithCancel(t.Context())
		cancel()
		_, err := facetedrecord.ScaleImage(ctx, solid, scaleSource(), anchor, 2)
		require.ErrorIs(t, err, context.Canceled)
	})
}

// builtUnmarkedSource builds an unmarked faceted solid of two tetrahedra with
// integer corners. Overlapping places the second inside the first's faces,
// so the two shells cross; otherwise the shells are far apart. Each facet is
// its own face, so every face has a boundary.
func builtUnmarkedSource(t *testing.T, overlapping bool) (*topo.Body, facetedrecord.Payload) {
	t.Helper()
	shift := r3.Vec{X: 10}
	if overlapping {
		shift = r3.Vec{X: 1, Y: 1, Z: 1}
	}
	corners := []r3.Vec{{}, {X: 4}, {Y: 4}, {Z: 4}}
	verts := append([]r3.Vec(nil), corners...)
	for _, c := range corners {
		verts = append(verts, c.Add(shift))
	}
	tris := [][3]int{{0, 2, 1}, {0, 1, 3}, {0, 3, 2}, {1, 2, 3}, {4, 6, 5}, {4, 5, 7}, {4, 7, 6}, {5, 6, 7}}
	src := make([]int, len(tris))
	groups := make([]facetedrecord.Group, len(tris))
	for i := range src {
		src[i] = i
		groups[i] = facetedrecord.Group{Origins: []provenance.Ref{{Producer: 3, Role: fmt.Sprintf("side(%d)", i)}}, Planar: true}
	}
	env := topo.Env{Doc: &topo.Document{}, Ref: 3, Alloc: topotest.Alloc{}}
	body, err := facetedrecord.Build(t.Context(), env, facetedrecord.Payload{
		Xform: r3.Identity(), Verts: verts, Tris: tris, Src: src,
		VertexBound: make([]float64, len(verts)), DPair: 20,
		Groups: groups,
	})
	require.NoError(t, err)
	fp, ok := body.Payload.(facetedrecord.Payload)
	require.True(t, ok)
	return body, fp
}
