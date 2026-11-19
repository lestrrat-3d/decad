package facetedrecord_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"

	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topo/topotest"
	"github.com/lestrrat-3d/decad/internal/topobuild"
)

// An oblique wall's retained Plane tag states ExtentDelta: how far its true patch lies from the tag over the face
// (charged-window-chamfer-design §2). These tests follow it from a group through assembly, the mesh Boolean's
// SourceTag, every placement and the scale, and into the edge curve's slack.

const (
	obliqueTheta  = 3e-14
	obliqueExtent = 5e-15
)

func obliquePlane(t *testing.T) surfacegeom.Plane {
	t.Helper()
	frame, err := r3.NewFrame(r3.NewVec(1, 1, 1), r3.NewVec(0, 1, 0), r3.NewVec(0, 0, 1))
	require.NoError(t, err)
	return surfacegeom.Plane{Frame: frame}
}

func obliqueGroup(t *testing.T) facetedrecord.Group {
	t.Helper()
	return facetedrecord.Group{Surface: obliquePlane(t), Retained: true,
		NormalBound: obliqueTheta, ExtentDelta: obliqueExtent}
}

// obliqueTetra builds a tetrahedron whose slanted face is the retained plane group g, and returns the payload and
// that face.
func obliqueTetra(t *testing.T, g facetedrecord.Group) (facetedrecord.Payload, *topo.Face) {
	t.Helper()
	fp := facetedrecord.Payload{
		Verts:       []r3.Vec{{}, {X: 2}, {Y: 2}, {Z: 2}},
		VertexBound: []float64{1e-6, 1e-6, 1e-6, 1e-6},
		Tris:        [][3]int{{0, 2, 1}, {0, 1, 3}, {0, 3, 2}, {1, 2, 3}},
		Src:         []int{0, 1, 2, 3},
		DPair:       4,
		Xform:       r3.Identity(),
	}
	for i := range 3 {
		fp.Groups = append(fp.Groups, facetedrecord.Group{
			Origins: []provenance.Ref{{Producer: 9, Role: string(rune('a' + i))}}})
	}
	g.Origins = []provenance.Ref{{Producer: 9, Role: "wall"}}
	fp.Groups = append(fp.Groups, g)
	env := topo.Env{Doc: &topo.Document{}, Ref: 9, Alloc: topotest.Alloc{}}
	b, err := facetedrecord.Build(t.Context(), env, fp)
	require.NoError(t, err)
	return fp, obliqueFace(t, b)
}

// obliqueFace is the largest face of b, the tetrahedron's slanted one.
func obliqueFace(t *testing.T, b *topo.Body) *topo.Face {
	t.Helper()
	var wall *topo.Face
	for _, f := range topobuild.Faces(b) {
		if wall == nil || f.Area > wall.Area {
			wall = f
		}
	}
	require.NotNil(t, wall)
	return wall
}

func TestAssembleCarriesExtentDelta(t *testing.T) {
	t.Parallel()
	_, f := obliqueTetra(t, obliqueGroup(t))
	require.True(t, f.RetainedTag)
	require.Equal(t, obliqueExtent, f.ExtentDelta)
	require.Equal(t, obliqueTheta, f.TagDeparture)
	// A plane adds no held term of its own: NormalBound is the tag's tilt.
	require.Equal(t, obliqueTheta, f.NormalBound)
	require.False(t, f.HasAxialDelta, "an oblique wall states no parallel offset")
}

func TestSourceTagCarriesExtentDelta(t *testing.T) {
	t.Parallel()
	plane := obliquePlane(t)
	wall := func(extent float64) *topo.Face {
		return &topo.Face{Surface: plane, RetainedTag: true, TagDeparture: obliqueTheta, NormalBound: obliqueTheta,
			ExtentDelta: extent}
	}

	t.Run("a finite extent rides with the tag and its tilt", func(t *testing.T) {
		t.Parallel()
		g, ok := facetedrecord.SourceTag(wall(obliqueExtent), false)
		require.True(t, ok)
		require.Equal(t, obliqueExtent, g.ExtentDelta)
		require.Equal(t, obliqueTheta, g.NormalBound)
		require.False(t, g.HasAxialDelta)
	})
	t.Run("an extent that bounds nothing gives no tag", func(t *testing.T) {
		t.Parallel()
		for _, extent := range []float64{-1, math.NaN(), math.Inf(1), math.Inf(-1)} {
			g, ok := facetedrecord.SourceTag(wall(extent), false)
			require.False(t, ok, "extent %v", extent)
			require.Nil(t, g.Surface)
		}
	})
	t.Run("a face with no extent states none", func(t *testing.T) {
		t.Parallel()
		g, ok := facetedrecord.SourceTag(wall(0), false)
		require.True(t, ok)
		require.Zero(t, g.ExtentDelta)
	})
	t.Run("only a retained plane carries it", func(t *testing.T) {
		t.Parallel()
		built := &topo.Face{Surface: plane, ExtentDelta: obliqueExtent}
		g, ok := facetedrecord.SourceTag(built, false)
		require.True(t, ok)
		require.Zero(t, g.ExtentDelta)
	})
}

func TestPlacementDropsExtentDelta(t *testing.T) {
	t.Parallel()
	fp, _ := obliqueTetra(t, obliqueGroup(t))
	shift, err := r3.Translation(r3.NewVec(0.1, 0.2, 0.3))
	require.NoError(t, err)
	env := topo.Env{Doc: &topo.Document{}, Ref: 9, Alloc: topotest.Alloc{}}
	placed, err := fp.Placed(t.Context(), env, shift)
	require.NoError(t, err)
	f := obliqueFace(t, placed)
	require.True(t, f.RetainedTag, "the departed tag stays")
	require.Zero(t, f.ExtentDelta)
	require.GreaterOrEqual(t, f.TagDeparture, obliqueTheta, "the tilt carries over")
	require.GreaterOrEqual(t, f.NormalBound, obliqueTheta)
}

func TestPlacementPreservesCircularFitPrimitive(t *testing.T) {
	fp, _ := obliqueTetra(t, obliqueGroup(t))
	fp.CircularFitPrimitive = true
	shift, err := r3.Translation(r3.NewVec(0.1, 0.2, 0.3))
	require.NoError(t, err)
	env := topo.Env{Doc: &topo.Document{}, Ref: 9, Alloc: topotest.Alloc{}}
	placed, err := fp.Placed(t.Context(), env, shift)
	require.NoError(t, err)
	got, ok := placed.Payload.(facetedrecord.Payload)
	require.True(t, ok)
	require.True(t, got.CircularFitPrimitive)
	require.GreaterOrEqual(t, got.MeshBound, fp.MeshBound)
}

func TestExactGroupPlacedDropsExtentDelta(t *testing.T) {
	t.Parallel()
	q, delta, verts := exactGroupFixture(t, 20)
	g := obliqueGroup(t)
	require.NoError(t, facetedrecord.ExactGroupPlaced(&g, q, delta, verts, 1e-9))
	require.IsType(t, surfacegeom.Plane{}, g.Surface)
	require.True(t, g.Retained)
	require.Zero(t, g.ExtentDelta)
	require.GreaterOrEqual(t, g.NormalBound, obliqueTheta)
}

func TestScaleImageDropsExtentDelta(t *testing.T) {
	t.Parallel()
	solid := &topo.Body{Kind: topo.BodySolid, Solid: true}
	src := scaleSource()
	src.Groups = []facetedrecord.Group{obliqueGroup(t)}
	got, err := facetedrecord.ScaleImage(t.Context(), solid, src, r3.Vec{X: 1}, 2)
	require.NoError(t, err)
	require.Len(t, got.Groups, 1)
	require.True(t, got.Groups[0].Retained, "the departed tag stays")
	require.Zero(t, got.Groups[0].ExtentDelta)
	require.GreaterOrEqual(t, got.Groups[0].NormalBound, obliqueTheta)
}

// TestRetainedEdgeCurveExtentDelta widens a plane-plane rim's falsifier slack by planeRimAmp times the extent. The
// floor z = 0 and the wall x = 0 meet in the Y axis; a chain point 1e-5 off it, with bound 1e-9, disproves the line
// until the wall states an extent whose amplified value reaches that offset. A negative or non-finite extent
// declines.
func TestRetainedEdgeCurveExtentDelta(t *testing.T) {
	t.Parallel()
	chain := []r3.Vec{r3.NewVec(0, 0, 0), r3.NewVec(1e-5, 3, 0), r3.NewVec(0, 7, 0)}
	bounds := []float64{1e-9, 1e-9, 1e-9}
	for _, tc := range []struct {
		name   string
		extent float64
		ok     bool
	}{
		{"no extent", 0, false},
		{"too small to reach the offset", 1e-9, false},
		{"amplified past the offset", 1e-7, true},
		{"negative", -1e-7, false},
		{"NaN", math.NaN(), false},
		{"infinite", math.Inf(1), false},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			floor := retained(planeAt(t, r3.Vec{}, r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0)))
			wall := retained(planeAt(t, r3.Vec{}, r3.NewVec(0, 1, 0), r3.NewVec(0, 0, 1)))
			wall.ExtentDelta = tc.extent
			curve, ok := facetedrecord.RetainedEdgeCurve(floor, wall, chain, bounds, false)
			require.Equal(t, tc.ok, ok)
			if !tc.ok {
				require.Nil(t, curve)
				return
			}
			require.IsType(t, surfacegeom.Line3{}, curve)
		})
	}
}
