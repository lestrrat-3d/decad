package facetedrecord_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topo/topotest"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestSourceTagChargedCylinder(t *testing.T) {
	t.Parallel()
	cyl := surfacegeom.Cylinder{Axis: r3.NewVec(0, 0, 1), Radius: units.Millimeters(4)}
	wall := func(delta float64) *topo.Face {
		return &topo.Face{Surface: cyl, RetainedTag: true, HasAxialDelta: true, AxialDelta: delta, ViewCharge: true}
	}

	t.Run("a finite displacement rides with the tag, the view mark does not", func(t *testing.T) {
		t.Parallel()
		g, ok := facetedrecord.SourceTag(wall(1e-6), false)
		require.True(t, ok)
		require.True(t, g.HasAxialDelta)
		require.Equal(t, 1e-6, g.AxialDelta)
		require.False(t, g.ViewCharge)
	})
	t.Run("a zero displacement is stated, not dropped", func(t *testing.T) {
		t.Parallel()
		g, ok := facetedrecord.SourceTag(wall(0), false)
		require.True(t, ok)
		require.True(t, g.HasAxialDelta)
		require.Zero(t, g.AxialDelta)
	})
	t.Run("a displacement that bounds nothing gives no tag", func(t *testing.T) {
		t.Parallel()
		for _, delta := range []float64{-1, math.NaN(), math.Inf(1)} {
			g, ok := facetedrecord.SourceTag(wall(delta), false)
			require.False(t, ok, "delta %v", delta)
			require.Nil(t, g.Surface)
		}
	})
	t.Run("an uncharged cylinder states nothing", func(t *testing.T) {
		t.Parallel()
		g, ok := facetedrecord.SourceTag(&topo.Face{Surface: cyl, RetainedTag: true}, false)
		require.True(t, ok)
		require.False(t, g.HasAxialDelta)
	})
}

// chargedTetra builds a tetrahedron whose slanted face is a retained cylinder tag of radius 4, with vertex
// bound 1e-6, stating delta when set; it returns the slanted face, which keeps its cylinder tag unless the
// held normal term declined it.
func chargedTetra(t *testing.T, delta float64, set bool) *topo.Face {
	t.Helper()
	const radius, bound = 4.0, 1e-6
	fp := facetedrecord.Payload{
		Verts:       []r3.Vec{{}, {X: 2}, {Y: 2}, {Z: 2}},
		VertexBound: []float64{bound, bound, bound, bound},
		Tris:        [][3]int{{0, 2, 1}, {0, 1, 3}, {0, 3, 2}, {1, 2, 3}},
		Src:         []int{0, 1, 2, 3},
		DPair:       4,
	}
	for i := range 3 {
		fp.Groups = append(fp.Groups, facetedrecord.Group{
			Origins: []provenance.Ref{{Producer: 9, Role: string(rune('a' + i))}}})
	}
	fp.Groups = append(fp.Groups, facetedrecord.Group{
		Origins:  []provenance.Ref{{Producer: 9, Role: "wall"}},
		Surface:  surfacegeom.Cylinder{Axis: r3.NewVec(0, 0, 1), Radius: units.Millimeters(radius)},
		Retained: true, HasAxialDelta: set, AxialDelta: delta,
	})
	env := topo.Env{Doc: &topo.Document{}, Ref: 9, Alloc: topotest.Alloc{}}
	b, err := facetedrecord.Build(t.Context(), env, fp)
	require.NoError(t, err)
	// The slanted face is the largest of the four.
	var wall *topo.Face
	for _, f := range topobuild.Faces(b) {
		if wall == nil || f.Area > wall.Area {
			wall = f
		}
	}
	require.NotNil(t, wall)
	return wall
}

func TestChargedCylinderNormalBound(t *testing.T) {
	t.Parallel()
	const radius, bound, delta = 4.0, 1e-6, 1e-4
	twin := chargedTetra(t, 0, false)
	charged := chargedTetra(t, delta, true)

	require.True(t, charged.HasAxialDelta)
	require.Equal(t, delta, charged.AxialDelta)
	require.False(t, charged.ViewCharge)
	// The uncharged twin states 2 delta_f / R; delta_f is read back from it.
	deltaF := twin.NormalBound * radius / 2
	require.Positive(t, deltaF)
	// The charged wall states 2 * 2 (delta_f + delta) / R, rounded up.
	want := 4 * (deltaF + delta) / radius
	require.GreaterOrEqual(t, charged.NormalBound, want*(1-1e-12))
	require.InDelta(t, want, charged.NormalBound, 1e-12)
	require.Greater(t, charged.NormalBound, twin.NormalBound+3.99*delta/radius)
}

func TestChargedCylinderDeclinedPastOne(t *testing.T) {
	t.Parallel()
	// The tag stands while 4 (delta_f + delta) / R < 1; at or past it the face reads Faceted.
	require.True(t, chargedTetra(t, 0.9, true).RetainedTag)
	for _, delta := range []float64{1.1, math.Inf(1)} {
		f := chargedTetra(t, delta, true)
		require.False(t, f.RetainedTag, "delta %v", delta)
		require.Equal(t, surfacegeom.KindFaceted, f.Surface.Kind(), "delta %v", delta)
	}
}

func TestExactGroupPlacedChargedCylinder(t *testing.T) {
	t.Parallel()
	q, delta, verts := exactGroupFixture(t, 35)
	cyl := surfacegeom.Cylinder{Origin: r3.NewVec(20, 0, 10), Axis: r3.NewVec(0, 1, 0), Radius: units.Millimeters(4)}
	g := facetedrecord.Group{Surface: cyl, Retained: true, HasAxialDelta: true, AxialDelta: 0.03, ViewCharge: true}
	require.NoError(t, facetedrecord.ExactGroupPlaced(&g, q, delta, verts, 1e-9))
	require.IsType(t, surfacegeom.Cylinder{}, g.Surface)
	require.True(t, g.HasAxialDelta)
	require.Equal(t, 0.03, g.AxialDelta)
	require.False(t, g.ViewCharge)

	plain := facetedrecord.Group{Surface: cyl, Retained: true}
	require.NoError(t, facetedrecord.ExactGroupPlaced(&plain, q, delta, verts, 1e-9))
	require.False(t, plain.HasAxialDelta)
}

func TestScaleImageChargedWall(t *testing.T) {
	t.Parallel()
	solid := &topo.Body{Kind: topo.BodySolid, Solid: true}
	anchor := r3.Vec{X: 1}
	frame, err := r3.NewFrame(r3.Vec{X: 3}, r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	src := scaleSource()
	src.Groups = []facetedrecord.Group{
		{Surface: surfacegeom.Cylinder{Axis: r3.NewVec(0, 0, 1), Radius: units.Millimeters(4)}, Retained: true,
			HasAxialDelta: true, AxialDelta: 0.25},
		{Surface: surfacegeom.Plane{Frame: frame}, Retained: true, HasAxialDelta: true, AxialDelta: 0.125},
		{Surface: surfacegeom.Plane{Frame: frame}, Retained: true, HasAxialDelta: true, AxialDelta: 0.5,
			CapRecord: &topo.CapRecord{}},
		{Surface: surfacegeom.Cylinder{Axis: r3.NewVec(0, 0, 1), Radius: units.Millimeters(4)}, Retained: true},
	}
	got, err := facetedrecord.ScaleImage(t.Context(), solid, src, anchor, 2)
	require.NoError(t, err)
	require.Len(t, got.Groups, 4)

	require.True(t, got.Groups[0].HasAxialDelta)
	require.GreaterOrEqual(t, got.Groups[0].AxialDelta, 0.5)
	require.InDelta(t, 0.5, got.Groups[0].AxialDelta, 1e-15)
	require.True(t, got.Groups[1].HasAxialDelta, "a record-less displaced plane keeps its displacement")
	require.InDelta(t, 0.25, got.Groups[1].AxialDelta, 1e-15)
	require.False(t, got.Groups[2].HasAxialDelta, "a displacement against a record drops with the record")
	require.Nil(t, got.Groups[2].CapRecord)
	require.False(t, got.Groups[3].HasAxialDelta, "an uncharged cylinder stays uncharged")
}
