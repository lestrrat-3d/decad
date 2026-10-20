package planarband_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/planarband"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func squarePrism(t *testing.T) prismrecord.Payload {
	t.Helper()
	frame, err := r3.NewFrame(r3.Vec{X: 10}, r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	corners := []sectionrecord.Point2{{U: 0, V: 0}, {U: 2, V: 0}, {U: 2, V: 2}, {U: 0, V: 2}}
	var segs []sectionrecord.CurveSegment
	for i, c := range corners {
		segs = append(segs, sectionrecord.LineSeg{Start: c, End: corners[(i+1)%len(corners)], TStart: 0, TEnd: 1})
	}
	return prismrecord.Payload{Frame: frame, Xform: r3.Identity(), Z0: 1, Z1: 4,
		Profile: momentinput.Profile{Outer: sectionrecord.LoopRecord{Segments: segs}}}
}

func TestPrismSolid(t *testing.T) {
	t.Parallel()

	budget := func() *proofbound.WorkBudget { return proofbound.NewWorkBudget(t.Context()) }
	roles := map[string]int{provenance.RoleCapStart: 0, provenance.RoleCapEnd: 1,
		"side(0,0)": 2, "side(0,1)": 3, "side(0,2)": 4, "side(0,3)": 5}

	t.Run("lifts the section through the frame and levels", func(t *testing.T) {
		t.Parallel()
		solid, ok, err := planarband.PrismSolid(t.Context(), budget(), squarePrism(t), roles)
		require.NoError(t, err)
		require.True(t, ok)
		require.Len(t, solid.Verts, 8)
		zs := map[float64]int{}
		for i, v := range solid.Verts {
			x, _ := v[0].Float64()
			y, _ := v[1].Float64()
			z, _ := v[2].Float64()
			zs[z]++
			require.GreaterOrEqual(t, x, 10.0, "vertex %d", i)
			require.LessOrEqual(t, x, 12.0, "vertex %d", i)
			require.GreaterOrEqual(t, y, 0.0, "vertex %d", i)
			require.LessOrEqual(t, y, 2.0, "vertex %d", i)
		}
		require.Equal(t, map[float64]int{1: 4, 4: 4}, zs, "four vertices at each level")
		require.Equal(t, len(solid.Tris), len(solid.Faces), "every triangle names its face through the roles")
		require.Equal(t, []int{0, 1}, solid.Faces[:2], "the first cap triangle pair is the start cap then the end cap")
	})
	t.Run("leaves the faces unset without the roles", func(t *testing.T) {
		t.Parallel()
		solid, ok, err := planarband.PrismSolid(t.Context(), budget(), squarePrism(t), nil)
		require.NoError(t, err)
		require.True(t, ok)
		require.Nil(t, solid.Faces)
	})
	t.Run("refuses a displaced or open prism", func(t *testing.T) {
		t.Parallel()
		for name, edit := range map[string]func(*prismrecord.Payload){
			"section displacement": func(p *prismrecord.Payload) { p.SectionDelta = 1e-3 },
			"start displacement":   func(p *prismrecord.Payload) { p.Z0Delta = 1e-3 },
			"end displacement":     func(p *prismrecord.Payload) { p.Z1Delta = 1e-3 },
			"surface result":       func(p *prismrecord.Payload) { p.SurfaceResult = true },
			"levels reversed":      func(p *prismrecord.Payload) { p.Z0, p.Z1 = p.Z1, p.Z0 },
			"hole beside the outer": func(p *prismrecord.Payload) {
				p.Profile.Holes = []sectionrecord.LoopRecord{p.Profile.Outer}
			},
		} {
			p := squarePrism(t)
			edit(&p)
			_, ok, err := planarband.PrismSolid(t.Context(), budget(), p, roles)
			require.NoError(t, err, name)
			require.False(t, ok, name)
		}
	})
}

func TestFacetedSolid(t *testing.T) {
	t.Parallel()

	budget := func() *proofbound.WorkBudget { return proofbound.NewWorkBudget(t.Context()) }
	mesh := func() facetedrecord.Payload {
		return facetedrecord.Payload{
			Verts:  []r3.Vec{{}, {X: 1}, {Y: 1}, {Z: 1}},
			Tris:   [][3]int{{0, 2, 1}, {0, 1, 3}, {0, 3, 2}, {1, 2, 3}},
			FaceOf: []int{0, 1, 2, 3},
			Xform:  r3.Identity(),
		}
	}

	t.Run("an exact mesh is its own boundary", func(t *testing.T) {
		t.Parallel()
		solid, delta, ok, err := planarband.FacetedSolid(budget(), mesh())
		require.NoError(t, err)
		require.True(t, ok)
		require.Len(t, solid.Verts, 4)
		require.Equal(t, []int{0, 1, 2, 3}, solid.Faces)
		require.Zero(t, delta.Sign())
	})
	t.Run("a bounded mesh reports its bound as the displacement", func(t *testing.T) {
		t.Parallel()
		p := mesh()
		p.MeshBound = 0.25
		_, delta, ok, err := planarband.FacetedSolid(budget(), p)
		require.NoError(t, err)
		require.True(t, ok)
		got, exact := delta.Float64()
		require.True(t, exact)
		require.Equal(t, 0.25, got)
	})
	t.Run("an empty mesh is refused", func(t *testing.T) {
		t.Parallel()
		p := mesh()
		p.Tris = nil
		_, _, ok, err := planarband.FacetedSolid(budget(), p)
		require.NoError(t, err)
		require.False(t, ok)
	})
	t.Run("a volume difference without a mesh bound is refused", func(t *testing.T) {
		t.Parallel()
		p := mesh()
		p.VolSymDiff = 0.1
		_, _, ok, err := planarband.FacetedSolid(budget(), p)
		require.NoError(t, err)
		require.False(t, ok)
	})
	t.Run("a mesh bound that is not finite is refused", func(t *testing.T) {
		t.Parallel()
		p := mesh()
		p.MeshBound = -1
		_, _, ok, err := planarband.FacetedSolid(budget(), p)
		require.NoError(t, err)
		require.False(t, ok)
	})
	t.Run("the exact source is read under a translation", func(t *testing.T) {
		t.Parallel()
		p := mesh()
		p.MeshBound = 0.5
		p.ExactSourceVerts = []r3.Vec{{}, {X: 1}, {Y: 1}, {Z: 1}}
		p.ExactSourceTris = p.Tris
		_, delta, ok, err := planarband.FacetedSolid(budget(), p)
		require.NoError(t, err)
		require.True(t, ok)
		require.Zero(t, delta.Sign(), "the saved exact mesh is a zero-bound boundary")
	})
}
