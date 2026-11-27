package meshbool

// CI: isolated-tests

import (
	"math"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// TestSimplifyPlanarFacesMatchesMapReference runs simplifyPlanarFaces and the
// map-based copy it replaced on boxes of several grid sizes, with source faces
// merged, split or relabelled at random, facets reversed, rotated, duplicated
// or dropped (open and non-manifold meshes), and vertices moved or claimed. The
// two must agree on the result, the changed flag and the error, byte for byte.
func TestSimplifyPlanarFacesMatchesMapReference(t *testing.T) {
	t.Parallel()
	bases := []PlanarMesh{planarBox(0), planarBox(1e-9), tentWithCollinearFirst()}
	for _, n := range []int{1, 2, 3, 4} {
		bases = append(bases, simplifyGridBox(n, false), simplifyGridBox(n, true))
	}
	var changedCount, unchangedClosed, mutated int
	for seed := range uint64(300) {
		rng := rand.New(rand.NewPCG(seed, 0x51))
		in := clonePlanarMesh(bases[rng.IntN(len(bases))])
		if seed >= uint64(len(bases)) { // the first runs keep each base as given
			for range rng.IntN(4) {
				mutated++
				f := rng.IntN(len(in.Tris))
				switch rng.IntN(7) {
				case 0:
					in.Src[f] = rng.IntN(8)
				case 1:
					in.Tris[f] = [3]int{in.Tris[f][0], in.Tris[f][2], in.Tris[f][1]}
				case 2:
					tr := in.Tris[f]
					in.Tris[f] = [3]int{tr[1], tr[2], tr[0]}
				case 3:
					in.Tris = append(in.Tris, in.Tris[f])
					in.Src = append(in.Src, in.Src[f])
					in.FacetBound = append(in.FacetBound, 0)
				case 4:
					in.Tris = append(in.Tris[:f], in.Tris[f+1:]...)
					in.Src = append(in.Src[:f], in.Src[f+1:]...)
					in.FacetBound = append(in.FacetBound[:f], in.FacetBound[f+1:]...)
					if len(in.Tris) == 0 {
						in = clonePlanarMesh(bases[0])
					}
				case 5:
					v := rng.IntN(len(in.Xverts))
					c := in.Approx[v].Add(r3.Vec{X: 1.0 / 1024, Y: 1.0 / 2048, Z: 1.0 / 4096})
					in.Xverts[v], in.Approx[v] = proof.XptOf(c), c
				case 6:
					in.Xbeta[rng.IntN(len(in.Xbeta))] = 1e-9
				}
			}
		}
		want, wantChanged, wantErr := mapRefSimplifyPlanarFaces(t.Context(),
			proofbound.NewWorkBudget(t.Context()), clonePlanarMesh(in))
		got, gotChanged, gotErr := simplifyPlanarFaces(t.Context(),
			proofbound.NewWorkBudget(t.Context()), clonePlanarMesh(in))
		require.Equal(t, wantErr == nil, gotErr == nil, "seed %d", seed)
		require.Equal(t, wantChanged, gotChanged, "seed %d", seed)
		require.Equal(t, planarMeshDigest(want), planarMeshDigest(got), "seed %d", seed)
		switch {
		case gotChanged:
			changedCount++
		case stitchEdgesPaired(in.Tris, map[[2]int]uint8{}):
			unchangedClosed++
		}
	}
	require.Positive(t, changedCount, "some input is simplified")
	require.Positive(t, unchangedClosed, "some closed input is left alone")
	require.Positive(t, mutated, "some input is mutated")
}

// TestFlipNeedlesHeldMatchesMapReference runs flipNeedlesHeld and the
// map-based copy it replaced on random facet pairs over points that sit on or a
// few float steps off a line, so needles are common, with duplicate and
// shared edges, repeated vertices and mixed sources. Results, flip counts,
// bounds and the facet list must agree exactly.
func TestFlipNeedlesHeldMatchesMapReference(t *testing.T) {
	t.Parallel()
	tiny := math.Ldexp(1, -50)
	var flipped, rejected int
	for seed := range uint64(300) {
		rng := rand.New(rand.NewPCG(seed, 0xf1))
		var pts []r3.Vec
		for x := range 6 {
			pts = append(pts, r3.Vec{X: float64(x), Y: tiny * float64(rng.IntN(3))})
		}
		pts = append(pts, r3.Vec{X: 2, Y: -3}, r3.Vec{X: 3, Y: 3}, r3.Vec{X: 1, Y: 2, Z: 1}, r3.Vec{X: 2.5, Y: -2})
		verts := make([]proof.Xpt, len(pts))
		for i, p := range pts {
			verts[i] = proof.XptOf(p)
		}
		var tris [][3]int
		var src []int
		var bounds []float64
		add := func(tri [3]int, s int) {
			tris, src = append(tris, tri), append(src, s)
			bounds = append(bounds, float64(len(tris)))
		}
		for range 2 + rng.IntN(8) {
			u, w, b, d := rng.IntN(len(pts)), rng.IntN(len(pts)), rng.IntN(len(pts)), rng.IntN(len(pts))
			s := rng.IntN(2)
			add([3]int{u, w, b}, s)
			if rng.IntN(6) > 0 {
				add([3]int{w, u, d}, s)
			}
			if rng.IntN(5) == 0 {
				add([3]int{u, w, d}, rng.IntN(2)) // a second facet on the directed edge
			}
		}
		held := make([]r3.Vec, len(verts))
		for i, p := range verts {
			held[i] = p.Vec()
		}
		wantT, wantB := append([][3]int(nil), tris...), append([]float64(nil), bounds...)
		gotT, gotB := append([][3]int(nil), tris...), append([]float64(nil), bounds...)
		want, wantErr := mapRefFlipNeedlesHeld(t.Context(), verts, held, wantT, src, wantB)
		got, gotErr := flipNeedlesHeld(t.Context(), verts, held, gotT, src, gotB)
		require.NoError(t, wantErr)
		require.NoError(t, gotErr)
		require.Equal(t, want, got, "seed %d", seed)
		require.Equal(t, wantT, gotT, "seed %d", seed)
		require.Equal(t, wantB, gotB, "seed %d", seed)
		if got > 0 {
			flipped++
		} else {
			rejected++
		}
	}
	require.Positive(t, flipped, "some input flips")
	require.Positive(t, rejected, "some input does not flip")
}
