package facetproof_test

import (
	"math"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/decad/internal/facetproof"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// The Dyadic affine image spells the same homogeneous point as the big.Rat
// image meshbool.ExactRigidImage builds, and XptDenotes agrees with the sign of
// CoordDistance.
func TestExactAffineImageMatchesRigidImage(t *testing.T) {
	t.Parallel()
	r := rand.New(rand.NewPCG(5, 9))
	for i := range 1500 {
		axis := r3.Vec{X: r.Float64() - 0.5, Y: r.Float64() - 0.5, Z: r.Float64() - 0.5}
		rot, err := r3.Rotation(axis, units.Degrees(r.Float64()*360))
		require.NoError(t, err)
		tr := r3.Vec{X: (r.Float64() - 0.5) * 20, Y: (r.Float64() - 0.5) * 20, Z: (r.Float64() - 0.5) * 20}
		if i%7 == 0 {
			tr = r3.Vec{}
		}
		if i%11 == 0 {
			rot = r3.Identity()
		}
		shift, err := r3.Translation(tr)
		require.NoError(t, err)
		xf, err := shift.Then(rot)
		require.NoError(t, err)
		b := xf.Basis()
		scale := math.Ldexp(1, r.IntN(20)-10)
		p := r3.Vec{X: (r.Float64() - 0.5) * scale, Y: (r.Float64() - 0.5) * scale, Z: (r.Float64() - 0.5) * scale}
		if i%5 == 0 {
			p.Y = 0
		}
		want := meshbool.ExactRigidImage(b, xf.Translation(), p)
		got := proof.ExactAffineImage(b.EX, b.EY, b.EZ, xf.Translation(), p)
		require.Equal(t, 0, want.X.Cmp(got.X))
		require.Equal(t, 0, want.Y.Cmp(got.Y))
		require.Equal(t, 0, want.Z.Cmp(got.Z))
		require.Equal(t, 0, want.W.Cmp(got.W))
		held := got.Vec()
		require.Equal(t, meshbool.CoordDistance(got, held).Sign() == 0, proof.XptDenotes(got, held))
		require.Equal(t, meshbool.CoordDistance(got, p).Sign() == 0, proof.XptDenotes(got, p))
	}
}

// PlaceMesh reads its area-round terms from the area pass; the resulting
// AreaSlack equals the per-facet loop it replaced, bit for bit.
func TestPlaceMeshAreaRoundMatchesPerFacetLoop(t *testing.T) {
	t.Parallel()
	verts := make([]r3.Vec, 0, 8)
	for i := range 8 {
		verts = append(verts, r3.Vec{X: float64(i&1) * 3.3, Y: float64(i>>1&1) * 0.7, Z: float64(i>>2&1) * 12.9})
	}
	tris := [][3]int{
		{0, 2, 1}, {1, 2, 3}, {4, 5, 6}, {5, 7, 6}, {0, 1, 4}, {1, 5, 4},
		{2, 6, 3}, {3, 6, 7}, {0, 4, 2}, {2, 4, 6}, {1, 3, 5}, {3, 7, 5},
	}
	for _, reflect := range []bool{false, true} {
		rot, err := r3.Rotation(r3.Vec{X: 0.3, Y: 1, Z: -0.2}, units.Degrees(37))
		require.NoError(t, err)
		if reflect {
			fr, err := r3.NewFrame(r3.Vec{}, r3.Vec{Y: 1}, r3.Vec{Z: 1})
			require.NoError(t, err)
			m, err := r3.Reflection(fr)
			require.NoError(t, err)
			rot, err = rot.Then(m)
			require.NoError(t, err)
		}
		shift, err := r3.Translation(r3.Vec{X: 1.5, Y: -2.25, Z: 0.125})
		require.NoError(t, err)
		xf, err := shift.Then(rot)
		require.NoError(t, err)
		placed, err := facetproof.PlaceMesh(t.Context(), proofbound.NewWorkBudget(t.Context()), verts, tris,
			make([]float64, len(verts)), 0, 0, 0, 0, xf)
		require.NoError(t, err)
		// With zero source bounds the vertex bound is the per-vertex allow.
		allow := 0.0
		for _, b := range placed.VertexBound {
			allow = math.Max(allow, b)
		}
		areaRound := 0.0
		for _, tri := range placed.Tris {
			a, b, c := placed.Verts[tri[0]], placed.Verts[tri[1]], placed.Verts[tri[2]]
			u, v := proofbound.HeldDelta(b, a), proofbound.HeldDelta(c, a)
			term := proofbound.AbsSumUpper(
				proofbound.ProductUpper(allow,
					proofbound.AbsSumUpper(proofbound.DvLenUpper(u), proofbound.DvLenUpper(v))),
				proofbound.ProductUpper(2, proofbound.ProductUpper(allow, allow)))
			areaRound = proofbound.AbsSumUpper(areaRound, term)
		}
		heldArea := facetproof.MeshAreaUpper(verts, tris)
		want := proofbound.AbsSumUpper(0, proofbound.ProductUpper(0, 0),
			proofbound.ProductUpper(2, proofbound.ProductUpper(0, heldArea)), areaRound)
		require.Equal(t, math.Float64bits(want), math.Float64bits(placed.AreaSlack), "reflect=%v", reflect)
	}
}
