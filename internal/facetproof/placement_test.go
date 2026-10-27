package facetproof_test

import (
	"math"
	"math/big"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/exactrotation"
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

func tetraMesh() ([]r3.Vec, [][3]int) {
	verts := []r3.Vec{{X: 1.1, Y: 0.3, Z: -0.7}, {X: 9.3, Y: 0.1, Z: 0.2}, {X: 0.4, Y: 7.7, Z: 0.9}, {X: 0.6, Y: 0.8, Z: 6.1}}
	return verts, [][3]int{{0, 2, 1}, {0, 1, 3}, {0, 3, 2}, {1, 2, 3}}
}

// meshVolume is the exact signed volume of a closed mesh of float vertices.
func meshVolume(verts []r3.Vec, tris [][3]int) *big.Rat {
	vol := new(big.Rat)
	for _, tri := range tris {
		var p [3][3]*big.Rat
		for k, vi := range tri {
			v := verts[vi]
			p[k] = [3]*big.Rat{proof.FloatRat(v.X), proof.FloatRat(v.Y), proof.FloatRat(v.Z)}
		}
		mul := func(a, b *big.Rat) *big.Rat { return new(big.Rat).Mul(a, b) }
		sub := func(a, b *big.Rat) *big.Rat { return new(big.Rat).Sub(a, b) }
		cx := sub(mul(p[1][1], p[2][2]), mul(p[1][2], p[2][1]))
		cy := sub(mul(p[1][2], p[2][0]), mul(p[1][0], p[2][2]))
		cz := sub(mul(p[1][0], p[2][1]), mul(p[1][1], p[2][0]))
		triple := new(big.Rat).Add(mul(p[0][0], cx), new(big.Rat).Add(mul(p[0][1], cy), mul(p[0][2], cz)))
		vol.Add(vol, triple)
	}
	return vol.Quo(vol, big.NewRat(6, 1))
}

// The placed vertices of a PlaceMeshDenoting result lie within their vertex
// bounds of Q*v + t, with d any float transform within the stated gap of the
// exact record, and the volume change stays within VolSymDiff. A d that is NOT
// the record's own float transform makes the gap term load-bearing.
func TestPlaceMeshDenotingCoversExactImage(t *testing.T) {
	t.Parallel()
	verts, tris := tetraMesh()
	for _, tc := range []struct {
		name   string
		offset float64
	}{{"own float transform", 0}, {"transform off by 1e-6 rad", 1e-6}} {
		t.Run(tc.name, func(t *testing.T) {
			center, axis, theta := r3.Vec{X: 2, Y: -1, Z: 3}, r3.Vec{X: 1, Y: 2, Z: 2}, 1.1
			rec, _, err := exactrotation.New(center, axis, units.Radians(theta))
			require.NoError(t, err)
			d, err := r3.RotationAround(center, axis, units.Radians(theta+tc.offset))
			require.NoError(t, err)
			linear, translation, ok := rec.Gap(d)
			require.True(t, ok)
			bounds := make([]float64, len(verts))
			placed, err := facetproof.PlaceMeshDenoting(t.Context(), proofbound.NewWorkBudget(t.Context()),
				verts, tris, bounds, 0, 0, d, linear, translation)
			require.NoError(t, err)
			m, tr := rec.Linear(), rec.Translation()
			for i, v := range verts {
				vv := [3]*big.Rat{proof.FloatRat(v.X), proof.FloatRat(v.Y), proof.FloatRat(v.Z)}
				h := placed.Verts[i]
				held := [3]*big.Rat{proof.FloatRat(h.X), proof.FloatRat(h.Y), proof.FloatRat(h.Z)}
				dist2 := new(big.Rat)
				for r := range 3 {
					img := new(big.Rat).Set(tr[r])
					for c := range 3 {
						img.Add(img, new(big.Rat).Mul(m[r][c], vv[c]))
					}
					diff := img.Sub(img, held[r])
					dist2.Add(dist2, diff.Mul(diff, diff))
				}
				bound := proof.FloatRat(placed.VertexBound[i])
				require.LessOrEqual(t, dist2.Cmp(bound.Mul(bound, bound)), 0,
					"vertex %d lies within its bound of Q*v + t", i)
			}
			// Q keeps volume, so the exact source volume is the true one.
			change := new(big.Rat).Sub(meshVolume(placed.Verts, placed.Tris), meshVolume(verts, tris))
			require.LessOrEqual(t, change.Abs(change).Cmp(proof.FloatRat(placed.VolSymDiff)), 0)
			require.False(t, math.IsInf(placed.MeshBound, 0) || math.IsNaN(placed.MeshBound))
			require.False(t, math.IsInf(placed.AreaSlack, 0) || math.IsNaN(placed.AreaSlack))
		})
	}
}

// The gap term is charged per vertex: a larger gap widens every vertex bound.
func TestPlaceMeshDenotingChargesGapPerVertex(t *testing.T) {
	t.Parallel()
	verts, tris := tetraMesh()
	d, err := r3.RotationAround(r3.Vec{}, r3.Vec{Z: 1}, units.Radians(0.5))
	require.NoError(t, err)
	place := func(linear, translation float64) facetproof.PlacedMesh {
		placed, err := facetproof.PlaceMeshDenoting(t.Context(), proofbound.NewWorkBudget(t.Context()),
			verts, tris, make([]float64, len(verts)), 0, 0, d, linear, translation)
		require.NoError(t, err)
		return placed
	}
	base, wide := place(0, 0), place(1e-3, 1e-3)
	for i, v := range verts {
		maxAbs := math.Max(math.Abs(v.X), math.Max(math.Abs(v.Y), math.Abs(v.Z)))
		// The gap is at least linear*max|v| + translation, independent of the
		// float rounding allowance that base already holds.
		require.GreaterOrEqual(t, wide.VertexBound[i], 1e-3*maxAbs+1e-3)
		require.Greater(t, wide.VertexBound[i], base.VertexBound[i])
	}
	require.Greater(t, wide.VolSymDiff, base.VolSymDiff)
	require.Greater(t, wide.AreaSlack, base.AreaSlack)
}

// PlaceMeshDenoting places the vertices exactly as PlaceMesh does and keeps the
// source's volume and area charges apart from the placement's own rounding: it
// adds no stretch or determinant term.
func TestPlaceMeshDenotingPlacesVerticesAsPlaceMesh(t *testing.T) {
	t.Parallel()
	verts, tris := tetraMesh()
	d, err := r3.RotationAround(r3.Vec{}, r3.Vec{X: 1}, units.Radians(math.Pi/2))
	require.NoError(t, err)
	zero := make([]float64, len(verts))
	want, err := facetproof.PlaceMesh(t.Context(), proofbound.NewWorkBudget(t.Context()), verts, tris, zero, 0, 0, 0, 0, d)
	require.NoError(t, err)
	got, err := facetproof.PlaceMeshDenoting(t.Context(), proofbound.NewWorkBudget(t.Context()),
		verts, tris, zero, 0, 0, d, 0, 0)
	require.NoError(t, err)
	require.Equal(t, want.Verts, got.Verts)
	require.Equal(t, want.Tris, got.Tris)
	// Zero gaps add no term beyond the one-ulp outward rounding of the sum.
	for i := range want.VertexBound {
		require.InDelta(t, want.VertexBound[i], got.VertexBound[i], 1e-20)
	}
}

func TestPlaceMeshDenotingRefusesBadGap(t *testing.T) {
	t.Parallel()
	verts, tris := tetraMesh()
	for _, gap := range [][2]float64{{-1, 0}, {0, -1}, {math.Inf(1), 0}, {0, math.NaN()}} {
		_, err := facetproof.PlaceMeshDenoting(t.Context(), proofbound.NewWorkBudget(t.Context()), verts, tris,
			make([]float64, len(verts)), 0, 0, r3.Identity(), gap[0], gap[1])
		require.ErrorIs(t, err, decaderr.ErrUnsupported, "gap %v", gap)
	}
}

// A vertex whose exact image is a float pays no rounding, and one whose image is not pays its allowance.
func TestPlaceMeshChargesNoRoundingForAFloatImage(t *testing.T) {
	t.Parallel()
	verts := []r3.Vec{{}, {X: 3}, {Y: 5}, {Z: 7}}
	tris := [][3]int{{0, 2, 1}, {0, 1, 3}, {0, 3, 2}, {1, 2, 3}}
	place := func(shift r3.Vec) facetproof.PlacedMesh {
		move, err := r3.Translation(shift)
		require.NoError(t, err)
		placed, err := facetproof.PlaceMesh(t.Context(), proofbound.NewWorkBudget(t.Context()), verts, tris,
			make([]float64, len(verts)), 0, 0, 0, 0, move)
		require.NoError(t, err)
		return placed
	}
	for _, b := range place(r3.Vec{X: 100, Y: -4, Z: 0.5}).VertexBound {
		require.Zero(t, b, "whole and dyadic translations of whole coordinates are exact")
	}
	// 0.1 is not a dyadic with a short mantissa: 3 + 0.1 rounds, and the coordinate 0 + 0.1 does not.
	paid := 0
	for _, b := range place(r3.Vec{X: 0.1, Y: 0.1, Z: 0.1}).VertexBound {
		if b > 0 {
			paid++
		}
	}
	require.Positive(t, paid, "a translation whose sums round pays for them")
}
