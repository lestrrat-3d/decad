package facetproof_test

import (
	"math"
	"math/big"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/decad/internal/facetproof"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestFirstMomentsMatchExactFacetFormula(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewPCG(0x57dfe12a, 0x4d75b3af))
	verts := []r3.Vec{{}, {X: 1}, {Y: 1}}
	for range 50 {
		coordinate := func() float64 {
			return math.Ldexp(2*rng.Float64()-1, rng.IntN(80)-40)
		}
		verts = append(verts, r3.Vec{X: coordinate(), Y: coordinate(), Z: coordinate()})
	}
	xverts := make([]proof.Xpt, len(verts))
	for i, v := range verts {
		xverts[i] = proof.XptOf(v)
	}
	tris := [][3]int{{0, 1, 2}}
	for range 100 {
		a, b, c := rng.IntN(len(verts)), rng.IntN(len(verts)), rng.IntN(len(verts))
		if a != b && b != c && c != a {
			tris = append(tris, [3]int{a, b, c})
		}
	}
	want := [3]*big.Rat{new(big.Rat), new(big.Rat), new(big.Rat)}
	for _, tri := range tris {
		a, b, c := xverts[tri[0]], xverts[tri[1]], xverts[tri[2]]
		det := proof.XdotRat(a, proof.Xcross(b, c))
		ax, ay, az := proof.XhpRat(proof.Xhp(a))
		bx, by, bz := proof.XhpRat(proof.Xhp(b))
		cx, cy, cz := proof.XhpRat(proof.Xhp(c))
		for axis, coords := range [3][3]*big.Rat{{ax, bx, cx}, {ay, by, cy}, {az, bz, cz}} {
			sum := new(big.Rat).Add(coords[0], coords[1])
			sum.Add(sum, coords[2])
			want[axis].Add(want[axis], new(big.Rat).Mul(det, sum))
		}
	}
	got, err := facetproof.FirstMoments(proofbound.NewWorkBudget(t.Context()), xverts, tris)
	require.NoError(t, err)
	for axis := range got {
		require.Zero(t, got[axis].Cmp(want[axis]), "axis %d", axis)
	}
	scaled := append([]proof.Xpt(nil), xverts...)
	three := big.NewInt(3)
	p := scaled[3]
	scaled[3] = proof.Xpt{
		X: new(big.Int).Mul(p.X, three), Y: new(big.Int).Mul(p.Y, three),
		Z: new(big.Int).Mul(p.Z, three), W: new(big.Int).Mul(p.W, three),
	}
	got, err = facetproof.FirstMoments(proofbound.NewWorkBudget(t.Context()), scaled, tris)
	require.NoError(t, err)
	for axis := range got {
		require.Zero(t, got[axis].Cmp(want[axis]), "non-dyadic axis %d", axis)
	}
}

// testBox is an outward-oriented box appended to verts and tris. inward
// reverses the facets so the box reads as a void.
func testBox(verts []r3.Vec, tris [][3]int, lo, hi r3.Vec, inward bool) ([]r3.Vec, [][3]int) {
	base := len(verts)
	for i := range 8 {
		p := lo
		if i&1 != 0 {
			p.X = hi.X
		}
		if i&2 != 0 {
			p.Y = hi.Y
		}
		if i&4 != 0 {
			p.Z = hi.Z
		}
		verts = append(verts, p)
	}
	for _, t := range [][3]int{
		{0, 2, 1}, {1, 2, 3}, {4, 5, 6}, {5, 7, 6}, {0, 1, 4}, {1, 5, 4},
		{2, 6, 3}, {3, 6, 7}, {0, 4, 2}, {2, 4, 6}, {1, 3, 5}, {3, 7, 5},
	} {
		if inward {
			t[1], t[2] = t[2], t[1]
		}
		tris = append(tris, [3]int{base + t[0], base + t[1], base + t[2]})
	}
	return verts, tris
}

func TestVolumeFromAuditMatchesVolume(t *testing.T) {
	t.Parallel()
	cases := map[string]func() ([]r3.Vec, [][3]int){
		"box": func() ([]r3.Vec, [][3]int) {
			return testBox(nil, nil, r3.Vec{}, r3.Vec{X: 2, Y: 3, Z: 5}, false)
		},
		"non-dyadic box": func() ([]r3.Vec, [][3]int) {
			return testBox(nil, nil, r3.Vec{X: 0.1, Y: -0.3, Z: 0.7}, r3.Vec{X: 1.3, Y: 0.9, Z: 2.1}, false)
		},
		"box with void": func() ([]r3.Vec, [][3]int) {
			verts, tris := testBox(nil, nil, r3.Vec{}, r3.Vec{X: 10, Y: 10, Z: 10}, false)
			return testBox(verts, tris, r3.Vec{X: 2.5, Y: 3.1, Z: 4}, r3.Vec{X: 5, Y: 6.7, Z: 8}, true)
		},
		"two boxes": func() ([]r3.Vec, [][3]int) {
			verts, tris := testBox(nil, nil, r3.Vec{}, r3.Vec{X: 1, Y: 1, Z: 1}, false)
			return testBox(verts, tris, r3.Vec{X: 5.5, Y: 5.5, Z: 5.5}, r3.Vec{X: 7.25, Y: 6.125, Z: 9}, false)
		},
	}
	for name, build := range cases {
		t.Run(name, func(t *testing.T) {
			t.Parallel()
			verts, tris := build()
			audit, err := facetproof.AuditFacetedMesh(t.Context(), verts, tris)
			require.NoError(t, err)
			direct := new(big.Rat)
			for ci, members := range audit.Members {
				part := new(big.Rat)
				for _, fi := range members {
					tr := tris[fi]
					part.Add(part, proof.XdotRat(audit.XVerts[tr[0]],
						proof.Xcross(audit.XVerts[tr[1]], audit.XVerts[tr[2]])))
				}
				part.Mul(part, big.NewRat(1, 6))
				require.Zero(t, part.Cmp(audit.CompVol[ci]), "component %d", ci)
				direct.Add(direct, part)
			}
			wantF, wantBound, wantRat, err := facetproof.Volume(t.Context(), audit.XVerts, tris, 1e-9)
			require.NoError(t, err)
			require.Zero(t, direct.Cmp(wantRat))
			gotF, gotBound, gotRat, err := facetproof.VolumeFromAudit(audit, 1e-9)
			require.NoError(t, err)
			require.Zero(t, gotRat.Cmp(wantRat))
			require.Equal(t, math.Float64bits(wantF), math.Float64bits(gotF))
			require.Equal(t, math.Float64bits(wantBound), math.Float64bits(gotBound))
		})
	}
}
