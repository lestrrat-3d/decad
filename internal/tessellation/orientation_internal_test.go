package tessellation

import (
	"math"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// legacyOrientationSum is the signed tetrahedron sum as a DyAdd chain, one
// exact addition per triangle.
func legacyOrientationSum(vertices []r3.Vec, triangles [][3]int, anchor r3.Vec) proof.Dyadic {
	xa := proof.DyVec(anchor)
	sum := proof.DyZero()
	for _, t := range triangles {
		a := proof.DvSub(proof.DyVec(vertices[t[0]]), xa)
		b := proof.DvSub(proof.DyVec(vertices[t[1]]), xa)
		c := proof.DvSub(proof.DyVec(vertices[t[2]]), xa)
		sum = proof.DyAdd(sum, proof.DvDot(a, proof.DvCross(b, c)))
	}
	return sum
}

// orientationUVSphere is a closed sphere of 2*slices*(stacks-1) triangles.
func orientationUVSphere(slices, stacks int) ([]r3.Vec, [][3]int) {
	verts := []r3.Vec{{Z: 1}, {Z: -1}}
	for i := 1; i < stacks; i++ {
		phi := math.Pi * float64(i) / float64(stacks)
		for j := range slices {
			theta := 2 * math.Pi * float64(j) / float64(slices)
			verts = append(verts, r3.Vec{
				X: math.Sin(phi) * math.Cos(theta),
				Y: math.Sin(phi) * math.Sin(theta),
				Z: math.Cos(phi),
			})
		}
	}
	ring := func(i, j int) int { return 2 + (i-1)*slices + j%slices }
	var tris [][3]int
	for j := range slices {
		tris = append(tris, [3]int{0, ring(1, j), ring(1, j+1)})
		tris = append(tris, [3]int{1, ring(stacks-1, j+1), ring(stacks-1, j)})
	}
	for i := 1; i < stacks-1; i++ {
		for j := range slices {
			tris = append(tris,
				[3]int{ring(i, j), ring(i+1, j), ring(i+1, j+1)},
				[3]int{ring(i, j), ring(i+1, j+1), ring(i, j+1)})
		}
	}
	return verts, tris
}

func reverseTriangles(tris [][3]int) [][3]int {
	out := make([][3]int, len(tris))
	for i, t := range tris {
		out[i] = [3]int{t[0], t[2], t[1]}
	}
	return out
}

func TestOrientationSumMatchesTheDyAddChain(t *testing.T) {
	t.Parallel()
	sv, st := orientationUVSphere(50, 26)
	require.Len(t, st, 2500)
	rev := reverseTriangles(st)

	// The sphere followed by its reversed copy sums to exactly zero.
	pairVerts := append(append([]r3.Vec{}, sv...), sv...)
	pairTris := append([][3]int{}, st...)
	for _, tri := range rev {
		pairTris = append(pairTris, [3]int{tri[0] + len(sv), tri[1] + len(sv), tri[2] + len(sv)})
	}

	rng := rand.New(rand.NewPCG(11, 13))
	spread := func() float64 {
		return (rng.Float64()*2 - 1) * math.Ldexp(1, rng.IntN(61)-30)
	}
	soupVerts := make([]r3.Vec, 900)
	for i := range soupVerts {
		soupVerts[i] = r3.Vec{X: spread(), Y: spread(), Z: spread()}
	}
	soupTris := make([][3]int, 3000)
	for i := range soupTris {
		soupTris[i] = [3]int{rng.IntN(900), rng.IntN(900), rng.IntN(900)}
	}

	anchors := []r3.Vec{{}, {X: 1e3, Y: -2, Z: 0.5}}
	cases := []struct {
		name  string
		verts []r3.Vec
		tris  [][3]int
		sign  int // 2 when the sign is not pinned
	}{
		{"sphere", sv, st, 2},
		{"reversed sphere", sv, rev, 2},
		{"sphere and its reverse", pairVerts, pairTris, 0},
		{"soup", soupVerts, soupTris, 2},
	}
	for _, tc := range cases {
		for _, anchor := range anchors {
			want := legacyOrientationSum(tc.verts, tc.tris, anchor)
			for _, workers := range []int{1, 4} {
				got := orientationSum(tc.verts, tc.tris, anchor, workers)
				require.Zero(t, proof.DyCmp(want, got), "%s workers %d", tc.name, workers)
			}
			require.Equal(t, want.Sign(), OrientationSign(tc.verts, tc.tris, anchor), tc.name)
			if tc.sign != 2 {
				require.Equal(t, tc.sign, want.Sign(), tc.name)
			}
		}
	}

	// The two orientations of the sphere have opposite non-zero signs.
	fwd := OrientationSign(sv, st, r3.Vec{})
	require.NotZero(t, fwd)
	require.Equal(t, -fwd, OrientationSign(sv, rev, r3.Vec{}))
}

func TestOrientationSumPanicsOnANonFiniteVertexInBothPaths(t *testing.T) {
	t.Parallel()
	verts, tris := orientationUVSphere(50, 26)
	verts[len(verts)/2].Y = math.NaN()
	for _, workers := range []int{1, 4} {
		require.Panics(t, func() { legacyOrientationSum(verts, tris, r3.Vec{}) })
		require.Panics(t, func() { orientationSum(verts, tris, r3.Vec{}, workers) }, "workers %d", workers)
	}
}
