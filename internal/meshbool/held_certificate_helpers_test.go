package meshbool

import (
	"errors"
	"math"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// certIcosphere is a subdivided icosahedron of the given radius centred at
// centre: 20·4^subdiv facets, every vertex shared, embedded.
func certIcosphere(subdiv int, radius float64, centre r3.Vec) ([]r3.Vec, [][3]int) {
	phi := (1 + math.Sqrt(5)) / 2
	verts := []r3.Vec{{X: -1, Y: phi}, {X: 1, Y: phi}, {X: -1, Y: -phi}, {X: 1, Y: -phi},
		{Y: -1, Z: phi}, {Y: 1, Z: phi}, {Y: -1, Z: -phi}, {Y: 1, Z: -phi},
		{X: phi, Z: -1}, {X: phi, Z: 1}, {X: -phi, Z: -1}, {X: -phi, Z: 1}}
	tris := [][3]int{{0, 11, 5}, {0, 5, 1}, {0, 1, 7}, {0, 7, 10}, {0, 10, 11}, {1, 5, 9}, {5, 11, 4},
		{11, 10, 2}, {10, 7, 6}, {7, 1, 8}, {3, 9, 4}, {3, 4, 2}, {3, 2, 6}, {3, 6, 8}, {3, 8, 9},
		{4, 9, 5}, {2, 4, 11}, {6, 2, 10}, {8, 6, 7}, {9, 8, 1}}
	for range subdiv {
		mid := map[[2]int]int{}
		midpoint := func(a, b int) int {
			key := [2]int{min(a, b), max(a, b)}
			if m, ok := mid[key]; ok {
				return m
			}
			verts = append(verts, verts[a].Add(verts[b]).Scale(0.5))
			mid[key] = len(verts) - 1
			return len(verts) - 1
		}
		var next [][3]int
		for _, t := range tris {
			ab, bc, ca := midpoint(t[0], t[1]), midpoint(t[1], t[2]), midpoint(t[2], t[0])
			next = append(next, [3]int{t[0], ab, ca}, [3]int{t[1], bc, ab}, [3]int{t[2], ca, bc}, [3]int{ab, bc, ca})
		}
		tris = next
	}
	out := make([]r3.Vec, len(verts))
	for i, v := range verts {
		out[i] = v.Scale(radius / v.Len()).Add(centre)
	}
	return out, tris
}

// certMerge appends the second mesh to the first.
func certMerge(av []r3.Vec, at [][3]int, bv []r3.Vec, bt [][3]int) ([]r3.Vec, [][3]int) {
	verts := append(append([]r3.Vec(nil), av...), bv...)
	tris := append([][3]int(nil), at...)
	for _, t := range bt {
		tris = append(tris, [3]int{t[0] + len(av), t[1] + len(av), t[2] + len(av)})
	}
	return verts, tris
}

// certRotationBasis is the float basis of a rotation about axis by degrees.
func certRotationBasis(t *testing.T, axis r3.Vec, degrees float64) r3.Basis {
	t.Helper()
	rot, err := r3.Rotation(axis, units.Degrees(degrees))
	require.NoError(t, err)
	return rot.Basis()
}

// certPlace is a placement's nearest-float image of the source under basis and
// translation, with the largest per-vertex allowance PlaceMesh would charge.
// A basis of negative determinant swaps two corners of each facet, as a
// reflection does.
func certPlace(basis r3.Basis, tr r3.Vec, verts []r3.Vec, tris [][3]int) ([]r3.Vec, [][3]int, float64) {
	maxTrans := math.Max(math.Abs(tr.X), math.Max(math.Abs(tr.Y), math.Abs(tr.Z)))
	held := make([]r3.Vec, len(verts))
	allow := 0.0
	for i, v := range verts {
		held[i] = proof.ExactAffineImage(basis.EX, basis.EY, basis.EZ, tr, v).Vec()
		m := math.Max(math.Abs(v.X), math.Max(math.Abs(v.Y), math.Abs(v.Z)))
		allow = max(allow, proofbound.RigidRoundAllow(m, maxTrans))
	}
	out := tris
	if basis.EX.Dot(basis.EY.Cross(basis.EZ)) < 0 {
		out = make([][3]int, len(tris))
		for i, t := range tris {
			out[i] = [3]int{t[0], t[2], t[1]}
		}
	}
	return held, out, allow
}

// certScanClean is today's verdict on a copy: the held embedding scan, with
// every vertex marked moved, finds nothing to move and nothing to refuse.
func certScanClean(t *testing.T, held []r3.Vec, tris [][3]int) bool {
	t.Helper()
	h := HeldRounding{Verts: slicesCloneVecs(held), Tris: tris, Moved: make([]bool, len(held)),
		Movable: make([]bool, len(held))}
	for i := range h.Moved {
		h.Moved[i] = true
	}
	n, summary, err := EnforceHeldEmbeddingWithSummary(t.Context(), h)
	if fold := (*HeldFoldError)(nil); errors.As(err, &fold) {
		return false
	}
	require.NoError(t, err)
	return n == 0 && summary == nil
}

func slicesCloneVecs(v []r3.Vec) []r3.Vec { return append([]r3.Vec(nil), v...) }

// certRandomSoup is count disjoint random triangles near the origin with
// distinct vertices, for pair-enumeration checks.
func certRandomSoup(rng *rand.Rand, count int, spread float64) ([]r3.Vec, [][3]int) {
	var verts []r3.Vec
	var tris [][3]int
	for range count {
		c := r3.Vec{X: rng.Float64() * spread, Y: rng.Float64() * spread, Z: rng.Float64() * spread}
		for range 3 {
			verts = append(verts, c.Add(r3.Vec{X: rng.Float64(), Y: rng.Float64(), Z: rng.Float64()}))
		}
		n := len(verts)
		tris = append(tris, [3]int{n - 3, n - 2, n - 1})
	}
	return verts, tris
}
