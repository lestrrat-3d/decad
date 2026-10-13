package meshbool

import (
	"math/big"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// oldPlaneKey is the decimal-string key planeOf used before the binary key.
func oldPlaneKey(d [3]*big.Int, off, w *big.Int) string {
	return d[0].String() + "," + d[1].String() + "," + d[2].String() + "|" +
		new(big.Rat).SetFrac(off, w).RatString()
}

// TestPlaneKeyGroupsLikeDecimalKey checks that the binary plane key puts two
// facets in one plane exactly when the old decimal key did, on random facets
// that share planes under both windings, lie on parallel and nearly equal
// planes, and have coordinates of mixed binary exponents.
func TestPlaneKeyGroupsLikeDecimalKey(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewPCG(7, 11))
	scale := func() float64 { return float64(int(1) << rng.IntN(6)) }
	point := func() float64 { return float64(rng.IntN(9)-4) * scale() / 8 }
	var facets [][3]r3.Vec
	for len(facets) < 500 {
		a := r3.Vec{X: point(), Y: point(), Z: point()}
		b := r3.Vec{X: point(), Y: point(), Z: point()}
		c := r3.Vec{X: point(), Y: point(), Z: point()}
		facets = append(facets, [3]r3.Vec{a, b, c})
		switch rng.IntN(4) {
		case 0: // same plane, reversed winding
			facets = append(facets, [3]r3.Vec{a, c, b})
		case 1: // parallel plane
			s := r3.Vec{X: 0.5, Y: 0.25, Z: 1}
			facets = append(facets, [3]r3.Vec{a.Add(s), b.Add(s), c.Add(s)})
		case 2: // nearly equal plane: one corner nudged by a few ulps
			c.Z = c.Z + 1e-12
			facets = append(facets, [3]r3.Vec{a, b, c})
		}
	}
	h := HeldRounding{}
	for _, f := range facets {
		base := len(h.Verts)
		h.Verts = append(h.Verts, f[0], f[1], f[2])
		h.Tris = append(h.Tris, [3]int{base, base + 1, base + 2})
	}
	h.Exact = make([]proof.Xpt, len(h.Verts))
	h.Moved = make([]bool, len(h.Verts))
	h.Movable = make([]bool, len(h.Verts))
	for i, v := range h.Verts {
		h.Exact[i] = proof.XptOf(v)
	}
	e := newHeldEmbedding(h, nil, false)
	oldID := map[string]int{}
	for i := range h.Tris {
		_, x, n := e.facet(i)
		if zeroXpt(n) {
			require.Equal(t, -2, e.planeOf(i))
			continue
		}
		g := new(big.Int).GCD(nil, nil, new(big.Int).Abs(n.X), new(big.Int).Abs(n.Y))
		g.GCD(nil, nil, g, new(big.Int).Abs(n.Z))
		d := [3]*big.Int{new(big.Int).Quo(n.X, g), new(big.Int).Quo(n.Y, g), new(big.Int).Quo(n.Z, g)}
		lead := d[0].Sign()
		if lead == 0 {
			lead = d[1].Sign()
		}
		if lead == 0 {
			lead = d[2].Sign()
		}
		if lead < 0 {
			for k := range d {
				d[k].Neg(d[k])
			}
		}
		off := new(big.Int).Mul(d[0], x[0].X)
		off.Add(off, new(big.Int).Mul(d[1], x[0].Y))
		off.Add(off, new(big.Int).Mul(d[2], x[0].Z))
		k := oldPlaneKey(d, off, x[0].W)
		id, ok := oldID[k]
		if !ok {
			id = len(oldID)
			oldID[k] = id
		}
		// The binary key is a function of the same canonical pair.
		require.Equal(t, id, e.planeOf(i))
	}
	require.Greater(t, len(oldID), 50)
	require.Less(t, len(oldID), len(facets))
}

// TestPlaneKeyOffsetReduction checks that equal offset fractions give equal
// keys however the fraction is written, for power-of-two and other
// denominators, and that unequal fractions give unequal keys.
func TestPlaneKeyOffsetReduction(t *testing.T) {
	t.Parallel()
	d := [3]*big.Int{big.NewInt(1), big.NewInt(-2), big.NewInt(3)}
	key := func(n, w int64) string { return string(planeKey(d, big.NewInt(n), big.NewInt(w))) }
	for _, c := range [][4]int64{
		{6, 8, 3, 4}, {-6, 8, -3, 4}, {0, 8, 0, 1}, {0, 6, 0, 5}, {6, 9, 2, 3}, {-12, 18, -2, 3}, {5, 1, 5, 1},
	} {
		require.Equal(t, key(c[2], c[3]), key(c[0], c[1]), "%v", c)
	}
	require.NotEqual(t, key(3, 4), key(-3, 4))
	require.NotEqual(t, key(3, 4), key(3, 8))
	require.NotEqual(t, key(2, 3), key(3, 2))
}
