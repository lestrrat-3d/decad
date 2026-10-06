package meshbool_test

import (
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// heldFragment interns float facets into one indexed held mesh, every vertex
// marked moved and none movable: the search can only check it.
func heldFragment(facets [][3]r3.Vec) meshbool.HeldRounding {
	index := map[r3.Vec]int{}
	var h meshbool.HeldRounding
	for _, f := range facets {
		var tri [3]int
		for k, p := range f {
			i, ok := index[p]
			if !ok {
				i = len(h.Verts)
				index[p] = i
				h.Verts = append(h.Verts, p)
			}
			tri[k] = i
		}
		h.Tris = append(h.Tris, tri)
	}
	h.Exact = make([]proofbound.Xpt, len(h.Verts))
	h.Moved = make([]bool, len(h.Verts))
	h.Movable = make([]bool, len(h.Verts))
	for i, v := range h.Verts {
		h.Exact[i] = proofbound.XptOf(v)
		h.Moved[i] = true
	}
	return h
}

// TestEnforceHeldEmbeddingRefusesTheUnion14Fold holds the three facets of the
// 25° / 1.1 mm clustered tree's held mesh after union 13 that union 14's
// contact crossed: a trunk facet, a sliver whose apex rounding pushed across
// the trunk facet's edge, and a branch facet that now passes through the
// trunk facet. Nothing may move, so the check must refuse.
func TestEnforceHeldEmbeddingRefusesTheUnion14Fold(t *testing.T) {
	t.Parallel()
	h := heldFragment([][3]r3.Vec{
		{{X: 3.5767911948474933, Y: -3.47378800959096, Z: 19.07175669307605}, {X: 3.5355339059327373, Y: -3.5355339059327386, Z: 0}, {X: 3.7026241073138704, Y: -3.2854657476607976, Z: 19.83405176975887}},
		{{X: 3.7026241073138704, Y: -3.2854657476607976, Z: 19.83405176975887}, {X: 3.596461041130055, Y: -3.4443500042937334, Z: 19.190916511333544}, {X: 3.5767911948474933, Y: -3.47378800959096, Z: 19.07175669307605}},
		{{X: 3.7026241073138704, Y: -3.2854657476607976, Z: 19.83405176975887}, {X: 4.688268876166217, Y: -4.202479295295983, Z: 21.663312635311296}, {X: 3.596461041130055, Y: -3.4443500042937334, Z: 19.190916511333544}},
	})
	_, err := meshbool.EnforceHeldEmbedding(t.Context(), h)
	require.ErrorIs(t, err, decaderr.ErrUnsupported)
}

// TestEnforceHeldEmbeddingRefusesTheUnion23Fold is the same refusal for the
// six facets of the 7° / 2.2 mm tree's held mesh after union 22 whose folds
// union 23's contact chain crossed twice.
func TestEnforceHeldEmbeddingRefusesTheUnion23Fold(t *testing.T) {
	t.Parallel()
	h := heldFragment([][3]r3.Vec{
		{{X: -2.1380603196475523, Y: 1.4045410616999345, Z: 52.35326365704232}, {X: -1.1336663870396693, Y: 1.720760486827916, Z: 60.41336663870397}, {X: -2.112932039400581, Y: 1.5222364091527227, Z: 52.36680280957662}},
		{{X: -2.074079236133132, Y: 1.7042144090450266, Z: 52.38773675430554}, {X: -1.1336663870396693, Y: 1.720760486827916, Z: 60.41336663870397}, {X: -2.0651895358370274, Y: 1.7458518133885779, Z: 52.39252653725568}},
		{{X: -2.0596200573352497, Y: 1.771938027843169, Z: 52.395527380060315}, {X: -2.6116363462768177, Y: 1.6960164739427677, Z: 52.95623168740932}, {X: -2.141073237120844, Y: 1.3904292177782493, Z: 52.35164029289471}},
		{{X: -2.0596200573352497, Y: 1.771938027843169, Z: 52.395527380060315}, {X: -2.141073237120844, Y: 1.3904292177782493, Z: 52.35164029289471}, {X: -2.1380603196475523, Y: 1.4045410616999345, Z: 52.35326365704232}},
		{{X: -2.0596200573352497, Y: 1.771938027843169, Z: 52.395527380060315}, {X: -2.1380603196475523, Y: 1.4045410616999345, Z: 52.35326365704232}, {X: -2.112932039400581, Y: 1.5222364091527227, Z: 52.36680280957662}},
		{{X: -2.0596200573352497, Y: 1.771938027843169, Z: 52.395527380060315}, {X: -2.112932039400581, Y: 1.5222364091527227, Z: 52.36680280957662}, {X: -2.074079236133132, Y: 1.7042144090450266, Z: 52.38773675430554}},
	})
	_, err := meshbool.EnforceHeldEmbedding(t.Context(), h)
	require.ErrorIs(t, err, decaderr.ErrUnsupported)
}

// subUlpSliver is two coplanar facets sharing the edge A–B: A, C, B below the
// line y = x, and A, B, P whose exact apex P lies 2⁻⁶⁰ off that line on the
// side the sign of offset names. Nearest rounding puts P on the line, which
// flattens the sliver to no area.
func subUlpSliver(offset int64) meshbool.HeldRounding {
	exactY := new(big.Rat).Add(big.NewRat(1, 2), new(big.Rat).SetFrac(big.NewInt(offset), new(big.Int).Lsh(big.NewInt(1), 60)))
	w := new(big.Int).Lsh(big.NewInt(1), 61)
	p := proofbound.Xpt{
		X: new(big.Int).Rsh(w, 1),
		Y: new(big.Int).Div(new(big.Int).Mul(exactY.Num(), w), exactY.Denom()),
		Z: big.NewInt(0),
		W: w,
	}
	verts := []r3.Vec{{X: 0, Y: 0, Z: 0}, {X: 1, Y: 1, Z: 0}, {X: 1, Y: 0, Z: 0}, p.Vec()}
	return meshbool.HeldRounding{
		Verts:   verts,
		Tris:    [][3]int{{0, 2, 1}, {0, 1, 3}},
		Exact:   []proofbound.Xpt{proofbound.XptOf(verts[0]), proofbound.XptOf(verts[1]), proofbound.XptOf(verts[2]), p},
		Moved:   []bool{false, false, false, true},
		Movable: []bool{false, false, false, true},
	}
}

// TestEnforceHeldEmbeddingMovesAFlattenedSliver: the sliver's apex rounds onto
// the shared edge's line, and the first other corner of its float box — the
// float above 0.5 in y — restores it, on the side the exact apex lies.
func TestEnforceHeldEmbeddingMovesAFlattenedSliver(t *testing.T) {
	t.Parallel()
	h := subUlpSliver(1)
	require.Equal(t, 0.5, h.Verts[3].Y, `nearest rounding puts the apex on the edge's line`)
	moved, err := meshbool.EnforceHeldEmbedding(t.Context(), h)
	require.NoError(t, err)
	require.Equal(t, 1, moved)
	require.Equal(t, r3.Vec{X: 0.5, Y: math.Nextafter(0.5, 1), Z: 0}, h.Verts[3])
}

// TestEnforceHeldEmbeddingRefusesWhenNoCornerClears: with the exact apex on
// the wrong side of the shared edge, the only other corner of its float box
// folds the sliver back over its neighbour, so no float clears it and the
// check refuses.
func TestEnforceHeldEmbeddingRefusesWhenNoCornerClears(t *testing.T) {
	t.Parallel()
	h := subUlpSliver(-1)
	_, err := meshbool.EnforceHeldEmbedding(t.Context(), h)
	require.ErrorIs(t, err, decaderr.ErrUnsupported)
	require.Equal(t, 0.5, h.Verts[3].Y, `a refused search leaves the first rounding in place`)
}

// TestFloatBoxCornersOrder pins the deterministic corner order the search
// reads: x before y before z, the lower float before the upper, one value for
// a coordinate that is itself a float.
func TestFloatBoxCornersOrder(t *testing.T) {
	t.Parallel()
	h := subUlpSliver(1)
	require.Equal(t, []r3.Vec{{X: 0.5, Y: 0.5, Z: 0}, {X: 0.5, Y: math.Nextafter(0.5, 1), Z: 0}}, meshbool.FloatBoxCorners(h.Exact[3]))
}
