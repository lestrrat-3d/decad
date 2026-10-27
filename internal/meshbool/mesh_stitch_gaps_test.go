package meshbool

import (
	"context"
	"fmt"
	"math"
	"math/big"
	"math/rand/v2"
	"slices"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestCoordDistanceMatchesRationalAtFloatExtremes(t *testing.T) {
	t.Parallel()
	shared := big.NewInt(3)
	points := []proof.Xpt{
		{X: big.NewInt(-1), Y: big.NewInt(2), Z: big.NewInt(-3), W: big.NewInt(7)},
		{X: shared, Y: shared, Z: shared, W: shared},
		{X: big.NewInt(1), Y: big.NewInt(-2), Z: big.NewInt(3), W: new(big.Int).Lsh(big.NewInt(1), 1100)},
		{X: new(big.Int).Lsh(big.NewInt(1), 1100), Y: big.NewInt(-1), Z: big.NewInt(0), W: big.NewInt(3)},
		proof.XptOf(r3.Vec{X: math.MaxFloat64, Y: math.SmallestNonzeroFloat64, Z: -math.MaxFloat64}),
	}
	held := []r3.Vec{
		{},
		{X: math.Copysign(0, -1), Y: math.SmallestNonzeroFloat64, Z: -math.SmallestNonzeroFloat64},
		{X: 1, Y: math.Nextafter(1, 2), Z: math.Nextafter(-1, -2)},
		{X: math.MaxFloat64, Y: -math.MaxFloat64, Z: math.SmallestNonzeroFloat64},
	}
	for pi, p := range points {
		fields := [4]*big.Int{p.X, p.Y, p.Z, p.W}
		var before [4]string
		for i, n := range fields {
			before[i] = n.Text(16)
		}
		for vi, v := range held {
			want, got := rawCoordDistance(p, v), CoordDistance(p, v)
			require.Zero(t, want.Cmp(got), "point %d held %d", pi, vi)
			require.Equal(t, want.RatString(), got.RatString(), "point %d held %d canonical value", pi, vi)
			wf, we := want.Float64()
			gf, ge := got.Float64()
			require.Equal(t, math.Float64bits(wf), math.Float64bits(gf), "point %d held %d float", pi, vi)
			require.Equal(t, we, ge, "point %d held %d exactness", pi, vi)
			got.SetInt64(-9)
			for i, n := range fields {
				require.Equal(t, before[i], n.Text(16), "point %d held %d input %d", pi, vi, i)
			}
		}
		// Exact float lifts must have a zero gap, including a non-reduced
		// homogeneous spelling whose four fields share one integer.
		if pi == 1 || pi == 4 {
			require.Zero(t, CoordDistance(p, p.Vec()).Sign(), "point %d exact float", pi)
		}
	}
}

// keepRoundedEmbeddedBeforeGapReuse freezes the original final-gap loop.
func keepRoundedEmbeddedBeforeGapReuse(ctx context.Context, xverts []proof.Xpt, remap []int,
	out *StitchedMesh,
) ([]*big.Rat, error) {
	gaps := make([]*big.Rat, len(xverts))
	h := HeldRounding{
		Verts: out.Verts, Tris: out.Tris,
		Exact: make([]proof.Xpt, len(out.Verts)),
		Moved: make([]bool, len(out.Verts)), Movable: make([]bool, len(out.Verts)),
	}
	count := make([]int, len(out.Verts))
	for i, p := range xverts {
		if i%256 == 0 {
			if err := ctx.Err(); err != nil {
				return nil, err
			}
		}
		fi := remap[i]
		count[fi]++
		h.Exact[fi] = p
		gaps[i] = CoordDistance(p, out.Verts[fi])
		if gaps[i].Sign() != 0 {
			h.Moved[fi] = true
		}
	}
	for fi, n := range count {
		if n > 1 {
			h.Moved[fi] = true
		}
		h.Movable[fi] = n == 1 && h.Moved[fi]
	}
	moved, err := EnforceHeldEmbedding(ctx, h)
	if err != nil {
		return nil, err
	}
	if moved > 0 {
		for i, p := range xverts {
			if i%256 == 0 {
				if err := ctx.Err(); err != nil {
					return nil, err
				}
			}
			gaps[i] = CoordDistance(p, out.Verts[remap[i]])
		}
	}
	return gaps, nil
}

func movedGapFixture() ([]proof.Xpt, []int, *StitchedMesh) {
	xverts, out := stitchSliver(1)
	for k := range 128 {
		x := float64(10 + 3*k)
		for _, v := range []r3.Vec{{X: x}, {X: x + 1}, {X: x, Y: 1}} {
			xverts = append(xverts, proof.XptOf(v))
			out.Verts = append(out.Verts, v)
		}
		base := len(out.Verts) - 3
		out.Tris = append(out.Tris, [3]int{base, base + 1, base + 2})
	}
	remap := make([]int, len(xverts))
	for i := range remap {
		remap[i] = i
	}
	return xverts, remap, out
}

func TestKeepRoundedEmbeddedGapReuseMatchesOldLoop(t *testing.T) {
	type gapCase struct {
		name   string
		xverts []proof.Xpt
		remap  []int
		out    *StitchedMesh
	}
	var cases []gapCase
	for _, offset := range []int64{-1, 1} {
		xverts, out := stitchSliver(offset)
		cases = append(cases, gapCase{fmt.Sprintf("sliver %d", offset), xverts, []int{0, 1, 2, 3}, out})
	}
	xverts, out := stitchSliver(1)
	cases = append(cases, gapCase{"welded", append(xverts, xverts[3]), []int{0, 1, 2, 3, 3}, out})
	xverts, remap, out := movedGapFixture()
	cases = append(cases, gapCase{"many unchanged vertices", xverts, remap, out})
	noMove := slices.Clone(xverts)
	noMove[3] = proof.XptOf(out.Verts[3])
	cases = append(cases, gapCase{"no movable vertices", noMove, remap, out})
	for seed := range uint64(80) {
		h := stitchLayered(rand.New(rand.NewPCG(seed, 0)), 2+int(seed%4))
		remap := make([]int, len(h.Verts))
		for i := range remap {
			remap[i] = i
		}
		cases = append(cases, gapCase{fmt.Sprintf("layered %d", seed), h.Exact, remap,
			&StitchedMesh{Verts: h.Verts, Tris: h.Tris}})
	}
	var moved, refused int
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			keys := make([]string, len(tc.xverts))
			for i, p := range tc.xverts {
				keys[i] = p.Key()
			}
			before := slices.Clone(tc.out.Verts)
			wantOut := &StitchedMesh{Verts: slices.Clone(before), Tris: tc.out.Tris}
			gotOut := &StitchedMesh{Verts: slices.Clone(before), Tris: tc.out.Tris}
			want, wantErr := keepRoundedEmbeddedBeforeGapReuse(t.Context(), tc.xverts, tc.remap, wantOut)
			got, gotErr := keepRoundedEmbedded(t.Context(), tc.xverts, tc.remap, gotOut)
			require.Equal(t, fmt.Sprint(wantErr), fmt.Sprint(gotErr))
			for i, p := range tc.xverts {
				require.Equal(t, keys[i], p.Key())
			}
			for i, v := range wantOut.Verts {
				w := gotOut.Verts[i]
				require.Equal(t, math.Float64bits(v.X), math.Float64bits(w.X))
				require.Equal(t, math.Float64bits(v.Y), math.Float64bits(w.Y))
				require.Equal(t, math.Float64bits(v.Z), math.Float64bits(w.Z))
			}
			if wantErr != nil {
				refused++
				require.ErrorIs(t, wantErr, decaderr.ErrUnsupported)
				require.ErrorIs(t, gotErr, decaderr.ErrUnsupported)
				require.Nil(t, got)
				return
			}
			if !slices.Equal(before, gotOut.Verts) {
				moved++
			}
			beta := make([]float64, len(tc.xverts))
			for i := range want {
				require.Equal(t, want[i].RatString(), got[i].RatString())
				beta[i] = float64(i%7) * 1e-9
			}
			wantBounds, err := heldVertexBoundsFromGaps(t.Context(), want, beta, tc.remap, len(before))
			require.NoError(t, err)
			gotBounds, err := heldVertexBoundsFromGaps(t.Context(), got, beta, tc.remap, len(before))
			require.NoError(t, err)
			for i := range wantBounds {
				require.Equal(t, math.Float64bits(wantBounds[i]), math.Float64bits(gotBounds[i]))
			}
		})
	}
	require.Positive(t, moved)
	require.Positive(t, refused)
}

func TestKeepRoundedEmbeddedGapReuseCancellation(t *testing.T) {
	xverts, remap, out := movedGapFixture()
	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	_, wantErr := keepRoundedEmbeddedBeforeGapReuse(ctx, xverts, remap,
		&StitchedMesh{Verts: slices.Clone(out.Verts), Tris: out.Tris})
	_, gotErr := keepRoundedEmbedded(ctx, xverts, remap,
		&StitchedMesh{Verts: slices.Clone(out.Verts), Tris: out.Tris})
	require.ErrorIs(t, wantErr, context.Canceled)
	require.ErrorIs(t, gotErr, context.Canceled)
}

var gapReuseSink []*big.Rat

func BenchmarkKeepRoundedEmbeddedMovedGap(b *testing.B) {
	xverts, remap, template := movedGapFixture()
	b.ReportAllocs()
	for b.Loop() {
		out := &StitchedMesh{Verts: slices.Clone(template.Verts), Tris: template.Tris}
		gaps, err := keepRoundedEmbedded(b.Context(), xverts, remap, out)
		if err != nil {
			b.Fatal(err)
		}
		gapReuseSink = gaps
	}
}
