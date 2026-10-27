package meshbool

import (
	"fmt"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// keysRotate turns a point by a about z, then by b about x.
func keysRotate(a, b float64) func(r3.Vec) r3.Vec {
	return func(v r3.Vec) r3.Vec {
		x := v.X*math.Cos(a) - v.Y*math.Sin(a)
		y := v.X*math.Sin(a) + v.Y*math.Cos(a)
		z := v.Z*math.Cos(b) - y*math.Sin(b)
		y = v.Z*math.Sin(b) + y*math.Cos(b)
		return r3.Vec{X: x, Y: y, Z: z}
	}
}

// keysCylinder is parityCylinder(n) turned by rot and shifted by at, with
// distinct non-zero vertex bounds so a corner welded to the wrong exact
// vertex shows in the stitched bounds.
func keysCylinder(tb testing.TB, n int, rot func(r3.Vec) r3.Vec, at r3.Vec) *BoolMesh {
	tb.Helper()
	verts, tris := parityCylinder(n)
	for i, v := range verts {
		p := rot(v)
		verts[i] = r3.Vec{X: p.X + at.X, Y: p.Y + at.Y, Z: p.Z + at.Z}
	}
	src := make([]int, len(tris))
	for i := range src {
		src[i] = i / 4
	}
	bm, err := PrepContext(tb.Context(), verts, tris, src)
	require.NoError(tb, err)
	beta := make([]float64, len(verts))
	for i := range beta {
		beta[i] = float64(i+1) * 1e-9
	}
	bm.SetVertexBounds(beta)
	return bm
}

type keysCase struct {
	name string
	a, b func(testing.TB) *BoolMesh
}

func keysCases() []keysCase {
	identity := func(v r3.Vec) r3.Vec { return v }
	box := func(rot func(r3.Vec) r3.Vec, at r3.Vec) func(testing.TB) *BoolMesh {
		return func(tb testing.TB) *BoolMesh {
			t, ok := tb.(*testing.T)
			if !ok {
				tb.Fatal("box fixtures need a *testing.T")
			}
			return boxMesh(t, r3.Vec{X: 1, Y: 1, Z: 1}, rot, at)
		}
	}
	cyl := func(n int, rot func(r3.Vec) r3.Vec, at r3.Vec) func(testing.TB) *BoolMesh {
		return func(tb testing.TB) *BoolMesh { return keysCylinder(tb, n, rot, at) }
	}
	return []keysCase{
		{"boxes tilted", box(identity, r3.Vec{}), box(keysRotate(0.3, 0.2), r3.Vec{X: 0.9, Y: 0.4, Z: 0.2})},
		{"boxes yawed", box(identity, r3.Vec{}), box(keysRotate(0.7, 0), r3.Vec{X: 1.1, Y: 0.3, Z: 0.1})},
		{"cylinders crossed", cyl(24, identity, r3.Vec{}), cyl(20, keysRotate(0.21, 1.3), r3.Vec{X: 0.37, Y: 0.11, Z: 0.53})},
		{"cylinders leaning", cyl(32, identity, r3.Vec{}), cyl(28, keysRotate(0.13, 0.31), r3.Vec{X: 1.7, Y: -0.23, Z: 1.1})},
	}
}

// requireStitchedEqual compares two stitched meshes field by field, every
// float by its bits.
func requireStitchedEqual(t *testing.T, want, got *StitchedMesh) {
	t.Helper()
	require.Equal(t, want.Verts, got.Verts)
	require.Equal(t, want.Tris, got.Tris)
	require.Equal(t, want.Src, got.Src)
	require.Equal(t, math.Float64bits(want.Round), math.Float64bits(got.Round))
	require.Equal(t, math.Float64bits(want.PreArea), math.Float64bits(got.PreArea))
	require.Equal(t, math.Float64bits(want.DropArea), math.Float64bits(got.DropArea))
	require.Len(t, got.VertexBound, len(want.VertexBound))
	for i := range want.VertexBound {
		require.Equal(t, math.Float64bits(want.VertexBound[i]), math.Float64bits(got.VertexBound[i]), `vertex %d`, i)
	}
	require.Equal(t, want.EmbeddingReach, got.EmbeddingReach)
}

// TestMeshBooleanKeysMatchStitchByKey holds the keys MeshBoolean carries on
// its kept facets to Xpt.Key, and the stitch that welds by them to the stitch
// that reads every key itself: the kept facets of a memoized run must equal
// those of a run keying every point with Xpt.Key, each carried key must be
// its corner's Key, and stitching with the carried keys and with the keys
// cleared must give bit-identical meshes.
func TestMeshBooleanKeysMatchStitchByKey(t *testing.T) {
	plain := func() func(proof.Xpt) string { return proof.Xpt.Key }
	for _, tc := range keysCases() {
		for _, op := range []OperationKind{OpUnion, OpCut, OpIntersect} {
			t.Run(fmt.Sprintf("%s/%v", tc.name, op), func(t *testing.T) {
				run := func(newKeyOf func() func(proof.Xpt) string) ([]KeptFacet, float64) {
					a, b := tc.a(t), tc.b(t)
					kept, rim, err := meshBoolean(t.Context(), op, a, b, NewContactMemo(a, b), newKeyOf)
					require.NoError(t, err)
					return kept, rim
				}
				want, wantRim := run(plain)
				got, gotRim := run(newKeyMemo)
				require.NotEmpty(t, got)
				require.Equal(t, math.Float64bits(wantRim), math.Float64bits(gotRim))
				require.Len(t, got, len(want))
				for i := range want {
					require.Equal(t, want[i].Src, got[i].Src, `facet %d`, i)
					require.Equal(t, want[i].Beta, got[i].Beta, `facet %d`, i)
					for k := range 3 {
						key := got[i].V[k].Key()
						require.Equal(t, want[i].V[k].Key(), key, `facet %d corner %d`, i, k)
						require.Equal(t, key, got[i].Keys[k], `facet %d corner %d carried key`, i, k)
					}
				}

				keyed, err := StitchFacetsContext(t.Context(), got)
				require.NoError(t, err)
				cleared := make([]KeptFacet, len(got))
				for i, f := range got {
					f.Keys = [3]string{}
					cleared[i] = f
				}
				unkeyed, err := StitchFacetsContext(t.Context(), cleared)
				require.NoError(t, err)
				requireStitchedEqual(t, unkeyed, keyed)
			})
		}
	}
}

// BenchmarkMeshBooleanStitch runs the boolean and its stitch over two crossed
// prism tessellations, the path whose exact-point keys the shared memo reads.
func BenchmarkMeshBooleanStitch(b *testing.B) {
	for _, n := range []int{24, 96} {
		b.Run(fmt.Sprintf("%d_gon", n), func(b *testing.B) {
			b.ReportAllocs()
			for b.Loop() {
				b.StopTimer()
				ma := keysCylinder(b, n, func(v r3.Vec) r3.Vec { return v }, r3.Vec{})
				mb := keysCylinder(b, n-4, keysRotate(0.21, 1.3), r3.Vec{X: 0.37, Y: 0.11, Z: 0.53})
				b.StartTimer()
				kept, _, err := MeshBoolean(b.Context(), OpCut, ma, mb, NewContactMemo(ma, mb))
				if err != nil {
					b.Fatal(err)
				}
				got, err := StitchFacetsContext(b.Context(), kept)
				if err != nil {
					b.Fatal(err)
				}
				if len(got.Tris) == 0 {
					b.Fatal("empty stitch")
				}
			}
		})
	}
}
