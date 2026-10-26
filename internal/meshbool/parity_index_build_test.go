package meshbool

import (
	"encoding/binary"
	"hash/fnv"
	"testing"

	"github.com/stretchr/testify/require"
)

// parityBuildHash folds the built index's start and ids slices into one hash.
func parityBuildHash(idx *ParityIndex) (uint64, int, int) {
	h := fnv.New64a()
	var buf [4]byte
	for _, s := range [][]int32{idx.start, idx.ids} {
		for _, x := range s {
			binary.LittleEndian.PutUint32(buf[:], uint32(x))
			_, _ = h.Write(buf[:])
		}
		_, _ = h.Write([]byte{0xff, 0xff, 0xff, 0xff})
	}
	return h.Sum64(), len(idx.start), len(idx.ids)
}

// TestParityIndexBuildIsStable pins the exact start and ids slices built for a
// 6000-facet closed shell on each swept axis.
func TestParityIndexBuildIsStable(t *testing.T) {
	t.Parallel()
	verts, tris := parityCylinder(1500)
	pm := NewParityMesh(verts, tris)
	want := []uint64{0xe233565e4203299, 0xb0a2b33a9c112095, 0x7f885a5cd47948f}
	for i, ax := range []struct{ axis, u, v int }{{0, 1, 2}, {1, 2, 0}, {2, 0, 1}} {
		idx := pm.buildParityIndex(ax.axis, ax.u, ax.v)
		require.NotNil(t, idx)
		sum, ns, ni := parityBuildHash(idx)
		require.Equal(t, want[i], sum, "axis %d", ax.axis)
		require.Equal(t, 24650, ns)
		require.Positive(t, ni)
	}
}

func BenchmarkBuildParityIndex(b *testing.B) {
	verts, tris := parityCylinder(1500)
	pm := NewParityMesh(verts, tris)
	require.NotNil(b, pm.buildParityIndex(0, 1, 2))
	b.ReportAllocs()
	b.ResetTimer()
	for range b.N {
		_ = pm.buildParityIndex(0, 1, 2)
	}
}
