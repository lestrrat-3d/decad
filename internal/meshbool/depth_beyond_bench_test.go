package meshbool

import (
	"testing"

	"github.com/stretchr/testify/require"
)

// BenchmarkProvenDepthExceeds times the depth-witness search on 512-facet
// cylinder pairs: a shallow meet and a touch, where no witness exists and the
// search runs to the end, and a deep crossing, where one is found. The ref
// legs run the reference copy of the search without depthBeyond's shortcuts.
func BenchmarkProvenDepthExceeds(b *testing.B) {
	pairs := map[string]depthPair{}
	for _, pc := range depthPairs(b, 128) {
		pairs[pc.name] = pc
	}
	for _, name := range []string{"shallow/0.125", "touch/0.125", "deep/0.125"} {
		pc, ok := pairs[name]
		require.True(b, ok, name)
		want, err := refProvenDepthExceeds(b.Context(), pc.a, pc.nc.CloseA, pc.b, pc.nc.CloseB, pc.nc.Spans, pc.slack)
		require.NoError(b, err)
		b.Run(name, func(b *testing.B) {
			b.ReportAllocs()
			for b.Loop() {
				got, err := ProvenDepthExceeds(b.Context(), pc.a, pc.nc.CloseA, pc.b, pc.nc.CloseB, pc.nc.Spans, pc.slack)
				require.NoError(b, err)
				require.Equal(b, want, got)
			}
		})
		b.Run(name+"/ref", func(b *testing.B) {
			b.ReportAllocs()
			for b.Loop() {
				got, err := refProvenDepthExceeds(b.Context(), pc.a, pc.nc.CloseA, pc.b, pc.nc.CloseB, pc.nc.Spans, pc.slack)
				require.NoError(b, err)
				require.Equal(b, want, got)
			}
		})
	}
}
