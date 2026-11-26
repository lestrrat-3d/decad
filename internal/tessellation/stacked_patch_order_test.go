package tessellation_test

// CI: isolated-tests

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// TestProveStackedTermOrderRepeats gives ProveStacked many planar faces whose axial displacements differ and requires
// the proof's terms to come out in the same order, with the same bits, on every call. The face terms used to be read
// from the FaceAxial map, so their order, and with it the upward-rounded sum, changed from run to run.
func TestProveStackedTermOrderRepeats(t *testing.T) {
	const faces = 24
	in := tessellation.StackedProofInput[int]{
		SlabHeights:  []float64{},
		SectionDelta: 0,
		FaceAxial:    map[int]float64{},
		IsPlanar:     func(int) bool { return true },
		StoreMax:     1e-12,
	}
	for f := range faces {
		in.Faces = append(in.Faces, f)
		in.FaceAxial[f] = 1e-12 * float64(f+1) * (1 + 1e-3*float64(f))
		base := len(in.Vertices)
		in.Vertices = append(in.Vertices,
			r3.Vec{X: float64(f)}, r3.Vec{X: float64(f) + 1 + 0.1*float64(f)}, r3.Vec{X: float64(f), Y: 1 + 0.07*float64(f)})
		in.Store = append(in.Store, 1e-12, 1e-12, 1e-12)
		in.Triangles = append(in.Triangles, [3]int{base, base + 1, base + 2})
		in.Source = append(in.Source, f)
	}
	bits := func() []uint64 {
		var out []uint64
		for _, term := range tessellation.ProveStacked(in).Terms {
			out = append(out, math.Float64bits(term))
		}
		return out
	}
	want := bits()
	require.Len(t, want, faces+1)
	for i := range 30 {
		require.Equal(t, want, bits(), "call %d", i+2)
	}
}
