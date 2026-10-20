package prismcells_test

import (
	"context"
	"testing"

	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismcells"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topo/topotest"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// polygonLoop is the closed directed walk through pts as straight segments.
func polygonLoop(pts ...[2]float64) sectionrecord.LoopRecord {
	var segs []sectionrecord.CurveSegment
	for i, p := range pts {
		q := pts[(i+1)%len(pts)]
		segs = append(segs, sectionrecord.LineSeg{Start: sectionrecord.Point2{U: p[0], V: p[1]},
			End: sectionrecord.Point2{U: q[0], V: q[1]}, TStart: 0, TEnd: 1})
	}
	return sectionrecord.LoopRecord{Segments: segs}
}

func overlapPrism(t *testing.T, z0, z1 float64, outer sectionrecord.LoopRecord) prismrecord.Payload {
	t.Helper()
	frame, err := r3.NewFrame(r3.Vec{}, r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	return prismrecord.Payload{Frame: frame, Xform: r3.Identity(), Z0: z0, Z1: z1,
		Profile: momentinput.Profile{Outer: outer}}
}

// uShape is a U whose two arms rise from a base: x in [0,10] and [20,30] above
// y = 10.
func uShape() sectionrecord.LoopRecord {
	return polygonLoop([2]float64{0, 0}, [2]float64{30, 0}, [2]float64{30, 30}, [2]float64{20, 30},
		[2]float64{20, 10}, [2]float64{10, 10}, [2]float64{10, 30}, [2]float64{0, 30})
}

// TestOverlapVolume reads multi-region overlap volumes off real prism
// payloads. The gates that need an arrangement-cap overflow, an amplified
// crossing, a cell the region integrals refuse or a work budget that runs out
// mid-loop are not reachable from a small fixture and are covered only by the
// Verify tests in apitest; the frame is shared by admission, so reading the
// other operand's frame changes nothing.
func TestOverlapVolume(t *testing.T) {
	t.Parallel()

	envs := 0
	newEnv := func() topo.Env {
		envs++
		return topo.Env{Doc: &topo.Document{}, Ref: provenance.Producer(envs), Alloc: topotest.Alloc{}}
	}
	u := uShape()

	t.Run("a bar across both arms sums the two regions", func(t *testing.T) {
		envs = 0
		// The bar overlaps each arm in a 10 x 5 region: 100 mm^2 in all.
		bar := polygonLoop([2]float64{-5, 20}, [2]float64{35, 20}, [2]float64{35, 25}, [2]float64{-5, 25})
		sum, bound, ok, err := prismcells.OverlapVolume(t.Context(), overlapPrism(t, 0, 10, u),
			overlapPrism(t, 0, 10, bar), newEnv)
		require.NoError(t, err)
		require.True(t, ok)
		require.InDelta(t, 1000, sum, bound+1e-9)
		require.Positive(t, bound)
		require.Equal(t, 2, envs, "one scratch body per overlap region")
	})
	t.Run("the shared z interval scales every region", func(t *testing.T) {
		bar := polygonLoop([2]float64{-5, 20}, [2]float64{35, 20}, [2]float64{35, 25}, [2]float64{-5, 25})
		sum, bound, ok, err := prismcells.OverlapVolume(t.Context(), overlapPrism(t, 0, 10, u),
			overlapPrism(t, 4, 14, bar), newEnv)
		require.NoError(t, err)
		require.True(t, ok)
		require.InDelta(t, 600, sum, bound+1e-9, "the z interval [4, 10] is 6 tall")
	})
	t.Run("axial displacements widen the bound", func(t *testing.T) {
		bar := polygonLoop([2]float64{-5, 20}, [2]float64{35, 20}, [2]float64{35, 25}, [2]float64{-5, 25})
		a, b := overlapPrism(t, 0, 10, u), overlapPrism(t, 0, 10, bar)
		_, base, ok, err := prismcells.OverlapVolume(t.Context(), a, b, newEnv)
		require.NoError(t, err)
		require.True(t, ok)
		for name, edit := range map[string]func(a, b *prismrecord.Payload){
			"a lower": func(a, _ *prismrecord.Payload) { a.Z0Delta = 1e-3 },
			"b lower": func(_, b *prismrecord.Payload) { b.Z0Delta = 1e-3 },
			"a upper": func(a, _ *prismrecord.Payload) { a.Z1Delta = 1e-3 },
			"b upper": func(_, b *prismrecord.Payload) { b.Z1Delta = 1e-3 },
		} {
			da, db := a, b
			edit(&da, &db)
			_, widened, ok, err := prismcells.OverlapVolume(t.Context(), da, db, newEnv)
			require.NoError(t, err, name)
			require.True(t, ok, name)
			require.Greater(t, widened, base+0.05, name+" end's displacement times the 100 mm^2 overlap")
		}
	})
	t.Run("section displacements widen the bound", func(t *testing.T) {
		bar := polygonLoop([2]float64{-5, 20}, [2]float64{35, 20}, [2]float64{35, 25}, [2]float64{-5, 25})
		a, b := overlapPrism(t, 0, 10, u), overlapPrism(t, 0, 10, bar)
		_, base, _, err := prismcells.OverlapVolume(t.Context(), a, b, newEnv)
		require.NoError(t, err)
		a.SectionDelta = 1e-3
		_, widened, ok, err := prismcells.OverlapVolume(t.Context(), a, b, newEnv)
		require.NoError(t, err)
		require.True(t, ok)
		require.Greater(t, widened, base)
	})
	t.Run("an exactly tangent pair answers nothing", func(t *testing.T) {
		// The box meets the U's right edge x = 30 along a line only.
		box := polygonLoop([2]float64{30, 0}, [2]float64{40, 0}, [2]float64{40, 10}, [2]float64{30, 10})
		_, _, ok, err := prismcells.OverlapVolume(t.Context(), overlapPrism(t, 0, 10, u),
			overlapPrism(t, 0, 10, box), newEnv)
		require.NoError(t, err)
		require.False(t, ok)
	})
	t.Run("a canceled context refuses", func(t *testing.T) {
		bar := polygonLoop([2]float64{-5, 20}, [2]float64{35, 20}, [2]float64{35, 25}, [2]float64{-5, 25})
		ctx, cancel := context.WithCancel(t.Context())
		cancel()
		_, _, ok, err := prismcells.OverlapVolume(ctx, overlapPrism(t, 0, 10, u), overlapPrism(t, 0, 10, bar), newEnv)
		require.ErrorIs(t, err, context.Canceled)
		require.False(t, ok)
	})
}
