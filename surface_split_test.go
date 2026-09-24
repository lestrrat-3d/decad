package decad_test

import (
	"fmt"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func splitVolume(t *testing.T, b *decad.Body) decad.Measurement {
	t.Helper()
	v, err := b.Volume()
	require.NoError(t, err)
	return v
}

func splitVolumeInterval(v decad.Measurement) (*big.Rat, *big.Rat) {
	value := new(big.Rat).SetFloat64(v.Value.Base())
	bound := new(big.Rat).SetFloat64(v.Bound.Base())
	return new(big.Rat).Sub(value, bound), new(big.Rat).Add(value, bound)
}

func requireSplitVolumeEncloses(t *testing.T, v decad.Measurement, wantLo, wantHi *big.Rat) {
	t.Helper()
	lo, hi := splitVolumeInterval(v)
	require.LessOrEqual(t, lo.Cmp(wantLo), 0, "volume lower bound exceeds the reference")
	require.GreaterOrEqual(t, hi.Cmp(wantHi), 0, "volume upper bound misses the reference")
}

func requireSplitVolumeSumEncloses(t *testing.T, a, b decad.Measurement, want *big.Rat) {
	t.Helper()
	aLo, aHi := splitVolumeInterval(a)
	bLo, bHi := splitVolumeInterval(b)
	require.LessOrEqual(t, new(big.Rat).Add(aLo, bLo).Cmp(want), 0)
	require.GreaterOrEqual(t, new(big.Rat).Add(aHi, bHi).Cmp(want), 0)
}

func splitBlock(t *testing.T, d *decad.Document) *decad.Body {
	t.Helper()
	s, p := plateSketch(t)
	b, err := d.Extrude(s, p, decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	require.NoError(t, err)
	return b
}

func splitRibbon(t *testing.T, d *decad.Document, endU float64) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	a := s.CreatePoint(-10, 30)
	s.Fix(a)
	s.CreateLine(a, s.CreatePoint(endU, 30))
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	require.Len(t, s.Chains(), 1)
	b, err := d.ExtrudeChain(s, s.Chains()[0], decad.TwoSided{
		One: decad.DistanceSide{D: units.Millimeters(15)},
		Two: decad.DistanceSide{D: units.Millimeters(5)},
	})
	require.NoError(t, err)
	return b
}

func TestSurfaceSplitRibbonPartitionsBlock(t *testing.T) {
	t.Parallel()
	d := decad.New()
	target := splitBlock(t, d)
	tool := splitRibbon(t, d, 110)
	pieces, err := d.Split(t.Context(), target, tool)
	require.NoError(t, err)
	require.Len(t, pieces, 2)
	volumes := make([]decad.Measurement, len(pieces))
	for i, piece := range pieces {
		require.Equal(t, decad.BodySolid, piece.Kind())
		require.True(t, piece.IsSolid())
		volume := splitVolume(t, piece)
		// Deleting the cut charge leaves 5.8e-13 mm³ of incidental
		// arithmetic bound; the charged result exceeds 1.1e-9 mm³.
		t.Run(fmt.Sprintf("piece %d bound", i), func(t *testing.T) {
			require.Greater(t, volume.Bound.Base(), 1e-10)
			require.Equal(t, decad.Approximate, volume.Exactness)
			requireSplitVolumeEncloses(t, volume, big.NewRat(30000, 1), big.NewRat(30000, 1))
		})
		volumes[i] = volume
		t.Logf("ribbon piece %d volume %g ± %g mm³", i, volume.Value.Base(), volume.Bound.Base())
	}
	t.Run("sum interval", func(t *testing.T) {
		require.Greater(t, volumes[0].Bound.Base()+volumes[1].Bound.Base(), 1e-10)
		requireSplitVolumeSumEncloses(t, volumes[0], volumes[1], big.NewRat(60000, 1))
	})
	require.Equal(t, pieces, d.Bodies())
	_, err = d.Split(t.Context(), target, tool)
	require.ErrorIs(t, err, decad.ErrRetiredBody)
	replay := decad.New()
	replayTarget := splitBlock(t, replay)
	replayTool := splitRibbon(t, replay, 110)
	replayPieces, err := replay.Split(t.Context(), replayTarget, replayTool)
	require.NoError(t, err)
	require.Len(t, replayPieces, len(pieces))
	for i, piece := range replayPieces {
		require.Equal(t, volumes[i], splitVolume(t, piece), "cell order differs on replay")
		wantBounds, err := pieces[i].Bounds()
		require.NoError(t, err)
		gotBounds, err := piece.Bounds()
		require.NoError(t, err)
		require.Equal(t, wantBounds, gotBounds, "cell order differs on replay")
	}
}

func TestSurfaceSplitCirclePartitionsBlock(t *testing.T) {
	t.Parallel()
	d := decad.New()
	target := splitBlock(t, d)
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	center := s.CreatePoint(50, 30)
	s.Fix(center)
	s.CreateCircle(center, 20)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	require.Len(t, s.Profiles(), 1)
	tool, err := d.Extrude(s, s.Profiles()[0], decad.TwoSided{
		One: decad.DistanceSide{D: units.Millimeters(15)},
		Two: decad.DistanceSide{D: units.Millimeters(5)},
	}, decad.WithSurfaceResult())
	require.NoError(t, err)
	pieces, err := d.Split(t.Context(), target, tool)
	require.NoError(t, err)
	require.Len(t, pieces, 2)
	a, b := splitVolume(t, pieces[0]), splitVolume(t, pieces[1])
	if a.Value.Base() > b.Value.Base() {
		a, b = b, a
	}
	diskLo := new(big.Rat).Mul(big.NewRat(4000, 1), piRefLo)
	diskHi := new(big.Rat).Mul(big.NewRat(4000, 1), piRefHi)
	remainderLo := new(big.Rat).Sub(big.NewRat(60000, 1), diskHi)
	remainderHi := new(big.Rat).Sub(big.NewRat(60000, 1), diskLo)
	t.Run("disk interval", func(t *testing.T) { requireSplitVolumeEncloses(t, a, diskLo, diskHi) })
	t.Run("remainder interval", func(t *testing.T) {
		// Without the circular region-area proof this fixture retains only
		// 3.6e-12 mm³ of incidental arithmetic bound.
		require.Greater(t, b.Bound.Base(), 5e-12)
		requireSplitVolumeEncloses(t, b, remainderLo, remainderHi)
	})
	t.Run("sum interval", func(t *testing.T) {
		// Removing that proof leaves the two bounds' sum near 4.1e-12 mm³.
		require.Greater(t, a.Bound.Base()+b.Bound.Base(), 6e-12)
		requireSplitVolumeSumEncloses(t, a, b, big.NewRat(60000, 1))
	})
	t.Logf("circle disk volume %g ± %g mm³; remainder %g ± %g mm³",
		a.Value.Base(), a.Bound.Base(), b.Value.Base(), b.Bound.Base())
}

func TestSurfaceSplitInsideStubDoesNotSeparate(t *testing.T) {
	t.Parallel()
	d := decad.New()
	target := splitBlock(t, d)
	tool := splitRibbon(t, d, 40)
	before := d.Bodies()
	targetVolume := splitVolume(t, target)
	toolArea, err := tool.Area()
	require.NoError(t, err)
	pieces, err := d.Split(t.Context(), target, tool)
	require.Nil(t, pieces)
	require.ErrorIs(t, err, decad.ErrDegenerate)
	require.ErrorContains(t, err, "tool separates no part of the target")
	require.Equal(t, before, d.Bodies())
	require.Equal(t, targetVolume, splitVolume(t, target))
	afterArea, err := tool.Area()
	require.NoError(t, err)
	require.Equal(t, toolArea, afterArea)
}

func TestSurfaceSplitOutsideRibbonDoesNotSeparate(t *testing.T) {
	t.Parallel()
	d := decad.New()
	target := splitBlock(t, d)
	tool := splitRibbon(t, d, -20)
	before := d.Bodies()
	pieces, err := d.Split(t.Context(), target, tool)
	require.Nil(t, pieces)
	require.ErrorIs(t, err, decad.ErrDegenerate)
	require.ErrorContains(t, err, "tool separates no part of the target")
	require.Equal(t, before, d.Bodies())
}

func TestSurfaceSplitCoincidentBoundaryRefusesInvalidCell(t *testing.T) {
	t.Parallel()
	d := decad.New()
	target := splitBlock(t, d)
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	a := s.CreatePoint(0, 0)
	s.Fix(a)
	s.CreateLine(a, s.CreatePoint(100, 0))
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	tool, err := d.ExtrudeChain(s, s.Chains()[0], decad.TwoSided{
		One: decad.DistanceSide{D: units.Millimeters(15)},
		Two: decad.DistanceSide{D: units.Millimeters(5)},
	})
	require.NoError(t, err)
	before := d.Bodies()
	pieces, err := d.Split(t.Context(), target, tool)
	require.Nil(t, pieces)
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.ErrorContains(t, err, "unchanged target cell is invalid")
	require.Equal(t, before, d.Bodies())
}
