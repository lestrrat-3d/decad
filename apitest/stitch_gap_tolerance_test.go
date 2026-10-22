package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// stitchBoxVolume and stitchBoxArea are the volume and area of the 100×60×10 box
// stitchBoxSheets draws.
const (
	stitchBoxVolume = 60000.0
	stitchBoxArea   = 15200.0
)

// TestStitchGapTolerance pins Stitch welding free edges whose vertices miss by no
// more than the document's gap tolerance (agent-docs/gap-tolerance-design.md
// §5.3). The top patch of the box is drawn off the wall's rim by a gap.
func TestStitchGapTolerance(t *testing.T) {
	t.Parallel()

	stitched := func(t *testing.T, gap float64, opts ...decad.DocumentOption) *decad.Body {
		t.Helper()
		doc := decad.New(opts...)
		walls, bottom, top := stitchBoxSheets(t, doc, 10+gap)
		body, err := decad.Stitch(t.Context(), walls, bottom, top)
		require.NoError(t, err)
		return body
	}

	t.Run("a gap below the default closes the box", func(t *testing.T) {
		t.Parallel()
		for _, gap := range []float64{1e-9, 1e-4, 0.005} {
			box := stitched(t, gap)
			require.Equal(t, decad.BodySolid, box.Kind(), "gap %g", gap)
			require.Len(t, box.Lumps(), 1)
			_, err := decad.Edges(decad.Free()).SelectEdges(box)
			require.ErrorIs(t, err, decad.ErrNoMatch, "no free edge remains")

			volume, err := box.Volume()
			require.NoError(t, err)
			require.Equal(t, decad.Approximate, volume.Exactness)
			trueVolume := stitchBoxVolume + gap*100*60
			require.LessOrEqual(t, math.Abs(volume.Value.Base()-trueVolume), volume.Bound.Base(),
				"the volume bound covers the distance to the volume of the box as drawn")

			area, err := box.Area()
			require.NoError(t, err)
			trueArea := stitchBoxArea + gap*2*(100+60)
			require.LessOrEqual(t, math.Abs(area.Value.Base()-trueArea), area.Bound.Base())

			bounds, err := box.Bounds()
			require.NoError(t, err)
			require.LessOrEqual(t, math.Abs(bounds.Max.Z-(10+gap)), bounds.Bound.Base()*(1+1e-9),
				"the box bound covers the patch as drawn")
		}
	})

	t.Run("a gap beyond the default leaves the edges free", func(t *testing.T) {
		t.Parallel()
		sheet := stitched(t, 0.02)
		require.Equal(t, decad.BodySheet, sheet.Kind())
		free, err := decad.Edges(decad.Free()).SelectEdges(sheet)
		require.NoError(t, err)
		require.Len(t, free, 8)
	})

	t.Run("a larger tolerance closes a larger gap", func(t *testing.T) {
		t.Parallel()
		box := stitched(t, 0.02, decad.WithGapTolerance(units.Millimeters(0.05)))
		require.Equal(t, decad.BodySolid, box.Kind())
	})

	t.Run("ExactOnly leaves every edge free", func(t *testing.T) {
		t.Parallel()
		sheet := stitched(t, 1e-9, decad.ExactOnly())
		require.Equal(t, decad.BodySheet, sheet.Kind())
		free, err := decad.Edges(decad.Free()).SelectEdges(sheet)
		require.NoError(t, err)
		require.Len(t, free, 8)
	})

	t.Run("a box with no gap stays exact", func(t *testing.T) {
		t.Parallel()
		box := stitched(t, 0)
		volume, err := box.Volume()
		require.NoError(t, err)
		require.Equal(t, decad.Exact, volume.Exactness)
		require.Equal(t, stitchBoxVolume, volume.Value.Base())
	})

	t.Run("a placed tolerant box replays its weld", func(t *testing.T) {
		t.Parallel()
		box := stitched(t, 1e-4)
		slide, err := r3.Translation(r3.Vec{X: 5, Y: 7, Z: 11})
		require.NoError(t, err)
		placed, err := decad.Transform(t.Context(), box, slide)
		require.NoError(t, err)
		require.Equal(t, decad.BodySolid, placed.Kind())
		volume, err := placed.Volume()
		require.NoError(t, err)
		require.LessOrEqual(t, math.Abs(volume.Value.Base()-(stitchBoxVolume+1e-4*100*60)), volume.Bound.Base())
	})

	t.Run("a refused tolerant merge keeps the exact weld", func(t *testing.T) {
		t.Parallel()
		// Every vertex of this 0.001 mm cube lies within the tolerance of its
		// neighbours, so merging them would collapse its edges. The stitch keeps
		// the exact weld and reads exactly what an ExactOnly document reads.
		read := func(opts ...decad.DocumentOption) decad.Measurement {
			doc := decad.New(opts...)
			walls, bottom, top := stitchBoxSheetsAtOffset(t, doc, 0, 0, 0.001, 0.001, 0, 0.001)
			cube, err := decad.Stitch(t.Context(), walls, bottom, top)
			require.NoError(t, err)
			require.Equal(t, decad.BodySolid, cube.Kind())
			volume, err := cube.Volume()
			require.NoError(t, err)
			return volume
		}
		require.Equal(t, read(decad.ExactOnly()), read())
	})
}
