package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/decadtest"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file is agent-docs/fillet-chord-design.md's public-surface tests for
// FilletChord on the 20 x 20 x 10 block of chamfer_angle_test.go.

func TestFilletChordSetsTheRadius(t *testing.T) {
	t.Parallel()
	for _, chord := range []float64{2, 4, 6} {
		t.Run("", func(t *testing.T) {
			t.Parallel()
			doc, block := angleBlock(t)
			out, err := decad.FilletChord(t.Context(), block, angleCornerEdge(), units.Millimeters(chord))
			require.NoError(t, err)
			require.Equal(t, []*decad.Body{out}, doc.Bodies())
			// A quarter-turn arc of radius chord/√2 removes (1 - π/4)·r² per mm of height.
			r2 := chord * chord / 2
			decadtest.MeasuresVolume(t, out, units.CubicMillimeters(4000-(1-math.Pi/4)*r2*10),
				decadtest.WithinRel(units.Scalar(1e-9)))
			operations := doc.OperationHistory().Operations()
			require.Equal(t, decad.OperationFillet, operations[len(operations)-1].Kind())
		})
	}
}

func TestFilletChordRefusals(t *testing.T) {
	t.Parallel()
	cases := []struct {
		name  string
		chord units.Value
		want  error
	}{
		{"zero chord", units.Millimeters(0), decad.ErrDegenerate},
		{"negative chord", units.Millimeters(-2), decad.ErrNegativeMagnitude},
		{"chord as an angle", units.Degrees(4), decad.ErrUnitKind},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			doc, block := angleBlock(t)
			_, err := decad.FilletChord(t.Context(), block, angleCornerEdge(), tc.chord)
			require.ErrorIs(t, err, tc.want)
			require.Equal(t, []*decad.Body{block}, doc.Bodies(), "a refused fillet leaves the block live and alone")
		})
	}
}
