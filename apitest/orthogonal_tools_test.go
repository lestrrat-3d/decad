package apitest_test

import (
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file is agent-docs/orthogonal-boolean-design.md B13's public-surface test: a feature built from tool bodies
// reads exactly on a faceted solid every face of which squares to a world axis.

func TestEmbossOnAMeshSolidReadsExactly(t *testing.T) {
	t.Parallel()
	for _, tc := range []struct {
		name   string
		mode   decad.EmbossMode
		volume float64
		top    float64
	}{
		{"raised", decad.EmbossRaised, 40*30*10 + 10*6*2, 12},
		{"engraved", decad.EmbossEngraved, 40*30*10 - 10*6*2, 10},
	} {
		t.Run(tc.name, func(t *testing.T) {
			doc := decad.New()
			plate := blockIn(t, doc, r3.NewVec(0, 0, 0), r3.NewVec(40, 30, 10))
			s, profiles := embossSketch(t, 10, [4]float64{10, 10, 20, 16})
			out, err := decad.Emboss(t.Context(), plate, s, profiles, decad.Faces(decad.Facing(r3.NewVec(0, 0, 1))),
				tc.mode, units.Millimeters(2))
			require.NoError(t, err)
			require.Equal(t, []*decad.Body{out}, doc.Bodies())
			volume, err := out.Volume()
			require.NoError(t, err)
			require.Equal(t, decad.Exact, volume.Exactness, "the relief meets the plate on a grid, with no rounding")
			require.Equal(t, tc.volume, volume.Value.Base())
			require.Zero(t, volume.Bound.Base())
			box, err := out.Bounds()
			require.NoError(t, err)
			require.Equal(t, tc.top, box.Max.Z)
		})
	}
}
