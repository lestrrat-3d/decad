package extent_test

import (
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/extent"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestThroughStopsDenotedLevel(t *testing.T) {
	t.Parallel()
	denoted := big.NewRat(127, 25)
	type stop struct {
		hi, axial float64
		denoted   *big.Rat
	}
	for _, tc := range []struct {
		name  string
		stops []stop
		want  *big.Rat
	}{
		{"one denoted body", []stop{{5.08, 1e-15, denoted}}, denoted},
		{"one body without a denotation", []stop{{5.08, 1e-15, nil}}, nil},
		{"nearer body below the far one's lower end", []stop{{4, 1e-15, nil}, {5.08, 1e-15, denoted}}, denoted},
		{"competing far side within the displacement", []stop{{5.08, 0, nil}, {5.08, 1e-15, denoted}}, nil},
		{"competing far side reaching past the lower end", []stop{{5.08 - 1e-15, 1e-15, nil}, {5.08, 1e-15, denoted}}, nil},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			stops := extent.NewThroughStops[int](r3.Vec{}, r3.Vec{Z: 1}, 1)
			for i, s := range tc.stops {
				require.NoError(t, stops.Add(s.hi, 0, s.axial, i, s.denoted))
			}
			_, _, _, got, err := stops.Finish()
			require.NoError(t, err)
			if tc.want == nil {
				require.Nil(t, got)
				return
			}
			require.NotNil(t, got)
			require.Zero(t, got.Cmp(tc.want))
		})
	}
}
