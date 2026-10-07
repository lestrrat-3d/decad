package box_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/pair"
	"github.com/lestrrat-3d/decad/internal/pair/box"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/stretchr/testify/require"
)

func TestAxisBoxGapRefusesUnrepresentableLength(t *testing.T) {
	dy := proof.MustDyOf
	a := box.AxisBox{
		Lo: [3]proof.Dyadic{dy(-math.MaxFloat64), dy(0), dy(0)},
		Hi: [3]proof.Dyadic{dy(-0.75 * math.MaxFloat64), dy(1), dy(1)},
	}
	b := box.AxisBox{
		Lo: [3]proof.Dyadic{dy(0.75 * math.MaxFloat64), dy(0), dy(0)},
		Hi: [3]proof.Dyadic{dy(math.MaxFloat64), dy(1), dy(1)},
	}
	result := box.ClassifyAxisBoxes(a, b, box.AxisBoxRequest{PointResolutionMM: 1})
	require.Equal(t, pair.Undecided, result.Relation)
	require.Equal(t, pair.NoGapProof, result.Reason)
	require.Nil(t, result.Gap)
	require.Nil(t, result.Patch)
}
