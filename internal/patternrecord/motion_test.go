package patternrecord

import (
	"math/big"
	"testing"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestLinearMotionUsesDenotedStep(t *testing.T) {
	rp, err := ResolveLinear(r3.NewVec(3, 4, 0), units.Millimeters(10), 2)
	require.NoError(t, err)
	frame, err := r3.NewFrame(r3.Vec{}, r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	motion, err := rp.LinearMotion(frame, r3.Identity(), 1)
	require.NoError(t, err)
	point, charge, err := motion(Point2{U: 1, V: 2})
	require.NoError(t, err)
	require.Equal(t, Point2{U: 7, V: 10}, point)
	require.Zero(t, charge)
	require.Zero(t, proofarith.RationalFloatError(big.NewRat(7, 1), point.U))
}
