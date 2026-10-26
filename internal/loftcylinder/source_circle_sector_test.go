package loftcylinder

import (
	"math/big"
	"testing"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// TestSegmentInSectorMatchesTriangle compares segmentInSector with the
// triangleInSector call it replaced, {a, b, b}.
func TestSegmentInSectorMatchesTriangle(t *testing.T) {
	c := fixtureCircle()
	level := big.NewRat(1, 2)
	boxes, err := c.CrossingBox(level, level)
	require.NoError(t, err)

	var pts []proofarith.Xpt
	for i := -10; i <= 10; i++ {
		pts = append(pts, RingMappedPoint(proofarith.XptOf(r3.Vec{X: 3.5, Y: 0.1 * float64(i), Z: 0}), level))
	}
	pts = append(pts,
		RingMappedPoint(proofarith.XptOf(r3.Vec{X: 3.5, Y: 0, Z: 0}), big.NewRat(1, 3)), // z below level
		RingMappedPoint(proofarith.XptOf(r3.Vec{X: 3.5, Y: 0, Z: 0}), big.NewRat(3, 4)), // z above level
		RingMappedPoint(proofarith.XptOf(r3.Vec{X: 0.2, Y: 0.1, Z: 0}), level),
		proofarith.Xpt{},
		proofarith.Xpt{X: big.NewInt(1), Y: big.NewInt(0), Z: big.NewInt(1), W: big.NewInt(0)},
		proofarith.Xpt{X: big.NewInt(1), Y: big.NewInt(0), Z: big.NewInt(1), W: big.NewInt(-2)},
		proofarith.Xpt{X: big.NewInt(7), Y: big.NewInt(0), Z: big.NewInt(1), W: nil},
		proofarith.Xpt{X: nil, Y: big.NewInt(0), Z: big.NewInt(1), W: big.NewInt(2)},
	)
	var sawTrue, sawFalse bool
	for _, a := range pts {
		for _, b := range pts {
			want := triangleInSector(boxes, level, level, [3]proofarith.Xpt{a, b, b})
			require.Equal(t, want, segmentInSector(boxes, level, level, a, b))
			sawTrue = sawTrue || want
			sawFalse = sawFalse || !want
		}
	}
	require.True(t, sawTrue)
	require.True(t, sawFalse)
}
