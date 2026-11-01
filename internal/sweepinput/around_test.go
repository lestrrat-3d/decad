package sweepinput_test

import (
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/sweeparc"
	"github.com/lestrrat-3d/decad/internal/sweepinput"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestAroundExactTangentsAndTurns(t *testing.T) {
	start := r3.NewVec(0, 0, 5)
	segment := sweepinput.Segment{Arc: true, Around: true, Center: r3.NewVec(5, 0, 5),
		Axis: r3.NewVec(0, 2, 0), End: r3.NewVec(10, 0, 5)}
	record, err := sweepinput.RecordSegment(start, segment, 0)
	require.NoError(t, err)
	require.Zero(t, record.ArcAngle.Turn.Cmp(big.NewRat(1, 2)))
	before := sweeparc.VecOf(r3.NewVec(0, 0, 5))
	after := sweeparc.VecOf(r3.NewVec(0, 0, -5))
	require.True(t, sweeparc.IsZero(sweeparc.Cross(before, record.TangentIn)))
	require.Positive(t, sweeparc.Dot(before, record.TangentIn).Sign())
	require.True(t, sweeparc.IsZero(sweeparc.Cross(after, record.TangentOut)))
	require.Positive(t, sweeparc.Dot(after, record.TangentOut).Sign())
	segment.Axis.Y = 1
	scaled, err := sweepinput.RecordSegment(start, segment, 0)
	require.NoError(t, err)
	require.True(t, sweeparc.IsZero(sweeparc.Cross(record.TangentIn, scaled.TangentIn)))
	require.Positive(t, sweeparc.Dot(record.TangentIn, scaled.TangentIn).Sign())
	segment.Axis.Y = -2
	reversed, err := sweepinput.RecordSegment(start, segment, 0)
	require.NoError(t, err)
	require.Negative(t, sweeparc.Dot(record.TangentIn, reversed.TangentIn).Sign())
	require.Zero(t, reversed.ArcAngle.Turn.Cmp(big.NewRat(1, 2)))
	segment.Center = r3.Vec{}
	segment.Axis = r3.NewVec(0, 0, 1)
	segment.End = r3.NewVec(0, 5, 0)
	quarter, err := sweepinput.RecordSegment(r3.NewVec(5, 0, 0), segment, 0)
	require.NoError(t, err)
	require.Zero(t, quarter.ArcAngle.Turn.Cmp(big.NewRat(1, 4)))
	segment.Axis.Z = -1
	major, err := sweepinput.RecordSegment(r3.NewVec(5, 0, 0), segment, 0)
	require.NoError(t, err)
	require.Zero(t, major.ArcAngle.Turn.Cmp(big.NewRat(3, 4)))
}
