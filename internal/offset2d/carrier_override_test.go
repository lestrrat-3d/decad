package offset2d_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/offset2d"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/stretchr/testify/require"
)

// twoWalks is a right-angle corner at (10, 0) whose walks carry anchors and
// tangents a little off the recorded carriers: the leaving walk's start sits
// 1e-3 to the right of its recorded line x = 10 and its tangent is skewed.
func twoWalks() []survey2d.SideWalk {
	arrive := survey2d.SideWalk{}
	arrive.StartU, arrive.StartV, arrive.EndU, arrive.EndV = 0, 0, 10, 0
	arrive.TanInU, arrive.TanOutU = 1, 1
	arrive.Length = 10
	leave := survey2d.SideWalk{}
	leave.StartU, leave.StartV, leave.EndU, leave.EndV = 10.001, 0, 10.001, 10
	leave.TanInU, leave.TanInV, leave.TanOutU, leave.TanOutV = 1e-3, 1, 1e-3, 1
	leave.Length = 10
	return []survey2d.SideWalk{arrive, leave}
}

// TestFilletReadsRecordedCarriers: the carrier override replaces the walk's
// anchor and tangent with the recorded segment's own data, so the feet lie on
// the recorded lines and the turn is the recorded corner's.
func TestFilletReadsRecordedCarriers(t *testing.T) {
	walks := twoWalks()
	line := func(a, b sectionrecord.Point2) sectionrecord.LineSeg {
		return sectionrecord.LineSeg{Start: a, End: b, TStart: 0, TEnd: 1}
	}
	a := line(sectionrecord.Point2{U: 0, V: 0}, sectionrecord.Point2{U: 10, V: 0})
	b := line(sectionrecord.Point2{U: 10, V: 0}, sectionrecord.Point2{U: 10, V: 10})
	ovA := offset2d.LineCarrier(a.Start, a.End, walks[0])
	ovB := offset2d.LineCarrier(b.Start, b.End, walks[1])

	blend, dec, err := offset2d.FilletWithCarriers(walks, 1, 1, 1e-9, &ovA, &ovB)
	require.NoError(t, err)
	require.Equal(t, 1.0, dec.Cross, `the turn is the recorded corner's`)
	require.Equal(t, [2]float64{9, 1}, dec.Center)
	require.Equal(t, sectionrecord.Point2{U: 9, V: 0}, blend.FA)
	require.Equal(t, sectionrecord.Point2{U: 10, V: 1}, blend.FB, `the foot lies on the recorded line x = 10`)

	plain, plainDec, err := offset2d.FilletWithRoots(walks, 1, 1, 1e-9)
	require.NoError(t, err)
	require.NotEqual(t, 1.0, plainDec.Cross)
	require.Greater(t, plain.FB.U, 10.0005, `without the override the walk's anchor and tangent are read`)

	cross, ok := offset2d.CornerCrossWithCarriers(walks, 1, &ovA, &ovB)
	require.True(t, ok)
	require.Equal(t, 1.0, cross)

	chamfer, err := offset2d.ChamferWithCarriers(walks, 1, 1, 1, 1e-9, &ovA, &ovB)
	require.NoError(t, err)
	require.Equal(t, sectionrecord.Point2{U: 9, V: 0}, chamfer.FA)
}

// TestCircleCarrierTangent: a circle's tangent at the corner is rot90 of the
// normalized corner − Center in the walk's sense.
func TestCircleCarrierTangent(t *testing.T) {
	arrive := survey2d.SideWalk{}
	arrive.Th0, arrive.Th1 = 0, 1
	arrive.CU, arrive.CV, arrive.Radius = 0, 0, 5
	arrive.StartU, arrive.StartV = 5, 0
	leave := survey2d.SideWalk{}
	leave.StartU, leave.StartV, leave.EndU, leave.EndV = 5, 0, 5, 10
	leave.TanInU, leave.TanInV = 0, 1
	walks := []survey2d.SideWalk{arrive, leave}
	ovA := offset2d.CircleCarrier(sectionrecord.Point2{}, 5)
	ovB := offset2d.LineCarrier(sectionrecord.Point2{U: 5, V: 0}, sectionrecord.Point2{U: 5, V: 10}, leave)
	// A CCW circle at (5, 0) travels +v: tangent (0, 1), parallel to the
	// line, so the corner is smooth.
	cross, ok := offset2d.CornerCrossWithCarriers(walks, 1, &ovA, &ovB)
	require.True(t, ok)
	require.Zero(t, cross)
}
