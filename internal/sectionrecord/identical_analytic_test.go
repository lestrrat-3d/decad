package sectionrecord_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestIdenticalAnalyticSegmentMatchesRecordedIdentity(t *testing.T) {
	line := sectionrecord.LineSeg{Start: sectionrecord.Point2{U: 0, V: 1},
		End: sectionrecord.Point2{U: 2, V: 3}, TEnd: 1}
	circle := sectionrecord.CircleSeg{Center: sectionrecord.Point2{U: 1, V: 2},
		Radius: units.Millimeters(3), CCW: true, TEnd: 1}
	arc := sectionrecord.ArcSeg{Center: sectionrecord.Point2{U: 0, V: 2},
		Start: sectionrecord.Point2{U: 1, V: 2}, End: sectionrecord.Point2{U: -1, V: 2}, TEnd: 1}
	negativeZeroLine := line
	negativeZeroLine.Start.U = math.Copysign(0, -1)
	otherRadius := circle
	otherRadius.Radius = units.Millimeters(4)
	otherArc := arc
	otherArc.End.V = 3
	negativeZeroCircle := circle
	negativeZeroCircle.TStart = math.Copysign(0, -1)
	negativeZeroArc := arc
	negativeZeroArc.Center.U = math.Copysign(0, -1)
	for _, tc := range []struct {
		a, b sectionrecord.CurveSegment
	}{
		{line, line}, {line, negativeZeroLine}, {circle, circle},
		{circle, otherRadius}, {circle, negativeZeroCircle}, {arc, arc},
		{arc, otherArc}, {arc, negativeZeroArc}, {line, circle},
	} {
		require.Equal(t, sectionrecord.IdenticalRecord(tc.a, tc.b),
			sectionrecord.IdenticalAnalyticSegment(tc.a, tc.b))
	}
}
