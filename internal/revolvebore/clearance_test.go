package revolvebore_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/revolvebore"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestInsideRadiusProvesWholeCarriers(t *testing.T) {
	for _, tc := range []struct {
		name string
		seg  sectionrecord.CurveSegment
		want bool
	}{
		{"line inside", sectionrecord.LineSeg{Start: sectionrecord.Point2{U: 3}, End: sectionrecord.Point2{V: 4}, TEnd: 1}, true},
		{"line touches", sectionrecord.LineSeg{Start: sectionrecord.Point2{U: 5}, End: sectionrecord.Point2{V: 1}, TEnd: 1}, false},
		{"centered circle", sectionrecord.CircleSeg{Radius: units.Millimeters(4), CCW: true, TEnd: 1}, true},
		{"offset circle touches", sectionrecord.CircleSeg{Center: sectionrecord.Point2{U: 1}, Radius: units.Millimeters(4), CCW: true, TEnd: 1}, false},
		{"offset circle clear", sectionrecord.CircleSeg{Center: sectionrecord.Point2{U: 1}, Radius: units.Millimeters(3), CCW: true, TEnd: 1}, true},
		{"nonfinite point", sectionrecord.LineSeg{Start: sectionrecord.Point2{U: math.Inf(1)}, TEnd: 1}, false},
	} {
		t.Run(tc.name, func(t *testing.T) {
			loop := sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{tc.seg}}
			require.Equal(t, tc.want, revolvebore.InsideRadius(loop, 5))
		})
	}
}
