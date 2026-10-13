package prismcells_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismcells"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestScaleSourceProfile(t *testing.T) {
	square := func(first sectionrecord.LineSeg) momentinput.Profile {
		return momentinput.Profile{Outer: sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{
			first,
			sectionrecord.LineSeg{Start: sectionrecord.Point2{U: 1}, End: sectionrecord.Point2{U: 1, V: 1}, TStart: 0, TEnd: 1},
			sectionrecord.LineSeg{Start: sectionrecord.Point2{U: 1, V: 1}, End: sectionrecord.Point2{V: 1}, TStart: 0, TEnd: 1},
			sectionrecord.LineSeg{Start: sectionrecord.Point2{V: 1}, End: sectionrecord.Point2{}, TStart: 0, TEnd: 1},
		}}}
	}
	base := sectionrecord.LineSeg{End: sectionrecord.Point2{U: 1}, TStart: 0, TEnd: 1}
	reversed := base
	reversed.TStart, reversed.TEnd = 1, 0
	partial := base
	partial.TStart = 0.5
	circle := sectionrecord.CircleSeg{Radius: units.Millimeters(1), CCW: true, TStart: 0, TEnd: 1}

	tests := []struct {
		name         string
		profile      momentinput.Profile
		curved, elig bool
	}{
		{"square", square(base), false, true},
		{"reversed edge", square(reversed), false, true},
		{"partial edge", square(partial), false, false},
		{"whole circle", momentinput.Profile{Outer: sectionrecord.LoopRecord{
			Segments: []sectionrecord.CurveSegment{circle}}}, true, true},
		{"circle in a two-segment loop", momentinput.Profile{Outer: sectionrecord.LoopRecord{
			Segments: []sectionrecord.CurveSegment{circle, base}}}, false, false},
		{"empty loop", momentinput.Profile{}, false, false},
	}
	for _, tc := range tests {
		t.Run(tc.name, func(t *testing.T) {
			curved, eligible := prismcells.ScaleSourceProfile(tc.profile)
			require.Equal(t, tc.curved, curved)
			require.Equal(t, tc.elig, eligible)
		})
	}
}
