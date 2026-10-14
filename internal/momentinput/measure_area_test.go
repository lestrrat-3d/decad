package momentinput_test

import (
	"math"
	"slices"
	"testing"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func areaRectangle(u, v, width, height float64) momentinput.Profile {
	points := [4]sectionrecord.Point2{
		{U: u, V: v}, {U: u + width, V: v},
		{U: u + width, V: v + height}, {U: u, V: v + height},
	}
	segments := make([]sectionrecord.CurveSegment, len(points))
	for i := range points {
		segments[i] = sectionrecord.LineSeg{Start: points[i], End: points[(i+1)%len(points)], TStart: 0, TEnd: 1}
	}
	return momentinput.Profile{Outer: sectionrecord.LoopRecord{Segments: segments}}
}

func TestEvaluatorAreaMatchesFullAreaIntegral(t *testing.T) {
	p := sectionrecord.Point2{U: 3, V: -4}
	cases := []struct {
		name    string
		profile momentinput.Profile
	}{
		{"square", areaRectangle(0, 0, 20, 10)},
		{"fractional", areaRectangle(1.25, -7.125, 0.125, 0.375)},
		{"large_offset", areaRectangle(math.Ldexp(1, 300), -math.Ldexp(1, 300),
			math.Ldexp(1, 250), math.Ldexp(1, 249))},
		{"small_area", areaRectangle(0, 0, math.Ldexp(1, -540), math.Ldexp(1, -540))},
		{"collapsed", areaRectangle(1, 2, 0, 0)},
		{"circle", momentinput.Profile{Outer: sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{
			sectionrecord.CircleSeg{Center: p, Radius: units.Millimeters(10), CCW: true, TStart: 0, TEnd: 1},
		}}}},
	}
	hole := areaRectangle(0, 0, 20, 10)
	hole.Holes = []sectionrecord.LoopRecord{rectangleLoop(false)}
	cases = append(cases, struct {
		name    string
		profile momentinput.Profile
	}{"hole", hole})
	negative := areaRectangle(0, 0, 2, 1)
	negative.Outer = rectangleLoop(false)
	cases = append(cases, struct {
		name    string
		profile momentinput.Profile
	}{"reversed", negative})

	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			before := slices.Clone(tc.profile.Outer.Segments)
			oldWork, newWork := freeform.NewFreeformWork(), freeform.NewFreeformWork()
			old, oldErr := tc.profile.EvaluatorIntegrals(freeform.MomentAreaOrder, oldWork)
			got, gotErr := tc.profile.EvaluatorArea(newWork)
			require.Equal(t, oldErr == nil, gotErr == nil)
			if oldErr != nil {
				require.EqualError(t, gotErr, oldErr.Error())
			} else {
				require.Equal(t, math.Float64bits(old.Area), math.Float64bits(got))
			}
			require.Equal(t, oldWork.Spent, newWork.Spent)
			require.Equal(t, before, tc.profile.Outer.Segments)
		})
	}
}

func BenchmarkEvaluatorArea(b *testing.B) {
	profile := areaRectangle(0, 0, 20, 10)
	for _, tc := range []struct {
		name string
		run  func() error
	}{
		{"full", func() error {
			_, err := profile.EvaluatorIntegrals(freeform.MomentAreaOrder, freeform.NewFreeformWork())
			return err
		}},
		{"area_only", func() error {
			_, err := profile.EvaluatorArea(freeform.NewFreeformWork())
			return err
		}},
	} {
		b.Run(tc.name, func(b *testing.B) {
			b.ReportAllocs()
			for b.Loop() {
				if err := tc.run(); err != nil {
					b.Fatal(err)
				}
			}
		})
	}
}
