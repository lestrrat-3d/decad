package revolvebore_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/revolveaxis"
	"github.com/lestrrat-3d/decad/internal/revolvebore"
	"github.com/lestrrat-3d/decad/internal/revolveplan"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/stretchr/testify/require"
)

func TestLinearMeridian(t *testing.T) {
	for _, tc := range []struct {
		name   string
		points []sectionrecord.Point2
		ok     bool
	}{
		{"frustum", []sectionrecord.Point2{{}, {V: 10}, {U: 32, V: 8}, {U: 32}}, true},
		{"mixed walls", []sectionrecord.Point2{{}, {V: 10}, {U: 8, V: 9}, {U: 16, V: 9}, {U: 32, V: 8}, {U: 32}}, true},
		{"shoulder", []sectionrecord.Point2{{}, {V: 10}, {U: 8, V: 9}, {U: 8, V: 8}, {U: 32, V: 8}, {U: 32}}, true},
		{"overhang", []sectionrecord.Point2{{}, {V: 10}, {U: 16, V: 9}, {U: 8, V: 8}, {U: 32, V: 8}, {U: 32}}, false},
		{"extra axis run", []sectionrecord.Point2{{}, {V: 10}, {U: 16, V: 9}, {U: 16}, {U: 32}, {U: 32, V: 8}}, false},
	} {
		t.Run(tc.name, func(t *testing.T) {
			loop := sectionrecord.LoopRecord{}
			for i, p := range tc.points {
				loop.Segments = append(loop.Segments, sectionrecord.LineSeg{
					Start: p, End: tc.points[(i+1)%len(tc.points)], TEnd: 1,
				})
			}
			rp := revolveplan.Payload{Profile: momentinput.Profile{Outer: loop}, Ax: revolveaxis.Frame{DU: 1}}
			reading, ok := revolvebore.LinearMeridian(rp)
			require.Equal(t, tc.ok, ok)
			if ok {
				require.Equal(t, 0.0, reading.Low)
				require.Equal(t, 32.0, reading.High)
				require.Equal(t, 8.0, reading.MinRadius)
			}
		})
	}
}
