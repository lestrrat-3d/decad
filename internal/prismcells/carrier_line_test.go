package prismcells_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/prismcells"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/stretchr/testify/require"
)

func TestSameLineCarrier(t *testing.T) {
	line := func(x0, y0, x1, y1 float64) sectionrecord.LineSeg {
		return sectionrecord.LineSeg{
			Start: sectionrecord.Point2{U: x0, V: y0},
			End:   sectionrecord.Point2{U: x1, V: y1},
		}
	}
	base := line(0, 0, 2, 2)
	for _, tc := range []struct {
		name string
		line sectionrecord.LineSeg
		want bool
	}{
		{"separate span", line(3, 3, 4, 4), true},
		{"reversed span", line(4, 4, 3, 3), true},
		{"parallel offset", line(0, 1, 2, 3), false},
		{"crossing", line(1, 0, 1, 2), false},
		{"one endpoint only", line(1, 1, 1, 2), false},
	} {
		t.Run(tc.name, func(t *testing.T) {
			require.Equal(t, tc.want, prismcells.SameLineCarrier(base, tc.line))
		})
	}
}
