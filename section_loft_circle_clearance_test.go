package decad

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestLoftSectionCircularHoleClearanceAcrossLevels(t *testing.T) {
	circle := func(x, y, radius float64) circleSeg {
		return circleSeg{
			Center: sectionrecord.Point2{U: x, V: y},
			Radius: units.Millimeters(radius),
		}
	}
	stationary := [2]circleSeg{circle(0, 0, 1), circle(0, 0, 1)}
	passing := [2]circleSeg{circle(2, -1, 1), circle(2, 1, 1)}
	require.False(t, loftSectionCircleGapAllLevels(stationary, passing, false),
		"holes tangent only between valid caps must refuse")
	separated := [2]circleSeg{circle(3, -1, 1), circle(3, 1, 1)}
	require.True(t, loftSectionCircleGapAllLevels(stationary, separated, false))

	outer := [2]circleSeg{circle(0, 0, 5), circle(0, 0, 5)}
	movingHole := [2]circleSeg{circle(-3, 0, 1), circle(3, 0, 1)}
	require.True(t, loftSectionCircleGapAllLevels(outer, movingHole, true))
	tangentOuter := [2]circleSeg{circle(0, 0, 4), circle(0, 0, 4)}
	require.False(t, loftSectionCircleGapAllLevels(tangentOuter, movingHole, true))
}
