package momentinput_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/stretchr/testify/require"
)

func TestAxisRectangle(t *testing.T) {
	t.Parallel()
	// The CCW (0,0)-(2,1) loop, with the right side stored reversed: an exact
	// dyadic footprint, compared exactly.
	whole := rectangleProfile(
		rectangleLine(0, 0, 2, 0, 0, 1),
		rectangleLine(2, 0, 2, 1, 1, 0),
		rectangleLine(2, 1, 0, 1, 0, 1),
		rectangleLine(0, 1, 0, 0, 0, 1),
	)
	r, ok := momentinput.AxisRectangle(whole)
	require.True(t, ok)
	require.Equal(t, momentinput.AxisRect{U0: 0, V0: 0, U1: 2, V1: 1}, r)

	for name, p := range map[string]momentinput.Profile{
		// Every line lies on a side of the box, but the top side is missing
		// and the left one is recorded twice.
		"missing side": rectangleProfile(
			rectangleLine(0, 0, 2, 0, 0, 1),
			rectangleLine(2, 0, 2, 1, 0, 1),
			rectangleLine(0, 1, 0, 0, 0, 1),
			rectangleLine(0, 0, 0, 1, 0, 1),
		),
		"partial line": rectangleProfile(
			rectangleLine(0, 0, 2, 0, 0, 0.5),
			rectangleLine(2, 0, 2, 1, 0, 1),
			rectangleLine(2, 1, 0, 1, 0, 1),
			rectangleLine(0, 1, 0, 0, 0, 1),
		),
		"diagonal side": rectangleProfile(
			rectangleLine(0, 0, 2, 0, 0, 1),
			rectangleLine(2, 0, 2, 1, 0, 1),
			rectangleLine(2, 1, 0, 0, 0, 1),
			rectangleLine(0, 0, 0, 0, 0, 1),
		),
		"non-finite": rectangleProfile(
			rectangleLine(0, 0, math.Inf(1), 0, 0, 1),
			rectangleLine(2, 0, 2, 1, 0, 1),
			rectangleLine(2, 1, 0, 1, 0, 1),
			rectangleLine(0, 1, 0, 0, 0, 1),
		),
		// Every side matches the box, so only the finiteness check refuses it.
		"infinite rectangle": rectangleProfile(
			rectangleLine(0, 0, math.Inf(1), 0, 0, 1),
			rectangleLine(math.Inf(1), 0, math.Inf(1), 1, 0, 1),
			rectangleLine(math.Inf(1), 1, 0, 1, 0, 1),
			rectangleLine(0, 1, 0, 0, 0, 1),
		),
		"three lines": rectangleProfile(
			rectangleLine(0, 0, 2, 0, 0, 1),
			rectangleLine(2, 0, 2, 1, 0, 1),
			rectangleLine(2, 1, 0, 0, 0, 1),
		),
	} {
		_, ok := momentinput.AxisRectangle(p)
		require.False(t, ok, name)
	}

	// A hole disqualifies the profile even when the outer loop is a rectangle.
	withHole := whole
	withHole.Holes = []sectionrecord.LoopRecord{whole.Outer}
	_, ok = momentinput.AxisRectangle(withHole)
	require.False(t, ok, "hole")
}
