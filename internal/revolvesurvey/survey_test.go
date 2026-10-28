package revolvesurvey_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/revolvesurvey"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/stretchr/testify/require"
)

// TestOnAxisCentre pins each conjunct of the carrier-pair gate: only a
// circular walk whose recorded centre has an exactly zero radial coordinate,
// with a zero bound on it, is on the axis.
func TestOnAxisCentre(t *testing.T) {
	circular := func(cv, cvBound float64) survey2d.SideWalk {
		return survey2d.SideWalk{Kind: survey2d.WalkCircular, CU: 5, CV: cv, CVBound: cvBound, Radius: 5}
	}
	require.True(t, revolvesurvey.OnAxisCentre(circular(0, 0)))
	require.False(t, revolvesurvey.OnAxisCentre(circular(1e-13, 0)), "a centre off the axis")
	require.False(t, revolvesurvey.OnAxisCentre(circular(0, 1e-15)), "a centre the axis frame cannot prove on the axis")
	line := survey2d.SideWalk{Kind: survey2d.WalkLine, EndU: 10}
	require.False(t, revolvesurvey.OnAxisCentre(line), "a line walk carries no centre")
}
