package prismextent_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismextent"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/stretchr/testify/require"
)

// squareLoop is a counterclockwise line square [x0, x1] x [y0, y1].
func squareLoop(x0, y0, x1, y1 float64) sectionrecord.LoopRecord {
	corners := [4]sectionrecord.Point2{{U: x0, V: y0}, {U: x1, V: y0}, {U: x1, V: y1}, {U: x0, V: y1}}
	segs := make([]sectionrecord.CurveSegment, 4)
	for i := range segs {
		segs[i] = sectionrecord.LineSeg{Start: corners[i], End: corners[(i+1)%4], TStart: 0, TEnd: 1}
	}
	return sectionrecord.LoopRecord{Segments: segs}
}

func TestProfileBoundaryExtremesReadsHoles(t *testing.T) {
	// The hole lies outside the outer square on purpose: the function does
	// not validate the profile, so only a reader that visits the hole loops
	// reaches u = 4.
	profile := momentinput.Profile{
		Outer: squareLoop(0, 0, 2, 2),
		Holes: []sectionrecord.LoopRecord{squareLoop(3, 3, 4, 4)},
	}
	lo, hi, bound, err := prismextent.ProfileBoundaryExtremes(t.Context(), profile, 1, 0, freeform.NewFreeformWork(), nil)
	require.NoError(t, err)
	require.Equal(t, 0.0, lo)
	require.Equal(t, 4.0, hi)
	require.Equal(t, 0.0, bound)

	nanLo, nanHi, nanBound, err := prismextent.ProfileBoundaryExtremes(t.Context(), profile, math.NaN(), 0, freeform.NewFreeformWork(), nil)
	require.Zero(t, nanLo+nanHi+nanBound)
	require.Error(t, err)

	other := momentinput.Profile{Outer: squareLoop(0, 0, 1, 1)}
	walks := &momentinput.ProfileWalks{Profile: other}
	misLo, misHi, misBound, err := prismextent.ProfileBoundaryExtremes(t.Context(), profile, 1, 0, freeform.NewFreeformWork(), walks)
	require.Zero(t, misLo+misHi+misBound)
	require.ErrorIs(t, err, momentinput.ErrResolvedWalksMismatch)
}
