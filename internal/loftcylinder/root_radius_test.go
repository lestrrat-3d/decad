package loftcylinder_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/loftcylinder"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestRootRadiusGap(t *testing.T) {
	line := sectionrecord.LineSeg{TStart: 0, TEnd: 1}
	profile := func(root sectionrecord.CurveSegment) momentinput.Profile {
		segments := []sectionrecord.CurveSegment{line, line, line, line, line, root}
		return momentinput.Profile{Outer: sectionrecord.LoopRecord{Segments: segments}}
	}
	arcAt := func(u, v float64) sectionrecord.CurveSegment {
		return sectionrecord.ArcSeg{Start: sectionrecord.Point2{U: u, V: v}}
	}
	identity := r3.Identity()

	gap, ok := loftcylinder.RootRadiusGap(identity,
		[2]momentinput.Profile{profile(arcAt(3, 4)), profile(arcAt(6, 8))}, 5)
	require.False(t, ok)
	require.GreaterOrEqual(t, gap, 5.0)

	// Both roots sit at radius 5, which is an exact square root.
	gap, ok = loftcylinder.RootRadiusGap(identity,
		[2]momentinput.Profile{profile(arcAt(3, 4)), profile(arcAt(3, 4))}, 5)
	require.True(t, ok)
	require.Equal(t, 0.0, gap)

	gap, ok = loftcylinder.RootRadiusGap(identity,
		[2]momentinput.Profile{profile(arcAt(3, 4)), profile(line)}, 5)
	require.False(t, ok)
	require.Equal(t, 0.0, gap)
}
