package decad

import (
	"math"
	"testing"

	"github.com/stretchr/testify/require"
)

// This file is a deliberate internal-test exception (like
// selector_internal_test.go): the kernel's sub-resolution web semantic is
// unreachable through the public API — sketch's arrangement refuses the
// near-identical concentric circles that would carry a web under the
// kernel's candidate floor — yet the kernel must still never let such a
// candidate underwrite a pass if a future geometry source produces one.

func TestWallKernelFlagsOffJunctionSubTolerance(t *testing.T) {
	// Two concentric full circles 2e-8 apart at scale 10: the web disk is
	// under the candidate floor (4·1e-9·10) and no junction vertex is
	// supplied, so the kernel must flag it rather than silently treat the
	// boundary as web-free.
	outer, ok := arcElem(0, 0, 10, 0, 2*math.Pi, true)
	require.True(t, ok)
	inner, ok := arcElem(0, 0, 10-2e-8, 2*math.Pi, 0, true)
	require.True(t, ok)
	k := newWallKernel([]surveyElem{outer, inner}, nil, nil, 15*math.Pi/180, 0, false, math.Inf(1))
	out := k.run()
	require.True(t, out.subTolFar, `an off-junction sub-tolerance candidate must be flagged`)
}

func TestWallKernelCleanProfileDoesNotFlag(t *testing.T) {
	// A plain 100×60 rectangle produces no off-junction sub-tolerance
	// candidates: the flag stays clear and the reading is decided.
	pts := [][2]float64{{0, 0}, {100, 0}, {100, 60}, {0, 60}}
	elems := make([]surveyElem, 0, 4)
	for i := range pts {
		e, ok := lineElem(pts[i][0], pts[i][1], pts[(i+1)%4][0], pts[(i+1)%4][1])
		require.True(t, ok)
		elems = append(elems, e)
	}
	k := newWallKernel(elems, nil, pts, 15*math.Pi/180, 0, false, math.Inf(1))
	out := k.run()
	require.True(t, out.ok)
	require.False(t, out.subTolFar)
	require.True(t, out.hasSpan)
	require.InDelta(t, 60.0, out.span, 1e-9)
}
