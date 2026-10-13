package revolveaxis_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/revolveangle"
	"github.com/lestrrat-3d/decad/internal/revolveaxis"
	"github.com/lestrrat-3d/decad/internal/revolvemesh"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestDenotationInputRefusals(t *testing.T) {
	frame, err := r3.NewFrame(r3.Vec{}, r3.Vec{X: 1}, r3.Vec{Y: 1})
	require.NoError(t, err)
	d := revolveaxis.DenotationInput{
		Lift:  revolvemesh.RevolveLift{Frame: frame, DU: 1},
		Xform: r3.Identity(),
	}

	require.Nil(t, d.WallDenotation(survey2d.SideWalk{}, revolveaxis.WallFreeform, nil))

	// A zero angle states nothing, so the cap keeps its tag's proof.
	require.Nil(t, d.CapDenotation(revolveangle.Angle{}, 0, true))

	// The cap circle of the plane circle (CU 0, CV 1, radius 2) at angle 0
	// sits about (0, 1, 0) with axis +Z, exactly where the record states it.
	center, axis := r3.Vec{Y: 1}, r3.Vec{Z: 1}
	circle := func(cu float64) survey2d.SegmentWalk {
		return survey2d.SegmentWalk{Kind: survey2d.WalkCircular, CU: cu, CV: 1, Radius: 2}
	}
	bound, ok := d.CapArcCurveBound([]survey2d.SegmentWalk{circle(0)}, revolveangle.Zero(), 0, center, axis, 2)
	require.True(t, ok)
	require.Zero(t, bound)

	bound, ok = d.CapArcCurveBound(nil, revolveangle.Zero(), 0, center, axis, 2)
	require.False(t, ok)
	require.Zero(t, bound)

	// Two circular walks that are not one circle answer false.
	bound, ok = d.CapArcCurveBound([]survey2d.SegmentWalk{circle(0), circle(1)},
		revolveangle.Zero(), 0, center, axis, 2)
	require.False(t, ok)
	require.Zero(t, bound)
}
