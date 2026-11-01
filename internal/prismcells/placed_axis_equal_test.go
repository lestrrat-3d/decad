package prismcells_test

import (
	"context"
	"errors"
	"testing"

	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/prismcells"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestPlacedAxisEqualProfiles(t *testing.T) {
	circle := sectionrecord.CircleSeg{Radius: units.Millimeters(1), CCW: true, TStart: 0, TEnd: 1}
	loop := sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{circle}}
	a, b := overlapPrism(t, 0, 2, loop), overlapPrism(t, 0, 2, loop)
	var err error
	b.Xform, err = r3.Translation(r3.Vec{Z: 2})
	require.NoError(t, err)
	admit := func(a, b prismrecord.Payload, op meshbool.OperationKind) (bool, error) {
		_, _, ok, err := prismcells.AdmitPairBudget(proofbound.NewWorkBudget(t.Context()), op, a, b)
		return ok, err
	}
	ok, err := admit(a, b, meshbool.OpUnion)
	require.NoError(t, err)
	require.True(t, ok)
	for _, op := range []meshbool.OperationKind{meshbool.OpCut, meshbool.OpIntersect} {
		ok, err = admit(a, b, op)
		require.NoError(t, err)
		require.False(t, ok)
	}
	changed := b
	circle.Radius = units.Millimeters(2)
	changed.Profile.Outer.Segments = []sectionrecord.CurveSegment{circle}
	ok, err = admit(a, changed, meshbool.OpUnion)
	require.NoError(t, err)
	require.False(t, ok)
	changed = b
	changed.Xform, err = r3.Translation(r3.Vec{X: 1, Z: 2})
	require.NoError(t, err)
	ok, err = admit(a, changed, meshbool.OpUnion)
	require.NoError(t, err)
	require.False(t, ok)
	for _, change := range []func(*prismrecord.Payload, *prismrecord.Payload){
		func(a, _ *prismrecord.Payload) { a.SectionDelta = 1e-6 },
		func(a, _ *prismrecord.Payload) { a.Z0Delta = 1e-6 },
		func(a, _ *prismrecord.Payload) { a.Z1Delta = 1e-6 },
		func(_, b *prismrecord.Payload) { b.SectionDelta = 1e-6 },
		func(_, b *prismrecord.Payload) { b.Z0Delta = 1e-6 },
		func(_, b *prismrecord.Payload) { b.Z1Delta = 1e-6 },
	} {
		x, y := a, b
		change(&x, &y)
		ok, err = admit(x, y, meshbool.OpUnion)
		require.NoError(t, err)
		require.False(t, ok)
	}
}

func TestPlacedAxisEqualProfilesUsesBudget(t *testing.T) {
	circle := sectionrecord.CircleSeg{Radius: units.Millimeters(1), CCW: true, TStart: 0, TEnd: 1}
	loop := sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{circle}}
	a, b := overlapPrism(t, 0, 2, loop), overlapPrism(t, 0, 2, loop)
	limit := errors.New("budget exhausted")
	for _, n := range []int{0, 1, 2} {
		left := n
		budget := &proofbound.WorkBudget{StepFn: func() error {
			if left == 0 {
				return limit
			}
			left--
			return nil
		}, ErrFn: func() error { return nil }}
		ok, err := prismcells.PlacedAxisProfileAdmitted(budget, meshbool.OpUnion, a, b)
		if n < 2 {
			require.ErrorIs(t, err, limit)
			require.False(t, ok)
		} else {
			require.NoError(t, err)
			require.True(t, ok)
		}
	}
	canceled, cancel := context.WithCancel(t.Context())
	cancel()
	budget := &proofbound.WorkBudget{StepFn: canceled.Err, ErrFn: canceled.Err}
	_, err := prismcells.PlacedAxisProfileAdmitted(budget, meshbool.OpUnion, a, b)
	require.ErrorIs(t, err, context.Canceled)
}

func TestPlacedAxisRectangleRulesRemain(t *testing.T) {
	a := overlapPrism(t, 0, 2, polygonLoop([2]float64{-2, -2}, [2]float64{2, -2},
		[2]float64{2, 2}, [2]float64{-2, 2}))
	b := overlapPrism(t, 0, 2, polygonLoop([2]float64{-1, -1}, [2]float64{1, -1},
		[2]float64{1, 1}, [2]float64{-1, 1}))
	budget := proofbound.NewWorkBudget(t.Context())
	ok, err := prismcells.PlacedAxisProfileAdmitted(budget, meshbool.OpCut, a, b)
	require.NoError(t, err)
	require.True(t, ok)
	ok, err = prismcells.PlacedAxisProfileAdmitted(budget, meshbool.OpIntersect, a, a)
	require.NoError(t, err)
	require.True(t, ok)
	ok, err = prismcells.PlacedAxisProfileAdmitted(budget, meshbool.OpIntersect, a, b)
	require.NoError(t, err)
	require.False(t, ok)
}
