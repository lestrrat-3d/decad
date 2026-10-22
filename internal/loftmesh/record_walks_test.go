package loftmesh_test

import (
	"reflect"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"

	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/loftmesh"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
)

// walkSection is a closed section of a fit spline and a closing line.
func walkSection() momentinput.Profile {
	fit := []sectionrecord.Point2{{U: 0, V: 0}, {U: 1, V: 0.5}, {U: 2, V: 0.75}, {U: 3, V: 0.5}, {U: 4, V: 0}}
	return momentinput.Profile{Outer: sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{
		sectionrecord.FitSplineSeg{Fit: fit, TStart: 0, TEnd: 1},
		sectionrecord.LineSeg{Start: fit[4], End: fit[0], TStart: 0, TEnd: 1},
	}}}
}

// TestValidateRecordWalksReusesTheWalkOfAnIdenticalRecord pins the reuse to what walking
// both records would do: the walks are equal and the second counter spends exactly what
// walking its own copy of the record spends.
func TestValidateRecordWalksReusesTheWalkOfAnIdenticalRecord(t *testing.T) {
	t.Parallel()
	p0, p1 := walkSection(), walkSection()
	pl0 := sectionrecord.PlaneRecord{Origin: r3.Vec{}, U: r3.NewVec(1, 0, 0), V: r3.NewVec(0, 1, 0)}
	pl1 := sectionrecord.PlaneRecord{Origin: r3.NewVec(0, 0, 5), U: r3.NewVec(1, 0, 0), V: r3.NewVec(0, 1, 0)}

	work0, work1 := freeform.NewFreeformWork(), freeform.NewFreeformWork()
	_, walks0, walks1, err := loftmesh.ValidateRecordWalks(p0, p1, pl0, pl1, nil, work0, work1)
	require.NoError(t, err)
	require.True(t, reflect.DeepEqual(walks0, walks1), "an identical record walks the same way")

	// The cost of walking the record once, on a counter of its own.
	alone := freeform.NewFreeformWork()
	for _, seg := range p0.Outer.Segments {
		_, err := boundarywalk.WalkOf(seg, alone)
		require.NoError(t, err)
	}
	require.Positive(t, alone.Spent, "the fit spline's walk is metered")
	require.Equal(t, alone.Spent, work0.Spent)
	require.Equal(t, alone.Spent, work1.Spent, "the second counter is charged what its own walk would cost")
	require.Equal(t, alone.ReconstructionSpent, work1.ReconstructionSpent)
}

// TestValidateRecordWalksWalksADifferentRecordItself keeps the other half: a record that
// differs in one coordinate gets its own walk.
func TestValidateRecordWalksWalksADifferentRecordItself(t *testing.T) {
	t.Parallel()
	p0, p1 := walkSection(), walkSection()
	fit := p1.Outer.Segments[0].(sectionrecord.FitSplineSeg)
	fit.Fit = append([]sectionrecord.Point2(nil), fit.Fit...)
	fit.Fit[2].V = 0.8
	p1.Outer.Segments[0] = fit
	pl0 := sectionrecord.PlaneRecord{Origin: r3.Vec{}, U: r3.NewVec(1, 0, 0), V: r3.NewVec(0, 1, 0)}
	pl1 := sectionrecord.PlaneRecord{Origin: r3.NewVec(0, 0, 5), U: r3.NewVec(1, 0, 0), V: r3.NewVec(0, 1, 0)}
	_, walks0, walks1, err := loftmesh.ValidateRecordWalks(p0, p1, pl0, pl1, nil, freeform.NewFreeformWork(), freeform.NewFreeformWork())
	require.NoError(t, err)
	require.False(t, reflect.DeepEqual(walks0[0][0], walks1[0][0]))
}
