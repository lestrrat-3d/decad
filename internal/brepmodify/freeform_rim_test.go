package brepmodify

import (
	"context"
	"errors"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/opdiagnostic"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topo/topotest"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// The real rim cutter builder runs in both cases. The Boolean hook supplies
// bounded accepted-step summaries to test their transport independently of
// which repairs a particular cutter's contact arrangement happens to require.
func TestCutRimsAggregatesOnlyAnAcceptedAttempt(t *testing.T) {
	t.Parallel()
	frame, err := r3.NewFrame(r3.Vec{}, r3.Vec{X: 1}, r3.Vec{Y: 1})
	require.NoError(t, err)
	pp := prismrecord.Payload{Profile: momentinput.Profile{Outer: rimWarningCircle()},
		Frame: frame, Z0: 0, Z1: 10, Xform: r3.Identity()}
	refusal := errors.New("the second private Cut refused")
	for _, failSecond := range []bool{false, true} {
		name := "both accepted"
		if failSecond {
			name = "later failure"
		}
		t.Run(name, func(t *testing.T) {
			env := topo.Env{Doc: &topo.Document{}, Ref: 3, Alloc: topotest.Alloc{}}
			source := &topo.Body{OperationWarnings: []opdiagnostic.Record{
				{Code: opdiagnostic.CodeStitchMergeApplied, DisplacementBound: 8},
			}}
			calls := 0
			hooks := Hooks{Boolean: func(_ context.Context, op meshbool.OperationKind,
				_ topo.Env, _, tool *topo.Body) (*topo.Body, error) {
				calls++
				require.Equal(t, meshbool.OpCut, op)
				require.True(t, tool.Solid)
				require.Positive(t, tool.Volume.Value.Base())
				if failSecond && calls == 2 {
					return nil, refusal
				}
				return &topo.Body{OperationWarnings: []opdiagnostic.Record{{
					Code: opdiagnostic.CodeGapShiftApplied, Operand: 1,
					DisplacementBound: float64(calls) * 0.001, AllowedReach: 0.01,
				}}}, nil
			}}
			out, used, retry, err := cutRims(t.Context(), env, hooks, source, pp,
				[]rim{{start: true}, {start: false}}, 0.5, rimAttempts[len(rimAttempts)-1], 0)
			if failSecond {
				require.ErrorIs(t, err, refusal)
			} else {
				require.NoError(t, err)
			}
			require.Equal(t, 2, calls)
			require.Equal(t, 4, used)
			require.Len(t, source.OperationWarnings, 1)
			require.Equal(t, opdiagnostic.CodeStitchMergeApplied, source.OperationWarnings[0].Code)
			if failSecond {
				require.ErrorIs(t, err, refusal)
				require.Nil(t, out)
				require.True(t, retry)
				return
			}
			require.NoError(t, err)
			require.False(t, retry)
			require.Len(t, out.OperationWarnings, 1)
			warning := out.OperationWarnings[0]
			require.Equal(t, opdiagnostic.CodeGapShiftApplied, warning.Code)
			require.Equal(t, -1, warning.Operand)
			require.GreaterOrEqual(t, warning.DisplacementBound, 0.003)
			require.Less(t, warning.DisplacementBound, 0.0031)
		})
	}
}

func rimWarningCircle() sectionrecord.LoopRecord {
	var loop sectionrecord.LoopRecord
	for s := range 8 {
		fit := make([]sectionrecord.Point2, 9)
		for i := range fit {
			a := 2 * math.Pi * (float64(s) + float64(i)/8) / 8
			fit[i] = sectionrecord.Point2{U: 10 * math.Cos(a), V: 10 * math.Sin(a)}
		}
		if s == 7 {
			fit[8] = sectionrecord.Point2{U: 10}
		}
		loop.Segments = append(loop.Segments, sectionrecord.FitSplineSeg{Fit: fit, TStart: 0, TEnd: 1})
	}
	return loop
}
