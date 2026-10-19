package cupwall_test

import (
	"context"
	"math"
	"runtime"
	"strings"
	"testing"

	"github.com/lestrrat-3d/decad/internal/cupwall"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/offset2d"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/stretchr/testify/require"
)

func TestEvaluateCancellationReachesEachProofPhase(t *testing.T) {
	outer := polygonProfile(proofbound.WorkPollInterval + 64)
	cavity, err := offset2d.OffsetProfile(proofbound.NewWorkBudget(t.Context()), outer, 1, 5, 1e-9)
	require.NoError(t, err)
	input := cupwall.Input{Outer: outer, Cavity: cavity, ZOuter: 0, ZCav: 5, ZOpen: 20, Thickness: 5, Inward: true}
	for _, phase := range []string{"coalesceWalksBudget", "Crossing", "reverseLoopRecordWithPoll", "ExactLoopEqual"} {
		t.Run(phase, func(t *testing.T) {
			budget, entered := cancelInFrame(phase)
			_, err := cupwall.Evaluate(budget, input, 15*math.Pi/180, operations())
			require.ErrorIs(t, err, context.Canceled)
			require.True(t, *entered)
		})
	}
}

func TestEvaluateCancellationDuringProfileIntegrals(t *testing.T) {
	line := func(u0, v0, u1, v1 float64) sectionrecord.CurveSegment {
		return sectionrecord.LineSeg{Start: sectionrecord.Point2{U: u0, V: v0}, End: sectionrecord.Point2{U: u1, V: v1}, TStart: 0, TEnd: 1}
	}
	outer := momentinput.Profile{Outer: sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{
		line(0, 0, 100, 0), line(100, 0, 100, 60), line(100, 60, 0, 60), line(0, 60, 0, 0),
	}}}
	cavity, err := offset2d.OffsetProfile(proofbound.NewWorkBudget(t.Context()), outer, 1, 5, 1e-9)
	require.NoError(t, err)
	input := cupwall.Input{Outer: outer, Cavity: cavity, ZOuter: 0, ZCav: 5, ZOpen: 20, Thickness: 5, Inward: true}
	budget, entered := cancelInFrame("IntegralsBudget")
	_, err = cupwall.Evaluate(budget, input, 15*math.Pi/180, operations())
	require.ErrorIs(t, err, context.Canceled)
	require.True(t, *entered)
}

func operations() cupwall.Operations {
	return cupwall.Operations{
		Offset: func(budget *proofbound.WorkBudget, profile momentinput.Profile, sense, thickness float64) (momentinput.Profile, error) {
			return offset2d.OffsetProfile(budget, profile, sense, thickness, 1e-9)
		},
		Equal: momentinput.ExactProfileEqual, Audit: offset2d.AuditOffsetSectionBudget,
		Reverse: offset2d.ReverseLoopRecordBudget,
	}
}

func polygonProfile(count int) momentinput.Profile {
	segments := make([]sectionrecord.CurveSegment, count)
	for i := range segments {
		a := 2 * math.Pi * float64(i) / float64(count)
		b := 2 * math.Pi * float64(i+1) / float64(count)
		segments[i] = sectionrecord.LineSeg{
			Start: sectionrecord.Point2{U: 100 * math.Cos(a), V: 100 * math.Sin(a)},
			End:   sectionrecord.Point2{U: 100 * math.Cos(b), V: 100 * math.Sin(b)}, TStart: 0, TEnd: 1,
		}
	}
	return momentinput.Profile{Outer: sectionrecord.LoopRecord{Segments: segments}}
}

func cancelInFrame(target string) (*proofbound.WorkBudget, *bool) {
	entered, canceled := false, false
	poll := func() error {
		if canceled {
			return context.Canceled
		}
		pcs := make([]uintptr, 32)
		frames := runtime.CallersFrames(pcs[:runtime.Callers(2, pcs)])
		for {
			frame, more := frames.Next()
			if strings.HasSuffix(frame.Function, "."+target) {
				entered, canceled = true, true
				return context.Canceled
			}
			if !more {
				return nil
			}
		}
	}
	return &proofbound.WorkBudget{StepFn: poll, ErrFn: poll}, &entered
}
