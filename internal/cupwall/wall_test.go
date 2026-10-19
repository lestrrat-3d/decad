package cupwall_test

import (
	"context"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/cupwall"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/offset2d"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/stretchr/testify/require"
)

func TestEvaluateRequiresExactOffsetMorphology(t *testing.T) {
	line := func(u0, v0, u1, v1 float64) sectionrecord.CurveSegment {
		return sectionrecord.LineSeg{
			Start: sectionrecord.Point2{U: u0, V: v0},
			End:   sectionrecord.Point2{U: u1, V: v1}, TStart: 0, TEnd: 1,
		}
	}
	outer := momentinput.Profile{Outer: sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{
		line(0, 0, 100, 0), line(100, 0, 100, 60),
		line(100, 60, 0, 60), line(0, 60, 0, 0),
	}}}
	budget := proofbound.NewWorkBudget(t.Context())
	cavity, err := offset2d.OffsetProfile(budget, outer, 1, 5, 1e-9)
	require.NoError(t, err)
	ops := cupwall.Operations{
		Offset: func(budget *proofbound.WorkBudget, profile momentinput.Profile, sense, thickness float64) (momentinput.Profile, error) {
			return offset2d.OffsetProfile(budget, profile, sense, thickness, 1e-9)
		},
		Equal:   momentinput.ExactProfileEqual,
		Audit:   offset2d.AuditOffsetSectionBudget,
		Reverse: offset2d.ReverseLoopRecordBudget,
	}
	input := cupwall.Input{
		Outer: outer, Cavity: cavity,
		ZOuter: 0, ZCav: 5, ZOpen: 20, Thickness: 5, Inward: true,
	}
	out, err := cupwall.Evaluate(budget, input, 15*math.Pi/180, ops)
	require.NoError(t, err)
	require.True(t, out.OK)
	require.NotNil(t, out.Reading)
	require.Equal(t, 5.0, *out.Reading)

	bad := input
	bad.Cavity.Outer.Segments = append([]sectionrecord.CurveSegment(nil), input.Cavity.Outer.Segments...)
	for i, segment := range bad.Cavity.Outer.Segments {
		moved := segment.(sectionrecord.LineSeg)
		moved.Start.U += 0.25
		moved.End.U += 0.25
		bad.Cavity.Outer.Segments[i] = moved
	}
	out, err = cupwall.Evaluate(proofbound.NewWorkBudget(t.Context()), bad, 15*math.Pi/180, ops)
	require.NoError(t, err)
	require.False(t, out.OK)

	canceled, cancel := context.WithCancel(t.Context())
	cancel()
	_, err = cupwall.Evaluate(proofbound.NewWorkBudget(canceled), input, 15*math.Pi/180, ops)
	require.ErrorIs(t, err, context.Canceled)
}
