package stackedrecord

import (
	"context"
	"errors"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismcells"
	"github.com/lestrrat-3d/decad/internal/prismplacement"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

type blindValidationMeter struct{ spent, limit uint64 }

var errBlindValidationLimit = errors.New("blind validation limit")

func (m *blindValidationMeter) SpendWork(ctx context.Context, n uint64) error {
	if err := ctx.Err(); err != nil {
		return err
	}
	if n > m.limit-m.spent {
		return errBlindValidationLimit
	}
	m.spent += n
	return nil
}
func (*blindValidationMeter) ReserveRecords(context.Context, uint64) error { return nil }
func (*blindValidationMeter) ReserveBytes(context.Context, uint64) error   { return nil }

func TestReplayValidateBlindGroup(t *testing.T) {
	circle := func(x float64, ccw bool) sectionrecord.LoopRecord {
		c := sectionrecord.CircleSeg{Center: sectionrecord.Point2{U: x}, Radius: units.Millimeters(1), CCW: ccw, TStart: 0, TEnd: 1}
		if !ccw {
			c.TStart, c.TEnd = 1, 0
		}
		return sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{c}}
	}
	target := prismrecord.Payload{Z0: 0, Z1: 10, Profile: momentinput.Profile{Outer: circle(0, true), Holes: []sectionrecord.LoopRecord{circle(1, false)}}}
	cut := momentinput.Profile{Outer: target.Profile.Outer, Holes: []sectionrecord.LoopRecord{target.Profile.Holes[0], circle(3, false), circle(6, false)}}
	section := prismcells.ReplayBlindSection{Profile: cut, HoleRegions: []int{0, 1}, SectionDelta: 1e-12}
	out := Payload{SectionDelta: section.SectionDelta, Slabs: []Slab{{Z0: 0, Z1: 6, Regions: []momentinput.Profile{target.Profile}}, {Z0: 6, Z1: 10, Regions: []momentinput.Profile{cut}}},
		Interfaces: []Interface{{LowerExposed: []momentinput.Profile{{Outer: circle(3, true)}, {Outer: circle(6, true)}}}}}
	meter := proofbound.NewReplayTrialBudget()
	ok, err := replayValidateBlindGroup(t.Context(), out, target, section, 6, true, meter)
	require.NoError(t, err)
	require.True(t, ok)
	work, _ := meter.Usage()
	for _, tc := range []struct {
		limit uint64
		pass  bool
	}{{work, true}, {work - 1, false}} {
		m := &blindValidationMeter{limit: tc.limit}
		ok, err := replayValidateBlindGroup(t.Context(), out, target, section, 6, true, m)
		if tc.pass {
			require.NoError(t, err)
			require.True(t, ok)
		} else {
			require.ErrorIs(t, err, errBlindValidationLimit)
			require.False(t, ok)
		}
	}
	target.Z1Delta = 1
	target.Z1Denoted = prismplacement.DenoteLevel(big.NewRat(10, 1), 10, 1)
	out.Slabs[1].Z1Delta = 1
	out.Slabs[1].Z1Denoted = prismplacement.DenoteLevel(big.NewRat(19, 2), 10, 1)
	ok, err = replayValidateBlindGroup(t.Context(), out, target, section, 6, true, proofbound.NewReplayTrialBudget())
	require.NoError(t, err)
	require.False(t, ok)
	target.Z1Delta = 0
	target.Z1Denoted = prismplacement.LevelDenotation{}
	out.Slabs[1].Z1Delta = 0
	out.Slabs[1].Z1Denoted = prismplacement.LevelDenotation{}
	out.ClosedShell = true
	ok, err = replayValidateBlindGroup(t.Context(), out, target, section, 6, true, proofbound.NewReplayTrialBudget())
	require.NoError(t, err)
	require.False(t, ok)
	out.ClosedShell = false
	out.Interfaces[0].LowerExposed[1].Outer = circle(7, true)
	ok, err = replayValidateBlindGroup(t.Context(), out, target, section, 6, true, proofbound.NewReplayTrialBudget())
	require.NoError(t, err)
	require.False(t, ok)
	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	ok, err = replayValidateBlindGroup(ctx, out, target, section, 6, true, proofbound.NewReplayTrialBudget())
	require.ErrorIs(t, err, context.Canceled)
	require.False(t, ok)
}
