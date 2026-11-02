package prismcells

import (
	"context"
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/stretchr/testify/require"
)

type arcChargeMeter struct{ work, records, bytes, maxWork, maxRecords, maxBytes uint64 }

func (m *arcChargeMeter) SpendWork(ctx context.Context, n uint64) error {
	if err := ctx.Err(); err != nil {
		return err
	}
	if n > m.maxWork-m.work {
		return errOriginLimit
	}
	m.work += n
	return nil
}
func (m *arcChargeMeter) ReserveRecords(ctx context.Context, n uint64) error {
	if err := ctx.Err(); err != nil {
		return err
	}
	if n > m.maxRecords-m.records {
		return errOriginLimit
	}
	m.records += n
	return nil
}
func (m *arcChargeMeter) ReserveBytes(ctx context.Context, n uint64) error {
	if err := ctx.Err(); err != nil {
		return err
	}
	if n > m.maxBytes-m.bytes {
		return errOriginLimit
	}
	m.bytes += n
	return nil
}

func TestReplayWholeArcCharge(t *testing.T) {
	arcs := []sectionrecord.ArcSeg{
		{Center: sectionrecord.Point2{U: -180, V: -25}, Start: sectionrecord.Point2{U: -176.7, V: -25}, End: sectionrecord.Point2{U: -183.3, V: -25}, TStart: 1, TEnd: 0},
		{Center: sectionrecord.Point2{U: 1e12, V: -1e12}, Start: sectionrecord.Point2{U: 1e12 + 0.25, V: -1e12 + 0.125}, End: sectionrecord.Point2{U: 1e12 - 0.25, V: -1e12 - 0.125}, TStart: 0, TEnd: 1},
		{Center: sectionrecord.Point2{}, Start: sectionrecord.Point2{U: 2}, End: sectionrecord.Point2{V: 2.1}, TStart: 0, TEnd: 1},
		{Center: sectionrecord.Point2{}, Start: sectionrecord.Point2{U: math.SmallestNonzeroFloat64}, End: sectionrecord.Point2{V: math.SmallestNonzeroFloat64}, TStart: 0, TEnd: 1},
	}
	for _, invalid := range []sectionrecord.ArcSeg{
		{Center: sectionrecord.Point2{}, Start: sectionrecord.Point2{U: 1}, End: sectionrecord.Point2{V: 1}, TStart: 0, TEnd: .5},
		{Center: sectionrecord.Point2{}, Start: sectionrecord.Point2{U: 1}, End: sectionrecord.Point2{V: 1}, TStart: 1, TEnd: 1},
		{Center: sectionrecord.Point2{}, Start: sectionrecord.Point2{U: math.NaN()}, End: sectionrecord.Point2{V: 1}, TStart: 0, TEnd: 1},
		{Center: sectionrecord.Point2{U: -math.MaxFloat64}, Start: sectionrecord.Point2{U: math.MaxFloat64}, End: sectionrecord.Point2{V: 1}, TStart: 0, TEnd: 1},
	} {
		_, ok, err := ReplayWholeArcCharge(t.Context(), invalid, proofbound.NewReplayTrialBudget())
		require.NoError(t, err)
		require.False(t, ok)
	}
	cancelled, cancel := context.WithCancel(t.Context())
	cancel()
	_, ok, err := ReplayWholeArcCharge(cancelled, arcs[0], proofbound.NewReplayTrialBudget())
	require.ErrorIs(t, err, context.Canceled)
	require.False(t, ok)
	for i, arc := range arcs {
		meter := proofbound.NewReplayTrialBudget()
		bound, ok, err := ReplayWholeArcCharge(t.Context(), arc, meter)
		require.NoError(t, err)
		require.True(t, ok, "arc %d", i)
		require.GreaterOrEqual(t, bound, 0.0)
		const precision = 4096
		toFloat := func(x float64) *big.Float { return new(big.Float).SetPrec(precision).SetFloat64(x) }
		norm := func(p sectionrecord.Point2) *big.Float {
			dx := new(big.Float).SetPrec(precision).Sub(toFloat(p.U), toFloat(arc.Center.U))
			dy := new(big.Float).SetPrec(precision).Sub(toFloat(p.V), toFloat(arc.Center.V))
			dx.Mul(dx, dx)
			dy.Mul(dy, dy)
			dx.Add(dx, dy)
			return dx.Sqrt(dx)
		}
		r0, r1 := norm(arc.Start), norm(arc.End)
		held := toFloat(math.Hypot(arc.Start.U-arc.Center.U, arc.Start.V-arc.Center.V))
		radiusGap := new(big.Float).SetPrec(precision).Sub(held, r0)
		radiusGap.Abs(radiusGap)
		endGap := new(big.Float).SetPrec(precision).Sub(r1, r0)
		endGap.Abs(endGap)
		want := new(big.Float).SetPrec(precision).Add(radiusGap, endGap)
		require.GreaterOrEqual(t, toFloat(bound).Cmp(want), 0, "arc %d", i)
		work, records := meter.Usage()
		bytes := meter.Bytes()
		for _, tc := range []struct {
			w, r, b uint64
			pass    bool
		}{{work, records, bytes, true}, {work - 1, records, bytes, false}, {work, records - 1, bytes, false}, {work, records, bytes - 1, false}} {
			m := &arcChargeMeter{maxWork: tc.w, maxRecords: tc.r, maxBytes: tc.b}
			_, accepted, e := ReplayWholeArcCharge(t.Context(), arc, m)
			if tc.pass {
				require.NoError(t, e)
				require.True(t, accepted)
			} else {
				require.ErrorIs(t, e, errOriginLimit)
				require.False(t, accepted)
			}
		}
	}
}
