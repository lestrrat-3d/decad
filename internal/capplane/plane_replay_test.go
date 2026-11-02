package capplane_test

import (
	"context"
	"errors"
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/capplane"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

var errCapReplayLimit = errors.New("test cap replay limit")

type capReplayMeter struct{ work, records, bytes, maxWork, maxRecords, maxBytes uint64 }

func (m *capReplayMeter) SpendWork(ctx context.Context, n uint64) error {
	if err := ctx.Err(); err != nil {
		return err
	}
	if n > m.maxWork-m.work {
		return errCapReplayLimit
	}
	m.work += n
	return nil
}
func (m *capReplayMeter) ReserveRecords(ctx context.Context, n uint64) error {
	if err := ctx.Err(); err != nil {
		return err
	}
	if n > m.maxRecords-m.records {
		return errCapReplayLimit
	}
	m.records += n
	return nil
}
func (m *capReplayMeter) ReserveBytes(ctx context.Context, n uint64) error {
	if err := ctx.Err(); err != nil {
		return err
	}
	if n > m.maxBytes-m.bytes {
		return errCapReplayLimit
	}
	m.bytes += n
	return nil
}

func TestReplayOfFaceAndSame(t *testing.T) {
	f := frame(t, ex, ey)
	aX, err := r3.Translation(r3.Vec{Z: 525})
	require.NoError(t, err)
	bX, err := r3.FromBasis(r3.Basis{EX: r3.Vec{Y: -1}, EY: r3.Vec{X: 1}, EZ: r3.Vec{Z: 1}}, r3.Vec{X: -183, Y: -14.5, Z: 532})
	require.NoError(t, err)
	a := &topo.Face{CapRecord: &topo.CapRecord{Frame: f, Xform: aX, Z: 15, ZDenoted: big.NewRat(15, 1)}, HasAxialDelta: true}
	b := &topo.Face{CapRecord: &topo.CapRecord{Frame: f, Xform: bX, Z: 8, ZDenoted: big.NewRat(8, 1)}, HasAxialDelta: true}
	check := func(budget interface {
		SpendWork(context.Context, uint64) error
		ReserveRecords(context.Context, uint64) error
		ReserveBytes(context.Context, uint64) error
	}) (bool, error) {
		pa, ok, err := capplane.ReplayOfFace(t.Context(), a, budget)
		if err != nil || !ok {
			return false, err
		}
		pb, ok, err := capplane.ReplayOfFace(t.Context(), b, budget)
		if err != nil || !ok {
			return false, err
		}
		return capplane.ReplaySame(t.Context(), pa, pb, budget)
	}
	meter := proofbound.NewReplayTrialBudget()
	same, err := check(meter)
	require.NoError(t, err)
	require.True(t, same)
	ordinaryA, ok := capplane.OfFace(a)
	require.True(t, ok)
	ordinaryB, ok := capplane.OfFace(b)
	require.True(t, ok)
	require.True(t, ordinaryA.Same(ordinaryB))
	work, records := meter.Usage()
	bytes := meter.Bytes()
	t.Logf("work=%d records=%d bytes=%d", work, records, bytes)
	for _, tc := range []struct {
		name                 string
		work, records, bytes uint64
		pass                 bool
	}{{"exact", work, records, bytes, true}, {"work short", work - 1, records, bytes, false}, {"records short", work, records - 1, bytes, false}, {"bytes short", work, records, bytes - 1, false}} {
		t.Run(tc.name, func(t *testing.T) {
			m := &capReplayMeter{maxWork: tc.work, maxRecords: tc.records, maxBytes: tc.bytes}
			same, err := check(m)
			if tc.pass {
				require.NoError(t, err)
				require.True(t, same)
			} else {
				require.ErrorIs(t, err, errCapReplayLimit)
				require.False(t, same)
			}
		})
	}
	b.CapRecord.ZDenoted = nil
	b.CapRecord.Z = math.Nextafter(8, 9)
	meter = proofbound.NewReplayTrialBudget()
	same, err = check(meter)
	require.NoError(t, err)
	require.False(t, same)
	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	_, ok, err = capplane.ReplayOfFace(ctx, a, proofbound.NewReplayTrialBudget())
	require.ErrorIs(t, err, context.Canceled)
	require.False(t, ok)
}
