package prismplacement_test

import (
	"context"
	"errors"
	"math"
	"math/big"
	"testing"
	"unsafe"

	"github.com/lestrrat-3d/decad/internal/prismplacement"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

type signedMeter struct {
	work, records, bytes          uint64
	maxWork, maxRecords, maxBytes uint64
}

func TestReplayWorldIsometry(t *testing.T) {
	frame, err := r3.NewFrame(r3.Vec{}, r3.Vec{X: 1}, r3.Vec{Y: 1})
	require.NoError(t, err)
	permuted, err := r3.FromBasis(r3.Basis{EX: r3.Vec{Y: -1}, EY: r3.Vec{X: 1}, EZ: r3.Vec{Z: 1}}, r3.Vec{})
	require.NoError(t, err)
	operand := prismplacement.Operand{Frame: frame, Xform: permuted}
	full := proofbound.NewReplayTrialBudget()
	ok, err := prismplacement.ReplayWorldIsometry(t.Context(), operand, full)
	require.NoError(t, err)
	require.True(t, ok)
	work, records := full.Usage()
	bytes := full.Bytes()
	t.Logf("isometry work=%d records=%d bytes=%d", work, records, bytes)
	for _, tc := range []struct {
		name    string
		w, r, b uint64
		pass    bool
	}{
		{"exact", work, records, bytes, true}, {"work short", work - 1, records, bytes, false},
		{"records short", work, records - 1, bytes, false}, {"bytes short", work, records, bytes - 1, false},
	} {
		t.Run(tc.name, func(t *testing.T) {
			m, err := proofbound.NewReplayBudgetLimits(tc.w, tc.r, tc.b)
			require.NoError(t, err)
			ok, err := prismplacement.ReplayWorldIsometry(t.Context(), operand, m)
			if tc.pass {
				require.NoError(t, err)
				require.True(t, ok)
				return
			}
			require.Error(t, err)
			require.False(t, ok)
		})
	}
	// Corrupt private transform fields to exercise rejection of an authenticated
	// producer record whose common host/tool transform was changed in memory.
	type transformWords struct{ ex, ey, ez, translation r3.Vec }
	require.Equal(t, unsafe.Sizeof(r3.Transform{}), unsafe.Sizeof(transformWords{}))
	for _, tc := range []struct {
		name     string
		basis    transformWords
		relative bool
	}{
		{"scale", transformWords{ex: r3.Vec{X: 2}, ey: r3.Vec{Y: 2}, ez: r3.Vec{Z: 2}}, true},
		{"shear", transformWords{ex: r3.Vec{X: 1}, ey: r3.Vec{X: .125, Y: 1}, ez: r3.Vec{Z: 1}}, true},
		{"reflection", transformWords{ex: r3.Vec{X: -1}, ey: r3.Vec{Y: 1}, ez: r3.Vec{Z: 1}}, true},
		{"nonfinite", transformWords{ex: r3.Vec{X: 1}, ey: r3.Vec{Y: 1}, ez: r3.Vec{Z: math.Inf(1)}}, false},
	} {
		t.Run(tc.name, func(t *testing.T) {
			xf := *(*r3.Transform)(unsafe.Pointer(&tc.basis))
			bad := prismplacement.Operand{Frame: frame, Xform: xf}
			if tc.relative {
				_, matched, err := prismplacement.ReplayParallelSignedMap(t.Context(), bad, bad, proofbound.NewReplayTrialBudget())
				require.NoError(t, err)
				require.True(t, matched)
			}
			ok, err := prismplacement.ReplayWorldIsometry(t.Context(), bad, proofbound.NewReplayTrialBudget())
			if tc.relative {
				require.NoError(t, err)
			} else {
				require.Error(t, err)
			}
			require.False(t, ok)
		})
	}
	rotation, err := r3.Rotation(r3.Vec{Z: 1}, units.Degrees(45))
	require.NoError(t, err)
	ok, err = prismplacement.ReplayWorldIsometry(t.Context(), prismplacement.Operand{Frame: frame, Xform: rotation}, proofbound.NewReplayTrialBudget())
	require.NoError(t, err)
	require.False(t, ok)
	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	_, err = prismplacement.ReplayWorldIsometry(ctx, operand, proofbound.NewReplayTrialBudget())
	require.ErrorIs(t, err, context.Canceled)
}

var errSignedLimit = errors.New("signed map meter exhausted")

func (m *signedMeter) SpendWork(ctx context.Context, n uint64) error {
	if e := ctx.Err(); e != nil {
		return e
	}
	if n > m.maxWork-m.work {
		return errSignedLimit
	}
	m.work += n
	return nil
}
func (m *signedMeter) ReserveRecords(ctx context.Context, n uint64) error {
	if e := ctx.Err(); e != nil {
		return e
	}
	if n > m.maxRecords-m.records {
		return errSignedLimit
	}
	m.records += n
	return nil
}
func (m *signedMeter) ReserveBytes(ctx context.Context, n uint64) error {
	if e := ctx.Err(); e != nil {
		return e
	}
	if n > m.maxBytes-m.bytes {
		return errSignedLimit
	}
	m.bytes += n
	return nil
}

func TestReplayParallelSignedMap(t *testing.T) {
	frame, err := r3.NewFrame(r3.Vec{}, r3.Vec{X: 1}, r3.Vec{Y: 1})
	require.NoError(t, err)
	aTransform, err := r3.Translation(r3.Vec{Z: 525})
	require.NoError(t, err)
	bTransform, err := r3.FromBasis(r3.Basis{EX: r3.Vec{Y: -1}, EY: r3.Vec{X: 1}, EZ: r3.Vec{Z: 1}}, r3.Vec{X: -183, Y: -14.5, Z: 532})
	require.NoError(t, err)
	a := prismplacement.Operand{Frame: frame, Xform: aTransform}
	b := prismplacement.Operand{Frame: frame, Xform: bTransform}
	meter := proofbound.NewReplayTrialBudget()
	got, ok, err := prismplacement.ReplayParallelSignedMap(t.Context(), a, b, meter)
	require.NoError(t, err)
	require.True(t, ok)
	require.Equal(t, -2, got.U)
	require.Equal(t, 1, got.V)
	require.Zero(t, got.OffsetU.Cmp(big.NewRat(-183, 1)))
	require.Zero(t, got.OffsetV.Cmp(big.NewRat(-29, 2)))
	require.Zero(t, got.Shift.Cmp(big.NewRat(7, 1)))
	w, r := meter.Usage()
	bytes := meter.Bytes()
	t.Logf("work=%d records=%d bytes=%d", w, r, bytes)
	for _, tc := range []struct {
		name    string
		w, r, b uint64
		pass    bool
	}{{"exact", w, r, bytes, true}, {"work short", w - 1, r, bytes, false}, {"records short", w, r - 1, bytes, false}, {"bytes short", w, r, bytes - 1, false}} {
		t.Run(tc.name, func(t *testing.T) {
			m := &signedMeter{maxWork: tc.w, maxRecords: tc.r, maxBytes: tc.b}
			_, ok, e := prismplacement.ReplayParallelSignedMap(t.Context(), a, b, m)
			if tc.pass {
				require.NoError(t, e)
				require.True(t, ok)
			} else {
				require.ErrorIs(t, e, errSignedLimit)
				require.False(t, ok)
			}
		})
	}
	angled, e := r3.Rotation(r3.Vec{Z: 1}, units.Degrees(45))
	require.NoError(t, e)
	opposite, e := r3.FromBasis(r3.Basis{EX: r3.Vec{X: 1}, EY: r3.Vec{Y: -1}, EZ: r3.Vec{Z: -1}}, r3.Vec{})
	require.NoError(t, e)
	for _, transform := range []r3.Transform{angled, opposite} {
		bad := b
		bad.Xform = transform
		_, accepted, e := prismplacement.ReplayParallelSignedMap(t.Context(), a, bad, proofbound.NewReplayTrialBudget())
		require.NoError(t, e)
		require.False(t, accepted)
	}
	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	_, ok, err = prismplacement.ReplayParallelSignedMap(ctx, a, b, proofbound.NewReplayTrialBudget())
	require.ErrorIs(t, err, context.Canceled)
	require.False(t, ok)
}

func TestReplayBlindLevels(t *testing.T) {
	target := prismplacement.Operand{Z0: 0, Z1: 15}
	tool := prismplacement.Operand{Z0: 0, Z1: 8}
	shift := big.NewRat(7, 1)
	meter := proofbound.NewReplayTrialBudget()
	top, ok, err := prismplacement.ReplayBlindLevels(t.Context(), target, tool, shift, meter)
	require.NoError(t, err)
	require.True(t, ok)
	require.True(t, top)
	work, records := meter.Usage()
	bytes := meter.Bytes()
	for _, tc := range []struct {
		name    string
		w, r, b uint64
		pass    bool
	}{
		{"exact", work, records, bytes, true}, {"work short", work - 1, records, bytes, false},
		{"records short", work, records - 1, bytes, false}, {"bytes short", work, records, bytes - 1, false},
	} {
		t.Run(tc.name, func(t *testing.T) {
			m := &signedMeter{maxWork: tc.w, maxRecords: tc.r, maxBytes: tc.b}
			_, ok, err := prismplacement.ReplayBlindLevels(t.Context(), target, tool, shift, m)
			if tc.pass {
				require.NoError(t, err)
				require.True(t, ok)
				return
			}
			require.ErrorIs(t, err, errSignedLimit)
			require.False(t, ok)
		})
	}
	for _, invalid := range []prismplacement.Operand{
		{Z0: 0, Z1: 7},             // separated cap
		{Z0: -8, Z1: 8},            // overrun
		{Z0: -7, Z1: 8},            // both caps coincide
		{Z0: 0, Z1: 8, Z0Delta: 8}, // uncertain inner end
	} {
		_, ok, err := prismplacement.ReplayBlindLevels(t.Context(), target, invalid, shift, proofbound.NewReplayTrialBudget())
		require.NoError(t, err)
		require.False(t, ok)
	}
}
