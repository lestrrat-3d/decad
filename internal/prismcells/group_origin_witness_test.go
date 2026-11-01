package prismcells

import (
	"context"
	"errors"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/sketch"
	"github.com/stretchr/testify/require"
)

func TestGroupWholeCircleOrigins(t *testing.T) {
	profiles, tags := curvedScene(t, sceneCircle(0, 1), sceneCircle(5, 1))
	scene := GroupScene{Profiles: profiles, Tags: tags}
	meter := proofbound.NewReplayTrialBudget()
	loops, origins, _, ok, err := GroupWholeCircleOrigins(meter.WorkBudget(t.Context()), scene)
	require.NoError(t, err)
	require.True(t, ok)
	require.Len(t, loops, 2)
	require.Len(t, origins, 2)
	seenA, seenB := false, false
	for _, row := range origins {
		require.Len(t, row, 1)
		require.Equal(t, -1, row[0].Hole)
		if row[0].IsB {
			seenB = true
		} else {
			seenA = true
		}
	}
	require.True(t, seenA)
	require.True(t, seenB)
	work, records := meter.Usage()
	t.Logf("work=%d records=%d bytes=%d", work, records, meter.Bytes())

	for _, tc := range []struct {
		name    string
		w, r, b uint64
		pass    bool
	}{
		{"exact", work, records, meter.Bytes(), true}, {"work short", work - 1, records, meter.Bytes(), false},
		{"records short", work, records - 1, meter.Bytes(), false}, {"bytes short", work, records, meter.Bytes() - 1, false},
	} {
		t.Run(tc.name, func(t *testing.T) {
			m := &originTestMeter{maxWork: tc.w, maxRecords: tc.r, maxBytes: tc.b}
			loops, origins, _, ok, err := GroupWholeCircleOrigins(m.workBudget(t.Context()), scene)
			if tc.pass {
				require.NoError(t, err)
				require.True(t, ok)
				return
			}
			require.ErrorIs(t, err, errOriginLimit)
			require.False(t, ok)
			require.Nil(t, loops)
			require.Nil(t, origins)
		})
	}
	duplicate := scene
	duplicate.Profiles = append([]*sketch.Profile(nil), scene.Profiles...)
	duplicate.Profiles[1] = duplicate.Profiles[0]
	_, _, _, duplicateOK, duplicateErr := GroupWholeCircleOrigins(proofbound.NewReplayTrialBudget().WorkBudget(t.Context()), duplicate) //nolint:dogsled // This test checks only the refusal and its error.
	require.NoError(t, duplicateErr)
	require.False(t, duplicateOK)
	scene.Profiles[0], scene.Profiles[1] = scene.Profiles[1], scene.Profiles[0]
	_, reordered, _, ok, err := GroupWholeCircleOrigins(proofbound.NewReplayTrialBudget().WorkBudget(t.Context()), scene)
	require.NoError(t, err)
	require.True(t, ok)
	require.Len(t, reordered, 2)
	require.NotEqual(t, origins[0][0].IsB, reordered[0][0].IsB)

	delete(scene.Tags, scene.Profiles[0].Outer[0].Entity)
	_, _, _, ok, err = GroupWholeCircleOrigins(proofbound.NewReplayTrialBudget().WorkBudget(t.Context()), scene) //nolint:dogsled // This test checks only the refusal and its error.
	require.NoError(t, err)
	require.False(t, ok)
}

type originTestMeter struct{ work, records, bytes, maxWork, maxRecords, maxBytes uint64 }

var errOriginLimit = errors.New("origin replay limit")

func (m *originTestMeter) workBudget(ctx context.Context) *proofbound.WorkBudget {
	return &proofbound.WorkBudget{
		ErrFn: ctx.Err,
		StepFn: func() error {
			if err := ctx.Err(); err != nil {
				return err
			}
			if m.work >= m.maxWork {
				return errOriginLimit
			}
			m.work++
			return nil
		},
		ReserveFn: func(n uint64) error {
			if err := ctx.Err(); err != nil {
				return err
			}
			if n > m.maxRecords-m.records {
				return errOriginLimit
			}
			m.records += n
			return nil
		},
		ReserveBytesFn: func(n uint64) error {
			if err := ctx.Err(); err != nil {
				return err
			}
			if n > m.maxBytes-m.bytes {
				return errOriginLimit
			}
			m.bytes += n
			return nil
		},
	}
}
