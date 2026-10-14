package decad

import (
	"context"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/stretchr/testify/require"
)

// gateMemoBodies returns a disc, a box and the disc placed twice.
func gateMemoBodies(t *testing.T) (disc, box, placed1, placed2 *Body) {
	t.Helper()
	doc := New()
	disc = internalDiscBody(t, doc, 3, 1)
	box = internalBoxBody(t, doc, 8, 0, 12, 2, 1)
	var err error
	placed1, err = Copy(t.Context(), disc, WithCopyPlacement(gateFarPlacement(t, 1)))
	require.NoError(t, err)
	placed2, err = Copy(t.Context(), disc, WithCopyPlacement(gateFarPlacement(t, 2)))
	require.NoError(t, err)
	return disc, box, placed1, placed2
}

// TestGatePointsReadOncePerBody checks that a call's bodyGeomCache gives each
// pair the diameter a cache-free pair reads, bit for bit, keeps each body's
// gate points in the body's entry, shares one station list between the
// placed copies of a profile, and drops a body's reading with its entry.
//
// Shown to fail: storing a reading under the other body's entry (the box's
// reading kept in the disc's entry) changes the diameter here; a package-level
// station memo makes the fresh cache's Len nonzero; and not keeping the
// reading in the entry leaves the entry's gate nil.
func TestGatePointsReadOncePerBody(t *testing.T) {
	t.Parallel()
	disc, box, placed1, placed2 := gateMemoBodies(t)
	cache := &bodyGeomCache{}
	for _, a := range []*Body{disc, placed1, placed2} {
		want, err := clearancePair(t.Context(), a, box, false)
		require.NoError(t, err)
		got, err := clearancePairCached(t.Context(), a, box, false, cache)
		require.NoError(t, err)
		require.Equal(t, math.Float64bits(want.diam), math.Float64bits(got.diam))
		require.Positive(t, got.diam)
	}
	require.NotNil(t, cache.entries[box].gate, "the box's reading is stored in its entry")
	require.NotNil(t, cache.entries[disc].gate)
	require.Equal(t, 2, cache.stations.Len(), "one station list per profile, shared by the placed discs")

	// A later call starts with nothing.
	fresh := &bodyGeomCache{}
	require.Zero(t, fresh.stations.Len())

	// Evicting the entry drops the reading; reading again rebuilds it with the same answer.
	old := cache.entries[box].gate
	delete(cache.entries, box)
	want, err := clearancePair(t.Context(), disc, box, false)
	require.NoError(t, err)
	got, err := clearancePairCached(t.Context(), disc, box, false, cache)
	require.NoError(t, err)
	require.Equal(t, math.Float64bits(want.diam), math.Float64bits(got.diam))
	require.NotSame(t, old, cache.entries[box].gate)
}

// TestGatePointsStoreOnlyInAnEntry checks that a body with no entry is read
// without storing, that a cancelled reading is never stored, and that a
// stored reading is returned unchanged by a later read.
//
// Shown to fail: storing a reading for a body with no entry adds an entry;
// storing a cancelled reading leaves gate set after the cancelled read.
func TestGatePointsStoreOnlyInAnEntry(t *testing.T) {
	t.Parallel()
	_, box, placed, _ := gateMemoBodies(t)
	require.NotNil(t, placed)
	cache := &bodyGeomCache{}
	budget := proofbound.NewWorkBudget(t.Context())
	first, ok, err := cache.gatePoints(t.Context(), budget, box)
	require.NoError(t, err)
	require.True(t, ok)
	require.Empty(t, cache.entries, "a body with no entry is read without storing")

	// A cold cache, so the station memo cannot answer without stepping.
	cold := &bodyGeomCache{}
	_, _, err = cold.get(budget, box)
	require.NoError(t, err)
	cancelled := &proofbound.WorkBudget{
		StepFn: func() error { return context.Canceled },
		ErrFn:  func() error { return context.Canceled },
	}
	_, _, err = cold.gatePoints(t.Context(), cancelled, box)
	require.ErrorIs(t, err, context.Canceled)
	require.Nil(t, cold.entries[box].gate, "a cancelled reading is never stored")
	require.Zero(t, cold.stations.Len(), "a cancelled station reading is never stored")

	_, _, err = cache.get(budget, box)
	require.NoError(t, err)

	stored, _, err := cache.gatePoints(t.Context(), budget, box)
	require.NoError(t, err)
	again, _, err := cache.gatePoints(t.Context(), budget, box)
	require.NoError(t, err)
	require.Equal(t, first, stored)
	require.Same(t, &stored.Points[0], &again.Points[0], "a hit returns the stored slice")

	var nilCache *bodyGeomCache
	viaNil, ok, err := nilCache.gatePoints(t.Context(), budget, box)
	require.NoError(t, err)
	require.True(t, ok)
	require.Equal(t, first, viaNil)
}
