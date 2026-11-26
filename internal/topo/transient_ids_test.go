package topo_test

// CI: isolated-tests

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/denotation"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// TestWithTransientIDsSeedsAboveCounters checks that the first transient id
// is the document's counter plus one and that minting never writes the
// document's counters.
func TestWithTransientIDsSeedsAboveCounters(t *testing.T) {
	t.Parallel()
	d := &topo.Document{NextLevel: 7, NextCurve: 11}
	ctx := topo.WithTransientIDs(t.Context(), d)

	origin, normal := r3.Vec{}, r3.Vec{Z: 1}
	require.Equal(t, denotation.LevelID(8), d.MintLevel(ctx, origin, normal).ID)
	require.Equal(t, denotation.LevelID(9), d.MintLevel(ctx, origin, normal).ID)
	require.Equal(t, denotation.CurveID(12), d.MintCurve(ctx).ID)
	require.Equal(t, denotation.CurveID(13), d.MintCurve(ctx).ID)
	require.Equal(t, denotation.LevelID(7), d.NextLevel, "level counter")
	require.Equal(t, denotation.CurveID(11), d.NextCurve, "curve counter")
}

// TestWithTransientIDsNestedSharesSource checks that a nested call keeps the
// caller's source, so ids continue instead of restarting.
func TestWithTransientIDsNestedSharesSource(t *testing.T) {
	t.Parallel()
	d := &topo.Document{NextLevel: 3}
	outer := topo.WithTransientIDs(t.Context(), d)
	require.Equal(t, denotation.LevelID(4), d.MintLevel(outer, r3.Vec{}, r3.Vec{Z: 1}).ID)

	inner := topo.WithTransientIDs(outer, d)
	require.Equal(t, outer, inner)
	require.Equal(t, denotation.LevelID(5), d.MintLevel(inner, r3.Vec{}, r3.Vec{Z: 1}).ID)
	require.Equal(t, denotation.LevelID(6), d.MintLevel(outer, r3.Vec{}, r3.Vec{Z: 1}).ID)
}

// TestWithTransientIDsOtherDocumentUsesOwnCounter checks that a source made
// for one document does not serve another: the other document mints from its
// own counters.
func TestWithTransientIDsOtherDocumentUsesOwnCounter(t *testing.T) {
	t.Parallel()
	a, b := &topo.Document{NextLevel: 3}, &topo.Document{NextLevel: 40}
	ctx := topo.WithTransientIDs(t.Context(), a)

	require.Equal(t, denotation.LevelID(41), b.MintLevel(ctx, r3.Vec{}, r3.Vec{Z: 1}).ID)
	require.Equal(t, denotation.LevelID(41), b.NextLevel, "other document's own counter advances")
	require.Equal(t, denotation.LevelID(3), a.NextLevel)
	require.Equal(t, denotation.LevelID(4), a.MintLevel(ctx, r3.Vec{}, r3.Vec{Z: 1}).ID)
}
