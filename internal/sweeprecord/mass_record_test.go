package sweeprecord_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/revolveplan"
	"github.com/lestrrat-3d/decad/internal/sweeprecord"
	"github.com/stretchr/testify/require"
)

func TestSweepMassRecord(t *testing.T) {
	t.Parallel()

	prism := prismrecord.Payload{Z0: 1, Z1: 5}
	revolve := revolveplan.Payload{Phi0: 0.5, Phi1: 2}
	otherPrism := prismrecord.Payload{Z0: 2, Z1: 6}
	otherRevolve := revolveplan.Payload{Phi0: 0.75, Phi1: 3}

	t.Run("a one-span straight sweep carries its prism", func(t *testing.T) {
		t.Parallel()
		got := sweeprecord.Payload{Prism: prism, Revolve: revolve}.MassRecord()
		require.False(t, got.Arc)
		require.Equal(t, prism.MassRecord(), got.Prism)
		require.Zero(t, got.Revolve)
		require.Empty(t, got.Spans)
	})
	t.Run("a one-span arc sweep carries its revolve", func(t *testing.T) {
		t.Parallel()
		got := sweeprecord.Payload{Prism: prism, Revolve: revolve, Arc: true}.MassRecord()
		require.True(t, got.Arc)
		require.Equal(t, revolve.MassRecord(), got.Revolve)
		require.Zero(t, got.Prism)
		require.Empty(t, got.Spans)
	})
	t.Run("a composite sweep carries each span", func(t *testing.T) {
		t.Parallel()
		got := sweeprecord.Payload{Prism: otherPrism, Revolve: otherRevolve, Arc: true, Spans: []sweeprecord.SpanPayload{
			{Prism: prism, Revolve: otherRevolve},
			{Prism: otherPrism, Revolve: revolve, Arc: true},
		}}.MassRecord()
		require.True(t, got.Arc)
		require.Zero(t, got.Prism, "the whole-sweep reductions are not read once spans exist")
		require.Zero(t, got.Revolve)
		require.Len(t, got.Spans, 2)
		require.False(t, got.Spans[0].Arc)
		require.Equal(t, prism.MassRecord(), got.Spans[0].Prism)
		require.Zero(t, got.Spans[0].Revolve)
		require.True(t, got.Spans[1].Arc)
		require.Equal(t, revolve.MassRecord(), got.Spans[1].Revolve)
		require.Zero(t, got.Spans[1].Prism)
	})
}
