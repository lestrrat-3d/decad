package booloperand_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/booloperand"
	"github.com/lestrrat-3d/decad/internal/coilshell"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/stackedrecord"
	"github.com/lestrrat-3d/decad/internal/sweepmitre"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/twistsweep"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func boxBody(lo, hi r3.Vec, bound float64, payload topo.Payload) *topo.Body {
	return &topo.Body{
		Bounds:  measurement.Box{Min: lo, Max: hi, Bound: units.Millimeters(bound)},
		Payload: payload,
	}
}

func TestHeldFloor(t *testing.T) {
	cases := []struct {
		name      string
		payload   topo.Payload
		floor     float64
		restating bool
	}{
		{"faceted result", facetedrecord.Payload{MeshBound: 0.25}, 0.25, true},
		{"mitred sweep", sweepmitre.Payload{Delta: 0.125}, 0.125, true},
		{"coil", coilshell.Payload{Delta: 0.5}, 0.5, true},
		{"twisted sweep", twistsweep.Payload{Delta: 0.75}, 0.75, true},
		{"prism", prismrecord.Payload{SectionDelta: 0.5}, 0, false},
		{"no payload", nil, 0, false},
	}
	for _, c := range cases {
		t.Run(c.name, func(t *testing.T) {
			floor, restating := booloperand.HeldFloor(&topo.Body{Payload: c.payload})
			require.Equal(t, c.restating, restating)
			require.Equal(t, c.floor, floor)
		})
	}
}

func TestChordShare(t *testing.T) {
	lo, hi := r3.NewVec(1, 2, 3), r3.NewVec(4, 6, 8)
	t.Run("box inflated by its own bound", func(t *testing.T) {
		got := booloperand.ChordShare(t.Context(), boxBody(lo, hi, 0.5, nil))
		require.Equal(t, r3.NewVec(0.5, 1.5, 2.5), got.Lo)
		require.Equal(t, r3.NewVec(4.5, 6.5, 8.5), got.Hi)
		require.Zero(t, got.Floor)
	})
	t.Run("section displacement raises the floor past twice itself", func(t *testing.T) {
		for _, payload := range []topo.Payload{
			prismrecord.Payload{SectionDelta: 0.01},
			stackedrecord.Payload{SectionDelta: 0.01},
		} {
			got := booloperand.ChordShare(t.Context(), boxBody(lo, hi, 0, payload))
			require.GreaterOrEqual(t, got.Floor, 0.02, "%T", payload)
			require.Less(t, got.Floor, 0.0201, "%T", payload)
		}
	})
	t.Run("held primitive floor", func(t *testing.T) {
		for _, c := range []struct {
			payload topo.Payload
			floor   float64
		}{
			{facetedrecord.Payload{LoftFacesPrimitive: true, MeshBound: 0.25}, 0.25},
			{coilshell.Payload{Delta: 0.125}, 0.125},
			{sweepmitre.Payload{Delta: 0.375}, 0.375},
			{twistsweep.Payload{Delta: 0.0625}, 0.0625},
			// A boolean result's bound grows with the chain; it never raises
			// the pair tolerance.
			{facetedrecord.Payload{MeshBound: 0.25}, 0},
			// A result keeps the floor of the fixed held mesh it inherited, not its bound.
			{facetedrecord.Payload{MeshBound: 0.25, HeldFloor: 0.125}, 0.125},
		} {
			got := booloperand.ChordShare(t.Context(), boxBody(lo, hi, 0, c.payload))
			require.Equal(t, c.floor, got.Floor, "%#v", c.payload)
		}
	})
}

func TestPairChord(t *testing.T) {
	origin := r3.NewVec(0, 0, 0)
	t.Run("diameter of the joint box", func(t *testing.T) {
		a := boxBody(origin, r3.NewVec(3, 4, 0), 0, nil)
		b := boxBody(origin, r3.NewVec(0, 0, 12), 0, nil)
		tol, diameter, err := booloperand.PairChord(t.Context(), a, b)
		require.NoError(t, err)
		require.Equal(t, 13.0, diameter)
		require.Equal(t, diameter*meshbool.ChordFactor, tol)
	})
	t.Run("held primitive raises the pair tolerance", func(t *testing.T) {
		a := boxBody(origin, r3.NewVec(3, 4, 0), 0, nil)
		b := boxBody(origin, r3.NewVec(0, 0, 12), 0, facetedrecord.Payload{LoftFacesPrimitive: true, MeshBound: 0.5})
		tol, diameter, err := booloperand.PairChord(t.Context(), a, b)
		require.NoError(t, err)
		require.Equal(t, 13.0, diameter)
		require.Equal(t, 0.5, tol)
	})
	t.Run("no extent is degenerate", func(t *testing.T) {
		a := boxBody(origin, origin, 0, nil)
		_, _, err := booloperand.PairChord(t.Context(), a, a)
		require.ErrorIs(t, err, decaderr.ErrDegenerate)
	})
}
