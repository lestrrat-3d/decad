package booleaneval_test

import (
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"

	"github.com/lestrrat-3d/decad/internal/booleaneval"
	"github.com/lestrrat-3d/decad/internal/meshbool"
)

func contact(n r3.Vec, opposite bool) meshbool.CoplanarContact {
	return meshbool.CoplanarContact{NormalA: n, Opposite: opposite}
}

// requireOpens asserts every direction opens every contact: no component of the
// unit shift along a contact's normal is closer to zero than the margin the
// evaluator itself demands.
func requireOpens(t *testing.T, dirs []r3.Vec, contacts []meshbool.CoplanarContact) {
	t.Helper()
	require.NotEmpty(t, dirs)
	for _, u := range dirs {
		require.InDelta(t, 1, u.Len(), 1e-12)
		for _, c := range contacts {
			require.Greater(t, abs(u.Dot(c.NormalA)), 0.02)
		}
	}
}

func abs(x float64) float64 {
	if x < 0 {
		return -x
	}
	return x
}

func TestShiftDirections(t *testing.T) {
	t.Parallel()
	up, down := r3.Vec{Z: 1}, r3.Vec{Z: -1}
	north, south := r3.Vec{Y: 1}, r3.Vec{Y: -1}

	t.Run("a Union closes a face-on-face touch into the first operand", func(t *testing.T) {
		t.Parallel()
		contacts := []meshbool.CoplanarContact{contact(up, true)}
		dirs := booleaneval.ShiftDirections(meshbool.OpUnion, contacts)
		requireOpens(t, dirs, contacts)
		for _, u := range dirs {
			require.Less(t, u.Dot(up), -0.05, "the second operand moves down, into the first operand's top")
		}
	})

	t.Run("a Cut and an Intersect take a face-on-face touch apart", func(t *testing.T) {
		t.Parallel()
		contacts := []meshbool.CoplanarContact{contact(up, true)}
		for _, op := range []meshbool.OperationKind{meshbool.OpCut, meshbool.OpIntersect} {
			dirs := booleaneval.ShiftDirections(op, contacts)
			requireOpens(t, dirs, contacts)
			for _, u := range dirs {
				require.Greater(t, u.Dot(up), 0.05)
			}
		}
	})

	t.Run("a Union opens flush walls on opposite sides whichever way", func(t *testing.T) {
		t.Parallel()
		contacts := []meshbool.CoplanarContact{contact(north, false), contact(south, false)}
		requireOpens(t, booleaneval.ShiftDirections(meshbool.OpUnion, contacts), contacts)
	})

	t.Run("a Cut declines flush walls on opposite sides", func(t *testing.T) {
		t.Parallel()
		// A tool flush with both walls of a slot would leave a skin on one wall
		// if it moved, and no shift pokes through both.
		contacts := []meshbool.CoplanarContact{contact(north, false), contact(south, false)}
		require.Empty(t, booleaneval.ShiftDirections(meshbool.OpCut, contacts))
		require.Empty(t, booleaneval.ShiftDirections(meshbool.OpIntersect, contacts))
	})

	t.Run("a Cut pokes a flush tool through the wall", func(t *testing.T) {
		t.Parallel()
		contacts := []meshbool.CoplanarContact{contact(up, false)}
		dirs := booleaneval.ShiftDirections(meshbool.OpCut, contacts)
		requireOpens(t, dirs, contacts)
		for _, u := range dirs {
			require.Greater(t, u.Dot(up), 0.05)
		}
	})

	t.Run("a face-on-face touch decides the direction beside flush walls", func(t *testing.T) {
		t.Parallel()
		contacts := []meshbool.CoplanarContact{contact(down, true), contact(north, false), contact(south, false)}
		dirs := booleaneval.ShiftDirections(meshbool.OpCut, contacts)
		requireOpens(t, dirs, contacts)
		for _, u := range dirs {
			require.Greater(t, u.Dot(down), 0.05)
		}
	})

	t.Run("conflicting face-on-face touches decline", func(t *testing.T) {
		t.Parallel()
		contacts := []meshbool.CoplanarContact{contact(up, true), contact(down, true)}
		require.Empty(t, booleaneval.ShiftDirections(meshbool.OpUnion, contacts))
		require.Empty(t, booleaneval.ShiftDirections(meshbool.OpCut, contacts))
	})
}
