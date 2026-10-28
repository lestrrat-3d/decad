package booleaneval_test

import (
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"

	"github.com/lestrrat-3d/decad/internal/booleaneval"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/topo"
)

func contact(n r3.Vec, opposite bool) meshbool.TouchContact {
	return meshbool.TouchContact{NormalA: n, Decisive: opposite}
}

// requireOpens asserts every direction opens every contact: no component of the
// unit shift along a contact's normal is closer to zero than the margin the
// evaluator itself demands.
func requireOpens(t *testing.T, dirs []r3.Vec, contacts []meshbool.TouchContact) {
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
		contacts := []meshbool.TouchContact{contact(up, true)}
		dirs := booleaneval.ShiftDirections(meshbool.OpUnion, contacts)
		requireOpens(t, dirs, contacts)
		for _, u := range dirs {
			require.Less(t, u.Dot(up), -0.05, "the second operand moves down, into the first operand's top")
		}
	})

	t.Run("a Cut and an Intersect take a face-on-face touch apart", func(t *testing.T) {
		t.Parallel()
		contacts := []meshbool.TouchContact{contact(up, true)}
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
		contacts := []meshbool.TouchContact{contact(north, false), contact(south, false)}
		requireOpens(t, booleaneval.ShiftDirections(meshbool.OpUnion, contacts), contacts)
	})

	t.Run("a Cut declines flush walls on opposite sides", func(t *testing.T) {
		t.Parallel()
		// A tool flush with both walls of a slot would leave a skin on one wall
		// if it moved, and no shift pokes through both.
		contacts := []meshbool.TouchContact{contact(north, false), contact(south, false)}
		require.Empty(t, booleaneval.ShiftDirections(meshbool.OpCut, contacts))
		require.Empty(t, booleaneval.ShiftDirections(meshbool.OpIntersect, contacts))
	})

	t.Run("a contact with no normal names no direction and does not close the search", func(t *testing.T) {
		t.Parallel()
		// A sliver facet too thin to normalise gives a contact whose normal is the zero vector; the contacts beside it
		// still choose the directions, and the shift reaches its slack along its own length.
		blind := meshbool.TouchContact{Decisive: true, Slack: 2e-6}
		contacts := []meshbool.TouchContact{contact(up, true), blind}
		dirs := booleaneval.ShiftDirections(meshbool.OpUnion, contacts)
		require.NotEmpty(t, dirs)
		for _, u := range dirs {
			require.Less(t, u.Dot(up), -0.05)
		}
		require.GreaterOrEqual(t, booleaneval.ShiftNeed(dirs[0], []meshbool.TouchContact{blind}), 2e-6)
	})

	t.Run("a Cut pokes a flush tool through the wall", func(t *testing.T) {
		t.Parallel()
		contacts := []meshbool.TouchContact{contact(up, false)}
		dirs := booleaneval.ShiftDirections(meshbool.OpCut, contacts)
		requireOpens(t, dirs, contacts)
		for _, u := range dirs {
			require.Greater(t, u.Dot(up), 0.05)
		}
	})

	t.Run("a face-on-face touch decides the direction beside flush walls", func(t *testing.T) {
		t.Parallel()
		contacts := []meshbool.TouchContact{contact(down, true), contact(north, false), contact(south, false)}
		dirs := booleaneval.ShiftDirections(meshbool.OpCut, contacts)
		requireOpens(t, dirs, contacts)
		for _, u := range dirs {
			require.Greater(t, u.Dot(down), 0.05)
		}
	})

	t.Run("conflicting face-on-face touches decline", func(t *testing.T) {
		t.Parallel()
		contacts := []meshbool.TouchContact{contact(up, true), contact(down, true)}
		require.Empty(t, booleaneval.ShiftDirections(meshbool.OpUnion, contacts))
		require.Empty(t, booleaneval.ShiftDirections(meshbool.OpCut, contacts))
	})
}

func TestShiftNeed(t *testing.T) {
	t.Parallel()
	up := r3.Vec{Z: 1}
	contacts := []meshbool.TouchContact{{NormalA: up, Decisive: true, Slack: 0.001}}
	require.InDelta(t, 1.05*0.001, booleaneval.ShiftNeed(up, contacts), 1e-15)
	slanted := r3.Vec{X: 0.6, Z: 0.8}
	require.InDelta(t, 1.05*0.001/0.8, booleaneval.ShiftNeed(slanted, contacts), 1e-15)
	require.Zero(t, booleaneval.ShiftNeed(up, []meshbool.TouchContact{{NormalA: up}}))
}

func TestShiftPiecesChargeEachPieceOnItsOwn(t *testing.T) {
	t.Parallel()
	// Two separate tetrahedra, 100 mm apart.
	tet := func(dx float64) []r3.Vec {
		return []r3.Vec{{X: dx}, {X: dx + 1}, {X: dx, Y: 1}, {X: dx, Z: 1}}
	}
	verts := append(tet(0), tet(100)...)
	tris := [][3]int{{0, 2, 1}, {0, 1, 3}, {1, 2, 3}, {0, 3, 2}, {4, 6, 5}, {4, 5, 7}, {5, 6, 7}, {4, 7, 6}}
	m := &topo.Mesh{Vertices: verts, Triangles: tris, VertexBound: make([]float64, len(verts))}
	pieces := booleaneval.PieceOfFacet(m)
	require.Equal(t, []int{0, 0, 0, 0, 1, 1, 1, 1}, pieces)

	shift := r3.Vec{Z: -0.004}
	decision, charged, err := booleaneval.ShiftPieces(m, map[int]r3.Vec{1: shift})
	require.NoError(t, err)
	for i := range verts {
		moved := i >= 4
		if moved {
			require.InDelta(t, verts[i].Z+shift.Z, decision.Vertices[i].Z, 1e-15)
			require.GreaterOrEqual(t, charged.VertexBound[i], 0.004)
		} else {
			require.Equal(t, verts[i], decision.Vertices[i])
			require.Zero(t, charged.VertexBound[i], "a piece that does not move is not charged")
		}
	}
	require.Equal(t, verts[0], m.Vertices[0], "the input mesh is not modified")
}
