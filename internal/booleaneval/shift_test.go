package booleaneval_test

import (
	"math"
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

	t.Run("a Union pulls a body resting on a flat face into it", func(t *testing.T) {
		t.Parallel()
		// The proximity gate's contact on a flat face of the first operand is not Decisive and carries Slack: the gate
		// cannot tell a body resting on the face from one flush beneath it. Moving the second operand into the first
		// joins the resting body and keeps the flush one inside, where moving it out would leave the resting body
		// floating above the face.
		contacts := []meshbool.TouchContact{{NormalA: up, Slack: 0.004638}}
		dirs := booleaneval.ShiftDirections(meshbool.OpUnion, contacts)
		requireOpens(t, dirs, contacts)
		for _, u := range dirs {
			require.Less(t, u.Dot(up), -0.02, "the second operand moves down, into the first operand's top")
		}
	})

	t.Run("a Union with no Decisive contact pulls inward along each contact the pull does not lean along", func(t *testing.T) {
		t.Parallel()
		east := r3.Vec{X: 1}
		contacts := []meshbool.TouchContact{{NormalA: up, Slack: 0.004638}, {NormalA: east, Slack: 0.001}}
		dirs := booleaneval.ShiftDirections(meshbool.OpUnion, contacts)
		requireOpens(t, dirs, contacts)
		for _, u := range dirs {
			require.Less(t, u.Dot(up), -0.02)
			require.Less(t, u.Dot(east), -0.02)
		}
	})

	t.Run("a Union with a Decisive contact opens the other contacts outward on a tie", func(t *testing.T) {
		t.Parallel()
		// A curved wall touching the first operand's side decides the Union, and the
		// flush top and bottom caps beside it only need to open. A cap orthogonal to
		// the Decisive pull takes the outward tie, and the opposite cap follows it.
		contacts := []meshbool.TouchContact{contact(south, true), contact(up, false), contact(down, false)}
		dirs := booleaneval.ShiftDirections(meshbool.OpUnion, contacts)
		requireOpens(t, dirs, contacts)
		for _, u := range dirs {
			require.Less(t, u.Dot(south), -0.05, "the Decisive contact closes inward")
			require.Greater(t, u.Dot(up), 0.02, "the tie between the caps opens along the first cap's normal")
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

func TestSnapShift(t *testing.T) {
	t.Parallel()
	// Each shift with the coordinates it must move exactly: any coordinate
	// below 2^(52-24) times the shift's length that is a multiple of the grid.
	cases := []struct {
		v       r3.Vec
		exactAt []float64
	}{
		{r3.Vec{X: 0.0010544506367295983, Y: 0.0006516862194985151, Z: -0.004869488999247551}, []float64{0, 50, -30, 17.75, 1024}},
		{r3.Vec{X: 0.2098498229406450, Y: 0.1296943230999956, Z: -0.9690936148625756}, []float64{0, 50, -30, 17.75, 1024}},
		{r3.Vec{X: 0.0001, Y: 0.0001, Z: 0.0001}, []float64{0, 50, -30, 17.75}},
		{r3.Vec{X: -3.3e-7, Y: 1.7e-9}, []float64{0, 1, 17.75}},
	}
	for _, tc := range cases {
		v, got := tc.v, booleaneval.SnapShift(tc.v)
		for _, pair := range [][2]float64{{v.X, got.X}, {v.Y, got.Y}, {v.Z, got.Z}} {
			require.LessOrEqual(t, math.Abs(pair[1]), math.Abs(pair[0]), "%v: a snapped component never grows", v)
			require.False(t, pair[0]*pair[1] < 0, "%v: a snapped component keeps its sign", v)
		}
		require.LessOrEqual(t, got.Sub(v).Len(), math.Ldexp(v.Len(), -23), "%v moves by under 2^-23 of its length", v)
		for _, o := range tc.exactAt {
			requireExactSum(t, o, got.X, o+got.X)
			requireExactSum(t, o, got.Y, o+got.Y)
			requireExactSum(t, o, got.Z, o+got.Z)
		}
	}
	require.Equal(t, r3.Vec{}, booleaneval.SnapShift(r3.Vec{}))
}
