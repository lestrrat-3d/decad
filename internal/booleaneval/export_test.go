package booleaneval

import (
	"context"

	"github.com/lestrrat-3d/r3"

	"github.com/lestrrat-3d/decad/internal/topo"
)

// ShiftDirections exposes shiftDirections to the external test package.
var ShiftDirections = shiftDirections

// ShiftNeed exposes shiftNeed to the external test package.
var ShiftNeed = shiftNeed

// PieceOfFacet returns the connected piece of each facet of m.
func PieceOfFacet(m *topo.Mesh) []int { return piecesOf(m).ofFacet }

// ShiftPieces moves each connected piece of m by its shift and returns the
// decision and charged copies.
func ShiftPieces(m *topo.Mesh, shifts map[int]r3.Vec) (*topo.Mesh, *topo.Mesh, error) {
	return shiftOperandMesh(context.Background(), m, piecesOf(m), shifts)
}
