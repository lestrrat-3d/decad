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
	moves := make(map[int]pieceShift, len(shifts))
	for c, sh := range shifts {
		moves[c] = pieceShift{Move: sh}
	}
	decision, charged, _, err := shiftOperandMesh(context.Background(), m, piecesOf(m), moves)
	return decision, charged, err
}

// GrowPieces grows each connected piece of m about its centre so that its farthest vertex moves by its grow, and
// returns the decision and charged copies and the displacement bound.
func GrowPieces(m *topo.Mesh, grows map[int]float64) (*topo.Mesh, *topo.Mesh, float64, error) {
	moves := make(map[int]pieceShift, len(grows))
	for c, g := range grows {
		moves[c] = pieceShift{Grow: g}
	}
	return shiftOperandMesh(context.Background(), m, piecesOf(m), moves)
}

// MoveRounding exposes moveRounding to the external test package.
var MoveRounding = moveRounding

// PieceShift exposes pieceShift to the external test package.
type PieceShift = pieceShift

// SnapShift exposes snapShift to the external test package.
var SnapShift = snapShift

// MovedGroup exposes movedGroup to the external test package.
var MovedGroup = movedGroup

// SpanningPieces exposes spanningPieces to the external test package.
const SpanningPieces = spanningPieces

// FacePieces returns the connected piece of each source face of m, or
// SpanningPieces for a face whose facets lie in several.
func FacePieces(m *topo.Mesh) map[*topo.Face]int { return facePieces(m, piecesOf(m)) }
