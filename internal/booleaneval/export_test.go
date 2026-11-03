package booleaneval

import (
	"context"

	"github.com/lestrrat-3d/r3"

	"github.com/lestrrat-3d/decad/internal/boolshift"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/topo"
)

// ShiftDirections exposes shiftDirections to the external test package.
var ShiftDirections = shiftDirections

// ShiftNeed exposes shiftNeed to the external test package.
var ShiftNeed = shiftNeed

// PieceOfFacet returns the connected piece of each facet of m.
func PieceOfFacet(m *topo.Mesh) []int {
	pc := boolshift.PiecesOf(m)
	out := make([]int, pc.FacetCount())
	for i := range out {
		out[i] = pc.Facet(i)
	}
	return out
}

// ShiftPieces moves each connected piece of m by its shift and returns the
// decision and charged copies.
func ShiftPieces(m *topo.Mesh, shifts map[int]r3.Vec) (*topo.Mesh, *topo.Mesh, error) {
	moves := make(map[int]pieceShift, len(shifts))
	for c, sh := range shifts {
		moves[c] = pieceShift{Move: sh}
	}
	decision, charged, _, err := boolshift.ShiftOperandMesh(context.Background(), m, boolshift.PiecesOf(m), moves)
	return decision, charged, err
}

// GrowPieces grows each connected piece of m about its centre so that its farthest vertex moves by its grow, and
// returns the decision and charged copies and the displacement bound.
func GrowPieces(m *topo.Mesh, grows map[int]float64) (*topo.Mesh, *topo.Mesh, float64, error) {
	moves := make(map[int]pieceShift, len(grows))
	for c, g := range grows {
		moves[c] = pieceShift{Grow: g}
	}
	return boolshift.ShiftOperandMesh(context.Background(), m, boolshift.PiecesOf(m), moves)
}

// MoveRounding exposes boolshift.MoveRounding to the external test package.
var MoveRounding = boolshift.MoveRounding

// PieceShift exposes pieceShift to the external test package.
type PieceShift = pieceShift

// SnapShift exposes snapShift to the external test package.
var SnapShift = snapShift

// MovedGroup exposes boolshift.MovedGroup to the external test package.
var MovedGroup = boolshift.MovedGroup

// SpanningPieces exposes spanningPieces to the external test package.
const SpanningPieces = spanningPieces

// FacePieces returns the connected piece of each source face of m, or
// SpanningPieces for a face whose facets lie in several.
func FacePieces(m *topo.Mesh) map[*topo.Face]int {
	return boolshift.FacePieces(m, boolshift.PiecesOf(m))
}

// GrowFolded runs growFolded over count pieces with attempt, which sees each
// piece's grow.
func GrowFolded(ctx context.Context, refusal error, op meshbool.OperationKind, count int,
	attempt func(map[int]float64) (Evaluation, error)) (Evaluation, error) {
	return growFolded(ctx, refusal, op, boolshift.NewPieces(nil, nil, count), func(shifts map[int]pieceShift) (Evaluation, error) {
		grows := make(map[int]float64, len(shifts))
		for c, sh := range shifts {
			grows[c] = sh.Grow
		}
		return attempt(grows)
	})
}
