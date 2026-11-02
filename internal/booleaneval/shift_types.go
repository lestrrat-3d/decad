package booleaneval

import (
	"github.com/lestrrat-3d/decad/internal/boolshift"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

type pieces = boolshift.Pieces
type pieceShift = boolshift.PieceShift
type uniformShrink = boolshift.UniformShrink
type ReplayShifts = boolshift.ReplayShifts

func shiftBoundTo(s *ReplayShifts, b *proofbound.ReplayBudget) bool { _, ok := s.Single(b); return ok }

const spanningPieces = boolshift.SpanningPieces
