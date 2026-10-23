package revolveloftfaces

import (
	"math/big"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/loftface"
	"github.com/lestrrat-3d/decad/internal/polynomial"
)

// VerticalTraceSourcePiece holds only authenticated exact source geometry:
// one retained piece of a vertical source cell and the constant-X side that
// owns it. The root trace keeps graph identity separately.
type VerticalTraceSourcePiece struct {
	Piece    loftface.VerticalPiece
	Y, Z     []*big.Rat
	Arc      *arcSeg
	Affine   *loftface.VerticalAffine
	Radius2  *big.Rat
	ArcBox   [2]loftFaceInterval
	From, To *big.Rat
	// Band and Span are the cell's LoftFaces band and Bezier source span
	// (nil for a whole live Sketch arc); UpperRoot is its upper radial root.
	Band      int
	Span      freeform.BezierSpan
	UpperRoot loftface.Interval
	// X, Time and UpperRadius are the side's exact X, band time and cap radius.
	X, Time, UpperRadius *big.Rat
}

// VerticalTraceSourcePoint evaluates a line parameter or mapped arc endpoint.
func VerticalTraceSourcePoint(piece VerticalTraceSourcePiece,
	s *big.Rat) [2]*big.Rat {
	if piece.Arc != nil {
		if s.Cmp(lfZero()) == 0 {
			return VerticalTraceMapPoint(piece.Affine, piece.Arc.Start)
		}
		if s.Cmp(lfOne()) == 0 {
			return VerticalTraceMapPoint(piece.Affine, piece.Arc.End)
		}
		return [2]*big.Rat{}
	}
	return [2]*big.Rat{polynomial.BernsteinValue(piece.Y, s),
		polynomial.BernsteinValue(piece.Z, s)}
}
