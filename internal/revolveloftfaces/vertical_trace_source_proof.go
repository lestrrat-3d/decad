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

// VerticalTracePieceSigns proves that each source piece has a strict angular
// direction. Adjacent pieces may reverse direction.
func VerticalTracePieceSigns(pieces []VerticalTraceSourcePiece) ([]int, bool) {
	signs := make([]int, len(pieces))
	for index, piece := range pieces {
		if piece.Arc != nil {
			// A positive similarity about the blank axis preserves the live
			// arc's counter-clockwise sweep. The certified source box keeps
			// the entire short arc in the positive-Y half-plane.
			if piece.ArcBox[0].Lo == nil || piece.ArcBox[0].Hi == nil ||
				piece.ArcBox[1].Lo == nil || piece.ArcBox[1].Hi == nil ||
				piece.Piece.Source.Lo.Cmp(lfZero()) != 0 ||
				piece.Piece.Source.Hi.Cmp(lfOne()) != 0 {
				return nil, false
			}
			minY := (*big.Rat)(nil)
			for _, u := range []*big.Rat{piece.ArcBox[0].Lo, piece.ArcBox[0].Hi} {
				for _, v := range []*big.Rat{piece.ArcBox[1].Lo, piece.ArcBox[1].Hi} {
					y := lfAdd(piece.Affine.Offset[0], lfAdd(
						lfMul(piece.Affine.Column[0][0], u),
						lfMul(piece.Affine.Column[1][0], v)))
					if minY == nil || y.Cmp(minY) < 0 {
						minY = y
					}
				}
			}
			if minY == nil || minY.Sign() <= 0 {
				return nil, false
			}
			sign := 1
			if piece.From.Cmp(piece.To) > 0 {
				sign = -1
			}
			signs[index] = sign
			continue
		}
		y := polynomial.BernsteinRestrict(piece.Y, piece.Piece.Source.Lo, piece.Piece.Source.Hi)
		z := polynomial.BernsteinRestrict(piece.Z, piece.Piece.Source.Lo, piece.Piece.Source.Hi)
		for _, control := range y {
			if control.Sign() <= 0 {
				return nil, false
			}
		}
		angular := polynomial.BernsteinSub(
			polynomial.BernsteinMul(y, polynomial.BernsteinDeriv(z)),
			polynomial.BernsteinMul(z, polynomial.BernsteinDeriv(y)))
		sign := angular[0].Sign()
		if piece.From.Cmp(piece.To) > 0 {
			sign = -sign
		}
		if sign == 0 {
			return nil, false
		}
		for _, control := range angular {
			if control.Sign() != sign && piece.From.Cmp(piece.To) < 0 ||
				control.Sign() != -sign && piece.From.Cmp(piece.To) > 0 {
				return nil, false
			}
		}
		signs[index] = sign
	}
	return signs, len(signs) > 0
}

// VerticalTraceSourceProof checks the supported line and centered-arc path.
// Adjacent line and circular pieces separate radially; disjoint lines
// separate by their exact hulls.
func VerticalTraceSourceProof(pieces []VerticalTraceSourcePiece,
	outerRadius *big.Rat) bool {
	if len(pieces) < 2 || outerRadius == nil || outerRadius.Sign() <= 0 {
		return false
	}
	outer2 := lfMul(outerRadius, outerRadius)
	for _, piece := range pieces {
		if piece.From == nil || piece.To == nil || piece.From.Cmp(piece.To) == 0 {
			return false
		}
		if piece.Arc != nil {
			if piece.Radius2 == nil || piece.Radius2.Sign() <= 0 ||
				piece.Radius2.Cmp(outer2) >= 0 {
				return false
			}
			continue
		}
		if len(piece.Y) != 2 || len(piece.Z) != 2 {
			return false
		}
		for _, parameter := range []*big.Rat{piece.Piece.Source.Lo, piece.Piece.Source.Hi} {
			point := VerticalTraceSourcePoint(piece, parameter)
			if point[0] == nil || point[1] == nil || point[0].Sign() <= 0 ||
				lfAdd(lfMul(point[0], point[0]), lfMul(point[1], point[1])).Cmp(outer2) >= 0 {
				return false
			}
		}
	}
	for i := range pieces {
		for j := i + 1; j < len(pieces); j++ {
			if j == i+1 {
				if !verticalTraceAdjacentProof(pieces[i], pieces[j]) {
					return false
				}
				continue
			}
			if !verticalTraceSeparateLineHulls(pieces[i], pieces[j]) {
				return false
			}
		}
	}
	return true
}

func verticalTraceAdjacentProof(a, b VerticalTraceSourcePiece) bool {
	if a.Arc == nil && b.Arc == nil || a.Arc != nil && b.Arc != nil {
		return false
	}
	line, circle := a, b
	shared, far := a.To, a.From
	if a.Arc != nil {
		line, circle = b, a
		shared, far = b.From, b.To
	}
	if len(line.Y) != 2 || len(line.Z) != 2 ||
		circle.Radius2 == nil || circle.Radius2.Sign() <= 0 {
		return false
	}
	contact := VerticalTraceSourcePoint(line, shared)
	other := VerticalTraceSourcePoint(line, far)
	if contact[0] == nil || other[0] == nil ||
		lfAdd(lfMul(contact[0], contact[0]), lfMul(contact[1], contact[1])).Cmp(circle.Radius2) != 0 {
		return false
	}
	// Along a line from the circle, radius squared is r² + 2t(dot) +
	// t²|delta|². A positive dot keeps every open line point outside it.
	return lfAdd(lfMul(contact[0], lfSub(other[0], contact[0])),
		lfMul(contact[1], lfSub(other[1], contact[1]))).Sign() > 0
}

func verticalTraceSeparateLineHulls(a, b VerticalTraceSourcePiece) bool {
	if a.Arc != nil || b.Arc != nil || len(a.Y) != 2 || len(b.Y) != 2 {
		return false
	}
	box := func(piece VerticalTraceSourcePiece) [2][2]*big.Rat {
		y := polynomial.BernsteinRestrict(piece.Y, piece.Piece.Source.Lo, piece.Piece.Source.Hi)
		z := polynomial.BernsteinRestrict(piece.Z, piece.Piece.Source.Lo, piece.Piece.Source.Hi)
		out := [2][2]*big.Rat{{y[0], y[0]}, {z[0], z[0]}}
		for axis, controls := range [2][]*big.Rat{y, z} {
			for _, value := range controls[1:] {
				if value.Cmp(out[axis][0]) < 0 {
					out[axis][0] = value
				}
				if value.Cmp(out[axis][1]) > 0 {
					out[axis][1] = value
				}
			}
		}
		return out
	}
	left, right := box(a), box(b)
	for axis := range left {
		if left[axis][1].Cmp(right[axis][0]) < 0 ||
			right[axis][1].Cmp(left[axis][0]) < 0 {
			return true
		}
	}
	return false
}
