package decad

import (
	"math/big"

	"github.com/lestrrat-3d/decad/internal/polynomial"
)

// The supported line/arc contacts have simple source geometry. A strict
// angular derivative proves each piece is simple. Adjacent line and circular
// pieces separate radially; disjoint lines separate by their exact hulls.
func revolveLoftFacesVerticalTraceSourceProof(pieces []revolveLoftFacesVerticalTracePiece,
	outerRadius *big.Rat) bool {
	if len(pieces) < 2 || outerRadius == nil || outerRadius.Sign() <= 0 {
		return false
	}
	outer2 := lfMul(outerRadius, outerRadius)
	for _, piece := range pieces {
		if piece.from == nil || piece.to == nil || piece.from.Cmp(piece.to) == 0 {
			return false
		}
		if piece.arc != nil {
			if piece.radius2 == nil || piece.radius2.Sign() <= 0 ||
				piece.radius2.Cmp(outer2) >= 0 {
				return false
			}
			continue
		}
		if len(piece.y) != 2 || len(piece.z) != 2 {
			return false
		}
		for _, parameter := range []*big.Rat{piece.piece.Source.Lo, piece.piece.Source.Hi} {
			point := revolveLoftFacesVerticalTracePoint(piece, parameter)
			if point[0] == nil || point[1] == nil || point[0].Sign() <= 0 ||
				lfAdd(lfMul(point[0], point[0]), lfMul(point[1], point[1])).Cmp(outer2) >= 0 {
				return false
			}
		}
	}
	for i := range pieces {
		for j := i + 1; j < len(pieces); j++ {
			if j == i+1 {
				if !revolveLoftFacesVerticalTraceAdjacentProof(pieces[i], pieces[j]) {
					return false
				}
				continue
			}
			if !revolveLoftFacesVerticalTraceSeparateLineHulls(pieces[i], pieces[j]) {
				return false
			}
		}
	}
	return true
}

func revolveLoftFacesVerticalTraceAdjacentProof(a, b revolveLoftFacesVerticalTracePiece) bool {
	if a.arc == nil && b.arc == nil || a.arc != nil && b.arc != nil {
		return false
	}
	line, circle := a, b
	shared, far := a.to, a.from
	if a.arc != nil {
		line, circle = b, a
		shared, far = b.from, b.to
	}
	if len(line.y) != 2 || len(line.z) != 2 ||
		circle.radius2 == nil || circle.radius2.Sign() <= 0 {
		return false
	}
	contact := revolveLoftFacesVerticalTracePoint(line, shared)
	other := revolveLoftFacesVerticalTracePoint(line, far)
	if contact[0] == nil || other[0] == nil ||
		lfAdd(lfMul(contact[0], contact[0]), lfMul(contact[1], contact[1])).Cmp(circle.radius2) != 0 {
		return false
	}
	// Along a line from the circle, radius squared is r² + 2t(dot) +
	// t²|delta|². A positive dot keeps every open line point outside it.
	return lfAdd(lfMul(contact[0], lfSub(other[0], contact[0])),
		lfMul(contact[1], lfSub(other[1], contact[1]))).Sign() > 0
}

func revolveLoftFacesVerticalTraceSeparateLineHulls(a, b revolveLoftFacesVerticalTracePiece) bool {
	if a.arc != nil || b.arc != nil || len(a.y) != 2 || len(b.y) != 2 {
		return false
	}
	box := func(piece revolveLoftFacesVerticalTracePiece) [2][2]*big.Rat {
		y := polynomial.BernsteinRestrict(piece.y, piece.piece.Source.Lo, piece.piece.Source.Hi)
		z := polynomial.BernsteinRestrict(piece.z, piece.piece.Source.Lo, piece.piece.Source.Hi)
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
