package decad

import (
	"context"
	"math/big"
)

// A radial joint is one authenticated Sketch fit/arc adjacency mapped through
// a LoftFaces band. The whole arc has the same conic side and axial position
// as the fit endpoint at each band time. Its exact time roots are split into
// disjoint rational brackets; only roots inside the finite Revolve section
// may contribute a contact curve.
type revolveLoftFacesRadialJoint struct {
	vertex   revolveLoftFacesJointVertex
	radial   mappedArcRadialBand
	roots    []revolveLoftFacesEndpointRoot
	pieces   []revolveLoftFacesTimePiece
	fitCells []revolveLoftFacesJointCell
}

type revolveLoftFacesTimePiece struct {
	lo, hi         *big.Rat
	loOpen, hiOpen bool
	side           int
	rootIndex      int // -1 for a strict-side piece
}

func certifyRevolveLoftFacesRadialJoint(ctx context.Context, trim *loftFacesTrimRecord,
	section revolveSectionSpan, fitIndex, arcIndex, band int,
	fitAtEnd bool) (revolveLoftFacesRadialJoint, bool) {
	var empty revolveLoftFacesRadialJoint
	if ctx == nil || ctx.Err() != nil {
		return empty, false
	}
	vertex, ok := revolveLoftFacesFitArcVertex(trim, fitIndex, arcIndex, fitAtEnd)
	if !ok {
		return empty, false
	}
	radial, ok := mappedArcRadialTimeBand(ctx, trim, arcIndex, band, section)
	if !ok {
		return empty, false
	}
	m, b := revolveLoftFacesMeridian(section)
	q := spiralRootPolynomial(trim.maps[band], trim.maps[band+1], vertex.fitSpan, m, b)
	index := 0
	if vertex.fitAtEnd {
		index = len(q[0]) - 1
	}
	g0 := trim.maps[band].denominator(vertex.coordinate[0], vertex.coordinate[1])
	g1 := trim.maps[band+1].denominator(vertex.coordinate[0], vertex.coordinate[1])
	if g0.Sign() <= 0 || g1.Sign() <= 0 {
		return empty, false
	}
	d := lfMul(g0, g1)
	d2 := lfMul(d, d)
	for row := range q {
		if lfDiv(q[row][index], d2).Cmp(radial.q[row]) != 0 {
			return empty, false
		}
	}
	for capIndex, mapped := range [...]loftFaceMap{trim.maps[band], trim.maps[band+1]} {
		if mapped.at(vertex.coordinate[0], vertex.coordinate[1])[0].Cmp(radial.x[capIndex]) != 0 {
			return empty, false
		}
	}
	roots, ok := revolveLoftFacesEndpointFiniteRoots(trim, section,
		vertex.fitSpan, band, vertex.fitAtEnd)
	if !ok {
		return empty, false
	}
	pieces, ok := revolveLoftFacesPartitionTime(radial.q, roots)
	if !ok {
		return empty, false
	}
	fitCells := make([]revolveLoftFacesJointCell, len(roots))
	for i, root := range roots {
		fitCells[i], ok = revolveLoftFacesKnotRootCell(trim, section,
			vertex.fitSpan, band, vertex.fitAtEnd, root)
		if !ok {
			return empty, false
		}
	}
	return revolveLoftFacesRadialJoint{vertex: vertex, radial: radial,
		roots: roots, pieces: pieces, fitCells: fitCells}, true
}

// Quadratic degree plus isolated sign-changing root brackets proves there is
// no unlisted side change in any strict piece. The bracket pieces remain
// undecidable as point cells; the radial-joint source certificate binds each
// entire arc contact there instead.
func revolveLoftFacesPartitionTime(q [3]*big.Rat,
	roots []revolveLoftFacesEndpointRoot) ([]revolveLoftFacesTimePiece, bool) {
	if len(roots) == 0 || len(roots) > 2 || q[0] == nil || q[1] == nil || q[2] == nil {
		return nil, false
	}
	coeff := []*big.Rat{q[0], q[1], q[2]}
	pieces := make([]revolveLoftFacesTimePiece, 0, 2*len(roots)+1)
	previous := lfZero()
	for index, root := range roots {
		if root.time.lo == nil || root.time.hi == nil ||
			root.time.lo.Cmp(previous) <= 0 || root.time.hi.Cmp(root.time.lo) < 0 ||
			root.time.hi.Cmp(lfOne()) >= 0 || root.direction == 0 {
			return nil, false
		}
		atLo, atHi := spiralBernSign(coeff, root.time.lo),
			spiralBernSign(coeff, root.time.hi)
		if root.time.lo.Cmp(root.time.hi) == 0 {
			if atLo != 0 {
				return nil, false
			}
		} else if atLo != -root.direction || atHi != root.direction {
			return nil, false
		}
		side := spiralBernSign(coeff, lfMul(lfAdd(previous, root.time.lo), big.NewRat(1, 2)))
		if side == 0 || side != -root.direction {
			return nil, false
		}
		pieces = append(pieces, revolveLoftFacesTimePiece{
			lo: previous, hi: root.time.lo, loOpen: index != 0,
			hiOpen: true, side: side, rootIndex: -1})
		pieces = append(pieces, revolveLoftFacesTimePiece{
			lo: root.time.lo, hi: root.time.hi, rootIndex: index})
		previous = root.time.hi
	}
	side := spiralBernSign(coeff, lfMul(lfAdd(previous, lfOne()), big.NewRat(1, 2)))
	if side == 0 || side != roots[len(roots)-1].direction {
		return nil, false
	}
	pieces = append(pieces, revolveLoftFacesTimePiece{
		lo: previous, hi: lfOne(), loOpen: true, side: side, rootIndex: -1})
	return pieces, true
}
