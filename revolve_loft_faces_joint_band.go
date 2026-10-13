package decad

import (
	"context"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/polynomial"
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
	Lo, Hi         *big.Rat
	LoOpen, HiOpen bool
	Side           int
	RootIndex      int // -1 for a strict-side piece
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
	g0 := trim.maps[band].Denominator(vertex.coordinate[0], vertex.coordinate[1])
	g1 := trim.maps[band+1].Denominator(vertex.coordinate[0], vertex.coordinate[1])
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
		if mapped.At(vertex.coordinate[0], vertex.coordinate[1])[0].Cmp(radial.x[capIndex]) != 0 {
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
		if root.Time.Lo == nil || root.Time.Hi == nil ||
			root.Time.Lo.Cmp(previous) <= 0 || root.Time.Hi.Cmp(root.Time.Lo) < 0 ||
			root.Time.Hi.Cmp(lfOne()) >= 0 || root.Direction == 0 {
			return nil, false
		}
		atLo, atHi := polynomial.BernsteinSign(coeff, root.Time.Lo),
			polynomial.BernsteinSign(coeff, root.Time.Hi)
		if root.Time.Lo.Cmp(root.Time.Hi) == 0 {
			if atLo != 0 {
				return nil, false
			}
		} else if atLo != -root.Direction || atHi != root.Direction {
			return nil, false
		}
		side := polynomial.BernsteinSign(coeff, lfMul(lfAdd(previous, root.Time.Lo), big.NewRat(1, 2)))
		if side == 0 || side != -root.Direction {
			return nil, false
		}
		pieces = append(pieces, revolveLoftFacesTimePiece{
			Lo: previous, Hi: root.Time.Lo, LoOpen: index != 0,
			HiOpen: true, Side: side, RootIndex: -1})
		pieces = append(pieces, revolveLoftFacesTimePiece{
			Lo: root.Time.Lo, Hi: root.Time.Hi, RootIndex: index})
		previous = root.Time.Hi
	}
	side := polynomial.BernsteinSign(coeff, lfMul(lfAdd(previous, lfOne()), big.NewRat(1, 2)))
	if side == 0 || side != roots[len(roots)-1].Direction {
		return nil, false
	}
	pieces = append(pieces, revolveLoftFacesTimePiece{
		Lo: previous, Hi: lfOne(), LoOpen: true, Side: side, RootIndex: -1})
	return pieces, true
}
