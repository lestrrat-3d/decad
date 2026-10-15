package decad

import (
	"context"

	"github.com/lestrrat-3d/decad/internal/loftface"
)

// A radial joint is one authenticated Sketch fit/arc adjacency mapped through
// a LoftFaces band. The whole arc has the same conic side and axial position
// as the fit endpoint at each band time. Its exact time roots are split into
// disjoint rational brackets; only roots inside the finite Revolve section
// may contribute a contact curve.
type revolveLoftFacesRadialJoint struct {
	vertex   revolveLoftFacesJointVertex
	radial   mappedArcRadialBand
	roots    []loftface.EndpointRoot
	pieces   []loftface.TimePiece
	fitCells []revolveLoftFacesJointCell
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
	memo := revolveLoftFacesMeridianMemoOf(ctx)
	q := memo.QRows(trim.Maps[band], trim.Maps[band+1], vertex.fitSpan, m, b)
	index := 0
	if vertex.fitAtEnd {
		index = len(q[0]) - 1
	}
	g0 := trim.Maps[band].Denominator(vertex.coordinate[0], vertex.coordinate[1])
	g1 := trim.Maps[band+1].Denominator(vertex.coordinate[0], vertex.coordinate[1])
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
	for capIndex, mapped := range [...]loftFaceMap{trim.Maps[band], trim.Maps[band+1]} {
		if mapped.At(vertex.coordinate[0], vertex.coordinate[1])[0].Cmp(radial.x[capIndex]) != 0 {
			return empty, false
		}
	}
	roots, ok := revolveLoftFacesEndpointFiniteRootsWith(memo, trim, section,
		vertex.fitSpan, band, vertex.fitAtEnd)
	if !ok {
		return empty, false
	}
	pieces, ok := loftface.PartitionTime(radial.q, roots)
	if !ok {
		return empty, false
	}
	fitCells := make([]revolveLoftFacesJointCell, len(roots))
	for i, root := range roots {
		fitCells[i], ok = revolveLoftFacesKnotRootCellWith(memo, trim, section,
			vertex.fitSpan, band, vertex.fitAtEnd, root)
		if !ok {
			return empty, false
		}
	}
	return revolveLoftFacesRadialJoint{vertex: vertex, radial: radial,
		roots: roots, pieces: pieces, fitCells: fitCells}, true
}
