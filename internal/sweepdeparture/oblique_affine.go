package sweepdeparture

import (
	"math/big"

	pairbox "github.com/lestrrat-3d/decad/internal/pair/box"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
)

// ObliqueAffine proves an open-time gap between touching source boxes when
// their exact common support plane separates under translation.
func ObliqueAffine(paths [2]Path, contactPoints int) (*big.Rat, bool) {
	if paths[0].Drift || paths[1].Drift || contactPoints != 4 {
		return nil, false
	}
	for axis := range 3 {
		i, j := (axis+1)%3, (axis+2)%3
		normal := proofarith.DvCross(paths[0].Box.Edge[i], paths[0].Box.Edge[j])
		if proofarith.DvIsZero(normal) {
			continue
		}
		alo, ahi := pairbox.OrientedProjection(paths[0].Box, normal)
		blo, bhi := pairbox.OrientedProjection(paths[1].Box, normal)
		side := 0
		switch {
		case proofarith.DyCmp(ahi, blo) == 0:
			side = 1
		case proofarith.DyCmp(bhi, alo) == 0:
			side = -1
		default:
			continue
		}
		relative := proofarith.DyV3{}
		for k := range 3 {
			relative[k] = proofarith.DySubScalar(paths[1].Delta[k], paths[0].Delta[k])
		}
		slope := proofarith.DvDot(relative, normal)
		if side < 0 {
			slope = proofarith.DyNeg(slope)
		}
		if slope.Sign() > 0 {
			return big.NewRat(1, 2), true
		}
	}
	return nil, false
}
