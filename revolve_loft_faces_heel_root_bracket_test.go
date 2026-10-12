package decad

import (
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/stretchr/testify/require"
)

func TestCurvedHeelRootBracketRefusesNearTangentSourceTravel(t *testing.T) {
	epsilon := new(big.Rat).SetFrac(big.NewInt(1), new(big.Int).Lsh(big.NewInt(1), 40))
	delta := big.NewRat(1, 128)
	mapAt := func(y *big.Rat) loftFaceMap {
		return loftFaceMap{constant: loftFaceRatVec{lfZero(), lfZero(), lfZero()},
			q0: loftFaceRatVec{lfOne(), lfSub(y, lfMul(epsilon, delta)), lfZero()},
			qu: loftFaceRatVec{lfZero(), epsilon, lfOne()},
			qv: loftFaceRatVec{lfZero(), lfZero(), lfZero()},
			g0: lfOne(), gu: lfZero(), gv: lfZero()}
	}
	first, last := mapAt(lfOne()), mapAt(lfZero())
	span := freeform.BezierSpan{{U: lfZero(), V: lfZero()},
		{U: lfOne(), V: lfZero()}}
	cell := revolveLoftFacesJointCell{sLo: lfZero(), sHi: lfOne(),
		time:        loftFaceInterval{lo: lfZero(), hi: epsilon},
		outsideSide: -1, sDerivative: 1, finite: true}
	// The root travels from source U=1/128 to U=1 inside one narrow time
	// bracket, although the fixed U=0 end stays strictly outside the cone.
	require.Less(t, first.at(lfZero(), lfZero())[1].Cmp(lfOne()), 0)
	farAtEnd := lfAdd(lfMul(lfSub(lfOne(), epsilon),
		first.at(lfOne(), lfZero())[1]),
		lfMul(epsilon, last.at(lfOne(), lfZero())[1]))
	require.Less(t, farAtEnd.Cmp(lfOne()), 0)
	_, _, ok := revolveLoftFacesHeelJointSliverBox(first, last, span,
		revolveSectionSpan{start: Point2{U: 0, V: 1}, end: Point2{U: 2, V: 1}},
		cell, true, big.NewRat(1, 50))
	require.False(t, ok)
}
