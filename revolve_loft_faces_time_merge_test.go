package decad

import (
	"math/big"
	"testing"

	"github.com/lestrrat-3d/sketch"
	"github.com/stretchr/testify/require"
)

func TestRevolveLoftFacesOwnedTimeRootsRemainDisjoint(t *testing.T) {
	knotPoint, arcPoint := new(sketch.Point), new(sketch.Point)
	makeRoot := func(lo, hi *big.Rat, kind revolveLoftFacesJointKind,
		point *sketch.Point, atEnd bool) revolveLoftFacesOwnedRoot {
		sLo, sHi := big.NewRat(0, 1), big.NewRat(1, 8)
		if atEnd {
			sLo, sHi = big.NewRat(7, 8), big.NewRat(1, 1)
		}
		return revolveLoftFacesOwnedRoot{
			root: revolveLoftFacesEndpointRoot{
				time: loftFaceInterval{lo: lo, hi: hi}, finite: true},
			cell: revolveLoftFacesJointCell{
				time: loftFaceInterval{lo: lo, hi: hi},
				sLo:  sLo, sHi: sHi, finite: true},
			kind: kind, point: point, atEnd: atEnd,
		}
	}
	knot := makeRoot(big.NewRat(1, 4), big.NewRat(1, 3),
		revolveLoftFacesFitKnotContact, knotPoint, false)
	arc := makeRoot(big.NewRat(2, 3), big.NewRat(3, 4),
		revolveLoftFacesRadialArcContact, arcPoint, true)
	roots := []revolveLoftFacesOwnedRoot{arc, knot}
	require.True(t, revolveLoftFacesOrderOwnedRoots(roots))
	require.Equal(t, revolveLoftFacesFitKnotContact, roots[0].kind)
	require.Same(t, knotPoint, roots[0].point)
	require.Equal(t, revolveLoftFacesRadialArcContact, roots[1].kind)
	require.Same(t, arcPoint, roots[1].point)

	overlap := makeRoot(big.NewRat(1, 3), big.NewRat(1, 2),
		revolveLoftFacesRadialArcContact, arcPoint, true)
	require.False(t, revolveLoftFacesOrderOwnedRoots([]revolveLoftFacesOwnedRoot{knot, overlap}))
	wrongEnd := arc
	wrongEnd.cell.sHi = big.NewRat(7, 8)
	require.False(t, revolveLoftFacesOrderOwnedRoots([]revolveLoftFacesOwnedRoot{knot, wrongEnd}))
}
