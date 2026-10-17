package decad

import (
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/loftface"
	"github.com/lestrrat-3d/decad/internal/revolveloftfaces"
	"github.com/lestrrat-3d/sketch"
	"github.com/stretchr/testify/require"
)

func TestRevolveLoftFacesOwnedTimeRootsRemainDisjoint(t *testing.T) {
	knotPoint, arcPoint := new(sketch.Point), new(sketch.Point)
	makeRoot := func(lo, hi *big.Rat, kind revolveloftfaces.RevolveLoftFacesJointKind,
		point *sketch.Point, atEnd bool) revolveloftfaces.RevolveLoftFacesOwnedRoot {
		sLo, sHi := big.NewRat(0, 1), big.NewRat(1, 8)
		if atEnd {
			sLo, sHi = big.NewRat(7, 8), big.NewRat(1, 1)
		}
		return revolveloftfaces.RevolveLoftFacesOwnedRoot{
			Root: loftface.EndpointRoot{
				Time: loftFaceInterval{Lo: lo, Hi: hi}, Finite: true},
			Cell: revolveloftfaces.RevolveLoftFacesJointCell{
				Time: loftFaceInterval{Lo: lo, Hi: hi},
				SLo:  sLo, SHi: sHi, Finite: true},
			Kind: kind, Point: point, AtEnd: atEnd,
		}
	}
	knot := makeRoot(big.NewRat(1, 4), big.NewRat(1, 3),
		revolveloftfaces.RevolveLoftFacesFitKnotContact, knotPoint, false)
	arc := makeRoot(big.NewRat(2, 3), big.NewRat(3, 4),
		revolveloftfaces.RevolveLoftFacesRadialArcContact, arcPoint, true)
	roots := []revolveloftfaces.RevolveLoftFacesOwnedRoot{arc, knot}
	require.True(t, revolveloftfaces.RevolveLoftFacesOrderOwnedRoots(roots))
	require.Equal(t, revolveloftfaces.RevolveLoftFacesFitKnotContact, roots[0].Kind)
	require.Same(t, knotPoint, roots[0].Point)
	require.Equal(t, revolveloftfaces.RevolveLoftFacesRadialArcContact, roots[1].Kind)
	require.Same(t, arcPoint, roots[1].Point)

	overlap := makeRoot(big.NewRat(1, 3), big.NewRat(1, 2),
		revolveloftfaces.RevolveLoftFacesRadialArcContact, arcPoint, true)
	require.False(t, revolveloftfaces.RevolveLoftFacesOrderOwnedRoots([]revolveloftfaces.RevolveLoftFacesOwnedRoot{knot, overlap}))
	wrongEnd := arc
	wrongEnd.Cell.SHi = big.NewRat(7, 8)
	require.False(t, revolveloftfaces.RevolveLoftFacesOrderOwnedRoots([]revolveloftfaces.RevolveLoftFacesOwnedRoot{knot, wrongEnd}))
}
