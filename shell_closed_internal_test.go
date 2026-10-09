package decad

import (
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// TestShellClosedPrismLevelsCarryThicknessConversion shells a 100×60×20 box
// closed with a 0.1 in wall, which converts to 2.54 mm only within a proven
// displacement: the two derived levels z0 + t and z1 − t carry it, and every
// reading built on them covers the exact rational body at t* = 254/100 mm.
//
// Legs shown to fail (each deleted, the fixture watched go red, then
// restored): the thickness-conversion and float-sum terms of the derived
// levels (shellClosedPrism's step), without which the level deltas and the
// cavity's vertical edge lengths miss. The volume and centroid stay covered
// without them, by the offset's own section displacement.
func TestShellClosedPrismLevelsCarryThicknessConversion(t *testing.T) {
	doc := New()
	box := internalBoxBody(t, doc, 0, 0, 100, 60, 20)
	closed, err := box.Shell(t.Context(), nil, units.Inches(0.1), WithNoOpenings())
	require.NoError(t, err)
	sp := closed.payload.(stackedPrismPayload)
	require.Len(t, sp.slabs, 3)
	require.Positive(t, sp.slabs[0].Z1Delta, `z0 + t carries the conversion`)
	require.Positive(t, sp.slabs[2].Z0Delta, `z1 − t carries the conversion`)

	th := big.NewRat(254, 100)
	two := big.NewRat(2, 1)
	inner := func(full int64) *big.Rat {
		return new(big.Rat).Sub(big.NewRat(full, 1), new(big.Rat).Mul(two, th))
	}
	cavity := new(big.Rat).Mul(new(big.Rat).Mul(inner(100), inner(60)), inner(20))
	want := new(big.Rat).Sub(big.NewRat(100*60*20, 1), cavity)
	volume, err := closed.Volume()
	require.NoError(t, err)
	requireRatCovered(t, volume, want)
	// The cavity's vertical edges run between the two derived levels.
	vertical := 0
	for _, f := range closed.Shells()[1].Faces() {
		for _, e := range f.Edges() {
			_, ok := e.Curve().(Line3)
			if !ok || math.Abs(e.End().position.Z-e.Start().position.Z) < 1 {
				continue
			}
			vertical++
			length, err := e.Length()
			require.NoError(t, err)
			requireRatCovered(t, length, inner(20))
		}
	}
	require.Positive(t, vertical)
	// The centroid is the box's centre by symmetry.
	centroid, err := closed.Centroid()
	require.NoError(t, err)
	requireVecCovered(t, centroid, [3]*big.Rat{big.NewRat(50, 1), big.NewRat(30, 1), big.NewRat(10, 1)})
}

// TestStackedLumpsNeedsOneOuterShellForACavity reads stackedLumps directly: a
// connected face set with no anchor is a void, attached to the one outer
// shell, and several outer shells beside a cavity are ErrUnsupported.
func TestStackedLumpsNeedsOneOuterShellForACavity(t *testing.T) {
	outerA, outerB, cavity := &Face{}, &Face{}, &Face{}
	anchors := map[*Face]struct{}{outerA: {}, outerB: {}}
	lumps, err := stackedLumps([]*Face{outerA, cavity}, anchors)
	require.NoError(t, err)
	require.Len(t, lumps, 1)
	require.Len(t, lumps[0].shells, 2)
	require.True(t, lumps[0].shells[1].void)
	_, err = stackedLumps([]*Face{outerA, outerB, cavity}, anchors)
	require.ErrorIs(t, err, ErrUnsupported)
	lumps, err = stackedLumps([]*Face{outerA, outerB}, anchors)
	require.NoError(t, err)
	require.Len(t, lumps, 2)
}
