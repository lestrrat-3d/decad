package box_test

import (
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/pair/box"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/stretchr/testify/require"
)

func TestAffineSourceBoxDepartureChecksOpposedSupport(t *testing.T) {
	dy := proof.MustDyOf
	a := box.AxisBox{Lo: [3]proof.Dyadic{dy(0), dy(0), dy(0)}, Hi: [3]proof.Dyadic{dy(1), dy(1), dy(1)}}
	b := box.AxisBox{Lo: [3]proof.Dyadic{dy(1), dy(0), dy(0)}, Hi: [3]proof.Dyadic{dy(2), dy(1), dy(1)}}
	var still [3]proof.Dyadic
	outward := [3]proof.Dyadic{dy(1)}
	reverse := [3]proof.Dyadic{dy(-1)}
	tangent := [3]proof.Dyadic{dy(0), dy(1)}

	require.True(t, box.ProvesDeparture(a, b, still, outward))
	require.True(t, box.ProvesDeparture(b, a, outward, still))
	require.False(t, box.ProvesDeparture(a, b, still, reverse))
	require.False(t, box.ProvesDeparture(a, b, still, tangent))
}

func TestAffineSourceBoxContactSpanKeepsExactEntryAndExit(t *testing.T) {
	dy := proof.MustDyOf
	a := box.AxisBox{Lo: [3]proof.Dyadic{dy(0), dy(0), dy(0)}, Hi: [3]proof.Dyadic{dy(1), dy(1), dy(1)}}
	b := box.AxisBox{Lo: [3]proof.Dyadic{dy(2), dy(0), dy(0)}, Hi: [3]proof.Dyadic{dy(3), dy(1), dy(1)}}
	var still [3]proof.Dyadic
	toward := [3]proof.Dyadic{dy(-4)}

	entry, exit, ok := box.ContactSpan(a, b, still, toward)
	require.True(t, ok)
	require.Zero(t, entry.Cmp(big.NewRat(1, 4)))
	require.Zero(t, exit.Cmp(big.NewRat(3, 4)))

	entry, exit, ok = box.ContactSpan(b, a, toward, still)
	require.True(t, ok)
	require.Zero(t, entry.Cmp(big.NewRat(1, 4)))
	require.Zero(t, exit.Cmp(big.NewRat(3, 4)))

	entry, exit, ok = box.ContactSpan(a, b, still, still)
	require.False(t, ok)
	require.Zero(t, entry.Cmp(big.NewRat(0, 1)))
	require.Zero(t, exit.Cmp(big.NewRat(1, 1)))
}
