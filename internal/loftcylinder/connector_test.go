package loftcylinder

import (
	"math/big"
	"testing"

	"github.com/stretchr/testify/require"
)

// TestConnectorReachCollar widens the collar past the heights where a
// connector's zero-length first piece starts inside the circle, and keeps a
// collar that already settles the endpoints.
func TestConnectorReachCollar(t *testing.T) {
	t.Parallel()
	scale := func(p XY, s *big.Rat) XY {
		return XY{new(big.Rat).Mul(p.X, s), new(big.Rat).Mul(p.Y, s)}
	}
	// The unit vector (99, 20)/101 lies 0.2 rad from the X axis. Both root
	// endpoints sit 2^-40 of the radius inside the circle of radius 5, and
	// the first piece has zero length on the bottom cap.
	inside := new(big.Rat).Sub(big.NewRat(5, 1), new(big.Rat).SetFrac64(5, 1<<40))
	turned := XY{big.NewRat(99, 101), big.NewRat(20, 101)}
	a0 := XY{inside, new(big.Rat)}
	a1 := scale(turned, inside)
	f0 := XY{big.NewRat(6, 1), new(big.Rat)}
	f1 := scale(turned, big.NewRat(6, 1))
	c := Connector{a: [2]XY{a0, a1}, d: [2]XY{a0, f1}, f: [2]XY{f0, f1}}
	r2 := big.NewRat(25, 1)
	height := big.NewRat(1, 1)

	require.False(t, connectorEndsSettled(c, r2, new(big.Rat), height),
		"the far end must start inside the circle at the bottom cap")
	collar, ok := connectorReachCollar([]Connector{c}, r2, height, 0)
	require.True(t, ok, "a dyadic collar must settle the endpoints")
	require.Greater(t, collar, 0.0, "the collar must widen past the bottom cap")
	lo := new(big.Rat).SetFloat64(collar)
	require.True(t, connectorEndsSettled(c, r2, lo, new(big.Rat).Sub(height, lo)),
		"the returned collar must settle the endpoints")
	half := new(big.Rat).SetFloat64(collar / 2)
	require.False(t, connectorEndsSettled(c, r2, half, new(big.Rat).Sub(height, half)),
		"the next smaller dyadic collar must not settle the endpoints")

	kept, ok := connectorReachCollar([]Connector{c}, r2, height, 4*collar)
	require.True(t, ok, "a wider settled collar must be admitted")
	require.Equal(t, 4*collar, kept, "a settled collar must be kept")

	// A far end inside the circle at both caps never settles.
	short := Connector{a: c.a, d: [2]XY{a0, a1}, f: [2]XY{a0, a1}}
	_, ok = connectorReachCollar([]Connector{short}, r2, height, 0)
	require.False(t, ok, "a connector without an outside far end must be refused")
}
