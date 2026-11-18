package meshbool

import (
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/surfacenormal"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// revolvedSegment is a revolve record about the z axis whose meridian runs from (0, 2) to (z1, rho1).
func revolvedSegment(t *testing.T, z1, rho1 float64) *surfacenormal.Revolved {
	t.Helper()
	point := func(x float64) proofbound.RatInterval {
		r := new(big.Rat).SetFloat64(x)
		return proofbound.RatInterval{Lo: r, Hi: r}
	}
	vec := func(v r3.Vec) proofbound.IvVec3 {
		iv, ok := proofbound.IvVec3Of(v)
		require.True(t, ok)
		return iv
	}
	return &surfacenormal.Revolved{
		Origin: vec(r3.Vec{}),
		Basis:  [3]proofbound.IvVec3{vec(r3.NewVec(1, 0, 0)), vec(r3.NewVec(0, 1, 0)), vec(r3.NewVec(0, 0, 1))},
		Ends:   [][2][2]proofbound.RatInterval{{{point(0), point(2)}, {point(z1), point(rho1)}}},
		Valid:  true,
	}
}

// TestDenotedTransversal reads a revolve record's rulings against a facet's plane (§16.2).
func TestDenotedTransversal(t *testing.T) {
	t.Parallel()
	// A facet of the plane z = 0.
	a, b, c := r3.NewVec(0, 0, 0), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0)
	t.Run("a 30 degree chamfer cone leaves the plane", func(t *testing.T) {
		t.Parallel()
		require.True(t, denotedTransversal(revolvedSegment(t, 0.5, 2.866), a, b, c))
	})
	t.Run("a cylinder leaves the plane", func(t *testing.T) {
		t.Parallel()
		require.True(t, denotedTransversal(revolvedSegment(t, 3, 2), a, b, c))
	})
	// Rejected: a meridian that lies in the plane, or leans into it below seatSinMin, is no wall that leaves it.
	t.Run("an annulus in the plane refuses", func(t *testing.T) {
		t.Parallel()
		require.False(t, denotedTransversal(revolvedSegment(t, 0, 3), a, b, c))
	})
	t.Run("a cone flatter than seatSinMin refuses", func(t *testing.T) {
		t.Parallel()
		require.False(t, denotedTransversal(revolvedSegment(t, 1e-4, 3), a, b, c))
	})
	t.Run("a facet across the axis refuses", func(t *testing.T) {
		t.Parallel()
		// The plane x = 0 contains the cylinder's rulings.
		require.False(t, denotedTransversal(revolvedSegment(t, 3, 2), r3.NewVec(0, 0, 0), r3.NewVec(0, 1, 0), r3.NewVec(0, 0, 1)))
	})
}

// TestTrueFacePlane reads a placed face's exact plane from its record (§17.1).
func TestTrueFacePlane(t *testing.T) {
	t.Parallel()
	frame, err := r3.NewFrame(r3.NewVec(0, 0, 6), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	tag := surfacegeom.Plane{Frame: frame}
	t.Run("a proved placed plane", func(t *testing.T) {
		t.Parallel()
		f := &topo.Face{Surface: tag, RetainedTag: true, NormalBound: 1e-15, PlacedPlane: &tag}
		pl := trueFacePlane(f)
		require.NotNil(t, pl)
		require.True(t, pl.holds(r3.NewVec(3, -2, 6)))
		require.False(t, pl.holds(r3.NewVec(3, -2, 6.000000000000001)))
	})
	t.Run("a revolve record at one level", func(t *testing.T) {
		t.Parallel()
		r := revolvedSegment(t, 0, 3)
		pl := trueFacePlane(&topo.Face{Surface: tag, RetainedTag: true, NormalBound: 2, Denoted: r})
		require.NotNil(t, pl)
		require.True(t, pl.holds(r3.NewVec(1, 1, 0)))
	})
	// Rejected: a claim that names another tag, or a record whose meridian climbs, states no plane.
	t.Run("a stale claim", func(t *testing.T) {
		t.Parallel()
		other, err := r3.NewFrame(r3.NewVec(0, 0, 7), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
		require.NoError(t, err)
		claim := surfacegeom.Plane{Frame: other}
		require.Nil(t, trueFacePlane(&topo.Face{Surface: tag, RetainedTag: true, PlacedPlane: &claim}))
	})
	t.Run("a climbing meridian", func(t *testing.T) {
		t.Parallel()
		require.Nil(t, trueFacePlane(&topo.Face{Surface: tag, RetainedTag: true, Denoted: revolvedSegment(t, 1, 3)}))
	})
}
