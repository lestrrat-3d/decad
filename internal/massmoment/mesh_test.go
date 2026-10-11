package massmoment_test

import (
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/massmoment"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestHeldMeshIntervalsKeepDyadicTetraMomentsExact(t *testing.T) {
	verts := []r3.Vec{
		r3.NewVec(0, 0, 0),
		r3.NewVec(1, 0, 0),
		r3.NewVec(0, 0.5, 0),
		r3.NewVec(0, 0, 0.25),
	}
	tris := [][3]int{{0, 2, 1}, {0, 1, 3}, {0, 3, 2}, {1, 2, 3}}
	got, err := massmoment.HeldMeshIntervals(t.Context(), massmoment.MeshBounds{
		Min: r3.NewVec(0, 0, 0), Max: r3.NewVec(1, 0.5, 0.25),
		Bound: units.Millimeters(0),
	}, r3.NewVec(0.125, -0.0625, 0.03125), verts, tris, 0)
	require.NoError(t, err)

	checks := []struct {
		name string
		got  proofbound.RatInterval
		want *big.Rat
	}{
		{"volume", got.Volume, big.NewRat(1, 48)},
		{"center x", got.Center[0], big.NewRat(1, 4)},
		{"center y", got.Center[1], big.NewRat(1, 8)},
		{"center z", got.Center[2], big.NewRat(1, 16)},
		{"central xx", got.Central[0][0], big.NewRat(1, 1280)},
		{"central yy", got.Central[1][1], big.NewRat(1, 5120)},
		{"central zz", got.Central[2][2], big.NewRat(1, 20480)},
		{"central xy", got.Central[0][1], big.NewRat(-1, 7680)},
		{"central xz", got.Central[0][2], big.NewRat(-1, 15360)},
		{"central yz", got.Central[1][2], big.NewRat(-1, 30720)},
	}
	for _, check := range checks {
		require.Zero(t, check.got.Lo.Cmp(check.want), check.name+" lower")
		require.Zero(t, check.got.Hi.Cmp(check.want), check.name+" upper")
	}
}
