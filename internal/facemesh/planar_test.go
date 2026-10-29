package facemesh_test

import (
	"math/big"
	"slices"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facemesh"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// normalZ is the exact normal (0, 0, z).
func normalZ(z int64) [3]*big.Rat {
	return [3]*big.Rat{new(big.Rat), new(big.Rat), big.NewRat(z, 1)}
}

func loopOf(bound float64, pts ...r3.Vec) facemesh.Loop {
	l := facemesh.Loop{Points: pts}
	for range pts {
		l.Bounds = append(l.Bounds, bound)
	}
	return l
}

// triArea is the planar area of the triangles in the xy plane, exactly, as a rational of float inputs.
func triArea(pts []r3.Vec, tris [][3]int) *big.Rat {
	sum := new(big.Rat)
	for _, t := range tris {
		a, b, c := pts[t[0]], pts[t[1]], pts[t[2]]
		cross := (b.X-a.X)*(c.Y-a.Y) - (b.Y-a.Y)*(c.X-a.X)
		sum.Add(sum, new(big.Rat).SetFloat64(cross))
	}
	return sum.Quo(sum, big.NewRat(2, 1))
}

func TestPlanarPentagon(t *testing.T) {
	pts := []r3.Vec{r3.NewVec(0, 0, 0), r3.NewVec(20, 0, 0), r3.NewVec(20, 20, 0), r3.NewVec(2, 20, 0), r3.NewVec(0, 18, 0)}
	tris, err := facemesh.Planar(t.Context(), normalZ(1), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0),
		[]facemesh.Loop{loopOf(0, pts...)})
	require.NoError(t, err)
	require.Len(t, tris, 3)
	// Shoelace: 20*20 minus the cut corner triangle of area 2.
	require.Equal(t, 0, triArea(pts, tris).Cmp(big.NewRat(398, 1)))
}

func TestPlanarFlippedFrameStillCounterClockwise(t *testing.T) {
	// Outward normal -z: the loop is counter-clockwise about -z, i.e. clockwise in the xy plane. The frame
	// (x, y) has u×v = +z, opposite the normal, so Planar projects with (y, x).
	pts := []r3.Vec{r3.NewVec(0, 0, 0), r3.NewVec(0, 4, 0), r3.NewVec(4, 4, 0), r3.NewVec(4, 0, 0)}
	tris, err := facemesh.Planar(t.Context(), normalZ(-1), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0),
		[]facemesh.Loop{loopOf(0, pts...)})
	require.NoError(t, err)
	require.Len(t, tris, 2)
	// Every triangle winds counter-clockwise about -z, i.e. negative signed xy area.
	for _, tr := range tris {
		a, b, c := pts[tr[0]], pts[tr[1]], pts[tr[2]]
		require.Negative(t, (b.X-a.X)*(c.Y-a.Y)-(b.Y-a.Y)*(c.X-a.X))
	}
}

func TestPlanarKeepsCollinearPoint(t *testing.T) {
	// Triangulate keeps a collinear boundary point as a triangle corner, so every loop point is used.
	pts := []r3.Vec{r3.NewVec(0, 0, 0), r3.NewVec(2, 0, 0), r3.NewVec(4, 0, 0), r3.NewVec(4, 4, 0), r3.NewVec(0, 4, 0)}
	tris, err := facemesh.Planar(t.Context(), normalZ(1), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0),
		[]facemesh.Loop{loopOf(0, pts...)})
	require.NoError(t, err)
	seen := map[int]bool{}
	for _, tr := range tris {
		for _, i := range tr {
			seen[i] = true
		}
	}
	require.Len(t, seen, len(pts))
	require.Equal(t, 0, triArea(pts, tris).Cmp(big.NewRat(16, 1)))
}

func TestCoverageRefusesDroppedPoint(t *testing.T) {
	require.NoError(t, facemesh.CheckCoverage(3, [][3]int{{0, 1, 2}}))
	err := facemesh.CheckCoverage(4, [][3]int{{0, 1, 2}})
	require.ErrorIs(t, err, facemesh.ErrDropped)
	require.ErrorIs(t, err, decaderr.ErrUnsupported)
}

func TestPlanarRefusesSliverBeyondItsBounds(t *testing.T) {
	pts := []r3.Vec{r3.NewVec(0, 0, 0), r3.NewVec(10, 0, 0), r3.NewVec(5, 1e-6, 0)}
	frame := func(bound float64) error {
		_, err := facemesh.Planar(t.Context(), normalZ(1), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0),
			[]facemesh.Loop{loopOf(bound, slices.Clone(pts)...)})
		return err
	}
	require.NoError(t, frame(0), "the sliver is positively oriented as held")
	err := frame(1e-3)
	require.ErrorIs(t, err, facemesh.ErrMargin)
	require.ErrorIs(t, err, decaderr.ErrUnsupported)
}
