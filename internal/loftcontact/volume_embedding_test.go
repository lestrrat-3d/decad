package loftcontact_test

import (
	"context"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/loftcontact"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/stretchr/testify/require"
)

type volumeTestShell struct {
	world     [][3]*big.Rat
	triangles [][3]int
}

func volumeTestPoint(x, y, z int64) [3]*big.Rat {
	return [3]*big.Rat{big.NewRat(x, 1), big.NewRat(y, 1), big.NewRat(z, 1)}
}

func volumeTestTetra() volumeTestShell {
	return volumeTestShell{
		world: [][3]*big.Rat{
			volumeTestPoint(0, 0, 0), volumeTestPoint(1, 0, 0),
			volumeTestPoint(0, 1, 0), volumeTestPoint(0, 0, 1),
		},
		triangles: [][3]int{{0, 2, 1}, {0, 1, 3}, {0, 3, 2}, {1, 2, 3}},
	}
}

func volumeTestCube() volumeTestShell {
	return volumeTestShell{
		world: [][3]*big.Rat{
			volumeTestPoint(0, 0, 0), volumeTestPoint(1, 0, 0),
			volumeTestPoint(1, 1, 0), volumeTestPoint(0, 1, 0),
			volumeTestPoint(0, 0, 1), volumeTestPoint(1, 0, 1),
			volumeTestPoint(1, 1, 1), volumeTestPoint(0, 1, 1),
		},
		triangles: [][3]int{
			{0, 2, 1}, {0, 3, 2}, {4, 5, 6}, {4, 6, 7},
			{0, 1, 5}, {0, 5, 4}, {1, 2, 6}, {1, 6, 5},
			{2, 3, 7}, {2, 7, 6}, {3, 0, 4}, {3, 4, 7},
		},
	}
}

func volumeExactShell(budget *proofbound.WorkBudget, shell volumeTestShell) (*big.Rat, bool, error) {
	return loftcontact.VolumeExactShell(budget, shell.world, shell.triangles)
}

func TestLoftVolumeExactShellEmbeddedPositive(t *testing.T) {
	for _, tc := range []struct {
		name  string
		shell volumeTestShell
		want  *big.Rat
	}{
		{"tetra", volumeTestTetra(), big.NewRat(1, 6)},
		{"triangulated cube", volumeTestCube(), big.NewRat(1, 1)},
	} {
		t.Run(tc.name, func(t *testing.T) {
			volume, ok, err := volumeExactShell(proofbound.NewWorkBudget(t.Context()), tc.shell)
			require.NoError(t, err)
			require.True(t, ok)
			require.Zero(t, volume.Cmp(tc.want))
		})
	}
}

func TestLoftVolumeExactShellRefusesCrossingAndReversal(t *testing.T) {
	for _, tc := range []struct {
		name  string
		shell volumeTestShell
	}{
		{"crossing", func() volumeTestShell {
			s := volumeTestCube()
			// The unchanged winding still has positive signed volume 1/4;
			// a signed-volume check alone would publish this crossed shell.
			s.world[6] = [3]*big.Rat{big.NewRat(1, 2), big.NewRat(1, 2), big.NewRat(-1, 2)}
			return s
		}()},
		{"one reversed facet", func() volumeTestShell {
			s := volumeTestTetra()
			s.triangles[0][1], s.triangles[0][2] = s.triangles[0][2], s.triangles[0][1]
			return s
		}()},
		{"globally reversed", func() volumeTestShell {
			s := volumeTestTetra()
			for i := range s.triangles {
				s.triangles[i][1], s.triangles[i][2] = s.triangles[i][2], s.triangles[i][1]
			}
			return s
		}()},
	} {
		t.Run(tc.name, func(t *testing.T) {
			volume, ok, err := volumeExactShell(proofbound.NewWorkBudget(t.Context()), tc.shell)
			require.NoError(t, err)
			require.False(t, ok)
			require.Nil(t, volume)
		})
	}
}

func TestLoftVolumeExactShellRefusesSeparateComponents(t *testing.T) {
	shell := volumeTestTetra()
	second := volumeTestTetra()
	for _, v := range second.world {
		v[0] = new(big.Rat).Add(v[0], big.NewRat(3, 1))
		shell.world = append(shell.world, v)
	}
	for _, tri := range second.triangles {
		for k := range tri {
			tri[k] += 4
		}
		shell.triangles = append(shell.triangles, tri)
	}
	volume, ok, err := volumeExactShell(proofbound.NewWorkBudget(t.Context()), shell)
	require.NoError(t, err)
	require.False(t, ok)
	require.Nil(t, volume)
}

func TestLoftVolumeExactShellRefusesOversizeAndCancellation(t *testing.T) {
	shell := volumeTestTetra()
	huge := new(big.Int).Lsh(big.NewInt(1), loftcontact.RationalBits)
	shell.world[0][0] = new(big.Rat).SetInt(huge)
	volume, ok, err := volumeExactShell(proofbound.NewWorkBudget(t.Context()), shell)
	require.NoError(t, err)
	require.False(t, ok)
	require.Nil(t, volume)

	shell = volumeTestTetra()
	denominator := new(big.Int).Lsh(big.NewInt(1), 5000)
	shell.world[1][0] = new(big.Rat).SetFrac(big.NewInt(1), denominator)
	volume, ok, err = volumeExactShell(proofbound.NewWorkBudget(t.Context()), shell)
	require.NoError(t, err)
	require.False(t, ok)
	require.Nil(t, volume)

	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	volume, ok, err = volumeExactShell(proofbound.NewWorkBudget(ctx), volumeTestTetra())
	require.ErrorIs(t, err, context.Canceled)
	require.False(t, ok)
	require.Nil(t, volume)

	steps := 0
	budget := &proofbound.WorkBudget{
		StepFn: func() error {
			steps++
			if steps == 35 {
				return context.Canceled
			}
			return nil
		},
		ErrFn: func() error { return nil },
	}
	volume, ok, err = volumeExactShell(budget, volumeTestCube())
	require.Equal(t, 35, steps)
	require.ErrorIs(t, err, context.Canceled)
	require.False(t, ok)
	require.Nil(t, volume)
}

func TestLoftVolumeExactShellRefusesNonadjacentCrossing(t *testing.T) {
	points := []loftcontact.Vec{
		{big.NewRat(0, 1), big.NewRat(0, 1), big.NewRat(0, 1)},
		{big.NewRat(2, 1), big.NewRat(0, 1), big.NewRat(0, 1)},
		{big.NewRat(0, 1), big.NewRat(2, 1), big.NewRat(0, 1)},
		{big.NewRat(1, 2), big.NewRat(1, 2), big.NewRat(-1, 1)},
		{big.NewRat(1, 2), big.NewRat(1, 2), big.NewRat(1, 1)},
		{big.NewRat(1, 2), big.NewRat(1, 1), big.NewRat(0, 1)},
	}
	c := &loftcontact.Arithmetic{OK: true}
	a := loftcontact.NewVolumeTriangle(c, points, [3]int{0, 1, 2})
	b := loftcontact.NewVolumeTriangle(c, points, [3]int{3, 4, 5})
	require.True(t, c.OK)
	allowed, err := loftcontact.VolumePairAllowed(proofbound.NewWorkBudget(t.Context()), c, points, a, b)
	require.NoError(t, err)
	require.False(t, allowed)
}

func TestLoftVolumeExactShellRefusesCoplanarOverlap(t *testing.T) {
	points := []loftcontact.Vec{
		{big.NewRat(0, 1), big.NewRat(0, 1), big.NewRat(0, 1)},
		{big.NewRat(2, 1), big.NewRat(0, 1), big.NewRat(0, 1)},
		{big.NewRat(0, 1), big.NewRat(2, 1), big.NewRat(0, 1)},
		{big.NewRat(1, 1), big.NewRat(1, 2), big.NewRat(0, 1)},
	}
	c := &loftcontact.Arithmetic{OK: true}
	// Both triangles face +z and share edge 0-1; vertex 3 is inside the first.
	a := loftcontact.NewVolumeTriangle(c, points, [3]int{0, 1, 2})
	b := loftcontact.NewVolumeTriangle(c, points, [3]int{0, 1, 3})
	require.True(t, c.OK)
	allowed, err := loftcontact.VolumePairAllowed(proofbound.NewWorkBudget(t.Context()), c, points, a, b)
	require.NoError(t, err)
	require.False(t, allowed)
}

func TestLoftVolumeEmbeddingRefusesNearCoplanarCrossing(t *testing.T) {
	zero, one, two := big.NewRat(0, 1), big.NewRat(1, 1), big.NewRat(2, 1)
	half := big.NewRat(1, 2)
	epsilon := new(big.Rat).SetFrac(big.NewInt(1), new(big.Int).Lsh(big.NewInt(1), 1000))
	below := new(big.Rat).Neg(epsilon)
	points := []loftcontact.Vec{
		{zero, zero, zero}, {two, zero, zero}, {zero, two, zero},
		{half, half, below}, {half, half, epsilon}, {half, one, epsilon},
	}

	// Both crossing points have z=0 at parameter 1/2 on the edges from
	// point 3. Their (x,y) coordinates are (1/2,1/2) and (1/2,3/4), strictly
	// inside x>0, y>0, x+y<2. The triangles have no common vertex ID.
	time := new(big.Rat).Quo(new(big.Rat).Neg(below), new(big.Rat).Sub(epsilon, below))
	require.Equal(t, "1/2", time.RatString())
	firstY := new(big.Rat).Add(half, new(big.Rat).Mul(time, new(big.Rat).Sub(half, half)))
	secondY := new(big.Rat).Add(half, new(big.Rat).Mul(time, new(big.Rat).Sub(one, half)))
	require.Equal(t, "1/2", firstY.RatString())
	require.Equal(t, "3/4", secondY.RatString())
	require.Negative(t, new(big.Rat).Add(half, secondY).Cmp(two))

	c := &loftcontact.Arithmetic{OK: true}
	a := loftcontact.NewVolumeTriangle(c, points, [3]int{0, 1, 2})
	b := loftcontact.NewVolumeTriangle(c, points, [3]int{3, 4, 5})
	allowed, err := loftcontact.VolumePairAllowed(proofbound.NewWorkBudget(t.Context()), c, points, a, b)
	require.NoError(t, err)
	require.True(t, c.OK)
	require.False(t, allowed)
}

func TestLoftVolumeEmbeddingRefusesOneVertexTJunction(t *testing.T) {
	zero, one, two := big.NewRat(0, 1), big.NewRat(1, 1), big.NewRat(2, 1)
	minusOne := big.NewRat(-1, 1)
	points := []loftcontact.Vec{
		{zero, zero, zero}, {two, zero, zero}, {zero, two, zero},
		{one, zero, zero}, {zero, minusOne, zero},
	}

	// The triangles share only vertex ID 0. Their coplanar edges from 0
	// overlap on 0<x<1; (1/2,0,0) is an exact contact beyond that vertex.
	witness := big.NewRat(1, 2)
	require.Positive(t, witness.Cmp(zero))
	require.Negative(t, witness.Cmp(one))
	require.Negative(t, witness.Cmp(two))

	c := &loftcontact.Arithmetic{OK: true}
	a := loftcontact.NewVolumeTriangle(c, points, [3]int{0, 1, 2})
	b := loftcontact.NewVolumeTriangle(c, points, [3]int{0, 3, 4})
	allowed, err := loftcontact.VolumePairAllowed(proofbound.NewWorkBudget(t.Context()), c, points, a, b)
	require.NoError(t, err)
	require.True(t, c.OK)
	require.False(t, allowed)
}
