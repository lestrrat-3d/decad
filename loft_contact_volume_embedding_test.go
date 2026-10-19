package decad

import (
	"context"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/loftcontact"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/stretchr/testify/require"
)

func loftVolumeTestPoint(x, y, z int64) loftOutputVertex {
	return loftOutputVertex{world: [3]*big.Rat{
		big.NewRat(x, 1), big.NewRat(y, 1), big.NewRat(z, 1),
	}}
}

func loftVolumeTestTetra() loftOutputShell {
	return loftOutputShell{
		vertices: []loftOutputVertex{
			loftVolumeTestPoint(0, 0, 0), loftVolumeTestPoint(1, 0, 0),
			loftVolumeTestPoint(0, 1, 0), loftVolumeTestPoint(0, 0, 1),
		},
		triangles: [][3]int{{0, 2, 1}, {0, 1, 3}, {0, 3, 2}, {1, 2, 3}},
	}
}

func loftVolumeTestCube() loftOutputShell {
	return loftOutputShell{
		vertices: []loftOutputVertex{
			loftVolumeTestPoint(0, 0, 0), loftVolumeTestPoint(1, 0, 0),
			loftVolumeTestPoint(1, 1, 0), loftVolumeTestPoint(0, 1, 0),
			loftVolumeTestPoint(0, 0, 1), loftVolumeTestPoint(1, 0, 1),
			loftVolumeTestPoint(1, 1, 1), loftVolumeTestPoint(0, 1, 1),
		},
		triangles: [][3]int{
			{0, 2, 1}, {0, 3, 2}, {4, 5, 6}, {4, 6, 7},
			{0, 1, 5}, {0, 5, 4}, {1, 2, 6}, {1, 6, 5},
			{2, 3, 7}, {2, 7, 6}, {3, 0, 4}, {3, 4, 7},
		},
	}
}

func TestLoftVolumeExactShellEmbeddedPositive(t *testing.T) {
	for _, tc := range []struct {
		name  string
		shell loftOutputShell
		want  *big.Rat
	}{
		{"tetra", loftVolumeTestTetra(), big.NewRat(1, 6)},
		{"triangulated cube", loftVolumeTestCube(), big.NewRat(1, 1)},
	} {
		t.Run(tc.name, func(t *testing.T) {
			volume, ok, err := loftVolumeExactShell(proofbound.NewWorkBudget(t.Context()), tc.shell)
			require.NoError(t, err)
			require.True(t, ok)
			require.Zero(t, volume.Cmp(tc.want))
		})
	}
}

func TestLoftVolumeExactShellRealVaryingTab(t *testing.T) {
	prism, loft := loftPartitionRealBodies(t,
		[4]float64{8, 4, 12, 6}, [4]float64{8, 4, 13, 6}, 0, 4)
	for _, pair := range [][2]*Body{{prism, loft}, {loft, prism}} {
		operation := proofbound.NewWorkBudget(t.Context())
		proof, admitted, err := certifyLoftOutputBoundary(t.Context(), pair[0], pair[1], operation)
		require.NoError(t, err)
		require.True(t, admitted)
		require.Less(t, proof.workVisits, uint64(65_536))
		budget := proofbound.NewWallWorkBudgetWithOperation(65_536-proof.workVisits, operation)
		volume, ok, err := loftVolumeExactShell(budget, proof.shell)
		require.NoError(t, err)
		require.True(t, ok)
		require.Zero(t, volume.Cmp(big.NewRat(520, 1)))
	}
}

func TestLoftVolumeExactShellRefusesCrossingAndReversal(t *testing.T) {
	for _, tc := range []struct {
		name  string
		shell loftOutputShell
	}{
		{"crossing", func() loftOutputShell {
			s := loftVolumeTestCube()
			// The unchanged winding still has positive signed volume 1/4;
			// a signed-volume check alone would publish this crossed shell.
			s.vertices[6] = loftOutputVertex{world: [3]*big.Rat{
				big.NewRat(1, 2), big.NewRat(1, 2), big.NewRat(-1, 2),
			}}
			return s
		}()},
		{"one reversed facet", func() loftOutputShell {
			s := loftVolumeTestTetra()
			s.triangles[0][1], s.triangles[0][2] = s.triangles[0][2], s.triangles[0][1]
			return s
		}()},
		{"globally reversed", func() loftOutputShell {
			s := loftVolumeTestTetra()
			for i := range s.triangles {
				s.triangles[i][1], s.triangles[i][2] = s.triangles[i][2], s.triangles[i][1]
			}
			return s
		}()},
	} {
		t.Run(tc.name, func(t *testing.T) {
			volume, ok, err := loftVolumeExactShell(proofbound.NewWorkBudget(t.Context()), tc.shell)
			require.NoError(t, err)
			require.False(t, ok)
			require.Nil(t, volume)
		})
	}
}

func TestLoftVolumeExactShellRefusesSeparateComponents(t *testing.T) {
	shell := loftVolumeTestTetra()
	second := loftVolumeTestTetra()
	for _, v := range second.vertices {
		v.world[0] = new(big.Rat).Add(v.world[0], big.NewRat(3, 1))
		shell.vertices = append(shell.vertices, v)
	}
	for _, tri := range second.triangles {
		for k := range tri {
			tri[k] += 4
		}
		shell.triangles = append(shell.triangles, tri)
	}
	volume, ok, err := loftVolumeExactShell(proofbound.NewWorkBudget(t.Context()), shell)
	require.NoError(t, err)
	require.False(t, ok)
	require.Nil(t, volume)
}

func TestLoftVolumeExactShellRefusesOversizeAndCancellation(t *testing.T) {
	shell := loftVolumeTestTetra()
	huge := new(big.Int).Lsh(big.NewInt(1), loftcontact.RationalBits)
	shell.vertices[0].world[0] = new(big.Rat).SetInt(huge)
	volume, ok, err := loftVolumeExactShell(proofbound.NewWorkBudget(t.Context()), shell)
	require.NoError(t, err)
	require.False(t, ok)
	require.Nil(t, volume)

	shell = loftVolumeTestTetra()
	denominator := new(big.Int).Lsh(big.NewInt(1), 5000)
	shell.vertices[1].world[0] = new(big.Rat).SetFrac(big.NewInt(1), denominator)
	volume, ok, err = loftVolumeExactShell(proofbound.NewWorkBudget(t.Context()), shell)
	require.NoError(t, err)
	require.False(t, ok)
	require.Nil(t, volume)

	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	volume, ok, err = loftVolumeExactShell(proofbound.NewWorkBudget(ctx), loftVolumeTestTetra())
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
	volume, ok, err = loftVolumeExactShell(budget, loftVolumeTestCube())
	require.Equal(t, 35, steps)
	require.ErrorIs(t, err, context.Canceled)
	require.False(t, ok)
	require.Nil(t, volume)
}

func TestLoftVolumeExactShellRefusesNonadjacentCrossing(t *testing.T) {
	points := []loftcontact.Vec{
		{loftOutputRat(0), loftOutputRat(0), loftOutputRat(0)},
		{loftOutputRat(2), loftOutputRat(0), loftOutputRat(0)},
		{loftOutputRat(0), loftOutputRat(2), loftOutputRat(0)},
		{big.NewRat(1, 2), big.NewRat(1, 2), loftOutputRat(-1)},
		{big.NewRat(1, 2), big.NewRat(1, 2), loftOutputRat(1)},
		{big.NewRat(1, 2), loftOutputRat(1), loftOutputRat(0)},
	}
	c := &loftcontact.Arithmetic{OK: true}
	makeTriangle := func(ids [3]int) loftVolumeTriangle {
		a, b, d := points[ids[0]], points[ids[1]], points[ids[2]]
		tri := loftVolumeTriangle{ids: ids, normal: c.Cross(c.Diff(b, a), c.Diff(d, a))}
		for axis := range 3 {
			tri.minimum[axis], tri.maximum[axis] = a[axis], a[axis]
			for _, p := range []loftcontact.Vec{b, d} {
				if c.Cmp(p[axis], tri.minimum[axis]) < 0 {
					tri.minimum[axis] = p[axis]
				}
				if c.Cmp(p[axis], tri.maximum[axis]) > 0 {
					tri.maximum[axis] = p[axis]
				}
			}
		}
		return tri
	}
	a, b := makeTriangle([3]int{0, 1, 2}), makeTriangle([3]int{3, 4, 5})
	require.True(t, c.OK)
	allowed, err := loftVolumePairAllowed(proofbound.NewWorkBudget(t.Context()), c, points, a, b)
	require.NoError(t, err)
	require.False(t, allowed)
}

func TestLoftVolumeExactShellRefusesCoplanarOverlap(t *testing.T) {
	points := []loftcontact.Vec{
		{loftOutputRat(0), loftOutputRat(0), loftOutputRat(0)},
		{loftOutputRat(2), loftOutputRat(0), loftOutputRat(0)},
		{loftOutputRat(0), loftOutputRat(2), loftOutputRat(0)},
		{loftOutputRat(1), big.NewRat(1, 2), loftOutputRat(0)},
	}
	c := &loftcontact.Arithmetic{OK: true}
	a := loftVolumeTriangle{ids: [3]int{0, 1, 2},
		normal: loftcontact.Vec{loftOutputRat(0), loftOutputRat(0), loftOutputRat(1)}}
	b := loftVolumeTriangle{ids: [3]int{0, 1, 3}, normal: a.normal}
	for axis := range 3 {
		a.minimum[axis], a.maximum[axis] = loftOutputRat(0), loftOutputRat(2)
		b.minimum[axis], b.maximum[axis] = loftOutputRat(0), loftOutputRat(2)
	}
	allowed, err := loftVolumePairAllowed(proofbound.NewWorkBudget(t.Context()), c, points, a, b)
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

	a := loftVolumeTriangle{ids: [3]int{0, 1, 2},
		normal:  loftcontact.Vec{zero, zero, one},
		minimum: loftcontact.Vec{zero, zero, zero}, maximum: loftcontact.Vec{two, two, zero}}
	b := loftVolumeTriangle{ids: [3]int{3, 4, 5},
		normal:  loftcontact.Vec{one, zero, zero},
		minimum: loftcontact.Vec{half, half, below}, maximum: loftcontact.Vec{half, one, epsilon}}
	c := &loftcontact.Arithmetic{OK: true}
	allowed, err := loftVolumePairAllowed(proofbound.NewWorkBudget(t.Context()), c, points, a, b)
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

	a := loftVolumeTriangle{ids: [3]int{0, 1, 2},
		normal:  loftcontact.Vec{zero, zero, one},
		minimum: loftcontact.Vec{zero, zero, zero}, maximum: loftcontact.Vec{two, two, zero}}
	b := loftVolumeTriangle{ids: [3]int{0, 3, 4},
		normal:  loftcontact.Vec{zero, zero, minusOne},
		minimum: loftcontact.Vec{zero, minusOne, zero}, maximum: loftcontact.Vec{one, zero, zero}}
	c := &loftcontact.Arithmetic{OK: true}
	allowed, err := loftVolumePairAllowed(proofbound.NewWorkBudget(t.Context()), c, points, a, b)
	require.NoError(t, err)
	require.True(t, c.OK)
	require.False(t, allowed)
}
