package flushface_test

import (
	"context"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/flushface"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// square returns the outline of the square [x0, x0+2] × [z0, z0+2] in the plane y = 0, every corner bounded by r.
func square(x0, z0, r float64, line bool) flushface.Outline {
	const s = 2
	p := []r3.Vec{{X: x0, Z: z0}, {X: x0 + s, Z: z0}, {X: x0 + s, Z: z0 + s}, {X: x0, Z: z0 + s}}
	var o flushface.Outline
	o.Axis = 1
	for i := range p {
		o.Edges = append(o.Edges, flushface.Edge{A: p[i], B: p[(i+1)%4], RA: r, RB: r, Line: line, Straight: true})
	}
	return o
}

func budget(t *testing.T) *proofbound.WorkBudget {
	t.Helper()
	return proofbound.NewWorkBudget(t.Context())
}

func TestClearFarParts(t *testing.T) {
	t.Parallel()
	v, err := flushface.Clear(budget(t), square(0, 0, 1e-3, false), square(3, 0, 1e-3, false))
	require.NoError(t, err)
	require.True(t, v.Clear)
	require.Zero(t, v.Cross)
}

func TestClearNearVertex(t *testing.T) {
	t.Parallel()
	exact := square(0, 0, 0, true)
	// A vertex bounded by 1e-14 lying on the exact square's edge x = 2.
	near := flushface.Outline{Axis: 1, Edges: []flushface.Edge{
		{A: r3.Vec{X: 2, Z: 1}, B: r3.Vec{X: 3, Z: 1}, RA: 1e-14, Straight: true},
		{A: r3.Vec{X: 3, Z: 1}, B: r3.Vec{X: 3, Z: 2}, Straight: true, Line: true},
		{A: r3.Vec{X: 3, Z: 2}, B: r3.Vec{X: 2, Z: 1}, RB: 1e-14, Straight: true},
	}}
	v, err := flushface.Clear(budget(t), exact, near)
	require.NoError(t, err)
	require.False(t, v.Clear, "a moving vertex on the other outline")

	// The same triangle moved past the pad.
	for i := range near.Edges {
		near.Edges[i].A.X += 1e-9
		near.Edges[i].B.X += 1e-9
	}
	v, err = flushface.Clear(budget(t), exact, near)
	require.NoError(t, err)
	require.True(t, v.Clear)
}

// cross is one straight edge from (1, -1) to (1, 3), exact at its start and bounded by r at its end, against the
// exact square [0, 2]² whose edge z = 2 it crosses at right angles.
func crossing(r float64, straight bool) (flushface.Outline, flushface.Outline) {
	s := flushface.Outline{Axis: 1, Edges: []flushface.Edge{
		{A: r3.Vec{X: 1, Z: -1}, B: r3.Vec{X: 1, Z: 3}, RB: r, Straight: straight},
	}}
	return square(0, 0, 0, true), s
}

func TestClearRobustCrossing(t *testing.T) {
	t.Parallel()
	a, b := crossing(3e-4, true)
	v, err := flushface.Clear(budget(t), a, b)
	require.NoError(t, err)
	require.True(t, v.Clear)
	require.Positive(t, v.Cross)
	require.LessOrEqual(t, v.Cross, 3e-4*math.Sqrt(3)*(1+1e-9))
}

func TestClearFragileCrossing(t *testing.T) {
	t.Parallel()
	// A curved rim's chord may not cross.
	a, b := crossing(3e-4, false)
	v, err := flushface.Clear(budget(t), a, b)
	require.NoError(t, err)
	require.False(t, v.Clear)

	// A crossing at so shallow an angle that the end's bound can undo it: the edge ends 1e-4 past z = 2.
	s := flushface.Outline{Axis: 1, Edges: []flushface.Edge{
		{A: r3.Vec{X: 0.5, Z: 2 - 1e-4}, B: r3.Vec{X: 1.5, Z: 2 + 1e-4}, RA: 3e-4, RB: 3e-4, Straight: true},
	}}
	v, err = flushface.Clear(budget(t), square(0, 0, 0, true), s)
	require.NoError(t, err)
	require.False(t, v.Clear)
}

func TestClearNoMovingParts(t *testing.T) {
	t.Parallel()
	v, err := flushface.Clear(budget(t), square(0, 0, 0, true), square(1, 1, 0, true))
	require.NoError(t, err)
	require.True(t, v.Clear)
	require.Zero(t, v.Cross)
}

func TestClearNonFiniteBound(t *testing.T) {
	t.Parallel()
	v, err := flushface.Clear(budget(t), square(0, 0, math.NaN(), false), square(5, 0, 0, true))
	require.NoError(t, err)
	require.False(t, v.Clear)
}

func TestClearBudget(t *testing.T) {
	t.Parallel()
	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	b := proofbound.NewWorkBudget(ctx)
	var err error
	for range 100000 {
		if _, err = flushface.Clear(b, square(0, 0, 1e-3, false), square(3, 0, 1e-3, false)); err != nil {
			break
		}
	}
	require.ErrorIs(t, err, context.Canceled)
}

func BenchmarkClear(b *testing.B) {
	ring := func(cx float64, r float64) flushface.Outline {
		var o flushface.Outline
		o.Axis = 1
		const n = 1000
		for i := range n {
			a0, a1 := 2*math.Pi*float64(i)/n, 2*math.Pi*float64(i+1)/n
			o.Edges = append(o.Edges, flushface.Edge{
				A: r3.Vec{X: cx + 10*math.Cos(a0), Z: 10 * math.Sin(a0)}, B: r3.Vec{X: cx + 10*math.Cos(a1), Z: 10 * math.Sin(a1)},
				RA: r, RB: r, Straight: true,
			})
		}
		return o
	}
	x, y := ring(0, 1e-6), ring(25, 1e-6)
	for b.Loop() {
		if _, err := flushface.Clear(proofbound.NewWorkBudget(b.Context()), x, y); err != nil {
			b.Fatal(err)
		}
	}
}
