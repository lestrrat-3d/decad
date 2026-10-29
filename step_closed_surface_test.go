package decad

import (
	"context"
	"testing"

	"github.com/lestrrat-3d/decad/internal/closedstep"
	"github.com/lestrrat-3d/decad/internal/revolveplan"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/stretchr/testify/require"
)

func TestClosedSphereCoverage(t *testing.T) {
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	o := s.CreatePoint(0, 0)
	s.Fix(o)
	e := s.CreatePoint(10, 0)
	c := s.CreatePoint(5, 0)
	s.CreateLine(o, e)
	s.CreateArc(c, e, o)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	b, err := Revolve(t.Context(), New(), s, s.Profiles()[0], ConstructionAxis{Origin: r3.Vec{}, Dir: r3.NewVec(1, 0, 0)}, FullRevolution{})
	require.NoError(t, err)
	f := b.Faces()[0]
	got, ok, err := closedstep.Read(t.Context(), b, f)
	require.NoError(t, err)
	require.True(t, ok)
	require.Equal(t, r3.NewVec(5, 0, 0), got.Center)
	require.Equal(t, 5.0, got.Radius)
	_, ok, err = closedstep.Read(t.Context(), b, bornFace(&Face{}))
	require.NoError(t, err)
	require.False(t, ok)
	_, ok, err = closedstep.Read(t.Context(), b, nil)
	require.NoError(t, err)
	require.False(t, ok)
	_, ok, err = closedstep.Read(t.Context(), nil, f)
	require.NoError(t, err)
	require.False(t, ok)
	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	_, ok, err = closedstep.Read(ctx, b, f)
	require.ErrorIs(t, err, context.Canceled)
	require.False(t, ok)
	rp := b.n.Payload.(revolveplan.Payload)
	mutations := []func(*revolveplan.Payload){
		func(p *revolveplan.Payload) { p.SectionWhole = true },
		func(p *revolveplan.Payload) { p.Full = false },
		func(p *revolveplan.Payload) { p.Den.Phi1.Turn = nil },
		func(p *revolveplan.Payload) { p.Ax.RadialAdmitAllow = 1 },
		func(p *revolveplan.Payload) { p.Xform, _ = r3.Translation(r3.NewVec(1, 0, 0)) },
	}
	for _, mutate := range mutations {
		changed := rp
		mutate(&changed)
		b.n.Payload = changed
		_, ok, err = closedstep.Read(t.Context(), b, f)
		require.NoError(t, err)
		require.False(t, ok)
	}
	b.n.Payload = rp
}

func TestClosedRingCoverage(t *testing.T) {
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	c := s.CreatePoint(0, 10)
	s.Fix(c)
	s.CreateCircle(c, 3)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	b, err := Revolve(t.Context(), New(), s, s.Profiles()[0], ConstructionAxis{Origin: r3.Vec{}, Dir: r3.NewVec(1, 0, 0)}, FullRevolution{})
	require.NoError(t, err)
	f := b.Faces()[0]
	got, ok, err := closedstep.Read(t.Context(), b, f)
	require.NoError(t, err)
	require.True(t, ok)
	require.True(t, got.Ring)
	require.Equal(t, 10.0, got.Major)
	require.Equal(t, 3.0, got.Minor)
	require.Equal(t, r3.NewVec(1, 0, 0), got.Axis)
	rp := b.n.Payload.(revolveplan.Payload)
	rotated, err := r3.NewFrame(r3.Vec{}, r3.NewVec(1, 1, 0), r3.NewVec(-1, 1, 0))
	require.NoError(t, err)
	skewed := rp
	skewed.Frame = rotated
	_, ok, err = skewed.CompleteRing(t.Context())
	require.NoError(t, err)
	require.False(t, ok)
	changed := rp
	changed.Full = false
	b.n.Payload = changed
	_, ok, err = closedstep.Read(t.Context(), b, f)
	require.NoError(t, err)
	require.False(t, ok)
	b.n.Payload = rp
	old := f.n.Surface
	f.n.Surface = nil
	_, ok, err = closedstep.Read(t.Context(), b, f)
	require.NoError(t, err)
	require.False(t, ok)
	f.n.Surface = old
}
