package revolveplan_test

import (
	"errors"
	"testing"

	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/revolveplan"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func meridian(radius float64) momentinput.Profile {
	corners := []sectionrecord.Point2{{U: 1}, {U: 3}, {U: 3, V: radius}, {U: 1, V: radius}}
	var segs []sectionrecord.CurveSegment
	for i, c := range corners {
		segs = append(segs, sectionrecord.LineSeg{Start: c, End: corners[(i+1)%4], TStart: 0, TEnd: 1})
	}
	return momentinput.Profile{Outer: sectionrecord.LoopRecord{Segments: segs}}
}

func TestPayloadSameSet(t *testing.T) {
	t.Parallel()

	frame, err := r3.NewFrame(r3.Vec{}, r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	other, err := r3.NewFrame(r3.Vec{Z: 1}, r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	move, err := r3.Translation(r3.Vec{X: 1})
	require.NoError(t, err)
	base := func() revolveplan.Payload {
		return revolveplan.Payload{Frame: frame, Xform: r3.Identity(), Phi1: 3, Full: true, Profile: meridian(2)}
	}
	budget := func() *proofbound.WorkBudget { return proofbound.NewWorkBudget(t.Context()) }

	same, err := base().SameSet(budget(), base())
	require.NoError(t, err)
	require.True(t, same)

	for name, edit := range map[string]func(*revolveplan.Payload){
		"frame":     func(p *revolveplan.Payload) { p.Frame = other },
		"axis":      func(p *revolveplan.Payload) { p.Ax.AU = 1 },
		"start":     func(p *revolveplan.Payload) { p.Phi0 = 0.5 },
		"end":       func(p *revolveplan.Payload) { p.Phi1 = 4 },
		"full turn": func(p *revolveplan.Payload) { p.Full = false },
		"placement": func(p *revolveplan.Payload) { p.Xform = move },
		"profile":   func(p *revolveplan.Payload) { p.Profile = meridian(3) },
		"section":   func(p *revolveplan.Payload) { p.SectionDelta = 1e-3 },
	} {
		changed := base()
		edit(&changed)
		same, err := base().SameSet(budget(), changed)
		require.NoError(t, err, name)
		require.False(t, same, name)
		same, err = changed.SameSet(budget(), base())
		require.NoError(t, err, name)
		require.False(t, same, name)
	}

	displaced := base()
	displaced.SectionDelta = 1e-3
	same, err = displaced.SameSet(budget(), displaced)
	require.NoError(t, err)
	require.False(t, same, "two equal displaced records stay undecided")

	spent := errors.New("spent")
	_, err = base().SameSet(&proofbound.WorkBudget{StepFn: func() error { return spent }}, base())
	require.ErrorIs(t, err, spent)
}
