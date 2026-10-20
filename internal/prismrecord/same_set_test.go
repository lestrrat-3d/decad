package prismrecord_test

import (
	"errors"
	"testing"

	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func squareProfile(side float64) momentinput.Profile {
	corners := []sectionrecord.Point2{{}, {U: side}, {U: side, V: side}, {V: side}}
	var segs []sectionrecord.CurveSegment
	for i, c := range corners {
		segs = append(segs, sectionrecord.LineSeg{Start: c, End: corners[(i+1)%4], TStart: 0, TEnd: 1})
	}
	return momentinput.Profile{Outer: sectionrecord.LoopRecord{Segments: segs}}
}

func TestSameSet(t *testing.T) {
	t.Parallel()

	frame, err := r3.NewFrame(r3.Vec{}, r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	other, err := r3.NewFrame(r3.Vec{Z: 1}, r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	move, err := r3.Translation(r3.Vec{X: 1})
	require.NoError(t, err)
	base := func() prismrecord.Payload {
		return prismrecord.Payload{Frame: frame, Xform: r3.Identity(), Z0: 0, Z1: 4, Profile: squareProfile(2)}
	}
	budget := func() *proofbound.WorkBudget { return proofbound.NewWorkBudget(t.Context()) }

	same, err := base().SameSet(budget(), base())
	require.NoError(t, err)
	require.True(t, same)

	for name, edit := range map[string]func(*prismrecord.Payload){
		"frame":               func(p *prismrecord.Payload) { p.Frame = other },
		"start level":         func(p *prismrecord.Payload) { p.Z0 = 1 },
		"end level":           func(p *prismrecord.Payload) { p.Z1 = 5 },
		"placement":           func(p *prismrecord.Payload) { p.Xform = move },
		"profile":             func(p *prismrecord.Payload) { p.Profile = squareProfile(3) },
		"section on the left": func(p *prismrecord.Payload) { p.SectionDelta = 1e-3 },
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
	b := &proofbound.WorkBudget{StepFn: func() error { return spent }}
	_, err = base().SameSet(b, base())
	require.ErrorIs(t, err, spent)
}
