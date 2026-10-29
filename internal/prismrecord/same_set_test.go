package prismrecord_test

import (
	"errors"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismplacement"
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
	exact := new(big.Rat).Mul(new(big.Rat).SetFloat64(0.2), new(big.Rat).SetFloat64(25.4))
	held, _ := exact.Float64()
	for name, set := range map[string]func(*prismrecord.Payload, float64, prismplacement.LevelDenotation){
		"start": func(p *prismrecord.Payload, delta float64, d prismplacement.LevelDenotation) {
			p.Z0, p.Z0Delta, p.Z0Denoted, p.Z1 = held, delta, d, 8
		},
		"end": func(p *prismrecord.Payload, delta float64, d prismplacement.LevelDenotation) {
			p.Z1, p.Z1Delta, p.Z1Denoted = held, delta, d
		},
	} {
		t.Run(name, func(t *testing.T) {
			a := base()
			set(&a, 1e-15, prismplacement.DenoteLevel(exact, held, 1e-15))
			for _, tc := range []struct {
				name    string
				delta   float64
				denoted prismplacement.LevelDenotation
				want    bool
			}{
				{"equal", 2e-15, prismplacement.DenoteLevel(exact, held, 2e-15), true},
				{"unequal", 0, prismplacement.LevelDenotation{}, false},
				{"unknown", 1e-15, prismplacement.LevelDenotation{}, false},
				{"stale", 2e-15, prismplacement.DenoteLevel(exact, held, 1e-15), false},
			} {
				t.Run(tc.name, func(t *testing.T) {
					b := base()
					set(&b, tc.delta, tc.denoted)
					for _, pair := range [][2]prismrecord.Payload{{a, b}, {b, a}} {
						same, err := pair[0].SameSet(budget(), pair[1])
						require.NoError(t, err)
						require.Equal(t, tc.want, same)
					}
				})
			}
		})
	}

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
