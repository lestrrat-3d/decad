package cupwall_test

import (
	"errors"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/cupwall"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismplacement"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func squareOf(side float64) momentinput.Profile {
	corners := []sectionrecord.Point2{{}, {U: side}, {U: side, V: side}, {V: side}}
	var segs []sectionrecord.CurveSegment
	for i, c := range corners {
		segs = append(segs, sectionrecord.LineSeg{Start: c, End: corners[(i+1)%4], TStart: 0, TEnd: 1})
	}
	return momentinput.Profile{Outer: sectionrecord.LoopRecord{Segments: segs}}
}

func TestViewSameSet(t *testing.T) {
	t.Parallel()

	frame, err := r3.NewFrame(r3.Vec{}, r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	other, err := r3.NewFrame(r3.Vec{Z: 1}, r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	move, err := r3.Translation(r3.Vec{X: 1})
	require.NoError(t, err)
	base := func() cupwall.View {
		return cupwall.View{Frame: frame, Xform: r3.Identity(), ZOpen: 20, ZOuter: 0, ZCav: 5,
			Outer: squareOf(10), Cavity: squareOf(6)}
	}
	budget := func() *proofbound.WorkBudget { return proofbound.NewWorkBudget(t.Context()) }
	exact := new(big.Rat).Mul(new(big.Rat).SetFloat64(0.2), new(big.Rat).SetFloat64(25.4))
	held, _ := exact.Float64()
	for name, set := range map[string]func(*cupwall.View, float64, prismplacement.LevelDenotation){
		"open": func(p *cupwall.View, delta float64, d prismplacement.LevelDenotation) {
			p.ZOpen, p.ZOpenDelta, p.ZOpenDenoted, p.ZCav = held, delta, d, 1
		},
		"outer": func(p *cupwall.View, delta float64, d prismplacement.LevelDenotation) {
			p.ZOuter, p.ZOuterDelta, p.ZOuterDenoted, p.ZCav = held, delta, d, 10
		},
		"cavity": func(p *cupwall.View, delta float64, d prismplacement.LevelDenotation) {
			p.ZCav, p.ZCavDelta, p.ZCavDenoted = held, delta, d
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
					for _, pair := range [][2]cupwall.View{{a, b}, {b, a}} {
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

	for name, edit := range map[string]func(*cupwall.View){
		"frame":          func(v *cupwall.View) { v.Frame = other },
		"open":           func(v *cupwall.View) { v.ZOpen = 21 },
		"outer":          func(v *cupwall.View) { v.ZOuter = 1 },
		"cavity":         func(v *cupwall.View) { v.ZCav = 6 },
		"placement":      func(v *cupwall.View) { v.Xform = move },
		"offset":         func(v *cupwall.View) { v.OffsetDelta = 1e-3 },
		"outer profile":  func(v *cupwall.View) { v.Outer = squareOf(11) },
		"cavity profile": func(v *cupwall.View) { v.Cavity = squareOf(7) },
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
	displaced.OffsetDelta = 1e-3
	same, err = displaced.SameSet(budget(), displaced)
	require.NoError(t, err)
	require.False(t, same, "two equal displaced cups stay undecided")

	spent := errors.New("spent")
	_, err = base().SameSet(&proofbound.WorkBudget{StepFn: func() error { return spent }}, base())
	require.ErrorIs(t, err, spent)
}
