package verifyrun

import (
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/cupwall"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismplacement"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestAnalyticBodiesEqualDenotedLevels(t *testing.T) {
	square := func(lo, hi float64) momentinput.Profile {
		corners := []sectionrecord.Point2{{U: lo, V: lo}, {U: hi, V: lo}, {U: hi, V: hi}, {U: lo, V: hi}}
		var segments []sectionrecord.CurveSegment
		for i, p := range corners {
			segments = append(segments, sectionrecord.LineSeg{Start: p, End: corners[(i+1)%4], TEnd: 1})
		}
		return momentinput.Profile{Outer: sectionrecord.LoopRecord{Segments: segments}}
	}
	frame, err := r3.NewFrame(r3.Vec{}, r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	exact := new(big.Rat).Sub(big.NewRat(4, 1), big.NewRat(1, 10000000000000000))
	d := prismplacement.DenoteLevel(exact, 4, 1e-15)
	prism := prismrecord.Payload{Frame: frame, Xform: r3.Identity(), Z0: 0, Z1: 4, Profile: square(0, 10)}
	cup := cupwall.View{Frame: frame, Xform: r3.Identity(), ZOuter: 0, ZCav: 1, ZOpen: 4, Outer: square(0, 10), Cavity: square(2, 8)}
	for _, kind := range []string{"prism", "cup"} {
		t.Run(kind, func(t *testing.T) {
			var a, b topo.Body
			if kind == "prism" {
				a.Payload = prism
				b.Payload = prism
			} else {
				payload, err := cup.Payload(t.Context())
				require.NoError(t, err)
				a.Payload = payload
				b.Payload = payload
			}
			same, err := analyticBodiesEqual(proofbound.NewWorkBudget(t.Context()), &a, &b)
			require.NoError(t, err)
			require.True(t, same)
			if kind == "prism" {
				changed := prism
				changed.Z1Delta, changed.Z1Denoted = 1e-15, d
				b.Payload = changed
			} else {
				changed := cup
				changed.ZOpenDelta, changed.ZOpenDenoted = 1e-15, d
				payload, err := changed.Payload(t.Context())
				require.NoError(t, err)
				b.Payload = payload
			}
			same, err = analyticBodiesEqual(proofbound.NewWorkBudget(t.Context()), &a, &b)
			require.NoError(t, err)
			require.False(t, same)
		})
	}
}
