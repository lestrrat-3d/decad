package rimstrip

import (
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

func TestPolarDomainCover(t *testing.T) {
	var g PolarDomain
	g.Polar = Polar{CenterX: rat(0), CenterZ: rat(0), RadiusSquared: rat(25), Bottom: rat(-1), Top: rat(1)}
	g.Rays[0] = [2]*big.Rat{rat(5), rat(0)}
	g.Rays[1] = [2]*big.Rat{rat(0), rat(5)}
	for _, tc := range []struct {
		name         string
		p            r3.Vec
		b            float64
		away, inside bool
	}{
		{"same arc", r3.NewVec(3, 0, 4), .2, false, true},
		{"intersects top", r3.NewVec(3, 1.25, 4), .25, false, false},
		{"above", r3.NewVec(3, 1.25, 4), .125, true, false},
		{"inner disk", r3.NewVec(.5, 0, .5), .25, true, true},
		{"radial touch", r3.NewVec(4, 0, 0), 1, false, false},
		{"outside radius", r3.NewVec(6, 0, 1), .25, true, true},
		{"past angular end", r3.NewVec(-.5, 0, 5), .25, true, false},
	} {
		t.Run(tc.name, func(t *testing.T) {
			tri := [3]r3.Vec{tc.p, tc.p, tc.p}
			away, err := g.Away(t.Context(), tri, tc.b, proofbound.NewStrictReplayBudget())
			if err != nil || away != tc.away {
				t.Fatalf("away=%t err=%v", away, err)
			}
			if got := g.Witness(tri, tc.b); got != tc.inside {
				t.Fatalf("inside angular/height interval=%t", got)
			}
		})
	}
}

func TestPolarDomainInsetKeepsOriginalUncertainty(t *testing.T) {
	g := PolarDomain{}
	g.CenterX, g.CenterZ = rat(0), rat(0)
	g.Rays = [2][2]*big.Rat{{rat(5), rat(0)}, {rat(0), rat(5)}}
	core, ok := g.InsetEnd(1)
	if !ok {
		t.Fatal("quarter circle lost its interior")
	}
	p := r3.NewVec(.125, 0, 5)
	tri := [3]r3.Vec{p, p, p}
	if !core.InsideAngularBoundary(tri, .12, 1) {
		t.Fatal("strictly separated correspondence was refused")
	}
	if core.InsideAngularBoundary(tri, .125, 1) {
		t.Fatal("original uncertainty reaches the omitted strip")
	}
	if core.InsideAngularBoundary([3]r3.Vec{p, p, r3.NewVec(0, 0, 5)}, 0, 1) {
		t.Fatal("the chord crosses the angular guard")
	}
}
