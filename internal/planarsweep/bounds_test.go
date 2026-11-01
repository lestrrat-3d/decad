package planarsweep_test

import (
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/motionbound"
	"github.com/lestrrat-3d/decad/internal/planarsweep"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/stretchr/testify/require"
)

const oneHalf = "1/2"

func TestRates(t *testing.T) {
	tests := []struct {
		name   string
		points []proofarith.DyV3
		normal proofarith.DyV3
		m, o   planarsweep.Motion
	}{
		{
			name: "both rotating about offset centers",
			points: []proofarith.DyV3{
				dv(0.5, -1.25, 2),
				dv(-2, 0.75, 1.5),
			},
			normal: dv(1, 2, -1),
			m: motion(
				[3]string{oneHalf, "-1", "3/4"},
				[3]string{oneHalf, "1", "-1/2"},
				[3]string{"2", "-1", oneHalf},
			),
			o: motion(
				[3]string{"-1/4", oneHalf, "0"},
				[3]string{"-1", oneHalf, "3/4"},
				[3]string{"-1/2", "2", "-1"},
			),
		},
		{
			name:   "translating owner",
			points: []proofarith.DyV3{dv(1.5, -0.5, 2.25), dv(-1, 3, 0.5)},
			normal: dv(2, -1, 3),
			m: motion(
				[3]string{"1", "-2", oneHalf},
				[3]string{"-1/2", "1/4", "1"},
				[3]string{oneHalf, "1", "-2"},
			),
			o: motion(
				[3]string{"-3/2", "1/4", "2"},
				[3]string{"0", "0", "0"},
				[3]string{"-2", oneHalf, "1"},
			),
		},
		{
			name:   "zero spin",
			points: []proofarith.DyV3{dv(-0.25, 0.5, 1.75), dv(2.5, -3, 0)},
			normal: dv(1, -2, 2),
			m: motion(
				[3]string{oneHalf, "-1/4", "2"},
				[3]string{"0", "0", "0"},
				[3]string{"3", "-2", oneHalf},
			),
			o: motion(
				[3]string{"-1", "3/2", "1/4"},
				[3]string{"0", "0", "0"},
				[3]string{"-1/2", "2", "-3"},
			),
		},
	}

	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			got := planarsweep.Rates(tt.points, tt.normal, tt.m, tt.o)
			require.Len(t, got, len(tt.points))
			for i, point := range tt.points {
				require.Zero(t, wantRate(point, tt.normal, tt.m, tt.o).Cmp(got[i]), "point %d", i)
			}
		})
	}
}

func dv(x, y, z float64) proofarith.DyV3 {
	return proofarith.DyV3{
		proofarith.MustDyOf(x),
		proofarith.MustDyOf(y),
		proofarith.MustDyOf(z),
	}
}

func motion(velocity, omega, center [3]string) planarsweep.Motion {
	return planarsweep.Motion{
		Velocity: ratVec(velocity),
		Omega:    ratVec(omega),
		Center:   ratVec(center),
	}
}

func ratVec(values [3]string) motionbound.RatVec {
	var out motionbound.RatVec
	for i, value := range values {
		out[i] = new(big.Rat)
		if _, ok := out[i].SetString(value); !ok {
			panic("invalid test rational: " + value)
		}
	}
	return out
}

func wantRate(point, normal proofarith.DyV3, m, o planarsweep.Motion) *big.Rat {
	p := ratOf(point)
	n := ratOf(normal)
	guest := add3(m.Velocity, cross3(m.Omega, sub3(p, m.Center)))
	owner := add3(o.Velocity, cross3(o.Omega, sub3(p, o.Center)))
	return dot3(n, sub3(guest, owner))
}

func ratOf(v proofarith.DyV3) motionbound.RatVec {
	return motionbound.RatVec{v[0].Rat(), v[1].Rat(), v[2].Rat()}
}

func add3(a, b motionbound.RatVec) motionbound.RatVec {
	return motionbound.RatVec{
		new(big.Rat).Add(a[0], b[0]),
		new(big.Rat).Add(a[1], b[1]),
		new(big.Rat).Add(a[2], b[2]),
	}
}

func sub3(a, b motionbound.RatVec) motionbound.RatVec {
	return motionbound.RatVec{
		new(big.Rat).Sub(a[0], b[0]),
		new(big.Rat).Sub(a[1], b[1]),
		new(big.Rat).Sub(a[2], b[2]),
	}
}

func cross3(a, b motionbound.RatVec) motionbound.RatVec {
	return motionbound.RatVec{
		new(big.Rat).Sub(new(big.Rat).Mul(a[1], b[2]), new(big.Rat).Mul(a[2], b[1])),
		new(big.Rat).Sub(new(big.Rat).Mul(a[2], b[0]), new(big.Rat).Mul(a[0], b[2])),
		new(big.Rat).Sub(new(big.Rat).Mul(a[0], b[1]), new(big.Rat).Mul(a[1], b[0])),
	}
}

func dot3(a, b motionbound.RatVec) *big.Rat {
	return new(big.Rat).Add(
		new(big.Rat).Add(new(big.Rat).Mul(a[0], b[0]), new(big.Rat).Mul(a[1], b[1])),
		new(big.Rat).Mul(a[2], b[2]),
	)
}
