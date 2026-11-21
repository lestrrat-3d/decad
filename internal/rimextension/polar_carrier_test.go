package rimextension

import (
	"math/big"
	"testing"
)

func TestOpenArcCarrier(t *testing.T) {
	r := func(a int64) *big.Rat { return big.NewRat(a, 1) }
	rays := [2][2]*big.Rat{{r(5), r(0)}, {r(0), r(5)}}
	for _, tc := range []struct {
		name    string
		x, z, h int64
		away    bool
	}{
		{"endpoint tangent", 1, 0, 5, true}, {"interior crossing", 1, 0, 4, false},
		{"interior tangent", 3, 4, 25, false}, {"remote line", 3, 4, 26, true},
		{"diagonal chord", 1, 1, 5, true}, {"crosses twice", 1, 1, 6, false},
		{"reverse tangent", -3, -4, -25, false}, {"outside opposite half", -1, -1, 5, true},
	} {
		t.Run(tc.name, func(t *testing.T) {
			got := lineOffOpenArc(r(25), rays, r(tc.x), r(tc.z), r(tc.h))
			if got != tc.away {
				t.Fatalf("separated=%t", got)
			}
		})
	}
}
func TestArcEndClosure(t *testing.T) {
	r := func(x int64) *big.Rat { return big.NewRat(x, 1) }
	for _, tc := range []struct {
		name string
		x, z int64
		want float64
		ok   bool
	}{{"no residual", 5, 0, 0, true}, {"radial gap", 6, 0, 1, true}, {"nonradial", 5, 1, 0, false}, {"opposite ray", -5, 0, 0, false}} {
		t.Run(tc.name, func(t *testing.T) {
			gap, ok := arcEndClosure(r(25), r(5), r(0), r(tc.x), r(tc.z))
			if ok != tc.ok || ok && gap != tc.want {
				t.Fatalf("gap %g admitted %t", gap, ok)
			}
		})
	}
}
