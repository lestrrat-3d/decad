package rimstrip

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

func TestPolarCylindricalSector(t *testing.T) {
	g := Polar{rat(0), rat(0), rat(25), rat(0), rat(1), rat(1), rat(1)}
	v := []r3.Vec{{X: 5}, {X: 5, Y: 1}, {Y: 1, Z: 5}, {Z: 5}}
	faces := [][3]int{{0, 1, 2}, {0, 2, 3}}
	for _, tc := range []struct {
		name string
		t    [][3]int
		ids  []int
		ok   bool
	}{
		{"complete", faces, []int{0, 1}, true},
		{"missing triangle", faces, []int{0}, false},
		{"reversed", [][3]int{{0, 2, 1}, {0, 2, 3}}, []int{0, 1}, false},
		{"multiple covering", faces, []int{0, 1, 0}, false},
	} {
		t.Run(tc.name, func(t *testing.T) {
			b, _, ok, e := g.Chart(t.Context(), v, tc.t, tc.ids, proofbound.NewStrictReplayBudget())
			if ok != tc.ok || e != nil {
				t.Fatalf("admitted %t error %v", ok, e)
			}
			if ok && (b[0] < 1.46 || b[0] > 1.47) {
				t.Fatalf("chord sagitta %v", b[0])
			}
		})
	}
}
