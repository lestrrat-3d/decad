package rimextension

// CI: isolated-tests

import (
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

func TestCloseDiskGenerator(t *testing.T) {
	for _, tc := range []struct {
		name       string
		mid        r3.Vec
		registered bool
		want       bool
	}{
		{"interior", r3.Vec{Y: 1}, true, true}, {"foreign", r3.Vec{Y: 1}, false, false},
		{"off cylinder generator", r3.Vec{Y: 1, Z: 1e-15}, true, false}, {"beyond endpoint", r3.Vec{Y: 3}, true, false},
	} {
		t.Run(tc.name, func(t *testing.T) {
			m := rewrite{ctx: t.Context(), budget: proofbound.NewStrictReplayBudget(), exactPoints: map[int]point{},
				groups: Groups{Bevel: 8, Extension: 9, RetainedDiskGroups: map[int]bool{7: tc.registered}},
				out:    Mesh{Vertices: []r3.Vec{{}, {Y: 2}, {X: 1}, {Z: 1}, tc.mid}, Triangles: [][3]int{{0, 1, 2}, {1, 4, 3}, {4, 0, 3}, {1, 3, 2}, {0, 2, 3}}, Sources: []int{9, 7, 7, 6, 6}},
				chart:  chart{start: lift(r3.Vec{}), slope: new(big.Rat)}, support: SourceWindow{XMin: -2, XMax: 2, YMin: -2, YMax: 4, OffsetMin: -2, OffsetMax: 2}}
			err := m.closeDiskGenerator()
			if (err == nil) != tc.want {
				t.Fatalf("closed=%v, error=%v", tc.want, err)
			}
			if tc.want {
				es, err := m.boundary()
				if err != nil || len(es) != 0 {
					t.Fatalf("remaining edges %d: %v", len(es), err)
				}
			}
		})
	}
}

func TestWindowSeparatesPaddedCorner(t *testing.T) {
	w := SourceWindow{Side0: r3.Vec{}, Side1: r3.Vec{X: 1, Z: 1}, XMin: 0, XMax: 1, YMin: -1, YMax: 1, OffsetMin: -.4, OffsetMax: .02}
	v := []r3.Vec{{X: 1.08, Z: 1.08}, {X: 1.09, Z: 1.09}, {X: 1.08, Y: .01, Z: 1.09}}
	for _, tc := range []struct {
		radius float64
		want   bool
	}{{.09, true}, {.12, false}} {
		ok, err := WindowSourcesCovered(t.Context(), v, [][3]int{{0, 1, 2}}, []int{4}, []float64{tc.radius, tc.radius, tc.radius}, nil, w, proofbound.NewStrictReplayBudget())
		if err != nil || ok != tc.want {
			t.Fatalf("radius %g: covered=%v err=%v", tc.radius, ok, err)
		}
	}
}
