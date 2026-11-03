package meshbool

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// TestLocalSlackReadsPlanarBounds holds the gate's local slack to the planar
// record: a contact far from the face's chords reads the facet's planar
// bound, and one within the chords' reach keeps the facet's own bound.
//
// Leg shown to fail: ignoring the chords reads the planar bound beside them.
func TestLocalSlackReadsPlanarBounds(t *testing.T) {
	t.Parallel()
	chords := [][2]r3.Vec{{{X: 0, Y: 0}, {X: 1, Y: 0}}}
	a := &BoolMesh{
		Boxes:        [][2]r3.Vec{{{X: 0, Y: 0}, {X: 10, Y: 10}}},
		FacetBound:   []float64{0.0137},
		PlanarOwn:    []float64{1e-6},
		PlanarChords: [][][2]r3.Vec{chords},
		PlanarReach:  []float64{0.0138},
	}
	for _, tc := range []struct {
		name string
		box  [2]r3.Vec
		want float64
	}{
		{"contact far from the chord", [2]r3.Vec{{X: 8, Y: 8}, {X: 9, Y: 9}}, 1e-6},
		{"contact at the chord", [2]r3.Vec{{X: 0.5, Y: 0.001}, {X: 0.6, Y: 0.002}}, 0.0137},
	} {
		b := &BoolMesh{Boxes: [][2]r3.Vec{tc.box}, FacetBound: []float64{0}}
		got := localSlackFromContacts(a, b, NearContacts{CloseA: []int{0}, CloseB: []int{0}}, 0.0137)
		require.InDelta(t, tc.want, got, 1e-12, tc.name)
	}
	require.True(t, ChordClear(r3.Vec{X: 2}, r3.Vec{X: 3}, chords, 0.5))
	require.False(t, ChordClear(r3.Vec{X: 1.2}, r3.Vec{X: 3}, chords, 0.5))
	require.True(t, ChordClear(r3.Vec{X: 3}, r3.Vec{X: 2}, chords, math.Inf(1)), "an empty box is clear")
}
