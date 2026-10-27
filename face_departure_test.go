package decad

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/facedeparture"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/stretchr/testify/require"
)

// TestFaceDepartureReader runs the reader the STEP writer's tag gate calls on
// face nodes that state no departure, a NormalBound, a retained denoted
// face's HeldNormalBound, and on a value that is not a face. Only the first
// reads false. Shown to fail with each of the two bound legs deleted (that
// face read false) and with the non-face case returning false.
func TestFaceDepartureReader(t *testing.T) {
	t.Parallel()
	for _, tc := range []struct {
		name string
		face any
		want bool
	}{
		{"exact tag", &Face{}, false},
		{"normal bound", &Face{n: topo.Face{NormalBound: 1e-16}}, true},
		{"held normal bound", &Face{n: topo.Face{HeldNormalBound: 1e-16}}, true},
		{"not a face", &Edge{}, true},
		{"nil face", (*Face)(nil), true},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			require.Equal(t, tc.want, facedeparture.Stated(tc.face))
		})
	}
}
