package facedeparture_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/facedeparture"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// TestStatedWithoutReader runs Stated in a binary that never loads the root
// package, so Reader is unset: every value reads as stating a departure, and
// a reader that only refuses on it refuses. Shown to fail with Stated
// returning false when Reader is nil.
func TestStatedWithoutReader(t *testing.T) {
	t.Parallel()
	require.Nil(t, facedeparture.Reader)
	require.True(t, facedeparture.Stated(struct{}{}))
}

// TestChargedWithoutReader runs Charged in a binary that never loads the root
// package, so ChargedReader is unset: every value reads as charged and a
// reader that only refuses on it refuses.
func TestChargedWithoutReader(t *testing.T) {
	t.Parallel()
	require.Nil(t, facedeparture.ChargedReader)
	require.True(t, facedeparture.Charged(struct{}{}))
}

func TestChargedTag(t *testing.T) {
	t.Parallel()
	frame, err := r3.NewFrame(r3.Vec{}, r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	plane := surfacegeom.Plane{Frame: frame}
	cyl := surfacegeom.Cylinder{Axis: r3.NewVec(0, 0, 1), Radius: units.Millimeters(2)}
	for name, tc := range map[string]struct {
		face *topo.Face
		want bool
	}{
		"a cylinder with a charge":           {&topo.Face{Surface: cyl, HasAxialDelta: true, AxialDelta: 1e-9}, true},
		"a cylinder with an unusable charge": {&topo.Face{Surface: cyl, HasAxialDelta: true, AxialDelta: math.NaN()}, true},
		"a cylinder with a zero charge":      {&topo.Face{Surface: cyl, HasAxialDelta: true}, false},
		"a cylinder with no charge":          {&topo.Face{Surface: cyl, AxialDelta: 1e-9}, false},
		"a displaced plane":                  {&topo.Face{Surface: plane, HasAxialDelta: true, AxialDelta: 1e-9}, false},
	} {
		require.Equal(t, tc.want, facedeparture.ChargedTag(tc.face), name)
	}
}
