package decad

import (
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// TestNormalAtRefusesNURBSSurface pins docs/spline-design.md §7: recovering
// the (u, v) of a given point on a NURBSSurface is a root-find, not a closed
// form, so NormalAt has no bound to publish and refuses. A public Extrude of a
// free-form section now reaches a NURBSSurface-tagged Face
// (extrude_freeform_test.go's TestExtrudeFreeformNormalAtRefuses covers that
// path); this test builds the face directly, the way the package's own
// internal tests build a bare *Face elsewhere, to isolate the refusal from any
// build machinery.
func TestNormalAtRefusesNURBSSurface(t *testing.T) {
	f := &Face{surface: NURBSSurface{}}
	_, err := f.NormalAt(r3.NewVec(0, 0, 0))
	require.ErrorIs(t, err, ErrUnsupported,
		`a NURBSSurface has no closed-form normal to publish a bound for`)
}
