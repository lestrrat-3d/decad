package decad

import (
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// TestNormalAtRefusesNURBSSurface pins docs/spline-design.md §7: recovering
// the (u, v) of a given point on a NURBSSurface is a root-find, not a closed
// form, so NormalAt has no bound to publish and refuses. No public build path
// reaches a NURBSSurface-tagged Face yet (that lands with the free-form
// side-face build), so the face is built directly the way the package's own
// internal tests build a bare *Face elsewhere.
func TestNormalAtRefusesNURBSSurface(t *testing.T) {
	f := &Face{surface: NURBSSurface{}}
	_, err := f.NormalAt(r3.NewVec(0, 0, 0))
	require.ErrorIs(t, err, ErrUnsupported,
		`a NURBSSurface has no closed-form normal to publish a bound for`)
}
