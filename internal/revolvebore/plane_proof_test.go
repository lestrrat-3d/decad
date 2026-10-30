package revolvebore

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/prismplacement"
	"github.com/lestrrat-3d/decad/internal/revolveaxis"
	"github.com/lestrrat-3d/decad/internal/revolveplan"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestSameRevolveCapsChecksExactPlanes(t *testing.T) {
	meridian, err := r3.NewFrame(r3.Vec{}, r3.Vec{X: 1}, r3.Vec{Y: 1})
	require.NoError(t, err)
	frame, err := r3.NewFrame(r3.Vec{}, r3.Vec{Y: 1}, r3.Vec{Z: 1})
	require.NoError(t, err)
	rp := revolveplan.Payload{Frame: meridian, Ax: revolveaxis.Frame{DU: 1}, Xform: r3.Identity()}
	target := prismplacement.Operand{Frame: frame, Xform: r3.Identity()}
	require.True(t, sameRevolveCaps(rp, target, Meridian{Low: 0, High: 3}))
	target.Frame, err = r3.NewFrame(r3.Vec{X: 0.125}, r3.Vec{Y: 1}, r3.Vec{Z: 1})
	require.NoError(t, err)
	require.False(t, sameRevolveCaps(rp, target, Meridian{Low: 0, High: 3}))
	target.Frame, err = r3.NewFrame(r3.Vec{Y: 0.125}, r3.Vec{Y: 1}, r3.Vec{Z: 1})
	require.NoError(t, err)
	require.False(t, sameRevolveCaps(rp, target, Meridian{Low: 0, High: 3}))
}
