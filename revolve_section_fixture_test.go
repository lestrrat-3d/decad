package decad

import (
	"testing"

	"github.com/lestrrat-3d/sketch"
	"github.com/stretchr/testify/require"
)

func sectionSketchBody(t *testing.T, build func(*sketch.Sketch), extent AngularExtent) *Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	build(s)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	body, err := Revolve(t.Context(), New(), s, s.Profiles()[0], revolveAxisU, extent)
	require.NoError(t, err)
	return body
}
