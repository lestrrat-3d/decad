package decad

import (
	"testing"

	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// Remove produces no body, so it mints no producer identity: the next
// feature occupies exactly the identity it would have without the Remove.
func TestDocumentRemoveMintsNoProducer(t *testing.T) {
	t.Parallel()
	d := New()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(0, 0, 10, 10)
	s.Fix(rect.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	body, err := d.Extrude(s, s.Profiles()[0], Distance{D: units.Millimeters(5), Dir: Along})
	require.NoError(t, err)

	before := d.nextProducerID()
	require.NoError(t, d.Remove(body))
	require.Equal(t, before, d.nextProducerID())
	require.Empty(t, d.bodies)
	require.Equal(t, before-1, body.originProducer(), `the removed body keeps the identity it was built under`)
}
