package facetedrecord_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/revolveplan"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/stretchr/testify/require"
)

func TestRecognizePrismUnionFillet(t *testing.T) {
	t.Parallel()

	prism := func() *topo.Body { return &topo.Body{Payload: prismrecord.Payload{}} }
	a, b := prism(), prism()
	got := facetedrecord.RecognizePrismUnionFillet(a, b)
	require.NotNil(t, got)
	require.Same(t, a, got.First)
	require.Same(t, b, got.Second)

	other := &topo.Body{Payload: revolveplan.Payload{}}
	require.Nil(t, facetedrecord.RecognizePrismUnionFillet(other, b))
	require.Nil(t, facetedrecord.RecognizePrismUnionFillet(a, other))
	require.Nil(t, facetedrecord.RecognizePrismUnionFillet(&topo.Body{}, &topo.Body{}))
}
