package modifyselect_test

import (
	"context"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/modifyselect"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func squarePrism(t *testing.T) prismrecord.Payload {
	t.Helper()
	frame, err := r3.NewFrame(r3.Vec{}, r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	corners := []sectionrecord.Point2{{}, {U: 4}, {U: 4, V: 4}, {V: 4}}
	var segs []sectionrecord.CurveSegment
	for i, c := range corners {
		segs = append(segs, sectionrecord.LineSeg{Start: c, End: corners[(i+1)%4], TEnd: 1})
	}
	return prismrecord.Payload{Frame: frame, Xform: r3.Identity(), Z0: 0, Z1: 6,
		Profile: momentinput.Profile{Outer: sectionrecord.LoopRecord{Segments: segs}}}
}

func lineEdge(a, b r3.Vec) *topo.Edge {
	return &topo.Edge{Curve: surfacegeom.Line3{}, Start: &topo.Vertex{Position: a}, End: &topo.Vertex{Position: b}}
}

func TestRequireLateral(t *testing.T) {
	t.Parallel()

	pp := squarePrism(t)
	corner := lineEdge(r3.Vec{X: 4}, r3.Vec{X: 4, Z: 6})
	other := lineEdge(r3.Vec{X: 4, Y: 4, Z: 6}, r3.Vec{X: 4, Y: 4})
	capEdge := lineEdge(r3.Vec{}, r3.Vec{X: 4})

	require.NoError(t, modifyselect.RequireLateral(t.Context(), pp, nil))
	require.NoError(t, modifyselect.RequireLateral(t.Context(), pp, []*topo.Edge{corner}))
	require.NoError(t, modifyselect.RequireLateral(t.Context(), pp, []*topo.Edge{corner, other}))

	err := modifyselect.RequireLateral(t.Context(), pp, []*topo.Edge{corner, capEdge})
	require.ErrorIs(t, err, decaderr.ErrUnsupported)
	require.ErrorContains(t, err, "is not a lateral edge of the prism")
	require.ErrorContains(t, err, modifyselect.EdgeContext(1, capEdge), "the message names the refused edge")
	require.NotContains(t, err.Error(), modifyselect.EdgeContext(0, corner))

	cancelled, cancel := context.WithCancel(t.Context())
	cancel()
	require.Error(t, modifyselect.RequireLateral(cancelled, pp, []*topo.Edge{corner}))
}
