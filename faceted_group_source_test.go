package decad

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// sourceScene builds the 40x20x20 plate of agent-docs/retained-edge-blend-design.md
// §9 and a translated Ø8 z-cylinder, so a Cut takes the mesh path.
type sourceScene struct {
	w     *sketch.World
	doc   *Document
	plate *Body
}

func newSourceScene(t *testing.T) *sourceScene {
	t.Helper()
	sc := &sourceScene{w: sketch.NewWorld(), doc: New()}
	s, err := sc.w.CreateSketch(sc.w.XY())
	require.NoError(t, err)
	r := s.CreateRectangle(0, 0, 40, 20)
	s.Fix(r.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	sc.plate, err = Extrude(t.Context(), sc.doc, s, s.Profiles()[0],
		Distance{D: units.Millimeters(20), Dir: Along})
	require.NoError(t, err)
	return sc
}

func (sc *sourceScene) tool(t *testing.T, cx float64) *Body {
	t.Helper()
	ts, err := sc.w.CreateSketch(sc.w.XY())
	require.NoError(t, err)
	c := ts.CreatePoint(cx, 10)
	ts.Fix(c)
	ts.CreateCircle(c, 4)
	_, err = ts.Solve(t.Context())
	require.NoError(t, err)
	pin, err := Extrude(t.Context(), sc.doc, ts, ts.Profiles()[0],
		Distance{D: units.Millimeters(60), Dir: Along})
	require.NoError(t, err)
	move, err := r3.Translation(r3.Vec{Z: -29.999})
	require.NoError(t, err)
	placed, err := Transform(t.Context(), pin, move)
	require.NoError(t, err)
	return placed
}

func facetedGroups(t *testing.T, b *Body) []facetedrecord.Group {
	t.Helper()
	fp, ok := b.n.Payload.(facetedPayload)
	require.True(t, ok, "mesh-built payload")
	return fp.Groups
}

func TestBooleanGroupsRecordSource(t *testing.T) {
	sc := newSourceScene(t)
	tool := sc.tool(t, 20)
	bored, err := Cut(t.Context(), sc.plate, tool)
	require.NoError(t, err)

	plateFaces := len(topobuild.Faces(&sc.plate.n))
	toolFaces := len(topobuild.Faces(&tool.n))
	groups := facetedGroups(t, bored)
	require.Len(t, groups, plateFaces+toolFaces)
	for i, g := range groups {
		require.NotNil(t, g.Source, "group %d", i)
		wantOperand, wantOrdinal, wantProducer := 0, i, sc.plate.n.Origin.Producer
		if i >= plateFaces {
			wantOperand, wantOrdinal, wantProducer = 1, i-plateFaces, tool.n.Origin.Producer
		}
		require.Equal(t, wantProducer, g.Source.Producer, "group %d producer", i)
		require.Equal(t, wantOperand, g.Source.Operand, "group %d operand", i)
		require.Equal(t, wantOrdinal, g.Source.Ordinal, "group %d ordinal", i)
		require.False(t, g.Source.Blend, "group %d", i)
		require.Nil(t, g.Source.Parent, "group %d built face has no parent", i)
	}

	// A later Cut of the faceted result names the first result's records as
	// parents of the plate-side groups.
	second, err := Cut(t.Context(), bored, sc.tool(t, 30))
	require.NoError(t, err)
	boredFaces := topobuild.Faces(&bored.n)
	for i, g := range facetedGroups(t, second)[:len(boredFaces)] {
		require.NotNil(t, boredFaces[i].Source, "face %d", i)
		require.Same(t, boredFaces[i].Source, g.Source.Parent, "group %d parent", i)
		require.Equal(t, bored.n.Origin.Producer, g.Source.Producer)
		require.Equal(t, i, g.Source.Ordinal)
	}
}

func TestRetainedBlendGroupsRecordSource(t *testing.T) {
	sc := newSourceScene(t)
	bored, err := Cut(t.Context(), sc.plate, sc.tool(t, 20))
	require.NoError(t, err)
	var sel EdgeSelector
	for _, e := range bored.Edges() {
		c, ok := e.Curve().(Circle3)
		if ok && math.Abs(c.Center.Z-20) < 1e-6 {
			sel = Edges(EndpointAt(e.Start().Position().Value)).Exactly(1)
			break
		}
	}
	require.NotNil(t, sel, "bore mouth at the top")
	filleted, err := Fillet(t.Context(), bored, sel, units.Millimeters(1))
	require.NoError(t, err)

	before, after := facetedGroups(t, bored), facetedGroups(t, filleted)
	require.Len(t, after, len(before)+1)
	for i := range before {
		require.Same(t, before[i].Source, after[i].Source, "copied group %d keeps its record", i)
	}
	blend := after[len(before)].Source
	require.NotNil(t, blend)
	require.Equal(t, filleted.n.Origin.Producer, blend.Producer)
	require.Equal(t, 0, blend.Ordinal)
	require.True(t, blend.Blend)
	require.Nil(t, blend.Parent)
}
