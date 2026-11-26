package decad

import (
	"fmt"
	"maps"
	"slices"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"

	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/topo"
)

// This file holds the helpers of intersection_volume_tables_test.go, which must sit in the root package
// because the document tables it compares are unexported.

// ivBox is a 10 mm tall box over the rectangle (x0, y0)-(x1, y1) on the plane z = z0.
func ivBox(t *testing.T, doc *Document, x0, y0, x1, y1, z0 float64) *Body {
	t.Helper()
	w := sketch.NewWorld()
	plane, err := w.CreateOffsetPlane(w.XY(), z0)
	require.NoError(t, err)
	s, err := w.CreateSketch(plane)
	require.NoError(t, err)
	rect := s.CreateRectangle(x0, y0, x1, y1)
	s.Fix(rect.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	b, err := Extrude(t.Context(), doc, s, s.Profiles()[0], Distance{D: units.Millimeters(10), Dir: Along})
	require.NoError(t, err)
	return b
}

func ivCylinder(t *testing.T, doc *Document, cx, cy, r, z0, h float64) *Body {
	t.Helper()
	w := sketch.NewWorld()
	plane, err := w.CreateOffsetPlane(w.XY(), z0)
	require.NoError(t, err)
	s, err := w.CreateSketch(plane)
	require.NoError(t, err)
	c := s.CreatePoint(cx, cy)
	s.Fix(c)
	s.CreateCircle(c, r)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	b, err := Extrude(t.Context(), doc, s, s.Profiles()[0], Distance{D: units.Millimeters(h), Dir: Along})
	require.NoError(t, err)
	return b
}

// ivMeshBox turns a box 20 degrees about a diagonal axis and back, so it is a mesh body whose float corners round.
func ivMeshBox(t *testing.T, b *Body) *Body {
	t.Helper()
	for _, deg := range []float64{20, -20} {
		turn, err := r3.Rotation(r3.Vec{X: 1, Y: 1, Z: 1}, units.Degrees(deg))
		require.NoError(t, err)
		b, err = Transform(t.Context(), b, turn)
		require.NoError(t, err)
	}
	return b
}

// ivGridBlock is a SolidFromMesh block every face of which squares to a world axis.
func ivGridBlock(t *testing.T, doc *Document, lo, hi r3.Vec) *Body {
	t.Helper()
	v := []r3.Vec{
		{X: lo.X, Y: lo.Y, Z: lo.Z}, {X: hi.X, Y: lo.Y, Z: lo.Z}, {X: hi.X, Y: hi.Y, Z: lo.Z}, {X: lo.X, Y: hi.Y, Z: lo.Z},
		{X: lo.X, Y: lo.Y, Z: hi.Z}, {X: hi.X, Y: lo.Y, Z: hi.Z}, {X: hi.X, Y: hi.Y, Z: hi.Z}, {X: lo.X, Y: hi.Y, Z: hi.Z},
	}
	var tris [][3]int
	for _, q := range [][4]int{{0, 3, 2, 1}, {4, 5, 6, 7}, {0, 1, 5, 4}, {3, 7, 6, 2}, {0, 4, 7, 3}, {1, 2, 6, 5}} {
		tris = append(tris, [3]int{q[0], q[1], q[2]}, [3]int{q[0], q[2], q[3]})
	}
	m, err := NewMeshBody(t.Context(), v, tris)
	require.NoError(t, err)
	body, err := SolidFromMesh(t.Context(), doc, m)
	require.NoError(t, err)
	return body
}

// ivTables is a copy of the document state an operation writes and a read-only query must leave alone. The
// level and curve counters are not in it: a query may advance them.
type ivTables struct {
	nextProducer       producerID
	live               []*topo.Body
	operations         []*Operation
	producers          map[*Body]*Operation
	consumers          map[*Body][]*Operation
	circleBuilds       map[*Body]*prismrecord.CircleBuildWitness
	circlePlacements   map[*Body]*circlePlacementWitness
	identityPlacements map[*Body]struct{}
	ofBody             map[*topo.Body]*Container
	rootChildren       []*Container
}

func ivTablesOf(d *Document) ivTables {
	d.n.LiveMu.RLock()
	defer d.n.LiveMu.RUnlock()
	tb := ivTables{
		nextProducer:       d.n.NextProducer,
		operations:         slices.Clone(d.operations),
		producers:          maps.Clone(d.producers),
		consumers:          map[*Body][]*Operation{},
		circleBuilds:       maps.Clone(d.circleBuilds),
		circlePlacements:   maps.Clone(d.circlePlacements),
		identityPlacements: maps.Clone(d.identityPlacements),
		ofBody:             maps.Clone(d.groups.ofBody),
	}
	tb.live = slices.Clone(d.n.LiveSet)
	for b, ops := range d.consumers {
		tb.consumers[b] = slices.Clone(ops)
	}
	if root := d.groups.root; root != nil {
		tb.rootChildren = slices.Clone(root.children)
	}
	return tb
}

// ivDiff names every table that differs, by count and name only, so a failure prints bounded text.
func ivDiff(before, after ivTables) []string {
	var out []string
	if before.nextProducer != after.nextProducer {
		out = append(out, fmt.Sprintf("NextProducer %d -> %d", before.nextProducer, after.nextProducer))
	}
	check := func(name string, same bool, nb, na int) {
		if !same {
			out = append(out, fmt.Sprintf("%s changed (%d -> %d entries)", name, nb, na))
		}
	}
	check("LiveSet", slices.Equal(before.live, after.live), len(before.live), len(after.live))
	check("operations", slices.Equal(before.operations, after.operations), len(before.operations), len(after.operations))
	check("producers", maps.Equal(before.producers, after.producers), len(before.producers), len(after.producers))
	check("consumers", maps.EqualFunc(before.consumers, after.consumers, slices.Equal), len(before.consumers),
		len(after.consumers))
	check("circleBuilds", maps.Equal(before.circleBuilds, after.circleBuilds), len(before.circleBuilds),
		len(after.circleBuilds))
	check("circlePlacements", maps.Equal(before.circlePlacements, after.circlePlacements),
		len(before.circlePlacements), len(after.circlePlacements))
	check("identityPlacements", maps.Equal(before.identityPlacements, after.identityPlacements),
		len(before.identityPlacements), len(after.identityPlacements))
	check("container membership", maps.Equal(before.ofBody, after.ofBody), len(before.ofBody), len(after.ofBody))
	check("root children", slices.Equal(before.rootChildren, after.rootChildren), len(before.rootChildren),
		len(after.rootChildren))
	return out
}
