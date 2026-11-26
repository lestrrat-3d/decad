package decad

import (
	"context"
	"fmt"
	"hash/fnv"
	"math"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// The helpers here return errors and never call t, because a build step runs on a goroutine of its own.

// stepsBox extrudes the rectangle x0..x1, y0..y1 on the XY plane up by 4 mm.
func stepsBox(ctx context.Context, doc *Document, x0, y0, x1, y1 float64) (*Body, error) {
	world := sketch.NewWorld()
	s, err := world.CreateSketch(world.XY())
	if err != nil {
		return nil, err
	}
	rect := s.CreateRectangle(x0, y0, x1, y1)
	s.Fix(rect.A)
	s.Fix(rect.C)
	if _, err := s.Solve(ctx); err != nil {
		return nil, err
	}
	return Extrude(doc, s, s.Profiles()[0], Distance{D: units.Millimeters(4), Dir: Along}).Do(ctx)
}

// stepsBoxes builds n boxes in a row, moves each, and returns what the last Transform produced.
func stepsBoxes(ctx context.Context, doc *Document, x0 float64, n int) error {
	for i := range n {
		b, err := stepsBox(ctx, doc, x0+float64(i)*30, 0, x0+float64(i)*30+10, 10)
		if err != nil {
			return err
		}
		move, err := r3.Translation(r3.Vec{X: 1, Y: 1, Z: 1})
		if err != nil {
			return err
		}
		if _, err := Transform(b, move).Do(ctx); err != nil {
			return err
		}
	}
	return nil
}

// stepIDRanges collects the level and curve ids a body carries on its vertices and edges.
func stepIDRanges(b *Body) (levels, curves map[int]struct{}) {
	levels, curves = map[int]struct{}{}, map[int]struct{}{}
	add := func(l, c int) {
		if l != 0 {
			levels[l] = struct{}{}
		}
		if c != 0 {
			curves[c] = struct{}{}
		}
	}
	for _, e := range b.Edges() {
		add(int(e.n.Level.ID), int(e.n.Denot.ID))
	}
	for _, v := range b.Vertices() {
		add(int(v.n.Level.ID), int(v.n.Denot.ID))
	}
	return levels, curves
}

func stepsLabel(i int) string { return fmt.Sprintf("step %d", i) }

// stepsPrint is what two runs must agree on for one body.
type stepsPrint struct {
	Origin   FeatureRef
	Volume   float64
	Vertices int
	Hash     uint64
}

func stepsPrints(t *testing.T, doc *Document) []stepsPrint {
	t.Helper()
	var out []stepsPrint
	for _, b := range doc.Bodies() {
		v, err := b.Volume()
		require.NoError(t, err)
		m, err := b.Tessellate(t.Context(), units.Millimeters(0.1))
		require.NoError(t, err)
		h := fnv.New64a()
		for _, p := range m.Vertices() {
			fmt.Fprintf(h, "%x %x %x;", math.Float64bits(p.X), math.Float64bits(p.Y), math.Float64bits(p.Z))
		}
		for _, f := range b.Faces() {
			fmt.Fprintf(h, "%v;", f.Origins())
		}
		out = append(out, stepsPrint{Origin: b.Origin(), Volume: v.Value.Base(), Vertices: len(m.Vertices()), Hash: h.Sum64()})
	}
	return out
}

// stepsMeshBlock builds the faceted block lo..hi as a mesh solid. A mesh block is a prism Draft reads from its mesh
// (agent-docs/orthogonal-design.md B21), which is the route that retires its input through removeConsumed.
func stepsMeshBlock(ctx context.Context, doc *Document, lo, hi r3.Vec) (*Body, error) {
	v := []r3.Vec{
		{X: lo.X, Y: lo.Y, Z: lo.Z}, {X: hi.X, Y: lo.Y, Z: lo.Z}, {X: hi.X, Y: hi.Y, Z: lo.Z}, {X: lo.X, Y: hi.Y, Z: lo.Z},
		{X: lo.X, Y: lo.Y, Z: hi.Z}, {X: hi.X, Y: lo.Y, Z: hi.Z}, {X: hi.X, Y: hi.Y, Z: hi.Z}, {X: lo.X, Y: hi.Y, Z: hi.Z},
	}
	var tris [][3]int
	for _, q := range [][4]int{{0, 3, 2, 1}, {4, 5, 6, 7}, {0, 1, 5, 4}, {3, 7, 6, 2}, {0, 4, 7, 3}, {1, 2, 6, 5}} {
		tris = append(tris, [3]int{q[0], q[1], q[2]}, [3]int{q[0], q[2], q[3]})
	}
	m, err := NewMeshBody(ctx, v, tris)
	if err != nil {
		return nil, err
	}
	return SolidFromMesh(doc, m).Do(ctx)
}

// stepsDraftBlock drafts every wall of a mesh block about its bottom, the call that takes the mesh prism route.
func stepsDraftBlock(ctx context.Context, block *Body) (*Body, error) {
	walls := Faces(Facing(r3.NewVec(1, 0, 0))).Or(Facing(r3.NewVec(-1, 0, 0))).
		Or(Facing(r3.NewVec(0, 1, 0))).Or(Facing(r3.NewVec(0, -1, 0)))
	bottom := NeutralFace{Body: block, Face: Faces(Facing(r3.NewVec(0, 0, -1)))}
	return Draft(block, walls, bottom, units.Degrees(5)).Do(ctx)
}
