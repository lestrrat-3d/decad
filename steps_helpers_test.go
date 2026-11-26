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
