package apitest_test

import (
	"context"
	"fmt"
	"hash/fnv"
	"math"
	"strings"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// The step helpers return errors and never call t, because a build step runs on a goroutine of its own.

// rsBlock extrudes the rectangle x0..x1, y0..y1 on the plane z by e.
func rsBlock(ctx context.Context, doc *decad.Document, x0, y0, x1, y1, z float64, e decad.Extent) (*decad.Body, error) {
	w := sketch.NewWorld()
	plane := w.XY()
	if z != 0 {
		var err error
		if plane, err = w.CreateOffsetPlane(w.XY(), z); err != nil {
			return nil, err
		}
	}
	s, err := w.CreateSketch(plane)
	if err != nil {
		return nil, err
	}
	r := s.CreateRectangle(x0, y0, x1, y1)
	s.Fix(r.A)
	s.Fix(r.C)
	if _, err := s.Solve(ctx); err != nil {
		return nil, err
	}
	return decad.Extrude(doc, s, s.Profiles()[0], e).Do(ctx)
}

func rsHeight(h float64) decad.Extent {
	return decad.Distance{D: units.Millimeters(h), Dir: decad.Along}
}

func rsShift(x, y, z float64) (r3.Transform, error) { return r3.Translation(r3.Vec{X: x, Y: y, Z: z}) }

func rsAllEdges() *decad.EdgeQuery { return decad.Edges(decad.Convex()).Exactly(12) }

// rsFixture is a document holding one plate built before the steps, and eight steps that cover Extrude with a
// ThroughAll extent, Chamfer, Fillet, Union, Cut, Transform, Copy of a pre-run body, a circular Pattern and a
// Verify. Step 5's ThroughAll sweep reaches farther down in step 0's body than in the plate, so a step that saw
// step 0's body would build a longer solid.
func rsFixture(t *testing.T) (*decad.Document, *decad.Body, []decad.Step) {
	t.Helper()
	doc := decad.New()
	plate, err := rsBlock(t.Context(), doc, 0, 0, 40, 40, 0, rsHeight(4))
	require.NoError(t, err)
	steps := []decad.Step{
		{Name: "chamfer", Build: func(ctx context.Context) error {
			b, err := rsBlock(ctx, doc, 100, 0, 120, 20, 0, rsHeight(10))
			if err != nil {
				return err
			}
			_, err = decad.Chamfer(b, rsAllEdges(), units.Millimeters(1)).Do(ctx)
			return err
		}},
		{Name: "fillet", Build: func(ctx context.Context) error {
			b, err := rsBlock(ctx, doc, 200, 0, 220, 20, 0, rsHeight(10))
			if err != nil {
				return err
			}
			_, err = decad.Fillet(b, rsAllEdges(), units.Millimeters(1)).Do(ctx)
			return err
		}},
		{Name: "union-cut", Build: func(ctx context.Context) error {
			a, err := rsBlock(ctx, doc, 300, 0, 320, 20, 0, rsHeight(10))
			if err != nil {
				return err
			}
			b, err := rsBlock(ctx, doc, 310, 10, 330, 30, 0, rsHeight(10))
			if err != nil {
				return err
			}
			u, err := decad.Union(a, b).Do(ctx)
			if err != nil {
				return err
			}
			tool, err := rsBlock(ctx, doc, 305, 5, 315, 15, 0, rsHeight(5))
			if err != nil {
				return err
			}
			_, err = decad.Cut(u, tool).Do(ctx)
			return err
		}},
		{Name: "transform-copy", Build: func(ctx context.Context) error {
			b, err := rsBlock(ctx, doc, 400, 0, 420, 20, 0, rsHeight(10))
			if err != nil {
				return err
			}
			move, err := rsShift(5, 5, 5)
			if err != nil {
				return err
			}
			if _, err = decad.Transform(b, move).Do(ctx); err != nil {
				return err
			}
			_, err = decad.Copy(plate).Place(decad.CopyTransform{Transform: move}).Do(ctx)
			return err
		}},
		{Name: "pattern", Build: func(ctx context.Context) error {
			b, err := rsBlock(ctx, doc, 500, 0, 504, 4, 0, rsHeight(6))
			if err != nil {
				return err
			}
			_, err = decad.PatternCopies(b, decad.CircularPattern{Center: r3.Vec{}, Axis: r3.NewVec(0, 0, 1), Count: 5}).Do(ctx)
			return err
		}},
		{Name: "through-all", Build: func(ctx context.Context) error {
			_, err := rsBlock(ctx, doc, 0, 0, 10, 10, 20, decad.ThroughAll{Dir: decad.Against})
			return err
		}},
		{Name: "verify", Build: func(ctx context.Context) error {
			if _, err := rsBlock(ctx, doc, 600, 0, 610, 10, 0, rsHeight(4)); err != nil {
				return err
			}
			if _, err := rsBlock(ctx, doc, 605, 5, 615, 15, 0, rsHeight(4)); err != nil {
				return err
			}
			if _, err := rsBlock(ctx, doc, 602, 2, 612, 12, 0, rsHeight(4)); err != nil {
				return err
			}
			rep, err := doc.Verify(ctx)
			if err != nil {
				return err
			}
			if len(rep.Bodies) < 4 {
				return fmt.Errorf("a step's Verify saw %d bodies, want the plate and the step's three", len(rep.Bodies))
			}
			return nil
		}},
		{Name: "low-block", Build: func(ctx context.Context) error {
			// Step 0's partner in the ThroughAll test: it reaches down to z = -5 where the plate stops at 0.
			_, err := rsBlock(ctx, doc, 0, 0, 10, 10, -5, rsHeight(15))
			return err
		}},
	}
	return doc, plate, steps
}

// rsPrint is what two runs must agree on for one body: the origin, a hash of every face's origins, the
// volume, and a hash of the tessellation's vertex bits and triangle indices.
type rsPrint struct {
	Origin    decad.FeatureRef
	Faces     uint64
	Volume    float64
	Mesh      uint64
	Vertex    int
	Triangles int
}

func rsPrintBodies(ctx context.Context, bodies []*decad.Body) ([]rsPrint, error) {
	out := make([]rsPrint, len(bodies))
	for i, b := range bodies {
		fh := fnv.New64a()
		for _, f := range b.Faces() {
			fmt.Fprintf(fh, "%v;", f.Origins())
		}
		out[i].Origin = b.Origin()
		out[i].Faces = fh.Sum64()
		if v, err := b.Volume(); err == nil {
			out[i].Volume = v.Value.Base()
		}
		m, err := b.Tessellate(ctx, units.Millimeters(0.1))
		if err != nil {
			return nil, fmt.Errorf("body %d: %w", i, err)
		}
		mh := fnv.New64a()
		var buf [8]byte
		put := func(u uint64) {
			for k := range buf {
				buf[k] = byte(u >> (8 * k))
			}
			_, _ = mh.Write(buf[:])
		}
		for _, v := range m.Vertices() {
			put(math.Float64bits(v.X))
			put(math.Float64bits(v.Y))
			put(math.Float64bits(v.Z))
		}
		for _, tri := range m.Triangles() {
			put(uint64(tri[0]))
			put(uint64(tri[1]))
			put(uint64(tri[2]))
		}
		out[i].Mesh = mh.Sum64()
		out[i].Vertex = len(m.Vertices())
		out[i].Triangles = len(m.Triangles())
	}
	return out, nil
}

// rsRequireSame fails at the first body and field that differ. It reports bounded data only.
func rsRequireSame(t *testing.T, want, got []rsPrint) {
	t.Helper()
	require.Len(t, got, len(want), "body count")
	for i := range want {
		if want[i] == got[i] {
			continue
		}
		var diff []string
		if want[i].Origin != got[i].Origin {
			diff = append(diff, "origin")
		}
		if want[i].Faces != got[i].Faces {
			diff = append(diff, "face origins")
		}
		if want[i].Volume != got[i].Volume {
			diff = append(diff, fmt.Sprintf("volume %v != %v", want[i].Volume, got[i].Volume))
		}
		if want[i].Mesh != got[i].Mesh {
			diff = append(diff, "mesh bits")
		}
		require.FailNowf(t, "bodies differ", "body %d differs in: %s", i, strings.Join(diff, ", "))
	}
}
