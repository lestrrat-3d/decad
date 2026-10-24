package apitest_test

import (
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"

	"github.com/lestrrat-3d/decad"
)

// containerRect returns a sketch of an axis-aligned rectangle, added to c when c
// is not nil, and its one profile.
func containerRect(t *testing.T, c *decad.Container, x0, y0, x1, y1 float64) (*sketch.Sketch, *sketch.Profile) {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(x0, y0, x1, y1)
	s.Fix(rect.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	if c != nil {
		require.NoError(t, c.AddSketch(s))
	}
	return s, s.Profiles()[0]
}

// containerBox extrudes a rectangle from a sketch placed in c.
func containerBox(t *testing.T, doc *decad.Document, c *decad.Container, x0, y0, x1, y1, h float64) *decad.Body {
	t.Helper()
	s, p := containerRect(t, c, x0, y0, x1, y1)
	body, err := decad.Extrude(t.Context(), doc, s, p, decad.Distance{D: units.Millimeters(h), Dir: decad.Along})
	require.NoError(t, err)
	return body
}

func containerOf(t *testing.T, doc *decad.Document, b *decad.Body) *decad.Container {
	t.Helper()
	c, err := doc.ContainerOf(b)
	require.NoError(t, err)
	return c
}

func TestContainerTree(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	root := doc.Root()
	require.Same(t, root, doc.Root())
	require.Equal(t, "", root.Name())
	require.Nil(t, root.Parent())

	gear, err := root.NewChild("gear")
	require.NoError(t, err)
	pinion, err := root.NewChild("pinion")
	require.NoError(t, err)
	shaft, err := gear.NewChild("shaft")
	require.NoError(t, err)

	require.Equal(t, []*decad.Container{gear, pinion}, root.Children())
	require.Equal(t, []*decad.Container{shaft}, gear.Children())
	require.Empty(t, pinion.Children())
	require.Same(t, gear, shaft.Parent())
	require.Equal(t, "shaft", shaft.Name())

	_, err = root.NewChild("gear")
	require.ErrorIs(t, err, decad.ErrDegenerate, "sibling names are unique")
	_, err = root.NewChild("")
	require.ErrorIs(t, err, decad.ErrDegenerate, "a child needs a name")
	_, err = gear.NewChild("gear")
	require.NoError(t, err, "the same name under another parent is fine")

	var nilContainer *decad.Container
	_, err = nilContainer.NewChild("x")
	require.ErrorIs(t, err, decad.ErrDegenerate)

	other := decad.New()
	require.NotSame(t, root, other.Root())
	require.ErrorIs(t, other.MoveBody(containerBox(t, other, nil, 0, 0, 1, 1, 1), gear), decad.ErrForeignBody)
}

func TestContainerSketchAndPlaneMembership(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	a, err := doc.Root().NewChild("a")
	require.NoError(t, err)
	b, err := doc.Root().NewChild("b")
	require.NoError(t, err)

	w := sketch.NewWorld()
	s1, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	s2, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	plane, err := w.CreateOffsetPlane(w.XY(), 2)
	require.NoError(t, err)

	require.Same(t, doc.Root(), doc.ContainerOfSketch(s1), "an unplaced sketch belongs to the root")
	require.NoError(t, a.AddSketch(s1))
	require.NoError(t, a.AddSketch(s2))
	require.NoError(t, a.AddSketch(s1), "adding again changes nothing")
	require.Equal(t, []*sketch.Sketch{s1, s2}, a.Sketches())
	require.NoError(t, b.AddSketch(s1))
	require.Equal(t, []*sketch.Sketch{s2}, a.Sketches(), "adding elsewhere takes it out")
	require.Equal(t, []*sketch.Sketch{s1}, b.Sketches())
	require.Same(t, b, doc.ContainerOfSketch(s1))

	require.NoError(t, a.AddPlane(plane))
	require.Equal(t, []*sketch.Plane{plane}, a.Planes())
	require.NoError(t, b.AddPlane(plane))
	require.Empty(t, a.Planes())
	require.Equal(t, []*sketch.Plane{plane}, b.Planes())

	require.ErrorIs(t, a.AddSketch(nil), decad.ErrDegenerate)
	require.ErrorIs(t, a.AddPlane(nil), decad.ErrDegenerate)
}

func TestContainerPlacement(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	a, err := doc.Root().NewChild("a")
	require.NoError(t, err)

	loose := containerBox(t, doc, nil, 0, 0, 2, 2, 2)
	require.Same(t, doc.Root(), containerOf(t, doc, loose), "a sketch never added extrudes into the root")

	base := containerBox(t, doc, a, 0, 0, 4, 4, 4)
	tool := containerBox(t, doc, a, 1, 1, 2, 2, 2)
	require.Same(t, a, containerOf(t, doc, base), "an extrude lands in its sketch's container")

	cut, err := decad.Cut(t.Context(), base, tool)
	require.NoError(t, err)
	require.Same(t, a, containerOf(t, doc, cut), "a boolean lands in its target's container")

	moved, err := decad.Transform(t.Context(), cut, mustTranslation(t, 5, 0, 0))
	require.NoError(t, err)
	require.Same(t, a, containerOf(t, doc, moved), "a transform stays with its source")

	copied, err := decad.Copy(t.Context(), moved, decad.WithCopyPlacement(mustTranslation(t, 0, 9, 0)))
	require.NoError(t, err)
	require.Same(t, a, containerOf(t, doc, copied), "a copy stays with its source")
	require.Same(t, a, containerOf(t, doc, moved), "the source stays too")

	pattern, err := decad.PatternCopies(t.Context(), copied, decad.LinearPattern{Dir: r3.NewVec(1, 0, 0), Step: units.Millimeters(20), Count: 3})
	require.NoError(t, err)
	require.Len(t, pattern, 2)
	for _, b := range pattern {
		require.Same(t, a, containerOf(t, doc, b), "every pattern copy lands with its source")
	}

	require.ElementsMatch(t, []*decad.Body{moved, copied, pattern[0], pattern[1]}, a.Bodies())
	require.Equal(t, []*decad.Body{loose}, doc.Root().Bodies())
	require.Len(t, doc.Bodies(), 5, "Bodies lists them all, containers or not")

	// A loft takes its first sketch's container.
	s0, p0 := containerRect(t, a, 0, 0, 3, 3)
	w := sketch.NewWorld()
	plane, err := w.CreateOffsetPlane(w.XY(), 5)
	require.NoError(t, err)
	s1, err := w.CreateSketch(plane)
	require.NoError(t, err)
	r := s1.CreateRectangle(0, 0, 2, 2)
	s1.Fix(r.A)
	_, err = s1.Solve(t.Context())
	require.NoError(t, err)
	require.NoError(t, a.AddSketch(s1))
	lofted, err := decad.Loft(t.Context(), doc, s0, p0, s1, s1.Profiles()[0])
	require.NoError(t, err)
	require.Same(t, a, containerOf(t, doc, lofted))
}

func TestContainerCrossRefusal(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	a, err := doc.Root().NewChild("a")
	require.NoError(t, err)
	b, err := doc.Root().NewChild("b")
	require.NoError(t, err)

	inA := containerBox(t, doc, a, 0, 0, 4, 4, 4)
	inB := containerBox(t, doc, b, 1, 1, 2, 2, 2)
	live := len(doc.Bodies())

	for name, run := range map[string]func() error{
		"Union of two": func() error { _, err := decad.Union(t.Context(), inA, inB); return err },
		"Union of three": func() error {
			_, err := decad.Union(t.Context(), inA, inB, containerBox(t, doc, a, 9, 9, 10, 10, 1))
			return err
		},
		"Cut by a tool":    func() error { _, err := decad.Cut(t.Context(), inA, inB); return err },
		"Intersect of two": func() error { _, err := decad.Intersect(t.Context(), inA, inB); return err },
		"Extrude to a face": func() error {
			s, p := containerRect(t, a, 5, 5, 6, 6)
			_, err := decad.Extrude(t.Context(), doc, s, p, decad.ToFace{Body: inB, Face: capEndFace(inB)})
			return err
		},
		"Loft of two sketches": func() error {
			s0, p0 := containerRect(t, a, 0, 0, 1, 1)
			s1, p1 := containerRect(t, b, 0, 0, 1, 1)
			_, err := decad.Loft(t.Context(), doc, s0, p0, s1, p1)
			return err
		},
	} {
		t.Run(name, func(t *testing.T) {
			err := run()
			require.ErrorIs(t, err, decad.ErrCrossContainer)
			require.ErrorContains(t, err, `"/a"`)
			require.ErrorContains(t, err, `"/b"`)
		})
	}
	// The refusals left both operands live; only the helper boxes were added.
	require.GreaterOrEqual(t, len(doc.Bodies()), live)
	require.Contains(t, doc.Bodies(), inA)
	require.Contains(t, doc.Bodies(), inB)
}

func TestContainerMoveBody(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	a, err := doc.Root().NewChild("a")
	require.NoError(t, err)
	b, err := doc.Root().NewChild("b")
	require.NoError(t, err)

	inA := containerBox(t, doc, a, 0, 0, 4, 4, 4)
	inB := containerBox(t, doc, b, 1, 1, 2, 2, 2)
	volume, err := inB.Volume()
	require.NoError(t, err)
	box, err := inB.Bounds()
	require.NoError(t, err)

	_, err = decad.Cut(t.Context(), inA, inB)
	require.ErrorIs(t, err, decad.ErrCrossContainer)

	require.NoError(t, doc.MoveBody(inB, a))
	require.Same(t, a, containerOf(t, doc, inB))
	after, err := inB.Volume()
	require.NoError(t, err)
	require.Equal(t, volume, after, "a move changes no measurement")
	afterBox, err := inB.Bounds()
	require.NoError(t, err)
	require.Equal(t, box, afterBox)
	require.Empty(t, b.Bodies())
	require.ElementsMatch(t, []*decad.Body{inA, inB}, a.Bodies())

	cut, err := decad.Cut(t.Context(), inA, inB)
	require.NoError(t, err)
	require.Same(t, a, containerOf(t, doc, cut), "after the move the operands share a container")
	require.NoError(t, doc.MoveBody(cut, doc.Root()))
	require.Same(t, doc.Root(), containerOf(t, doc, cut))

	require.ErrorIs(t, doc.MoveBody(inA, b), decad.ErrRetiredBody, "a consumed body cannot move")
	_, err = doc.ContainerOf(inA)
	require.ErrorIs(t, err, decad.ErrRetiredBody)
	require.ErrorIs(t, doc.MoveBody(cut, nil), decad.ErrDegenerate)

	report, err := doc.Verify(t.Context())
	require.NoError(t, err)
	require.True(t, report.Passed())
}
