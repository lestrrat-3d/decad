package decad

import (
	"context"
	"math"
	"math/big"
	"testing"
	"time"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/revolvemesh"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func internalBoxBody(t *testing.T, doc *Document, x0, y0, x1, y1, height float64) *Body {
	t.Helper()
	world := sketch.NewWorld()
	s, err := world.CreateSketch(world.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(x0, y0, x1, y1)
	s.Fix(rect.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	body, err := Extrude(t.Context(), doc, s, s.Profiles()[0], Distance{D: units.Millimeters(height), Dir: Along})
	require.NoError(t, err)
	return body
}

func internalDiscBody(t testing.TB, doc *Document, radius, height float64) *Body {
	t.Helper()
	world := sketch.NewWorld()
	s, err := world.CreateSketch(world.XY())
	require.NoError(t, err)
	center := s.CreatePoint(0, 0)
	s.Fix(center)
	s.CreateCircle(center, radius)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	body, err := Extrude(t.Context(), doc, s, s.Profiles()[0], Distance{D: units.Millimeters(height), Dir: Along})
	require.NoError(t, err)
	return body
}

func internalOffsetBox(t *testing.T, doc *Document, x0, y0, x1, y1, z float64, extent Extent) *Body {
	t.Helper()
	world := sketch.NewWorld()
	plane, err := world.CreateOffsetPlane(world.XY(), z)
	require.NoError(t, err)
	s, err := world.CreateSketch(plane)
	require.NoError(t, err)
	rect := s.CreateRectangle(x0, y0, x1, y1)
	s.Fix(rect.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	body, err := Extrude(t.Context(), doc, s, s.Profiles()[0], extent)
	require.NoError(t, err)
	return body
}

func internalPolyPrismBody(t *testing.T, doc *Document, points [][2]float64, height float64) *Body {
	t.Helper()
	world := sketch.NewWorld()
	s, err := world.CreateSketch(world.XY())
	require.NoError(t, err)
	vertices := make([]*sketch.Point, len(points))
	for i, point := range points {
		vertices[i] = s.CreatePoint(point[0], point[1])
		s.Fix(vertices[i])
	}
	for i := range vertices {
		s.CreateLine(vertices[i], vertices[(i+1)%len(vertices)])
	}
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	profiles := s.Profiles()
	require.Len(t, profiles, 1)
	body, err := Extrude(t.Context(), doc, s, profiles[0], Distance{D: units.Millimeters(height), Dir: Along})
	require.NoError(t, err)
	return body
}

type cancelAfterContext struct {
	deadline func() (time.Time, bool)
	done     <-chan struct{}
	value    func(any) any
	err      func() error
	cancelAt int
	calls    int
}

func newCancelAfterContext(parent context.Context, cancelAt int) *cancelAfterContext {
	return &cancelAfterContext{
		deadline: parent.Deadline,
		done:     parent.Done(),
		value:    parent.Value,
		err:      parent.Err,
		cancelAt: cancelAt,
	}
}

func (c *cancelAfterContext) Deadline() (time.Time, bool) { return c.deadline() }

func (c *cancelAfterContext) Done() <-chan struct{} { return c.done }

func (c *cancelAfterContext) Value(key any) any { return c.value(key) }

func (c *cancelAfterContext) Err() error {
	c.calls++
	if c.calls >= c.cancelAt {
		return context.Canceled
	}
	return c.err()
}

func internalBoxBodyAtZ(t *testing.T, doc *Document, x0, y0, x1, y1, z0, height float64) *Body {
	t.Helper()
	world := sketch.NewWorld()
	plane, err := world.CreateOffsetPlane(world.XY(), z0)
	require.NoError(t, err)
	s, err := world.CreateSketch(plane)
	require.NoError(t, err)
	rect := s.CreateRectangle(x0, y0, x1, y1)
	s.Fix(rect.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	body, err := Extrude(t.Context(), doc, s, s.Profiles()[0], Distance{D: units.Millimeters(height), Dir: Along})
	require.NoError(t, err)
	return body
}

func internalCircleBody(t *testing.T, doc *Document, x, radius, z float64, extent Extent) *Body {
	t.Helper()
	world := sketch.NewWorld()
	plane, err := world.CreateOffsetPlane(world.XY(), z)
	require.NoError(t, err)
	s, err := world.CreateSketch(plane)
	require.NoError(t, err)
	center := s.CreatePoint(x, 0)
	s.Fix(center)
	s.CreateCircle(center, radius)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	body, err := Extrude(t.Context(), doc, s, s.Profiles()[0], extent)
	require.NoError(t, err)
	return body
}

func internalRoundBoss(t *testing.T) *Body {
	t.Helper()
	doc := New()
	plate := internalBoxBody(t, doc, 0, 0, 40, 40, 10)
	world := sketch.NewWorld()
	plane, err := world.CreateOffsetPlane(world.XY(), 10)
	require.NoError(t, err)
	s, err := world.CreateSketch(plane)
	require.NoError(t, err)
	center := s.CreatePoint(20, 20)
	s.Fix(center)
	s.CreateCircle(center, 5)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	boss, err := Extrude(t.Context(), doc, s, s.Profiles()[0], Distance{D: units.Millimeters(15), Dir: Along})
	require.NoError(t, err)
	body, err := Union(t.Context(), plate, boss)
	require.NoError(t, err)
	return body
}

func internalMeshVolumeRat(mesh *Mesh) *big.Rat {
	total := new(big.Rat)
	for _, triangle := range mesh.n.Triangles {
		var vertices [3][3]*big.Rat
		for k, vertexIndex := range triangle {
			vertex := mesh.n.Vertices[vertexIndex]
			vertices[k] = [3]*big.Rat{
				new(big.Rat).SetFloat64(vertex.X),
				new(big.Rat).SetFloat64(vertex.Y),
				new(big.Rat).SetFloat64(vertex.Z),
			}
		}
		determinant := new(big.Rat)
		for i := range 3 {
			j, k := (i+1)%3, (i+2)%3
			term := new(big.Rat).Sub(
				new(big.Rat).Mul(vertices[1][j], vertices[2][k]),
				new(big.Rat).Mul(vertices[1][k], vertices[2][j]),
			)
			determinant.Add(determinant, term.Mul(term, vertices[0][i]))
		}
		total.Add(total, determinant.Quo(determinant, big.NewRat(6, 1)))
	}
	return total
}

func pointLoftTooth(t *testing.T, doc *Document) *Body {
	t.Helper()
	world := sketch.NewWorld()
	plane, err := world.CreateOffsetPlane(world.YZ(), 8)
	require.NoError(t, err)
	s, err := world.CreateSketch(plane)
	require.NoError(t, err)
	points := []*sketch.Point{
		s.CreatePoint(3, -1), s.CreatePoint(4, -1.5), s.CreatePoint(5, -1),
		s.CreatePoint(5, 1), s.CreatePoint(3, 1),
	}
	for _, point := range points {
		s.Fix(point)
	}
	_, err = s.CreateFitSpline(points[0], points[1], points[2])
	require.NoError(t, err)
	for i := 2; i < len(points); i++ {
		s.CreateLine(points[i], points[(i+1)%len(points)])
	}
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	body, err := LoftFromPoint(t.Context(), doc, r3.Vec{}, s, s.Profiles()[0])
	require.NoError(t, err)
	return body
}

func pointLoftConeTool(t *testing.T, doc *Document, apex float64) *Body {
	t.Helper()
	world := sketch.NewWorld()
	s, err := world.CreateSketch(world.XY())
	require.NoError(t, err)
	points := []*sketch.Point{s.CreatePoint(-2, 0), s.CreatePoint(-2, apex+2), s.CreatePoint(apex, 0)}
	for _, point := range points {
		s.Fix(point)
	}
	for i := range points {
		s.CreateLine(points[i], points[(i+1)%len(points)])
	}
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	tool, err := Revolve(t.Context(), doc, s, s.Profiles()[0],
		SketchLine{Start: planePoint{}, End: planePoint{U: 1}}, FullRevolution{})
	require.NoError(t, err)
	return tool
}

const (
	wedgeRadius = 5.0
	wedgeHeight = 10.0
)

func wedgePlanes(t testing.TB) (*sketch.World, *sketch.Plane, *sketch.Plane) {
	t.Helper()
	world := sketch.NewWorld()
	frame, err := r3.NewFrame(r3.Vec{}, r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	base, err := world.CreatePlaneFromFrame(frame)
	require.NoError(t, err)
	top, err := world.CreateOffsetPlane(base, wedgeHeight)
	require.NoError(t, err)
	return world, base, top
}

func wedgeSplineSketch(t *testing.T, world *sketch.World, plane *sketch.Plane) (*sketch.Sketch, *sketch.Profile) {
	t.Helper()
	s, err := world.CreateSketch(plane)
	require.NoError(t, err)
	origin := s.CreatePoint(0, 0)
	s.Fix(origin)
	points := make([]*sketch.Point, 5)
	for i := range points {
		angle := float64(i) * math.Pi / 8
		point := s.CreatePoint(wedgeRadius*math.Cos(angle), wedgeRadius*math.Sin(angle))
		s.Fix(point)
		points[i] = point
	}
	_, err = s.CreateFitSpline(points...)
	require.NoError(t, err)
	s.CreateLine(origin, points[0])
	s.CreateLine(points[len(points)-1], origin)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	profiles := s.Profiles()
	require.Len(t, profiles, 1)
	return s, profiles[0]
}

func pt(u, v float64) planePoint { return planePoint{U: u, V: v} }

func planeAt(origin r3.Vec) planeRecord {
	return planeRecord{Origin: origin, U: r3.NewVec(1, 0, 0), V: r3.NewVec(0, 1, 0)}
}

func mustFrame(t *testing.T, plane planeRecord) r3.Frame {
	t.Helper()
	frame, err := r3.NewFrame(plane.Origin, plane.U, plane.V)
	require.NoError(t, err)
	return frame
}

func loftRecordAreasOrZero(profile0, profile1 profileRecord) [2]float64 {
	var areas [2]float64
	if integrals, err := profile0.EvaluatorIntegrals(freeform.MomentAreaOrder, nil); err == nil {
		areas[0] = integrals.Area
	}
	if integrals, err := profile1.EvaluatorIntegrals(freeform.MomentAreaOrder, nil); err == nil {
		areas[1] = integrals.Area
	}
	return areas
}

func arcCellFixture(center, radius, start, delta float64) revolvemesh.RevArcCell {
	return revolvemesh.RevArcCell{
		CV: big.NewRat(0, 1).SetFloat64(center), Radius: big.NewRat(0, 1).SetFloat64(radius),
		Th0: big.NewRat(0, 1).SetFloat64(start), Dth: big.NewRat(0, 1).SetFloat64(delta),
	}
}
