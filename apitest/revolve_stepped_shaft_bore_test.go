package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func steppedBoreHost(t *testing.T, doc *decad.Document, w *sketch.World, radii ...float64) *decad.Body {
	t.Helper()
	s, err := w.CreateSketch(w.XZ())
	require.NoError(t, err)
	points := [][2]float64{{0, 0}, {radii[0], 0}}
	stepLength := 32 / float64(len(radii))
	for i, radius := range radii {
		z := stepLength * float64(i+1)
		points = append(points, [2]float64{radius, z})
		if i+1 < len(radii) {
			points = append(points, [2]float64{radii[i+1], z})
		}
	}
	points = append(points, [2]float64{0, 32})
	vertices := make([]*sketch.Point, len(points))
	for i, point := range points {
		vertices[i] = s.CreatePoint(point[0], point[1])
	}
	s.Fix(vertices[0])
	for i := range vertices {
		s.CreateLine(vertices[i], vertices[(i+1)%len(vertices)])
	}
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	require.Len(t, s.Profiles(), 1)
	host, err := decad.Revolve(t.Context(), doc, s, s.Profiles()[0],
		decad.ConstructionAxis{Dir: r3.Vec{Z: 1}}, decad.FullRevolution{})
	require.NoError(t, err)
	return host
}

func steppedBoreTool(t *testing.T, doc *decad.Document, w *sketch.World, shape string) *decad.Body {
	t.Helper()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	switch shape {
	case "D":
		const radius, flat = 4.1, 3.1
		h := math.Sqrt(radius*radius - flat*flat)
		a, z := s.CreatePoint(flat, h), s.CreatePoint(flat, -h)
		arc := s.CreateArc(s.Origin(), a, z)
		line := s.CreateLine(z, a)
		s.AddConstraint(sketch.NewRadius(arc, radius),
			sketch.NewHorizontalDistance(s.Origin(), a, flat), sketch.NewVertical(line))
	case "round":
		center := s.Origin()
		s.Fix(center)
		s.CreateCircle(center, 4)
	case "slot":
		rect := s.CreateRectangle(-3, -2, 3, 2)
		s.Fix(rect.A)
	default:
		t.Fatalf("unknown bore shape %q", shape)
	}
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	require.Len(t, s.Profiles(), 1)
	tool, err := decad.Extrude(t.Context(), doc, s, s.Profiles()[0],
		decad.Distance{D: units.Millimeters(32), Dir: decad.Along})
	require.NoError(t, err)
	return tool
}

func TestSteppedRevolveAxialThroughBores(t *testing.T) {
	t.Parallel()
	for _, tc := range []struct {
		name         string
		shape        string
		radii        []float64
		area         float64
		cylinders    int
		transformed  bool
		rotated      bool
		antiparallel bool
	}{
		{name: "D bore", shape: "D", radii: []float64{10, 8}, area: math.Pi*4.1*4.1 - (4.1*4.1*math.Acos(3.1/4.1) - 3.1*math.Sqrt(4.1*4.1-3.1*3.1)), cylinders: 3},
		{name: "five-step D bore", shape: "D", radii: []float64{12, 10, 13, 9, 11}, area: math.Pi*4.1*4.1 - (4.1*4.1*math.Acos(3.1/4.1) - 3.1*math.Sqrt(4.1*4.1-3.1*3.1)), cylinders: 6},
		{name: "round bore", shape: "round", radii: []float64{10, 8}, area: math.Pi * 16, cylinders: 3},
		{name: "slot bore", shape: "slot", radii: []float64{10, 8}, area: 24, cylinders: 2},
		{name: "translated D bore", shape: "D", radii: []float64{10, 8}, area: math.Pi*4.1*4.1 - (4.1*4.1*math.Acos(3.1/4.1) - 3.1*math.Sqrt(4.1*4.1-3.1*3.1)), cylinders: 3, transformed: true},
		{name: "rotated D bore", shape: "D", radii: []float64{10, 8}, area: math.Pi*4.1*4.1 - (4.1*4.1*math.Acos(3.1/4.1) - 3.1*math.Sqrt(4.1*4.1-3.1*3.1)), cylinders: 3, transformed: true, rotated: true},
		{name: "antiparallel D bore", shape: "D", radii: []float64{10, 8}, area: math.Pi*4.1*4.1 - (4.1*4.1*math.Acos(3.1/4.1) - 3.1*math.Sqrt(4.1*4.1-3.1*3.1)), cylinders: 3, antiparallel: true},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			doc, w := decad.New(), sketch.NewWorld()
			host := steppedBoreHost(t, doc, w, tc.radii...)
			tool := steppedBoreTool(t, doc, w, tc.shape)
			if tc.antiparallel {
				c, s := math.Cos(math.Pi/3), math.Sin(math.Pi/3)
				hostFrame, err := r3.NewFrame(r3.Vec{Y: -4}, r3.Vec{X: c, Z: s}, r3.Vec{X: s, Z: -c})
				require.NoError(t, err)
				toolFrame, err := r3.NewFrame(r3.Vec{Y: 28}, hostFrame.U(), hostFrame.V().Scale(-1))
				require.NoError(t, err)
				hostTransform, err := r3.FromFrame(hostFrame)
				require.NoError(t, err)
				toolTransform, err := r3.FromFrame(toolFrame)
				require.NoError(t, err)
				host, err = decad.Transform(t.Context(), host, hostTransform)
				require.NoError(t, err)
				tool, err = decad.Transform(t.Context(), tool, toolTransform)
				require.NoError(t, err)
			}
			if tc.transformed {
				move, err := r3.Translation(r3.Vec{X: 20, Y: -3, Z: 9})
				if tc.rotated {
					move, err = r3.RotationAround(r3.Vec{}, r3.Vec{X: 1}, units.Degrees(90))
				}
				require.NoError(t, err)
				host, err = decad.Transform(t.Context(), host, move)
				require.NoError(t, err)
				tool, err = decad.Transform(t.Context(), tool, move)
				require.NoError(t, err)
			}
			result, err := decad.Cut(t.Context(), host, tool)
			require.NoError(t, err)
			require.Equal(t, tc.cylinders, countSurface[decad.Cylinder](result))
			require.Zero(t, countSurface[decad.Faceted](result))
			for _, face := range result.Faces() {
				_, faceted := face.Surface().(decad.Faceted)
				require.False(t, faceted, "axial bore keeps an analytic boundary")
			}
			volume, err := result.Volume()
			require.NoError(t, err)
			bound, err := volume.Bound.In(units.CubicMillimeter)
			require.NoError(t, err)
			require.Less(t, bound, 1e-6)
			value, err := volume.Value.In(units.CubicMillimeter)
			require.NoError(t, err)
			hostArea := 0.0
			stepLength := 32 / float64(len(tc.radii))
			for _, radius := range tc.radii {
				hostArea += math.Pi * radius * radius * stepLength
			}
			want := hostArea - tc.area*32
			require.LessOrEqual(t, math.Abs(value-want), bound+1e-10)
			mesh, err := result.Tessellate(t.Context(), units.Millimeters(0.1),
				decad.WithVerification(decad.VerifyAll))
			require.NoError(t, err)
			require.True(t, mesh.BoundaryVerified())
			require.True(t, mesh.VolumeVerified())
		})
	}
}

func countSurface[T any](body *decad.Body) int {
	count := 0
	for _, face := range body.Faces() {
		if _, ok := face.Surface().(T); ok {
			count++
		}
	}
	return count
}
