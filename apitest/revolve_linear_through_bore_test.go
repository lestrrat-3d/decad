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

func TestLinearRevolveDThroughBore(t *testing.T) {
	doc, w := decad.New(), sketch.NewWorld()
	s, err := w.CreateSketch(w.XZ())
	require.NoError(t, err)
	points := [][2]float64{{0, 0}, {10, 0}, {8, 32}, {0, 32}}
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
	tool := steppedBoreTool(t, doc, w, "D")
	density := units.KilogramsPerCubicMillimeter(0.001)
	hostMass, err := host.MassProperties(t.Context(), density)
	require.NoError(t, err)
	toolMass, err := tool.MassProperties(t.Context(), density)
	require.NoError(t, err)
	result, err := decad.Cut(t.Context(), host, tool)
	require.NoError(t, err)
	require.Equal(t, decad.BodySolid, result.Kind())
	require.Equal(t, 1, countSurface[decad.Cone](result))
	require.Equal(t, 1, countSurface[decad.Cylinder](result))
	require.Zero(t, countSurface[decad.Faceted](result))
	holedCaps, planarWalls := 0, 0
	for _, face := range result.Faces() {
		if _, ok := face.Surface().(decad.Plane); ok {
			if len(face.Loops()) == 2 {
				holedCaps++
			} else {
				planarWalls++
			}
		}
	}
	require.Equal(t, 2, holedCaps)
	require.Equal(t, 1, planarWalls)
	require.Len(t, result.Shells(), 1)
	require.False(t, result.Shells()[0].IsOpen())
	volume, err := result.Volume()
	require.NoError(t, err)
	bound, err := volume.Bound.In(units.CubicMillimeter)
	require.NoError(t, err)
	value, err := volume.Value.In(units.CubicMillimeter)
	require.NoError(t, err)
	area := math.Pi*4.1*4.1 - (4.1*4.1*math.Acos(3.1/4.1) - 3.1*math.Sqrt(4.1*4.1-3.1*3.1))
	want := math.Pi*32/3*(10*10+10*8+8*8) - 32*area
	require.LessOrEqual(t, math.Abs(value-want), bound)
	mass, err := result.MassProperties(t.Context(), density)
	require.NoError(t, err)
	require.LessOrEqual(t, math.Abs(mass.Mass.Value.Base()-want*0.001), mass.Mass.Bound.Base()+1e-13)
	parallelZ := func(source decad.MassProperties) float64 {
		dx := source.Center.Value.X - mass.Center.Value.X
		dy := source.Center.Value.Y - mass.Center.Value.Y
		return source.Inertia.ZZ.Value.Base() + source.Mass.Value.Base()*(dx*dx+dy*dy)
	}
	require.InDelta(t, parallelZ(hostMass)-parallelZ(toolMass), mass.Inertia.ZZ.Value.Base(), 1e-8)
	mesh, err := result.Tessellate(t.Context(), units.Millimeters(0.1), decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
	turn, err := r3.RotationAround(r3.Vec{}, r3.Vec{X: 1, Y: 1}, units.Radians(0.37))
	require.NoError(t, err)
	placed, err := decad.Copy(t.Context(), result, decad.WithCopyPlacement(turn))
	require.NoError(t, err)
	placedMesh, err := placed.Tessellate(t.Context(), units.Millimeters(0.1), decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, placedMesh.BoundaryVerified())
	require.True(t, placedMesh.VolumeVerified())
	placedVolume, err := placed.Volume()
	require.NoError(t, err)
	require.LessOrEqual(t, math.Abs(placedVolume.Value.Base()-want), placedVolume.Bound.Base())
	mirrorFrame, err := r3.NewFrame(r3.Vec{}, r3.Vec{Y: 1}, r3.Vec{Z: 1})
	require.NoError(t, err)
	reflection, err := r3.Reflection(mirrorFrame)
	require.NoError(t, err)
	reflected, err := decad.Copy(t.Context(), result, decad.WithCopyPlacement(reflection))
	require.NoError(t, err)
	reflectedMesh, err := reflected.Tessellate(t.Context(), units.Millimeters(0.1), decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, reflectedMesh.BoundaryVerified())
	require.True(t, reflectedMesh.VolumeVerified())
}
