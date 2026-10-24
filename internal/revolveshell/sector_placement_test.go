package revolveshell_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/coneaxis"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/revolveshell"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// generalAxes are the axes the general-axis placement tests run about: an
// irrational-length axis and one exactly congruent to +X.
var generalAxes = map[string]r3.Vec{"(1,2,0)": {X: 1, Y: 2}, "(0,1,0)": {Y: 1}}

func axisValue(t *testing.T, v r3.Vec) coneaxis.Axis {
	t.Helper()
	axis, ok := coneaxis.New(v)
	require.True(t, ok)
	return axis
}

func unitOf(v r3.Vec) r3.Vec {
	n := math.Sqrt(v.X*v.X + v.Y*v.Y + v.Z*v.Z)
	return r3.Vec{X: v.X / n, Y: v.Y / n, Z: v.Z / n}
}

func turnAbout(t *testing.T, axis r3.Vec, radians float64) r3.Transform {
	t.Helper()
	turn, err := r3.Rotation(unitOf(axis), units.Radians(radians))
	require.NoError(t, err)
	return turn
}

// tiltedTurnAbout is a turn about axis by radians followed by a rotation by
// tilt about the unit vector along axis × at, which is perpendicular to the
// axis. The axis then moves by the angle tilt, and a point at on a cone about
// the axis with its apex at the origin leaves that cone by tilt·|at|.
func tiltedTurnAbout(t *testing.T, axis r3.Vec, radians, tilt float64, at r3.Vec) r3.Transform {
	t.Helper()
	turn := turnAbout(t, axis, radians)
	lean, err := r3.Rotation(unitOf(unitOf(axis).Cross(turn.Apply(at))), units.Radians(tilt))
	require.NoError(t, err)
	out, err := turn.Then(lean)
	require.NoError(t, err)
	return out
}

// prismAbout returns the six vertices of a triangular prism beside axis: at
// two stations 4 and 6 mm along it, a triangle 1 to 4 mm out along the unit
// vector axis × Z and 2 mm to each side along Z.
func prismAbout(axis r3.Vec) []r3.Vec {
	a := unitOf(axis)
	radial := unitOf(a.Cross(r3.Vec{Z: 1}))
	side := r3.Vec{Z: 1}
	vertices := make([]r3.Vec, 0, 6)
	for _, x := range []float64{4, 6} {
		station := a.Scale(x)
		vertices = append(vertices,
			station.Add(radial),
			station.Add(radial.Scale(4)).Add(side.Scale(-2)),
			station.Add(radial.Scale(4)).Add(side.Scale(2)))
	}
	return vertices
}

func placedPrism(vertices []r3.Vec, motion r3.Transform) facetedrecord.Payload {
	placed := make([]r3.Vec, len(vertices))
	for i, vertex := range vertices {
		placed[i] = motion.Apply(vertex)
	}
	return facetedrecord.Payload{Verts: placed, MeshBound: 0.01, Xform: motion,
		RevolveLoftSector: &facetedrecord.RevolveLoftSectorPlacement{Motion: motion}}
}

func TestRevolveSectorSourcesSeparateAboutAxis(t *testing.T) {
	t.Parallel()
	for name, axisVec := range generalAxes {
		t.Run(name, func(t *testing.T) {
			t.Parallel()
			axis := axisValue(t, axisVec)
			vertices := prismAbout(axisVec)
			first := facetedrecord.Payload{Verts: vertices, MeshBound: 0.01, Xform: r3.Identity()}
			separate := func(motion r3.Transform) bool {
				return revolveshell.RevolveSectorSourcesSeparate(
					[]facetedrecord.Payload{first, placedPrism(vertices, motion)}, axis)
			}

			// A quarter turn about the axis separates the two prisms along the
			// box-centre difference with its axis component removed.
			require.True(t, separate(turnAbout(t, axisVec, math.Pi/2)))
			// A five-degree turn overlaps.
			require.False(t, separate(turnAbout(t, axisVec, 5*math.Pi/180)))

			// Each placement AxisRotation declines refuses, however far apart
			// the copies are.
			tilted := tiltedTurnAbout(t, axisVec, math.Pi/2, 0x1p-40, vertices[0])
			require.False(t, separate(tilted), "a tilt past DriftSinLimit")
			shift, err := r3.Translation(r3.Vec{Z: 1})
			require.NoError(t, err)
			translated, err := turnAbout(t, axisVec, math.Pi/2).Then(shift)
			require.NoError(t, err)
			require.False(t, separate(translated), "a translated copy")
			mirror, err := r3.NewFrame(r3.Vec{}, unitOf(axisVec), r3.Vec{Z: 1})
			require.NoError(t, err)
			reflected, err := r3.Reflection(mirror)
			require.NoError(t, err)
			require.False(t, separate(reflected), "a reflected copy")
			obtuse := turnAbout(t, r3.Vec{Z: 1}, math.Pi)
			require.False(t, separate(obtuse), "a copy that reverses the axis")
		})
	}
}
