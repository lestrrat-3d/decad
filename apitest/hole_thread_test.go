package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file is agent-docs/hole-design.md §8's public-surface tests for the
// Thread field of HoleSpec, on the 20 x 20 x 10 plate of hole_test.go: a 4.2 mm
// tap drill and a 0.8 mm pitch, an M5 thread.

const (
	tapRadius = 2.1
	tapPitch  = 0.8
)

// removedPartialInternal is the volume a groove removes out of a bore of the
// given radius when it starts before the bore's start plane and carries turns
// turns: the integral over radius and angle of its axial overlap with
// [0, span] (agent-docs/thread-design.md §5, for an internal groove).
func removedPartialInternal(radius, pitch, span, turns float64) float64 {
	height := threadHeight
	depth := height * 5 / 8 * pitch
	root := radius + depth
	mouth := radius - 0.05*pitch
	flat := pitch / 4
	half := flat/2 + math.Abs(mouth-root)*math.Tan(math.Pi/6)
	first := -half - 0.05*pitch
	const radii, angles = 300, 20000
	dRho := depth / radii
	dPhi := 2 * math.Pi * turns / angles
	total := 0.0
	for i := range radii {
		rho := radius + (float64(i)+0.5)*dRho
		h := flat/2 + (root-rho)*math.Tan(math.Pi/6)
		slice := 0.0
		for j := range angles {
			c := first + pitch*(float64(j)+0.5)*dPhi/(2*math.Pi)
			slice += max(0, min(span, c+h)-max(0, c-h)) * dPhi
		}
		total += rho * slice * dRho
	}
	return total
}

func tapSpec(plate *decad.Body, depth decad.HoleDepth, thread *decad.HoleThread) decad.HoleSpec {
	return decad.HoleSpec{
		Face:     capFace(true, plate),
		At:       r3.Vec{X: 10, Y: 10, Z: 10},
		Diameter: units.Millimeters(2 * tapRadius),
		Depth:    depth,
		Thread:   thread,
	}
}

func TestHoleThreadThroughPlate(t *testing.T) {
	t.Parallel()
	doc, plate := drillPlate(t)
	out, err := decad.Hole(t.Context(), plate, tapSpec(plate, decad.ThroughHole{},
		&decad.HoleThread{Pitch: units.Millimeters(tapPitch)}))
	require.NoError(t, err)
	operations := doc.OperationHistory().Operations()
	require.Equal(t, decad.OperationHole, operations[len(operations)-1].Kind(), "one hole entry holds the drill and the thread")
	require.Equal(t, []*decad.Body{out}, doc.Bodies())
	volume, err := out.Volume()
	require.NoError(t, err)
	requireThreadVolume(t, volume, drillPlateVolume-math.Pi*tapRadius*tapRadius*10-removedInternal(tapRadius, 10, tapPitch))
}

func TestHoleThreadBlindStopsAtTheLength(t *testing.T) {
	t.Parallel()
	_, plate := drillPlate(t)
	out, err := decad.Hole(t.Context(), plate, tapSpec(plate, decad.BlindHole{Depth: units.Millimeters(6)},
		&decad.HoleThread{Pitch: units.Millimeters(tapPitch), Length: units.Millimeters(4)}))
	require.NoError(t, err)
	volume, err := out.Volume()
	require.NoError(t, err)
	turns := (4 + 0.05*tapPitch) / tapPitch
	requireThreadVolume(t, volume,
		drillPlateVolume-math.Pi*tapRadius*tapRadius*6-removedPartialInternal(tapRadius, tapPitch, 4, turns))
}

func TestHoleThreadBlindDefaultsToTheWholeBore(t *testing.T) {
	t.Parallel()
	_, plate := drillPlate(t)
	out, err := decad.Hole(t.Context(), plate, tapSpec(plate, decad.BlindHole{Depth: units.Millimeters(6)},
		&decad.HoleThread{Pitch: units.Millimeters(tapPitch)}))
	require.NoError(t, err)
	volume, err := out.Volume()
	require.NoError(t, err)
	turns := (6 + 0.05*tapPitch) / tapPitch
	requireThreadVolume(t, volume,
		drillPlateVolume-math.Pi*tapRadius*tapRadius*6-removedPartialInternal(tapRadius, tapPitch, 6, turns))
}

func TestHoleThreadStartsAtTheFloorOfACounterbore(t *testing.T) {
	t.Parallel()
	_, plate := drillPlate(t)
	spec := tapSpec(plate, decad.ThroughHole{}, &decad.HoleThread{Pitch: units.Millimeters(tapPitch)})
	spec.Head = decad.Counterbore{Diameter: units.Millimeters(8), Depth: units.Millimeters(3)}
	out, err := decad.Hole(t.Context(), plate, spec)
	require.NoError(t, err)
	volume, err := out.Volume()
	require.NoError(t, err)
	// The 7 mm of bore below the 3 mm counterbore carries the thread.
	requireThreadVolume(t, volume, drillPlateVolume-math.Pi*16*3-math.Pi*tapRadius*tapRadius*7-removedInternal(tapRadius, 7, tapPitch))
}

// grooveCentre is the lowest z in [lo, hi] at which the held mesh is empty at
// radius rho and the angle about the hole axis (10, 10): the centre of the first
// whole empty run in the range.
func grooveCentre(t *testing.T, held *decad.MeshBody, rho, angle, lo, hi float64) float64 {
	t.Helper()
	const step = 0.06
	start := math.NaN()
	seenWall := false // a run that begins at lo may be cut short, so only whole runs count
	for z := lo; z <= hi; z += step {
		where, err := held.LocatePoint(t.Context(), r3.NewVec(10-rho*math.Sin(angle), 10+rho*math.Cos(angle), z))
		require.NoError(t, err)
		switch {
		case where != decad.PointOutside:
			seenWall = true
			if !math.IsNaN(start) {
				return (start + z - step) / 2
			}
		case seenWall && math.IsNaN(start):
			start = z
		}
	}
	require.Fail(t, "no groove found", "angle %v", angle)
	return 0
}

func TestHoleThreadHandedness(t *testing.T) {
	t.Parallel()
	for _, left := range []bool{false, true} {
		_, plate := drillPlate(t)
		out, err := decad.Hole(t.Context(), plate, tapSpec(plate, decad.ThroughHole{},
			&decad.HoleThread{Pitch: units.Millimeters(tapPitch), LeftHand: left}))
		require.NoError(t, err)
		mesh, err := out.Tessellate(t.Context(), units.Millimeters(0.05))
		require.NoError(t, err)
		held, err := decad.NewMeshBody(t.Context(), mesh.Vertices(), mesh.Triangles())
		require.NoError(t, err)
		// A groove, at radius 0.15 mm into the wall, and the same groove's next
		// position a quarter turn about +Z on: a right-handed thread climbs by a
		// quarter pitch, a left-handed one falls by it.
		rho := tapRadius + 0.15
		at0 := grooveCentre(t, held, rho, 0, 4, 4+2*tapPitch)
		// A quarter turn on, the groove has moved a quarter pitch up or down, and
		// the other side's quarter pitch is half a pitch from any groove.
		empty := func(angle, z float64) bool {
			where, err := held.LocatePoint(t.Context(), r3.NewVec(10-rho*math.Sin(angle), 10+rho*math.Cos(angle), z))
			require.NoError(t, err)
			return where == decad.PointOutside
		}
		up, down := empty(math.Pi/2, at0+tapPitch/4), empty(math.Pi/2, at0-tapPitch/4)
		require.Equal(t, !left, up, "left-handed %v: the groove a quarter pitch above its place a quarter turn on", left)
		require.Equal(t, left, down, "left-handed %v: the groove a quarter pitch below its place a quarter turn on", left)
	}
}

func TestHoleThreadRefusals(t *testing.T) {
	t.Parallel()
	_, plate := drillPlate(t)
	pitch := units.Millimeters(tapPitch)
	cases := []struct {
		name string
		spec func() decad.HoleSpec
		want error
	}{
		{"zero pitch", func() decad.HoleSpec {
			return tapSpec(plate, decad.ThroughHole{}, &decad.HoleThread{Pitch: units.Millimeters(0)})
		}, decad.ErrDegenerate},
		{"pitch as an angle", func() decad.HoleSpec {
			return tapSpec(plate, decad.ThroughHole{}, &decad.HoleThread{Pitch: units.Degrees(1)})
		}, decad.ErrUnitKind},
		{"pitch too coarse for the bore", func() decad.HoleSpec {
			return tapSpec(plate, decad.ThroughHole{}, &decad.HoleThread{Pitch: units.Millimeters(50)})
		}, decad.ErrDegenerate},
		{"a length longer than the bore", func() decad.HoleSpec {
			return tapSpec(plate, decad.BlindHole{Depth: units.Millimeters(6)},
				&decad.HoleThread{Pitch: pitch, Length: units.Millimeters(7)})
		}, decad.ErrDegenerate},
		{"a negative length", func() decad.HoleSpec {
			return tapSpec(plate, decad.ThroughHole{}, &decad.HoleThread{Pitch: pitch, Length: units.Millimeters(-1)})
		}, decad.ErrNegativeMagnitude},
		{"a shaped hole", func() decad.HoleSpec {
			spec := tapSpec(plate, decad.ThroughHole{}, &decad.HoleThread{Pitch: pitch})
			spec.Diameter = units.Value{}
			spec.Shape = decad.SlotHole{Length: units.Millimeters(8), Width: units.Millimeters(4), Angle: units.Degrees(0)}
			return spec
		}, decad.ErrDegenerate},
	}
	for _, tc := range cases {
		doc, plate := drillPlate(t)
		spec := tc.spec()
		spec.Face = capFace(true, plate)
		_, err := decad.Hole(t.Context(), plate, spec)
		require.ErrorIs(t, err, tc.want, tc.name)
		require.Equal(t, []*decad.Body{plate}, doc.Bodies(), "%s: a refused hole leaves the plate live and alone", tc.name)
	}
	t.Run("on a cylindrical face", func(t *testing.T) {
		t.Parallel()
		doc := decad.New()
		rod := circleBodyAtZ(t, doc, 0, 5, 0, 20)
		_, err := decad.Hole(t.Context(), rod, decad.HoleSpec{
			Face: decad.Faces(decad.Cylindrical()), At: r3.NewVec(5, 0, 10),
			Diameter: units.Millimeters(3), Depth: decad.ThroughHole{},
			Thread: &decad.HoleThread{Pitch: pitch},
		})
		require.ErrorIs(t, err, decad.ErrUnsupported)
		require.Equal(t, []*decad.Body{rod}, doc.Bodies())
	})
}
