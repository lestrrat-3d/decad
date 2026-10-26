package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file is agent-docs/thread-design.md §5's public-surface tests for the
// left-hand and the partial-length thread, on the rod of thread_test.go.

// threadGroove is the external groove of the 5 mm rod: the radius of its root
// and where its first centre sits.
func threadGroove() (root, first float64) {
	root = 5 - threadHeight*17/24*threadPitch
	half := threadPitch/16 + (threadHeight*17/24+0.05)*threadPitch*math.Tan(math.Pi/6)
	return root, -half - 0.05*threadPitch
}

// removedPartial is the volume a groove removes from the rod when it starts
// before z = 0 and carries turns turns, found by integrating the groove's axial
// overlap with [0, length] over radius and angle. A right-hand groove's centre
// rises by one pitch per turn; the left-hand one turns the other way and
// removes the same volume.
func removedPartial(length, turns float64) float64 {
	root, first := threadGroove()
	const radii, angles = 400, 20000
	dRho := (5 - root) / radii
	dPhi := 2 * math.Pi * turns / angles
	total := 0.0
	for i := range radii {
		rho := root + (float64(i)+0.5)*dRho
		h := threadPitch/16 + (rho-root)*math.Tan(math.Pi/6)
		slice := 0.0
		for j := range angles {
			c := first + threadPitch*(float64(j)+0.5)*dPhi/(2*math.Pi)
			slice += max(0, min(length, c+h)-max(0, c-h)) * dPhi
		}
		total += rho * slice * dRho
	}
	return total
}

func heldMesh(t *testing.T, b *decad.Body) *decad.MeshBody {
	t.Helper()
	mesh, err := b.Tessellate(t.Context(), units.Millimeters(0.01))
	require.NoError(t, err)
	held, err := decad.NewMeshBody(t.Context(), mesh.Vertices(), mesh.Triangles())
	require.NoError(t, err)
	require.True(t, held.Report().Closed)
	return held
}

func TestThreadLeftHandRod(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	rod := circleBodyAtZ(t, doc, 0, 5, 0, 20)
	out, err := decad.Thread(t.Context(), rod, decad.ThreadSpec{
		Face: decad.Faces(decad.Cylindrical()), Pitch: units.Millimeters(threadPitch), LeftHand: true})
	require.NoError(t, err)
	volume, err := out.Volume()
	require.NoError(t, err)
	requireThreadVolume(t, volume, math.Pi*25*20-removedExternal(5, 20, threadPitch))

	// Seen from +Z the groove of a left-hand thread descends as the angle turns
	// counter-clockwise: a quarter turn on it is a quarter pitch lower.
	root, first := threadGroove()
	centre := math.Mod(first, threadPitch) + threadPitch*3
	rho := root + 0.3
	held := heldMesh(t, out)
	at := func(angle, z float64) decad.PointLocation {
		where, err := held.LocatePoint(t.Context(), r3.NewVec(-rho*math.Sin(angle), rho*math.Cos(angle), z))
		require.NoError(t, err)
		return where
	}
	require.Equal(t, decad.PointOutside, at(0, centre), "the groove at angle 0")
	require.Equal(t, decad.PointOutside, at(math.Pi/2, centre-threadPitch/4), "a quarter pitch lower a quarter turn on")
	require.Equal(t, decad.PointInside, at(math.Pi/2, centre+threadPitch/4), "not where a right-handed thread would have it")
}

func TestThreadPartialLengthStopsShortOfTheFaceEnd(t *testing.T) {
	t.Parallel()
	const threaded = 8.0
	for _, hand := range []bool{false, true} {
		doc := decad.New()
		rod := circleBodyAtZ(t, doc, 0, 5, 0, 20)
		out, err := decad.Thread(t.Context(), rod, decad.ThreadSpec{
			Face: decad.Faces(decad.Cylindrical()), Pitch: units.Millimeters(threadPitch),
			LeftHand: hand, Length: units.Millimeters(threaded)})
		require.NoError(t, err)
		volume, err := out.Volume()
		require.NoError(t, err)
		turns := (threaded + 0.05*threadPitch) / threadPitch
		requireThreadVolume(t, volume, math.Pi*25*20-removedPartial(20, turns))

		// Beyond the threaded length the rod is whole at every angle.
		root, _ := threadGroove()
		held := heldMesh(t, out)
		for _, angle := range []float64{0, math.Pi / 2, math.Pi, 3 * math.Pi / 2} {
			rho := root + 0.3
			where, err := held.LocatePoint(t.Context(), r3.NewVec(-rho*math.Sin(angle), rho*math.Cos(angle), 14))
			require.NoError(t, err)
			require.Equal(t, decad.PointInside, where, "left-hand %v, angle %v, z = 14 is past the thread", hand, angle)
		}
	}
}

func TestThreadLengthRefusals(t *testing.T) {
	t.Parallel()
	cases := []struct {
		name   string
		length units.Value
		want   error
	}{
		{"zero length", units.Millimeters(0), decad.ErrDegenerate},
		{"negative length", units.Millimeters(-2), decad.ErrNegativeMagnitude},
		{"length as an angle", units.Degrees(2), decad.ErrUnitKind},
		{"longer than the face", units.Millimeters(21), decad.ErrDegenerate},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			doc := decad.New()
			rod := circleBodyAtZ(t, doc, 0, 5, 0, 20)
			_, err := decad.Thread(t.Context(), rod, decad.ThreadSpec{
				Face: decad.Faces(decad.Cylindrical()), Pitch: units.Millimeters(threadPitch), Length: tc.length})
			require.ErrorIs(t, err, tc.want)
			require.Equal(t, []*decad.Body{rod}, doc.Bodies(), "a refused thread leaves the rod live and alone")
		})
	}
}
