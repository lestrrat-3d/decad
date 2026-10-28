package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file is agent-docs/thread-design.md's public-surface tests for Thread,
// on a 5 mm radius rod 20 mm long and on a bore through a 20 x 20 x 10 block.

const (
	threadPitch  = 1.5
	threadHeight = 0.8660254037844386 // √3/2
)

// removedExternal is the groove volume inside the major radius for a groove
// that runs out of both ends of a rod: at each radius the groove covers
// width(rho)/pitch of the length, whatever the turn.
func removedExternal(radius, length, pitch float64) float64 {
	depth := threadHeight * 17 / 24 * pitch
	return integrateGroove(radius-depth, radius, length, pitch, pitch/8, func(rho float64) float64 { return rho - (radius - depth) })
}

// removedInternal is the same for a bore: the groove lies outside the minor
// radius and narrows toward its root.
func removedInternal(radius, length, pitch float64) float64 {
	depth := threadHeight * 5 / 8 * pitch
	return integrateGroove(radius, radius+depth, length, pitch, pitch/4, func(rho float64) float64 { return radius + depth - rho })
}

func integrateGroove(lo, hi, length, pitch, flat float64, fromRoot func(float64) float64) float64 {
	const steps = 100000
	step := (hi - lo) / steps
	total := 0.0
	for i := range steps {
		rho := lo + (float64(i)+0.5)*step
		width := flat + 2*fromRoot(rho)*math.Tan(math.Pi/6)
		total += 2 * math.Pi * rho * length * width / pitch * step
	}
	return total
}

func requireThreadVolume(t *testing.T, m decad.Measurement, want float64) {
	t.Helper()
	requireThreadVolumeWithin(t, m, want, 0)
}

// requireThreadVolumeWithin is requireThreadVolume for a want that is only known to within extra: the groove formula
// takes the profile in an axial section, and a helix whose lead angle is not zero cuts a section normal to it, which
// differs from the axial one by 1/cos(lead) - 1 of the thread's volume.
func requireThreadVolumeWithin(t *testing.T, m decad.Measurement, want, extra float64) {
	t.Helper()
	require.LessOrEqual(t, math.Abs(m.Value.Base()-want), m.Bound.Base()+extra,
		"volume %v ± %v (+ %v) should enclose %v", m.Value.Base(), m.Bound.Base(), extra, want)
	require.Less(t, m.Bound.Base(), 0.005*want, "the bound stays small enough to mean something")
}

func TestThreadExternalRod(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	rod := circleBodyAtZ(t, doc, 0, 5, 0, 20)
	out, err := decad.Thread(t.Context(), rod, decad.ThreadSpec{
		Face: decad.Faces(decad.Cylindrical()), Pitch: units.Millimeters(threadPitch)})
	require.NoError(t, err)
	require.Equal(t, []*decad.Body{out}, doc.Bodies(), "the rod and the coil leave the document")
	operations := doc.OperationHistory().Operations()
	require.Equal(t, decad.OperationThread, operations[len(operations)-1].Kind())
	volume, err := out.Volume()
	require.NoError(t, err)
	requireThreadVolume(t, volume, math.Pi*25*20-removedExternal(5, 20, threadPitch))
	box, err := out.Bounds()
	require.NoError(t, err)
	require.InDelta(t, 5, box.Max.X, 0.05+box.Bound.Base(), "the thread's crest stays inside the major diameter")
}

func TestThreadInternalBore(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	block := boxBody(t, doc, -10, -10, 10, 10, 10)
	bore := circleBodyAtZ(t, doc, 0, 4.2, -1, 12)
	part, err := decad.Cut(t.Context(), block, bore)
	require.NoError(t, err)
	// The bore ends at the block's faces, so the groove runs out of both ends.
	out, err := decad.Thread(t.Context(), part, decad.ThreadSpec{
		Face: decad.Faces(decad.Cylindrical()), Pitch: units.Millimeters(threadPitch)})
	require.NoError(t, err)
	volume, err := out.Volume()
	require.NoError(t, err)
	requireThreadVolume(t, volume, 20*20*10-math.Pi*4.2*4.2*10-removedInternal(4.2, 10, threadPitch))
}

// The thread is right-handed: seen from the +Z end of a rod, the groove climbs
// as the angle turns counter-clockwise, so a quarter turn later it is a quarter
// pitch higher.
func TestThreadIsRightHanded(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	rod := circleBodyAtZ(t, doc, 0, 5, 0, 20)
	out, err := decad.Thread(t.Context(), rod, decad.ThreadSpec{
		Face: decad.Faces(decad.Cylindrical()), Pitch: units.Millimeters(threadPitch)})
	require.NoError(t, err)
	// Probe a point mid-groove: 0.3 mm above the root, at the groove's centre
	// height, where the walls are 0.27 mm away on either side. At angle 0
	// (direction +Y) the groove centre sits at z = -(half + 0.05·pitch) mod pitch.
	root := 5 - threadHeight*17/24*threadPitch
	half := threadPitch/16 + (threadHeight*17/24+0.05)*threadPitch*math.Tan(math.Pi/6)
	centre := math.Mod(-(half+0.05*threadPitch), threadPitch) + threadPitch*3
	rho := root + 0.3
	// Body.LocatePoint declines a point this close to a faceted wall, so the
	// probe reads the held mesh, whose parity is exact. The walls lie 0.27 mm
	// from the probe, far beyond the mesh bound.
	mesh, err := out.Tessellate(t.Context(), units.Millimeters(0.01))
	require.NoError(t, err)
	held, err := decad.NewMeshBody(t.Context(), mesh.Vertices(), mesh.Triangles())
	require.NoError(t, err)
	require.True(t, held.Report().Closed)
	inGroove := func(angle float64, dz float64) decad.PointLocation {
		z := centre + dz + threadPitch*angle/(2*math.Pi)
		p := r3.NewVec(-rho*math.Sin(angle), rho*math.Cos(angle), z)
		where, err := held.LocatePoint(t.Context(), p)
		require.NoError(t, err)
		return where
	}
	require.Equal(t, decad.PointOutside, inGroove(0, 0), "the groove at angle 0")
	require.Equal(t, decad.PointInside, inGroove(0, threadPitch/2), "the crest half a pitch away at angle 0")
	require.Equal(t, decad.PointOutside, inGroove(math.Pi/2, 0), "the groove a quarter pitch up a quarter turn on")
	require.Equal(t, decad.PointInside, inGroove(math.Pi/2, -threadPitch/4), "not where a left-handed thread would have it")
}

func TestThreadRefusals(t *testing.T) {
	t.Parallel()
	cases := []struct {
		name string
		edit func(*decad.ThreadSpec)
		want error
	}{
		{"no selector", func(s *decad.ThreadSpec) { s.Face = nil }, decad.ErrDegenerate},
		{"zero pitch", func(s *decad.ThreadSpec) { s.Pitch = units.Millimeters(0) }, decad.ErrDegenerate},
		{"negative pitch", func(s *decad.ThreadSpec) { s.Pitch = units.Millimeters(-1) }, decad.ErrNegativeMagnitude},
		{"pitch as an angle", func(s *decad.ThreadSpec) { s.Pitch = units.Degrees(1) }, decad.ErrUnitKind},
		{"pitch too coarse for the rod", func(s *decad.ThreadSpec) { s.Pitch = units.Millimeters(20) }, decad.ErrDegenerate},
		{"two faces", func(s *decad.ThreadSpec) { s.Face = decad.Faces(decad.Planar()) }, decad.ErrDegenerate},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			doc := decad.New()
			rod := circleBodyAtZ(t, doc, 0, 5, 0, 20)
			spec := decad.ThreadSpec{Face: decad.Faces(decad.Cylindrical()), Pitch: units.Millimeters(threadPitch)}
			tc.edit(&spec)
			_, err := decad.Thread(t.Context(), rod, spec)
			require.ErrorIs(t, err, tc.want)
			require.Equal(t, []*decad.Body{rod}, doc.Bodies(), "a refused thread leaves the rod live and alone")
		})
	}
	t.Run("planar face", func(t *testing.T) {
		t.Parallel()
		doc := decad.New()
		block := boxBody(t, doc, 0, 0, 10, 10, 10)
		_, err := decad.Thread(t.Context(), block, decad.ThreadSpec{
			Face: decad.Faces(decad.FaceCreatedBy(decad.CapEnd(block))), Pitch: units.Millimeters(1)})
		require.ErrorIs(t, err, decad.ErrUnsupported)
		require.Equal(t, []*decad.Body{block}, doc.Bodies())
	})
}
