package decad

import (
	"math"
	"slices"
	"strings"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func reachTestSection(t *testing.T, world *sketch.World, plane *sketch.Plane,
	turn float64) (*sketch.Sketch, *sketch.Profile) {
	t.Helper()
	s, err := world.CreateSketch(plane)
	require.NoError(t, err)
	center := s.CreatePoint(0, 0)
	s.Fix(center)
	polar := func(radius, angle float64) *sketch.Point {
		point := s.CreatePoint(radius*math.Cos(angle+turn), radius*math.Sin(angle+turn))
		s.Fix(point)
		return point
	}
	rightRoot, leftRoot := polar(4, -0.12), polar(4, 0.12)
	right := make([]*sketch.Point, 3)
	left := make([]*sketch.Point, 3)
	for i := range right {
		radius := 5 + float64(i)
		angle := 0.09 - 0.02*float64(i)
		right[i] = polar(radius, -angle)
		left[i] = polar(radius, angle)
	}
	s.CreateLine(rightRoot, right[0])
	_, err = s.CreateFitSpline(right...)
	require.NoError(t, err)
	s.CreateArc(center, right[len(right)-1], left[len(left)-1])
	_, err = s.CreateFitSpline(left...)
	require.NoError(t, err)
	s.CreateLine(left[0], leftRoot)
	s.CreateArc(center, rightRoot, leftRoot)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	profiles := s.Profiles()
	require.Len(t, profiles, 1)
	require.True(t, profiles[0].Valid)
	return s, profiles[0]
}

func reachTestCircle(t *testing.T, doc *Document, radius, z, height float64) *Body {
	t.Helper()
	w := sketch.NewWorld()
	plane, err := w.CreateOffsetPlane(w.XY(), z)
	require.NoError(t, err)
	s, err := w.CreateSketch(plane)
	require.NoError(t, err)
	center := s.CreatePoint(0, 0)
	s.Fix(center)
	s.CreateCircle(center, radius)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	body, err := Extrude(t.Context(), doc, s, s.Profiles()[0],
		Distance{D: units.Millimeters(height), Dir: Along})
	require.NoError(t, err)
	return body
}

// reachTestInput builds the loft-mirror bore chamfer and returns the input of
// its reach proofs.
func reachTestInput(t *testing.T) loftCylinderChamferReachInput {
	t.Helper()
	doc := New()
	world := sketch.NewWorld()
	middleFrame, err := r3.NewFrame(r3.Vec{Z: 1}, r3.Vec{X: 1}, r3.Vec{Y: 1})
	require.NoError(t, err)
	middlePlane, err := world.CreatePlaneFromFrame(middleFrame)
	require.NoError(t, err)
	bottomSketch, bottomProfile := reachTestSection(t, world, world.XY(), 0)
	middleSketch, middleProfile := reachTestSection(t, world, middlePlane, 0.03)
	lower, err := Loft(t.Context(), doc, bottomSketch, bottomProfile, middleSketch, middleProfile)
	require.NoError(t, err)
	upper, err := MirroredCopy(t.Context(), lower, MirrorFrame{Frame: middleFrame})
	require.NoError(t, err)
	pair, err := Union(t.Context(), lower, upper)
	require.NoError(t, err)
	pattern := []*Body{pair}
	for i := 7; i >= 1; i-- {
		angle := units.Degrees(float64(45 * i))
		if i == 5 {
			angle = units.Radians(10 * 2 * math.Pi / 17)
		}
		turn, err := r3.Rotation(r3.Vec{Z: 1}, angle)
		require.NoError(t, err)
		turned, err := Copy(t.Context(), pair, WithCopyPlacement(turn))
		require.NoError(t, err)
		pattern = append(pattern, turned)
	}
	joined, err := Union(t.Context(), reachTestCircle(t, doc, 4, 0, 2), pattern...)
	require.NoError(t, err)
	cut, err := Cut(t.Context(), joined, reachTestCircle(t, doc, 0.5, -1, 4))
	require.NoError(t, err)
	var captured *loftCylinderChamferReachInput
	loftCylinderChamferReachHook = func(in *loftCylinderChamferReachInput) {
		// Later build steps rewrite the mesh in place, so keep a snapshot.
		copied := *in
		copied.mesh = &joinedMesh{
			Verts: slices.Clone(in.mesh.Verts), Tris: slices.Clone(in.mesh.Tris),
			Src: slices.Clone(in.mesh.Src), Groups: slices.Clone(in.mesh.Groups),
		}
		copied.ideal = slices.Clone(in.ideal)
		captured = &copied
	}
	t.Cleanup(func() { loftCylinderChamferReachHook = nil })
	outerCaps := Edges(OuterLoopOf(CapStart(cut))).Or(OuterLoopOf(CapEnd(cut)))
	_, err = Chamfer(t.Context(), cut, outerCaps, units.Millimeters(0.03))
	require.NoError(t, err)
	loftCylinderChamferReachHook = nil
	require.NotNil(t, captured)
	return *captured
}

func reachTestRun(t *testing.T, in loftCylinderChamferReachInput, workers int) error {
	t.Helper()
	return loftCylinderChamferReach(t.Context(), &in, workers)
}

func TestLoftCylinderChamferReachFanOutMatchesSerial(t *testing.T) {
	in := reachTestInput(t)
	require.NoError(t, reachTestRun(t, in, 4))

	// The near band is the slow item and the facets are the subject of the
	// next case, so it runs with empty bands, which skip the near band.
	noBands := in
	noBands.bands = [2][2]int{}

	t.Run("facets fail", func(t *testing.T) {
		failed := 0
		for _, clip := range []float64{0.1, 1.6} {
			raised := noBands
			raised.clipBound = clip
			raised.allowance = in.allowance + (clip - in.clipBound)
			serial := reachTestRun(t, raised, 1)
			fanned := reachTestRun(t, raised, 4)
			if serial == nil {
				require.NoError(t, fanned)
				continue
			}
			require.EqualError(t, fanned, serial.Error())
			if strings.Contains(serial.Error(), "retained middle") {
				failed++
			}
		}
		require.Positive(t, failed)
	})

	t.Run("near band fails first", func(t *testing.T) {
		starved := in
		starved.allowance = in.clipBound / 2
		serial := reachTestRun(t, starved, 1)
		require.ErrorIs(t, serial, ErrUnsupported)
		require.EqualError(t, reachTestRun(t, starved, 4), serial.Error())
		starved.clipBound = in.clipBound * 4
		starved.allowance = starved.clipBound / 2
		serial = reachTestRun(t, starved, 1)
		require.ErrorIs(t, serial, ErrUnsupported)
		require.EqualError(t, reachTestRun(t, starved, 4), serial.Error())
	})
}

// TestLoftCylinderChamferReachSerialPasses is the serial full run of the
// reach fan-out check, split out so each test fits one CI job.
func TestLoftCylinderChamferReachSerialPasses(t *testing.T) {
	in := reachTestInput(t)
	require.NoError(t, reachTestRun(t, in, 1))
}

// TestLoftCylinderChamferReachGridLimitFallsBack runs the fan-out with a
// grid work limit that stops it mid-way, split out of the reach fan-out check.
func TestLoftCylinderChamferReachGridLimitFallsBack(t *testing.T) {
	in := reachTestInput(t)
	noBands := in
	noBands.bands = [2][2]int{}

	t.Run("grid limit falls back mid-way", func(t *testing.T) {
		t.Cleanup(func() { middleSidePrepHook = nil })
		// This limit lets the shared grid serve about 1 800 facets, so the
		// fan-out replays them and then re-runs the rest serially.
		middleSidePrepHook = func(prep *middleSidePrep) { prep.grid.LimitWork(1 << 18) }
		serial := reachTestRun(t, noBands, 1)
		require.ErrorContains(t, serial, "retained middle facet 1873")
		require.EqualError(t, reachTestRun(t, noBands, 4), serial.Error())
	})
}
