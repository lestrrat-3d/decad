package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"

	"github.com/lestrrat-3d/decad"
)

const (
	twistBoreLength = 15.4
	twistBoreWidth  = 4.75
	// twistBoreSpan is how far the tool is swept along +Z.
	twistBoreSpan = 6.0
)

// twistBoreTool sweeps the screw gear's 15.4 x 4.75 mm centred rectangle along
// +Z from the origin XY plane through twistBoreSpan mm with a twist of deg degrees.
func twistBoreTool(t *testing.T, doc *decad.Document, deg float64) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	hx, hy := twistBoreLength/2, twistBoreWidth/2
	r := s.CreateRectangle(-hx, -hy, hx, hy)
	s.Fix(r.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	path, err := decad.NewPath(r3.Vec{}, decad.LineTo{End: r3.NewVec(0, 0, twistBoreSpan)})
	require.NoError(t, err)
	body, err := decad.Sweep(t.Context(), doc, s, s.Profiles()[0], path,
		decad.WithSweepTwist(units.Degrees(deg)))
	require.NoError(t, err)
	return body
}

// twistBoreOverlap integrates, independently of the evaluator, the volume
// shared by the twisted rectangle and an annulus of radii inner and outer
// about the Z axis. The rectangle sits at (cx, 0) and is clipped to
// z in [zLo, zHi]; its section turns by deg degrees per length of its own
// sweep, starting at zStart. It samples each horizontal slice on a midpoint
// grid and the slices by the midpoint rule.
func twistBoreOverlap(length, deg, cx, zStart, zLo, zHi, inner, outer float64) float64 {
	const slices, nu, nv = 120, 770, 238
	hx, hy := twistBoreLength/2, twistBoreWidth/2
	du, dv := twistBoreLength/nu, twistBoreWidth/nv
	total := 0.0
	dz := (zHi - zLo) / slices
	for k := range slices {
		z := zLo + (float64(k)+0.5)*dz
		a := deg * math.Pi / 180 * (z - zStart) / length
		sin, cos := math.Sincos(a)
		count := 0
		for i := range nu {
			u := -hx + (float64(i)+0.5)*du
			for j := range nv {
				v := -hy + (float64(j)+0.5)*dv
				x, y := cx+u*cos-v*sin, u*sin+v*cos
				if r2 := x*x + y*y; r2 >= inner*inner && r2 <= outer*outer {
					count++
				}
			}
		}
		total += float64(count) * du * dv * dz
	}
	return total
}

// A twisted Sweep of at most one radian along one line holds each wall as two
// flat triangles, so its own proven displacement is wider than the 4 mm plate
// it bores. The Cut restates the same sweep as its stationed refinement.
func TestCutTwistedSweepThroughBothCaps(t *testing.T) {
	t.Parallel()
	for _, deg := range []float64{30, 60, 87} {
		doc := decad.New()
		tool := twistBoreTool(t, doc, deg)
		plate := circleBodyAtZ(t, doc, 0, 18.5, 1, 4)
		plateVolume, err := plate.Volume()
		require.NoError(t, err)

		out, err := decad.Cut(t.Context(), plate, tool)
		require.NoError(t, err, "%g degrees", deg)
		require.True(t, out.IsSolid())
		want := plateVolume.Value.Base() - twistBoreLength*twistBoreWidth*4
		volume, err := out.Volume()
		require.NoError(t, err)
		// The rectangle stays well inside the plate for every angle, so the
		// removed volume is its area times the 4 mm plate height.
		require.LessOrEqual(t, math.Abs(volume.Value.Base()-want),
			volume.Bound.Base()+plateVolume.Bound.Base(), "%g degrees: %v want %v", deg, volume, want)
		require.Less(t, volume.Bound.Base(), 0.1*want, "%g degrees", deg)

		mesh, err := out.Tessellate(t.Context(), units.Millimeters(3), decad.WithVerification(decad.VerifyAll))
		require.NoError(t, err, "%g degrees", deg)
		require.True(t, mesh.BoundaryVerified())
		require.True(t, mesh.VolumeVerified())
	}
}

// The tool crosses the tube wall off-axis, so its twisted walls meet the
// outer and inner cylinders obliquely and never reach either cap.
func TestCutTwistedSweepCrossesTubeWall(t *testing.T) {
	t.Parallel()
	const outer, inner, height, length, lift = 18.5, 12.5, 24.0, 6.0, 9.0
	setup := func(deg, offset float64) (*decad.Body, *decad.Body) {
		doc := decad.New()
		tube := twistBoreTube(t, doc, outer, inner, height)
		tool := twistBoreTool(t, doc, deg)
		move, err := r3.Translation(r3.NewVec(offset, 0, lift))
		require.NoError(t, err)
		tool, err = decad.Transform(t.Context(), tool, move)
		require.NoError(t, err)
		return tube, tool
	}
	for _, tc := range []struct{ deg, offset float64 }{{30, 15.5}, {87, 15.5}, {60, 16.5}} {
		overlap := twistBoreOverlap(length, tc.deg, tc.offset, lift, lift, lift+length, inner, outer)
		require.Greater(t, overlap, 100.0)

		// The piece the Cut removes is what the same tool leaves in the tube,
		// so the Intersect volume must enclose the independent overlap.
		tube, tool := setup(tc.deg, tc.offset)
		kept, err := decad.Intersect(t.Context(), tube, tool)
		require.NoError(t, err, "%+v", tc)
		keptVolume, err := kept.Volume()
		require.NoError(t, err)
		require.LessOrEqual(t, math.Abs(keptVolume.Value.Base()-overlap),
			keptVolume.Bound.Base()+0.005*overlap, "%+v: intersect %v overlap %v", tc, keptVolume, overlap)

		tube, tool = setup(tc.deg, tc.offset)
		tubeVolume, err := tube.Volume()
		require.NoError(t, err)
		out, err := decad.Cut(t.Context(), tube, tool)
		require.NoError(t, err, "%+v", tc)
		require.True(t, out.IsSolid())
		want := tubeVolume.Value.Base() - overlap
		volume, err := out.Volume()
		require.NoError(t, err)
		// 0.5% of the overlap covers the independent integral's own sampling.
		slack := volume.Bound.Base() + tubeVolume.Bound.Base() + 0.005*overlap
		require.LessOrEqual(t, math.Abs(volume.Value.Base()-want), slack,
			"%+v: %v want %v (overlap %v)", tc, volume, want, overlap)

		mesh, err := out.Tessellate(t.Context(), units.Millimeters(3), decad.WithVerification(decad.VerifyAll))
		require.NoError(t, err, "%+v", tc)
		require.True(t, mesh.BoundaryVerified())
		require.True(t, mesh.VolumeVerified())
	}
}

// At 60 degrees and 15.05 mm the tool's far end turns to 18.54 mm from the
// tube axis and never reaches the 18.5 mm outer wall. That 0.04 mm true gap is
// narrower than the displacement the two held boundaries can place a surface
// within, so the Cut refuses rather than let the chords decide it.
func TestCutTwistedSweepRefusesTubeWallNearMiss(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	tube := twistBoreTube(t, doc, 18.5, 12.5, 24)
	tool := twistBoreTool(t, doc, 60)
	move, err := r3.Translation(r3.NewVec(15.05, 0, 9))
	require.NoError(t, err)
	tool, err = decad.Transform(t.Context(), tool, move)
	require.NoError(t, err)

	_, err = decad.Cut(t.Context(), tube, tool)
	require.ErrorIs(t, err, decad.ErrUnsupported)
	var booleanErr *decad.BooleanError
	require.ErrorAs(t, err, &booleanErr)
	require.Equal(t, decad.BooleanUnsupportedContact, booleanErr.Code)
	require.Len(t, doc.Bodies(), 2)
}

//nolint:unparam // the radii are inputs so a case can bore a different tube
func twistBoreTube(t *testing.T, doc *decad.Document, outer, inner, height float64) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	c := s.CreatePoint(0, 0)
	s.Fix(c)
	s.CreateCircle(c, outer)
	s.CreateCircle(c, inner)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	for _, p := range s.Profiles() {
		if len(p.Outer) == 1 && len(p.Holes) == 1 {
			body, err := decad.Extrude(t.Context(), doc, s, p, decad.Distance{D: units.Millimeters(height), Dir: decad.Along})
			require.NoError(t, err)
			return body
		}
	}
	require.Fail(t, "the annulus region should exist")
	return nil
}

// twistBoreTiltedOverlap is twistBoreOverlap for a tool turned tilt degrees
// about the Y axis through its own midpoint before it moves to (cx, 0, lift),
// with the tube spanning z in [0, height]. It samples the tool's own
// coordinates, so the rigid placement leaves the volume element unchanged.
func twistBoreTiltedOverlap(length, deg, tilt, cx, lift, height, inner, outer float64) float64 {
	const nu, nv, nw = 220, 68, 120
	hx, hy := twistBoreLength/2, twistBoreWidth/2
	du, dv, dw := twistBoreLength/nu, twistBoreWidth/nv, length/nw
	sinT, cosT := math.Sincos(tilt * math.Pi / 180)
	count := 0
	for k := range nw {
		w := (float64(k) + 0.5) * dw
		sin, cos := math.Sincos(deg * math.Pi / 180 * w / length)
		for i := range nu {
			u := -hx + (float64(i)+0.5)*du
			for j := range nv {
				v := -hy + (float64(j)+0.5)*dv
				lx, ly, lz := u*cos-v*sin, u*sin+v*cos, w-length/2
				x := cx + lx*cosT + lz*sinT
				y := ly
				z := lift + length/2 - lx*sinT + lz*cosT
				if r2 := x*x + y*y; r2 >= inner*inner && r2 <= outer*outer && z >= 0 && z <= height {
					count++
				}
			}
		}
	}
	return float64(count) * du * dv * dw
}

// The screw gear's bores lie along tilted axes, so the tool arrives through a
// rotation as well as a translation.
func TestCutTwistedSweepTiltedAcrossTubeWall(t *testing.T) {
	t.Parallel()
	const outer, inner, height, length, lift, offset, tilt, deg = 18.5, 12.5, 24.0, 6.0, 9.0, 16.5, 20.0, 30.0
	setup := func() (*decad.Body, *decad.Body) {
		doc := decad.New()
		tube := twistBoreTube(t, doc, outer, inner, height)
		tool := twistBoreTool(t, doc, deg)
		turn, err := r3.RotationAround(r3.NewVec(0, 0, length/2), r3.NewVec(0, 1, 0), units.Degrees(tilt))
		require.NoError(t, err)
		tool, err = decad.Transform(t.Context(), tool, turn)
		require.NoError(t, err)
		move, err := r3.Translation(r3.NewVec(offset, 0, lift))
		require.NoError(t, err)
		tool, err = decad.Transform(t.Context(), tool, move)
		require.NoError(t, err)
		return tube, tool
	}
	overlap := twistBoreTiltedOverlap(length, deg, tilt, offset, lift, height, inner, outer)
	require.Greater(t, overlap, 100.0)

	tube, tool := setup()
	kept, err := decad.Intersect(t.Context(), tube, tool)
	require.NoError(t, err)
	keptVolume, err := kept.Volume()
	require.NoError(t, err)
	require.LessOrEqual(t, math.Abs(keptVolume.Value.Base()-overlap),
		keptVolume.Bound.Base()+0.01*overlap, "intersect %v overlap %v", keptVolume, overlap)

	tube, tool = setup()
	tubeVolume, err := tube.Volume()
	require.NoError(t, err)
	out, err := decad.Cut(t.Context(), tube, tool)
	require.NoError(t, err)
	require.True(t, out.IsSolid())
	volume, err := out.Volume()
	require.NoError(t, err)
	want := tubeVolume.Value.Base() - overlap
	require.LessOrEqual(t, math.Abs(volume.Value.Base()-want),
		volume.Bound.Base()+tubeVolume.Bound.Base()+0.01*overlap, "%v want %v", volume, want)
	mesh, err := out.Tessellate(t.Context(), units.Millimeters(3), decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
}
