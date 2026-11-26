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

// lineChordMeridian is a sketch on the XZ plane through the world origin,
// U along X and V along Z, so a profile at positive U revolves or coils about
// world Z through lineChordAxis.
func lineChordMeridian(t *testing.T, loop [][2]float64) (*sketch.Sketch, *sketch.Profile) {
	t.Helper()
	frame, err := r3.NewFrame(r3.Vec{}, r3.Vec{X: 1}, r3.Vec{Z: 1})
	require.NoError(t, err)
	return lineChordSketch(t, frame, loop)
}

func lineChordSketch(t *testing.T, frame r3.Frame, loop [][2]float64) (*sketch.Sketch, *sketch.Profile) {
	t.Helper()
	w := sketch.NewWorld()
	plane, err := w.CreatePlaneFromFrame(frame)
	require.NoError(t, err)
	s, err := w.CreateSketch(plane)
	require.NoError(t, err)
	pts := make([]*sketch.Point, len(loop))
	for i, q := range loop {
		pts[i] = s.CreatePoint(q[0], q[1])
		s.Fix(pts[i])
	}
	for i := range pts {
		s.CreateLine(pts[i], pts[(i+1)%len(pts)])
	}
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	for _, p := range s.Profiles() {
		if p.Valid {
			return s, p
		}
	}
	t.Fatal("the sketch has no valid profile")
	return nil, nil
}

var lineChordAxis = decad.SketchLine{Start: decad.Point2{}, End: decad.Point2{V: 1}}

func lineChordRect(r0, r1, z0, z1 float64) [][2]float64 {
	return [][2]float64{{r0, z0}, {r1, z0}, {r1, z1}, {r0, z1}}
}

// lineChordWedgeCoil coils the rectangle r0..r1 × z −5.4..0 so that it rises
// 4 mm over spanDeg degrees about world Z.
func lineChordWedgeCoil(t *testing.T, d *decad.Document, r0, r1, spanDeg float64) *decad.Body {
	t.Helper()
	s, p := lineChordMeridian(t, lineChordRect(r0, r1, -5.4, 0))
	b, err := decad.Coil(t.Context(), d, s, p, lineChordAxis, units.Millimeters(4*360/spanDeg), units.Scalar(spanDeg/360))
	require.NoError(t, err)
	return b
}

func requireLineChordEncloses(t *testing.T, m decad.Measurement, unit units.Unit, want float64, what string) {
	t.Helper()
	got, err := m.Value.In(unit)
	require.NoError(t, err)
	bound, err := m.Bound.In(unit)
	require.NoError(t, err)
	require.LessOrEqual(t, math.Abs(got-want), bound+1e-9, "%s %v ± %v against %v", what, got, bound, want)
}

// TestCoilChordsLongRadialLines checks a coil whose radial profile lines run
// far past their inner radius at a steep screw (agent-docs/helix-design.md
// §5.3): Bounds reads within 0.003 mm (coil.LineChordTarget, 0.002 mm, which
// the widest steep line raises to fit coil.MaxLineChords) where one chord per
// line read 0.072 to 0.38 mm, Volume encloses
// Pappus' Θ·Q, and dense samples of both true helicoidal walls lie within the
// mesh's published bound. The samples are a falsifier and prove nothing.
func TestCoilChordsLongRadialLines(t *testing.T) {
	t.Parallel()
	for _, c := range []struct {
		name            string
		r0, r1, spanDeg float64
	}{
		{"ramp 32.5 deg at r 3..11", 3, 11, 32.5},
		{"steep 3.5 deg at r 3..11", 3, 11, 3.5},
		{"ramp 32.5 deg at r 2.5..14", 2.5, 14, 32.5},
		{"steep 3.5 deg at r 2.5..14", 2.5, 14, 3.5},
	} {
		t.Run(c.name, func(t *testing.T) {
			d := decad.New()
			b := lineChordWedgeCoil(t, d, c.r0, c.r1, c.spanDeg)
			box, err := b.Bounds()
			require.NoError(t, err)
			bound, err := box.Bound.In(units.Millimeter)
			require.NoError(t, err)
			require.LessOrEqual(t, bound, 0.003)

			theta := c.spanDeg * math.Pi / 180
			vol, err := b.Volume()
			require.NoError(t, err)
			requireLineChordEncloses(t, vol, units.CubicMillimeter, theta*(c.r1*c.r1-c.r0*c.r0)/2*5.4, "volume")

			mesh, err := b.Tessellate(t.Context(), box.Bound)
			require.NoError(t, err)
			meshBound, err := mesh.Bound().In(units.Millimeter)
			require.NoError(t, err)
			k := 4 / theta
			const n = 9
			for _, z0 := range []float64{-5.4, 0} {
				for i := range n {
					rho := c.r0 + (c.r1-c.r0)*(float64(i)+0.5)/n
					for j := range n {
						th := theta * (float64(j) + 0.5) / n
						p := r3.NewVec(rho*math.Cos(th), rho*math.Sin(th), z0+k*th)
						require.LessOrEqual(t, distanceToMesh(mesh, p), meshBound, "sample at ρ %v, θ %v, z0 %v", rho, th, z0)
					}
				}
			}
		})
	}
}

// lineChordWedgeJoinsRing trims the coiled wedge to z ≥ −0.4 with a slab and
// joins it to a revolved ring r ringR0..ringR1 × z −2..0 under a 0.1 mm chain
// tolerance. The trimmed wedge encloses (r1² − r0²)/2·Θ·(2 + 0.4) and the
// union encloses the ring plus the wedge above z = 0, (r1² − r0²)/2·2·Θ.
func lineChordWedgeJoinsRing(t *testing.T, r0, r1, ringR0, ringR1, spanDeg float64) {
	t.Helper()
	ctx := t.Context()
	d := decad.New(decad.WithChainTolerance(units.Millimeters(0.1)))
	wedge := lineChordWedgeCoil(t, d, r0, r1, spanDeg)
	frame, err := r3.NewFrame(r3.Vec{Z: -0.4}, r3.Vec{X: 1}, r3.Vec{Y: 1})
	require.NoError(t, err)
	s, p := lineChordSketch(t, frame, [][2]float64{{-30, -30}, {30, -30}, {30, 30}, {-30, 30}})
	slab, err := decad.Extrude(ctx, d, s, p, decad.Distance{D: units.Millimeters(5.4), Dir: decad.Along})
	require.NoError(t, err)
	trimmed, err := decad.Intersect(wedge, slab).Do(ctx)
	require.NoError(t, err)
	theta := spanDeg * math.Pi / 180
	half := (r1*r1 - r0*r0) / 2
	vol, err := trimmed.Volume()
	require.NoError(t, err)
	requireLineChordEncloses(t, vol, units.CubicMillimeter, half*theta*(2+0.4), "trimmed wedge volume")

	s, p = lineChordMeridian(t, lineChordRect(ringR0, ringR1, -2, 0))
	ring, err := decad.Revolve(ctx, d, s, p, lineChordAxis, decad.FullRevolution{})
	require.NoError(t, err)
	joined, err := decad.Union(ring, trimmed).Do(ctx)
	require.NoError(t, err)
	vol, err = joined.Volume()
	require.NoError(t, err)
	requireLineChordEncloses(t, vol, units.CubicMillimeter, 2*math.Pi*(ringR1*ringR1-ringR0*ringR0)+half*2*theta, "union volume")
	box, err := joined.Bounds()
	require.NoError(t, err)
	bound, err := box.Bound.In(units.Millimeter)
	require.NoError(t, err)
	require.Less(t, bound, 0.05, "the joined body's box holds within half the chain tolerance")
}

// TestCoilTrimmedRampWedgeJoinsRing joins a slab-trimmed wedge rising 4 mm
// over 32.5 degrees to a ring under a 0.1 mm chain tolerance. With one chord
// per radial line its walls held 0.072 to 0.096 mm, the Union still joined
// them, and the result's box bound read 0.14 to 0.18 mm.
func TestCoilTrimmedRampWedgeJoinsRing(t *testing.T) {
	t.Parallel()
	t.Run("r 3..11 on ring 2.5..14", func(t *testing.T) { lineChordWedgeJoinsRing(t, 3, 11, 2.5, 14, 32.5) })
	t.Run("r 2.5..14 on ring 2..14.5", func(t *testing.T) { lineChordWedgeJoinsRing(t, 2.5, 14, 2, 14.5, 32.5) })
}

// TestCoilTrimmedSteepWedgeJoinsRing joins a slab-trimmed wedge rising 4 mm
// over 3.5 degrees to a ring under a 0.1 mm chain tolerance. With one chord
// per radial line its walls held 0.36 to 0.38 mm and the Union refused them.
func TestCoilTrimmedSteepWedgeJoinsRing(t *testing.T) {
	t.Parallel()
	t.Run("r 3..11 on ring 2.5..14", func(t *testing.T) { lineChordWedgeJoinsRing(t, 3, 11, 2.5, 14, 3.5) })
	t.Run("r 2.5..14 on ring 2..14.5", func(t *testing.T) { lineChordWedgeJoinsRing(t, 2.5, 14, 2, 14.5, 3.5) })
}

// TestCoilSteepLongSpanStaysInsideTheAuditCeiling checks a steep coil over a
// long span (a rectangle ρ ∈ [2.7, 11.3] falling 4 mm per 3.5 degrees over 38
// degrees, 28 helix stations): its line chords stay inside
// coilshell.MaxChordedWallTriangles, so the crossing audit admits the shell
// where 256 chords per line exceeded its pair ceiling, and Bounds still reads
// far below the 0.4 mm one chord per line held.
func TestCoilSteepLongSpanStaysInsideTheAuditCeiling(t *testing.T) {
	t.Parallel()
	const span, r0, r1 = 38.0, 2.7, 11.3
	fall := 4 * span / 3.5
	s, p := lineChordMeridian(t, lineChordRect(r0, r1, -5.4, 0))
	axis := decad.SketchLine{Start: decad.Point2{V: 1}, End: decad.Point2{}}
	b, err := decad.Coil(t.Context(), decad.New(), s, p, axis, units.Millimeters(fall*360/span), units.Scalar(span/360),
		decad.WithLeftHand())
	require.NoError(t, err)
	box, err := b.Bounds()
	require.NoError(t, err)
	bound, err := box.Bound.In(units.Millimeter)
	require.NoError(t, err)
	require.Less(t, bound, 0.02)
	theta := span * math.Pi / 180
	vol, err := b.Volume()
	require.NoError(t, err)
	requireLineChordEncloses(t, vol, units.CubicMillimeter, theta*(r1*r1-r0*r0)/2*5.4, "volume")
}
