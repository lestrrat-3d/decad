package apitest_test

import (
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// levelPin sketches a 20x20 pin footprint on the XY plane.
func levelPin(t *testing.T) (*sketch.Sketch, *sketch.Profile) {
	t.Helper()
	return taperSketch(t, r3.Vec{}, 0, func(s *sketch.Sketch) {
		r := s.CreateRectangle(-10, -10, 10, 10)
		s.Fix(r.A)
	})
}

// requireTopCovered checks every vertex of pin at least 1 mm above the base
// frame's plane against the exact stop plane m·(x − p0) = 0: its exact
// distance from the plane must not exceed its published bound. The distance is
// compared squared, so no root is taken: r² ≤ bound²·|m|² for r = m·(x − p0).
func requireTopCovered(t *testing.T, pin *decad.Body, base r3.Frame, m, p0 [3]*big.Rat) {
	t.Helper()
	n, o := tiltRatVec(base.N()), tiltRatVec(base.Origin())
	diff := func(p r3.Vec, q [3]*big.Rat) [3]*big.Rat {
		return [3]*big.Rat{
			new(big.Rat).Sub(tiltRat(p.X), q[0]),
			new(big.Rat).Sub(tiltRat(p.Y), q[1]),
			new(big.Rat).Sub(tiltRat(p.Z), q[2]),
		}
	}
	top, uncovered := 0, 0
	for _, vertex := range pin.Vertices() {
		p := vertex.Position()
		if tiltRatDot(n, diff(p.Value, o)).Cmp(big.NewRat(1, 1)) < 0 {
			continue
		}
		top++
		r := tiltRatDot(m, diff(p.Value, p0))
		bound := tiltLengthRat(t, p.Bound)
		lhs := new(big.Rat).Mul(r, r)
		rhs := new(big.Rat).Mul(new(big.Rat).Mul(bound, bound), tiltRatDot(m, m))
		covered := lhs.Cmp(rhs) <= 0
		t.Logf("vertex %v: |m·(x-p0)|² = %s, (bound·|m|)² = %s, covered %v", p.Value, tiltRatE(lhs), tiltRatE(rhs), covered)
		if !covered {
			uncovered++
		}
	}
	require.Equal(t, 4, top)
	require.Zero(t, uncovered, `each top vertex's published bound must cover its distance from the true stop plane`)
}

// TestToFaceChargesStopOriginLift pins that a ToFace stop charges the rounding
// of the stop cap's own origin lift. The plate sits on a sketch plane at
// z = float64(0.1) and is extruded 10 mm, so its far cap denotes the plane
// z = Rat(0.1) + 10 while its held tag says float64(10.1). The stop must
// either cover that exact gap on every channel or refuse.
func TestToFaceChargesStopOriginLift(t *testing.T) {
	t.Parallel()
	plateSketch, plateProf := taperSketch(t, r3.NewVec(0, 0, 0.1), 0, drawSquare(0, 60))
	pinSketch, pinProf := levelPin(t)
	doc := decad.New()
	plate, err := decad.Extrude(t.Context(), doc, plateSketch, plateProf,
		decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	require.NoError(t, err)
	zStar := new(big.Rat).Add(tiltRat(0.1), big.NewRat(10, 1))

	before := len(doc.Bodies())
	pin, err := decad.Extrude(t.Context(), doc, pinSketch, pinProf,
		decad.ToFace{Body: plate, Face: capEndFace(plate)})
	if err != nil {
		require.ErrorIs(t, err, decad.ErrUnsupported)
		require.Len(t, doc.Bodies(), before, `a refused stop leaves the document unchanged`)
		return
	}
	box, err := pin.Bounds()
	require.NoError(t, err)
	heldTop := tiltRat(box.Max.Z)
	gap := tiltRatAbs(new(big.Rat).Sub(zStar, heldTop))
	t.Logf("held top z %v, exact %s, gap %s", box.Max.Z, tiltRatE(zStar), tiltRatE(gap))
	require.Positive(t, gap.Sign(), `the fixture must show the held top differs from the exact plane`)

	requireTopCovered(t, pin, xyFrame(t), [3]*big.Rat{new(big.Rat), new(big.Rat), big.NewRat(1, 1)},
		[3]*big.Rat{new(big.Rat), new(big.Rat), zStar})
	boxBound := tiltLengthRat(t, box.Bound)
	require.LessOrEqual(t, gap.Cmp(boxBound), 0, `the published box bound must cover the exact top`)

	// The pin's footprint is 20×20 from z = 0, so its true volume is 400·z*.
	vol, err := pin.Volume()
	require.NoError(t, err)
	volValue, err := vol.Value.In(units.CubicMillimeter)
	require.NoError(t, err)
	volBound, err := vol.Bound.In(units.CubicMillimeter)
	require.NoError(t, err)
	volGap := tiltRatAbs(new(big.Rat).Sub(tiltRat(volValue), new(big.Rat).Mul(big.NewRat(400, 1), zStar)))
	t.Logf("volume gap %s, published bound %s", tiltRatE(volGap), tiltRatE(tiltRat(volBound)))
	require.LessOrEqual(t, volGap.Cmp(tiltRat(volBound)), 0, `the published volume bound must cover the exact solid`)
}

// TestToFaceRefusesFootprintDependentStop pins that a stop whose denoted plane
// is not an exact level set of the sweep is refused or fully charged. The plate
// is extruded on the frame F = (U (1, 1, 0), V (-1, 1, 1)) and the pin sweeps
// from a frame spanned by U + V and V - U, which has the same held normal as F
// but sits a rounding step off F's exact normal. The pin footprint lies 1e6 mm
// along that frame's V axis.
func TestToFaceRefusesFootprintDependentStop(t *testing.T) {
	t.Parallel()
	f, err := r3.NewFrame(r3.Vec{}, r3.NewVec(1, 1, 0), r3.NewVec(-1, 1, 1))
	require.NoError(t, err)
	sweep, err := r3.NewFrame(f.Origin(), f.U().Add(f.V()), f.V().Sub(f.U()))
	require.NoError(t, err)

	plateWorld := sketch.NewWorld()
	plateSketch, err := plateWorld.CreateSketch(mustPlane(t, plateWorld, f))
	require.NoError(t, err)
	plateRect := plateSketch.CreateRectangle(0, 0, 20, 20)
	plateSketch.Fix(plateRect.A)
	_, err = plateSketch.Solve(t.Context())
	require.NoError(t, err)

	pinWorld := sketch.NewWorld()
	pinSketch, err := pinWorld.CreateSketch(mustPlane(t, pinWorld, sweep))
	require.NoError(t, err)
	pinRect := pinSketch.CreateRectangle(0, 1e6, 20, 1e6+20)
	pinSketch.Fix(pinRect.A)
	_, err = pinSketch.Solve(t.Context())
	require.NoError(t, err)

	doc := decad.New()
	const depth = 10.0
	plate, err := decad.Extrude(t.Context(), doc, plateSketch, plateSketch.Profiles()[0],
		decad.Distance{D: units.Millimeters(depth), Dir: decad.Along})
	require.NoError(t, err)

	before := len(doc.Bodies())
	pin, err := decad.Extrude(t.Context(), doc, pinSketch, pinSketch.Profiles()[0],
		decad.ToFace{Body: plate, Face: capEndFace(plate)})
	if err != nil {
		require.ErrorIs(t, err, decad.ErrUnsupported)
		require.Len(t, doc.Bodies(), before, `a refused stop leaves the document unchanged`)
		return
	}
	// The far cap denotes the plane through O + depth·N with normal m = U×V.
	m := tiltRatCross(tiltRatVec(f.U()), tiltRatVec(f.V()))
	p0 := tiltRatScale(tiltRatVec(f.N()), big.NewRat(10, 1))
	requireTopCovered(t, pin, sweep, m, p0)
}

func mustPlane(t *testing.T, w *sketch.World, f r3.Frame) *sketch.Plane {
	t.Helper()
	p, err := w.CreatePlaneFromFrame(f)
	require.NoError(t, err)
	return p
}

// TestToFaceRefusesFacetedLoftCap pins that a ToFace stop onto the far cap of
// a mirrored-loft union is refused or fully charged. The union copies the
// loft's cap tags into faceted groups that keep neither the loft's axial
// displacement nor the record of the exact plane, and the mirror lands the
// cap at a level its held tag rounds.
func TestToFaceRefusesFacetedLoftCap(t *testing.T) {
	t.Parallel()
	s0, p0, s1, p1 := loftSquaresAt(t, r3.NewVec(0, 0, 0.1), 2, 1, 10)
	doc := decad.New()
	lower, err := decad.Loft(t.Context(), doc, s0, p0, s1, p1)
	require.NoError(t, err)
	mirrorFrame, err := r3.NewFrame(r3.NewVec(0, 0, 10.1), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	upper, err := decad.MirroredCopy(t.Context(), lower, decad.MirrorFrame{Frame: mirrorFrame})
	require.NoError(t, err)
	joined, err := decad.Union(t.Context(), upper, lower)
	require.NoError(t, err)
	require.True(t, joined.IsSolid())

	// The far cap is the mirror image of z = Rat(0.1) under r3.Reflection.
	refl, err := r3.Reflection(mirrorFrame)
	require.NoError(t, err)
	zStar := new(big.Rat).Add(
		new(big.Rat).Mul(tiltRat(refl.Basis().EZ.Z), tiltRat(0.1)), tiltRat(refl.Translation().Z))
	faces, err := capEndFace(joined).SelectFaces(joined)
	require.NoError(t, err)
	require.Len(t, faces, 1)
	plane, ok := faces[0].Surface().(decad.Plane)
	require.True(t, ok)
	gap := tiltRatAbs(new(big.Rat).Sub(tiltRat(plane.Frame.Origin().Z), zStar))
	t.Logf("held cap z %v, exact %s, gap %s", plane.Frame.Origin().Z, tiltRatE(zStar), tiltRatE(gap))
	require.Positive(t, gap.Sign(), `the fixture must show the held cap level differs from the exact plane`)

	pinSketch, pinProf := levelPin(t)
	before := len(doc.Bodies())
	pin, err := decad.Extrude(t.Context(), doc, pinSketch, pinProf,
		decad.ToFace{Body: joined, Face: capEndFace(joined)})
	if err != nil {
		require.ErrorIs(t, err, decad.ErrUnsupported)
		require.Len(t, doc.Bodies(), before, `a refused stop leaves the document unchanged`)
		return
	}
	requireTopCovered(t, pin, xyFrame(t), [3]*big.Rat{new(big.Rat), new(big.Rat), big.NewRat(1, 1)},
		[3]*big.Rat{new(big.Rat), new(big.Rat), zStar})
}
