package decad

import (
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// joinRectangle extrudes the rectangle [u0, u1]×[0, 4] by 3 and joins it
// across its wall u = u0.
func joinRectangle(t *testing.T, u0, u1 float64) *Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(u0, 0, u1, 4)
	s.Fix(rect.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	doc := New()
	body, err := doc.Extrude(s, s.Profiles()[0], Distance{D: units.Millimeters(3), Dir: Along})
	require.NoError(t, err)
	joined, err := body.Mirrored(t.Context(), MirrorFace{
		Body: body, Face: Faces(Planar(), Facing(r3.NewVec(-1, 0, 0)))}, WithJoin())
	require.NoError(t, err)
	return joined
}

// TestMirrorJoinChargesTheReflectionRounding is §8's δ_mirror test. Across
// the line u = 0.1, which no float states exactly, the far wall u = 5
// reflects to 2·0.1 − 5, which rounds: the joined record's displacement is
// exactly that rounding, and the published volume bound contains the exact
// volume of the section the join denotes, (10 − 2·0.1)·4·3 with 0.1 the
// float the caller drew. Across u = 8 every image is a float and the
// displacement is exactly zero.
//
// Shown to fail with the reflection's rounding left out of the charge: the
// u = 0.1 join then publishes a zero displacement.
func TestMirrorJoinChargesTheReflectionRounding(t *testing.T) {
	t.Parallel()
	t.Run("u = 0.1", func(t *testing.T) {
		t.Parallel()
		joined := joinRectangle(t, 0.1, 5)
		pp, ok := joined.payload.(prismPayload)
		require.True(t, ok)
		tenth := proofarith.FloatRat(0.1)
		image := new(big.Rat).Sub(new(big.Rat).Mul(big.NewRat(2, 1), tenth), big.NewRat(5, 1))
		held, _ := image.Float64()
		want := proofarith.RationalFloatError(image, held)
		require.Positive(t, want, "the premise: 2·0.1 − 5 is not a float")
		require.Equal(t, want, pp.sectionDelta)

		v, err := joined.Volume()
		require.NoError(t, err)
		require.NotEqual(t, Exact, v.Exactness)
		value, err := v.Value.In(units.CubicMillimeter)
		require.NoError(t, err)
		bound, err := v.Bound.In(units.CubicMillimeter)
		require.NoError(t, err)
		exact := new(big.Rat).Mul(new(big.Rat).Sub(big.NewRat(10, 1), new(big.Rat).Mul(big.NewRat(2, 1), tenth)), big.NewRat(12, 1))
		gap := new(big.Rat).Sub(proofarith.FloatRat(value), exact)
		require.LessOrEqual(t, gap.Abs(gap).Cmp(proofarith.FloatRat(bound)), 0,
			"the volume %v ± %v misses the exact %s", value, bound, exact.FloatString(20))
	})
	t.Run("u = 8", func(t *testing.T) {
		t.Parallel()
		joined := joinRectangle(t, 8, 13)
		pp, ok := joined.payload.(prismPayload)
		require.True(t, ok)
		require.Zero(t, pp.sectionDelta)
		v, err := joined.Volume()
		require.NoError(t, err)
		require.Equal(t, Exact, v.Exactness)
		value, err := v.Value.In(units.CubicMillimeter)
		require.NoError(t, err)
		require.Equal(t, 120.0, value)
	})
}

// TestMirrorJoinCircleImageKeepsItsSense pins §5.2's circle row for the case
// the join builds — a reflection AND a reversed walk: a hole circle's image
// keeps the hole's own CCW flag and range, and walkOf, which refuses a CCW
// flag that contradicts its range order, reads it. The flag flipped is the
// record walkOf refuses.
func TestMirrorJoinCircleImageKeepsItsSense(t *testing.T) {
	t.Parallel()
	line, err := admitMirrorLine([]LineSeg{{Start: Point2{U: 0, V: 10}, End: Point2{U: 0, V: 0}, TStart: 0, TEnd: 1}})
	require.NoError(t, err)
	hole := CircleSeg{Center: Point2{U: 5, V: 5}, Radius: units.Millimeters(2), CCW: false, TStart: 1, TEnd: 0}
	img, charge, err := line.mirrorReversedRun(proofbound.NewWorkBudget(t.Context()), []CurveSegment{hole})
	require.NoError(t, err)
	require.Zero(t, charge)
	want := CircleSeg{Center: Point2{U: -5, V: 5}, Radius: units.Millimeters(2), CCW: false, TStart: 1, TEnd: 0}
	require.Equal(t, []CurveSegment{want}, img)
	_, err = boundarywalk.WalkOf(img[0], nil)
	require.NoError(t, err)

	flipped := want
	flipped.CCW = true
	_, err = boundarywalk.WalkOf(flipped, nil)
	require.Error(t, err, "the premise: a flipped flag contradicts the range")
}
