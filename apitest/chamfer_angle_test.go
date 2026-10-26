package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/decadtest"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file is agent-docs/chamfer-angle-design.md's public-surface tests for
// ChamferAngle on a 20 x 20 x 10 block, whose vertical corner edges are right
// angled.

func angleBlock(tb testing.TB) (*decad.Document, *decad.Body) {
	tb.Helper()
	doc := decad.New()
	return doc, decadtest.NewBlock(tb, doc, 0, 0, 20, 20, units.Millimeters(10))
}

func angleCornerEdge() *decad.EdgeQuery {
	return decad.Edges(decad.ParallelTo(r3.NewVec(0, 0, 1)), decad.EndpointAt(r3.NewVec(20, 20, 0))).Exactly(1)
}

func TestChamferAngleSetsTheOtherDistance(t *testing.T) {
	t.Parallel()
	for _, deg := range []float64{30, 45, 60} {
		t.Run("", func(t *testing.T) {
			t.Parallel()
			doc, block := angleBlock(t)
			out, err := decad.ChamferAngle(t.Context(), block, angleCornerEdge(), units.Millimeters(2),
				decad.Faces(decad.Facing(r3.NewVec(1, 0, 0))), units.Degrees(deg))
			require.NoError(t, err)
			require.Equal(t, []*decad.Body{out}, doc.Bodies())
			// The bevel removes a right triangle with legs 2 and 2·tan(angle), 10 mm tall.
			other := 2 * math.Tan(deg*math.Pi/180)
			decadtest.MeasuresVolume(t, out, units.CubicMillimeters(4000-0.5*2*other*10),
				decadtest.WithinRel(units.Scalar(1e-9)))
			operations := doc.OperationHistory().Operations()
			require.Equal(t, decad.OperationChamfer, operations[len(operations)-1].Kind())
		})
	}
}

// The reference face takes the distance, so swapping it moves the bevel's
// long leg from one wall to the other and shifts the body's centroid.
func TestChamferAngleHonoursTheReferenceFace(t *testing.T) {
	t.Parallel()
	_, a := angleBlock(t)
	_, b := angleBlock(t)
	outA, err := decad.ChamferAngle(t.Context(), a, angleCornerEdge(), units.Millimeters(2),
		decad.Faces(decad.Facing(r3.NewVec(1, 0, 0))), units.Degrees(30))
	require.NoError(t, err)
	outB, err := decad.ChamferAngle(t.Context(), b, angleCornerEdge(), units.Millimeters(2),
		decad.Faces(decad.Facing(r3.NewVec(0, 1, 0))), units.Degrees(30))
	require.NoError(t, err)
	ca, err := outA.Centroid()
	require.NoError(t, err)
	cb, err := outB.Centroid()
	require.NoError(t, err)
	require.InDelta(t, ca.Value.X, cb.Value.Y, 1e-9, "the two bevels mirror across x = y")
	require.NotEqual(t, ca.Value.X, ca.Value.Y)
}

func TestChamferAngleRefusals(t *testing.T) {
	t.Parallel()
	ref := decad.Faces(decad.Facing(r3.NewVec(1, 0, 0)))
	cases := []struct {
		name  string
		angle units.Value
		d     units.Value
		ref   decad.FaceSelector
		want  error
	}{
		{"zero angle", units.Degrees(0), units.Millimeters(2), ref, decad.ErrDegenerate},
		{"right angle", units.Degrees(90), units.Millimeters(2), ref, decad.ErrDegenerate},
		{"negative angle", units.Degrees(-5), units.Millimeters(2), ref, decad.ErrNegativeMagnitude},
		{"angle as a length", units.Millimeters(5), units.Millimeters(2), ref, decad.ErrUnitKind},
		{"distance as an angle", units.Degrees(30), units.Degrees(2), ref, decad.ErrUnitKind},
		{"no reference", units.Degrees(30), units.Millimeters(2), nil, decad.ErrDegenerate},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			doc, block := angleBlock(t)
			_, err := decad.ChamferAngle(t.Context(), block, angleCornerEdge(), tc.d, tc.ref, tc.angle)
			require.Error(t, err)
			require.ErrorIs(t, err, tc.want)
			require.Equal(t, []*decad.Body{block}, doc.Bodies(), "a refused chamfer leaves the block live and alone")
		})
	}
}

// A triangular prism's acute lateral edge is not a right angle, so the
// tangent reading of the second distance does not apply.
func TestChamferAngleRefusesAnEdgeThatIsNotRightAngled(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	s := decadtest.NewSketch(t)
	a := s.CreatePoint(0, 0)
	b := s.CreatePoint(20, 0)
	c := s.CreatePoint(0, 20)
	s.Fix(a)
	s.Fix(b)
	s.Fix(c)
	s.CreateLine(a, b)
	s.CreateLine(b, c)
	s.CreateLine(c, a)
	prism := decadtest.NewPrism(t, doc, s, decadtest.SolveRegion(t, s), units.Millimeters(10))
	edge := decad.Edges(decad.ParallelTo(r3.NewVec(0, 0, 1)), decad.EndpointAt(r3.NewVec(20, 0, 0))).Exactly(1)
	_, err := decad.ChamferAngle(t.Context(), prism, edge, units.Millimeters(2),
		decad.Faces(decad.Facing(r3.NewVec(0, -1, 0))), units.Degrees(30))
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.ErrorContains(t, err, "right angle")
	require.Equal(t, []*decad.Body{prism}, doc.Bodies())
}
