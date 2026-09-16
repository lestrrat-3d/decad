package decadtest_test

import (
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/decadtest"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// newPlate builds the "Exact plate" fixture: a 100x60x10 mm block, every
// reading Exact with a zero bound.
func newPlate(t *testing.T) *decad.Body {
	t.Helper()

	doc := decad.New()
	return decadtest.NewBlock(t, doc, 0, 0, 100, 60, units.Millimeters(10))
}

// newUnion builds the "Approximate union" fixture: two overlapping 10 mm
// tall blocks, (0,0)-(10,10) and (5,5)-(20,20), combined with decad.Union.
// Every reading is Approximate with a nonzero bound on the order of 1e-11,
// which is real on both architectures rather than a pinned literal.
func newUnion(t *testing.T) *decad.Body {
	t.Helper()

	doc := decad.New()
	b1 := decadtest.NewBlock(t, doc, 0, 0, 10, 10, units.Millimeters(10))
	b2 := decadtest.NewBlock(t, doc, 5, 5, 20, 20, units.Millimeters(10))
	u, err := decad.Union(b1, b2)
	require.NoError(t, err)
	return u
}

func TestMeasuresAcceptsAnExactReading(t *testing.T) {
	t.Parallel()

	plate := newPlate(t)
	vol, err := plate.Volume()
	require.NoError(t, err)

	decadtest.Measures(t, "plate volume", vol, units.CubicMillimeters(60000), decadtest.Exactly())
}

func TestMeasuresAcceptsAnApproximateReading(t *testing.T) {
	t.Parallel()

	u := newUnion(t)
	vol, err := u.Volume()
	require.NoError(t, err)
	require.Greater(t, vol.Bound.Base(), 0.0)

	decadtest.Measures(t, "union volume", vol, units.CubicMillimeters(3000))
}

func TestMeasuresAcceptsAnOracleSlack(t *testing.T) {
	t.Parallel()

	u := newUnion(t)
	vol, err := u.Volume()
	require.NoError(t, err)

	decadtest.Measures(t, "union volume", vol, units.CubicMillimeters(3000.0000001), decadtest.WithinRel(units.Scalar(1e-6)))
}

func TestMeasuresVecAcceptsTheCentroid(t *testing.T) {
	t.Parallel()

	plate := newPlate(t)
	c, err := plate.Centroid()
	require.NoError(t, err)

	decadtest.MeasuresVec(t, "plate centroid", c, r3.NewVec(50, 30, 5), decadtest.Exactly())
}

func TestMeasuresBoxAcceptsThePlateBox(t *testing.T) {
	t.Parallel()

	plate := newPlate(t)
	bx, err := plate.Bounds()
	require.NoError(t, err)

	decadtest.MeasuresBox(t, "plate bounds", bx, r3.NewVec(0, 0, 0), r3.NewVec(100, 60, 10), decadtest.Exactly())
}

func TestEnclosesAcceptsAnInteriorPoint(t *testing.T) {
	t.Parallel()

	plate := newPlate(t)
	bx, err := plate.Bounds()
	require.NoError(t, err)

	decadtest.Encloses(t, "plate bounds", bx, r3.NewVec(50, 30, 5))
}

func TestAgreeAcceptsTwoMeasurementsOfOneShape(t *testing.T) {
	t.Parallel()

	plateA := newPlate(t)
	plateB := newPlate(t)
	volA, err := plateA.Volume()
	require.NoError(t, err)
	volB, err := plateB.Volume()
	require.NoError(t, err)

	decadtest.Agree(t, "the two plates", volA, volB)
}

func TestHasBoundAtMostAcceptsAGenerousCeiling(t *testing.T) {
	t.Parallel()

	u := newUnion(t)
	vol, err := u.Volume()
	require.NoError(t, err)

	decadtest.HasBoundAtMost(t, "union volume bound", vol.Bound, units.CubicMillimeters(1e-3))
}

func TestMeasuresReportsAMiss(t *testing.T) {
	t.Parallel()

	plate := newPlate(t)
	vol, err := plate.Volume()
	require.NoError(t, err)

	out := captureFailure(t, func(tb testing.TB) {
		decadtest.Measures(tb, "plate volume", vol, units.CubicMillimeters(60001))
	})
	require.Contains(t, out, "does not enclose")
}

func TestMeasuresReportsAWrongKindWant(t *testing.T) {
	t.Parallel()

	plate := newPlate(t)
	vol, err := plate.Volume()
	require.NoError(t, err)

	out := captureFailure(t, func(tb testing.TB) {
		decadtest.Measures(tb, "plate volume", vol, units.Millimeters(60000))
	})
	require.Contains(t, out, "is a length (mm) but the reading is a volume (mm^3)")
}

func TestMeasuresReportsAWrongKindWithin(t *testing.T) {
	t.Parallel()

	plate := newPlate(t)
	vol, err := plate.Volume()
	require.NoError(t, err)

	out := captureFailure(t, func(tb testing.TB) {
		decadtest.Measures(tb, "plate volume", vol, units.CubicMillimeters(60000), decadtest.Within(units.Millimeters(1e-6)))
	})
	require.Contains(t, out, "is a length (mm) but the reading is a volume (mm^3)")
}

func TestMeasuresReportsAWrongKindWithinRel(t *testing.T) {
	t.Parallel()

	plate := newPlate(t)
	vol, err := plate.Volume()
	require.NoError(t, err)

	out := captureFailure(t, func(tb testing.TB) {
		decadtest.Measures(tb, "plate volume", vol, units.CubicMillimeters(60000), decadtest.WithinRel(units.Millimeters(1e-6)))
	})
	require.Contains(t, out, "is a length (mm) but the reading is a dimensionless")
}

func TestMeasuresReportsAnApproximateReadingUnderExactly(t *testing.T) {
	t.Parallel()

	u := newUnion(t)
	vol, err := u.Volume()
	require.NoError(t, err)

	out := captureFailure(t, func(tb testing.TB) {
		decadtest.Measures(tb, "union volume", vol, units.CubicMillimeters(3000), decadtest.Exactly())
	})
	require.Contains(t, out, "reading is Approximate")
}

// TestMeasuresFalsifiesAnExactReadingWithANonzeroBound is one of the three
// places in this package where a decad.Measurement literal with a chosen
// Bound appears: the point is the comparison rule, never decad's geometry.
func TestMeasuresFalsifiesAnExactReadingWithANonzeroBound(t *testing.T) {
	t.Parallel()

	bad := decad.Measurement{
		Value:     units.CubicMillimeters(1),
		Exactness: decad.Exact,
		Bound:     units.CubicMillimeters(1e-9),
	}

	out := captureFailure(t, func(tb testing.TB) {
		decadtest.Measures(tb, "bad reading", bad, units.CubicMillimeters(1))
	})
	require.Contains(t, out, "claims Exact but carries a nonzero bound")
}

func TestMeasuresNamesTheDefaultSlack(t *testing.T) {
	t.Parallel()

	plate := newPlate(t)
	vol, err := plate.Volume()
	require.NoError(t, err)

	out := captureFailure(t, func(tb testing.TB) {
		decadtest.Measures(tb, "plate volume", vol, units.CubicMillimeters(60001))
	})
	require.Contains(t, out, "slack is the default 1e-12 relative")
}

func TestMeasuresVecReportsAMiss(t *testing.T) {
	t.Parallel()

	plate := newPlate(t)
	c, err := plate.Centroid()
	require.NoError(t, err)

	out := captureFailure(t, func(tb testing.TB) {
		decadtest.MeasuresVec(tb, "plate centroid", c, r3.NewVec(50, 30, 6))
	})
	require.Contains(t, out, "does not enclose")
}

func TestMeasuresBoxNamesTheOffendingCoordinate(t *testing.T) {
	t.Parallel()

	plate := newPlate(t)
	bx, err := plate.Bounds()
	require.NoError(t, err)

	out := captureFailure(t, func(tb testing.TB) {
		decadtest.MeasuresBox(tb, "plate bounds", bx, r3.NewVec(0, 0, 0), r3.NewVec(100, 60, 9.5))
	})
	require.Contains(t, out, "max.Z off by")
}

func TestEnclosesReportsAnOutsidePoint(t *testing.T) {
	t.Parallel()

	plate := newPlate(t)
	bx, err := plate.Bounds()
	require.NoError(t, err)

	out := captureFailure(t, func(tb testing.TB) {
		decadtest.Encloses(tb, "plate bounds", bx, r3.NewVec(50, 30, 20))
	})
	require.Contains(t, out, "does not enclose point")
}

func TestAgreeReportsADisagreement(t *testing.T) {
	t.Parallel()

	plate := newPlate(t)
	plateVol, err := plate.Volume()
	require.NoError(t, err)

	u := newUnion(t)
	unionVol, err := u.Volume()
	require.NoError(t, err)

	out := captureFailure(t, func(tb testing.TB) {
		decadtest.Agree(tb, "plate vs union", plateVol, unionVol)
	})
	require.Contains(t, out, "readings do not agree")
}

// TestHasBoundAtMostReportsABoundOverTheCeiling is one of the three
// constructed-Measurement bound literals this package allows: it exercises
// the comparison, never decad's geometry.
func TestHasBoundAtMostReportsABoundOverTheCeiling(t *testing.T) {
	t.Parallel()

	m := decad.Measurement{Bound: units.CubicMillimeters(1)}

	out := captureFailure(t, func(tb testing.TB) {
		decadtest.HasBoundAtMost(tb, "bad bound", m.Bound, units.CubicMillimeters(1e-9))
	})
	require.Contains(t, out, "exceeds the stated ceiling")
}

func TestHasBoundAtMostReportsAWrongKindCeiling(t *testing.T) {
	t.Parallel()

	plate := newPlate(t)
	vol, err := plate.Volume()
	require.NoError(t, err)

	out := captureFailure(t, func(tb testing.TB) {
		decadtest.HasBoundAtMost(tb, "plate volume bound", vol.Bound, units.Millimeters(1))
	})
	require.Contains(t, out, "is a length (mm) but the reading is a volume (mm^3)")
}

func TestReadingsRejectANilOption(t *testing.T) {
	t.Parallel()

	plate := newPlate(t)
	vol, err := plate.Volume()
	require.NoError(t, err)

	out := captureFailure(t, func(tb testing.TB) {
		decadtest.Measures(tb, "x", vol, units.CubicMillimeters(60000), nil)
	})
	require.Contains(t, out, "a nil Option was passed")
}
