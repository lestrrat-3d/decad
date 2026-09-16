package decadtest_test

import (
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/decadtest"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestMeasuresVolumeMeasuresThePlate(t *testing.T) {
	t.Parallel()

	plate := newPlate(t)
	decadtest.MeasuresVolume(t, plate, units.CubicMillimeters(60000), decadtest.Exactly())
}

func TestMeasuresAreaMeasuresThePlate(t *testing.T) {
	t.Parallel()

	plate := newPlate(t)
	decadtest.MeasuresArea(t, plate, units.SquareMillimeters(15200), decadtest.Exactly())
}

func TestMeasuresCentroidMeasuresThePlate(t *testing.T) {
	t.Parallel()

	plate := newPlate(t)
	decadtest.MeasuresCentroid(t, plate, r3.NewVec(50, 30, 5), decadtest.Exactly())
}

func TestMeasuresBoundsMeasureThePlate(t *testing.T) {
	t.Parallel()

	plate := newPlate(t)
	decadtest.MeasuresBounds(t, plate, r3.NewVec(0, 0, 0), r3.NewVec(100, 60, 10), decadtest.Exactly())
}

func TestHasSurfaceKindsCountsThePlate(t *testing.T) {
	t.Parallel()

	plate := newPlate(t)
	decadtest.HasSurfaceKinds(t, plate, map[decad.SurfaceKind]int{decad.KindPlane: 6})
}

func TestMeasuresVolumeMeasuresTheApproximateUnion(t *testing.T) {
	t.Parallel()

	u := newUnion(t)
	decadtest.MeasuresVolume(t, u, units.CubicMillimeters(3000))
}

// TestBodyNameCarriesTheStepAndOp provokes a MeasuresVolume miss on a body
// whose step is a decad.Union and checks the message names that step and
// op.
func TestBodyNameCarriesTheStepAndOp(t *testing.T) {
	t.Parallel()

	u := newUnion(t)
	steps := u.Document().Recipe().Steps
	require.Len(t, steps, 3)
	require.Equal(t, decad.OpUnion, steps[u.Origin().Step].Op)

	out := captureFailure(t, func(tb testing.TB) {
		decadtest.MeasuresVolume(tb, u, units.CubicMillimeters(3001))
	})
	require.Contains(t, out, "(step 2 union)")
}

func TestMeasuresVolumeRejectsANilBody(t *testing.T) {
	t.Parallel()

	out := captureFailure(t, func(tb testing.TB) {
		decadtest.MeasuresVolume(tb, nil, units.CubicMillimeters(1))
	})
	require.Contains(t, out, "body must not be nil")
}

func TestMeasuresAreaRejectsANilBody(t *testing.T) {
	t.Parallel()

	out := captureFailure(t, func(tb testing.TB) {
		decadtest.MeasuresArea(tb, nil, units.SquareMillimeters(1))
	})
	require.Contains(t, out, "body must not be nil")
}

func TestMeasuresCentroidRejectsANilBody(t *testing.T) {
	t.Parallel()

	out := captureFailure(t, func(tb testing.TB) {
		decadtest.MeasuresCentroid(tb, nil, r3.NewVec(0, 0, 0))
	})
	require.Contains(t, out, "body must not be nil")
}

func TestMeasuresBoundsRejectANilBody(t *testing.T) {
	t.Parallel()

	out := captureFailure(t, func(tb testing.TB) {
		decadtest.MeasuresBounds(tb, nil, r3.NewVec(0, 0, 0), r3.NewVec(1, 1, 1))
	})
	require.Contains(t, out, "body must not be nil")
}

func TestHasSurfaceKindsRejectsANilBody(t *testing.T) {
	t.Parallel()

	out := captureFailure(t, func(tb testing.TB) {
		decadtest.HasSurfaceKinds(tb, nil, nil)
	})
	require.Contains(t, out, "body must not be nil")
}

func TestMeasuresVolumeReportsAMissWithTheBodyNamed(t *testing.T) {
	t.Parallel()

	plate := newPlate(t)

	out := captureFailure(t, func(tb testing.TB) {
		decadtest.MeasuresVolume(tb, plate, units.CubicMillimeters(60001))
	})
	require.Contains(t, out, "body[0] (step 0 extrude)")
}

func TestHasSurfaceKindsReportsAMissingKind(t *testing.T) {
	t.Parallel()

	plate := newPlate(t)

	out := captureFailure(t, func(tb testing.TB) {
		decadtest.HasSurfaceKinds(tb, plate, map[decad.SurfaceKind]int{decad.KindPlane: 5})
	})
	require.Contains(t, out, "surface kind counts are")
}

func TestHasSurfaceKindsReportsAnUnexpectedKind(t *testing.T) {
	t.Parallel()

	plate := newPlate(t)

	out := captureFailure(t, func(tb testing.TB) {
		decadtest.HasSurfaceKinds(tb, plate, map[decad.SurfaceKind]int{})
	})
	require.Contains(t, out, "surface kind counts are")
}
