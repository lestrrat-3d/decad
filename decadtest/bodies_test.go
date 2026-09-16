package decadtest_test

import (
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/decadtest"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestVolumeMeasuresThePlate(t *testing.T) {
	t.Parallel()

	plate := newPlate(t)
	decadtest.Volume(t, plate, units.CubicMillimeters(60000), decadtest.Exactly())
}

func TestAreaMeasuresThePlate(t *testing.T) {
	t.Parallel()

	plate := newPlate(t)
	decadtest.Area(t, plate, units.SquareMillimeters(15200), decadtest.Exactly())
}

func TestCentroidMeasuresThePlate(t *testing.T) {
	t.Parallel()

	plate := newPlate(t)
	decadtest.Centroid(t, plate, r3.NewVec(50, 30, 5), decadtest.Exactly())
}

func TestBoundsMeasureThePlate(t *testing.T) {
	t.Parallel()

	plate := newPlate(t)
	decadtest.Bounds(t, plate, r3.NewVec(0, 0, 0), r3.NewVec(100, 60, 10), decadtest.Exactly())
}

func TestSurfaceKindsCountsThePlate(t *testing.T) {
	t.Parallel()

	plate := newPlate(t)
	decadtest.SurfaceKinds(t, plate, map[decad.SurfaceKind]int{decad.KindPlane: 6})
}

func TestVolumeMeasuresTheApproximateUnion(t *testing.T) {
	t.Parallel()

	u := newUnion(t)
	decadtest.Volume(t, u, units.CubicMillimeters(3000))
}

// TestBodyNameCarriesTheStepAndOp provokes a Volume miss on a body whose
// step is a decad.Union and checks the message names that step and op.
func TestBodyNameCarriesTheStepAndOp(t *testing.T) {
	t.Parallel()

	u := newUnion(t)
	steps := u.Document().Recipe().Steps
	require.Len(t, steps, 3)
	require.Equal(t, decad.OpUnion, steps[u.Origin().Step].Op)

	out := captureFailure(t, func(tb testing.TB) {
		decadtest.Volume(tb, u, units.CubicMillimeters(3001))
	})
	require.Contains(t, out, "(step 2 union)")
}

func TestVolumeRejectsANilBody(t *testing.T) {
	t.Parallel()

	out := captureFailure(t, func(tb testing.TB) {
		decadtest.Volume(tb, nil, units.CubicMillimeters(1))
	})
	require.Contains(t, out, "body must not be nil")
}

func TestAreaRejectsANilBody(t *testing.T) {
	t.Parallel()

	out := captureFailure(t, func(tb testing.TB) {
		decadtest.Area(tb, nil, units.SquareMillimeters(1))
	})
	require.Contains(t, out, "body must not be nil")
}

func TestCentroidRejectsANilBody(t *testing.T) {
	t.Parallel()

	out := captureFailure(t, func(tb testing.TB) {
		decadtest.Centroid(tb, nil, r3.NewVec(0, 0, 0))
	})
	require.Contains(t, out, "body must not be nil")
}

func TestBoundsRejectANilBody(t *testing.T) {
	t.Parallel()

	out := captureFailure(t, func(tb testing.TB) {
		decadtest.Bounds(tb, nil, r3.NewVec(0, 0, 0), r3.NewVec(1, 1, 1))
	})
	require.Contains(t, out, "body must not be nil")
}

func TestSurfaceKindsRejectsANilBody(t *testing.T) {
	t.Parallel()

	out := captureFailure(t, func(tb testing.TB) {
		decadtest.SurfaceKinds(tb, nil, nil)
	})
	require.Contains(t, out, "body must not be nil")
}

func TestVolumeReportsAMissWithTheBodyNamed(t *testing.T) {
	t.Parallel()

	plate := newPlate(t)

	out := captureFailure(t, func(tb testing.TB) {
		decadtest.Volume(tb, plate, units.CubicMillimeters(60001))
	})
	require.Contains(t, out, "body[0] (step 0 extrude)")
}

func TestSurfaceKindsReportsAMissingKind(t *testing.T) {
	t.Parallel()

	plate := newPlate(t)

	out := captureFailure(t, func(tb testing.TB) {
		decadtest.SurfaceKinds(tb, plate, map[decad.SurfaceKind]int{decad.KindPlane: 5})
	})
	require.Contains(t, out, "surface kind counts are")
}

func TestSurfaceKindsReportsAnUnexpectedKind(t *testing.T) {
	t.Parallel()

	plate := newPlate(t)

	out := captureFailure(t, func(tb testing.TB) {
		decadtest.SurfaceKinds(tb, plate, map[decad.SurfaceKind]int{})
	})
	require.Contains(t, out, "surface kind counts are")
}
