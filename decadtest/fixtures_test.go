package decadtest_test

import (
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/decadtest"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// TestNewBlockBuildsTheMeasuredPlate proves NewBlock by asserting on the
// computed geometry of the body it builds, not merely that it ran.
func TestNewBlockBuildsTheMeasuredPlate(t *testing.T) {
	t.Parallel()

	doc := decad.New()
	body := decadtest.NewBlock(t, doc, 0, 0, 100, 60, units.Millimeters(10))

	vol, err := body.Volume()
	require.NoError(t, err)
	require.Equal(t, decad.Exact, vol.Exactness)
	require.True(t, vol.Value.Equal(units.CubicMillimeters(60000), 0))

	area, err := body.Area()
	require.NoError(t, err)
	require.True(t, area.Value.Equal(units.SquareMillimeters(15200), 0))

	c, err := body.Centroid()
	require.NoError(t, err)
	require.Equal(t, r3.NewVec(50, 30, 5), c.Value)

	bx, err := body.Bounds()
	require.NoError(t, err)
	require.Equal(t, r3.NewVec(0, 0, 0), bx.Min)
	require.Equal(t, r3.NewVec(100, 60, 10), bx.Max)

	require.True(t, body.IsSolid())

	kinds := map[decad.SurfaceKind]int{}
	for _, f := range body.Faces() {
		kinds[f.Surface().Kind()]++
	}
	require.Equal(t, map[decad.SurfaceKind]int{decad.KindPlane: 6}, kinds)
}

func TestNewBlockAddsOneBody(t *testing.T) {
	t.Parallel()

	doc := decad.New()
	decadtest.NewBlock(t, doc, 0, 0, 10, 10, units.Millimeters(2))

	require.Len(t, doc.Bodies(), 1)
}

// TestNewBlockIsSoundUnderVerify proves NewBlock's body clears decad's own
// verifier.
func TestNewBlockIsSoundUnderVerify(t *testing.T) {
	t.Parallel()

	doc := decad.New()
	decadtest.NewBlock(t, doc, 0, 0, 10, 10, units.Millimeters(2))

	report, err := doc.Verify(t.Context())
	require.NoError(t, err)
	require.True(t, report.Passed())
	require.Equal(t, decad.Sound, report.Status)
}

// TestNewPrismExtrudesTheSolvedRegion exercises NewSketch, SolveRegion and
// NewPrism directly rather than through NewBlock.
func TestNewPrismExtrudesTheSolvedRegion(t *testing.T) {
	t.Parallel()

	doc := decad.New()
	s := decadtest.NewSketch(t)
	rect := s.CreateRectangle(0, 0, 4, 5)
	s.Fix(rect.A)

	p := decadtest.SolveRegion(t, s)
	require.True(t, p.Valid)
	require.Greater(t, p.Area, 0.0)

	body := decadtest.NewPrism(t, doc, s, p, units.Millimeters(3))
	vol, err := body.Volume()
	require.NoError(t, err)
	require.True(t, vol.Value.Equal(units.CubicMillimeters(p.Area*3), 0))
}

// TestSolveRegionRejectsNilSketch shows SolveRegion fail when s is nil.
func TestSolveRegionRejectsNilSketch(t *testing.T) {
	t.Parallel()

	out := captureFailure(t, func(tb testing.TB) {
		decadtest.SolveRegion(tb, nil)
	})
	require.Contains(t, out, "s must not be nil")
}

// TestSolveRegionRejectsTwoRegions shows SolveRegion fail when the sketch
// holds more than one valid region.
func TestSolveRegionRejectsTwoRegions(t *testing.T) {
	t.Parallel()

	out := captureFailure(t, func(tb testing.TB) {
		s := decadtest.NewSketch(tb)
		r1 := s.CreateRectangle(0, 0, 2, 2)
		r2 := s.CreateRectangle(10, 10, 12, 12)
		s.Fix(r1.A)
		s.Fix(r2.A)
		decadtest.SolveRegion(tb, s)
	})
	require.Contains(t, out, "valid region(s), want exactly 1")
}

// TestSolveRegionRejectsNoRegion shows SolveRegion fail when the sketch
// holds no entities at all.
func TestSolveRegionRejectsNoRegion(t *testing.T) {
	t.Parallel()

	out := captureFailure(t, func(tb testing.TB) {
		s := decadtest.NewSketch(tb)
		decadtest.SolveRegion(tb, s)
	})
	require.Contains(t, out, "valid region(s), want exactly 1")
}

// TestNewPrismRejectsNilDocument shows NewPrism fail when doc is nil.
func TestNewPrismRejectsNilDocument(t *testing.T) {
	t.Parallel()

	s := decadtest.NewSketch(t)
	rect := s.CreateRectangle(0, 0, 2, 2)
	s.Fix(rect.A)
	p := decadtest.SolveRegion(t, s)

	out := captureFailure(t, func(tb testing.TB) {
		decadtest.NewPrism(tb, nil, s, p, units.Millimeters(1))
	})
	require.Contains(t, out, "doc must not be nil")
}

// TestNewPrismRejectsNilProfile shows NewPrism fail when p is nil.
func TestNewPrismRejectsNilProfile(t *testing.T) {
	t.Parallel()

	doc := decad.New()
	s := decadtest.NewSketch(t)

	out := captureFailure(t, func(tb testing.TB) {
		decadtest.NewPrism(tb, doc, s, nil, units.Millimeters(1))
	})
	require.Contains(t, out, "p must not be nil")
}

// TestNewPrismReportsAWrongKindHeight shows NewPrism fail when height is
// not a Length, surfacing decad's own ErrUnitKind through the Fatalf
// message.
func TestNewPrismReportsAWrongKindHeight(t *testing.T) {
	t.Parallel()

	doc := decad.New()
	s := decadtest.NewSketch(t)
	rect := s.CreateRectangle(0, 0, 2, 2)
	s.Fix(rect.A)
	p := decadtest.SolveRegion(t, s)

	out := captureFailure(t, func(tb testing.TB) {
		decadtest.NewPrism(tb, doc, s, p, units.Degrees(10))
	})
	require.Contains(t, out, "extruding by")
}

// TestCaptureFailureRefusesASilentHelper proves the harness itself: it must
// NOT use captureFailure, which would recurse into the failure it is
// testing. It builds a recordingTB directly and runs a function that
// reports nothing, so the require.True inside captureFailure is known to be
// the thing that would fire on a helper that never fails.
func TestCaptureFailureRefusesASilentHelper(t *testing.T) {
	t.Parallel()

	rec := &recordingTB{TB: t}
	func(tb testing.TB) {
		// Reports nothing: a silent, always-passing "helper".
		_ = tb
	}(rec)

	require.False(t, rec.failed)
	require.Empty(t, rec.output())
}
