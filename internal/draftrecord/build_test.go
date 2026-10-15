package draftrecord_test

import (
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/draftrecord"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topo/topotest"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// centeredSquare is a square section of half side h about the plane origin.
func centeredSquare(h float64) momentinput.Profile {
	pts := []sectionrecord.Point2{{U: -h, V: -h}, {U: h, V: -h}, {U: h, V: h}, {U: -h, V: h}}
	segs := make([]sectionrecord.CurveSegment, len(pts))
	for i := range pts {
		segs[i] = sectionrecord.LineSeg{Start: pts[i], End: pts[(i+1)%len(pts)], TStart: 0, TEnd: 1}
	}
	return momentinput.Profile{Outer: sectionrecord.LoopRecord{Segments: segs}}
}

func xyFrame(t *testing.T) r3.Frame {
	t.Helper()
	frame, err := r3.NewFrame(r3.Vec{}, r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	return frame
}

// TestOffsetEnclosesTheExactProduct pins §8.1's span: every corner of the
// exact product of the height span and the tangent enclosure lies within
// dDelta of the held d, and dDelta is positive even for a millimetre sweep and
// a degree angle, since the tangent enclosure has width. A taper whose cosine
// enclosure reaches zero is SD2.
func TestOffsetEnclosesTheExactProduct(t *testing.T) {
	t.Parallel()
	for _, tc := range []struct{ h, hDelta, deg float64 }{
		{10, 0, 5}, {8, 0, -3}, {25.4, 3.6e-15, 7.5}, {1, 0, 89},
	} {
		alpha := tc.deg * units.Degree.Factor()
		d, dDelta, err := draftrecord.Offset(tc.h, tc.hDelta, alpha, 0)
		require.NoError(t, err)
		require.Greater(t, dDelta, 0.0)
		require.Equal(t, math.Signbit(d), tc.deg < 0)
		tan, ok := proofbound.RadTanSpan(proofbound.PointInterval(proofarith.FloatRat(alpha)))
		require.True(t, ok)
		for _, h := range []float64{tc.h - tc.hDelta, tc.h + tc.hDelta} {
			for _, tv := range []*big.Rat{tan.Lo, tan.Hi} {
				exact := new(big.Rat).Mul(proofarith.FloatRat(h), tv)
				gap := new(big.Rat).Abs(new(big.Rat).Sub(exact, proofarith.FloatRat(d)))
				require.LessOrEqual(t, gap.Cmp(proofarith.FloatRat(dDelta)), 0)
			}
		}
	}
	_, _, err := draftrecord.Offset(10, 0, math.Pi/2, 1e-3)
	require.ErrorIs(t, err, decaderr.ErrDegenerate)
}

// TestPlacedBuildsTheFrustumUnderTheComposedMotion builds a 10 mm square
// tapered 5 degrees over 8 mm on bare nodes through Placed. The body records
// the composed motion, and its volume and centroid are the square frustum's
// closed forms moved by that motion.
func TestPlacedBuildsTheFrustumUnderTheComposedMotion(t *testing.T) {
	t.Parallel()
	const h = 8.0
	alpha := 5 * units.Degree.Factor()
	rec := draftrecord.Payload{
		Profile: centeredSquare(5), Frame: xyFrame(t), Z0: 0, Z1: h,
		NearStart: true, Taper: alpha, Xform: r3.Identity(),
	}
	shift := r3.NewVec(3, -2, 7)
	composed, err := r3.Translation(shift)
	require.NoError(t, err)
	doc := &topo.Document{}
	body, err := rec.Placed(t.Context(), topo.Env{Doc: doc, Ref: 4, Alloc: topotest.Alloc{}}, composed)
	require.NoError(t, err)
	require.Same(t, doc, body.Doc)
	got, ok := body.Payload.(draftrecord.Payload)
	require.True(t, ok)
	require.Equal(t, composed, got.Xform)
	require.InDelta(t, h*math.Tan(alpha), got.D, 1e-12)

	side := 10 - 2*h*math.Tan(alpha)
	a0, a1 := 100.0, side*side
	mid := math.Sqrt(a0 * a1)
	volume := h / 3 * (a0 + mid + a1)
	zc := h * (a0 + 2*mid + 3*a1) / (4 * (a0 + mid + a1))
	require.InDelta(t, volume, body.Volume.Value.Base(), body.Volume.Bound.Base()+1e-9)
	want := r3.NewVec(shift.X, shift.Y, shift.Z+zc)
	require.InDelta(t, want.X, body.Centroid.Value.X, body.Centroid.Bound.Base()+1e-9)
	require.InDelta(t, want.Y, body.Centroid.Value.Y, body.Centroid.Bound.Base()+1e-9)
	require.InDelta(t, want.Z, body.Centroid.Value.Z, body.Centroid.Bound.Base()+1e-9)
}

// TestCapPrismsReadTheRecordedLevels pins the gate stations' sections: the
// near section at the near level, the recorded far section at the far level,
// and their displacement, the axial and far contour displacements summed.
func TestCapPrismsReadTheRecordedLevels(t *testing.T) {
	t.Parallel()
	rec := draftrecord.Payload{
		Profile: centeredSquare(5), Far: centeredSquare(6), Frame: xyFrame(t),
		Z0: 0, Z1: 8, Z0Delta: 0.25, NearStart: false, FarDelta: 0.5, Xform: r3.Identity(),
	}
	prisms, displacement := rec.CapPrisms()
	require.Len(t, prisms, 2)
	require.Equal(t, rec.Profile, prisms[0].Profile)
	require.Equal(t, [2]float64{8, 8}, [2]float64{prisms[0].Z0, prisms[0].Z1})
	require.Equal(t, rec.Far, prisms[1].Profile)
	require.Equal(t, [2]float64{0, 0}, [2]float64{prisms[1].Z0, prisms[1].Z1})
	require.InDelta(t, 0.75, displacement, 1e-15)
	require.GreaterOrEqual(t, displacement, 0.75)
}

// TestMoverRecordRadiusCoversTheFarSection pins the record radius: the L1
// reach of the larger section coordinate envelope, far contour gap included,
// plus the sweep's axial reach.
func TestMoverRecordRadiusCoversTheFarSection(t *testing.T) {
	t.Parallel()
	rec := draftrecord.Payload{
		Profile: centeredSquare(5), Far: centeredSquare(6), Frame: xyFrame(t),
		Z0: 0, Z1: 8, NearStart: true, FarDelta: 0.5, Xform: r3.Identity(),
	}
	radius := rec.MoverRecordRadius()
	require.InDelta(t, 21.0, radius, 1e-12)
	require.GreaterOrEqual(t, radius, 21.0)
}
