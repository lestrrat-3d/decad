package apitest_test

import (
	"math"
	"strconv"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"

	"github.com/lestrrat-3d/decad"
)

// joinedByPattern patterns one lobe about the disc's axis and joins the copies.
func joinedByPattern(t *testing.T, r, height float64, n int, axis r3.Vec) (*decad.Body, error) {
	t.Helper()
	doc := decad.New()
	disc := circleBodyAtZ(t, doc, 0, r, 0, height)
	lobe := lobeLoft(t, doc, r, 0, 0.4*math.Pi/float64(n), height)
	copies, err := decad.PatternCopies(t.Context(), lobe, decad.CircularPattern{Axis: axis, Count: n})
	require.NoError(t, err)
	return decad.Union(t.Context(), disc, append([]*decad.Body{lobe}, copies...)...)
}

// drawnAtAngles joins the same lobes each drawn at its own angle.
func drawnAtAngles(t *testing.T, n int) *decad.Body {
	t.Helper()
	const r, height = 3.0, 2.0
	doc := decad.New()
	disc := circleBodyAtZ(t, doc, 0, r, 0, height)
	lobes := make([]*decad.Body, n)
	for k := range lobes {
		lobes[k] = lobeLoft(t, doc, r, 2*math.Pi*float64(k)/float64(n), 0.4*math.Pi/float64(n), height)
	}
	joined, err := decad.Union(t.Context(), disc, lobes...)
	require.NoError(t, err)
	return joined
}

// TestPatternedLobesJoinADisc joins one lobe and its pattern copies to a disc and
// checks the result against the same lobes each drawn at its angle: the two volumes
// agree within their bounds, and the join's bound carries the rotation's rounding.
func TestPatternedLobesJoinADisc(t *testing.T) {
	t.Parallel()
	for _, n := range []int{8, 12, 17, 24} {
		t.Run(strconv.Itoa(n)+" copies", func(t *testing.T) {
			t.Parallel()
			joined, err := joinedByPattern(t, 3.0, 2.0, n, r3.NewVec(0, 0, 1))
			require.NoError(t, err)
			require.True(t, joined.IsSolid())
			drawn := drawnAtAngles(t, n)
			got, err := joined.Volume()
			require.NoError(t, err)
			want, err := drawn.Volume()
			require.NoError(t, err)
			require.InDelta(t, want.Value.Base(), got.Value.Base(), want.Bound.Base()+got.Bound.Base())
			require.Greater(t, got.Bound.Base(), want.Bound.Base(),
				"placed operands are re-recorded at float precision, and the join says so")
			mesh, err := joined.Tessellate(t.Context(), units.Millimeters(0.05), decad.WithVerification(decad.VerifyAll))
			require.NoError(t, err)
			require.True(t, mesh.BoundaryVerified())
			require.True(t, mesh.VolumeVerified())
		})
	}
}

// TestPatternedLobesAboutATiltedAxisAreRefused turns the copies about an axis that
// tilts out of the lobes' plane: no in-plane map exists, so the join takes no
// axial-section route and refuses rather than rounding the tilt away.
func TestPatternedLobesAboutATiltedAxisAreRefused(t *testing.T) {
	t.Parallel()
	_, err := joinedByPattern(t, 2.5, 1.5, 8, r3.NewVec(0.05, 0, 1))
	require.ErrorIs(t, err, decad.ErrUnsupported)
}

// TestPatternJoinMatchesPatternCopiesThenUnion pins the one-call form to the two-call
// form on the same lobes, and its document effect: target and feature retire and no
// instance is registered.
func TestPatternJoinMatchesPatternCopiesThenUnion(t *testing.T) {
	t.Parallel()
	const r, height, n = 3.0, 2.0, 12
	doc := decad.New()
	disc := circleBodyAtZ(t, doc, 0, r, 0, height)
	lobe := lobeLoft(t, doc, r, 0, 0.4*math.Pi/n, height)
	spec := decad.CircularPattern{Axis: r3.NewVec(0, 0, 1), Count: n}
	joined, err := decad.PatternJoin(t.Context(), disc, lobe, spec)
	require.NoError(t, err)
	require.True(t, joined.IsSolid())
	require.Equal(t, []*decad.Body{joined}, doc.Bodies())

	want, err := joinedByPattern(t, r, height, n, r3.NewVec(0, 0, 1))
	require.NoError(t, err)
	wv, err := want.Volume()
	require.NoError(t, err)
	gv, err := joined.Volume()
	require.NoError(t, err)
	require.InDelta(t, wv.Value.Base(), gv.Value.Base(), wv.Bound.Base()+gv.Bound.Base())
}

func TestPatternJoinRefusalLeavesTheDocumentUnchanged(t *testing.T) {
	t.Parallel()
	const r, height, n = 3.0, 2.0, 8
	doc := decad.New()
	disc := circleBodyAtZ(t, doc, 0, r, 0, height)
	lobe := lobeLoft(t, doc, r, 0, 0.4*math.Pi/n, height)
	before := doc.Bodies()
	_, err := decad.PatternJoin(t.Context(), disc, lobe,
		decad.CircularPattern{Axis: r3.NewVec(0.05, 0, 1), Count: n})
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.Equal(t, before, doc.Bodies())

	_, err = decad.PatternJoin(t.Context(), lobe, lobe, decad.CircularPattern{Axis: r3.NewVec(0, 0, 1), Count: n})
	require.ErrorIs(t, err, decad.ErrDegenerate)
	_, err = decad.PatternJoin(t.Context(), nil, lobe, decad.CircularPattern{Axis: r3.NewVec(0, 0, 1), Count: n})
	require.ErrorIs(t, err, decad.ErrDegenerate)
	_, err = decad.PatternJoin(t.Context(), disc, lobe, decad.CircularPattern{Axis: r3.NewVec(0, 0, 1), Count: 1})
	require.ErrorIs(t, err, decad.ErrDegenerate)
	require.Equal(t, before, doc.Bodies())
}

// TestPatternJoinSlidesATongueAlongAPlate repeats a tongue with a fit-spline top along
// a plate, three times, each crossing the plate's top edge: the joined volume is the
// plate and three tongues less three overlaps of 8 mm² of section.
func TestPatternJoinSlidesATongueAlongAPlate(t *testing.T) {
	t.Parallel()
	const height = 5.0
	doc := decad.New()
	plate := polyPrismBody(t, doc, [][2]float64{{0, 0}, {30, 0}, {30, 6}, {0, 6}}, height)
	tongue := tongueLoft(t, doc, height)
	pv, err := plate.Volume()
	require.NoError(t, err)
	tv, err := tongue.Volume()
	require.NoError(t, err)
	joined, err := decad.PatternJoin(t.Context(), plate, tongue,
		decad.LinearPattern{Dir: r3.NewVec(1, 0, 0), Step: units.Millimeters(6), Count: 3})
	require.NoError(t, err)
	v, err := joined.Volume()
	require.NoError(t, err)
	want := pv.Value.Base() + 3*tv.Value.Base() - 3*8*height
	require.InDelta(t, want, v.Value.Base(), v.Bound.Base()+pv.Bound.Base()+3*tv.Bound.Base()+1e-9)
	require.True(t, joined.IsSolid())
}

// obliqueFrames returns a frame no axis aligns with and the frame lifted height along
// its normal.
func obliqueFrames(t *testing.T, height float64) (r3.Frame, r3.Frame) {
	t.Helper()
	rot, err := r3.RotationAround(r3.Vec{}, r3.NewVec(1, 1, 1), units.Radians(math.Pi/6))
	require.NoError(t, err)
	u, v := rot.ApplyDir(r3.NewVec(1, 0, 0)), rot.ApplyDir(r3.NewVec(0, 1, 0))
	origin := r3.NewVec(3, -2, 5)
	f0, err := r3.NewFrame(origin, u, v)
	require.NoError(t, err)
	f1, err := r3.NewFrame(origin.Add(f0.N().Scale(height)), u, v)
	require.NoError(t, err)
	return f0, f1
}

// obliquePatternJoin builds a disc and one lobe on an oblique frame, patterns the lobe
// about the frame's normal through its origin and joins everything.
func obliquePatternJoin(t *testing.T, r, height float64, n int) (*decad.Body, error) {
	t.Helper()
	f0, f1 := obliqueFrames(t, height)
	w := sketch.NewWorld()
	p0, err := w.CreatePlaneFromFrame(f0)
	require.NoError(t, err)
	p1, err := w.CreatePlaneFromFrame(f1)
	require.NoError(t, err)
	doc := decad.New()
	ds, err := w.CreateSketch(p0)
	require.NoError(t, err)
	ds.CreateCircle(ds.Origin(), r)
	_, err = ds.Solve(t.Context())
	require.NoError(t, err)
	disc, err := decad.Extrude(t.Context(), doc, ds, ds.Profiles()[0],
		decad.Distance{D: units.Millimeters(height), Dir: decad.Along})
	require.NoError(t, err)

	half := 0.4 * math.Pi / float64(n)
	section := func(plane *sketch.Plane) (*sketch.Sketch, *sketch.Profile) {
		s, err := w.CreateSketch(plane)
		require.NoError(t, err)
		pt := func(rad, a float64) *sketch.Point {
			p := s.CreatePoint(rad*math.Cos(a), rad*math.Sin(a))
			s.Fix(p)
			return p
		}
		const fit = 5
		rr, rl := pt(r, -half), pt(r, half)
		right, left := make([]*sketch.Point, fit), make([]*sketch.Point, fit)
		for i := range fit {
			f := float64(i) / float64(fit-1)
			right[i], left[i] = pt(r+0.35+f*0.65, -half*(1-0.4*f)), pt(r+0.35+f*0.65, half*(1-0.4*f))
		}
		s.CreateLine(rr, right[0])
		_, err = s.CreateFitSpline(right...)
		require.NoError(t, err)
		s.CreateArc(s.Origin(), right[fit-1], left[fit-1])
		_, err = s.CreateFitSpline(left...)
		require.NoError(t, err)
		s.CreateLine(left[0], rl)
		s.CreateArc(s.Origin(), rr, rl)
		_, err = s.Solve(t.Context())
		require.NoError(t, err)
		require.Len(t, s.Profiles(), 1)
		return s, s.Profiles()[0]
	}
	s0, q0 := section(p0)
	s1, q1 := section(p1)
	lobe, err := decad.Loft(t.Context(), doc, s0, q0, s1, q1)
	require.NoError(t, err)
	return decad.PatternJoin(t.Context(), disc, lobe,
		decad.CircularPattern{Center: f0.Origin(), Axis: f0.N(), Count: n})
}

// TestPatternJoinOnAnObliqueFrame joins patterned lobes on a frame no axis aligns with.
// A rigid motion keeps volume, so the result matches the same part on the world frame.
func TestPatternJoinOnAnObliqueFrame(t *testing.T) {
	t.Parallel()
	const r, height, n = 3.0, 2.0, 12
	joined, err := obliquePatternJoin(t, r, height, n)
	require.NoError(t, err)
	require.True(t, joined.IsSolid())
	want, err := joinedByPattern(t, r, height, n, r3.NewVec(0, 0, 1))
	require.NoError(t, err)
	gv, err := joined.Volume()
	require.NoError(t, err)
	wv, err := want.Volume()
	require.NoError(t, err)
	require.InDelta(t, wv.Value.Base(), gv.Value.Base(), wv.Bound.Base()+gv.Bound.Base())
	mesh, err := joined.Tessellate(t.Context(), units.Millimeters(0.05), decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
}

// TestPatternJoinManyCopies joins 64 and 128 patterned lobes to a disc and checks the volume
// against the disc and the lobes: the sum of the parts, within the bounds of all of them.
func TestPatternJoinManyCopies(t *testing.T) {
	t.Parallel()
	for _, n := range []int{64, 128} {
		t.Run(strconv.Itoa(n)+" copies", func(t *testing.T) {
			t.Parallel()
			const r, height = 3.0, 2.0
			doc := decad.New()
			disc := circleBodyAtZ(t, doc, 0, r, 0, height)
			lobe := lobeLoft(t, doc, r, 0, 0.4*math.Pi/float64(n), height)
			dv, err := disc.Volume()
			require.NoError(t, err)
			lv, err := lobe.Volume()
			require.NoError(t, err)
			joined, err := decad.PatternJoin(t.Context(), disc, lobe,
				decad.CircularPattern{Axis: r3.NewVec(0, 0, 1), Count: n})
			require.NoError(t, err)
			require.True(t, joined.IsSolid())
			v, err := joined.Volume()
			require.NoError(t, err)
			want := dv.Value.Base() + float64(n)*lv.Value.Base()
			require.InDelta(t, want, v.Value.Base(), v.Bound.Base()+dv.Bound.Base()+float64(n)*lv.Bound.Base())
		})
	}
}
