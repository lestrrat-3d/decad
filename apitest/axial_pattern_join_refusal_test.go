package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"

	"github.com/lestrrat-3d/decad"
)

// TestPatternJoinRefusesCopiesItCannotCertify slides a 4 mm wide tongue along a plate. At a
// 4 mm step the copies share whole side edges and join. At a 3 mm step the arched tops cross
// each other, and at a step a hair past 4 mm the sides nearly coincide: sketch cannot certify
// either arrangement, so the join refuses with ErrUnsupported and leaves the document as it was.
func TestPatternJoinRefusesCopiesItCannotCertify(t *testing.T) {
	t.Parallel()
	for name, step := range map[string]float64{"arches cross": 3, "sides nearly coincide": 4.0000001} {
		t.Run(name, func(t *testing.T) {
			t.Parallel()
			doc := decad.New()
			plate := polyPrismBody(t, doc, [][2]float64{{0, 0}, {30, 0}, {30, 6}, {0, 6}}, 5)
			tongue := tongueLoft(t, doc, 5)
			before := doc.Bodies()
			_, err := decad.PatternJoin(t.Context(), plate, tongue,
				decad.LinearPattern{Dir: r3.NewVec(1, 0, 0), Step: units.Millimeters(step), Count: 3})
			require.ErrorIs(t, err, decad.ErrUnsupported)
			require.Equal(t, before, doc.Bodies())
		})
	}
}

// TestPatternJoinJoinsCopiesThatShareWholeSides is the step the refusals above are measured against.
func TestPatternJoinJoinsCopiesThatShareWholeSides(t *testing.T) {
	t.Parallel()
	const height = 4.0
	doc := decad.New()
	plate := polyPrismBody(t, doc, [][2]float64{{0, 0}, {30, 0}, {30, 6}, {0, 6}}, height)
	tongue := tongueLoft(t, doc, height)
	pv, err := plate.Volume()
	require.NoError(t, err)
	tv, err := tongue.Volume()
	require.NoError(t, err)
	joined, err := decad.PatternJoin(t.Context(), plate, tongue,
		decad.LinearPattern{Dir: r3.NewVec(1, 0, 0), Step: units.Millimeters(4), Count: 3})
	require.NoError(t, err)
	v, err := joined.Volume()
	require.NoError(t, err)
	want := pv.Value.Base() + 3*tv.Value.Base() - 3*8*height
	require.InDelta(t, want, v.Value.Base(), v.Bound.Base()+pv.Bound.Base()+3*tv.Bound.Base()+1e-9)
}

// TestPatternJoinRefusesLobesThatOnlyTouchTheBase builds lobes whose bottom arc rests on the
// disc's circle with the flank ends on it, and lobes a hair above it. The first touch the disc along
// an arc and at the flank ends, the second leave a gap: neither joins into one region sketch can
// certify, so the join refuses with ErrUnsupported and leaves the document as it was.
func TestPatternJoinRefusesLobesThatOnlyTouchTheBase(t *testing.T) {
	t.Parallel()
	const r, height, n = 3.0, 2.0, 12
	doc := decad.New()
	disc := circleBodyAtZ(t, doc, 0, r, 0, height)
	lobe := gapLoft(t, doc, r, 0, 0.4*math.Pi/n, height)
	before := doc.Bodies()
	_, err := decad.PatternJoin(t.Context(), disc, lobe,
		decad.CircularPattern{Axis: r3.NewVec(0, 0, 1), Count: n})
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.Equal(t, before, doc.Bodies())
}

// TestPatternJoinKeepsLobesThatMissTheBaseByAMicronApart is the other case: lobes a micron off the disc leave a gap
// sketch cannot certify away, so no certified route takes the set, and the two-at-a-time fold builds the Union of
// bodies that do not touch: a body of the disc and the twelve lobes as thirteen lumps, whose volume is their sum, which
// the test reads from the operands and not from the join.
func TestPatternJoinKeepsLobesThatMissTheBaseByAMicronApart(t *testing.T) {
	t.Parallel()
	const r, height, n = 3.0, 2.0, 12
	doc := decad.New()
	disc := circleBodyAtZ(t, doc, 0, r, 0, height)
	lobe := gapLoft(t, doc, r, 1e-3, 0.4*math.Pi/n, height)
	discVolume, err := disc.Volume()
	require.NoError(t, err)
	lobeVolume, err := lobe.Volume()
	require.NoError(t, err)
	out, err := decad.PatternJoin(t.Context(), disc, lobe,
		decad.CircularPattern{Axis: r3.NewVec(0, 0, 1), Count: n})
	require.NoError(t, err)
	require.Equal(t, []*decad.Body{out}, doc.Bodies())
	require.Len(t, out.Lumps(), n+1, "the lobes stay clear of the disc")
	volume, err := out.Volume()
	require.NoError(t, err)
	want := discVolume.Value.Base() + n*lobeVolume.Value.Base()
	bound := volume.Bound.Base() + discVolume.Bound.Base() + n*lobeVolume.Bound.Base()
	require.InDelta(t, want, volume.Value.Base(), bound+0.01, "disc plus lobes, less nothing: the lobes are clear of the disc")
}

// gapLoft lofts a lobe whose bottom arc is concentric with the disc at radius r + gap.
func gapLoft(t *testing.T, doc *decad.Document, r, gap, half, height float64) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	section := func(plane *sketch.Plane) (*sketch.Sketch, *sketch.Profile) {
		s, err := w.CreateSketch(plane)
		require.NoError(t, err)
		pt := func(rad, a float64) *sketch.Point {
			p := s.CreatePoint(rad*math.Cos(a), rad*math.Sin(a))
			s.Fix(p)
			return p
		}
		const fit = 5
		right, left := make([]*sketch.Point, fit), make([]*sketch.Point, fit)
		for i := range fit {
			f := float64(i) / float64(fit-1)
			rad, a := r+gap+0.65*f, half*(1-0.4*f)
			right[i], left[i] = pt(rad, -a), pt(rad, a)
		}
		_, err = s.CreateFitSpline(right...)
		require.NoError(t, err)
		s.CreateArc(s.Origin(), right[fit-1], left[fit-1])
		_, err = s.CreateFitSpline(left...)
		require.NoError(t, err)
		s.CreateArc(s.Origin(), right[0], left[0])
		_, err = s.Solve(t.Context())
		require.NoError(t, err)
		require.Len(t, s.Profiles(), 1)
		return s, s.Profiles()[0]
	}
	s0, p0 := section(w.XY())
	s1, p1 := section(splitTopPlane(t, w, height))
	b, err := decad.Loft(t.Context(), doc, s0, p0, s1, p1)
	require.NoError(t, err)
	return b
}
