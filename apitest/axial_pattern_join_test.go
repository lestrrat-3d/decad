package apitest_test

import (
	"math"
	"strconv"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"

	"github.com/lestrrat-3d/decad"
)

// joinedByPattern patterns one lobe about the disc's axis and joins the copies.
func joinedByPattern(t *testing.T, r, height float64, n int, center r3.Vec, axis r3.Vec) (*decad.Body, error) {
	t.Helper()
	doc := decad.New()
	disc := circleBodyAtZ(t, doc, 0, r, 0, height)
	lobe := lobeLoft(t, doc, r, 0, 0.4*math.Pi/float64(n), height)
	copies, err := decad.PatternCopies(t.Context(), lobe, decad.CircularPattern{Center: center, Axis: axis, Count: n})
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
			joined, err := joinedByPattern(t, 3.0, 2.0, n, r3.Vec{}, r3.NewVec(0, 0, 1))
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
	_, err := joinedByPattern(t, 2.5, 1.5, 8, r3.Vec{}, r3.NewVec(0.05, 0, 1))
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

	want, err := joinedByPattern(t, r, height, n, r3.Vec{}, r3.NewVec(0, 0, 1))
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
