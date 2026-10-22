package apitest_test

import (
	"testing"

	"github.com/lestrrat-3d/r3"
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
