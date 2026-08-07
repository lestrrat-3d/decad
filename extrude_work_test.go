package decad

import (
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestEvalPrismContinuesCallerFreeformWork(t *testing.T) {
	profile := ProfileRecord{Outer: LoopRecord{Segments: []CurveSegment{
		SplineSeg{
			Control: []Point2{{U: 2}, {U: 2, V: 2}, {V: 2}, {}},
			TStart:  0,
			TEnd:    1,
		},
		LineSeg{Start: Point2{}, End: Point2{U: 2}, TStart: 0, TEnd: 1},
	}}}
	work := newFreeformWork()
	_, err := profile.evaluatorIntegrals(momentAreaOrder, work)
	require.NoError(t, err)
	spent := work.spent
	frame, err := r3.NewFrame(r3.Vec{}, r3.Vec{X: 1}, r3.Vec{Y: 1})
	require.NoError(t, err)

	_, err = evalPrism(New(), 0, prismPayload{
		profile: profile,
		frame:   frame,
		z1:      1,
		xform:   r3.Identity(),
	}, work)
	require.ErrorIs(t, err, ErrUnsupported)
	require.Greater(t, work.spent, spent)
}
