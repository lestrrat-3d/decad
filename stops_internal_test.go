package decad

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// TestResolveThroughAllComposesEveryFarEndInterval keeps the recorded maximum
// unchanged while bounding a lower held endpoint that can be farther in the
// denoted geometry. It also refuses a competing interval with no finite upper
// endpoint instead of publishing an unrepresentable stop bound.
func TestResolveThroughAllComposesEveryFarEndInterval(t *testing.T) {
	frame := canonicalPrismFrame(t)
	profile := ProfileRecord{Outer: synthRectLoop(0, 0, 1, 1)}
	stopBody := func(ref StepRef, z1, z1Delta float64) *Body {
		return &Body{
			origin: FeatureRef{Step: ref},
			payload: prismPayload{
				profile: profile,
				frame:   frame,
				z1:      z1,
				z1Delta: z1Delta,
				xform:   r3.Identity(),
			},
		}
	}

	t.Run("bounds a lower held contender", func(t *testing.T) {
		doc := &Document{bodies: []*Body{
			stopBody(1, 9.25, 1),
			stopBody(2, 10, 0),
		}}

		stop, bound, refs, err := doc.resolveThroughAll(frame, 1)
		require.NoError(t, err)
		require.Equal(t, 10.0, stop, "the held farthest endpoint remains the stop")
		require.Equal(t, []StepRef{1, 2}, refs, "the dependency order remains nearest-first")
		require.GreaterOrEqual(t, bound, 0.25, "the lower held endpoint can be 0.25 mm farther")
	})

	t.Run("refuses an unrepresentable contender", func(t *testing.T) {
		doc := &Document{bodies: []*Body{
			stopBody(1, math.MaxFloat64, math.MaxFloat64),
			stopBody(2, math.MaxFloat64, 0),
		}}

		_, _, _, err := doc.resolveThroughAll(frame, 1)
		require.ErrorIs(t, err, ErrUnsupported)
	})
}

// TestResolveToFaceUsesSelectedCapAxialDelta keeps a computed end's bound on
// that cap only. A later ToFace against the exact opposite cap must remain
// exact, while selecting the computed cap must retain its bound.
func TestResolveToFaceUsesSelectedCapAxialDelta(t *testing.T) {
	frame := canonicalPrismFrame(t)
	doc := &Document{}
	host, err := evalPrism(doc, 1, prismPayload{
		profile: ProfileRecord{Outer: synthRectLoop(0, 0, 1, 1)},
		frame:   frame,
		z0:      10,
		z1:      20,
		z1Delta: 0.25,
		xform:   r3.Identity(),
	}, newFreeformWork())
	require.NoError(t, err)
	doc.bodies = []*Body{host}

	for _, tc := range []struct {
		name  string
		role  string
		stop  float64
		delta float64
	}{
		{name: "exact start cap", role: roleCapStart, stop: 10, delta: 0},
		{name: "computed end cap", role: roleCapEnd, stop: 20, delta: 0.25},
	} {
		t.Run(tc.name, func(t *testing.T) {
			stop, delta, _, err := doc.resolveToFace(ToFace{
				Body: host,
				Face: Faces(FaceCreatedBy(FeatureRef{Step: 1, Role: tc.role})),
			}, frame, 1, "a to-face extent")
			require.NoError(t, err)
			require.Equal(t, tc.stop, stop)
			if tc.delta == 0 {
				require.Zero(t, delta)
				return
			}
			require.GreaterOrEqual(t, delta, tc.delta)
		})
	}
}
