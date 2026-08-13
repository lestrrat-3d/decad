package decad

import (
	"context"
	"math"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// stubExtent is a payload whose directional extent publishes exactly the far
// side and displacement a case asks for. The in-path test's three outcomes turn
// on where that displacement sits against the sketch plane, and driving them
// through real geometry would mean hunting for a section whose bracket lands on
// the scale-relative contact tolerance itself.
type stubExtent struct{ hi, bound float64 }

func (stubExtent) transform() r3.Transform { return r3.Identity() }

func (stubExtent) placed(context.Context, *Document, StepRef, r3.Transform) (*Body, error) {
	return nil, ErrUnsupported
}

//nolint:unparam // the error result is the directionalExtent interface's, and this stub never fails.
func (s stubExtent) extentAlong(r3.Vec) (float64, float64, float64, error) {
	return -s.hi, s.hi, s.bound, nil
}

// TestResolveThroughAllDecidesInPathOutsideDisplacement covers the three
// outcomes of the in-path test (docs/spline-design.md §6.4): a far side beyond
// the sketch plane by more than its own displacement is met and charges that
// displacement to the level, one short of the plane by more than it is not met,
// and one the displacement straddles the plane with refuses rather than guess a
// dependency.
func TestResolveThroughAllDecidesInPathOutsideDisplacement(t *testing.T) {
	frame := canonicalPrismFrame(t)
	doc := func(hi, bound float64) *Document {
		return &Document{bodies: []*Body{{
			origin:  FeatureRef{Step: 1},
			payload: stubExtent{hi: hi, bound: bound},
		}}}
	}

	t.Run("beyond the plane", func(t *testing.T) {
		stop, delta, refs, err := doc(10, 1).resolveThroughAll(frame, 1)
		require.NoError(t, err)
		require.Equal(t, 10.0, stop, "the sweep stops at the held far side")
		require.Equal(t, []StepRef{1}, refs)
		require.GreaterOrEqual(t, delta, 1.0, "the level carries the extent's own displacement")
	})

	t.Run("short of the plane", func(t *testing.T) {
		_, _, _, err := doc(-10, 1).resolveThroughAll(frame, 1)
		require.ErrorIs(t, err, ErrDegenerate, "no body is in the sweep's path")
	})

	t.Run("straddling the plane", func(t *testing.T) {
		_, _, _, err := doc(0.5, 1).resolveThroughAll(frame, 1)
		require.ErrorIs(t, err, ErrUnsupported)
		require.ErrorContains(t, err, "cannot decide whether a body is in its path")
	})
}

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
