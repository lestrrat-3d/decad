package decad_test

import (
	"encoding/json"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// outwardNormal reads a planar face's proven outward normal; NormalAt ignores
// the point for a plane, so any point serves.
func outwardNormal(t *testing.T, f *decad.Face) r3.Vec {
	t.Helper()
	nm, err := f.NormalAt(r3.NewVec(0, 0, 0))
	require.NoError(t, err)
	return nm.Value
}

func TestFacingPredicate(t *testing.T) {
	body := holePlateBody(t) // 100×60×8 plate, hole; caps at z=0 (capStart) and z=8 (capEnd)

	t.Run("PicksOneCapWhereNormalToPicksTwo", func(t *testing.T) {
		both, err := decad.Faces(decad.NormalTo(zAxis)).SelectFaces(body)
		require.NoError(t, err)
		require.Len(t, both, 2, `NormalTo matches both caps, either sense`)

		top, err := decad.Faces(decad.Planar(), decad.Facing(zAxis)).Exactly(1).SelectFaces(body)
		require.NoError(t, err)
		require.Len(t, top, 1, `Facing(+z) is the single upward-facing cap`)
		require.Positive(t, outwardNormal(t, top[0]).Dot(zAxis),
			`the matched cap's outward normal points ALONG +z`)
	})

	t.Run("OppositeSensePicksTheOtherCap", func(t *testing.T) {
		bottom, err := decad.Faces(decad.Planar(), decad.Facing(zAxis.Scale(-1))).Exactly(1).SelectFaces(body)
		require.NoError(t, err)
		require.Len(t, bottom, 1)
		require.Negative(t, outwardNormal(t, bottom[0]).Dot(zAxis),
			`Facing(-z) is the downward-facing cap`)

		// The two senses select disjoint single faces.
		top, err := decad.Faces(decad.Facing(zAxis)).SelectFaces(body)
		require.NoError(t, err)
		require.NotEqual(t, top[0].Origins(), bottom[0].Origins(),
			`Facing(+z) and Facing(-z) are different caps`)
	})

	t.Run("PicksOneSideWall", func(t *testing.T) {
		// The x=100 wall faces +x; the x=0 wall faces -x. NormalTo(x) matches
		// both, Facing(x) only the one whose material-leaving normal is +x.
		xWall, err := decad.Faces(decad.Facing(r3.NewVec(1, 0, 0))).Exactly(1).SelectFaces(body)
		require.NoError(t, err)
		require.Positive(t, outwardNormal(t, xWall[0]).Dot(r3.NewVec(1, 0, 0)))
	})

	t.Run("NonPlanarNeverMatches", func(t *testing.T) {
		// The hole cylinder is not planar, and no direction faces it.
		_, err := decad.Faces(decad.Facing(r3.NewVec(1, 0, 0)), decad.Cylindrical()).SelectFaces(body)
		require.ErrorIs(t, err, decad.ErrNoMatch)
	})

	t.Run("HugeFiniteDirectionParallelToNoWallMatchesNothing", func(t *testing.T) {
		// A finite but huge 45° direction (parallel to no axis-aligned wall's
		// normal) must match nothing. Taking its length squares its components
		// to +Inf, so a scale-blind parallel test would wrongly report it
		// parallel to every wall; the scaled direction keeps it honest.
		_, err := decad.Faces(decad.Facing(r3.NewVec(math.MaxFloat64, math.MaxFloat64, 0))).SelectFaces(body)
		require.ErrorIs(t, err, decad.ErrNoMatch,
			`a huge finite 45° direction is parallel to no box wall's normal`)

		// The +x wall still matches when the huge direction really is +x.
		xWall, err := decad.Faces(decad.Facing(r3.NewVec(math.MaxFloat64, 0, 0))).Exactly(1).SelectFaces(body)
		require.NoError(t, err)
		require.Positive(t, outwardNormal(t, xWall[0]).Dot(r3.NewVec(1, 0, 0)),
			`a huge +x direction still faces the +x wall`)
	})

	t.Run("SubnormalDirectionStillNamesTheRay", func(t *testing.T) {
		// A subnormal +x direction must still face the +x wall. Its
		// infinity-norm m is that same subnormal, so 1/m overflows to +Inf and
		// v.Scale(1/m) would yield {+Inf, NaN, NaN}; scaling by component
		// division keeps m/m = 1, a finite unit-∞-norm {1, 0, 0}.
		xWall, err := decad.Faces(decad.Facing(r3.NewVec(math.SmallestNonzeroFloat64, 0, 0))).Exactly(1).SelectFaces(body)
		require.NoError(t, err)
		require.Positive(t, outwardNormal(t, xWall[0]).Dot(r3.NewVec(1, 0, 0)),
			`a subnormal +x direction still faces the +x wall`)
	})

	t.Run("DegenerateDirectionRejectedAtResolve", func(t *testing.T) {
		_, err := decad.Faces(decad.Facing(r3.NewVec(0, 0, 0))).SelectFaces(body)
		require.ErrorIs(t, err, decad.ErrDegenerate, `a zero direction is rejected, as NormalTo's is`)
		_, err = decad.Faces(decad.Facing(r3.NewVec(math.NaN(), 0, 1))).SelectFaces(body)
		require.ErrorIs(t, err, decad.ErrNotFinite, `a non-finite direction is rejected`)
	})
}

func TestFacingCodec(t *testing.T) {
	// A facing predicate round-trips through a Step field for field, and its
	// dir payload is required — an absent dir is malformed, never a zero vector.
	step := decad.Step{
		Op:        decad.OpShell,
		Inputs:    []decad.StepRef{0},
		Selectors: []decad.Selector{decad.Faces(decad.Planar(), decad.Facing(zAxis)).Exactly(1)},
	}
	buf, err := json.Marshal(step)
	require.NoError(t, err)
	require.Contains(t, string(buf), `"kind":"facing"`)
	var got decad.Step
	require.NoError(t, json.Unmarshal(buf, &got))
	require.Equal(t, step, got, `a step carrying a facing predicate round-trips`)

	require.Error(t, json.Unmarshal(
		[]byte(`{"op":"shell","selectors":[{"kind":"faces","preds":[{"kind":"facing"}]}]}`), &got),
		`a facing predicate with no dir is malformed`)
}

// TestMissingDirErrorWording pins the human-readable missing-dir decode error
// for each direction-bearing face predicate. The wire token and the display
// name differ deliberately (normal_to -> normal-to), so this guards against the
// raw tag leaking back into the message a caller reads.
func TestMissingDirErrorWording(t *testing.T) {
	for _, tc := range []struct {
		kind string
		want string
	}{
		{"normal_to", `decad: a normal-to predicate requires dir`},
		{"facing", `decad: a facing predicate requires dir`},
	} {
		t.Run(tc.kind, func(t *testing.T) {
			var got decad.Step
			err := json.Unmarshal(
				[]byte(`{"op":"shell","selectors":[{"kind":"faces","preds":[{"kind":"`+tc.kind+`"}]}]}`),
				&got)
			require.ErrorContains(t, err, tc.want)
		})
	}
}

// TestMalformedDirErrorWording pins the OTHER decode-error path — a dir that is
// present but malformed (a non-vector) — for the same predicates. Like the
// missing-dir message, it must render the display name, never the raw wire tag,
// so NormalTo's established wording is unchanged.
func TestMalformedDirErrorWording(t *testing.T) {
	for _, tc := range []struct {
		kind string
		want string
	}{
		{"normal_to", `decad: failed to decode normal-to predicate`},
		{"facing", `decad: failed to decode facing predicate`},
	} {
		t.Run(tc.kind, func(t *testing.T) {
			var got decad.Step
			err := json.Unmarshal(
				[]byte(`{"op":"shell","selectors":[{"kind":"faces","preds":[{"kind":"`+tc.kind+`","dir":"bad"}]}]}`),
				&got)
			require.ErrorContains(t, err, tc.want)
		})
	}
}

func TestCapHelpers(t *testing.T) {
	body := holePlateBody(t)

	t.Run("CapStartAndCapEndNameTheRightFaces", func(t *testing.T) {
		require.Equal(t,
			decad.FeatureRef{Step: body.Origin().Step, Role: roleCapStart},
			decad.CapStart(body), `CapStart pairs the body's own step with the fixed role`)
		require.Equal(t,
			decad.FeatureRef{Step: body.Origin().Step, Role: roleCapEnd},
			decad.CapEnd(body))

		start, err := decad.Faces(decad.FaceCreatedBy(decad.CapStart(body))).Exactly(1).SelectFaces(body)
		require.NoError(t, err)
		require.Contains(t, start[0].Origins(), decad.CapStart(body))
		require.Negative(t, outwardNormal(t, start[0]).Dot(zAxis), `the start cap faces down (−z)`)

		end, err := decad.Faces(decad.FaceCreatedBy(decad.CapEnd(body))).Exactly(1).SelectFaces(body)
		require.NoError(t, err)
		require.Positive(t, outwardNormal(t, end[0]).Dot(zAxis), `the end cap faces up (+z)`)

		// CapStart names the same cap Facing(−z) selects by geometry.
		down, err := decad.Faces(decad.Facing(zAxis.Scale(-1))).SelectFaces(body)
		require.NoError(t, err)
		require.Equal(t, start[0].Origins(), down[0].Origins())
	})

	t.Run("MatchesNothingWhereTheStepMintsNoCap", func(t *testing.T) {
		// A full revolution mints no caps, so the well-formed ref matches nothing.
		rev := annularRevolveBody(t)
		_, err := decad.Faces(decad.FaceCreatedBy(decad.CapStart(rev))).SelectFaces(rev)
		require.ErrorIs(t, err, decad.ErrNoMatch)
	})
}
