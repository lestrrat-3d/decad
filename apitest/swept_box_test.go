package apitest_test

import (
	"context"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// sweptBoxSlack absorbs the float evaluation of a sampled pose and its
// Apply: on these coordinates (|x| < 200 mm) that error is below 1e-12 mm.
// A translation's corners sit exactly on its hull, so containment is asserted
// to within the slack; every escape asserted below exceeds 1 mm.
const sweptBoxSlack = 1e-9

// sweptBoxCorners are the exact corners of the 10 x 4 x 2 source box the
// fixtures below sweep.
func sweptBoxCorners() []r3.Vec {
	var corners []r3.Vec
	for _, x := range []float64{0, 10} {
		for _, y := range []float64{0, 4} {
			for _, z := range []float64{0, 2} {
				corners = append(corners, r3.Vec{X: x, Y: y, Z: z})
			}
		}
	}
	return corners
}

func sweptBoxTranslation(t *testing.T, v r3.Vec) r3.Transform {
	t.Helper()
	pose, err := r3.Translation(v)
	require.NoError(t, err)
	return pose
}

// sweptBoxDrift turns about the z line through (100, 0) at omega rad/s while
// its center moves at velocity mm/s, from a start pose that carries the
// source box to x = 100.
func sweptBoxDrift(t *testing.T, velocity r3.Vec, omega, seconds float64) decad.RigidDriftSegment {
	t.Helper()
	return decad.RigidDriftSegment{
		From:   sweptBoxTranslation(t, r3.Vec{X: 100}),
		Center: r3.Vec{X: 100},
		LinearVelocity: decad.QuantityVec{X: units.MillimetersPerSecond(velocity.X),
			Y: units.MillimetersPerSecond(velocity.Y), Z: units.MillimetersPerSecond(velocity.Z)},
		AngularVelocity: decad.QuantityVec{X: units.RadiansPerSecond(0),
			Y: units.RadiansPerSecond(0), Z: units.RadiansPerSecond(omega)},
		Duration: units.Seconds(seconds),
	}
}

// sweptBoxDriftPose evaluates a drift as contact-sweep §2 defines it: turn
// about the line through Center, then translate by the elapsed velocity.
func sweptBoxDriftPose(t *testing.T, drift decad.RigidDriftSegment, fraction float64) r3.Transform {
	t.Helper()
	elapsed := fraction * drift.Duration.Base()
	turn, err := r3.RotationAround(drift.Center, r3.Vec{Z: 1},
		units.Radians(drift.AngularVelocity.Z.Base()*elapsed))
	require.NoError(t, err)
	pose, err := drift.From.Then(turn)
	require.NoError(t, err)
	pose, err = pose.Then(sweptBoxTranslation(t, r3.Vec{
		X: drift.LinearVelocity.X.Base() * elapsed,
		Y: drift.LinearVelocity.Y.Base() * elapsed,
		Z: drift.LinearVelocity.Z.Base() * elapsed}))
	require.NoError(t, err)
	return pose
}

// sweptBoxScrewSegment screws the source box about the z line through
// (100, 0) by angle radians while sliding slide mm along z.
func sweptBoxScrewSegment(t *testing.T, angle, slide float64) decad.PoseSegment {
	t.Helper()
	from := sweptBoxTranslation(t, r3.Vec{X: 100})
	turn, err := r3.RotationAround(r3.Vec{X: 100}, r3.Vec{Z: 1}, units.Radians(angle))
	require.NoError(t, err)
	to, err := from.Then(turn)
	require.NoError(t, err)
	to, err = to.Then(sweptBoxTranslation(t, r3.Vec{Z: slide}))
	require.NoError(t, err)
	return decad.PoseSegment{From: from, To: to, Duration: units.Seconds(1)}
}

// sweptBoxScrewPose evaluates a PoseSegment as contact-sweep §2 defines it:
// From composed with the read screw of From⁻¹·To at the fraction.
func sweptBoxScrewPose(t *testing.T, segment decad.PoseSegment, fraction float64) r3.Transform {
	t.Helper()
	inverse, err := segment.From.Inverse()
	require.NoError(t, err)
	relative, err := inverse.Then(segment.To)
	require.NoError(t, err)
	screw, err := relative.Screw()
	require.NoError(t, err)
	step, err := screw.At(fraction)
	require.NoError(t, err)
	pose, err := segment.From.Then(step)
	require.NoError(t, err)
	return pose
}

// sweptBoxLeastMargin is the smallest signed distance from any corner at
// pose to the box's faces: positive inside, negative once a corner escapes.
func sweptBoxLeastMargin(box decad.Box, pose r3.Transform) float64 {
	least := math.Inf(1)
	for _, corner := range sweptBoxCorners() {
		p := pose.Apply(corner)
		least = min(least, p.X-box.Min.X, box.Max.X-p.X, p.Y-box.Min.Y, box.Max.Y-p.Y,
			p.Z-box.Min.Z, box.Max.Z-p.Z)
	}
	return least
}

// TestDocumentSweptBoxContainsThePath is multibody §13 PR 2's fixture: each
// path's swept box contains the source box's exact corners at fractions 0,
// 1/3 and 1, while the From-placed rest box, which carries no travel, lets a
// corner escape.
//
// Legs shown to fail (each deleted in swept_box.go, the test watched go red,
// then restored):
//   - the From mapping of the bounds corners (reading them at rest): every
//     fixture's corners at fraction 0 escape by about 100 mm;
//   - the translation hull's delta: the "translation" row escapes at fraction 1;
//   - the drift ρΩ term: the "rotating drift" row escapes at fraction 1/3;
//   - the drift V term: the "drifting" row escapes at fraction 1;
//   - the drift's multiplication by Duration: the "slow rotating drift" row
//     escapes at fraction 1;
//   - the screw ρ|θ| term: the "rotating screw" row escapes at fraction 1;
//   - the screw |slide| term: the "sliding screw" row escapes at fraction 1.
//
// Legs not shown to fail: the inflation by Bounds().Bound is zero for this
// exact box body, and the outward roundings of the square roots and of the
// travel are below one ulp; no float-evaluated corner can observe either.
func TestDocumentSweptBoxContainsThePath(t *testing.T) {
	doc := decad.New()
	body := boxBodyAtZ(t, doc, 0, 0, 10, 4, 0, 2)
	rest, err := doc.SweptBox(t.Context(), body, decad.PoseSegment{
		From: sweptBoxTranslation(t, r3.Vec{X: 100}), To: sweptBoxTranslation(t, r3.Vec{X: 100}),
		Duration: units.Seconds(1)})
	require.NoError(t, err)
	// A stationary path has zero travel: its box is the From-placed bounds.
	require.Equal(t, decad.Box{Min: r3.Vec{X: 100}, Max: r3.Vec{X: 110, Y: 4, Z: 2},
		Exactness: decad.Exact, Bound: units.Millimeters(0)}, rest.Box())

	type row struct {
		name   string
		path   decad.PairPath
		poseAt func(float64) r3.Transform
		escape float64 // the fraction at which a corner leaves the rest box
	}
	translation := decad.PoseSegment{From: sweptBoxTranslation(t, r3.Vec{X: 100}),
		To: sweptBoxTranslation(t, r3.Vec{X: 100, Z: -64}), Duration: units.Seconds(1)}
	driftRow := func(name string, drift decad.RigidDriftSegment, escape float64) row {
		return row{name: name, path: drift, escape: escape,
			poseAt: func(f float64) r3.Transform { return sweptBoxDriftPose(t, drift, f) }}
	}
	screwRow := func(name string, segment decad.PoseSegment, escape float64) row {
		return row{name: name, path: segment, escape: escape,
			poseAt: func(f float64) r3.Transform { return sweptBoxScrewPose(t, segment, f) }}
	}
	rows := []row{
		{name: "translation", path: translation, escape: 1, poseAt: func(f float64) r3.Transform {
			return sweptBoxTranslation(t, r3.Vec{X: 100, Z: -64 * f})
		}},
		driftRow("rotating drift", sweptBoxDrift(t, r3.Vec{}, 1, 1), 1./3),
		driftRow("slow rotating drift", sweptBoxDrift(t, r3.Vec{}, .5, 2), 1),
		driftRow("drifting", sweptBoxDrift(t, r3.Vec{X: 64}, 1./64, 1), 1),
		screwRow("rotating screw", sweptBoxScrewSegment(t, 1, 0), 1),
		screwRow("sliding screw", sweptBoxScrewSegment(t, 1./64, 64), 1),
	}
	for _, r := range rows {
		t.Run(r.name, func(t *testing.T) {
			swept, err := doc.SweptBox(t.Context(), body, r.path)
			require.NoError(t, err)
			box := swept.Box()
			for _, fraction := range []float64{0, 1. / 3, 1} {
				require.GreaterOrEqual(t, sweptBoxLeastMargin(box, r.poseAt(fraction)), -sweptBoxSlack,
					"fraction %v leaves %+v", fraction, box)
			}
			require.Less(t, sweptBoxLeastMargin(rest.Box(), r.poseAt(r.escape)), -1.,
				"the rest box must lose a corner at fraction %v", r.escape)
			require.True(t, swept.StrictlyDisjoint(sweptBoxFar(t, doc)))
		})
	}
	// A translation takes the exact hull of its start and end boxes, not the
	// travel expansion on every axis.
	hull, err := doc.SweptBox(t.Context(), body, translation)
	require.NoError(t, err)
	require.Equal(t, decad.Box{Min: r3.Vec{X: 100, Z: -64}, Max: r3.Vec{X: 110, Y: 4, Z: 2},
		Exactness: decad.Exact, Bound: units.Millimeters(0)}, hull.Box())
}

// sweptBoxFar is a stationary box far beyond every fixture's travel.
func sweptBoxFar(t *testing.T, doc *decad.Document) decad.SweptBox {
	t.Helper()
	far := boxBodyAtZ(t, doc, 1000, 0, 1010, 4, 0, 2)
	swept, err := doc.SweptBox(t.Context(), far, decad.PoseSegment{From: r3.Identity(), To: r3.Identity(),
		Duration: units.Seconds(1)})
	require.NoError(t, err)
	return swept
}

// TestSweptBoxStrictlyDisjoint compares exactly: boxes that meet on a face
// are not disjoint, a dyadic gap of 2⁻¹⁰ mm is, and a path that closes the
// gap is not. Legs shown to fail: comparing with <= instead of < makes the
// meeting boxes disjoint, and dropping the zero-value guard lets the zero
// box read as disjoint from a box that misses the origin.
func TestSweptBoxStrictlyDisjoint(t *testing.T) {
	doc := decad.New()
	left := boxBodyAtZ(t, doc, 0, 0, 10, 4, 0, 2)
	right := boxBodyAtZ(t, doc, 10, 0, 20, 4, 0, 2)
	still := decad.PoseSegment{From: r3.Identity(), To: r3.Identity(), Duration: units.Seconds(1)}
	gap := sweptBoxTranslation(t, r3.Vec{X: 1. / 1024})
	apart := decad.PoseSegment{From: gap, To: gap, Duration: units.Seconds(1)}
	closing := decad.PoseSegment{From: gap, To: sweptBoxTranslation(t, r3.Vec{X: -1}),
		Duration: units.Seconds(1)}
	boxOf := func(b *decad.Body, path decad.PairPath) decad.SweptBox {
		swept, err := doc.SweptBox(t.Context(), b, path)
		require.NoError(t, err)
		return swept
	}
	leftBox := boxOf(left, still)
	for _, c := range []struct {
		name     string
		right    decad.SweptBox
		disjoint bool
	}{
		{"meeting", boxOf(right, still), false},
		{"apart", boxOf(right, apart), true},
		{"closing", boxOf(right, closing), false},
	} {
		require.Equal(t, c.disjoint, leftBox.StrictlyDisjoint(c.right), c.name)
		require.Equal(t, c.disjoint, c.right.StrictlyDisjoint(leftBox), c.name+" reversed")
	}
	// The zero value proves nothing about any box, even one that misses
	// the origin it would otherwise read as.
	far := boxOf(right, apart)
	require.False(t, decad.SweptBox{}.StrictlyDisjoint(far))
	require.False(t, far.StrictlyDisjoint(decad.SweptBox{}))
	require.Equal(t, decad.Box{}, decad.SweptBox{}.Box())
}

// TestDocumentSweptBoxRefusals validates as SweepPair does and refuses a
// travel bound that does not fit a float.
func TestDocumentSweptBoxRefusals(t *testing.T) {
	doc := decad.New()
	body := boxBodyAtZ(t, doc, 0, 0, 10, 4, 0, 2)
	still := decad.PoseSegment{From: r3.Identity(), To: r3.Identity(), Duration: units.Seconds(1)}
	_, err := doc.SweptBox(t.Context(), nil, still)
	require.ErrorIs(t, err, decad.ErrDegenerate)
	_, err = doc.SweptBox(t.Context(), body, nil)
	require.ErrorIs(t, err, decad.ErrDegenerate)
	instant := still
	instant.Duration = units.Seconds(0)
	_, err = doc.SweptBox(t.Context(), body, instant)
	require.ErrorIs(t, err, decad.ErrDegenerate)
	runaway := sweptBoxDrift(t, r3.Vec{X: 1e300}, 1, 1e300)
	_, err = doc.SweptBox(t.Context(), body, runaway)
	require.ErrorIs(t, err, decad.ErrUnsupported)
	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	_, err = doc.SweptBox(ctx, body, still)
	require.ErrorIs(t, err, context.Canceled)
}
