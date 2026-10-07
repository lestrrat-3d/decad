package decad

import (
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// constantPin extrudes the circle of radius 5 about (cx, 0) over z ∈ [−1, 9].
func constantPin(t *testing.T, doc *Document, cx float64) *Body {
	t.Helper()
	w := sketch.NewWorld()
	plane, err := w.CreateOffsetPlane(w.XY(), -1)
	require.NoError(t, err)
	s, err := w.CreateSketch(plane)
	require.NoError(t, err)
	c := s.CreatePoint(cx, 0)
	s.Fix(c)
	s.CreateCircle(c, 5)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	body, err := doc.Extrude(s, s.Profiles()[0], Distance{D: units.Millimeters(10), Dir: Along})
	require.NoError(t, err)
	return body
}

// constantPairOf reports whether the run marked the pair of a and b constant.
func constantPairOf(t *testing.T, run *motionRun, a, b *Body) bool {
	t.Helper()
	for i, mv := range run.movers {
		for k := range run.pairs[i] {
			partner := run.partner(i, k)
			if (mv.body == a && partner == b) || (mv.body == b && partner == a) {
				return run.pairs[i][k].constant
			}
		}
	}
	t.Fatalf("no pair of the two bodies")
	return false
}

// TestLinkageConstantPairRule pins docs/linkage-check-design.md §5.9's rule:
// a pair is constant only when its relative path holds exactly one joint
// that does not hold 0, a revolute, and one of its bodies is symmetric about
// that joint's axis line. A bar turns on a shoulder about Z through the
// origin; a pin of radius 5 about (48, 0) hangs under it on joints about Z.
// Seen red: admitting a relative path with more than one moving joint marks
// the pin under an offset elbow and a wrist constant, where the elbow carries
// the pin off its zero-pose place, and the bar against the wall. A slide
// along the pin's own axis is refused by symmetricAboutJoint's own revolute
// test as well as the rule's.
func TestLinkageConstantPairRule(t *testing.T) {
	t.Parallel()
	z := r3.NewVec(0, 0, 1)
	quarter := func(link *Link, to float64) JointSweep {
		return JointSweep{Link: link, From: units.Degrees(0), To: units.Degrees(to)}
	}
	t.Run("a pin on its own joint", func(t *testing.T) {
		t.Parallel()
		doc := New()
		bar := internalBoxBodyAtZ(t, doc, 30, -20, 60, -10, 0, 8)
		pin := constantPin(t, doc, 48)
		wall := internalBoxBodyAtZ(t, doc, -100, 38, 150, 58, -10, 50)
		l := NewLinkage()
		shoulder, err := l.Ground().Revolute(r3.Vec{}, z, []*Body{bar})
		require.NoError(t, err)
		elbow, err := shoulder.Revolute(r3.NewVec(48, 0, 0), z, []*Body{pin})
		require.NoError(t, err)
		run := linkageRunOf(t, doc, l, Drive{quarter(shoulder, 90), quarter(elbow, -90)})
		require.True(t, constantPairOf(t, run, bar, pin), `one joint, the pin symmetric about it`)
		require.False(t, constantPairOf(t, run, bar, wall), `the bar is symmetric about no joint`)
		require.False(t, constantPairOf(t, run, pin, wall), `two joints carry the pin past the wall`)
	})
	t.Run("a held joint is passed over", func(t *testing.T) {
		t.Parallel()
		doc := New()
		bar := internalBoxBodyAtZ(t, doc, 30, -20, 60, -10, 0, 8)
		pin := constantPin(t, doc, 48)
		block := internalBoxBodyAtZ(t, doc, 100, 100, 104, 104, 20, 2)
		l := NewLinkage()
		shoulder, err := l.Ground().Revolute(r3.Vec{}, z, []*Body{bar})
		require.NoError(t, err)
		held, err := shoulder.Revolute(r3.NewVec(10, 0, 0), z, []*Body{block})
		require.NoError(t, err)
		wrist, err := held.Revolute(r3.NewVec(48, 0, 0), z, []*Body{pin})
		require.NoError(t, err)
		run := linkageRunOf(t, doc, l, Drive{quarter(shoulder, 90), quarter(wrist, -90)})
		require.True(t, constantPairOf(t, run, bar, pin), `the held elbow moves nothing`)
	})
	t.Run("two moving joints", func(t *testing.T) {
		t.Parallel()
		doc := New()
		bar := internalBoxBodyAtZ(t, doc, 30, -20, 60, -10, 0, 8)
		pin := constantPin(t, doc, 48)
		block := internalBoxBodyAtZ(t, doc, 100, 100, 104, 104, 20, 2)
		l := NewLinkage()
		shoulder, err := l.Ground().Revolute(r3.Vec{}, z, []*Body{bar})
		require.NoError(t, err)
		elbow, err := shoulder.Revolute(r3.NewVec(10, 0, 0), z, []*Body{block})
		require.NoError(t, err)
		wrist, err := elbow.Revolute(r3.NewVec(48, 0, 0), z, []*Body{pin})
		require.NoError(t, err)
		run := linkageRunOf(t, doc, l, Drive{quarter(shoulder, 90), quarter(elbow, 30), quarter(wrist, -90)})
		require.False(t, constantPairOf(t, run, bar, pin), `the elbow carries the pin off its place`)
	})
	t.Run("a slide", func(t *testing.T) {
		t.Parallel()
		doc := New()
		bar := internalBoxBodyAtZ(t, doc, 30, -20, 60, -10, 0, 8)
		pin := constantPin(t, doc, 48)
		l := NewLinkage()
		shoulder, err := l.Ground().Revolute(r3.Vec{}, z, []*Body{bar})
		require.NoError(t, err)
		slide, err := shoulder.Prismatic(z, []*Body{pin})
		require.NoError(t, err)
		run := linkageRunOf(t, doc, l, Drive{quarter(shoulder, 90),
			{Link: slide, From: units.Millimeters(0), To: units.Millimeters(5)}})
		require.False(t, constantPairOf(t, run, bar, pin), `a slide along the pin's axis is no revolute`)
	})
	t.Run("a static post", func(t *testing.T) {
		t.Parallel()
		doc := New()
		bar := internalBoxBodyAtZ(t, doc, 30, -20, 60, -10, 0, 8)
		post := constantPin(t, doc, 0)
		l := NewLinkage()
		shoulder, err := l.Ground().Revolute(r3.Vec{}, z, []*Body{bar})
		require.NoError(t, err)
		run := linkageRunOf(t, doc, l, Drive{quarter(shoulder, 90)})
		require.True(t, constantPairOf(t, run, bar, post), `the post is symmetric about the shoulder`)
	})
}
