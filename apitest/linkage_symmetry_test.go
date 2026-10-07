package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// TestVerifyLinkageSymmetricBody pins docs/linkage-check-design.md §5.2's
// rule for a body its own joint does not move. A disc of radius 5,
// z ∈ [−5, 5], turns a quarter turn beside a wall x ∈ [12, 22],
// y ∈ [−20, 20], z ∈ [−10, 10], 7 mm away.
//
//   - About its own axis the disc does not move: the one interval [0, 1]
//     certifies from its two ends with the exact 7 mm, and the report reads
//     Sound in 2 poses.
//   - About Z through (0, 3, 0), 3 mm off its axis, the rule must not apply:
//     the disc's centre sits at (3·sin θ, 3 − 3·cos θ), so the gap is
//     7 − 3·sin θ and falls to 4. Every IntervalClear interval's bound sits
//     at or below it at its ends, and the reading encloses 4.
//   - About X through its centre the disc tumbles: its highest point reaches
//     y = 5·(cos θ + sin θ), so against a wall y ∈ [12, 22] the gap
//     12 − 5·(cos θ + sin θ) falls to 12 − 5·√2. The rule must not apply.
//   - A full revolve, a cone about X, turning about its own axis 5 mm below a
//     wall: the rule applies and [0, 1] certifies from its two ends.
//
// Legs seen to fail when deleted: the rule (the disc on its axis refines to
// 513 poses); the centre test (the 3 mm-off disc certifies 7 mm, above its
// gap of 4); and the direction test (the tumbling disc certifies its rest
// gap 7 mm, above 12 − 5·√2).
func TestVerifyLinkageSymmetricBody(t *testing.T) {
	t.Parallel()
	quarter := func(link *decad.Link) decad.Drive {
		return decad.Drive{{Link: link, From: units.Degrees(0), To: units.Degrees(90)}}
	}
	t.Run("a disc on its own axis does not move", func(t *testing.T) {
		t.Parallel()
		doc := decad.New()
		disc := discBodySymmetric(t, doc, 0, 5, 5)
		boxBodyAtZ(t, doc, 12, -20, 22, 20, -10, 20)
		l := decad.NewLinkage()
		spin, err := l.Ground().Revolute(r3.Vec{}, zAxis, []*decad.Body{disc})
		require.NoError(t, err)
		report := verifyLinkage(t, doc, l, quarter(spin))
		require.Equal(t, decad.Sound, report.Status)
		require.Len(t, report.Poses, 2)
		require.Len(t, report.Intervals, 1)
		require.Equal(t, decad.IntervalClear, report.Intervals[0].Outcome)
		require.Equal(t, 7.0, report.Intervals[0].Clearance.Value.Mag(), `no travel: the exact 7 mm at both ends`)
		requireReadingEncloses(t, report, 7)
	})
	t.Run("a disc off its own axis keeps its joint", func(t *testing.T) {
		t.Parallel()
		doc := decad.New()
		disc := discBodySymmetric(t, doc, 0, 5, 5)
		boxBodyAtZ(t, doc, 12, -20, 22, 20, -10, 20)
		l := decad.NewLinkage()
		spin, err := l.Ground().Revolute(r3.NewVec(0, 3, 0), zAxis, []*decad.Body{disc})
		require.NoError(t, err)
		report := verifyLinkage(t, doc, l, quarter(spin))
		gap := func(s float64) float64 { return 7 - 3*math.Sin(s*math.Pi/2) }
		requireIntervalsBelow(t, report, gap, nil)
		requireReadingEncloses(t, report, 4)
	})
	t.Run("a tumbling disc keeps its joint", func(t *testing.T) {
		t.Parallel()
		doc := decad.New()
		disc := discBodySymmetric(t, doc, 0, 5, 5)
		boxBodyAtZ(t, doc, -20, 12, 20, 22, -30, 60)
		l := decad.NewLinkage()
		tumble, err := l.Ground().Revolute(r3.Vec{}, r3.NewVec(1, 0, 0), []*decad.Body{disc})
		require.NoError(t, err)
		report := verifyLinkage(t, doc, l, quarter(tumble))
		gap := func(s float64) float64 {
			th := s * math.Pi / 2
			return 12 - 5*(math.Cos(th)+math.Sin(th))
		}
		requireIntervalsBelow(t, report, gap, nil)
		requireReadingEncloses(t, report, 12-5*math.Sqrt2)
	})
	t.Run("a full revolve on its own axis does not move", func(t *testing.T) {
		t.Parallel()
		doc := decad.New()
		w := sketch.NewWorld()
		s, err := w.CreateSketch(w.XY())
		require.NoError(t, err)
		o := s.CreatePoint(0, 0)
		s.Fix(o)
		apex := s.CreatePoint(10, 0)
		top := s.CreatePoint(0, 10)
		s.CreateLine(o, apex)
		s.CreateLine(apex, top)
		s.CreateLine(top, o)
		_, err = s.Solve(t.Context())
		require.NoError(t, err)
		cone, err := doc.Revolve(s, s.Profiles()[0], uAxis, decad.FullRevolution{})
		require.NoError(t, err)
		boxBodyAtZ(t, doc, -20, 15, 30, 25, -30, 60)
		l := decad.NewLinkage()
		spin, err := l.Ground().Revolute(r3.Vec{}, r3.NewVec(1, 0, 0), []*decad.Body{cone})
		require.NoError(t, err)
		report := verifyLinkage(t, doc, l, quarter(spin))
		require.Len(t, report.Poses, 2)
		require.Len(t, report.Intervals, 1)
		require.Equal(t, decad.IntervalClear, report.Intervals[0].Outcome)
		require.LessOrEqual(t, report.Intervals[0].Clearance.Value.Mag(), 5.0)
		require.Greater(t, report.Intervals[0].Clearance.Value.Mag(), 0.0)
	})
}
