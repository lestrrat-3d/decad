package apitest_test

import (
	"context"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// linkageBlock extrudes the rectangle (x0, y0)-(x1, y1) over z ∈ [z0, z0+h].
func linkageBlock(tb testing.TB, doc *decad.Document, x0, y0, x1, y1, z0, h float64) *decad.Body {
	tb.Helper()
	w := sketch.NewWorld()
	plane, err := w.CreateOffsetPlane(w.XY(), z0)
	require.NoError(tb, err)
	s, err := w.CreateSketch(plane)
	require.NoError(tb, err)
	rect := s.CreateRectangle(x0, y0, x1, y1)
	s.Fix(rect.A)
	_, err = s.Solve(context.Background())
	require.NoError(tb, err)
	body, err := doc.Extrude(s, s.Profiles()[0], decad.Distance{D: units.Millimeters(h), Dir: decad.Along})
	require.NoError(tb, err)
	return body
}

// threeJointArm is docs/linkage-check-design.md §10's benchmark: three
// stacked 50 mm links, x ∈ [0, 50], [50, 100], [100, 150], y ∈ [−10, 10],
// on z ∈ [0, 10], [10, 20], [20, 30], each turning 0° → 90° about Z through
// its own root, the two elbow contacts declared. Two posts, x ∈ [160, 180]
// 10 mm past the wrist's tip and x ∈ [−120, −100], y ∈ [−80, −60], are the
// fixtures. Nine pairs per pose: six against the posts, the shoulder-wrist
// pair 10 mm apart in z, and the two declared elbows.
func threeJointArm(tb testing.TB) (*decad.Document, *decad.Linkage, decad.Drive) {
	tb.Helper()
	doc := decad.New()
	bodies := []*decad.Body{
		linkageBlock(tb, doc, 0, -10, 50, 10, 0, 10),
		linkageBlock(tb, doc, 50, -10, 100, 10, 10, 10),
		linkageBlock(tb, doc, 100, -10, 150, 10, 20, 10),
	}
	linkageBlock(tb, doc, 160, -10, 180, 10, 0, 30)
	linkageBlock(tb, doc, -120, -80, -100, -60, 0, 30)
	l := decad.NewLinkage()
	parent := l.Ground()
	var drive decad.Drive
	for k, b := range bodies {
		link, err := parent.Revolute(r3.NewVec(50*float64(k), 0, 0), zAxis, []*decad.Body{b})
		require.NoError(tb, err)
		drive = append(drive, decad.JointSweep{Link: link, From: units.Degrees(0), To: units.Degrees(90)})
		parent = link
	}
	require.NoError(tb, l.DeclareJointContact(bodies[0], bodies[1]))
	require.NoError(tb, l.DeclareJointContact(bodies[1], bodies[2]))
	return doc, l, drive
}

// BenchmarkVerifyLinkageThreeJointArm measures §10's three-joint arm, once at
// a resolution that settles the verdict and once at the default, where the
// projection bound (§5.8) closes the whole-drive reading at the verdict floor.
// It reports the poses each evaluates: 10 and 22.
func BenchmarkVerifyLinkageThreeJointArm(b *testing.B) {
	for _, tc := range []struct {
		name string
		opts []decad.MotionOption
	}{
		{"verdict at 1/64", []decad.MotionOption{decad.WithResolution(units.Scalar(1.0 / 64))}},
		{"default", nil},
	} {
		b.Run(tc.name, func(b *testing.B) {
			doc, l, drive := threeJointArm(b)
			var poses int
			for b.Loop() {
				report, err := doc.VerifyLinkage(b.Context(), l, drive, tc.opts...)
				require.NoError(b, err)
				require.Empty(b, report.Collisions)
				for _, iv := range report.Intervals {
					require.Equal(b, decad.IntervalClear, iv.Outcome)
				}
				poses = len(report.Poses)
			}
			b.ReportMetric(float64(poses), "poses/op")
		})
	}
}
