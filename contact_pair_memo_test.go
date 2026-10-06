package decad_test

import (
	"math"
	"sync"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// contactPolls runs one ContactPair query and returns its report with the
// number of times it polled the context. A query its first body's memo
// serves polls once, at entry; the hexagon on the tray polls more than that.
func contactPolls(t *testing.T, doc *decad.Document, a, b *decad.Body, poseA, poseB r3.Transform,
	req decad.ContactRequest) (*decad.ContactReport, int32) {
	t.Helper()
	counting := newCancelAfterContext(t.Context(), math.MaxInt32)
	report, err := doc.ContactPair(counting, a, b, poseA, poseB, req)
	require.NoError(t, err)
	return report, counting.calls.Load()
}

// TestContactPairRepeatReturnsEqualPrivateReport repeats a hexagon-on-tray
// query: the repeat polls once and returns a report equal to the first, yet
// no value of it is shared, so a caller writing into one report changes no
// later one.
func TestContactPairRepeatReturnsEqualPrivateReport(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	tray := trayBody(t, doc)
	hex := hexPrismBody(t, doc)
	pose := hexPose(t, r3.Vec{X: -10})
	first, polls := contactPolls(t, doc, tray, hex, r3.Identity(), pose, contactRequest())
	require.Greater(t, polls, int32(1))
	require.Equal(t, decad.ContactTouching, first.Relation)
	require.NotNil(t, first.Manifold)
	require.NotEmpty(t, first.Manifold.Points)
	second, polls := contactPolls(t, doc, tray, hex, r3.Identity(), pose, contactRequest())
	require.Equal(t, int32(1), polls)
	require.Equal(t, first, second)
	require.NotSame(t, first, second)
	require.NotSame(t, first.Gap, second.Gap)
	require.NotSame(t, first.Manifold, second.Manifold)
	require.NotSame(t, &first.Manifold.Points[0], &second.Manifold.Points[0])

	want := *second
	wantGap := *second.Gap
	wantPoints := append([]decad.ContactPoint(nil), second.Manifold.Points...)
	second.Relation = decad.ContactOverlapping
	second.Gap.Value = units.Millimeters(1)
	second.Manifold.Points[0].OnA.Value = r3.Vec{X: 1}
	second.Manifold.Points = second.Manifold.Points[:1]
	third, polls := contactPolls(t, doc, tray, hex, r3.Identity(), pose, contactRequest())
	require.Equal(t, int32(1), polls)
	require.Equal(t, want.Relation, third.Relation)
	require.Equal(t, wantGap, *third.Gap)
	require.Equal(t, wantPoints, third.Manifold.Points)
	require.Equal(t, first, third)
}

// TestContactPairMemoKeysEveryInput changes one input at a time after a
// completed query: a pose one ulp or one zero sign away, each request field,
// and the body order. Each is a new query that does the whole proof, so the
// memo serves only the exact inputs it stored.
func TestContactPairMemoKeysEveryInput(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	tray := trayBody(t, doc)
	hex := hexPrismBody(t, doc)
	pose := hexPose(t, r3.Vec{X: -10})
	req := contactRequest()
	_, full := contactPolls(t, doc, tray, hex, r3.Identity(), pose, req)
	require.Greater(t, full, int32(1))
	_, polls := contactPolls(t, doc, tray, hex, r3.Identity(), pose, req)
	require.Equal(t, int32(1), polls)

	ulp := hexPose(t, r3.Vec{X: math.Nextafter(-10, 0)})
	negativeZero := hexPose(t, r3.Vec{X: -10, Y: math.Copysign(0, -1)})
	require.True(t, negativeZero == pose, "premise: == cannot tell the zero signs apart")
	firstSign, err := r3.Translation(r3.Vec{Z: math.Copysign(0, -1)})
	require.NoError(t, err)
	widerPoint, widerNormal, band, chord := req, req, req, req
	widerPoint.PointResolution = units.Millimeters(2e-6)
	widerNormal.NormalResolution = units.Degrees(2)
	band.SupportBand = units.Millimeters(0.25)
	chord.HeldChord = units.Millimeters(0.05)
	for name, q := range map[string]struct {
		a, b         *decad.Body
		poseA, poseB r3.Transform
		req          decad.ContactRequest
	}{
		"pose ulp":           {tray, hex, r3.Identity(), ulp, req},
		"pose zero sign":     {tray, hex, r3.Identity(), negativeZero, req},
		"first pose sign":    {tray, hex, firstSign, pose, req},
		"point resolution":   {tray, hex, r3.Identity(), pose, widerPoint},
		"normal resolution":  {tray, hex, r3.Identity(), pose, widerNormal},
		"support band":       {tray, hex, r3.Identity(), pose, band},
		"held chord":         {tray, hex, r3.Identity(), pose, chord},
		"reversed body pair": {hex, tray, pose, r3.Identity(), req},
	} {
		report, polls := contactPolls(t, doc, q.a, q.b, q.poseA, q.poseB, q.req)
		require.Greater(t, polls, int32(1), name)
		require.Equal(t, q.req, report.Request, name)
		require.True(t, q.poseA == report.PoseA && q.poseB == report.PoseB, name)
		require.Same(t, q.a, report.A, name)
	}
}

// TestContactPairConcurrentQueriesAgree runs the same queries from many
// goroutines on one document, so misses race to fill the memo and hits read
// it while it fills, and requires every report equal to the one a
// sequential first read in an identical document returns, bodies and faces
// aside. Run under -race it is also the memo's data-race check.
func TestContactPairConcurrentQueriesAgree(t *testing.T) {
	t.Parallel()
	poses := make([]r3.Transform, 0, 6)
	for _, x := range []float64{-10, -12, -14} {
		poses = append(poses, hexPose(t, r3.Vec{X: x}), hexPose(t, r3.Vec{X: x, Z: 0.5}))
	}
	strip := func(r *decad.ContactReport) decad.ContactReport {
		out := *r
		out.A, out.B = nil, nil
		if r.Manifold != nil {
			points := append([]decad.ContactPoint(nil), r.Manifold.Points...)
			for i := range points {
				points[i].FaceA, points[i].FaceB = nil, nil
				points[i].FeatureA, points[i].FeatureB = decad.ContactFeature{}, decad.ContactFeature{}
			}
			out.Manifold = &decad.ContactManifold{Points: points}
		}
		return out
	}
	reference := decad.New()
	refTray, refHex := trayBody(t, reference), hexPrismBody(t, reference)
	want := make([]decad.ContactReport, len(poses))
	for i, pose := range poses {
		report, err := reference.ContactPair(t.Context(), refTray, refHex, r3.Identity(), pose, contactRequest())
		require.NoError(t, err)
		want[i] = strip(report)
	}
	require.Equal(t, decad.ContactTouching, want[0].Relation)
	require.Equal(t, decad.ContactSeparated, want[1].Relation)

	doc := decad.New()
	tray, hex := trayBody(t, doc), hexPrismBody(t, doc)
	const workers = 8
	got := make([][]*decad.ContactReport, workers)
	errs := make([]error, workers)
	var wg sync.WaitGroup
	for w := range workers {
		wg.Go(func() {
			for round := range 3 {
				for i := range poses {
					pose := poses[(i+w+round)%len(poses)]
					report, err := doc.ContactPair(t.Context(), tray, hex, r3.Identity(), pose, contactRequest())
					if err != nil {
						errs[w] = err
						return
					}
					got[w] = append(got[w], report)
				}
			}
		})
	}
	wg.Wait()
	for w := range workers {
		require.NoError(t, errs[w])
		require.Len(t, got[w], 3*len(poses))
		for n, report := range got[w] {
			i := (n%len(poses) + w + n/len(poses)) % len(poses)
			require.Equal(t, want[i], strip(report), "worker %d query %d", w, n)
			require.Same(t, tray, report.A)
		}
	}
}
