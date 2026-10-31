package verifyrun_test

import (
	"context"
	"fmt"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/internal/orderedwork"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

const bodySurveySheet = "sheet"

func bodySurveyDocument(tb testing.TB, shape string, count int) (*decad.Document, []*decad.Body, float64) {
	tb.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(tb, err)
	rect := s.CreateRectangle(-20, -20, 20, 20)
	s.Fix(rect.A)
	wantVolume := 40.0 * 40 * 200
	if shape == "plate" {
		o := s.CreatePoint(0, 0)
		p, q := s.CreatePoint(1.86, 1.02), s.CreatePoint(-1.86, -1.02)
		s.CreateArc(o, p, q)
		s.CreateLine(q, p)
		wantVolume -= math.Pi * (1.86*1.86 + 1.02*1.02) / 2 * 200
	}
	_, err = s.Solve(tb.Context())
	require.NoError(tb, err)
	var profile *sketch.Profile
	for _, p := range s.Profiles() {
		if (shape == "plate") == (len(p.Holes) == 1) {
			profile = p
			break
		}
	}
	require.NotNil(tb, profile)
	doc := decad.New()
	var opts []decad.ExtrudeOption
	if shape == bodySurveySheet {
		opts = append(opts, decad.WithSurfaceResult())
	}
	body, err := decad.Extrude(tb.Context(), doc, s, profile,
		decad.Distance{D: units.Millimeters(200), Dir: decad.Along}, opts...)
	require.NoError(tb, err)
	bodies := []*decad.Body{body}
	for i := 1; i < count; i++ {
		placement, err := r3.Translation(r3.NewVec(100*float64(i), 0, 0))
		require.NoError(tb, err)
		placed, err := decad.Copy(tb.Context(), body, decad.WithCopyPlacement(placement))
		require.NoError(tb, err)
		bodies = append(bodies, placed)
	}
	return doc, bodies, wantVolume
}

func TestParallelBodyWallSurveysPreserveReadingsAndOrder(t *testing.T) {
	for _, shape := range []string{"box", "plate", bodySurveySheet} {
		t.Run(shape, func(t *testing.T) {
			doc, bodies, wantVolume := bodySurveyDocument(t, shape, 4)
			opts := []decad.VerifyOption{
				decad.WithMinWallThickness(units.Millimeters(100)),
				decad.WithConcaveRadius(), decad.WithPullDirection(r3.NewVec(0, 0, -1)),
			}
			serial, err := doc.Verify(orderedwork.WithWorkers(t.Context(), 1), opts...)
			require.NoError(t, err)
			for _, workers := range []int{2, 4, 12} {
				got, err := doc.Verify(orderedwork.WithWorkers(t.Context(), workers), opts...)
				require.NoError(t, err)
				require.Equal(t, serial.Status, got.Status)
				require.Len(t, got.Bodies, len(bodies))
				require.Empty(t, got.Interferences)
				require.Len(t, got.Diagnostics, len(serial.Diagnostics))
				for i, d := range got.Diagnostics {
					want := serial.Diagnostics[i]
					require.Equal(t, want.Code, d.Code)
					require.Equal(t, want.Status, d.Status)
					require.True(t, want.Body == d.Body, "diagnostic body differs at %d", i)
				}
				for i, br := range got.Bodies {
					want := serial.Bodies[i]
					require.True(t, br.Body == bodies[i], "body order differs at %d", i)
					require.Equal(t, want.Status, br.Status)
					require.Equal(t, decad.ValidityValid, br.Validity.Outcome)
					require.Equal(t, want.Wall.Outcome, br.Wall.Outcome)
					if shape == bodySurveySheet {
						require.Nil(t, br.Region)
						require.Equal(t, decad.ScalarUnavailable, br.Wall.Outcome)
					} else {
						require.NotNil(t, br.Region)
						require.InDelta(t, wantVolume, br.Region.Volume.Value.Base(), 1e-6)
						require.NotNil(t, br.Wall.Minimum)
						if shape == "box" {
							require.InDelta(t, 40, br.Wall.Minimum.Value.Base(), 1e-8)
						}
						require.Equal(t, want.Wall.Minimum.Value.Base(), br.Wall.Minimum.Value.Base())
						require.Equal(t, want.Wall.Minimum.Bound.Base(), br.Wall.Minimum.Bound.Base())
					}
					require.Equal(t, want.Undercut.Coverage, br.Undercut.Coverage)
					require.Len(t, br.Undercut.Faces, len(want.Undercut.Faces))
					for j, face := range br.Undercut.Faces {
						require.True(t, face == want.Undercut.Faces[j], "undercut face differs at %d,%d", i, j)
					}
				}
			}
		})
	}
}

func TestParallelBodyWallSurveysCancellation(t *testing.T) {
	doc, bodies, _ := bodySurveyDocument(t, "box", 2)
	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	report, err := doc.Verify(ctx, decad.WithMinWallThickness(units.Millimeters(1)))
	require.ErrorIs(t, err, context.Canceled)
	require.Nil(t, report)
	live := doc.Bodies()
	require.Len(t, live, len(bodies))
	for i, b := range live {
		require.True(t, b == bodies[i], "body membership differs at %d", i)
	}
	empty, err := decad.New().Verify(ctx)
	require.NoError(t, err)
	require.True(t, empty.Passed())
}

func BenchmarkVerifyBodyWallSurveys(b *testing.B) {
	for _, shape := range []string{"box", "plate", bodySurveySheet, "default"} {
		for _, count := range []int{2, 4} {
			for _, workers := range []int{1, 4} {
				b.Run(fmt.Sprintf("%s/n%d/w%d", shape, count, workers), func(b *testing.B) {
					doc, bodies, wantVolume := bodySurveyDocument(b, shape, count)
					ctx := orderedwork.WithWorkers(b.Context(), workers)
					var opts []decad.VerifyOption
					if shape != "default" {
						opts = append(opts, decad.WithMinWallThickness(units.Millimeters(1)))
					}
					report, err := doc.Verify(ctx, opts...)
					require.NoError(b, err)
					require.Len(b, report.Bodies, len(bodies))
					require.Empty(b, report.Interferences)
					for _, br := range report.Bodies {
						if shape == bodySurveySheet {
							require.Equal(b, decad.ScalarUnavailable, br.Wall.Outcome)
							continue
						}
						require.NotNil(b, br.Region)
						require.InDelta(b, wantVolume, br.Region.Volume.Value.Base(), 1e-6)
						if shape != "default" {
							require.NotNil(b, br.Wall.Minimum)
						}
					}
					b.ReportAllocs()
					b.ResetTimer()
					for range b.N {
						if _, err := doc.Verify(ctx, opts...); err != nil {
							b.Fatal(err)
						}
					}
				})
			}
		}
	}
}
