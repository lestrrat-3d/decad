package smoothloft

import (
	"errors"
	"math"
	"slices"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func twistGatherProfile(reversed bool) momentinput.Profile {
	points := []sectionrecord.Point2{{U: 4, V: -2}, {U: 5}, {U: 4, V: 2}}
	fit := sectionrecord.FitSplineSeg{Fit: points, TStart: 0, TEnd: 1}
	start, end := points[0], points[2]
	if reversed {
		fit.TStart, fit.TEnd = 1, 0
		start, end = end, start
	}
	tip := sectionrecord.Point2{U: -1}
	return momentinput.Profile{Outer: sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{
		fit,
		sectionrecord.LineSeg{Start: end, End: tip, TStart: 0, TEnd: 1},
		sectionrecord.LineSeg{Start: tip, End: start, TStart: 0, TEnd: 1},
	}}}
}

func TestTwistedGatherMatchesResolvedWalkSpans(t *testing.T) {
	t.Parallel()
	for _, reversed := range []bool{false, true} {
		for rotation := range 3 {
			profile := twistGatherProfile(reversed)
			original := profile.Outer.Segments
			rotated := make([]sectionrecord.CurveSegment, len(original))
			for i := range rotated {
				rotated[i] = original[(i+rotation)%len(original)]
			}
			profile.Outer.Segments = rotated
			for firstFit := range 3 {
				oldWork, work := freeform.NewFreeformWork(), freeform.NewFreeformWork()
				want, err := fullWalkTwistGather(profile, oldWork, firstFit, 3)
				require.NoError(t, err)
				got, err := twistGatherOne(profile, work, firstFit, 3)
				require.NoError(t, err)
				require.Len(t, got, len(want))
				for edge := range want {
					require.Len(t, got[edge], len(want[edge]))
					for span := range want[edge] {
						for control := range want[edge][span] {
							p, q := got[edge][span][control], want[edge][span][control]
							require.Zero(t, p.U.Cmp(q.U), "reverse %t rotation %d fit %d edge %d span %d control %d U", reversed, rotation, firstFit, edge, span, control)
							require.Zero(t, p.V.Cmp(q.V), "reverse %t rotation %d fit %d edge %d span %d control %d V", reversed, rotation, firstFit, edge, span, control)
						}
					}
				}
				require.Less(t, work.Spent, oldWork.Spent, "gathering does not spend work on an unused curve length")
			}
		}
	}
}

func TestTwistedGatherKeepsSourceRefusals(t *testing.T) {
	t.Parallel()
	// momentinput validates line finiteness before gathering. These cases
	// exercise the converter and exact-join refusals this path also owns.
	for _, kind := range []string{"fit_not_finite", "all_collapsed", "zero_line", "disjoint_join", "collapsed_terminal", "trimmed_fit"} {
		t.Run(kind, func(t *testing.T) {
			profile := twistGatherProfile(false)
			fit := profile.Outer.Segments[0].(sectionrecord.FitSplineSeg)
			line := profile.Outer.Segments[1].(sectionrecord.LineSeg)
			switch kind {
			case "fit_not_finite":
				fit.Fit[1].V = math.Inf(1)
			case "all_collapsed":
				for i := range fit.Fit {
					fit.Fit[i] = fit.Fit[0]
				}
				line.Start = fit.Fit[0]
			case "zero_line":
				line.End = line.Start
			case "disjoint_join":
				line.Start.U += 1
			case "collapsed_terminal":
				last := fit.Fit[len(fit.Fit)-1]
				last.U += 3e-13
				fit.Fit = append(fit.Fit, last)
				line.Start = last
			case "trimmed_fit":
				fit.TStart = 0.25
			}
			profile.Outer.Segments[0], profile.Outer.Segments[1] = fit, line
			_, wantErr := fullWalkTwistGather(profile, freeform.NewFreeformWork(), 0, 3)
			require.Error(t, wantErr)
			got, gotErr := twistGatherOne(profile, freeform.NewFreeformWork(), 0, 3)
			require.Error(t, gotErr)
			require.Empty(t, got)
			for _, category := range []error{decaderr.ErrUnsupported, decaderr.ErrDegenerate, decaderr.ErrNotFinite} {
				require.Equal(t, errors.Is(wantErr, category), errors.Is(gotErr, category), "error category %s", category)
			}
		})
	}
}

func TestTwistedGatherChargesTheRecordCounter(t *testing.T) {
	t.Parallel()
	work := freeform.NewFreeformWork()
	work.Limit = 1
	got, err := twistGatherOne(twistGatherProfile(false), work, 0, 3)
	require.ErrorIs(t, err, decaderr.ErrUnsupported)
	require.Empty(t, got)
	require.Equal(t, uint64(1), work.Spent)
}

func TestBuildTwistedFitsRequiresRecordCounters(t *testing.T) {
	t.Parallel()
	profiles := make([]momentinput.Profile, 3)
	planes := make([]sectionrecord.PlaneRecord, 3)
	work := make([]*freeform.FreeformWork, 3)
	for i := range profiles {
		profiles[i] = twistGatherProfile(false)
		planes[i] = sectionrecord.PlaneRecord{Origin: r3.Vec{Z: float64(i)}, U: r3.Vec{X: 1}, V: r3.Vec{Y: 1}}
		work[i] = freeform.NewFreeformWork()
	}
	work[1] = nil
	_, err := BuildTwistedFits(t.Context(), profiles, planes, work)
	require.ErrorIs(t, err, decaderr.ErrUnsupported)
	require.ErrorContains(t, err, "record's free-form work counter")
	require.Zero(t, work[0].Spent, "no record is converted before the counter check")
}

// fullWalkTwistGather reads the real bounded profile walks, including their
// length and tangent calculations, before extracting the exact control spans.
func fullWalkTwistGather(profile momentinput.Profile, work *freeform.FreeformWork,
	fitAt0, count int) ([][]twistSpan, error) {
	fit, err := twistFitIndex(profile)
	if err != nil {
		return nil, err
	}
	m := len(profile.Outer.Segments)
	if m != count {
		return nil, twistUnsupported("twisted loft sections have different edge counts")
	}
	walks, err := momentinput.ResolveProfileWalks(profile, work)
	if err != nil {
		return nil, err
	}
	edges := make([][]twistSpan, m)
	for a := range m {
		orig := ((fit+a-fitAt0)%m + m) % m
		switch seg := profile.Outer.Segments[orig].(type) {
		case sectionrecord.LineSeg:
			start, end := twistRat(seg.Start), twistRat(seg.End)
			if seg.TStart == 1 {
				start, end = end, start
			}
			edges[a] = []twistSpan{twistLineSpan(start, end)}
		case sectionrecord.FitSplineSeg:
			spans := fullWalkSpans(walks.At(0, orig))
			if len(spans) == 0 || len(spans) > 32 {
				return nil, twistUnsupported("a twisted loft fit spline needs 1 to 32 curve spans")
			}
			edges[a] = make([]twistSpan, len(spans))
			for s, span := range spans {
				if len(span) != 4 {
					return nil, twistUnsupported("a twisted loft fit spline needs cubic Bézier spans")
				}
				copy(edges[a][s][:], span)
			}
		}
	}
	for a := range edges {
		last := edges[a][len(edges[a])-1][3]
		if !twistSame(last, edges[(a+1)%m][0][0]) {
			return nil, twistUnsupported("a twisted loft section has unequal recorded joins")
		}
		for s := 1; s < len(edges[a]); s++ {
			if !twistSame(edges[a][s-1][3], edges[a][s][0]) {
				return nil, twistUnsupported("a twisted loft fit spline has disjoint spans")
			}
		}
	}
	return edges, nil
}

func fullWalkSpans(w survey2d.SegmentWalk) []freeform.BezierSpan {
	if !w.Reversed {
		return w.Spans
	}
	out := make([]freeform.BezierSpan, len(w.Spans))
	for i, span := range w.Spans {
		reversed := slices.Clone(span)
		slices.Reverse(reversed)
		out[len(w.Spans)-1-i] = reversed
	}
	return out
}
