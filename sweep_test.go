package decad_test

import (
	"context"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/decadtest"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

type foreignSweepOption struct {
	decad.SweepOption
}

func TestSweepLineBuildsVerifiesAndReevaluatesPlacement(t *testing.T) {
	t.Parallel()

	s, profile := plateSketch(t)
	path, err := decad.NewPath(
		r3.NewVec(1000, 1000, 0),
		decad.LineTo{End: r3.NewVec(1000, 1000, 10)},
	)
	require.NoError(t, err)

	doc := decad.New()
	body, err := doc.Sweep(s, profile, path, decad.WithSweepTwist(units.Degrees(0)))
	require.NoError(t, err)
	extrudeDoc := decad.New()
	extruded, err := extrudeDoc.Extrude(
		s,
		profile,
		decad.Distance{D: units.Millimeters(10), Dir: decad.Along},
	)
	require.NoError(t, err)
	requireSameBodyReadings(t, extruded, body)
	require.ElementsMatch(t, bodyVertexValues(extruded), bodyVertexValues(body))

	require.True(t, body.IsSolid())
	require.Len(t, body.Faces(), 6)
	require.Len(t, body.Edges(), 12)
	require.Len(t, body.Vertices(), 8)
	decadtest.HasSurfaceKinds(t, body, map[decad.SurfaceKind]int{decad.KindPlane: 6})
	for _, edge := range body.Edges() {
		require.Len(t, edge.Faces(), 2)
	}

	var roles []string
	for _, face := range body.Faces() {
		origins := face.Origins()
		require.Len(t, origins, 1)
		roles = append(roles, origins[0].Role)
	}
	require.ElementsMatch(t, []string{
		"capStart",
		"capEnd",
		"side(0,0,0)",
		"side(0,0,1)",
		"side(0,0,2)",
		"side(0,0,3)",
	}, roles)

	decadtest.MeasuresVolume(t, body, units.CubicMillimeters(60000), decadtest.Exactly())
	decadtest.MeasuresArea(t, body, units.SquareMillimeters(15200), decadtest.Exactly())
	decadtest.MeasuresCentroid(t, body, r3.NewVec(50, 30, 5), decadtest.Exactly())
	decadtest.MeasuresBounds(
		t,
		body,
		r3.NewVec(0, 0, 0),
		r3.NewVec(100, 60, 10),
		decadtest.Exactly(),
	)

	report, err := doc.Verify(t.Context())
	require.NoError(t, err)
	require.Equal(t, decad.Sound, report.Status)
	require.True(t, report.Passed())
	bodyReport, err := report.ForBody(body)
	require.NoError(t, err)
	require.Equal(t, decad.Sound, bodyReport.Status)
	require.Equal(t, decad.ValidityValid, bodyReport.Validity.Outcome)
	require.Equal(t, 1, bodyReport.Topology.Lumps)
	require.Equal(t, 0, bodyReport.Topology.Voids)
	require.Equal(t, decad.Exact, bodyReport.Area.Exactness)
	require.Equal(t, decad.Exact, bodyReport.Bounds.Exactness)
	require.NotNil(t, bodyReport.Region)
	require.Equal(t, decad.Exact, bodyReport.Region.Volume.Exactness)
	require.Equal(t, decad.Exact, bodyReport.Region.Centroid.Exactness)

	move, err := r3.Translation(r3.NewVec(7, -3, 11))
	require.NoError(t, err)
	placed, err := body.Placed(move)
	require.NoError(t, err)

	decadtest.MeasuresVolume(t, placed, units.CubicMillimeters(60000), decadtest.Exactly())
	decadtest.MeasuresArea(t, placed, units.SquareMillimeters(15200), decadtest.Exactly())
	decadtest.MeasuresCentroid(t, placed, r3.NewVec(57, 27, 16), decadtest.Exactly())
	decadtest.MeasuresBounds(
		t,
		placed,
		r3.NewVec(7, -3, 11),
		r3.NewVec(107, 57, 21),
		decadtest.Exactly(),
	)
	require.Equal(t, []*decad.Body{placed}, doc.Bodies())
	decadtest.MeasuresVolume(t, body, units.CubicMillimeters(60000), decadtest.Exactly())
	_, err = body.Placed(move)
	require.ErrorIs(t, err, decad.ErrRetiredBody)

	placedReport, err := doc.Verify(t.Context())
	require.NoError(t, err)
	require.Equal(t, decad.Sound, placedReport.Status)
	require.True(t, placedReport.Passed())
	placedBodyReport, err := placedReport.ForBody(placed)
	require.NoError(t, err)
	require.Equal(t, decad.Sound, placedBodyReport.Status)
	require.Equal(t, decad.ValidityValid, placedBodyReport.Validity.Outcome)
}

func TestSweepLineStagesTessellation(t *testing.T) {
	t.Parallel()

	s, profile := plateSketch(t)
	path, err := decad.NewPath(
		r3.NewVec(0, 0, 0),
		decad.LineTo{End: r3.NewVec(0, 0, 10)},
	)
	require.NoError(t, err)
	doc := decad.New()
	body, err := doc.Sweep(s, profile, path)
	require.NoError(t, err)

	_, err = body.Tessellate(units.Millimeters(0.1))
	require.ErrorIs(t, err, decad.ErrUnsupported)
	_, err = body.Fillet(decad.Edges().AtLeast(1), units.Millimeters(1))
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.Equal(t, []*decad.Body{body}, doc.Bodies())
}

func TestSweepLineTiltedFrameCarriesEndpointGap(t *testing.T) {
	t.Parallel()

	w := sketch.NewWorld()
	frame, err := r3.NewFrame(
		r3.NewVec(0, 0, 0),
		r3.NewVec(1, 1, 0),
		r3.NewVec(-1, 1, 2),
	)
	require.NoError(t, err)
	plane, err := w.CreatePlaneFromFrame(frame)
	require.NoError(t, err)
	s, err := w.CreateSketch(plane)
	require.NoError(t, err)
	rect := s.CreateRectangle(0, 0, 10, 6)
	s.Fix(rect.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)

	delta := frame.N().Scale(8)
	path, err := decad.NewPath(r3.NewVec(0, 0, 0), decad.LineTo{End: delta})
	require.NoError(t, err)
	body, err := decad.New().Sweep(s, s.Profiles()[0], path)
	require.NoError(t, err)

	start := faceVertexReadings(faceByRole(t, body, "capStart"))
	end := faceVertexReadings(faceByRole(t, body, "capEnd"))
	require.Len(t, start, 4)
	require.Len(t, end, 4)
	for _, from := range start {
		want := from.Value.Add(delta)
		matched := false
		for _, to := range end {
			allow := from.Bound.Base() + to.Bound.Base()
			if to.Value.Sub(want).Len() <= allow {
				matched = true
				break
			}
		}
		require.True(t, matched, "an end-cap vertex bound must enclose the recorded path translation")
	}
	volume, err := body.Volume()
	require.NoError(t, err)
	require.Equal(t, decad.Approximate, volume.Exactness)
	require.Positive(t, volume.Bound.Base())
}

func TestSweepLineGatesLeaveDocumentUnchanged(t *testing.T) {
	t.Parallel()

	s, profile := plateSketch(t)
	validPath, err := decad.NewPath(
		r3.NewVec(0, 0, 0),
		decad.LineTo{End: r3.NewVec(0, 0, 10)},
	)
	require.NoError(t, err)
	offPlane, err := decad.NewPath(
		r3.NewVec(0, 0, 1),
		decad.LineTo{End: r3.NewVec(0, 0, 11)},
	)
	require.NoError(t, err)
	reversed, err := decad.NewPath(
		r3.NewVec(0, 0, 0),
		decad.LineTo{End: r3.NewVec(0, 0, -10)},
	)
	require.NoError(t, err)
	skew, err := decad.NewPath(
		r3.NewVec(0, 0, 0),
		decad.LineTo{End: r3.NewVec(1, 0, 10)},
	)
	require.NoError(t, err)
	composite, err := decad.NewPath(
		r3.NewVec(0, 0, 0),
		decad.LineTo{End: r3.NewVec(0, 0, 5)},
		decad.LineTo{End: r3.NewVec(0, 0, 10)},
	)
	require.NoError(t, err)
	offPlaneComposite, err := decad.NewPath(
		r3.NewVec(0, 0, 1),
		decad.LineTo{End: r3.NewVec(0, 0, 6)},
		decad.LineTo{End: r3.NewVec(0, 0, 11)},
	)
	require.NoError(t, err)
	cornerComposite, err := decad.NewPath(
		r3.NewVec(0, 0, 0),
		decad.LineTo{End: r3.NewVec(0, 0, 5)},
		decad.LineTo{End: r3.NewVec(1, 0, 5)},
	)
	require.NoError(t, err)
	closed, err := decad.NewPath(
		r3.NewVec(0, 0, 0),
		decad.LineTo{End: r3.NewVec(0, 0, 5)},
		decad.LineTo{End: r3.NewVec(0, 0, 0)},
	)
	require.NoError(t, err)

	var nilOption decad.SweepOption
	tests := []struct {
		name    string
		sketch  *sketch.Sketch
		profile *sketch.Profile
		path    *decad.Path
		opts    []decad.SweepOption
		want    error
	}{
		{name: "nil sketch", profile: profile, path: validPath, want: decad.ErrDegenerate},
		{name: "nil profile", sketch: s, path: validPath, want: decad.ErrDegenerate},
		{name: "nil path", sketch: s, profile: profile, want: decad.ErrDegenerate},
		{name: "nil option", sketch: s, profile: profile, path: validPath, opts: []decad.SweepOption{nilOption}, want: decad.ErrDegenerate},
		{name: "foreign option", sketch: s, profile: profile, path: validPath, opts: []decad.SweepOption{foreignSweepOption{}}, want: decad.ErrDegenerate},
		{name: "wrong twist unit", sketch: s, profile: profile, path: validPath, opts: []decad.SweepOption{decad.WithSweepTwist(units.Millimeters(1))}, want: decad.ErrUnitKind},
		{name: "non-finite twist", sketch: s, profile: profile, path: validPath, opts: []decad.SweepOption{decad.WithSweepTwist(units.Degrees(math.Inf(1)))}, want: decad.ErrNotFinite},
		{name: "nonzero twist", sketch: s, profile: profile, path: validPath, opts: []decad.SweepOption{decad.WithSweepTwist(units.Degrees(1))}, want: decad.ErrUnsupported},
		{
			name:    "repeated twist",
			sketch:  s,
			profile: profile,
			path:    validPath,
			opts: []decad.SweepOption{
				decad.WithSweepTwist(units.Degrees(0)),
				decad.WithSweepTwist(units.Degrees(0)),
			},
			want: decad.ErrDegenerate,
		},
		{
			name:    "repeated nonzero twist",
			sketch:  s,
			profile: profile,
			path:    validPath,
			opts: []decad.SweepOption{
				decad.WithSweepTwist(units.Degrees(1)),
				decad.WithSweepTwist(units.Degrees(2)),
			},
			want: decad.ErrDegenerate,
		},
		{name: "off-plane start", sketch: s, profile: profile, path: offPlane, want: decad.ErrDegenerate},
		{name: "reversed tangent", sketch: s, profile: profile, path: reversed, want: decad.ErrDegenerate},
		{name: "skew tangent", sketch: s, profile: profile, path: skew, want: decad.ErrDegenerate},
		{name: "off-plane composite", sketch: s, profile: profile, path: offPlaneComposite, want: decad.ErrDegenerate},
		{name: "corner composite", sketch: s, profile: profile, path: cornerComposite, want: decad.ErrUnsupported},
		{name: "composite is staged", sketch: s, profile: profile, path: composite, want: decad.ErrUnsupported},
		{name: "closed is staged", sketch: s, profile: profile, path: closed, want: decad.ErrUnsupported},
	}

	doc := decad.New()
	for _, test := range tests {
		t.Run(test.name, func(t *testing.T) {
			_, err := doc.Sweep(test.sketch, test.profile, test.path, test.opts...)
			require.ErrorIs(t, err, test.want)
			require.Empty(t, doc.Bodies())
		})
	}

	body, err := doc.Sweep(s, profile, validPath)
	require.NoError(t, err)
	require.Equal(t, []*decad.Body{body}, doc.Bodies())
}

func bodyVertexValues(body *decad.Body) []r3.Vec {
	values := make([]r3.Vec, 0, len(body.Vertices()))
	for _, vertex := range body.Vertices() {
		values = append(values, vertex.Position().Value)
	}
	return values
}

func faceVertexReadings(face *decad.Face) []decad.VecMeasurement {
	seen := map[*decad.Vertex]struct{}{}
	var readings []decad.VecMeasurement
	for _, edge := range face.Edges() {
		for _, vertex := range []*decad.Vertex{edge.Start(), edge.End()} {
			if _, ok := seen[vertex]; ok {
				continue
			}
			seen[vertex] = struct{}{}
			readings = append(readings, vertex.Position())
		}
	}
	return readings
}

func TestSweepContextCancellationLeavesDocumentUnchanged(t *testing.T) {
	t.Parallel()

	s, profile := plateSketch(t)
	path, err := decad.NewPath(
		r3.NewVec(0, 0, 0),
		decad.LineTo{End: r3.NewVec(0, 0, 10)},
	)
	require.NoError(t, err)
	doc := decad.New()
	ctx, cancel := context.WithCancel(t.Context())
	cancel()

	_, err = doc.SweepContext(ctx, s, profile, path)
	require.ErrorIs(t, err, context.Canceled)
	require.Empty(t, doc.Bodies())

	_, err = doc.SweepContext(nil, s, profile, path)
	require.ErrorIs(t, err, decad.ErrDegenerate)
	require.Empty(t, doc.Bodies())
}
