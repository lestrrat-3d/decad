package decad_test

import (
	"encoding/json"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestProfileRecordRoundTrip(t *testing.T) {
	t.Parallel()
	// One profile exercising every variant of the sealed set: whole edges,
	// a certified fragment range, and a reversed walk (TStart > TEnd).
	rec := decad.ProfileRecord{
		Outer: decad.LoopRecord{
			Segments: []decad.CurveSegment{
				decad.LineSeg{Start: decad.Point2{U: 0, V: 0}, End: decad.Point2{U: 100, V: 0}, TStart: 0, TEnd: 1},
				decad.ArcSeg{Center: decad.Point2{U: 100, V: 10}, Start: decad.Point2{U: 100, V: 0}, End: decad.Point2{U: 110, V: 10}, TStart: 0, TEnd: 1},
				// A certified fragment: the entity's own fields, narrowed to
				// the certified range — walked backwards, so the range order
				// carries the walk.
				decad.LineSeg{Start: decad.Point2{U: 110, V: 10}, End: decad.Point2{U: 110, V: 80}, TStart: 0.75, TEnd: 0.25},
				decad.EllipticalArcSeg{
					Center: decad.Point2{U: 60, V: 60}, Start: decad.Point2{U: 110, V: 62.5}, End: decad.Point2{U: 10, V: 57.5},
					Rx: units.Millimeters(50), Ry: units.Millimeters(20), Rotation: units.Radians(0.1),
					TStart: 0, TEnd: 1,
				},
				decad.SplineSeg{Control: []decad.Point2{{U: 10, V: 57.5}, {U: 5, V: 40}, {U: 2, V: 20}, {U: 0, V: 0}}, TStart: 0, TEnd: 1},
				decad.ConicSeg{Start: decad.Point2{U: 0, V: 0}, Apex: decad.Point2{U: -5, V: -5}, End: decad.Point2{U: 0, V: 0}, Rho: 0.4142, TStart: 0, TEnd: 1},
				decad.FitSplineSeg{Fit: []decad.Point2{{U: 0, V: 0}, {U: 1, V: 2}, {U: 3, V: 1}}, TStart: 0, TEnd: 0.5},
				decad.NURBSSeg{
					Degree:  2,
					Control: []decad.Point2{{U: 3, V: 1}, {U: 4, V: 4}, {U: 0, V: 0}},
					Knots:   []float64{0, 0, 0, 1, 1, 1},
					Weights: []float64{1, 0.7071, 1},
					TStart:  0, TEnd: 1,
				},
			},
		},
		Holes: []decad.LoopRecord{
			// The closed kinds each bound a loop on their own; a hole walks
			// clockwise, so CCW is false.
			{Segments: []decad.CurveSegment{decad.CircleSeg{Center: decad.Point2{U: 30, V: 20}, Radius: units.Millimeters(5), CCW: false, TStart: 1, TEnd: 0}}},
			{Segments: []decad.CurveSegment{decad.EllipseSeg{Center: decad.Point2{U: 70, V: 20}, Rx: units.Millimeters(8), Ry: units.Millimeters(3), Rotation: units.Degrees(30), CCW: false, TStart: 1, TEnd: 0}}},
			{Segments: []decad.CurveSegment{decad.ClosedSplineSeg{Control: []decad.Point2{{U: 40, V: 40}, {U: 50, V: 40}, {U: 45, V: 50}}, CCW: false, TStart: 1, TEnd: 0}}},
		},
	}

	buf, err := json.Marshal(rec)
	require.NoError(t, err, `encoding the profile record should succeed`)

	var got decad.ProfileRecord
	require.NoError(t, json.Unmarshal(buf, &got), `decoding the profile record should succeed`)
	require.Equal(t, rec, got, `the record should round-trip exactly, variants, ranges and units included`)

	// The quantities keep their declared units, not just their magnitudes:
	// a unit is never silently relabelled.
	hole1, ok := got.Holes[1].Segments[0].(decad.EllipseSeg)
	require.True(t, ok, `the second hole should decode as an EllipseSeg`)
	require.Equal(t, units.Degree, hole1.Rotation.Unit(), `a rotation recorded in degrees should come back in degrees`)
	require.Equal(t, 30.0, hole1.Rotation.Mag())

	// The reversed fragment keeps its range order — the walk is the record.
	frag, ok := got.Outer.Segments[2].(decad.LineSeg)
	require.True(t, ok)
	require.Greater(t, frag.TStart, frag.TEnd, `a reversed walk keeps TStart > TEnd`)
}

func TestCurveSegmentTagging(t *testing.T) {
	t.Parallel()
	// Each variant encodes under its own kind tag: the codec is a tagged
	// object, and the tag is the dispatch key.
	buf, err := json.Marshal(decad.LoopRecord{Segments: []decad.CurveSegment{
		decad.LineSeg{Start: decad.Point2{U: 0, V: 0}, End: decad.Point2{U: 1, V: 1}, TStart: 0, TEnd: 1},
	}})
	require.NoError(t, err)

	var raw struct {
		Segments []map[string]json.RawMessage `json:"segments"`
	}
	require.NoError(t, json.Unmarshal(buf, &raw))
	require.Len(t, raw.Segments, 1)
	require.JSONEq(t, `"line"`, string(raw.Segments[0]["kind"]), `a LineSeg should carry the "line" tag`)

	// A units.Value field encodes as its own text form.
	buf, err = json.Marshal(decad.LoopRecord{Segments: []decad.CurveSegment{
		decad.CircleSeg{Center: decad.Point2{U: 0, V: 0}, Radius: units.Millimeters(5), CCW: true, TStart: 0, TEnd: 1},
	}})
	require.NoError(t, err)
	require.NoError(t, json.Unmarshal(buf, &raw))
	require.JSONEq(t, `"circle"`, string(raw.Segments[0]["kind"]))
	require.JSONEq(t, `"5 mm"`, string(raw.Segments[0]["radius"]), `a quantity should encode as its text form`)
}

func TestCurveSegmentDecodeRejects(t *testing.T) {
	t.Parallel()
	// The variant set is closed: an unknown tag has no fallback.
	var loop decad.LoopRecord
	err := json.Unmarshal([]byte(`{"segments":[{"kind":"helix","t_start":0,"t_end":1}]}`), &loop)
	require.Error(t, err, `an unknown segment kind should be rejected`)
	require.Contains(t, err.Error(), "helix")

	// A segment with no tag at all is rejected too — never guessed.
	err = json.Unmarshal([]byte(`{"segments":[{"t_start":0,"t_end":1}]}`), &loop)
	require.Error(t, err, `a segment missing its kind tag should be rejected`)
}

func TestCurveSegmentDecodeRequiresEveryField(t *testing.T) {
	t.Parallel()
	const (
		startField   = "start"
		endField     = "end"
		centerField  = "center"
		ccwField     = "ccw"
		controlField = "control"
		tStartField  = "t_start"
		tEndField    = "t_end"
	)

	tests := []struct {
		name     string
		segment  string
		required []string
	}{
		{
			name:     "line",
			segment:  `{"kind":"line","start":{"u":0,"v":0},"end":{"u":1,"v":0},"t_start":0,"t_end":1}`,
			required: []string{startField, endField, tStartField, tEndField},
		},
		{
			name:     "circle",
			segment:  `{"kind":"circle","center":{"u":0,"v":0},"radius":"2 mm","ccw":true,"t_start":0,"t_end":1}`,
			required: []string{centerField, "radius", ccwField, tStartField, tEndField},
		},
		{
			name:     "arc",
			segment:  `{"kind":"arc","center":{"u":0,"v":0},"start":{"u":1,"v":0},"end":{"u":0,"v":1},"t_start":0,"t_end":1}`,
			required: []string{centerField, startField, endField, tStartField, tEndField},
		},
		{
			name:     "ellipse",
			segment:  `{"kind":"ellipse","center":{"u":0,"v":0},"rx":"2 mm","ry":"1 mm","rotation":"0 rad","ccw":true,"t_start":0,"t_end":1}`,
			required: []string{centerField, "rx", "ry", "rotation", ccwField, tStartField, tEndField},
		},
		{
			name:     "elliptical arc",
			segment:  `{"kind":"elliptical_arc","center":{"u":0,"v":0},"start":{"u":2,"v":0},"end":{"u":0,"v":1},"rx":"2 mm","ry":"1 mm","rotation":"0 rad","t_start":0,"t_end":1}`,
			required: []string{centerField, startField, endField, "rx", "ry", "rotation", tStartField, tEndField},
		},
		{
			name:     "spline",
			segment:  `{"kind":"spline","control":[{"u":0,"v":0},{"u":1,"v":1},{"u":2,"v":1},{"u":3,"v":0}],"t_start":0,"t_end":1}`,
			required: []string{controlField, tStartField, tEndField},
		},
		{
			name:     "NURBS",
			segment:  `{"kind":"nurbs","degree":2,"control":[{"u":0,"v":0},{"u":1,"v":1},{"u":2,"v":0}],"knots":[0,0,0,1,1,1],"weights":[1,1,1],"t_start":0,"t_end":1}`,
			required: []string{"degree", controlField, "knots", "weights", tStartField, tEndField},
		},
		{
			name:     "closed spline",
			segment:  `{"kind":"closed_spline","control":[{"u":0,"v":0},{"u":1,"v":0},{"u":0,"v":1}],"ccw":true,"t_start":0,"t_end":1}`,
			required: []string{controlField, ccwField, tStartField, tEndField},
		},
		{
			name:     "fit spline",
			segment:  `{"kind":"fit_spline","fit":[{"u":0,"v":0},{"u":1,"v":1}],"t_start":0,"t_end":1}`,
			required: []string{"fit", tStartField, tEndField},
		},
		{
			name:     "conic",
			segment:  `{"kind":"conic","start":{"u":0,"v":0},"apex":{"u":1,"v":1},"end":{"u":2,"v":0},"rho":0.5,"t_start":0,"t_end":1}`,
			required: []string{startField, "apex", endField, "rho", tStartField, tEndField},
		},
	}

	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			var loop decad.LoopRecord
			require.NoError(t, json.Unmarshal([]byte(`{"segments":[`+tt.segment+`]}`), &loop),
				`the complete variant should decode`)

			var object map[string]json.RawMessage
			require.NoError(t, json.Unmarshal([]byte(tt.segment), &object))
			for _, field := range tt.required {
				t.Run("missing "+field, func(t *testing.T) {
					truncated := make(map[string]json.RawMessage, len(object)-1)
					for key, value := range object {
						if key != field {
							truncated[key] = value
						}
					}
					segment, err := json.Marshal(truncated)
					require.NoError(t, err)

					err = json.Unmarshal([]byte(`{"segments":[`+string(segment)+`]}`), &loop)
					require.Error(t, err, `an omitted required field must not become its Go zero value`)
					require.ErrorIs(t, err, decad.ErrDegenerate)
					require.Contains(t, err.Error(), field)
				})
			}
		})
	}
}

func TestCurveSegmentDecodeRequiresNestedGeometry(t *testing.T) {
	t.Parallel()
	tests := []struct {
		name    string
		segment string
		want    string
	}{
		{
			name:    "point coordinate",
			segment: `{"kind":"line","start":{"u":0},"end":{"u":1,"v":0},"t_start":0,"t_end":1}`,
			want:    "coordinate \"v\"",
		},
		{
			name:    "null point",
			segment: `{"kind":"line","start":null,"end":{"u":1,"v":0},"t_start":0,"t_end":1}`,
			want:    "start",
		},
		{
			name:    "control point coordinate",
			segment: `{"kind":"spline","control":[{"u":0},{"u":1,"v":1},{"u":2,"v":1},{"u":3,"v":0}],"t_start":0,"t_end":1}`,
			want:    "control[0]",
		},
		{
			name:    "null control point",
			segment: `{"kind":"spline","control":[null,{"u":1,"v":1},{"u":2,"v":1},{"u":3,"v":0}],"t_start":0,"t_end":1}`,
			want:    "index 0",
		},
		{
			name:    "null knot",
			segment: `{"kind":"nurbs","degree":2,"control":[{"u":0,"v":0},{"u":1,"v":1},{"u":2,"v":0}],"knots":[0,0,null,1,1,1],"weights":[1,1,1],"t_start":0,"t_end":1}`,
			want:    "index 2",
		},
	}
	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			var loop decad.LoopRecord
			err := json.Unmarshal([]byte(`{"segments":[`+tt.segment+`]}`), &loop)
			require.Error(t, err)
			require.ErrorIs(t, err, decad.ErrDegenerate)
			require.Contains(t, err.Error(), tt.want)
		})
	}
}

func TestCurveSegmentDecodeRejectsInvalidVariant(t *testing.T) {
	t.Parallel()
	tests := []struct {
		name    string
		segment string
		wantErr error
	}{
		{
			name:    "line empty range",
			segment: `{"kind":"line","start":{"u":0,"v":0},"end":{"u":1,"v":0},"t_start":0.5,"t_end":0.5}`,
			wantErr: decad.ErrDegenerate,
		},
		{
			name:    "circle winding",
			segment: `{"kind":"circle","center":{"u":0,"v":0},"radius":"2 mm","ccw":false,"t_start":0,"t_end":1}`,
			wantErr: decad.ErrDegenerate,
		},
		{
			name:    "arc range",
			segment: `{"kind":"arc","center":{"u":0,"v":0},"start":{"u":1,"v":0},"end":{"u":0,"v":1},"t_start":0,"t_end":1.1}`,
			wantErr: decad.ErrDegenerate,
		},
		{
			name:    "ellipse rotation kind",
			segment: `{"kind":"ellipse","center":{"u":0,"v":0},"rx":"2 mm","ry":"1 mm","rotation":"1 mm","ccw":true,"t_start":0,"t_end":1}`,
			wantErr: decad.ErrUnitKind,
		},
		{
			name:    "elliptical arc negative semi-axis",
			segment: `{"kind":"elliptical_arc","center":{"u":0,"v":0},"start":{"u":2,"v":0},"end":{"u":0,"v":1},"rx":"-2 mm","ry":"1 mm","rotation":"0 rad","t_start":0,"t_end":1}`,
			wantErr: decad.ErrNegativeMagnitude,
		},
		{
			name:    "spline control count",
			segment: `{"kind":"spline","control":[{"u":0,"v":0},{"u":1,"v":1},{"u":2,"v":0}],"t_start":0,"t_end":1}`,
			wantErr: decad.ErrDegenerate,
		},
		{
			name:    "NURBS weight count",
			segment: `{"kind":"nurbs","degree":2,"control":[{"u":0,"v":0},{"u":1,"v":1},{"u":2,"v":0}],"knots":[0,0,0,1,1,1],"weights":[1,1],"t_start":0,"t_end":1}`,
			wantErr: decad.ErrDegenerate,
		},
		{
			name:    "closed spline winding",
			segment: `{"kind":"closed_spline","control":[{"u":0,"v":0},{"u":1,"v":0},{"u":0,"v":1}],"ccw":false,"t_start":0,"t_end":1}`,
			wantErr: decad.ErrDegenerate,
		},
		{
			name:    "fit spline point count",
			segment: `{"kind":"fit_spline","fit":[{"u":0,"v":0}],"t_start":0,"t_end":1}`,
			wantErr: decad.ErrDegenerate,
		},
		{
			name:    "conic rho",
			segment: `{"kind":"conic","start":{"u":0,"v":0},"apex":{"u":1,"v":1},"end":{"u":2,"v":0},"rho":1,"t_start":0,"t_end":1}`,
			wantErr: decad.ErrDegenerate,
		},
	}

	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			var loop decad.LoopRecord
			err := json.Unmarshal([]byte(`{"segments":[`+tt.segment+`]}`), &loop)
			require.Error(t, err)
			require.ErrorIs(t, err, tt.wantErr)
		})
	}
}

func TestNURBSSegmentDecodeRejectsInvalidShape(t *testing.T) {
	t.Parallel()
	tests := []struct {
		name    string
		segment string
	}{
		{
			name:    "degree",
			segment: `{"kind":"nurbs","degree":0,"control":[{"u":0,"v":0},{"u":1,"v":0}],"knots":[0,1,2],"weights":[1,1],"t_start":0,"t_end":1}`,
		},
		{
			name:    "control count",
			segment: `{"kind":"nurbs","degree":2,"control":[{"u":0,"v":0},{"u":1,"v":0}],"knots":[0,0,0,1,1],"weights":[1,1],"t_start":0,"t_end":1}`,
		},
		{
			name:    "knot count",
			segment: `{"kind":"nurbs","degree":2,"control":[{"u":0,"v":0},{"u":1,"v":1},{"u":2,"v":0}],"knots":[0,0,1,1,1],"weights":[1,1,1],"t_start":0,"t_end":1}`,
		},
		{
			name:    "knot order",
			segment: `{"kind":"nurbs","degree":2,"control":[{"u":0,"v":0},{"u":1,"v":1},{"u":2,"v":0}],"knots":[0,0,0,1,0.5,1],"weights":[1,1,1],"t_start":0,"t_end":1}`,
		},
		{
			name:    "start clamp",
			segment: `{"kind":"nurbs","degree":2,"control":[{"u":0,"v":0},{"u":1,"v":1},{"u":2,"v":0}],"knots":[0,0.1,0.1,1,1,1],"weights":[1,1,1],"t_start":0,"t_end":1}`,
		},
		{
			name:    "end clamp",
			segment: `{"kind":"nurbs","degree":2,"control":[{"u":0,"v":0},{"u":1,"v":1},{"u":2,"v":0}],"knots":[0,0,0,0.9,0.9,1],"weights":[1,1,1],"t_start":0,"t_end":1}`,
		},
		{
			name:    "empty knot domain",
			segment: `{"kind":"nurbs","degree":2,"control":[{"u":0,"v":0},{"u":1,"v":1},{"u":2,"v":0}],"knots":[0,0,0,0,0,0],"weights":[1,1,1],"t_start":0,"t_end":1}`,
		},
		{
			name:    "non-positive weight",
			segment: `{"kind":"nurbs","degree":2,"control":[{"u":0,"v":0},{"u":1,"v":1},{"u":2,"v":0}],"knots":[0,0,0,1,1,1],"weights":[1,0,1],"t_start":0,"t_end":1}`,
		},
		{
			// The interior knot 0.5 repeats degree+1 times, and its two one-sided
			// limits are the recorded control points (2,0) and (3,1) — different,
			// so the curve breaks apart and the record states no single connected
			// boundary curve.
			name:    "interior knot multiplicity above degree",
			segment: `{"kind":"nurbs","degree":2,"control":[{"u":0,"v":0},{"u":1,"v":1},{"u":2,"v":0},{"u":3,"v":1},{"u":4,"v":0},{"u":5,"v":1}],"knots":[0,0,0,0.5,0.5,0.5,1,1,1],"weights":[1,1,1,1,1,1],"t_start":0,"t_end":1}`,
		},
	}
	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			var loop decad.LoopRecord
			err := json.Unmarshal([]byte(`{"segments":[`+tt.segment+`]}`), &loop)
			require.Error(t, err)
			require.ErrorIs(t, err, decad.ErrDegenerate)
		})
	}
}

// Multiplicity above the degree is not itself a discontinuity, so record
// validation must not refuse it as one: the sentinel it would carry —
// ErrDegenerate — asserts that no such body exists, and these curves exist.
// Both records here are the same shapes the case above rejects, with the one
// control point that decides continuity moved onto its partner.
func TestNURBSSegmentDecodeAdmitsContinuousMultiplicity(t *testing.T) {
	t.Parallel()
	for _, tt := range []struct {
		name    string
		segment string
	}{
		{
			// The interior knot 0.5 repeats degree+1 times and its two one-sided
			// limits are the SAME recorded point (2,0), so the two quadratic pieces
			// meet and the record states one connected curve.
			name:    "interior knot with meeting limits",
			segment: `{"kind":"nurbs","degree":2,"control":[{"u":0,"v":0},{"u":1,"v":1},{"u":2,"v":0},{"u":2,"v":0},{"u":4,"v":0},{"u":5,"v":1}],"knots":[0,0,0,0.5,0.5,0.5,1,1,1],"weights":[1,1,1,1,1,1],"t_start":0,"t_end":1}`,
		},
		{
			// A start knot clamped one repeat past degree+1: a single quadratic
			// Bézier with one dead control point, continuous everywhere.
			name:    "over-clamped start knot",
			segment: `{"kind":"nurbs","degree":2,"control":[{"u":0,"v":0},{"u":0,"v":0},{"u":1,"v":2},{"u":2,"v":0}],"knots":[0,0,0,0,1,1,1],"weights":[1,1,1,1],"t_start":0,"t_end":1}`,
		},
	} {
		t.Run(tt.name, func(t *testing.T) {
			var loop decad.LoopRecord
			require.NoError(t, json.Unmarshal([]byte(`{"segments":[`+tt.segment+`]}`), &loop))
			require.Len(t, loop.Segments, 1)
		})
	}
}

func TestCurveSegmentEncodeRejects(t *testing.T) {
	t.Parallel()
	// The codec refuses to write what cannot be read back: a non-finite
	// magnitude has no text form.
	_, err := json.Marshal(decad.LoopRecord{Segments: []decad.CurveSegment{
		decad.CircleSeg{Center: decad.Point2{U: 0, V: 0}, Radius: units.Millimeters(math.Inf(1)), CCW: true, TStart: 0, TEnd: 1},
	}})
	require.Error(t, err, `a non-finite quantity should refuse to encode`)

	// And a value of an unnamed kind — a bare 1/length — has no registered
	// unit to name it by.
	inv, err := units.Scalar(1).Div(units.Millimeters(2))
	require.NoError(t, err)
	_, err = json.Marshal(decad.LoopRecord{Segments: []decad.CurveSegment{
		decad.CircleSeg{Center: decad.Point2{U: 0, V: 0}, Radius: inv, CCW: true, TStart: 0, TEnd: 1},
	}})
	require.Error(t, err, `a quantity of an unnamed kind should refuse to encode`)
}

func TestCurveSegmentPointerVariants(t *testing.T) {
	t.Parallel()
	// The sealed methods use value receivers, so a pointer to a variant
	// satisfies CurveSegment too; the codec accepts it and records the value
	// it names, and decode hands back the value form.
	line := decad.LineSeg{Start: decad.Point2{U: 0, V: 0}, End: decad.Point2{U: 1, V: 1}, TStart: 0, TEnd: 1}
	buf, err := json.Marshal(decad.LoopRecord{Segments: []decad.CurveSegment{&line}})
	require.NoError(t, err, `a pointer variant should encode`)

	var got decad.LoopRecord
	require.NoError(t, json.Unmarshal(buf, &got))
	require.Equal(t, line, got.Segments[0], `decode should hand back the value form`)

	// A nil pointer names no curve.
	_, err = json.Marshal(decad.LoopRecord{Segments: []decad.CurveSegment{(*decad.CircleSeg)(nil)}})
	require.Error(t, err, `a nil segment pointer should refuse to encode`)
}

func TestPlaneRecordRoundTrip(t *testing.T) {
	t.Parallel()
	rec := decad.PlaneRecord{
		Origin: r3.NewVec(10, 20, 30),
		U:      r3.NewVec(1, 0, 0),
		V:      r3.NewVec(0, 1, 0),
	}
	buf, err := json.Marshal(rec)
	require.NoError(t, err)

	var got decad.PlaneRecord
	require.NoError(t, json.Unmarshal(buf, &got))
	require.Equal(t, rec, got, `a plane record should round-trip exactly`)
}

func TestTransformRecordRoundTrip(t *testing.T) {
	t.Parallel()
	// A real placement: rotate about a skew axis, then translate.
	rot, err := r3.Rotation(r3.NewVec(1, 2, 3), units.Degrees(30))
	require.NoError(t, err)
	shift, err := r3.Translation(r3.NewVec(10, -5, 2.5))
	require.NoError(t, err)
	motion, err := rot.Then(shift)
	require.NoError(t, err)

	rec, err := decad.RecordTransform(motion)
	require.NoError(t, err)
	require.Equal(t, r3.NewVec(10, -5, 2.5), rec.T, `the translation is recorded verbatim`)

	buf, err := json.Marshal(rec)
	require.NoError(t, err)
	var decoded decad.TransformRecord
	require.NoError(t, json.Unmarshal(buf, &decoded))

	rebuilt, err := decoded.Transform()
	require.NoError(t, err, `a recorded placement rebuilds through FromBasis`)
	require.True(t, rebuilt.Equal(motion, 1e-12), `the rebuilt motion should be the recorded one`)

	// The motion acts identically: apply both to a probe point.
	p := r3.NewVec(7, 8, 9)
	require.InDelta(t, motion.Apply(p).X, rebuilt.Apply(p).X, 1e-12)
	require.InDelta(t, motion.Apply(p).Y, rebuilt.Apply(p).Y, 1e-12)
	require.InDelta(t, motion.Apply(p).Z, rebuilt.Apply(p).Z, 1e-12)
}

func TestTransformRecordReflection(t *testing.T) {
	t.Parallel()
	// A reflection is a legal rigid placement (det = −1) and must survive the
	// round trip as one — a reflected solid has inverted face normals, so
	// losing the handedness would silently flip the part.
	mirror, err := r3.NewFrame(r3.NewVec(0, 0, 0), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	refl, err := r3.Reflection(mirror)
	require.NoError(t, err)
	require.True(t, refl.IsReflection())

	rec, err := decad.RecordTransform(refl)
	require.NoError(t, err)
	rebuilt, err := rec.Transform()
	require.NoError(t, err)
	require.True(t, rebuilt.IsReflection(), `the rebuilt placement keeps det = −1`)
	require.True(t, rebuilt.Equal(refl, 1e-12))
}

func TestTransformRecordRejects(t *testing.T) {
	t.Parallel()
	// The zero transform is invalid: it names no placement to record.
	_, err := decad.RecordTransform(r3.Transform{})
	require.ErrorIs(t, err, decad.ErrDegenerate)

	for _, test := range []struct {
		name string
		rec  decad.TransformRecord
		text string
	}{
		{
			name: "non-unit basis",
			rec: decad.TransformRecord{
				EX: r3.NewVec(2, 0, 0),
				EY: r3.NewVec(0, 1, 0),
				EZ: r3.NewVec(0, 0, 1),
			},
			text: "basis is not orthonormal",
		},
		{
			name: "non-finite translation",
			rec: decad.TransformRecord{
				EX: r3.NewVec(1, 0, 0),
				EY: r3.NewVec(0, 1, 0),
				EZ: r3.NewVec(0, 0, 1),
				T:  r3.NewVec(math.Inf(1), 0, 0),
			},
			text: "non-finite value",
		},
	} {
		t.Run(test.name, func(t *testing.T) {
			_, err := test.rec.Transform()
			require.ErrorIs(t, err, decad.ErrDegenerate)
			require.Contains(t, err.Error(), test.text)
		})
	}
}

func TestRecordProfileRectangle(t *testing.T) {
	t.Parallel()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(0, 0, 100, 60)
	s.Fix(rect.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)

	profiles := s.Profiles()
	require.Len(t, profiles, 1)

	rec, plane, err := decad.RecordProfile(s, profiles[0])
	require.NoError(t, err, `a clean rectangle should record`)

	// The plane is the XY datum, recorded as vectors.
	require.Equal(t, r3.NewVec(0, 0, 0), plane.Origin)
	require.Equal(t, r3.NewVec(1, 0, 0), plane.U)
	require.Equal(t, r3.NewVec(0, 1, 0), plane.V)

	// Four whole line edges, each spanning its full domain; every recorded
	// coordinate is the entity's own.
	require.Len(t, rec.Outer.Segments, 4)
	require.Empty(t, rec.Holes)
	corners := map[decad.Point2]int{}
	for _, seg := range rec.Outer.Segments {
		line, ok := seg.(decad.LineSeg)
		require.True(t, ok, `a rectangle boundary records as LineSegs, got %T`, seg)
		// A whole edge spans the full domain; the walk may run it backwards,
		// which the range order (not the fields) carries.
		lo, hi := line.TStart, line.TEnd
		if lo > hi {
			lo, hi = hi, lo
		}
		require.Equal(t, 0.0, lo)
		require.Equal(t, 1.0, hi)
		corners[line.Start]++
		corners[line.End]++
	}
	// Four corners, each shared by exactly two of the recorded lines.
	require.Len(t, corners, 4)
	for c, n := range corners {
		require.Equal(t, 2, n, `corner %v should be shared by exactly two lines`, c)
	}
	require.Contains(t, corners, decad.Point2{U: 0, V: 0})
	require.Contains(t, corners, decad.Point2{U: 100, V: 60})

	// The whole record is a value that round-trips.
	buf, err := json.Marshal(rec)
	require.NoError(t, err)
	var got decad.ProfileRecord
	require.NoError(t, json.Unmarshal(buf, &got))
	require.Equal(t, rec, got)
}

func TestRecordProfileCircleHole(t *testing.T) {
	t.Parallel()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(0, 0, 100, 60)
	s.Fix(rect.A)
	center := s.CreatePoint(50, 30)
	s.CreateCircle(center, 10)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)

	// Find the rectangle-minus-circle region: four lines outside, one hole.
	var prof *sketch.Profile
	for _, p := range s.Profiles() {
		if len(p.Outer) == 4 && len(p.Holes) == 1 {
			prof = p
			break
		}
	}
	require.NotNil(t, prof, `the rectangle-with-hole region should exist`)

	rec, _, err := decad.RecordProfile(s, prof)
	require.NoError(t, err)
	require.Len(t, rec.Holes, 1)
	require.Len(t, rec.Holes[0].Segments, 1, `a circle bounds a hole loop on its own`)

	hole, ok := rec.Holes[0].Segments[0].(decad.CircleSeg)
	require.True(t, ok, `the hole should record as a CircleSeg, got %T`, rec.Holes[0].Segments[0])
	require.Equal(t, decad.Point2{U: 50, V: 30}, hole.Center)
	require.True(t, hole.Radius.Equal(units.Millimeters(10), 1e-9), `the hole radius should be the entity's own 10 mm, got %s`, hole.Radius)
	// A hole is walked clockwise — against the circle's natural CCW — so the
	// walk is baked in: CCW false, range order reversed.
	require.False(t, hole.CCW, `a hole walks clockwise`)
	require.Greater(t, hole.TStart, hole.TEnd, `a reversed walk carries TStart > TEnd`)
}

func TestRecordProfileCertifiedFragments(t *testing.T) {
	t.Parallel()
	// A circle crossed by a rectangle: every cut is a line-involved
	// line/circle crossing, which sketch's closed-form kernel certifies —
	// the fragments record, each through the falsifier.
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(0, 0, 100, 60)
	s.Fix(rect.A)
	center := s.CreatePoint(95, 30) // straddling the rectangle's right edge
	s.CreateCircle(center, 15)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)

	// Every region of this arrangement is recordable; at least one boundary
	// must carry a certified circle fragment (the circle is split by the
	// rectangle's edge into an inside and an outside piece).
	profiles := s.Profiles()
	require.NotEmpty(t, profiles)
	fragments := 0
	for _, p := range profiles {
		if !p.Valid {
			continue
		}
		rec, _, err := decad.RecordProfile(s, p)
		require.NoError(t, err, `every certified-cut region should record`)
		for _, loop := range append([]decad.LoopRecord{rec.Outer}, rec.Holes...) {
			for _, seg := range loop.Segments {
				c, ok := seg.(decad.CircleSeg)
				if !ok {
					continue
				}
				lo, hi := c.TStart, c.TEnd
				if lo > hi {
					lo, hi = hi, lo
				}
				if lo > 0 || hi < 1 {
					fragments++
					require.Greater(t, hi, lo, `a fragment's range is non-degenerate`)
				}
			}
		}
	}
	require.NotZero(t, fragments, `the crossed circle should record at least one certified fragment`)
}

func TestRecordProfileRejectsSampledCuts(t *testing.T) {
	t.Parallel()
	// A spline crossing a circle: free-form intersection has no closed form
	// upstream (docs/spline-design.md §9 ask 3), so sketch reports the cut
	// sampled (TExact false) and decad refuses to record the fragment — never
	// approximately, never as the whole curve.
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	_, err = s.CreateSpline(
		s.CreatePoint(-30, -30), s.CreatePoint(-10, 30),
		s.CreatePoint(10, -30), s.CreatePoint(30, 30),
	)
	require.NoError(t, err)
	c := s.CreatePoint(0, 0)
	s.Fix(c)
	s.CreateCircle(c, 20)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)

	profiles := s.Profiles()
	require.NotEmpty(t, profiles)
	rejected := 0
	for _, p := range profiles {
		if !p.Valid {
			continue
		}
		_, _, err := decad.RecordProfile(s, p)
		if err != nil {
			require.ErrorIs(t, err, decad.ErrUnrecordableProfile)
			rejected++
		}
	}
	require.NotZero(t, rejected, `regions bounded by sampled spline cuts should be rejected`)
}

func TestRecordProfileRejectsWholeSketchTExactWithholding(t *testing.T) {
	t.Parallel()
	// A distant spline withholds certification but leaves the analytic
	// circle/circle fragments and their ranges unchanged.
	newProfiles := func(withSpline bool) (*sketch.Sketch, []*sketch.Profile) {
		w := sketch.NewWorld()
		s, err := w.CreateSketch(w.XY())
		require.NoError(t, err)
		s.CreateCircle(s.CreatePoint(0, 0), 5)
		s.CreateCircle(s.CreatePoint(6, 0), 5)
		if withSpline {
			_, err = s.CreateSpline(
				s.CreatePoint(40, 0), s.CreatePoint(42, 4),
				s.CreatePoint(46, 4), s.CreatePoint(48, 0),
			)
			require.NoError(t, err)
		}
		return s, s.Profiles()
	}

	certifiedSketch, certified := newProfiles(false)
	withheldSketch, withheld := newProfiles(true)
	require.Len(t, certified, 3, `the circles should form two caps and one lens`)
	require.Len(t, withheld, len(certified), `the distant spline should not change the profile set`)
	for i, certifiedProfile := range certified {
		withheldProfile := withheld[i]
		require.Len(t, withheldProfile.Outer, len(certifiedProfile.Outer))
		for j, certifiedEdge := range certifiedProfile.Outer {
			withheldEdge := withheldProfile.Outer[j]
			require.True(t, certifiedEdge.Partial)
			require.True(t, certifiedEdge.TExact, `the circle/circle bound should be certified without the spline`)
			require.True(t, withheldEdge.Partial)
			require.False(t, withheldEdge.TExact, `the whole-sketch gate should withhold certification`)
			require.Equal(t, certifiedEdge.TStart, withheldEdge.TStart, `the gate must not change the start parameter`)
			require.Equal(t, certifiedEdge.TEnd, withheldEdge.TEnd, `the gate must not change the end parameter`)
		}
		_, _, err := decad.RecordProfile(certifiedSketch, certifiedProfile)
		require.NoError(t, err, `the certified circle profile should record`)
		_, _, err = decad.RecordProfile(withheldSketch, withheldProfile)
		require.ErrorIs(t, err, decad.ErrUnrecordableProfile)
	}
}

func TestRecordProfileReportsOuterEdgeIndex(t *testing.T) {
	t.Parallel()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(-20, -20, 20, 20)
	s.Fix(rect.A)
	_, err = s.CreateSpline(
		s.CreatePoint(-30, -5), s.CreatePoint(-5, 15),
		s.CreatePoint(5, -15), s.CreatePoint(30, 5),
	)
	require.NoError(t, err)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)

	// The reported index is the FIRST rejected edge, so a fixture whose whole
	// edge comes before its sampled one proves the index is walked, not zero.
	var prof *sketch.Profile
	for _, candidate := range s.Profiles() {
		if !candidate.Valid || len(candidate.Outer) != 4 {
			continue
		}
		if !candidate.Outer[0].Partial && candidate.Outer[1].Partial {
			prof = candidate
			break
		}
	}
	require.NotNil(t, prof, `the rectangle-cut-by-a-spline region should exist`)
	require.True(t, prof.Outer[1].Partial)
	require.False(t, prof.Outer[1].TExact)

	_, _, err = decad.RecordProfile(s, prof)
	require.ErrorIs(t, err, decad.ErrUnrecordableProfile)
	require.ErrorContains(t, err, `outer edge 1`)
}

func TestRecordProfileReportsHoleAndEdgeIndices(t *testing.T) {
	t.Parallel()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(-80, -50, 80, 50)
	s.Fix(rect.A)
	s.CreateCircle(s.CreatePoint(-50, 0), 8)
	s.CreateCircle(s.CreatePoint(20, 0), 20)
	_, err = s.CreateClosedSpline(
		s.CreatePoint(45, -18), s.CreatePoint(60, 0),
		s.CreatePoint(45, 18), s.CreatePoint(30, 0),
	)
	require.NoError(t, err)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)

	var prof *sketch.Profile
	for _, candidate := range s.Profiles() {
		if candidate.Valid && len(candidate.Outer) == 4 && len(candidate.Holes) == 2 {
			prof = candidate
			break
		}
	}
	require.NotNil(t, prof, `the rectangle-with-two-holes region should exist`)
	// Hole 0 is a whole circle, which records from the entity's own data and
	// never consults TExact. Hole 1 is cut by a spline, so it is sampled.
	require.False(t, prof.Holes[0][0].Partial)
	require.True(t, prof.Holes[1][0].Partial)
	require.False(t, prof.Holes[1][0].TExact)

	_, _, err = decad.RecordProfile(s, prof)
	require.ErrorIs(t, err, decad.ErrUnrecordableProfile)
	require.ErrorContains(t, err, `hole 1 edge 0`)
}

func TestRecordProfileGates(t *testing.T) {
	t.Parallel()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(0, 0, 40, 30)
	s.Fix(rect.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	prof := s.Profiles()[0]

	// Foreign: the profile's plane-local coordinates belong to its own sketch.
	other, err := w.CreateSketch(w.XZ())
	require.NoError(t, err)
	_, _, err = decad.RecordProfile(other, prof)
	require.ErrorIs(t, err, decad.ErrForeignProfile)

	// Nil input is degenerate, not a panic.
	_, _, err = decad.RecordProfile(s, nil)
	require.ErrorIs(t, err, decad.ErrDegenerate)
	_, _, err = decad.RecordProfile(nil, prof)
	require.ErrorIs(t, err, decad.ErrDegenerate)

	// Stale: move the geometry after the snapshot — the profile still holds
	// the old boundary, and recording it would build the wrong part.
	s.AddConstraint(sketch.NewDistance(rect.A, rect.B, 55))
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	require.True(t, prof.IsStale(), `the solve should have moved the sketch under the profile`)
	_, _, err = decad.RecordProfile(s, prof)
	require.ErrorIs(t, err, decad.ErrStaleProfile)

	// A fresh profile records again.
	fresh := s.Profiles()[0]
	_, _, err = decad.RecordProfile(s, fresh)
	require.NoError(t, err)
}

func TestRecordProfileRejectsForeignBoundaryEntity(t *testing.T) {
	t.Parallel()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(0, 0, 10, 10)
	s.Fix(rect.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	prof := s.Profiles()[0]

	foreign, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	foreignRect := foreign.CreateRectangle(100, 100, 110, 110)
	foreign.Fix(foreignRect.A)
	_, err = foreign.Solve(t.Context())
	require.NoError(t, err)

	// The replacement has the same area, so the later area falsifier cannot
	// distinguish it from the profile that came from s.
	prof.Outer = foreign.Profiles()[0].Outer
	doc := decad.New()
	_, err = doc.Extrude(s, prof, decad.Distance{D: units.Millimeters(5), Dir: decad.Along})
	require.ErrorIs(t, err, decad.ErrForeignProfile)
	require.Empty(t, doc.Bodies())
	require.Empty(t, doc.Recipe().Steps)
}

func TestRecordProfileRejectsForeignHoleEntity(t *testing.T) {
	t.Parallel()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(0, 0, 10, 10)
	s.Fix(rect.A)
	s.CreateCircle(s.CreatePoint(5, 5), 2)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)

	var prof *sketch.Profile
	for _, candidate := range s.Profiles() {
		if len(candidate.Holes) == 1 {
			prof = candidate
			break
		}
	}
	require.NotNil(t, prof)

	foreign, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	center := foreign.CreatePoint(100, 100)
	foreign.Fix(center)
	foreign.CreateCircle(center, 2)
	_, err = foreign.Solve(t.Context())
	require.NoError(t, err)
	prof.Holes[0] = foreign.Profiles()[0].Outer

	_, _, err = decad.RecordProfile(s, prof)
	require.ErrorIs(t, err, decad.ErrForeignProfile)
}

func TestRecordProfileRejectsChangedCurrentBoundary(t *testing.T) {
	t.Parallel()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	first := s.CreateRectangle(0, 0, 10, 10)
	second := s.CreateRectangle(20, 0, 30, 10)
	s.Fix(first.A)
	s.Fix(second.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)

	profiles := s.Profiles()
	require.Len(t, profiles, 2)
	profiles[0].Outer = profiles[1].Outer

	_, _, err = decad.RecordProfile(s, profiles[0])
	require.ErrorIs(t, err, decad.ErrInvalidProfile)
}

func TestRecordProfileRejectsTypedNilBoundaryEntity(t *testing.T) {
	t.Parallel()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(0, 0, 10, 10)
	s.Fix(rect.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	prof := s.Profiles()[0]
	prof.Outer[0].Entity = (*sketch.Line)(nil)

	_, _, err = decad.RecordProfile(s, prof)
	require.ErrorIs(t, err, decad.ErrInvalidProfile)
}

// nearMissTriangle builds the triangle (0,0), (10,0), (0,10) whose closing line
// STARTS at (du, 10+dv) instead of at (0, 10) — the corner the previous line
// actually ends at. sketch admits the region on its own proximity threshold, so
// the profile is valid and reads an area of 50 mm2 whatever the offset.
func nearMissTriangle(t *testing.T, du, dv float64) (*sketch.Sketch, *sketch.Profile) {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	origin := s.CreatePoint(0, 0)
	right := s.CreatePoint(10, 0)
	top := s.CreatePoint(0, 10)
	missed := s.CreatePoint(du, 10+dv)
	s.CreateLine(origin, right)
	s.CreateLine(right, top)
	s.CreateLine(missed, origin)

	profiles := s.Profiles()
	require.Len(t, profiles, 1)
	require.True(t, profiles[0].Valid, `sketch should still admit the near-miss region`)
	return s, profiles[0]
}

// uncutPartialLoop builds a snapped open loop whose final edge is partial only
// because it crosses the base. Its TStart == 0 bound stays at the vertical
// line's defining top point, while sketch's arrangement snaps that node to the
// preceding near-miss line endpoint.
func uncutPartialLoop(t *testing.T, gap float64) (*sketch.Sketch, *sketch.Profile) {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	origin := s.CreatePoint(0, 0)
	right := s.CreatePoint(10, 0)
	topRight := s.CreatePoint(10, 10)
	missed := s.CreatePoint(gap, 10)
	top := s.CreatePoint(0, 10)
	below := s.CreatePoint(0, -5)
	s.CreateLine(origin, right)
	s.CreateLine(right, topRight)
	s.CreateLine(topRight, missed)
	s.CreateLine(top, below)

	profiles := s.Profiles()
	require.Len(t, profiles, 1)
	require.True(t, profiles[0].Valid)
	return s, profiles[0]
}

func TestRecordProfileRejectsUnclosedLoop(t *testing.T) {
	t.Parallel()
	// A loop whose recorded segments do not meet bounds no region at all, so
	// there is nothing for an Exact, zero-bound area to be exact ABOUT. sketch
	// snapped the gap away and reports one valid region; decad records each
	// entity's own points verbatim, which is where the gap is visible, and
	// refuses (docs/sketch-seam-design.md §3).
	for _, tc := range []struct {
		name   string
		du, dv float64
	}{
		{name: "collinear, 3e-13 mm", dv: 3e-13},
		{name: "collinear, 1e-8 mm", dv: 1e-8},
		{name: "off the line, 1e-6 mm", du: 1e-6},
	} {
		t.Run(tc.name, func(t *testing.T) {
			s, p := nearMissTriangle(t, tc.du, tc.dv)
			require.Equal(t, 50.0, p.Area, `sketch reports the snapped region's area`)
			for _, e := range p.Outer {
				require.False(t, e.Partial, `the near miss is snapped, so every edge is whole`)
			}

			_, _, err := decad.RecordProfile(s, p)
			require.ErrorIs(t, err, decad.ErrUnrecordableProfile)
			require.Contains(t, err.Error(), `does not close`)

			// The same refusal reaches the solid evaluator, which used to
			// publish a 100 mm3 Exact volume over the same open loop.
			_, err = decad.New().Extrude(s, p, decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
			require.ErrorIs(t, err, decad.ErrUnrecordableProfile)
		})
	}
}

func TestRecordProfileRejectsUnclosedConstrainedLoop(t *testing.T) {
	t.Parallel()
	// The same rule, reached the way a caller most likely reaches it: two
	// distinct points driven together by a coincidence constraint. The solver
	// converges to within its own residual, not to the same float, so the
	// recorded loop does not close and no measurement is published over it.
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	origin := s.CreatePoint(0, 0)
	right := s.CreatePoint(10, 0)
	top := s.CreatePoint(0, 10)
	mate := s.CreatePoint(0.001, 10.002)
	s.CreateLine(origin, right)
	s.CreateLine(right, top)
	s.CreateLine(mate, origin)
	s.AddConstraint(sketch.NewCoincident(top, mate))
	s.Fix(origin)
	s.Fix(right)
	solved, err := s.Solve(t.Context())
	require.NoError(t, err)
	require.True(t, solved.Converged)
	require.NotEqual(t, [2]float64{top.X(), top.Y()}, [2]float64{mate.X(), mate.Y()},
		`the solver converges within a residual, so the two points stay distinct`)

	profiles := s.Profiles()
	require.Len(t, profiles, 1)
	_, _, err = decad.RecordProfile(s, profiles[0])
	require.ErrorIs(t, err, decad.ErrUnrecordableProfile)
}

func TestRecordProfileRejectsUnclosedLoopAtUncutPartialBound(t *testing.T) {
	t.Parallel()
	gap := math.Ldexp(1, -40)
	s, profile := uncutPartialLoop(t, gap)
	partial := 0
	for _, edge := range profile.Outer {
		if !edge.Partial {
			continue
		}
		require.True(t, edge.TExact)
		require.Equal(t, 0.0, edge.TStart, `the fragment begins at its defining endpoint`)
		require.Less(t, edge.TEnd, 1.0, `only the base crossing cuts the fragment`)
		partial++
	}
	require.Equal(t, 1, partial)

	_, _, err := decad.RecordProfile(s, profile)
	require.ErrorIs(t, err, decad.ErrUnrecordableProfile)
	require.Contains(t, err.Error(), `does not close`)
}

func TestRecordProfileRecordsReversedUncutPartialBound(t *testing.T) {
	t.Parallel()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	origin := s.CreatePoint(0, 0)
	right := s.CreatePoint(10, 0)
	topRight := s.CreatePoint(10, 10)
	top := s.CreatePoint(0, 10)
	below := s.CreatePoint(0, -5)
	s.CreateLine(origin, right)
	s.CreateLine(right, topRight)
	s.CreateLine(topRight, top)
	finalLine := s.CreateLine(below, top)

	profiles := s.Profiles()
	require.Len(t, profiles, 1)
	profile := profiles[0]
	require.True(t, profile.Valid)

	var fragment sketch.BoundaryEdge
	for _, edge := range profile.Outer {
		if edge.Entity == finalLine {
			fragment = edge
			break
		}
	}
	require.True(t, fragment.Partial)
	require.True(t, fragment.Reversed)
	require.True(t, fragment.TExact)
	require.Equal(t, 1.0/3.0, fragment.TStart)
	require.Equal(t, 1.0, fragment.TEnd)

	record, _, err := decad.RecordProfile(s, profile)
	require.NoError(t, err, `the uncut natural end is the reversed walk start`)

	var recorded decad.LineSeg
	found := false
	for _, segment := range record.Outer.Segments {
		line, ok := segment.(decad.LineSeg)
		if ok && line.Start == (decad.Point2{U: 0, V: -5}) && line.End == (decad.Point2{U: 0, V: 10}) {
			recorded = line
			found = true
			break
		}
	}
	require.True(t, found)
	require.Equal(t, 1.0, recorded.TStart)
	require.Equal(t, 1.0/3.0, recorded.TEnd)
}

func TestRecordProfileRecordsSnapThresholdTrim(t *testing.T) {
	t.Parallel()
	// The same shape one order of magnitude further out: past sketch's snap
	// threshold it TRIMS the two lines at their crossing instead, so the
	// recorded loop closes on sketch's own cut and the region measures. This is
	// the boundary the refusal above must not overrun.
	s, p := nearMissTriangle(t, 1e-5, 0)
	partial := 0
	for _, e := range p.Outer {
		if e.Partial {
			require.True(t, e.TExact, `an analytic line/line cut is certified`)
			partial++
		}
	}
	require.Equal(t, 2, partial, `the two lines that miss should arrive trimmed`)

	rec, _, err := decad.RecordProfile(s, p)
	require.NoError(t, err, `a loop closed on sketch's own cut records`)
	require.Len(t, rec.Outer.Segments, 3)

	area, err := rec.Area()
	require.NoError(t, err)
	value, err := area.Value.In(units.SquareMillimeter)
	require.NoError(t, err)
	require.InDelta(t, 49.99995, value, 1e-7)
	require.Equal(t, decad.Approximate, area.Exactness, `a trimmed section is bounded, never Exact`)
	bound, err := area.Bound.In(units.SquareMillimeter)
	require.NoError(t, err)
	require.Positive(t, bound)
	require.Less(t, bound, 1e-12)
}

func TestRecordProfileRecordsMixedWholeAndCertifiedPartialJoin(t *testing.T) {
	t.Parallel()
	// The arc and vertical line share top, but sketch trims the line where it
	// crosses the base. The partial line's uncut top bound records its defining
	// point, rather than sketch's rounded node, so the shared endpoint stays an
	// exact join and the record re-authenticates.
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	center := s.CreatePoint(0, 0)
	start := s.CreatePoint(10, 0)
	top := s.CreatePoint(0, 10)
	below := s.CreatePoint(0, -5)
	s.CreateArc(center, start, top)
	s.CreateLine(top, below)
	s.CreateLine(center, start)

	profiles := s.Profiles()
	require.Len(t, profiles, 1)
	profile := profiles[0]
	wholeArc := false
	partialLine := false
	for _, edge := range profile.Outer {
		switch edge.Entity.(type) {
		case *sketch.Arc:
			require.False(t, edge.Partial)
			wholeArc = true
		case *sketch.Line:
			if edge.Partial {
				require.True(t, edge.TExact)
				partialLine = true
			}
		}
	}
	require.True(t, wholeArc)
	require.True(t, partialLine)

	record, _, err := decad.RecordProfile(s, profile)
	require.NoError(t, err)
	area, err := record.Area()
	require.NoError(t, err, `the record should also pass reconstruction authentication`)
	value, err := area.Value.In(units.SquareMillimeter)
	require.NoError(t, err)
	require.Equal(t, 78.53981633974483, value)
}

func TestProfileRecordAreaRejectsUnclosedLoop(t *testing.T) {
	t.Parallel()
	// A decoded or caller-built record reaches the same verdict through the
	// moments validator: the reconstruction authenticates each candidate region
	// through RecordProfile, which now refuses an open loop, so no candidate
	// matches and the record is reported as bounding no closed region.
	triangle := func(gap float64) decad.ProfileRecord {
		return decad.ProfileRecord{Outer: decad.LoopRecord{Segments: []decad.CurveSegment{
			decad.LineSeg{Start: decad.Point2{U: 0, V: 0}, End: decad.Point2{U: 10, V: 0}, TEnd: 1},
			decad.LineSeg{Start: decad.Point2{U: 10, V: 0}, End: decad.Point2{U: 0, V: 10}, TEnd: 1},
			decad.LineSeg{Start: decad.Point2{U: 0, V: 10 + gap}, End: decad.Point2{U: 0, V: 0}, TEnd: 1},
		}}}
	}

	_, err := triangle(3e-13).Area()
	require.ErrorIs(t, err, decad.ErrDegenerate)

	area, err := triangle(0).Area()
	require.NoError(t, err, `the closed record still measures`)
	value, err := area.Value.In(units.SquareMillimeter)
	require.NoError(t, err)
	require.Equal(t, 50.0, value)
	require.Equal(t, decad.Exact, area.Exactness)
}

func TestProfileRecordAreaRejectsUnclosedLoopAtUncutPartialBound(t *testing.T) {
	t.Parallel()
	gap := math.Ldexp(1, -40)
	record := decad.ProfileRecord{Outer: decad.LoopRecord{Segments: []decad.CurveSegment{
		decad.LineSeg{Start: decad.Point2{U: 0, V: 0}, End: decad.Point2{U: 10, V: 0}, TEnd: 1},
		decad.LineSeg{Start: decad.Point2{U: 10, V: 0}, End: decad.Point2{U: 10, V: 10}, TEnd: 1},
		decad.LineSeg{Start: decad.Point2{U: 10, V: 10}, End: decad.Point2{U: gap, V: 10}, TEnd: 1},
		decad.LineSeg{Start: decad.Point2{U: 0, V: 10}, End: decad.Point2{U: 0, V: -5}, TEnd: 2.0 / 3.0},
	}}}

	_, err := record.Area()
	require.ErrorIs(t, err, decad.ErrDegenerate)
}
