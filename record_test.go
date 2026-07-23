package decad_test

import (
	"encoding/json"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestProfileRecordRoundTrip(t *testing.T) {
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

func TestCurveSegmentEncodeRejects(t *testing.T) {
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
	// The zero transform is invalid: it names no placement to record.
	_, err := decad.RecordTransform(r3.Transform{})
	require.ErrorIs(t, err, decad.ErrDegenerate)

	// A corrupted record — a basis that is no isometry — refuses to rebuild.
	bad := decad.TransformRecord{
		EX: r3.NewVec(2, 0, 0), // scaled: not orthonormal
		EY: r3.NewVec(0, 1, 0),
		EZ: r3.NewVec(0, 0, 1),
	}
	_, err = bad.Transform()
	require.Error(t, err, `a non-isometric record should not rebuild`)
}
