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
			{Segments: []decad.CurveSegment{decad.CircleSeg{Center: decad.Point2{U: 30, V: 20}, Radius: units.Millimeters(5), CCW: false, TStart: 0, TEnd: 1}}},
			{Segments: []decad.CurveSegment{decad.EllipseSeg{Center: decad.Point2{U: 70, V: 20}, Rx: units.Millimeters(8), Ry: units.Millimeters(3), Rotation: units.Degrees(30), CCW: false, TStart: 0, TEnd: 1}}},
			{Segments: []decad.CurveSegment{decad.ClosedSplineSeg{Control: []decad.Point2{{U: 40, V: 40}, {U: 50, V: 40}, {U: 45, V: 50}}, CCW: false, TStart: 0, TEnd: 1}}},
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
