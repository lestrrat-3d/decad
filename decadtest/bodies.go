package decadtest

import (
	"maps"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// This file is the six body helpers: each fetches one of body's readings
// and hands it to the matching comparison rule in readings.go. The
// delegation is the whole contract — every helper here adds the fetch, the
// error handling and the body's own name, and reimplements no comparison of
// its own.

// MeasuresVolume fails tb unless body's Volume reading Measures want. body
// MUST NOT be nil. A reading error fails the test; the label the message
// opens with is the body's own name (see bodyName), so a test with several
// bodies says which one missed.
func MeasuresVolume(tb testing.TB, body *decad.Body, want units.Value, opts ...Option) {
	tb.Helper()

	if body == nil {
		tb.Fatalf("decadtest.MeasuresVolume: body must not be nil")
		return
	}

	vol, err := body.Volume()
	if err != nil {
		tb.Fatalf("%s volume: reading it failed: %v", bodyName(body), err)
		return
	}
	Measures(tb, bodyName(body)+" volume", vol, want, opts...)
}

// MeasuresArea fails tb unless body's Area reading Measures want. body MUST
// NOT be nil. A reading error fails the test; the label the message opens
// with is the body's own name (see bodyName).
func MeasuresArea(tb testing.TB, body *decad.Body, want units.Value, opts ...Option) {
	tb.Helper()

	if body == nil {
		tb.Fatalf("decadtest.MeasuresArea: body must not be nil")
		return
	}

	area, err := body.Area()
	if err != nil {
		tb.Fatalf("%s area: reading it failed: %v", bodyName(body), err)
		return
	}
	Measures(tb, bodyName(body)+" area", area, want, opts...)
}

// MeasuresCentroid fails tb unless body's Centroid reading MeasuresVec
// want. body MUST NOT be nil. A reading error fails the test; the label the
// message opens with is the body's own name (see bodyName).
func MeasuresCentroid(tb testing.TB, body *decad.Body, want r3.Vec, opts ...Option) {
	tb.Helper()

	if body == nil {
		tb.Fatalf("decadtest.MeasuresCentroid: body must not be nil")
		return
	}

	c, err := body.Centroid()
	if err != nil {
		tb.Fatalf("%s centroid: reading it failed: %v", bodyName(body), err)
		return
	}
	MeasuresVec(tb, bodyName(body)+" centroid", c, want, opts...)
}

// MeasuresBounds fails tb unless body's Bounds reading MeasuresBox lo and
// hi. body MUST NOT be nil. A reading error fails the test; the label the
// message opens with is the body's own name (see bodyName).
func MeasuresBounds(tb testing.TB, body *decad.Body, lo, hi r3.Vec, opts ...Option) {
	tb.Helper()

	if body == nil {
		tb.Fatalf("decadtest.MeasuresBounds: body must not be nil")
		return
	}

	bx, err := body.Bounds()
	if err != nil {
		tb.Fatalf("%s bounds: reading it failed: %v", bodyName(body), err)
		return
	}
	MeasuresBox(tb, bodyName(body)+" bounds", bx, lo, hi, opts...)
}

// HasSurfaceKinds fails tb unless the body's face count per surface kind
// equals want. body MUST NOT be nil. A nil want is treated as an empty map:
// every kind must then be absent from body. A kind absent from want must be
// absent from the body — that equivalence is why this helper exists at
// all, rather than a loop of individual count assertions the caller would
// otherwise write out by hand.
func HasSurfaceKinds(tb testing.TB, body *decad.Body, want map[decad.SurfaceKind]int) {
	tb.Helper()

	if body == nil {
		tb.Fatalf("decadtest.HasSurfaceKinds: body must not be nil")
		return
	}

	got := map[decad.SurfaceKind]int{}
	for _, f := range body.Faces() {
		got[f.Surface().Kind()]++
	}

	if want == nil {
		want = map[decad.SurfaceKind]int{}
	}

	if maps.Equal(got, want) {
		return
	}

	tb.Fatalf("%s: surface kind counts are %s, want %s", bodyName(body), kindCountsText(got), kindCountsText(want))
}
