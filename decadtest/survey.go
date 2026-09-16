package decadtest

import (
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/units"
)

// This file is the three per-body survey helpers: each checks the survey's
// own primary outcome, then hands its measured reading to readings.go's
// Measures. decad.WallResult.Minimum and decad.ConcaveRadiusResult.Minimum
// are both *decad.ScalarReading — a pointer to a struct that EMBEDS
// decad.Measurement (verify_result.go:293, :314) — so the field each helper
// passes to Measures is .Minimum.Measurement, never .Minimum itself.
// Minimum is non-nil exactly when Outcome == decad.ScalarMeasured, and nil
// for every other outcome, so each helper checks Outcome first and never
// dereferences Minimum on any other branch.

// WallMinimum fails tb unless the wall survey measured a minimum matching
// want. br MUST NOT be nil.
func WallMinimum(tb testing.TB, br *decad.BodyReport, want units.Value, opts ...Option) {
	tb.Helper()

	if br == nil {
		tb.Fatalf("decadtest.WallMinimum: br must not be nil")
		return
	}

	if br.Wall.Outcome != decad.ScalarMeasured {
		tb.Fatalf("%s wall minimum: survey outcome is %s, want measured; %s",
			bodyName(br.Body), br.Wall.Outcome, diagnosticBlock(nil, br.Wall.Diagnostics))
		return
	}
	if br.Wall.Minimum == nil {
		tb.Fatalf("%s wall minimum: outcome is measured but the reading is absent", bodyName(br.Body))
		return
	}

	Measures(tb, bodyName(br.Body)+" wall minimum", br.Wall.Minimum.Measurement, want, opts...)
}

// ConcaveRadius fails tb unless the concave-radius survey measured a
// minimum matching want. br MUST NOT be nil. decad.ConcaveRadiusResult has
// no Assessment field — Verify accepts no radius requirement — so this
// helper looks for none.
func ConcaveRadius(tb testing.TB, br *decad.BodyReport, want units.Value, opts ...Option) {
	tb.Helper()

	if br == nil {
		tb.Fatalf("decadtest.ConcaveRadius: br must not be nil")
		return
	}

	if br.ConcaveRadius.Outcome != decad.ScalarMeasured {
		tb.Fatalf("%s concave radius: survey outcome is %s, want measured; %s",
			bodyName(br.Body), br.ConcaveRadius.Outcome, diagnosticBlock(nil, br.ConcaveRadius.Diagnostics))
		return
	}
	if br.ConcaveRadius.Minimum == nil {
		tb.Fatalf("%s concave radius: outcome is measured but the reading is absent", bodyName(br.Body))
		return
	}

	Measures(tb, bodyName(br.Body)+" concave radius", br.ConcaveRadius.Minimum.Measurement, want, opts...)
}

// UndercutFaces fails tb unless the undercut survey decided every face
// (br.Undercut.Coverage == decad.CoverageComplete) and found exactly n
// confirmed opposing faces, and returns them, in the body's own Faces()
// order. Every face UndercutFaces returns is a CONFIRMED opposing face
// against the requested pull; no uncertain face appears. The survey runs
// only when decad.WithPullDirection was passed to Verify; without it
// Coverage reads not_requested and this helper fails, which is the correct
// outcome for a test that forgot the option. br MUST NOT be nil.
func UndercutFaces(tb testing.TB, br *decad.BodyReport, n int) []*decad.Face {
	tb.Helper()

	if br == nil {
		tb.Fatalf("decadtest.UndercutFaces: br must not be nil")
		return nil
	}

	if br.Undercut.Coverage != decad.CoverageComplete {
		tb.Fatalf("%s undercut: coverage is %s, want complete; %s",
			bodyName(br.Body), br.Undercut.Coverage, diagnosticBlock(nil, br.Undercut.Diagnostics))
		return nil
	}
	if len(br.Undercut.Faces) != n {
		tb.Fatalf("%s undercut: %d opposing face(s), want %d", bodyName(br.Body), len(br.Undercut.Faces), n)
		return nil
	}
	return br.Undercut.Faces
}
