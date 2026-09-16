package decadtest

import (
	"fmt"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/units"
)

// This file is the verification helpers: running decad.Document.Verify for
// the test, judging the whole report, looking up one body's record, judging
// a body's proven solidity, and asserting on the report's diagnostic and
// pair inventories. Clearance and Interference are the two calls here whose
// comparison terminates in readings.go's Measures; every other helper
// compares only decided answers — an outcome, a count, a pointer identity —
// with ==.

// Verify runs doc.Verify under the test's own context and fails tb if it
// errors. doc MUST NOT be nil. The options are decad.VerifyOption, not
// decadtest.Option — this call configures decad's own verifier, never the
// kit's comparison, and opts is passed straight through. tb.Context() is
// used, so the verification is cancelled when the test ends.
func Verify(tb testing.TB, doc *decad.Document, opts ...decad.VerifyOption) *decad.Report {
	tb.Helper()

	if doc == nil {
		tb.Fatalf("decadtest.Verify: doc must not be nil")
		return nil
	}

	report, err := doc.Verify(tb.Context(), opts...)
	if err != nil {
		tb.Fatalf("verify: Document.Verify failed: %v", err)
		return nil
	}
	return report
}

// Sound runs Verify and fails tb unless the whole report passed
// (decad.Report.Passed, true only for decad.Sound). A Sound report does NOT
// imply that every optional survey ran: Wall, Undercut and ConcaveRadius
// each read NotRequested on a Sound report unless the matching
// decad.VerifyOption was passed to opts. On failure, every diagnostic in
// the report is printed.
func Sound(tb testing.TB, doc *decad.Document, opts ...decad.VerifyOption) *decad.Report {
	tb.Helper()

	report := Verify(tb, doc, opts...)
	if report == nil {
		return nil
	}
	if report.Passed() {
		return report
	}

	tb.Fatalf("verify: report is %s, want Sound; %s", report.Status, diagnosticBlock(report, report.Diagnostics))
	return nil
}

// Status fails tb unless report's own Status is want. report MUST NOT be
// nil. On mismatch, every diagnostic in the report is printed.
func Status(tb testing.TB, report *decad.Report, want decad.Status) {
	tb.Helper()

	if report == nil {
		tb.Fatalf("decadtest.Status: report must not be nil")
		return
	}
	if report.Status == want {
		return
	}

	tb.Fatalf("verify: report is %s, want %s; %s", report.Status, want, diagnosticBlock(report, report.Diagnostics))
}

// BodyReport looks body up in report and fails tb when the report holds no
// record of it. report and body MUST NOT be nil. decad.Report.ForBody
// matches by exact pointer identity and consults only the report's own
// recorded bodies, never current document membership, so a body later
// retired by a boolean is still resolvable here if the report was taken
// while it was live.
func BodyReport(tb testing.TB, report *decad.Report, body *decad.Body) *decad.BodyReport {
	tb.Helper()

	if report == nil {
		tb.Fatalf("decadtest.BodyReport: report must not be nil")
		return nil
	}
	if body == nil {
		tb.Fatalf("decadtest.BodyReport: body must not be nil")
		return nil
	}

	br, err := report.ForBody(body)
	if err != nil {
		tb.Fatalf("%s: the report holds no record of this body: %v", bodyName(body), err)
		return nil
	}
	return br
}

// Valid fails tb unless the body is a proven solid with exactly one lump
// and no voids: br.Validity.Outcome == decad.ValidityValid,
// br.Topology.Lumps == 1, br.Topology.Voids == 0. It reads ONLY Validity and
// Topology — NEVER br.Status or br.Diagnostics — because a tolerance
// diagnostic (an area or centroid reading beyond the caller's relative
// tolerance) does not make a body invalid; a caller who also wants that
// precision judgement should use Sound or Status.
func Valid(tb testing.TB, report *decad.Report, body *decad.Body) *decad.BodyReport {
	tb.Helper()

	br := BodyReport(tb, report, body)
	if br == nil {
		return nil
	}

	if br.Validity.Outcome != decad.ValidityValid {
		tb.Fatalf("%s: validity is %s, want valid; %s",
			reportBodyName(report, body), br.Validity.Outcome, diagnosticBlock(report, br.Validity.Diagnostics))
		return nil
	}
	if br.Topology.Lumps != 1 {
		tb.Fatalf("%s: body has %d lump(s), want exactly 1", reportBodyName(report, body), br.Topology.Lumps)
		return nil
	}
	if br.Topology.Voids != 0 {
		tb.Fatalf("%s: body has %d void(s), want none", reportBodyName(report, body), br.Topology.Voids)
		return nil
	}
	return br
}

// Diagnosed fails tb unless at least one diagnostic in report.Diagnostics
// carries code, and returns every diagnostic that does, for field checks by
// the caller. report MUST NOT be nil.
//
// report.Diagnostics is the canonical inventory for the whole call: body
// diagnostics in body order, then pair diagnostics in pair order, each
// finding present exactly once. The same finding is also reachable through
// its own result's local Diagnostics slice, so a caller must not add the
// two together.
func Diagnosed(tb testing.TB, report *decad.Report, code decad.DiagnosticCode) []decad.Diagnostic {
	tb.Helper()

	if report == nil {
		tb.Fatalf("decadtest.Diagnosed: report must not be nil")
		return nil
	}

	var found []decad.Diagnostic
	for _, d := range report.Diagnostics {
		if d.Code == code {
			found = append(found, d)
		}
	}
	if len(found) == 0 {
		tb.Fatalf("verify: no diagnostic carries %s; the report carries %s", code, diagnosticBlock(report, report.Diagnostics))
		return nil
	}
	return found
}

// OnlyDiagnostics fails tb when the report carries a diagnostic whose code
// is not among allowed. Passing no code at all requires an empty report.
// report MUST NOT be nil.
func OnlyDiagnostics(tb testing.TB, report *decad.Report, allowed ...decad.DiagnosticCode) {
	tb.Helper()

	if report == nil {
		tb.Fatalf("decadtest.OnlyDiagnostics: report must not be nil")
		return
	}

	ok := make(map[decad.DiagnosticCode]struct{}, len(allowed))
	for _, c := range allowed {
		ok[c] = struct{}{}
	}

	var bad []decad.Diagnostic
	for _, d := range report.Diagnostics {
		if _, found := ok[d.Code]; !found {
			bad = append(bad, d)
		}
	}
	if len(bad) == 0 {
		return
	}

	tb.Fatalf("verify: %d diagnostic(s) is not among the allowed diagnostic codes %v; %s",
		len(bad), allowed, diagnosticBlock(report, bad))
}

// Clearance fails tb unless the report holds a decad.Clearance row for the
// pair, in either order, whose Gap Measures want. A row exists only for a
// pair PROVEN disjoint with a measured gap, and only when
// decad.WithClearances() was passed to Verify; a pair whose gap the kernel
// cannot prove yields no row and the report reads Suspect. Gap is a
// decad.Measurement, so readings.go's whole interval rule applies to it
// unchanged. report, a and b MUST NOT be nil.
func Clearance(tb testing.TB, report *decad.Report, a, b *decad.Body, want units.Value, opts ...Option) {
	tb.Helper()

	if report == nil {
		tb.Fatalf("decadtest.Clearance: report must not be nil")
		return
	}
	if a == nil {
		tb.Fatalf("decadtest.Clearance: a must not be nil")
		return
	}
	if b == nil {
		tb.Fatalf("decadtest.Clearance: b must not be nil")
		return
	}

	for _, row := range report.Clearances {
		if (row.A == a && row.B == b) || (row.A == b && row.B == a) {
			what := fmt.Sprintf("clearance (%s, %s) gap", reportBodyName(report, a), reportBodyName(report, b))
			Measures(tb, what, row.Gap, want, opts...)
			return
		}
	}

	tb.Fatalf("clearance: the report holds no clearance row for the pair (%s, %s); it holds %d row(s)",
		reportBodyName(report, a), reportBodyName(report, b), len(report.Clearances))
}

// Interference fails tb unless the report holds a decad.Interference row
// for the pair, in either order, whose Volume Measures want.
// decad.Document.Verify checks interference even when no options are
// passed, so no decad.VerifyOption is needed to obtain one. report, a and b
// MUST NOT be nil.
func Interference(tb testing.TB, report *decad.Report, a, b *decad.Body, want units.Value, opts ...Option) {
	tb.Helper()

	if report == nil {
		tb.Fatalf("decadtest.Interference: report must not be nil")
		return
	}
	if a == nil {
		tb.Fatalf("decadtest.Interference: a must not be nil")
		return
	}
	if b == nil {
		tb.Fatalf("decadtest.Interference: b must not be nil")
		return
	}

	for _, row := range report.Interferences {
		if (row.A == a && row.B == b) || (row.A == b && row.B == a) {
			what := fmt.Sprintf("interference (%s, %s) volume", reportBodyName(report, a), reportBodyName(report, b))
			Measures(tb, what, row.Volume, want, opts...)
			return
		}
	}

	tb.Fatalf("interference: the report holds no interference row for the pair (%s, %s); it holds %d row(s)",
		reportBodyName(report, a), reportBodyName(report, b), len(report.Interferences))
}
