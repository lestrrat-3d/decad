package decadtest

import (
	"fmt"
	"maps"
	"slices"
	"strings"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/units"
)

// This file owns every string the kit prints. Nothing here decides pass or
// fail; every decision lives in readings.go and bodies.go.

// kindPhrase renders a units.Kind as "volume (mm^3)", or as the bare kind
// name when the kind has no unit symbol (Dimensionless has one — One —
// whose Symbol is "").
func kindPhrase(k units.Kind) string {
	u, ok := units.BaseUnit(k)
	if !ok || u.Symbol() == "" {
		return k.String()
	}
	return fmt.Sprintf("%s (%s)", k, u.Symbol())
}

// kindMismatch is the one wrong-Kind message every helper reports: subject
// is the caller's word for the thing whose Kind is wrong ("Within slack",
// "WithinRel", "expected value", "ceiling", "the first reading"); got is
// that thing's own Kind; want is the Kind it was compared against.
func kindMismatch(what, subject string, got, want units.Kind) string {
	return fmt.Sprintf("%s: %s is a %s but the reading is a %s", what, subject, kindPhrase(got), kindPhrase(want))
}

// bodyName names a body the way every failure message does: its index in
// its document's live body list, the recipe step that produced it, and that
// step's op — "body[2] (step 3 union)". It degrades rather than panics: a
// nil body renders "<nil body>"; a body retired from its document's live
// list (consumed by a boolean, say) renders without an index; a step index
// out of range for the recipe renders without an op.
func bodyName(b *decad.Body) string {
	if b == nil {
		return "<nil body>"
	}

	idxPart := "body"
	for i, other := range b.Document().Bodies() {
		if other == b {
			idxPart = fmt.Sprintf("body[%d]", i)
			break
		}
	}
	return bodyStepName(b, idxPart)
}

// bodyStepName appends the recipe step and op that produced b to idxPart —
// the shared tail of bodyName and reportBodyName. A step index out of range
// for the recipe renders without an op.
func bodyStepName(b *decad.Body, idxPart string) string {
	origin := b.Origin()
	step := int(origin.Step)
	steps := b.Document().Recipe().Steps
	if step < 0 || step >= len(steps) {
		return fmt.Sprintf("%s (step %d)", idxPart, step)
	}
	return fmt.Sprintf("%s (step %d %s)", idxPart, step, steps[step].Op)
}

// reportBodyName names a body by its position in the report's OWN body list
// — Report.Bodies order at the call — "body[0] (step 3 union)". A nil
// report, or a body the report does not hold, falls back to bodyName's
// Document.Bodies() order; that fallback is what lets diagnosticBlock be
// called with a nil report from survey.go, where no report is in scope. A
// nil b renders "<nil body>".
func reportBodyName(report *decad.Report, b *decad.Body) string {
	if b == nil {
		return "<nil body>"
	}
	if report == nil {
		return bodyName(b)
	}

	for i, br := range report.Bodies {
		if br.Body == b {
			return bodyStepName(b, fmt.Sprintf("body[%d]", i))
		}
	}
	return bodyName(b)
}

// diagnosticLine renders one decad.Diagnostic as a numbered message line:
//
//	[1] body[0] (step 3 union) measurement_beyond_tolerance area: 1256.63706 mm^2 ± 0.0213 mm^2 (Approximate), required bound ≤ 0.00125664 mm^2
//	[2] pair (body[0] (step 3 union), body[1] (step 4 extrude)) undecided_pair: the partition proof resolved neither way
//
// The subject is reportBodyName(report, d.Body) for a body diagnostic, or
// "pair (A, B)" for a pair diagnostic, empty for neither. d.Code follows,
// printed through its String() — the stable token, never the iota value.
// When d.Reading is not decad.ReadingNone, the reading's name and its
// observed form follow: ReadingCentroid reads d.ObservedVec, ReadingBounds
// reads d.ObservedBox, every other named reading reads d.Observed — exactly
// one of the three is meant to be non-nil, keyed by d.Reading, but a nil one
// renders nothing after the colon rather than panicking. d.Required, when
// non-nil, appends the required bound. Finally, when d.Reading is
// decad.ReadingNone and d.Message is non-empty, the message is appended.
func diagnosticLine(report *decad.Report, n int, d decad.Diagnostic) string {
	var subject string
	switch {
	case d.Body != nil:
		subject = reportBodyName(report, d.Body)
	case d.Pair != nil:
		subject = fmt.Sprintf("pair (%s, %s)", reportBodyName(report, d.Pair.A), reportBodyName(report, d.Pair.B))
	}

	line := fmt.Sprintf("  [%d] ", n)
	if subject != "" {
		line += subject + " "
	}
	line += d.Code.String()

	if d.Reading != decad.ReadingNone {
		line += fmt.Sprintf(" %s: ", d.Reading)
		switch d.Reading {
		case decad.ReadingCentroid:
			if d.ObservedVec != nil {
				line += vecMeasurementText(*d.ObservedVec)
			}
		case decad.ReadingBounds:
			if d.ObservedBox != nil {
				line += boxText(*d.ObservedBox)
			}
		default:
			if d.Observed != nil {
				line += measurementText(*d.Observed)
			}
		}
	}

	if d.Required != nil {
		line += fmt.Sprintf(", required bound ≤ %s", d.Required)
	}

	if d.Reading == decad.ReadingNone && d.Message != "" {
		line += ": " + d.Message
	}

	return line
}

// diagnosticBlock renders every diagnostic in ds, one numbered line each,
// prefixed by its own count. report may be nil (see reportBodyName).
func diagnosticBlock(report *decad.Report, ds []decad.Diagnostic) string {
	lines := make([]string, 0, len(ds))
	for i, d := range ds {
		lines = append(lines, diagnosticLine(report, i+1, d))
	}
	return fmt.Sprintf("%d diagnostic(s):\n", len(ds)) + strings.Join(lines, "\n")
}

// measurementText renders a scalar reading as "60000 mm^3 ± 0 mm^3 (Exact)".
func measurementText(m decad.Measurement) string {
	return fmt.Sprintf("%s ± %s (%s)", m.Value, m.Bound, m.Exactness)
}

// vecMeasurementText renders a vector reading as "{50 30 5} ± 0 mm (Exact)".
func vecMeasurementText(m decad.VecMeasurement) string {
	return fmt.Sprintf("%v ± %s (%s)", m.Value, m.Bound, m.Exactness)
}

// boxText renders a box as "min {0 0 0} max {100 60 10} ± 0 mm (Exact)".
func boxText(b decad.Box) string {
	return fmt.Sprintf("min %v max %v ± %s (%s)", b.Min, b.Max, b.Bound, b.Exactness)
}

// surfaceKindNames is the kit's own name table: decad.SurfaceKind has no
// String method, so %v on decad.KindPlane prints an integer.
var surfaceKindNames = map[decad.SurfaceKind]string{
	decad.KindPlane:    "plane",
	decad.KindCylinder: "cylinder",
	decad.KindCone:     "cone",
	decad.KindSphere:   "sphere",
	decad.KindTorus:    "torus",
	decad.KindNURBS:    "nurbs",
	decad.KindFaceted:  "faceted",
}

// surfaceKindName names a decad.SurfaceKind for a message. It carries a
// default branch rather than panicking on an unrecognized kind:
// docs/api-design.md §3 states that vN's surface set is a superset of v1's,
// so a switch (or lookup) over Surface always needs one.
func surfaceKindName(k decad.SurfaceKind) string {
	if name, ok := surfaceKindNames[k]; ok {
		return name
	}
	return fmt.Sprintf("SurfaceKind(%d)", int(k))
}

// kindCountsText renders a surface-kind census sorted by kind, as
// "{plane: 5, cylinder: 1}".
func kindCountsText(counts map[decad.SurfaceKind]int) string {
	kinds := slices.Collect(maps.Keys(counts))
	slices.Sort(kinds)

	parts := make([]string, 0, len(kinds))
	for _, k := range kinds {
		parts = append(parts, fmt.Sprintf("%s: %d", surfaceKindName(k), counts[k]))
	}
	return "{" + strings.Join(parts, ", ") + "}"
}
