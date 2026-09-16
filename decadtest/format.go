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

	origin := b.Origin()
	step := int(origin.Step)
	steps := b.Document().Recipe().Steps
	if step < 0 || step >= len(steps) {
		return fmt.Sprintf("%s (step %d)", idxPart, step)
	}
	return fmt.Sprintf("%s (step %d %s)", idxPart, step, steps[step].Op)
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
