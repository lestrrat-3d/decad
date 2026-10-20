// Package prismrecord owns the plain data of the evaluator's prism record: the
// recorded section, the plane frame it lifts through, the two sweep levels with
// their proven displacements, the accumulated rigid placement, the blend-role
// descriptors and the walk cache, with the builder that evaluates it over
// topology nodes (Build) and its topo.Payload methods. The root package
// names it prismPayload (agent-docs/evaluator-design.md §5, §8).
package prismrecord

import (
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/r3"
)

// Payload is the plain record a straight extrude evaluates from: the recorded
// region, the plane frame it lifts through, the signed sweep interval, and the
// accumulated rigid placement. Every measurement and the whole topology derive
// from it, which is what makes Transform exact: Placed re-evaluates the same
// record under the composed motion (agent-docs/evaluator-design.md §8).
//
// A prism-derived modify body (a filleted or chamfered prism, modify §2) also
// carries the blend-role descriptors of its own rewritten section: BlendSegs
// names, per loop, the (loop, segment) indices whose side face is a blend wall,
// and BlendKind is "fillet" or "chamfer". They are part of the re-evaluable
// record so a copy or placement re-mints its own blend roles from its own record
// (the modify §9 role rule) — a plain extrude leaves them empty (no-op).
//
// SurfaceResult is WithSurfaceResult's own flag (agent-docs/surface-design.md §4):
// true when the build must omit its closing caps and publish a sheet instead
// of a solid. It is part of the re-evaluable record for the same reason the
// blend descriptors are, so Transform and Copy reproduce the
// sheet with no further code — a plain solid extrude leaves it false.
//
// SectionDelta is the proven upper bound on how far any recorded boundary
// coordinate of the section sits from the section this payload's construction
// DENOTES (agent-docs/prism-boolean-design.md §7). It is zero for every payload a
// caller draws directly — a plain extrude, a placement and every modify rewrite
// record their own coordinates, so the record IS the section they denote — and
// the analytic prism boolean is the one construction that sets it, to the
// displacement its coordinate re-expression commits. Being a payload field it
// re-evaluates with the payload, so a placement or copy keeps it, and every
// measurement evalPrism publishes composes it (never Exact while it is nonzero).
//
// Z0Delta and Z1Delta are the AXIAL twin of that displacement, one per end: each
// bounds how far the sweep level recorded beside it sits from the level this
// payload's construction denotes. A level the caller stated is its own denotation
// and carries zero — a Distance in millimetres records the number it was given —
// while a COMPUTED level carries the computation's own proven rounding: a ToFace
// or ThroughAll stop resolves its level by float arithmetic over another body's
// face (stops.go), a magnitude in a non-base unit rounds in its rescale to
// millimetres (extent.MagnitudeInBounded), and a chamfered end pulls its level in by the
// setback (capblend_moments.go). The two displacements are tracked apart and
// neither ever stands in for the other — SectionDelta moves a boundary coordinate
// IN the plane, these move a level ALONG the normal — while a reading both of them
// displace, a side vertex or the box, sums the two into its own bound. Every
// level-derived reading takes these: the sweep
// height and the volume, wall area and centroid built on it, the box, the side
// vertices and the vertical edge lengths. Being payload fields they re-evaluate
// with the payload, so a placement or copy keeps them.
//
// Walks is the plane-local walk resolution of THIS payload's own profile,
// published by the evaluation that built the body and carried through every
// rigid re-evaluation of it (Placed). It is a pure cache: it
// holds nothing the record does not already determine, nothing
// placement-dependent, and every read of it is guarded by
// momentinput.ProfileWalks.reusable, so a payload whose profile differs by one
// float bit resolves afresh. A payload a caller draws directly — a plain
// extrude, a modify rewrite, a boolean result — leaves it nil and resolves as
// before. See agent-docs/evaluator-design.md §8.
type Payload struct {
	Profile       momentinput.Profile
	Frame         r3.Frame
	Z0, Z1        float64
	Z0Delta       float64
	Z1Delta       float64
	Xform         r3.Transform
	BlendSegs     []map[int]struct{}
	BlendKind     string
	SectionDelta  float64
	Walks         *momentinput.ProfileWalks
	SurfaceResult bool
}

// SurveyClass names the payload class a staged survey diagnostic reports
// (internal/reportvocab): the root type name this payload had before it moved.
func (Payload) SurveyClass() string { return "prismPayload" }
