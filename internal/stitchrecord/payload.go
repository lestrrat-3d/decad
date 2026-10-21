// Package stitchrecord holds the Stitch and Unstitch payloads and their
// builders over internal/topo nodes (agent-docs/surface-design.md §6). Root
// stitchPayload and unstitchPayload alias them.
package stitchrecord

import (
	"context"

	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
)

// Payload is Stitch's own record. It carries the operand faces
// (readable even after the operands retire — a Body is never mutated or
// freed on retirement, core §6) and the weld plan BuildPlan
// derived from them, never a previously built topology: Placed() replays
// Build from these same two values under the composed
// transform, so the admission Table J proved is REPLAYED, never re-derived,
// while the crossing audit and the global orientation sign are re-decided
// fresh from the placed triangle set every time, exactly as a rigid motion
// can change which pairs of placed triangles meet.
type Payload struct {
	Xform r3.Transform
	// Delta is the proven displacement proofbound.RigidRoundAllow charges against the
	// placed shared vertex table — zero exactly when Xform is the identity
	// transform, an exact struct comparison (agent-docs/loft-design.md §5's
	// identical fast path).
	Delta float64
	Faces []*topo.Face
	Plan  *Plan
	// LiveFaces parallels Faces after the orientation and global sign passes.
	// A curved source mesh uses this pairing without looking up copied roles.
	LiveFaces []*topo.Face
	// Verts is the shared vertex table actually built for THIS body, in
	// world coordinates — the gate diameter's stitch arm reads it back,
	// modelled on loftPayload's identical field.
	Verts []r3.Vec
	// VertBound is Verts' own per-vertex proven bound (millimetres), the
	// same widened class bound rebuildTopology's vertexForClass
	// stamps onto the live Vertex it builds for that class — read back here
	// so internal/stitchtess/stitch.go can publish a per-face bound without
	// recomputing it. Populated only alongside Tris/TriFaces below.
	VertBound []float64
	// Tris is the final wound triangle set (indices into Verts) an
	// all-planar (AllTetrahedronEligible) build assembled and
	// audited, CLOSED or OPEN — the exact set internal/stitchtess/stitch.go restates
	// with no chording. Nil for a curved or mixed stitched body: it has no
	// triangle set to restate.
	Tris [][3]int
	// TriFaces is Tris' own per-triangle LIVE rebuilt face, parallel to
	// Tris. It is never Payload.Faces (the retired operand faces) and
	// never attributed by role, since rebuildTopology copies a welded
	// operand's origins verbatim and two welded operands can share one role
	// string.
	TriFaces []*topo.Face
	// AuditClean records whether the crossing audit ran and passed for this
	// body: true for a solid (Table C requires it) and for an open sheet
	// whose audit was also run and admitted (the open-case decision,
	// agent-docs/surface-design.md §9.1's sheet validity leg 4). It is false for a
	// non-planar body (no audit runs at all) and for an open sheet whose
	// audit declined without erroring.
	AuditClean bool
}

func (sp Payload) Transform() r3.Transform { return sp.Xform }

// ProvesSimple implements topo.SimpleProver. A Payload is proven by an
// explicit build-time audit: Stitch runs agent-docs/loft-design.md §6's crossing
// audit on the OPEN case too (agent-docs/surface-design.md §6.3's open-case
// decision), so a payload whose audit ran and passed (AuditClean) admits leg 4
// the same as a proven-simple prism or loft does, letting a clean stitched
// sheet read ValidityValid instead of being permanently Suspect.
func (sp Payload) ProvesSimple(context.Context) bool { return sp.AuditClean }

func (sp Payload) Placed(ctx context.Context, env topo.Env, composed r3.Transform) (*topo.Body, error) {
	return Build(ctx, env, sp.Faces, sp.Plan, composed)
}

// SurveyClass names the payload class a staged survey diagnostic reports
// (agent-docs/evaluator-design.md §8): the root type this payload replaced.
func (Payload) SurveyClass() string { return "stitchPayload" }

// UnstitchPayload is Unstitch's own record: one face read straight off the
// retired receiver's own B-rep (never a previously built topology of its
// own), the rigid placement accumulated so far, and the receiver's own
// already-proven Bounds — the same "record, never rebuild" contract
// Payload keeps for a whole welded set (build.go), restricted here to
// one face. Placed() replays BuildUnstitchFace from these same values
// under the composed transform, exactly as Payload.Placed replays
// Build.
type UnstitchPayload struct {
	Xform r3.Transform
	// Delta is the proven displacement proofbound.RigidRoundAllow charges against this
	// face's placed geometry — zero exactly when Xform is the identity
	// transform, an exact struct comparison.
	Delta float64
	// Face is the ORIGINAL (now retired) receiver's own Face: read-only, and
	// never mutated or aliased into a live body's topology (BuildUnstitchFace
	// always mints a fresh copy from it).
	Face *topo.Face
	// Bounds is the retired receiver's own proven, UNPLACED Bounds, read by
	// unstitchBounds (faceBounds's fallback) for every face that is not
	// straight-edged and planar — reused rather than re-derived into a
	// tighter per-face box, which would understate a curved face's true
	// extent exactly as bounds's own doc comment states for a curved
	// Stitch operand (build.go). A single face's true box is always a
	// subset of the whole receiver's, so this stays sound even though it is
	// not the tightest box this one face could in principle earn. A
	// straight-edged planar face never reads this field: faceBounds computes
	// its own tight box off the placed face's held vertices instead.
	Bounds measurement.Box
}

func (up UnstitchPayload) Transform() r3.Transform { return up.Xform }

// AxialDelta reports the held face's own proven axial displacement
// (stops.go's axialDisplacement interface), so an unstitched sheet's
// payload-wide fallback (payloadAxialDelta) reads the figure the face holds
// (topo.Face.HasAxialDelta) — zero, honestly, for a face that does not.
func (up UnstitchPayload) AxialDelta() float64 { return up.Face.AxialDelta }

func (up UnstitchPayload) Placed(ctx context.Context, env topo.Env, composed r3.Transform) (*topo.Body, error) {
	return BuildUnstitchFace(ctx, env, up.Face, up.Bounds, composed)
}

// SurveyClass names the payload class a staged survey diagnostic reports
// (agent-docs/evaluator-design.md §8): the root type this payload replaced.
func (UnstitchPayload) SurveyClass() string { return "unstitchPayload" }
