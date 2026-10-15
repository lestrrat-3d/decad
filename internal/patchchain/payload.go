package patchchain

import (
	"context"

	"github.com/lestrrat-3d/decad/internal/denotation"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/r3"
)

// Payload records what a PatchProfile body was built from — profile, frame and
// placement — exactly as prismrecord.Payload does for an extrude, so Transform
// and Copy re-evaluate it the same way (docs/evaluator-design.md §8). It is its
// own type rather than a zero-height prismrecord.Payload: roughly sixteen call
// sites across the root package type-assert the prism payload, each ok-guarded
// or carrying a default arm, so a patch falls through to the existing refusal
// there instead of being silently handed a zero-height prism. Prism is the one
// bridge — a *view*, never routed through prismrecord.Build, which rejects
// h <= 0 with ErrDegenerate — that lets a patch reuse Payload.CapFrame,
// Payload.Point and prismrecord.Bounds, each of which is correct at zero
// height.
type Payload struct {
	Profile momentinput.Profile
	Frame   r3.Frame
	Xform   r3.Transform
	Walks   *momentinput.ProfileWalks
}

// Transform is the accumulated rigid placement.
func (pp Payload) Transform() r3.Transform { return pp.Xform }

// Placed re-evaluates the same record under the composed motion (core §8). A
// re-evaluation path: no moments preflight has run on this record within the
// call, so the build opens the record's one free-form work counter itself
// (docs/spline-design.md §5.2).
func (pp Payload) Placed(ctx context.Context, env topo.Env, composed r3.Transform) (*topo.Body, error) {
	pp.Xform = composed
	return Build(ctx, env, pp, freeform.NewFreeformWork())
}

// Prism is a *view* of pp as a zero-height prismrecord.Payload — never a body
// this evaluator builds — used only to feed Payload.CapFrame, Payload.Point
// and prismrecord.Bounds, each of which reads a section and a placement and is
// correct at z0 == z1 == 0. Every prism field this view leaves at its zero
// value (Z0Delta, Z1Delta, SectionDelta, blend roles, SurfaceResult) is
// meaningless for a patch, which denotes its own recorded section exactly as a
// plain extrude does.
func (pp Payload) Prism() prismrecord.Payload {
	return prismrecord.Payload{Profile: pp.Profile, Frame: pp.Frame, Xform: pp.Xform, Walks: pp.Walks}
}

// Chain is one proven chain, carried in the payload from the initial build to
// every later placement: the chain's own (receiver-owned, read-only) edges.
// Gate 3's proof is a pure yes/no decision over these same edges' own held
// data, never re-run once passed — a rigid motion preserves planarity exactly,
// so nothing about the proof changes across a placement. A chain the EXACT arm
// admitted publishes no value gate 3 computed: the new face's plane is derived
// fresh, every evaluation, from the chain's own (placed) oriented geometry. A
// chain the LEVEL arm admitted instead publishes ITS OWN token, below, which
// the face build reads for the plane's origin and normal directly — never a
// value fitted to held vertex coordinates, which are only approximately
// coplanar for a chain admitted this way (denotation.go's own doc comment
// states why).
type Chain struct {
	Edges []*topo.Edge
	// AxialBound and HasAxialBound are the proven axial displacement gate 3's
	// LEVEL arm admitted this chain under — stamped onto the new face's own
	// AxialDelta exactly as internal/prismrecord/build.go stamps a prism cap's
	// (docs/surface-design.md §5.2). HasAxialBound stays false, and AxialBound
	// zero, whenever gate 3 instead admitted the chain through its existing
	// exact (zero-bound) arm.
	AxialBound    float64
	HasAxialBound bool
	// Level is the token the LEVEL arm admitted this chain under — the zero
	// value whenever HasAxialBound is false. The face build reads its own
	// origin/normal, transformed by this evaluation's own placement, as the
	// published plane: never the fitted vector, which this token's own doc
	// comment (denotation.go) states is unsound for a chain admitted by
	// identity rather than by proven-exact coplanarity.
	Level denotation.LevelToken
}

// BodyPayload is PatchEdges's own record: the retiring receiver's own faces,
// its already-proven Bounds, and each chain's proven plane — never a built
// topology (unstitch.go's "held-B-rep-under-a-rigid-motion" idea, widened here
// from one face to a whole face set plus the faces the patch mints on top of
// it). Placed replays BuildBody from these same values under the composed
// transform, exactly as the unstitch payload replays its own face build.
type BodyPayload struct {
	Xform r3.Transform
	// Delta is the proven displacement proofbound.RigidRoundAllow charges
	// against this body's placed geometry — zero exactly when Xform is the
	// identity transform, an exact struct comparison (the unstitch payload's
	// own delta).
	Delta float64
	// Faces is the ORIGINAL (now retired) receiver's own face set: read-only,
	// and never mutated or aliased into a live body's topology (the copy
	// always mints a fresh one from it).
	Faces []*topo.Face
	// Bounds is the retiring receiver's own proven, UNPLACED Bounds — reused
	// rather than re-derived, on the same reasoning the unstitch bounds state:
	// every new face's boundary is already receiver geometry, and a bounded
	// planar region lies inside its own boundary's box, so the receiver's own
	// box already bounds the patched result too.
	Bounds measurement.Box
	Chains []Chain
}

// Transform is the accumulated rigid placement.
func (pp BodyPayload) Transform() r3.Transform { return pp.Xform }

// Placed replays BuildBody from the recorded faces, bounds and chains under
// the composed motion.
func (pp BodyPayload) Placed(ctx context.Context, env topo.Env, composed r3.Transform) (*topo.Body, error) {
	return BuildBody(ctx, env, pp.Faces, pp.Bounds, pp.Chains, composed)
}

// SurveyClass names the payload class a staged survey diagnostic reports
// (internal/reportvocab): the root type name this payload had before it moved.
func (Payload) SurveyClass() string { return "patchPayload" }

// ProvesSimple implements topo.SimpleProver. Rule P (docs/surface-design.md
// §6.4) is the construction-proof gate a BodyPayload earns from its own
// receiver, letting Rule S admit the "walls, then cap, then stitch" flow §6.1
// motivates. PatchEdges itself proves only its own chains simple in their own
// plane (gate 4, §5.2), never the whole assembled boundary's
// non-self-intersection, but a BodyPayload proves its own boundary does not
// self-intersect when all three hold:
//
//  1. its receiver — the body PatchEdges was called on — itself admits
//     under Rule S, decided by the identical topo.ProvesSimple predicate,
//     never a reimplementation of it;
//  2. every new face's chain is a COMPLETE free-edge chain of that
//     receiver's own end, not a proper subset of one;
//  3. every new face's plane is exactly one of the receiver feature's own
//     end planes.
//
// Under those three the patched assembly IS the receiver feature's own
// solid boundary — every chain vertex and edge PatchEdges selected is the
// SAME object the receiver's own build stamped, never a copy or a fit — so
// the receiver's own construction proof carries onto the patched body
// unchanged. Any chain this cannot decide keeps the payload undecided,
// reject-only exactly as every other Rule S arm.
//
// Conditions 2 and 3 are both decided through the LEVEL half of the
// shared-denotation certificate, never by a coordinate or a residual:
// prismrecord.Build stamps every rim vertex and edge at one end with the SAME
// level token, minted once per build per end whenever the build's own section
// is drawn straight from its record (SectionDelta == 0) — regardless of
// whether that end's own coordinate ends up zero-bound or not.
// chainSharedLevel reads that identity off the chain's own edges AND vertices
// directly: a shared non-zero id proves the chain's plane IS that recorded
// level's plane, on the same terms §5.2's own gate 3 level arm already reads
// the identical token for, regardless of which of gate 3's two arms actually
// admitted the chain's planarity. A chain with no shared id — the exact-arm
// case, a chain from any receiver whose build mints no level token at all, or
// a chain a SECOND PatchEdges call selects from an already-patched body
// (copyFacesUnder does not propagate a level token onto the copies it mints) —
// is undecided here, never guessed: Rule P names no distance threshold that
// could stand in for the identity check CLAUDE.md's reject-only rule requires.
//
// Completeness (condition 2) then asks whether that SAME id's full set of
// the receiver's own free edges is exactly the chain's own edge set: a
// receiver whose end holds more than one disjoint free-edge loop — an
// annular profile's inner and outer rims at one level — proves nothing
// about the WHOLE end's non-self-intersection from patching only one of
// them, so admitting on a proper subset would be unsound.
//
// Only prismrecord.Payload mints level tokens today, so Rule P admits a
// PatchEdges-capped surface-extruded tube's rims and nothing wider yet.
func (pp BodyPayload) ProvesSimple(ctx context.Context) bool {
	bodies := topobuild.OperandBodies(pp.Faces)
	if len(bodies) != 1 || bodies[0] == nil || bodies[0].Payload == nil {
		return false
	}
	receiver := bodies[0]
	if !topo.ProvesSimple(ctx, receiver.Payload) {
		return false
	}

	// Every receiver free edge, grouped by its own level id — the candidate
	// set condition 2 checks each chain against. An edge with no
	// certificate (id == 0) joins no group: it cannot be one of the
	// receiver's own end planes, and grouping by "no certificate" would
	// prove nothing about which plane it belongs to.
	receiverFreeByLevel := map[denotation.LevelID][]*topo.Edge{}
	for _, e := range topobuild.Edges(receiver) {
		if len(e.Faces) != 1 || e.Level.ID == 0 {
			continue
		}
		receiverFreeByLevel[e.Level.ID] = append(receiverFreeByLevel[e.Level.ID], e)
	}

	for _, chain := range pp.Chains {
		id, ok := chainSharedLevel(chain.Edges)
		if !ok {
			return false
		}
		if !edgeSetsEqual(receiverFreeByLevel[id], chain.Edges) {
			return false
		}
	}
	return true
}

// chainSharedLevel is Rule P condition 3's own decision: the single non-zero
// level id every edge AND vertex of edges carries, or false when any of them
// carries a different id, the zero id ("no certificate"), or edges is empty.
// Reading id alone, never origin or normal, is what keeps this an identity
// comparison rather than a fitted or residual one. Checking every VERTEX as
// well as every edge is load-bearing: an edge's own id says its CURVE was
// stamped at one level, never that both its endpoints were too.
func chainSharedLevel(edges []*topo.Edge) (denotation.LevelID, bool) {
	if len(edges) == 0 {
		return 0, false
	}
	id := edges[0].Level.ID
	if id == 0 {
		return 0, false
	}
	for _, e := range edges {
		if e.Level.ID != id || e.Start.Level.ID != id || e.End.Level.ID != id {
			return 0, false
		}
	}
	return id, true
}

// edgeSetsEqual reports whether a and b hold the same edge pointers, order
// and duplicates aside — Rule P condition 2's own set-equality decision,
// never a count alone or a residual.
func edgeSetsEqual(a, b []*topo.Edge) bool {
	if len(a) != len(b) {
		return false
	}
	set := make(map[*topo.Edge]struct{}, len(a))
	for _, e := range a {
		set[e] = struct{}{}
	}
	for _, e := range b {
		if _, ok := set[e]; !ok {
			return false
		}
	}
	return true
}

// SurveyClass names the payload class a staged survey diagnostic reports
// (internal/reportvocab): the root type name this payload had before it moved.
func (BodyPayload) SurveyClass() string { return "bodyPatchPayload" }
