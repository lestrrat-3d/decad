package decad

import (
	"context"
	"errors"
	"fmt"
	"math/big"
	"slices"
	"strings"

	"github.com/lestrrat-3d/decad/internal/booleandispatch"
	"github.com/lestrrat-3d/decad/internal/booleaneval"
	"github.com/lestrrat-3d/decad/internal/booloperand"
	"github.com/lestrrat-3d/decad/internal/contactfaceted"
	"github.com/lestrrat-3d/decad/internal/facetproof"
	"github.com/lestrrat-3d/decad/internal/loftdirect"
	"github.com/lestrrat-3d/decad/internal/massdispatch"
	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/opcommand"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/units"
)

// This file is the public boolean surface of core §8. Admitted prism and
// common-axis Revolve pairs use their source records; other pairs use the
// exact-predicate mesh boolean of agent-docs/evaluator-design.md §9. The mesh path
// tessellates both operands at an internal chord tolerance derived from the
// pair's diameter and produces a Faceted body with proven measurement bounds.
// The signatures follow core §3 invariant #4: no operand is
// mutated, no target-out parameter exists — each call retires its operands
// from their document and registers the result, reached through the operands'
// own owning document.

// Union returns the command that builds the body enclosing the volume of two
// or more operands, retiring them together from their document on Do (core §8).
// A nil operand or a repeated body is ErrDegenerate; fewer than two bodies is
// ErrDegenerate, returned by Do.
// Three or more bodies admit a one-scene exact union of equal-interval,
// same-placement, hole-free whole-line prisms with distinct source line
// carriers, plus certified straight bevel tooth patterns. Other
// prism inputs use the two-body evaluator in order, with one final commit.
// A three-source chain whose second pair refuses on its first operand's held
// bound can try the other grouping (prism-boolean §4.7). A two-body Union
// refused because a facet it touches holds an earlier rim's bound is rebuilt
// from its operands' recorded Union history with rim splitting on, as under
// WithRimSplit, and then joined (prism-boolean Part R, "Rim-split rebuild").
// Each pair must pass its own proof. Failure or cancellation leaves every
// input live. A two-body result from the mesh path is a Faceted body:
// its faces are grouped by the operands' source faces, with adjacent exactly
// coplanar same-facing faces of the two operands merged into one face
// (agent-docs/union-face-merge-design.md), so provenance
// (FaceCreatedBy) survives, and its measurements are Approximate with proven
// bounds (agent-docs/evaluator-design.md §9) — except that an all-planar pair whose
// contact points round exactly keeps an Exact VOLUME (the volume integral is
// computed in exact arithmetic); the surface area always carries at least an
// ulp-scale float-summation bound, so it reads Approximate with a bound tiny
// against any real tolerance. An admitted co-directional, coplanar analytic
// prism pair instead returns an analytic prism. Its faces have fresh roles under
// the Union step, including CapStart and CapEnd, and do not retain the operands'
// origins. A pair of full Revolves about one authenticated axis can instead
// return an analytic Revolve with the source-face origins of its kept walls.
// Two exact rectangular prisms with perpendicular sweeps can also join across
// a positive-area planar face patch as one analytic body. Operands from
// different documents are ErrForeignBody; retired operands ErrRetiredBody. A boolean
// that fails on the geometry returns a typed [BooleanError] wrapping the §12
// sentinel: a valid but unclassifiable contact — a curved-surface tangency or
// near-contact whose chord facets never provably interpenetrate, an exact
// coplanar / face-on-face overlap, a grazing edge, an isolated-point pinch, or
// an analytic prism-arrangement refusal wrapping ErrUnsupported —
// is BooleanUnsupportedContact, never ErrDegenerate;
// a result with no volume is BooleanEmpty (wrapping ErrBooleanFailed); an
// internal invariant break is BooleanEvaluatorFailure (wrapping
// ErrBooleanFailed). errors.As(err, &be) reads the Code.
// Every pair outside those analytic paths tessellates both operands at a chord
// tolerance derived from the pair's diameter, so an operand no boolean may
// consume — a cap-loop chamfer body whose band has a corner this evaluator
// cannot prove a line-line miter or an exactly tangent join, or a reflex
// corner, so its mesh carries no proof of the volume it and the body it stands
// for differ by (agent-docs/tessellation-reach-design.md §7) — surfaces a plain
// ErrUnsupported before any contact is examined: a capability limit, not a
// BooleanError. So does an operand restating a held mesh (a Faceted boolean
// result or a mitred sweep) whose facets where the pair meets carry bounds
// coarser than that pair tolerance, refused after the contacts are classified
// and before any facet is cut. A cap-loop chamfer whose
// every band is a whole turn or joins only line-line miters and exactly tangent
// corners — a filleted plate with drilled holes among them — is an ordinary
// operand, as is a tapered extrude whose line–arc and arc–arc joins turn by
// an angle whose sine is at most 2⁻³² (agent-docs/modify-design.md Part D §9.1.1). The
// matching axial loft's held-polygon cap band also has its own
// occupied-volume proof for reflex corners (loft §18). A valid operand whose
// boolean OUTPUT
// cannot be chorded finely enough to tessellate surfaces the retryable
// coarse-chording ErrDegenerate on that operand — a finer chord tolerance may
// clear it — not a BooleanError.
//
// The held-mesh half of that limit is what bounds how far booleans CHAIN, and
// it binds LOCALLY (agent-docs/api-design.md §8 "The chain depth"): a result's bound
// is composed per vertex from the operation that made it, and feeding that
// result back in as an operand refuses exactly when a facet the new pair
// meets, or comes within the tangency gate's slack of, carries a bound above
// the chord tolerance that pair derives from its own diameter. Geometry the
// pair does not touch never causes the refusal, so a result whose Bound
// exceeds that tolerance in untouched regions is an ordinary operand and
// chains. There is no caller-side tolerance to raise — §9 keeps the booleans
// free of a tolerance parameter on purpose — so where the comparison does
// refuse, it is geometry rather than an argument the caller got wrong. The
// bounds are readable, not only printed in that refusal: each [Vertex], edge
// and face of a Faceted body reports its own, and [Body.Tessellate] at any
// tolerance the faceted body already meets returns a [Mesh] whose Bound is the
// largest of them.
//
// A two-body call refused with ErrUnsupported on a mesh result of earlier Union
// calls is retried from the source bodies of that chain
// (agent-docs/prism-boolean-design.md Part R); only a failure of that retry too
// returns the refusal.
//
// A Union of three or more bodies that no certified route takes as a set joins
// the bodies two at a time, left to right, after trying the whole chain on copies
// (agent-docs/prism-boolean-design.md Part U): a chain whose step refuses returns the
// many-body refusal and leaves every body live.
//
// Do returns ctx.Err() unchanged when ctx is canceled before the document
// commit.
func Union(bodies ...*Body) *UnionCommand {
	c := &UnionCommand{bodies: slices.Clone(bodies)}
	if len(bodies) < 2 {
		c.state.Record("bodies", fmt.Errorf("%w: Union needs at least two bodies", ErrDegenerate))
	}
	return c
}

// UnionCommand joins bodies. Build it with [Union].
type UnionCommand struct {
	state    opcommand.State // consumed flag and first recorded fault
	bodies   []*Body         // copied at Union
	refine   bool            // set by RefineTo
	bound    float64         // the RefineTo bound in millimetres
	boundErr error           // the fault of the latest RefineTo bound, refused at Do
}

// RefineTo builds the union directly at the refinement bound maxBound, in one
// build (agent-docs/body-refine-design.md §7.1). It returns the join that
// [Union] followed by [Refine] returns.
//
// The bodies MUST hold a revolved blank and three or more copies of one
// point-Loft tooth — a straight-bevel sector join, as [Union] takes a bevel
// gear join. The join is built once at the bound min(maxBound, 0.1 mm). Union
// builds and audits the join at 0.1 mm, and Refine then discards that join and
// builds it again from the same sources at maxBound, so the single build saves
// the first. The result has the vertices, triangles, topology, bounds, Volume
// and Area that the pair returns, bit for bit. It differs from the pair's
// result in these ways. Its faces keep the producer and face identities the
// Union gives, where Refine makes fresh ones. The document's operation history
// holds one Union entry and no Refine entry, so the producer identities of
// later operations are one lower than after the pair. LocatePoint, a two-body
// Section and the regroup of a later Cut or Union read the result as a Union of
// its operands through that history, where they read the pair's refined body
// as a leaf. A join that cannot meet maxBound is refused and commits nothing,
// where the pair keeps the 0.1 mm join of its Union.
//
// Do returns the joined body, or an error with nothing committed: the document
// is unchanged and every operand stays live. A set that is not a sector join
// the direct build takes is ErrUnsupported, refused before anything is built,
// and its message says to call Union, then Refine. That covers a pair of
// bodies, a union of any other kind, and a join that Union folds two bodies at
// a time. A join that cannot meet maxBound is ErrUnsupported too, with the same
// advice. Union's own gates on the operands apply. Every error carries
// [OperationUnion].
//
// maxBound is a length: wrong kind is ErrUnitKind, non-finite ErrNotFinite,
// negative ErrNegativeMagnitude and zero ErrDegenerate, each returned by Do
// before anything is built and naming RefineTo as its subject. A second call
// replaces the first.
func (c *UnionCommand) RefineTo(maxBound units.Value) *UnionCommand {
	c.refine = true
	c.bound, c.boundErr = refinementBound(maxBound)
	return c
}

// Do runs the union once and returns the joined body. A second Do returns
// [ErrConsumed].
func (c *UnionCommand) Do(ctx context.Context) (*Body, error) {
	return runCommand(ctx, &c.state, OperationUnion, c.run)
}

func (c *UnionCommand) run(ctx context.Context) (*Body, error) {
	if c.refine {
		if c.boundErr != nil {
			return nil, subjectError(OperationUnion, "RefineTo", c.boundErr)
		}
		return unionRefined(ctx, c.bodies, c.bound)
	}
	return union(ctx, c.bodies)
}

// union is Union's implementation; its errors carry no operation tag.
func union(ctx context.Context, bodies []*Body) (*Body, error) {
	if len(bodies) == 2 {
		return performBoolean(ctx, meshbool.OpUnion, bodies[0], bodies[1], booleanConfig{})
	}
	body, err := performUnionMany(ctx, bodies)
	if errors.Is(err, ErrUnsupported) && isManyUnionRefusal(err) {
		// No certified route takes the set as a whole: join the bodies two at a time,
		// after trying the whole chain on copies (agent-docs/prism-boolean-design.md Part U).
		if folded, ferr := unionFold(ctx, bodies); ferr == nil {
			return folded, nil
		}
	}
	return body, err
}

// Cut returns the command that subtracts tool from target, retiring both
// operands from their document on Do (core §8). The target and tool roles are
// asymmetric. A cut that removes everything is ErrBooleanFailed;
// the other gates match Union's.
// A call refused with ErrUnsupported on a mesh result of earlier Cut and
// Intersect calls is retried from the source bodies of that chain
// (agent-docs/prism-boolean-design.md Part R); only a failure of that retry too
// returns the refusal.
// Do returns ctx.Err() unchanged when ctx is canceled before the document commit.
func Cut(target, tool *Body) *CutCommand {
	return &CutCommand{target: target, tool: tool}
}

// CutCommand subtracts one body from another. Build it with [Cut].
type CutCommand struct {
	state  opcommand.State
	target *Body
	tool   *Body
	cfg    booleanConfig
}

// MatchingFaces tells the cut that the operands' faces are meant to match
// where they meet; see [IntersectCommand.MatchingFaces].
func (c *CutCommand) MatchingFaces() *CutCommand {
	c.cfg.matchingFaces = true
	return c
}

// Do runs the cut once and returns the result body. A second Do returns
// [ErrConsumed].
func (c *CutCommand) Do(ctx context.Context) (*Body, error) {
	return runCommand(ctx, &c.state, OperationCut, c.run)
}

func (c *CutCommand) run(ctx context.Context) (*Body, error) {
	return cut(ctx, c.target, c.tool, c.cfg)
}

func cut(ctx context.Context, target, tool *Body, cfg booleanConfig) (*Body, error) {
	return performBoolean(ctx, meshbool.OpCut, target, tool, cfg)
}

// Intersect returns the command that builds the volume common to a and b,
// retiring both operands from their document on Do (core §8). Disjoint
// operands share nothing, so the empty result is ErrBooleanFailed; the other
// gates match Union's.
// A call refused with ErrUnsupported on a mesh result of earlier Cut and
// Intersect calls is retried from the source bodies of that chain
// (agent-docs/prism-boolean-design.md Part R); only a failure of that retry too
// returns the refusal.
// Do returns ctx.Err() unchanged when ctx is canceled before the document commit.
func Intersect(a, b *Body) *IntersectCommand {
	return &IntersectCommand{a: a, b: b}
}

// IntersectCommand builds the common volume of two bodies. Build it with
// [Intersect].
type IntersectCommand struct {
	state opcommand.State
	a, b  *Body
	cfg   booleanConfig
}

// MatchingFaces tells the intersection that the operands' faces are meant to
// match where they meet: a cap drawn to touch the tool's cone, say, or two
// faces meant to coincide. Faces drawn in floating point seldom match exactly;
// each rounds on its own, so a design that matches on paper can miss by a few
// last-place steps and be refused for it. With this method a miss of that size
// is taken as the match the design intends, and its effect is charged into
// the result's error bounds. A miss larger than rounding disproves the
// declaration, and the operation refuses as it would without it. It never
// admits a shape that would otherwise be refused for another reason, and it has
// no effect on a pair whose faces do not meet.
func (c *IntersectCommand) MatchingFaces() *IntersectCommand {
	c.cfg.matchingFaces = true
	return c
}

// Do runs the intersection once and returns the result body. A second Do
// returns [ErrConsumed].
func (c *IntersectCommand) Do(ctx context.Context) (*Body, error) {
	return runCommand(ctx, &c.state, OperationIntersect, c.run)
}

func (c *IntersectCommand) run(ctx context.Context) (*Body, error) {
	return intersect(ctx, c.a, c.b, c.cfg)
}

func intersect(ctx context.Context, a, b *Body, cfg booleanConfig) (*Body, error) {
	return performBoolean(ctx, meshbool.OpIntersect, a, b, cfg)
}

// booleanConfig is the folded form of a call's methods.
type booleanConfig struct{ matchingFaces bool }

// tessellateBooleanOperand meshes one boolean operand at its own request
// tolerance: the pair tolerance, raised to a restating operand's held floor
// (booloperand.HeldFloor) so that the restatement never refuses
// (agent-docs/faceted-vertex-bounds-design.md §5). It reports whether the operand
// restates, which arms the local gate. It stays in the root because it
// returns a root *Mesh built by the root's tessellateContext.
func tessellateBooleanOperand(ctx context.Context, b *Body, tolMM float64) (*Mesh, bool, error) {
	floor, restating := booloperand.HeldFloor(&b.n)
	m, err := tessellateContext(ctx, b, units.Millimeters(max(tolMM, floor)), VerifyAll)
	return m, restating, err
}

func asExpectedBoolean(err error) (*meshbool.BooleanExpectedError, bool) {
	var expected *meshbool.BooleanExpectedError
	return expected, errors.As(err, &expected)
}

// booleanEvaluation is the geometry-only result shared by public booleans and
// interference verification. It contains no document reference or public model
// state and cannot make the transient result live.
type booleanEvaluation struct {
	payload   facetedPayload
	volume    Measurement
	volumeRat *big.Rat
	audit     *facetproof.MeshAudit
}

// performBoolean gates the operands, runs the read-only geometry evaluator,
// then builds and commits the public result atomically. A failure before the
// commit leaves the live-body set, and operands unchanged.
func performBoolean(ctx context.Context, op meshbool.OperationKind, a, b *Body, cfg booleanConfig) (*Body, error) {
	if a == nil || a.n.Doc == nil {
		return nil, fmt.Errorf(`%w: the first operand belongs to no document`, ErrDegenerate)
	}
	d := documentHandle(a.n.Doc)
	if err := d.requireLive(a); err != nil {
		return nil, err
	}
	if err := d.requireLive(b); err != nil {
		return nil, err
	}
	if err := d.requireSameContainer([]*Body{a, b}); err != nil {
		return nil, err
	}
	if a == b {
		return nil, fmt.Errorf(`%w: a boolean needs two distinct bodies`, ErrDegenerate)
	}
	if op == meshbool.OpCut && cheekCutCandidate(d, a) && directCheekLoft(b) {
		ctx = withNativeCutBounds(ctx, a, b)
	}
	if body, ok := orthogonalBoolean(ctx, d, op, a, b); ok {
		kind := operationKindForBoolean(op)
		if err := d.commitOperationIn(ctx, nil, kind, operationInputsForBoolean(op, a, b), body, a, b); err != nil {
			return nil, err
		}
		return body, nil
	}
	_, privateHistory := ctx.Value(operationHistorySuppressedKey{}).(*operationHistoryCapture)
	if op == meshbool.OpCut && !cfg.matchingFaces && !privateHistory {
		evalCtx, err := d.evalContext(ctx)
		if err != nil {
			return nil, err
		}
		budget := proofbound.NewReplayTrialBudget()
		candidate, admitted, err := groupBlindCutBodyWithBudget(evalCtx, a, b, d.producerAt(ctx), budget)
		if err != nil {
			return nil, err
		}
		if admitted {
			if err := commitBlindGroupCut(ctx, d, a, b, candidate, budget); err != nil {
				return nil, err
			}
			return candidate, nil
		}
	}
	body, err := booleanBodyWith(ctx, op, a, b, d.producerAt(ctx), cfg)
	span := producerID(1)
	var paired pairedCutAncestry
	pairedUsed := false
	if err != nil && coarseTargetCut(op, cfg, err) {
		var used bool
		var rerr error
		body, span, paired, used, rerr = pairedCutContinuation(ctx, d, a, b, cfg, err)
		if rerr != nil {
			return nil, rerr
		}
		pairedUsed = used
	}
	if err != nil && !pairedUsed {
		rescued, used, ok, rerr := regroupBoolean(ctx, d, op, a, b, cfg, err)
		if rerr != nil {
			return nil, rerr
		}
		if !ok {
			rescued, used, ok, rerr = rimSplitRebuild(ctx, d, op, a, b, cfg, err)
			if rerr != nil {
				return nil, rerr
			}
		}
		if !ok {
			return nil, err
		}
		body, span = rescued, used
	}
	if op == meshbool.OpCut {
		if err := containedCutQuantities(ctx, body, a, b); err != nil {
			return nil, err
		}
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	if pairedUsed {
		base := body.n.Origin.Producer - span + 1
		if commitErr := commitPairedCut(ctx, d, a, b, body, span, base, paired, err); commitErr != nil {
			return nil, commitErr
		}
		return body, nil
	}
	kind := operationKindForBoolean(op)
	if err := d.commitOperationSpanIn(ctx, nil, kind, operationInputsForBoolean(op, a, b), body, span, a, b); err != nil {
		return nil, err
	}
	return body, nil
}

func operationInputsForBoolean(op meshbool.OperationKind, a, b *Body) []OperationInput {
	first := OperationInput{Body: a, Use: InputConsumed}
	second := OperationInput{Body: b, Use: InputConsumed}
	switch op {
	case meshbool.OpCut:
		first.Role, second.Role = operationRoleTarget, operationRoleTool
	case meshbool.OpUnion:
		first.Role, second.Role = "first", "others"
	case meshbool.OpIntersect:
		first.Role, second.Role = "a", "b"
	default:
		first.Role, second.Role = "operand", "operand"
	}
	return []OperationInput{first, second}
}

func operationKindForBoolean(op meshbool.OperationKind) OperationKind {
	switch op {
	case meshbool.OpCut:
		return OperationCut
	case meshbool.OpUnion:
		return OperationUnion
	case meshbool.OpIntersect:
		return OperationIntersect
	default:
		return OperationUnknown
	}
}

// containedCutQuantities unwraps private Body.n fields for massdispatch's
// quantity capture and maps its derivation failures to public BooleanError.
func containedCutQuantities(ctx context.Context, result, target, tool *Body) error {
	err := massdispatch.CaptureContainedCutQuantities(ctx, &result.n, &target.n, &tool.n)
	if evidence, ok := errors.AsType[*massdispatch.ContainedCutEvidenceError](err); ok {
		return newBooleanError(meshbool.OpCut, BooleanEvaluatorFailure, ErrBooleanFailed, evidence.Unwrap())
	}
	return err
}

// booleanBody builds a boolean's result body under producer identity ref
// without touching the document: no operand is retired and nothing is
// registered. performBoolean commits what it returns; Patterned's Union
// fallback (pattern.go) chains it over instances not yet registered and
// commits only the last result. a and b MUST belong to one document.
//
// Two axis-square faceted solids, or one beside an extruded rectilinear body, combine exactly on a grid first
// (agent-docs/orthogonal-design.md), so a feature built from tool bodies reads exactly on a mesh solid.
func booleanBody(ctx context.Context, op meshbool.OperationKind, a, b *Body, ref producerID) (*Body, error) {
	if body, ok := orthogonalBooleanAt(ctx, documentHandle(a.n.Doc), op, a, b, ref); ok {
		return body, nil
	}
	return booleanBodyWith(ctx, op, a, b, ref, booleanConfig{})
}

// booleanHooks supplies the root-owned parts of booleandispatch for one call's options.
func booleanHooks(cfg booleanConfig) booleandispatch.Hooks {
	return booleandispatch.Hooks{
		MatchingFaces:    cfg.matchingFaces,
		Mesher:           pairMeshes{},
		Verified:         verifyAllMesher{},
		AsBooleanError:   asBooleanError,
		DirectShellUnion: loftdirect.DirectShellUnion,
		LowerSupport: func(ctx context.Context, result, first, second *topo.Body) error {
			return contactfaceted.CertifyUnionLowerSupport(ctx, result, first, second)
		},
	}
}

func booleanBodyWith(ctx context.Context, op meshbool.OperationKind, a, b *Body, ref producerID,
	cfg booleanConfig) (*Body, error) {
	d := documentHandle(a.n.Doc)
	ctx, err := d.evalContext(ctx)
	if err != nil {
		return nil, err
	}
	var ceiling bool
	hooks := booleanHooks(cfg)
	hooks.Mesher = pairMeshes{ceiling: &ceiling}
	result, err := bodyResult(booleandispatch.Body(ctx, op, envOf(documentNode(d), ref), &a.n, &b.n, hooks))
	if ctxErr := ctx.Err(); ctxErr != nil {
		return nil, ctxErr
	}
	if err == nil || !ceiling {
		return result, err
	}
	// A revolve operand ran into a work ceiling at the pair tolerance. The pair is
	// retried with both operands requested at 2, 4 and 8 times it, the first rung
	// that answers winning; each mesh publishes the chording it actually took,
	// so every bound is the coarser one. A pair no rung answers reports the
	// refusal the pair tolerance itself hit
	// (agent-docs/revolve-loft-design.md, Operand coarsening §2).
	first := err
	for rung := 1; rung <= booloperand.MaxOperandCoarsenings; rung++ {
		hooks.Mesher = pairMeshes{rung: rung}
		result, err = bodyResult(booleandispatch.Body(ctx, op, envOf(documentNode(d), ref), &a.n, &b.n, hooks))
		if ctxErr := ctx.Err(); ctxErr != nil {
			return nil, ctxErr
		}
		if err == nil {
			return result, nil
		}
	}
	return nil, first
}

// asBooleanError maps an evaluateBoolean or facetedrecord.BuildWithProof error to the
// public typed BooleanError (agent-docs/api-design.md §8 / H2). The private
// expected-outcome kinds decide the Code and the wrapped sentinel: an empty
// result is BooleanEmpty; a valid but unclassifiable coplanar / tangent /
// grazing / isolated-point contact, and an in-pipeline reach of the boolean
// geometry on operands that DID tessellate (a proximity refusal, a collapsed or
// welded-away facet, a trim amplification past the pair diameter), are
// BooleanUnsupportedContact — the model is real and the refusal is the
// evaluator's contact reach, so the wrapped sentinel is ErrUnsupported, never
// ErrDegenerate. A capability/staging limit of one operand — an operand no
// boolean may consume (a cap-loop chamfer body whose
// band has a mitered circular wall or a reflex corner, so its mesh proves no
// occupied volume), or a held-mesh operand whose facets where the pair meets
// are coarser than the pair tolerance — is NOT a contact refusal and passes
// through unwrapped as a plain ErrUnsupported, not a BooleanError. A
// coarse-chording tessellation refusal is a retryable ErrDegenerate on a valid
// operand and likewise passes through unwrapped. An ordinary ErrBooleanFailed
// with no expected-outcome tag is an internal invariant break:
// BooleanEvaluatorFailure. Anything else — a nil, self, or extent-less operand,
// a cancelled context — passes through unchanged.
func asBooleanError(op meshbool.OperationKind, err error) error {
	if expected, ok := asExpectedBoolean(err); ok {
		switch expected.Kind {
		case meshbool.BooleanExpectedEmpty:
			return newBooleanError(op, BooleanEmpty, ErrBooleanFailed, err)
		case meshbool.BooleanExpectedContact, meshbool.BooleanExpectedUnsupported:
			return newBooleanError(op, BooleanUnsupportedContact, ErrUnsupported, err)
		case meshbool.BooleanExpectedStaging, meshbool.BooleanExpectedVolumeProof, meshbool.BooleanExpectedCoarseTessellation:
			return err
		}
	}
	if errors.Is(err, ErrBooleanFailed) {
		return newBooleanError(op, BooleanEvaluatorFailure, ErrBooleanFailed, err)
	}
	return err
}

// newBooleanError builds a *BooleanError carrying the operation, branchable
// Code, and the human text of the underlying error (its
// "decad: " sentinel prefix trimmed so BooleanError.Error does not repeat it).
func newBooleanError(op meshbool.OperationKind, code BooleanErrorCode, sentinel, orig error) error {
	result := &BooleanError{
		op:   op,
		Code: code,
		msg:  strings.TrimPrefix(orig.Error(), "decad: "),
		err:  sentinel,
	}
	var expected *meshbool.BooleanExpectedError
	if errors.As(orig, &expected) && expected.AppliedShift != nil {
		result.shift = &OperationChange{
			Mechanism:         OperationChangeGapShift,
			DisplacementBound: units.Millimeters(expected.AppliedShift.DisplacementBound),
			AllowedReach:      units.Millimeters(expected.AppliedShift.AllowedReach),
		}
	}
	return result
}

// evaluateBoolean runs the complete geometry pipeline without writing the
// document. It deliberately does not gate liveness or mint a producerID: the
// public wrapper owns those actions, while Verify already walks live bodies.
// Both operands are meshed at the pair's own chord tolerance (pairMeshes).
func evaluateBoolean(ctx context.Context, op meshbool.OperationKind, a, b *Body) (booleanEvaluation, error) {
	return evaluateBooleanMeshes(ctx, op, a, b, pairMeshes{}, 1)
}

// pairMeshes meshes each operand at the pair's own chord tolerance, raised to
// a restating operand's held floor: the public booleans' rule
// (agent-docs/evaluator-design.md §9, tessellateBooleanOperand).
//
// A pair with a revolve operand that ran into a work ceiling is requested
// coarser on a later rung (booleanBodyWith): rung r doubles both operands'
// requests r times, so neither is far finer than its partner, and ceiling,
// when set, records that one such refusal happened.
type pairMeshes struct {
	rung    int
	ceiling *bool
}

func (p pairMeshes) TessellateOperand(ctx context.Context, b *topo.Body, pairTol float64) (*topo.Mesh, bool, error) {
	pairTol *= float64(int(1) << p.rung)
	m, restating, err := tessellateBooleanOperand(ctx, bodyHandle(b), pairTol)
	if err != nil {
		if p.ceiling != nil && booloperand.CoarsensOperand(b, err) {
			*p.ceiling = true
		}
		return nil, false, err
	}
	return &m.n, restating, nil
}

// evaluateBooleanMeshes runs the mesh-boolean pipeline
// (internal/booleaneval, agent-docs/evaluator-design.md §9) over a and b, with the
// operand meshes drawn from meshes, and states its occupied volume as a public
// measurement. Verify passes its per-call cache (verifyMeshCache,
// internal/verifyrun/pairs.go), which meshes each body once at one chord for every pair it
// takes part in (agent-docs/interference-design.md §5.3); chordScale lets one
// source-certified modify route ask for finer operand chords, and a
// power-of-two scale preserves the pair tolerance's represented value exactly.
func evaluateBooleanMeshes(ctx context.Context, op meshbool.OperationKind, a, b *Body,
	meshes booleaneval.OperandMesher, chordScale float64) (booleanEvaluation, error) {
	ev, err := booleaneval.Evaluate(ctx, op, bodyNode(a), bodyNode(b), meshes, chordScale, nil, nil)
	if err != nil {
		return booleanEvaluation{}, err
	}
	return booleanEvaluation{payload: ev.Payload, volume: measurement.CubicMillimeters(ev.Volume, ev.VolumeBound),
		volumeRat: ev.VolumeRat, audit: ev.Audit}, nil
}
