package decad

import (
	"context"
	"errors"
	"fmt"
	"math/big"
	"strings"

	"github.com/lestrrat-3d/decad/internal/loftdirect"
	"github.com/lestrrat-go/option/v3"

	"github.com/lestrrat-3d/decad/internal/booleandispatch"
	"github.com/lestrrat-3d/decad/internal/booloperand"
	"github.com/lestrrat-3d/decad/internal/clearance/pairkernel"
	"github.com/lestrrat-3d/decad/internal/contactfaceted"
	"github.com/lestrrat-3d/decad/internal/facetproof"
	"github.com/lestrrat-3d/decad/internal/massdispatch"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/prooflineage"
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

// Union returns the body enclosing the volume of two or more operands,
// retiring them together from their document (core §8). A nil operand or a
// repeated body is ErrDegenerate; a call with no other body is ErrDegenerate.
// Three or more bodies admit a one-scene exact union of equal-interval,
// same-placement, hole-free whole-line prisms with distinct source line
// carriers, plus certified straight bevel tooth patterns. Other
// prism inputs use the two-body evaluator in order, with one final commit.
// Each pair must pass its own proof. Failure or cancellation leaves every
// input live. A two-body result from the mesh path is a Faceted body:
// its faces are grouped by the operands' source faces, so provenance
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
// an angle whose sine is at most 2⁻³² (agent-docs/draft-design.md §9.1.1). The
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
// (agent-docs/boolean-chain-regroup-design.md); only a failure of that retry too
// returns the refusal.
//
// A Union of three or more bodies that no certified route takes as a set joins
// the bodies two at a time, left to right, after trying the whole chain on copies
// (agent-docs/union-fold-design.md): a chain whose step refuses returns the
// many-body refusal and leaves every body live.
//
// It returns ctx.Err() unchanged when ctx is canceled before the document
// commit.
func Union(ctx context.Context, first *Body, others ...*Body) (*Body, error) {
	if ctx == nil {
		return nil, fmt.Errorf("%w: a nil context cannot control a boolean", ErrDegenerate)
	}
	if len(others) == 0 {
		return nil, fmt.Errorf("%w: Union needs at least two bodies", ErrDegenerate)
	}
	if len(others) == 1 {
		body, err := performBoolean(ctx, meshbool.OpUnion, first, others[0])
		return body, operationError(OperationUnion, err)
	}
	bodies := make([]*Body, 0, len(others)+1)
	bodies = append(bodies, first)
	bodies = append(bodies, others...)
	body, err := performUnionMany(ctx, bodies)
	if errors.Is(err, ErrUnsupported) && isManyUnionRefusal(err) {
		// No certified route takes the set as a whole: join the bodies two at a time,
		// after trying the whole chain on copies (agent-docs/union-fold-design.md).
		if folded, ferr := unionFold(ctx, bodies); ferr == nil {
			return folded, nil
		}
	}
	return body, operationError(OperationUnion, err)
}

// Cut returns target minus tool, retiring both operands from their document
// (core §8). The target and tool roles are asymmetric. A cut that removes
// everything is ErrBooleanFailed;
// the other gates match Union's.
// A call refused with ErrUnsupported on a mesh result of earlier Cut and
// Intersect calls is retried from the source bodies of that chain
// (agent-docs/boolean-chain-regroup-design.md); only a failure of that retry too
// returns the refusal.
// It returns ctx.Err() unchanged when ctx is canceled before the document commit.
func Cut(ctx context.Context, target, tool *Body, opts ...BooleanOption) (*Body, error) {
	body, err := performBoolean(ctx, meshbool.OpCut, target, tool, opts...)
	return body, operationError(OperationCut, err)
}

// Intersect returns the volume common to a and b, retiring both operands
// from their document (core §8). Disjoint operands share nothing, so the
// empty result is ErrBooleanFailed; the other gates match Union's.
// A call refused with ErrUnsupported on a mesh result of earlier Cut and
// Intersect calls is retried from the source bodies of that chain
// (agent-docs/boolean-chain-regroup-design.md); only a failure of that retry too
// returns the refusal.
// It returns ctx.Err() unchanged when ctx is canceled before the document commit.
func Intersect(ctx context.Context, a, b *Body, opts ...BooleanOption) (*Body, error) {
	body, err := performBoolean(ctx, meshbool.OpIntersect, a, b, opts...)
	return body, operationError(OperationIntersect, err)
}

// BooleanOption configures Cut and Intersect. An option from another package
// cannot implement this sealed tier.
type BooleanOption interface {
	option.Interface
	booleanOption()
}

type booleanOption struct{ option.Interface }

func (booleanOption) booleanOption() {}

type identMatchingFaces struct{}

// WithMatchingFaces tells Cut or Intersect that the operands' faces are meant
// to match where they meet: a cap drawn to touch the tool's cone, say, or two
// faces meant to coincide. Faces drawn in floating point seldom match exactly;
// each rounds on its own, so a design that matches on paper can miss by a few
// last-place steps and be refused for it. With this option a miss of that size
// is taken as the match the design intends, and its effect is charged into
// the result's error bounds. A miss larger than rounding disproves the
// declaration, and the operation refuses as it would without the option. The
// option never admits a shape that would otherwise be refused for another
// reason, and it has no effect on a pair whose faces do not meet.
func WithMatchingFaces() BooleanOption {
	return booleanOption{option.New(identMatchingFaces{}, true)}
}

// booleanConfig is the folded form of a call's BooleanOptions.
type booleanConfig struct{ matchingFaces bool }

func foldBooleanOptions(opts []BooleanOption) (booleanConfig, error) {
	var cfg booleanConfig
	for _, opt := range opts {
		if opt == nil {
			return cfg, fmt.Errorf(`%w: a nil option names nothing to apply`, ErrDegenerate)
		}
		if o, ok := opt.(booleanOption); ok && o.Interface == nil {
			return cfg, fmt.Errorf(`%w: a zero option names nothing to apply`, ErrDegenerate)
		}
		switch opt.Ident().(type) {
		case identMatchingFaces:
			cfg.matchingFaces = true
		default:
			return cfg, fmt.Errorf(`%w: the boolean option is not a decad boolean option (%T)`, ErrDegenerate, opt)
		}
	}
	return cfg, nil
}

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
func performBoolean(ctx context.Context, op meshbool.OperationKind, a, b *Body, opts ...BooleanOption) (*Body, error) {
	if ctx == nil {
		return nil, fmt.Errorf(`%w: a nil context cannot control a boolean`, ErrDegenerate)
	}
	cfg, err := foldBooleanOptions(opts)
	if err != nil {
		return nil, err
	}
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
	if body, ok := orthogonalBoolean(ctx, d, op, a, b); ok {
		kind := operationKindForBoolean(op)
		if err := d.commitOperationIn(ctx, nil, kind, operationInputsForBoolean(op, a, b), body, a, b); err != nil {
			return nil, err
		}
		return body, nil
	}
	body, err := booleanBodyWith(ctx, op, a, b, d.nextProducerID(), cfg)
	span := producerID(1)
	if err != nil {
		rescued, used, ok, rerr := regroupBoolean(ctx, d, op, a, b, cfg, err)
		if rerr != nil {
			return nil, rerr
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
		first.Role, second.Role = "target", "tool"
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

func containedCutQuantities(ctx context.Context, result, target, tool *Body) error {
	if _, ok := result.n.Payload.(facetedPayload); !ok {
		return nil
	}
	proof, err := pairkernel.Pair(ctx, &target.n, &tool.n, false, nil)
	if err != nil {
		return err
	}
	certificate := proof.Containment
	if certificate == nil || certificate.Inner != &tool.n || certificate.Outer != &target.n {
		return nil
	}
	volumeLineage, selectVolume, err := prooflineage.ContainedCutVolume(
		int(target.n.Origin.Producer), int(tool.n.Origin.Producer),
		target.n.Volume, tool.n.Volume, result.n.Volume,
		prooflineage.Containment{
			BoundaryGapLower:    units.Millimeters(certificate.BoundaryGapLower),
			InnerShellWitnesses: certificate.InnerShellWitnesses,
			OuterShellWitnesses: certificate.OuterShellWitnesses,
		},
	)
	if err != nil {
		return newBooleanError(meshbool.OpCut, BooleanEvaluatorFailure, ErrBooleanFailed, err)
	}
	centroidLineage, selectCentroid, err := prooflineage.ContainedCutCentroid(
		volumeLineage,
		prooflineage.SourceCentroid{Producer: int(target.n.Origin.Producer), Reading: target.n.Centroid},
		prooflineage.SourceCentroid{Producer: int(tool.n.Origin.Producer), Reading: tool.n.Centroid},
		result.n.Centroid,
	)
	if err != nil {
		return newBooleanError(meshbool.OpCut, BooleanEvaluatorFailure, ErrBooleanFailed, err)
	}
	momentLineage, err := massdispatch.CaptureContainedCutMoments(ctx, &result.n, &target.n, &tool.n, volumeLineage)
	if err != nil {
		return newBooleanError(meshbool.OpCut, BooleanEvaluatorFailure, ErrBooleanFailed, err)
	}
	if err := ctx.Err(); err != nil {
		return err
	}
	volumeLineage.Centroid = centroidLineage
	volumeLineage.Moments = momentLineage
	result.n.VolumeLineage = volumeLineage
	if selectVolume {
		result.n.Volume = volumeLineage.Derived
	}
	if selectCentroid {
		result.n.Centroid = centroidLineage.Derived
	}
	return nil
}

// booleanBody builds a boolean's result body under producer identity ref
// without touching the document: no operand is retired and nothing is
// registered. performBoolean commits what it returns; Patterned's Union
// fallback (pattern.go) chains it over instances not yet registered and
// commits only the last result. a and b MUST belong to one document.
func booleanBody(ctx context.Context, op meshbool.OperationKind, a, b *Body, ref producerID) (*Body, error) {
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
	hooks := booleanHooks(cfg)
	return bodyResult(booleandispatch.Body(ctx, op, envOf(documentNode(d), ref), &a.n, &b.n, hooks))
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
type pairMeshes struct{}

func (pairMeshes) TessellateOperand(ctx context.Context, b *topo.Body, pairTol float64) (*topo.Mesh, bool, error) {
	m, restating, err := tessellateBooleanOperand(ctx, bodyHandle(b), pairTol)
	if err != nil {
		return nil, false, err
	}
	return &m.n, restating, nil
}
