package decad

import (
	"context"
	"errors"
	"fmt"
	"math/big"
	"strings"

	"github.com/lestrrat-3d/decad/internal/facetproof"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/tessellation"

	"github.com/lestrrat-3d/decad/internal/freeform"

	"github.com/lestrrat-3d/decad/internal/proofbound"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// This file is the public boolean surface of core §8 over the exact-predicate
// mesh boolean of docs/evaluator-design.md §9: Union, Cut and Intersect
// tessellate both operands at an evaluator-internal chord tolerance derived
// from the pair's own diameter, intersect the meshes with adaptive-exact
// predicates, and produce a Faceted body whose measurements carry proven
// composed bounds. The signatures follow core §3 invariant #4: no operand is
// mutated, no target-out parameter exists — each call retires its operands
// from their document and registers the result, reached through the operands'
// own owning document.

// Union returns the body enclosing the volume of a or b, retiring both
// operands from their document (core §8). A nil operand or a body unioned with
// itself is ErrDegenerate. The general result is a Faceted body:
// its faces are grouped by the operands' source faces, so provenance
// (FaceCreatedBy) survives, and its measurements are Approximate with proven
// bounds (docs/evaluator-design.md §9) — except that an all-planar pair whose
// contact points round exactly keeps an Exact VOLUME (the volume integral is
// computed in exact arithmetic); the surface area always carries at least an
// ulp-scale float-summation bound, so it reads Approximate with a bound tiny
// against any real tolerance. An admitted co-directional, coplanar analytic
// prism pair instead returns an analytic prism. Its faces have fresh roles under
// the Union step, including CapStart and CapEnd, and do not retain the operands'
// origins. Operands from different
// documents are ErrForeignBody; retired operands ErrRetiredBody. A boolean
// that fails on the geometry returns a typed [BooleanError] wrapping the §12
// sentinel: a valid but unclassifiable contact — a curved-surface tangency or
// near-contact whose chord facets never provably interpenetrate, an exact
// coplanar / face-on-face overlap, a grazing edge, an isolated-point pinch, or
// an analytic prism-arrangement refusal wrapping ErrUnsupported —
// is BooleanUnsupportedContact, never ErrDegenerate;
// a result with no volume is BooleanEmpty (wrapping ErrBooleanFailed); an
// internal invariant break is BooleanEvaluatorFailure (wrapping
// ErrBooleanFailed). errors.As(err, &be) reads the Code.
// Every pair outside that analytic path tessellates both operands at a chord
// tolerance derived from the pair's diameter, so an operand no boolean may
// consume — a cap-loop chamfer body whose band has a corner this evaluator
// cannot prove a line-line miter or an exactly tangent join, or a reflex
// corner, so its mesh carries no proof of the volume it and the body it stands
// for differ by (docs/tessellation-reach-design.md §7) — surfaces a plain
// ErrUnsupported before any contact is examined: a capability limit, not a
// BooleanError. So does an operand restating a held mesh (a Faceted boolean
// result or a mitred sweep) whose facets where the pair meets carry bounds
// coarser than that pair tolerance, refused after the contacts are classified
// and before any facet is cut. A cap-loop chamfer whose
// every band is a whole turn or joins only line-line miters and exactly tangent
// corners — a filleted plate with drilled holes among them — is an ordinary
// operand. The matching axial loft's held-polygon cap band also has its own
// occupied-volume proof for reflex corners (loft §18). A valid operand whose
// boolean OUTPUT
// cannot be chorded finely enough to tessellate surfaces the retryable
// coarse-chording ErrDegenerate on that operand — a finer chord tolerance may
// clear it — not a BooleanError.
//
// The held-mesh half of that limit is what bounds how far booleans CHAIN, and
// it binds LOCALLY (docs/api-design.md §8 "The chain depth"): a result's bound
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
// It returns ctx.Err() unchanged when ctx is canceled before the document
// commit.
func Union(ctx context.Context, a, b *Body) (*Body, error) {
	return performBoolean(ctx, meshbool.OpUnion, a, b)
}

// Cut returns target minus tool, retiring both operands from their document
// (core §8). The target and tool roles are asymmetric. A cut that removes
// everything is ErrBooleanFailed;
// the other gates match Union's. It returns ctx.Err() unchanged when ctx is
// canceled before the document commit.
func Cut(ctx context.Context, target, tool *Body) (*Body, error) {
	return performBoolean(ctx, meshbool.OpCut, target, tool)
}

// Intersect returns the volume common to a and b, retiring both operands
// from their document (core §8). Disjoint operands share nothing, so the
// empty result is ErrBooleanFailed; the other gates match Union's. It returns
// ctx.Err() unchanged when ctx is canceled before the document commit.
func Intersect(ctx context.Context, a, b *Body) (*Body, error) {
	return performBoolean(ctx, meshbool.OpIntersect, a, b)
}

// booleanOperandStaging restates the local chain-depth gate's refusal
// (meshbool.CoarseHeldContactError; docs/faceted-vertex-bounds-design.md §5)
// in the boolean's own terms (docs/api-design.md §8, "The chain depth"): it
// names the operand, quotes the bound of the held facets the pair touches and
// the pair's chord tolerance, and says that a boolean takes no tolerance.
// Every other error passes through unchanged.
func booleanOperandStaging(op meshbool.OperationKind, err error) error {
	var held *meshbool.CoarseHeldContactError
	if !errors.As(err, &held) {
		return err
	}
	return meshbool.ExpectedBooleanForOperand(meshbool.BooleanExpectedStaging, held.Operand,
		fmt.Errorf(`%w: %s's %s holds its mesh, where this pair meets it, only within a bound of %s, coarser than this pair's chord tolerance %s; a boolean takes no tolerance, so the chain cannot continue through that geometry: place the new contact away from the earlier booleans' rims, reshape the construction with fewer booleans over that region, or draw the pair so the analytic prism path admits it (docs/api-design.md §8, "The chain depth")`,
			ErrUnsupported, op, booleanOperandRole(op, held.Operand), units.Millimeters(held.Bound), units.Millimeters(held.Tol)))
}

// heldFloorOf is the bound below which a RESTATING operand cannot give a mesh
// (docs/faceted-vertex-bounds-design.md §5): a boolean result's meshBound, or
// a mitred sweep's or a coil's delta (docs/helix-design.md Table CD row CD3). Its restatement returns the same vertices at any
// tolerance at or above the floor. restating is false for every other
// payload, whose request stays the pair tolerance and whose facets the local
// gate does not read.
func heldFloorOf(b *Body) (floor float64, restating bool) {
	switch p := b.payload.(type) {
	case facetedPayload:
		return p.meshBound, true
	case mitredSweepPayload:
		return p.delta, true
	case coilPayload:
		return p.delta, true
	default:
		return 0, false
	}
}

// tessellateBooleanOperand meshes one boolean operand at its own request
// tolerance: the pair tolerance, raised to a restating operand's held floor
// so that the restatement never refuses (docs/faceted-vertex-bounds-design.md
// §5). It reports whether the operand restates, which arms the local gate.
func tessellateBooleanOperand(ctx context.Context, b *Body, tolMM float64) (*Mesh, bool, error) {
	floor, restating := heldFloorOf(b)
	m, err := tessellateContext(ctx, b, units.Millimeters(max(tolMM, floor)), VerifyAll)
	return m, restating, err
}

// booleanOperandRole names an operand the way the public signatures do: Cut's
// target and tool, otherwise first and second operand.
func booleanOperandRole(op meshbool.OperationKind, operand int) string {
	switch {
	case op == meshbool.OpCut && operand == 0:
		return "target"
	case op == meshbool.OpCut:
		return "tool"
	case operand == 0:
		return "first operand"
	default:
		return "second operand"
	}
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
func performBoolean(ctx context.Context, op meshbool.OperationKind, a, b *Body) (*Body, error) {
	if ctx == nil {
		return nil, fmt.Errorf(`%w: a nil context cannot control a boolean`, ErrDegenerate)
	}
	if a == nil || a.doc == nil {
		return nil, fmt.Errorf(`%w: the first operand belongs to no document`, ErrDegenerate)
	}
	d := a.doc
	if err := d.requireLive(a); err != nil {
		return nil, err
	}
	if err := d.requireLive(b); err != nil {
		return nil, err
	}
	if a == b {
		return nil, fmt.Errorf(`%w: a boolean needs two distinct bodies`, ErrDegenerate)
	}
	body, err := booleanBody(ctx, op, a, b, d.nextProducerID())
	if err != nil {
		return nil, err
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	d.commit(body, a, b)
	return body, nil
}

// booleanBody builds a boolean's result body under producer identity ref
// without touching the document: no operand is retired and nothing is
// registered. performBoolean commits what it returns; Patterned's Union
// fallback (pattern.go) chains it over instances not yet registered and
// commits only the last result. a and b MUST belong to one document.
func booleanBody(ctx context.Context, op meshbool.OperationKind, a, b *Body, ref producerID) (*Body, error) {
	d := a.doc
	// Table X (docs/surface-design.md §11): a sheet operand in either position
	// is a plain ErrUnsupported, never a BooleanError — without this a sheet
	// falls through to tryPrismBoolean below, which type-asserts prismPayload
	// and a sheet still carries one, producing a confidently-wrong solid.
	if err := refuseSheetOperand(a, op.String()); err != nil {
		return nil, err
	}
	if err := refuseSheetOperand(b, op.String()); err != nil {
		return nil, err
	}
	// docs/prism-boolean-design.md: a reject-only analytic reduction for a
	// co-directional coplanar prism pair, dispatched ahead of the mesh path
	// for Union's select-all/merge/chain path (§4.2) and Cut/Intersect's
	// clean-nesting structural match (§4.2). ok=false is never an error —
	// the pair falls back to the unchanged mesh path below exactly as it did
	// before this design existed. A non-nil err here is a genuine, typed
	// analytic-resolution refusal (§3.4) and must propagate rather than
	// reroute to the mesh path: an ErrUnsupported refusal becomes the public
	// BooleanUnsupportedContact below, preserving errors.Is(err,
	// ErrUnsupported); ErrDegenerate and ErrUnrecordableProfile pass through
	// unwrapped, keeping their own documented sentinels.
	if pp, ok, err := tryPrismBoolean(ctx, op, a, b); err != nil {
		if errors.Is(err, ErrUnsupported) {
			return nil, asBooleanError(op, meshbool.ExpectedBoolean(meshbool.BooleanExpectedUnsupported, err))
		}
		return nil, err
	} else if ok {
		body, err := evalPrismContext(ctx, d, ref, pp, freeform.NewFreeformWork())
		if err != nil {
			return nil, err
		}
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		return body, nil
	}
	if op == meshbool.OpUnion {
		// docs/general-boolean-design.md §3 A1: unequal sweep intervals that
		// overlap or touch build a stacked prism, or a brep where an
		// interface is flush or crossing. Same contract as above.
		if payload, ok, err := tryStackedUnion(ctx, a, b); err != nil {
			if errors.Is(err, ErrUnsupported) {
				return nil, asBooleanError(op, meshbool.ExpectedBoolean(meshbool.BooleanExpectedUnsupported, err))
			}
			return nil, err
		} else if ok {
			body, err := analyticBooleanBody(ctx, d, ref, payload)
			if err != nil {
				return nil, err
			}
			if err := ctx.Err(); err != nil {
				return nil, err
			}
			return body, nil
		}
		// docs/general-boolean-design.md §3 A5: a prism-group operand, or
		// survivors that close into several disjoint loops.
		if payload, ok, err := tryPrismGroupUnion(ctx, a, b); err != nil {
			if errors.Is(err, ErrUnsupported) {
				return nil, asBooleanError(op, meshbool.ExpectedBoolean(meshbool.BooleanExpectedUnsupported, err))
			}
			return nil, err
		} else if ok {
			return analyticBooleanBody(ctx, d, ref, payload)
		}
	}
	if op == meshbool.OpCut {
		if body, ok, err := tryLoftThroughBoreCut(ctx, d, ref, a, b); err != nil {
			return nil, err
		} else if ok {
			return body, nil
		}
		if sp, ok, err := tryStackedThroughCut(ctx, a, b); err != nil {
			if errors.Is(err, ErrUnsupported) {
				return nil, asBooleanError(op, meshbool.ExpectedBoolean(meshbool.BooleanExpectedUnsupported, err))
			}
			return nil, err
		} else if ok {
			body, err := evalStackedContext(ctx, d, ref, sp)
			if err != nil {
				return nil, err
			}
			if err := ctx.Err(); err != nil {
				return nil, err
			}
			return body, nil
		}
		if sp, ok, err := tryStackedBlindCut(ctx, a, b); err != nil {
			if errors.Is(err, ErrUnsupported) {
				return nil, asBooleanError(op, meshbool.ExpectedBoolean(meshbool.BooleanExpectedUnsupported, err))
			}
			return nil, err
		} else if ok {
			body, err := evalStackedContext(ctx, d, ref, sp)
			if err != nil {
				return nil, err
			}
			if err := ctx.Err(); err != nil {
				return nil, err
			}
			return body, nil
		}
		if sp, ok, err := tryBlindStackedCut(ctx, a, b); err != nil {
			if errors.Is(err, ErrUnsupported) {
				return nil, asBooleanError(op, meshbool.ExpectedBoolean(meshbool.BooleanExpectedUnsupported, err))
			}
			return nil, err
		} else if ok {
			body, err := evalStackedContext(ctx, d, ref, sp)
			if err != nil {
				return nil, err
			}
			if err := ctx.Err(); err != nil {
				return nil, err
			}
			return body, nil
		}
		// docs/general-boolean-design.md §3 A5: a prism-group tool.
		if pp, ok, err := tryPrismGroupCut(ctx, a, b); err != nil {
			if errors.Is(err, ErrUnsupported) {
				return nil, asBooleanError(op, meshbool.ExpectedBoolean(meshbool.BooleanExpectedUnsupported, err))
			}
			return nil, err
		} else if ok {
			return analyticBooleanBody(ctx, d, ref, pp)
		}
	}
	// docs/general-boolean-design.md §3 class B: a perpendicular pair with a
	// prism, stacked or brep operand builds a brep body (or, for Intersect, a
	// prism). Same contract as above.
	if payload, ok, err := tryClassB(ctx, op, a, b); err != nil {
		if errors.Is(err, ErrUnsupported) {
			return nil, asBooleanError(op, meshbool.ExpectedBoolean(meshbool.BooleanExpectedUnsupported, err))
		}
		return nil, err
	} else if ok {
		return analyticBooleanBody(ctx, d, ref, payload)
	}

	eval, err := evaluateBoolean(ctx, op, a, b)
	if err != nil {
		return nil, asBooleanError(op, err)
	}

	body, err := buildFacetedBodyWithProof(ctx, d, ref, eval.payload, eval.audit, eval.volume, eval.volumeRat)
	if err != nil {
		return nil, asBooleanError(op, err)
	}
	if op == meshbool.OpUnion {
		if err := certifyFacetedUnionLowerSupport(ctx, body, a, b); err != nil {
			return nil, err
		}
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	return body, nil
}

// evaluateAnalyticIntersect is measuredInterference's read-only twin of
// performBoolean's analytic dispatch (docs/prism-boolean-design.md §14 PR4;
// docs/interference-design.md §5.2, §12): it runs the same admission and
// resolution an meshbool.OpIntersect performBoolean call would — tryPrismBoolean, then
// evalPrismContext over the admitted payload — under a producerID it mints for
// itself, but it never calls Document.commit, so it writes nothing to the
// document and consumes neither operand (docs/interference-design.md §5:
// "MUST NOT call nextProducerID, append a Step, retire an operand, register a
// body, or expose a transient result through the document" governs
// evaluateBoolean's mesh path; this analytic twin keeps the same promise by
// simply never reaching a commit). ok=false (err always nil in that case)
// means the pair is not admitted by the analytic path (§3.1/§3.4): the
// caller MUST fall back to evaluateBoolean unchanged, exactly as
// tryPrismBoolean's own contract requires. A non-nil err is a genuine
// analytic-resolution refusal, returned as a *meshbool.BooleanExpectedError so the
// caller can share evaluateBoolean's own expected-outcome classification.
func evaluateAnalyticIntersect(ctx context.Context, a, b *Body) (*Body, bool, error) {
	pp, ok, err := tryPrismBoolean(ctx, meshbool.OpIntersect, a, b)
	if err != nil {
		if errors.Is(err, ErrUnsupported) {
			return nil, false, meshbool.ExpectedBoolean(meshbool.BooleanExpectedUnsupported, err)
		}
		return nil, false, err
	}
	if !ok {
		return nil, false, nil
	}
	d := a.doc
	body, err := evalPrismContext(ctx, d, d.nextProducerID(), pp, freeform.NewFreeformWork())
	if err != nil {
		return nil, false, err
	}
	return body, true, nil
}

// asBooleanError maps an evaluateBoolean or buildFacetedBody error to the
// public typed BooleanError (docs/api-design.md §8 / H2). The private
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
	return &BooleanError{
		op:   op,
		Code: code,
		msg:  strings.TrimPrefix(orig.Error(), "decad: "),
		err:  sentinel,
	}
}

// evaluateBoolean runs the complete geometry pipeline without writing the
// document. It deliberately does not gate liveness or mint a producerID: the
// public wrapper owns those actions, while Verify already walks live bodies.
// Both operands are meshed at the pair's own chord tolerance (pairMeshes).
func evaluateBoolean(ctx context.Context, op meshbool.OperationKind, a, b *Body) (booleanEvaluation, error) {
	return evaluateBooleanMeshes(ctx, op, a, b, pairMeshes{})
}

// operandMeshes supplies the VerifyAll mesh evaluateBooleanMeshes composes for
// one operand, and whether the operand restates a held mesh (heldFloorOf),
// which arms the local chain-depth gate at the pair tolerance. pairTol is the
// pair's own chord tolerance (pairChordTolerance). Every implementation
// returns a mesh tessellateContext built at VerifyAll, so the mesh carries its
// own proven bounds whatever tolerance it was asked for; the composition below
// reads only those bounds, never the request.
type operandMeshes interface {
	operandMesh(ctx context.Context, b *Body, pairTol float64) (*Mesh, bool, error)
}

// pairMeshes meshes each operand at the pair's own chord tolerance, raised to
// a restating operand's held floor: the public booleans' rule
// (docs/evaluator-design.md §9, tessellateBooleanOperand).
type pairMeshes struct{}

func (pairMeshes) operandMesh(ctx context.Context, b *Body, pairTol float64) (*Mesh, bool, error) {
	return tessellateBooleanOperand(ctx, b, pairTol)
}

// evaluateBooleanMeshes is evaluateBoolean with the operand meshes drawn from
// meshes. Verify passes its per-call cache (verifyMeshCache,
// verify_pairs.go), which meshes each body once at one chord for every pair
// it takes part in (docs/interference-design.md §5.3).
func evaluateBooleanMeshes(ctx context.Context, op meshbool.OperationKind, a, b *Body, meshes operandMeshes) (booleanEvaluation, error) {
	if err := ctx.Err(); err != nil {
		return booleanEvaluation{}, err
	}

	tolMM, dPair, err := pairChordTolerance(ctx, a, b)
	if err != nil {
		return booleanEvaluation{}, err
	}
	if err := ctx.Err(); err != nil {
		return booleanEvaluation{}, err
	}
	// A payload whose mesh cannot carry an occupied-volume proof is refused
	// BEFORE it is meshed. operandSymDiff below is still the gate that decides
	// it — this only asks the same question of the payload class, where the
	// answer is already known, so a refusal costs a map lookup instead of a
	// complete tessellation and its facet-pair audit.
	if err := requireVolumeProvingPayload(ctx, a, 0); err != nil {
		return booleanEvaluation{}, err
	}
	if err := requireVolumeProvingPayload(ctx, b, 1); err != nil {
		return booleanEvaluation{}, err
	}
	// VerifyAll, passed explicitly rather than taken from Tessellate's default
	// (docs/tessellation-design.md §11 step 1) by every operandMeshes: the
	// composition below reads every proof a mesh can carry, and the one-entry
	// cache keys on the level, so a caller's own earlier unverified mesh at
	// this same internal tolerance is a different entry and is never handed
	// back here.
	ma, restatingA, err := meshes.operandMesh(ctx, a, tolMM)
	if err != nil {
		var coarse *tessellation.ExpectedError
		if errors.As(err, &coarse) {
			err = meshbool.ExpectedBoolean(meshbool.BooleanExpectedCoarseTessellation, err)
		} else if errors.Is(err, ErrUnsupported) {
			// A tessellation ErrUnsupported is a capability/staging limit on the
			// operand itself, reached before any contact is examined — never a
			// contact refusal (see meshbool.BooleanExpectedStaging).
			err = meshbool.ExpectedBooleanForOperand(meshbool.BooleanExpectedStaging, 0, err)
		}
		return booleanEvaluation{}, err
	}
	if err := ctx.Err(); err != nil {
		return booleanEvaluation{}, err
	}
	mb, restatingB, err := meshes.operandMesh(ctx, b, tolMM)
	if err != nil {
		var coarse *tessellation.ExpectedError
		if errors.As(err, &coarse) {
			err = meshbool.ExpectedBoolean(meshbool.BooleanExpectedCoarseTessellation, err)
		} else if errors.Is(err, ErrUnsupported) {
			// A tessellation ErrUnsupported is a capability/staging limit on the
			// operand itself, reached before any contact is examined — never a
			// contact refusal (see meshbool.BooleanExpectedStaging).
			err = meshbool.ExpectedBooleanForOperand(meshbool.BooleanExpectedStaging, 1, err)
		}
		return booleanEvaluation{}, err
	}
	if err := ctx.Err(); err != nil {
		return booleanEvaluation{}, err
	}

	// Global source-face ids: the operands' faces in order, so the payload
	// records provenance without holding live pointers.
	var groups []facetGroup
	faceID := map[*Face]int{}
	for i, f := range append(a.Faces(), b.Faces()...) {
		if i%256 == 0 {
			if err := ctx.Err(); err != nil {
				return booleanEvaluation{}, err
			}
		}
		faceID[f] = len(groups)
		groups = append(groups, facetGroup{
			origins: f.Origins(),
			planar:  f.isPlanar(),
		})
	}
	srcA, err := sourceIDs(ctx, ma, faceID)
	if err != nil {
		return booleanEvaluation{}, err
	}
	srcB, err := sourceIDs(ctx, mb, faceID)
	if err != nil {
		return booleanEvaluation{}, err
	}
	if err := ctx.Err(); err != nil {
		return booleanEvaluation{}, err
	}

	bmA, err := prepBoolMeshContext(ctx, ma, srcA)
	if err != nil {
		if errors.Is(err, ErrUnsupported) {
			err = meshbool.ExpectedBoolean(meshbool.BooleanExpectedUnsupported, err)
		}
		return booleanEvaluation{}, err
	}
	bmB, err := prepBoolMeshContext(ctx, mb, srcB)
	if err != nil {
		if errors.Is(err, ErrUnsupported) {
			err = meshbool.ExpectedBoolean(meshbool.BooleanExpectedUnsupported, err)
		}
		return booleanEvaluation{}, err
	}
	// A restating operand's facets where the pair meets must hold within the
	// pair tolerance; the gate reads them on the contact classification below
	// (docs/faceted-vertex-bounds-design.md §5). Any other operand is ungated.
	if restatingA {
		bmA.Gate = tolMM
	}
	if restatingB {
		bmB.Gate = tolMM
	}
	if err := ctx.Err(); err != nil {
		return booleanEvaluation{}, err
	}
	// One memo per call, over exactly the two prepared operand tessellations
	// the gate and the mesh pass both walk; it is dropped when the call
	// returns (boolean_mesh.go, meshbool.ContactMemo).
	memo := meshbool.NewContactMemo(bmA, bmB)
	if err := refuseUndecidableProximity(ctx, ma, mb, bmA, bmB, memo); err != nil {
		err = booleanOperandStaging(op, err)
		if errors.Is(err, ErrUnsupported) {
			err = meshbool.ExpectedBoolean(meshbool.BooleanExpectedContact, err)
		}
		return booleanEvaluation{}, err
	}
	if err := ctx.Err(); err != nil {
		return booleanEvaluation{}, err
	}
	kept, maxRim, err := meshbool.MeshBoolean(ctx, op, bmA, bmB, memo)
	if err != nil {
		return booleanEvaluation{}, booleanOperandStaging(op, err)
	}
	if len(kept) == 0 {
		return booleanEvaluation{}, meshbool.ExpectedBoolean(meshbool.BooleanExpectedEmpty,
			fmt.Errorf(`%w: the %s result is empty`, ErrBooleanFailed, op))
	}
	if err := ctx.Err(); err != nil {
		return booleanEvaluation{}, err
	}
	stitched, err := meshbool.StitchFacetsContext(ctx, kept)
	if err != nil {
		if errors.Is(err, ErrUnsupported) {
			err = meshbool.ExpectedBoolean(meshbool.BooleanExpectedUnsupported, err)
		}
		return booleanEvaluation{}, err
	}
	if err := ctx.Err(); err != nil {
		return booleanEvaluation{}, err
	}

	// Bound composition (§9, the verification-design shapes; the helpers own
	// every mechanism — internal/proofbound/bounds.go). The volume error obeys the
	// symmetric-difference identity |1_{A∘B} − 1_{A'∘B'}| ≤ |1_A − 1_A'| +
	// |1_B − 1_B'| for all three ops, so it is the sum of the operands' own
	// symmetric-difference bounds — each δ · (that operand's held area) — plus
	// what the final float rounding can move. The BOUNDARY bound is a different
	// question, answered per vertex (docs/faceted-vertex-bounds-design.md §3):
	// a vertex the boolean itself creates sits at the crossing of two chord
	// PLANES, so it is displaced by its own facet pair's trim-amplified
	// (δ(t_A) + δ(t_B))/sin θ, not by δ, and the stitch adds each vertex's own
	// weld. The amplification is refused where it reaches the pair diameter.
	if err := refuseRimPastPair(maxRim, dPair); err != nil {
		return booleanEvaluation{}, meshbool.ExpectedBoolean(meshbool.BooleanExpectedUnsupported, err)
	}
	symA, err := operandSymDiff(ma)
	if err != nil {
		return booleanEvaluation{}, meshbool.ExpectedBooleanForOperand(meshbool.BooleanExpectedVolumeProof, 0, err)
	}
	symB, err := operandSymDiff(mb)
	if err != nil {
		return booleanEvaluation{}, meshbool.ExpectedBooleanForOperand(meshbool.BooleanExpectedVolumeProof, 1, err)
	}
	// The final rounding's own volume error is what its vertex displacement
	// sweeps out over the surface it acted on — the stitched surface BEFORE the
	// weld dropped any collapsed facet from it (internal/proofbound/bounds.go, proofbound.SweptVolumeAllow).
	// The held mesh's area is the WRONG yardstick here: a dropped facet is
	// missing from it, and its swept volume would go uncharged. The area the
	// weld dropped is likewise missing from every area the result reports, so
	// it joins the operands' own chord deficit in areaSlack.
	roundVol := proofbound.SweptVolumeAllow(stitched.Round, stitched.PreArea)
	volSymDiff, areaSlack := booleanProofBounds(
		symA, symB, roundVol,
		ma.areaSlack, mb.areaSlack, stitched.DropArea,
	)
	payload := facetedPayload{
		verts:       stitched.Verts,
		vertexBound: stitched.VertexBound,
		tris:        stitched.Tris,
		src:         stitched.Src,
		groups:      groups,
		meshBound:   facetproof.FacetBoundMax(stitched.Tris, stitched.VertexBound),
		volSymDiff:  volSymDiff,
		areaSlack:   areaSlack,
		dPair:       dPair,
		xform:       r3.Identity(),
	}
	if err := ctx.Err(); err != nil {
		return booleanEvaluation{}, err
	}
	audit, err := facetproof.AuditFacetedMesh(ctx, payload.verts, payload.tris)
	if err != nil {
		return booleanEvaluation{}, err
	}
	volume, volumeRat, err := meshVolumeMeasurement(ctx, audit.XVerts, payload.tris, payload.volSymDiff)
	if err != nil {
		return booleanEvaluation{}, err
	}
	if err := ctx.Err(); err != nil {
		return booleanEvaluation{}, err
	}
	return booleanEvaluation{payload: payload, volume: volume, volumeRat: volumeRat, audit: audit}, nil
}

// booleanProofBounds composes the independent, non-negative volume and area
// proof terms that survive the mesh boolean into its faceted result.
func booleanProofBounds(symA, symB, roundVol, slackA, slackB, dropArea float64) (
	volSymDiff, areaSlack float64,
) {
	return proofbound.AbsSumUpper(symA, symB, roundVol),
		proofbound.AbsSumUpper(slackA, slackB, dropArea)
}

// sourceIDs maps a tessellation's per-facet source faces to the global
// group ids.
func sourceIDs(ctx context.Context, m *Mesh, faceID map[*Face]int) ([]int, error) {
	out := make([]int, len(m.source))
	for i, f := range m.source {
		if i%256 == 0 {
			if err := ctx.Err(); err != nil {
				return nil, err
			}
		}
		id, ok := faceID[f]
		if !ok {
			return nil, fmt.Errorf(`%w: a facet's source is not an operand face`, ErrBooleanFailed)
		}
		out[i] = id
	}
	return out, nil
}

// operandSymDiff reads the volume of the symmetric difference between the
// operand's held tessellation and the body it stands for from the MESH's own
// proof record (docs/tessellation-design.md §2's volSymDiff), which the
// tessellator composed for that payload class.
//
// There is no fallback. `Mesh.Bound × held area` is NOT an occupied-volume
// proof — a two-sided Hausdorff bound alone does not bound it, because a
// doubly-curved cell can gain material where another loses it and the signed
// error cancels while the symmetric difference does not — and
// docs/tessellation-design.md §11 forbids the substitution outright. A payload
// whose occupied-volume proof has not landed publishes symDiffOK false, and its
// mesh serves export only: the boolean refuses the operand with ErrUnsupported,
// a staging refusal reached before any contact is examined, so the caller routes
// it through meshbool.BooleanExpectedVolumeProof, which reaches the same public error as
// an untessellatable operand and a distinct Verify diagnostic message.
// requireVolumeProvingPayload refuses an operand whose payload class publishes
// no occupied-volume proof on its mesh, before that mesh is built.
//
// It is operandSymDiff's question asked one step earlier. A cap-loop chamfer,
// and a draft body through the same band (draftOccupiedVolumeAdmission),
// narrows rather than refuses outright: both arms read the SAME predicate,
// capBlendOccupiedVolumeAdmission (capblend_admit.go) — this one before the
// mesh is built, and tessellateCapBlend through symDiffOK after it — so a
// payload whose every band docs/tessellation-reach-design.md §7 admits (whole
// turns, line-line miters, exactly G1 joins) passes here and publishes its
// proof there, and every other one refuses here with the loop and corner it
// fails on. Meshing a refused operand for a boolean only to refuse it
// afterwards buys nothing and costs the whole tessellation and its facet-pair
// audit — which the evaluator's own internal tolerance makes the most
// expensive part of the call. Both this and operandSymDiff must name the same
// payload classes, and each proof retires both arms of its own row together.
// The matching axial loft's held-polygon cap band has a separate slab proof
// for its reflex corners; tessellateCapBlend publishes that proof for this
// marked payload, so it passes this precheck too (loft §18).
//
// A sheet operand refuses HERE too, ahead of the payload-class switch below:
// docs/surface-design.md §10 states that a sheet mesh carries no
// occupied-volume proof, on the same terms as a refused cap-loop chamfer's. This is
// defence in depth rather than dead code — performBoolean's own
// refuseSheetOperand already fires first in the ordinary call path (Table X),
// so this arm exists to keep this function's own stated invariant, that it
// and operandSymDiff name the same set of payload classes, true even for a
// caller that reaches this function by some other path.
//
// A stitchPayload operand narrows rather than refuses outright: a CLOSED,
// all-planar body (sp.tris != nil) whose every vertex carries a proven bound
// of exactly zero (stitchZeroVertexBound, stitch.go — the identical gate
// tessellate_stitch.go's own occupied-volume publication and
// clearance_geom.go's carrier-model dispatch both apply) will publish
// symDiffOK == true once meshed, so refusing it here would contradict what
// operandSymDiff decides one step later over the same payload. Every other
// stitched body — open, curved/mixed, placed, or certificate-welded — still
// refuses here, before it is ever meshed.
func requireVolumeProvingPayload(ctx context.Context, b *Body, index int) error {
	var err error
	switch {
	case b.Kind() == BodySheet:
		err = fmt.Errorf(`%w: a sheet's mesh carries no proof of the volume it and the body it stands for differ by, so no boolean may compose it`, ErrUnsupported)
	default:
		switch pl := b.payload.(type) {
		case capBlendPayload:
			if pl.loftSource != nil {
				// The loft-only cap band proves its occupied volume with
				// a bounded reflex-corner slab in tessellateCapBlend.
				return nil
			}
			refusal, aErr := capBlendOccupiedVolumeAdmission(proofbound.NewWorkBudget(ctx), pl)
			if aErr != nil {
				return aErr
			}
			if refusal == nil {
				return nil
			}
			err = refusal
		case brepPayload:
			// A route L body's mesh proves its occupied volume only for bands
			// the cap-loop chamfer's rule admits (modify-general Table DG's
			// DG4); a plain brep's proof is unconditional.
			if len(pl.loopBands) == 0 {
				return nil
			}
			refusal, aErr := brepBandsOccupiedVolumeAdmission(proofbound.NewWorkBudget(ctx), pl)
			if aErr != nil {
				return aErr
			}
			if refusal == nil {
				return nil
			}
			err = refusal
		case draftPayload:
			refusal, aErr := draftOccupiedVolumeAdmission(proofbound.NewWorkBudget(ctx), pl)
			if aErr != nil {
				return aErr
			}
			if refusal == nil {
				return nil
			}
			err = refusal
		case twoSidedDraftPayload:
			for _, half := range []draftPayload{pl.negative, pl.positive} {
				refusal, aErr := draftOccupiedVolumeAdmission(proofbound.NewWorkBudget(ctx), half)
				if aErr != nil {
					return aErr
				}
				if refusal != nil {
					err = refusal
					break
				}
			}
			if err == nil {
				return nil
			}
		case stitchPayload:
			if pl.tris != nil {
				zeroBound, zErr := stitchZeroVertexBound(proofbound.NewWorkBudget(ctx), b)
				if zErr != nil {
					return zErr
				}
				if zeroBound {
					return nil
				}
			}
			err = fmt.Errorf(`%w: a stitched body's mesh carries no proof of the volume it and the body it stands for differ by, so no boolean may compose it`, ErrUnsupported)
		default:
			return nil
		}
	}
	return meshbool.ExpectedBooleanForOperand(meshbool.BooleanExpectedVolumeProof, index, err)
}

// An operand mesh built below VerifyBoundary is refused FIRST, and with its own
// message. Every homotopy §11 composes has §9's facet-contact audit as an
// antecedent, so such a mesh fails for the audit it declined rather than for
// the payload class it belongs to, and the volume-proof message would misstate
// the cause. No public call path reaches it — evaluateBoolean tessellates both
// operands at VerifyAll and the one-entry cache keys on the level — so this
// arm holds the invariant for any other path to the same composition.
func operandSymDiff(m *Mesh) (float64, error) {
	if !m.boundaryOK {
		return 0, fmt.Errorf(`%w: this operand's tessellation ran no facet-contact audit, so its facets are not proven to be an embedded boundary and no boolean may compose it; tessellate it at VerifyAll`, ErrUnsupported)
	}
	if !m.symDiffOK {
		return 0, fmt.Errorf(`%w: this operand's tessellation carries no proof of the volume it and the body it stands for differ by, so no boolean may compose it`, ErrUnsupported)
	}
	return m.volSymDiff, nil
}

// refuseUndecidableProximity is the tangency gate (docs/evaluator-design.md §9,
// docs/tessellation-design.md §11 step 4). The mesh boolean decides every
// contact on the CHORDS, and a chord polygon lies strictly inside the curved
// surface it approximates — so a true tangency between two operands can vanish
// from the tessellation entirely, and a spurious meet can appear where the true
// surfaces stay apart (chording a concave hole wall inward adds held material
// into the void). Either way the verdict would be decided by where the chord
// samples happened to fall.
//
// The gate is reject-only, and it proves nothing it does not have. If the true
// surfaces of two faces touch, that point lies within δA of face A's facets and
// within δB of face B's, so the two facet sets come within b = δA + δB of each
// other. A face pair whose facets stay FARTHER apart than b hides no touch and
// is decided. Within b the gate decides the pair by the INTERPENETRATION DEPTH
// of the two facet sets: a held meet is admitted only when the facets provably
// cross with a signed depth strictly GREATER than b (meshbool.ProvenDepthExceeds). Then
// A's true surface (within δA of its facets) and B's (within δB of its) still
// overlap even after each is pulled back toward its own interior by its own
// bound, so the true patches provably cross and the contact is real. The chord
// error alone, bounded by b, cannot open a penetration deeper than b, so a bare
// touch, a shallow meet at depth ≤ b, or a non-meeting near-miss within b proves
// nothing and is refused (ErrUnsupported). Deciding such a pair for real is the
// analytic clearance kernel's job (docs/clearance-design.md), not a chord's;
// loud beats silently wrong.
//
// The question is asked per analytic FACE pair, not per facet pair, and the
// depth is proven over the whole face's facets: a facet of one face can pass
// arbitrarily close to a facet of the other while the FACES plainly cross (a
// cylinder wall threading a cap's own triangulation diagonal), and that is no
// tangency at all.
//
// It may refuse a valid model whose operands genuinely pass within a chord
// tolerance of each other. That is the accepted price; the alternative is a
// verdict decided by chord placement.
func refuseUndecidableProximity(ctx context.Context, ma, mb *Mesh, bmA, bmB *meshbool.BoolMesh, memo *meshbool.ContactMemo) error {
	// One counter spans the grouping and the pair scan it feeds: the grouping
	// walks every facet and every new source face's edges, which is work the
	// §7.2 interval covers just as the pair scan is.
	budget := proofbound.NewWorkBudget(ctx)
	fa, err := facesOfMesh(budget, ma)
	if err != nil {
		return err
	}
	fb, err := facesOfMesh(budget, mb)
	if err != nil {
		return err
	}
	for _, ga := range fa {
		for _, gb := range fb {
			if err := budget.Step(); err != nil {
				return err
			}
			slack := ga.delta + gb.delta
			if slack <= 0 {
				// Both faces are held exactly (a planar face with straight
				// edges triangulates exactly; a Faceted face IS its polygons,
				// core §6.1). A tangency between them is visible to the exact
				// predicates, so there is nothing here a chord could hide.
				continue
			}
			// The exactly-tangent case sits ON the boundary d = δA + δB, and
			// both sides of that comparison are float-computed: pad the
			// threshold so the rounding can only ever ADD a refusal, never
			// drop one. A relative 1e-9 is nothing against any real clearance.
			near, err := meshbool.FacesNearMiss(ctx, bmA, ga.facets, bmB, gb.facets, slack*(1+1e-9), memo)
			if err != nil {
				return err
			}
			if near {
				return fmt.Errorf(`%w: the operands' held facets come within the chord tolerance without provably interpenetrating deeper than it, so whether their true surfaces touch or cross is decided by where the chords fall — this evaluator refuses the question rather than answer it wrong`, ErrUnsupported)
			}
		}
	}
	return nil
}

// faceFacets is one analytic face of an operand: its facets, and the chord
// displacement between them and the true face.
type faceFacets struct {
	delta  float64
	facets []int
}

// facesOfMesh groups a tessellation's facets by their source face, in first-
// appearance order, and charges each face the displacement the MESH proved for
// it: docs/tessellation-design.md §2's sourceBound(face), which the tessellator
// composed from that face's own trim chording, coordinate construction, section
// and axial terms. The gate never re-infers a displacement from the surface
// kind — a planar carrier does not make a trimmed patch exact, and a zero here
// is the claim that the held polygon IS the true trimmed patch with stored
// coordinates that add nothing.
//
// A source face the mesh states no bound for is a broken evaluator, not a
// staged capability: it returns ErrBooleanFailed rather than an ErrUnsupported
// that Verify would hide as an undecided pair, and never a zero.
func facesOfMesh(budget *proofbound.WorkBudget, m *Mesh) ([]faceFacets, error) {
	index := map[*Face]int{}
	var out []faceFacets
	for i, f := range m.source {
		if err := budget.Step(); err != nil {
			return nil, err
		}
		k, ok := index[f]
		if !ok {
			delta, stated := m.sourceBound(f)
			if !stated {
				return nil, fmt.Errorf(`%w: an operand's tessellation states no displacement bound for one of its own source faces`, ErrBooleanFailed)
			}
			k = len(out)
			index[f] = k
			out = append(out, faceFacets{delta: delta})
		}
		out[k].facets = append(out[k].facets, i)
	}
	return out, nil
}

// sectionDisplacementOf is the proven displacement between a body's recorded
// boundary and the boundary its construction denotes: the analytic prism
// boolean's re-expression rounding and its surviving fragments' own cut
// parameters, and zero for every other body (docs/prism-boolean-design.md §7).
func sectionDisplacementOf(b *Body) float64 {
	if b == nil {
		return 0
	}
	switch p := b.payload.(type) {
	case prismPayload:
		return p.sectionDelta
	case stackedPrismPayload:
		return p.sectionDelta
	case brepPayload:
		return p.sectionDelta()
	default:
		return 0
	}
}

// pairChordTolerance derives the evaluator-internal chord tolerance and the
// pair diameter from the operands' own bounds boxes (inflated by their
// proven bounds). All-planar operands chord nothing regardless, so the
// tolerance only shapes curved boundaries.
func pairChordTolerance(ctx context.Context, a, b *Body) (float64, float64, error) {
	ca, err := chordOperandOf(ctx, a)
	if err != nil {
		return 0, 0, err
	}
	cb, err := chordOperandOf(ctx, b)
	if err != nil {
		return 0, 0, err
	}
	tol, diameter, ok := meshbool.PairChordFrom(ca, cb)
	if !ok {
		return 0, 0, fmt.Errorf(`%w: the operand pair has no extent to derive a chord tolerance from`, ErrDegenerate)
	}
	return tol, diameter, nil
}

// chordOperandOf reads b's share of pairChordTolerance.
//
// The floor raises the pair's tolerance past the operand's own reservations.
// An operand holding its section within a displacement of the one it denotes
// cannot be meshed within that displacement — tessellation reserves it from
// the requested tolerance (docs/tessellation-design.md §5) — and this mesh
// path is the designated fallback for exactly those operands
// (docs/prism-boolean-design.md §3.4). A diameter-derived tolerance under the
// displacement would refuse them. A revolve likewise reserves both of its
// coordinate stages out of the tolerance before it chords anything
// (docs/tessellation-design.md §8), so a diameter-derived tolerance under
// that reservation refuses a body whose own geometry is perfectly ordinary —
// a small part modelled at a large coordinate is the case that reaches it.
// Nothing is lost by the coarser chording: each mesh reports the sagitta it
// actually took, and the reserved figure already dominates the bound it
// publishes.
func chordOperandOf(ctx context.Context, b *Body) (meshbool.ChordOperand, error) {
	box, err := b.Bounds()
	if err != nil {
		return meshbool.ChordOperand{}, err
	}
	inf := box.Bound.Base()
	pad := r3.NewVec(inf, inf, inf)
	return meshbool.ChordOperand{
		Lo: box.Min.Sub(pad),
		Hi: box.Max.Add(pad),
		Floor: max(
			proofbound.ProductUpper(2, sectionDisplacementOf(b)),
			proofbound.ProductUpper(2, coordDisplacementOf(ctx, b)),
			heldPrimitiveFloorOf(b),
		),
	}, nil
}

// heldPrimitiveFloorOf is the proven held bound of an operand built as a
// fixed held mesh in one construction — a coil's or a mitred sweep's delta
// (docs/faceted-vertex-bounds-design.md §5). Its restatement cannot be
// meshed closer than that, and every facet it touches the partner with holds
// at most that bound, so raising the pair tolerance to it is what lets the
// chain-depth gate admit the pair. A boolean result's meshBound is not one:
// it grows with the chain, which is what the gate exists to refuse. Every
// other payload answers zero.
func heldPrimitiveFloorOf(b *Body) float64 {
	switch p := b.payload.(type) {
	case coilPayload:
		return p.delta
	case mitredSweepPayload:
		return p.delta
	default:
		return 0
	}
}

// coordDisplacementOf is the count-INDEPENDENT coordinate reservation an
// operand's own tessellation spends before it chords anything: for a revolve,
// docs/tessellation-design.md §8's deltaC and deltaR ceilings composed
// (resolveRevolve). Every other payload class answers zero, and so does a
// revolve whose resolution fails — the tessellation that follows refuses it
// with its own sentinel, and inventing a tolerance here would only hide which
// one.
func coordDisplacementOf(ctx context.Context, b *Body) float64 {
	if b == nil {
		return 0
	}
	rp, ok := b.payload.(revolvePayload)
	if !ok {
		return 0
	}
	res, err := resolveRevolve(ctx, rp)
	if err != nil {
		return 0
	}
	if rp.sectionDelta > 0 {
		// The revolve tessellator reserves its section displacement beside the
		// two coordinate stages (planRevolve), so all three come out of one
		// tolerance.
		return proofbound.AbsSumUpper(res.DeltaCPrior, res.DeltaRPrior, rp.sectionDelta)
	}
	return proofbound.AbsSumUpper(res.DeltaCPrior, res.DeltaRPrior)
}

// analyticBooleanBody evaluates an analytic boolean's result payload under
// producer identity ref, without committing it.
func analyticBooleanBody(ctx context.Context, d *Document, ref producerID, payload featurePayload) (*Body, error) {
	switch p := payload.(type) {
	case prismPayload:
		return evalPrismContext(ctx, d, ref, p, freeform.NewFreeformWork())
	case stackedPrismPayload:
		return evalStackedContext(ctx, d, ref, p)
	case brepPayload:
		return evalBrepContext(ctx, d, ref, p)
	default:
		return nil, fmt.Errorf(`%w: an analytic boolean produced a %T payload`, ErrUnsupported, payload)
	}
}
