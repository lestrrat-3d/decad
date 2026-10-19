package decad

import (
	"context"
	"errors"
	"fmt"
	"math/big"
	"strings"

	"github.com/lestrrat-3d/decad/internal/booloperand"
	"github.com/lestrrat-3d/decad/internal/breprecord"
	"github.com/lestrrat-3d/decad/internal/classbgeom"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/facetproof"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/obliqueboss"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/revolveplan"
	"github.com/lestrrat-3d/decad/internal/stackedrecord"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/units"
)

// This file is the public boolean surface of core §8. Admitted prism and
// common-axis Revolve pairs use their source records; other pairs use the
// exact-predicate mesh boolean of docs/evaluator-design.md §9. The mesh path
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
// carriers, plus certified straight and spiral bevel tooth patterns. Other
// prism inputs use the two-body evaluator in order, with one final commit.
// Each pair must pass its own proof. Failure or cancellation leaves every
// input live. A two-body result from the mesh path is a Faceted body:
// its faces are grouped by the operands' source faces, so provenance
// (FaceCreatedBy) survives, and its measurements are Approximate with proven
// bounds (docs/evaluator-design.md §9) — except that an all-planar pair whose
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
func Union(ctx context.Context, first *Body, others ...*Body) (*Body, error) {
	if ctx == nil {
		return nil, fmt.Errorf("%w: a nil context cannot control a boolean", ErrDegenerate)
	}
	if len(others) == 0 {
		return nil, fmt.Errorf("%w: Union needs at least two bodies", ErrDegenerate)
	}
	if len(others) == 1 {
		return performBoolean(ctx, meshbool.OpUnion, first, others[0])
	}
	bodies := make([]*Body, 0, len(others)+1)
	bodies = append(bodies, first)
	bodies = append(bodies, others...)
	return performUnionMany(ctx, bodies)
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

// tessellateBooleanOperand meshes one boolean operand at its own request
// tolerance: the pair tolerance, raised to a restating operand's held floor
// (booloperand.HeldFloor) so that the restatement never refuses
// (docs/faceted-vertex-bounds-design.md §5). It reports whether the operand
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
func performBoolean(ctx context.Context, op meshbool.OperationKind, a, b *Body) (*Body, error) {
	if ctx == nil {
		return nil, fmt.Errorf(`%w: a nil context cannot control a boolean`, ErrDegenerate)
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
	d := documentHandle(a.n.Doc)
	if body, ok, err := tryLoftMirrorUnion(ctx, op, d, ref, a, b); err != nil {
		return nil, err
	} else if ok {
		return body, nil
	}
	if body, ok, err := tryLoftMirrorCylinderUnion(ctx, op, d, ref, a, b); err != nil {
		return nil, err
	} else if ok {
		return body, nil
	}
	if body, ok, err := tryLoftCylinderUnion(ctx, op, d, ref, a, b); err != nil {
		return nil, err
	} else if ok {
		return body, nil
	}
	if body, ok, err := tryRevolveSectorCylinderCut(ctx, op, d, ref, a, b); err != nil {
		return nil, err
	} else if ok {
		return body, nil
	}
	if body, ok, err := tryLoftMirrorCylinderBoreCut(ctx, op, d, ref, a, b); err != nil {
		return nil, err
	} else if ok {
		return body, nil
	}
	if body, ok, err := tryLoftCylinderBoreCut(ctx, op, d, ref, a, b); err != nil {
		return nil, err
	} else if ok {
		return body, nil
	}
	if body, ok, err := tryRevolveLoftNextSectorUnion(ctx, op, d, ref, a, b); err != nil {
		return nil, err
	} else if ok {
		return body, nil
	}
	if body, ok, err := tryRevolveLoftSectorUnion(ctx, op, d, ref, a, b); err != nil {
		return nil, err
	} else if ok {
		return body, nil
	}
	if ok, err := tryRevolveLoftFacesUnion(ctx, op, a, b); err != nil {
		return nil, err
	} else if ok {
		return nil, ErrUnsupported
	}
	if body, ok, err := tryPointConeBoolean(ctx, op, d, ref, a, b); err != nil {
		return nil, err
	} else if ok {
		return body, nil
	}
	if body, ok, err := tryLoftFacesConeHeel(ctx, op, d, ref, a, b); err != nil {
		return nil, err
	} else if ok {
		return body, nil
	}
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
	if pp, ok, err := tryAxialLoftPrismUnion(ctx, op, a, b); err != nil {
		if errors.Is(err, ErrUnsupported) {
			return nil, asBooleanError(op, meshbool.ExpectedBoolean(meshbool.BooleanExpectedUnsupported, err))
		}
		return nil, err
	} else if ok {
		return analyticBooleanBody(ctx, d, ref, pp)
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
		body, err := bodyResult(prismrecord.Build(ctx, envOf(documentNode(d), ref), pp, freeform.NewFreeformWork()))
		if err != nil {
			return nil, err
		}
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		return body, nil
	}
	if op == meshbool.OpUnion {
		if payload, ok, err := tryPrismFacePatchUnion(ctx, a, b); err != nil {
			if errors.Is(err, ErrUnsupported) {
				return nil, asBooleanError(op, meshbool.ExpectedBoolean(meshbool.BooleanExpectedUnsupported, err))
			}
			return nil, err
		} else if ok {
			return analyticBooleanBody(ctx, d, ref, payload)
		}
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
			body, err := bodyResult(stackedrecord.Build(ctx, envOf(documentNode(d), ref), sp, stackedrecord.OwnPlan{}))
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
			body, err := bodyResult(stackedrecord.Build(ctx, envOf(documentNode(d), ref), sp, stackedrecord.OwnPlan{}))
			if err != nil {
				return nil, err
			}
			if err := ctx.Err(); err != nil {
				return nil, err
			}
			return body, nil
		}
		if bp, ok, err := tryCrossingBlindCut(ctx, a, b); err != nil {
			return nil, err
		} else if ok {
			return analyticBooleanBody(ctx, d, ref, bp)
		}
		if sp, ok, err := tryBlindStackedCut(ctx, a, b); err != nil {
			if errors.Is(err, ErrUnsupported) {
				return nil, asBooleanError(op, meshbool.ExpectedBoolean(meshbool.BooleanExpectedUnsupported, err))
			}
			return nil, err
		} else if ok {
			body, err := bodyResult(stackedrecord.Build(ctx, envOf(documentNode(d), ref), sp, stackedrecord.OwnPlan{}))
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
	if body, ok, err := tryCommonAxisRevolveBoolean(ctx, op, d, ref, a, b); err != nil {
		if errors.Is(err, ErrUnsupported) {
			return nil, asBooleanError(op, meshbool.ExpectedBoolean(meshbool.BooleanExpectedUnsupported, err))
		}
		return nil, err
	} else if ok {
		return body, nil
	}
	if body, ok, err := tryLoftContactDirectShellUnion(ctx, op, d, ref, a, b); err != nil {
		return nil, err
	} else if ok {
		return body, nil
	}

	eval, err := evaluateBoolean(ctx, op, a, b)
	if err != nil {
		return nil, asBooleanError(op, err)
	}
	if op == meshbool.OpUnion {
		eval.payload.ObliqueBoss = obliqueboss.Recognize(bodyNode(a), bodyNode(b))
		eval.payload.PrismUnionFillet = recognizePrismUnionFillet(a, b)
	}
	if op == meshbool.OpCut {
		eval.payload.CrossCylinderBore = recognizeCrossCylinderBore(a, b)
	}

	body, err := bodyResult(facetedrecord.BuildWithProof(ctx, envOf(documentNode(d), ref), eval.payload, eval.audit, measurementToInternal(eval.volume), eval.volumeRat))
	if err != nil {
		return nil, asBooleanError(op, err)
	}
	if op == meshbool.OpCut {
		if err := recordLoftFacesConeToe(ctx, a, b, body); err != nil {
			return nil, err
		}
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
// prismrecord.Build over the admitted payload — under a producerID it mints for
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
	d := documentHandle(a.n.Doc)
	body, err := bodyResult(prismrecord.Build(ctx, envOf(documentNode(d), d.nextProducerID()), pp, freeform.NewFreeformWork()))
	if err != nil {
		return nil, false, err
	}
	return body, true, nil
}

// asBooleanError maps an evaluateBoolean or facetedrecord.BuildWithProof error to the
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
// one operand, and whether the operand restates a held mesh (booloperand.HeldFloor),
// which arms the local chain-depth gate at the pair tolerance. pairTol is the
// pair's own chord tolerance (booloperand.PairChord). Every implementation
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
	return evaluateBooleanMeshesAtScale(ctx, op, a, b, meshes, 1)
}

// evaluateBooleanMeshesAtScale lets one source-certified modify route ask
// the same audited mesh Boolean for finer operand chords. A power-of-two
// scale preserves the pair tolerance's represented value exactly, while
// every operand still reports its own chord and occupied-volume bounds.
func evaluateBooleanMeshesAtScale(ctx context.Context, op meshbool.OperationKind, a, b *Body,
	meshes operandMeshes, chordScale float64) (booleanEvaluation, error) {
	return evaluateBooleanMeshesWithConeCap(ctx, op, a, b, meshes, chordScale, nil, nil)
}

// faceFacets is one analytic face of an operand: its facets, and the chord
// displacement between them and the true face.
type faceFacets struct {
	source *Face
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
	index := map[*topo.Face]int{}
	var out []faceFacets
	for i, f := range m.n.Source {
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
			out = append(out, faceFacets{source: faceHandle(f), delta: delta})
		}
		out[k].facets = append(out[k].facets, i)
	}
	return out, nil
}

// analyticBooleanBody evaluates an analytic boolean's result payload under
// producer identity ref, without committing it.
func analyticBooleanBody(ctx context.Context, d *Document, ref producerID, payload topo.Payload) (*Body, error) {
	switch p := payload.(type) {
	case prismPayload:
		return bodyResult(prismrecord.Build(ctx, envOf(documentNode(d), ref), p, freeform.NewFreeformWork()))
	case stackedPrismPayload:
		return bodyResult(stackedrecord.Build(ctx, envOf(documentNode(d), ref), p, stackedrecord.OwnPlan{}))
	case brepPayload:
		return bodyResult(breprecord.Build(ctx, envOf(documentNode(d), ref), p))
	default:
		return nil, fmt.Errorf(`%w: an analytic boolean produced a %T payload`, ErrUnsupported, payload)
	}
}

// tryClassB attempts op over the pair of bodies. ok=false (err nil) is a
// silent miss: the caller takes the mesh path unchanged. It is
// classbgeom.OfPayloads over the operands' payloads, placed as the first
// operand is.
func tryClassB(ctx context.Context, op meshbool.OperationKind, a, b *Body) (topo.Payload, bool, error) {
	// A body this evaluator did not build has no payload and no class.
	if a.n.Payload == nil || b.n.Payload == nil {
		return nil, false, nil
	}
	return classbgeom.OfPayloads(ctx, op, a.n.Payload, b.n.Payload, a.n.Payload.Transform(), 0)
}

// tryCommonAxisRevolveBoolean keeps the public body and error boundary after
// the recorded meridians have been admitted by revolveplan.
func tryCommonAxisRevolveBoolean(ctx context.Context, op meshbool.OperationKind, d *Document,
	ref producerID, a, b *Body) (*Body, bool, error) {
	result, admitted, empty, err := revolveplan.PrepareCommonAxis(ctx, op, &a.n, &b.n)
	if err != nil || !admitted {
		return nil, false, err
	}
	if empty {
		return nil, true, asBooleanError(op, meshbool.ExpectedBoolean(meshbool.BooleanExpectedEmpty,
			fmt.Errorf("%w: the common-axis result contains no volume", ErrBooleanFailed)))
	}
	body, err := bodyResult(revolveplan.Build(ctx, envOf(documentNode(d), ref), result, freeform.NewFreeformWork()))
	if err != nil {
		return nil, false, err
	}
	return body, true, nil
}
