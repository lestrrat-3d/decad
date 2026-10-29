// Package booleandispatch routes one Boolean to the first route that admits
// the operand pair, over topology nodes (agent-docs/evaluator-design.md §9). Root
// supplies what only root can: the operand mesher, the typed-error mapping and
// the routes that read public handles.
package booleandispatch

import (
	"context"
	"errors"
	"fmt"

	"github.com/lestrrat-3d/units"

	"github.com/lestrrat-3d/decad/internal/axialsection"
	"github.com/lestrrat-3d/decad/internal/booleaneval"
	"github.com/lestrrat-3d/decad/internal/classbgeom"
	"github.com/lestrrat-3d/decad/internal/crossbore"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/loftcylinder"
	"github.com/lestrrat-3d/decad/internal/loftfacestrim"
	"github.com/lestrrat-3d/decad/internal/loftmirror"
	"github.com/lestrrat-3d/decad/internal/loftrecord"
	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/obliqueboss"
	"github.com/lestrrat-3d/decad/internal/prismboolean"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/revolveplan"
	"github.com/lestrrat-3d/decad/internal/revolveshell"
	"github.com/lestrrat-3d/decad/internal/stackedrecord"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/verifiedmesh"
)

// Hooks are the root-owned parts of the dispatch.
type Hooks struct {
	// Mesher meshes each operand at the pair's chord tolerance.
	Mesher booleaneval.OperandMesher
	// Verified gives builders a body's mesh at VerifyAll.
	Verified verifiedmesh.Mesher
	// AsBooleanError maps an evaluator error to the public typed BooleanError.
	AsBooleanError func(meshbool.OperationKind, error) error
	// DirectShellUnion is the LoftFaces direct-shell Union route.
	DirectShellUnion func(ctx context.Context, op meshbool.OperationKind, env topo.Env, a, b *topo.Body) (*topo.Body, bool, error)
	// LowerSupport certifies a faceted Union result's lower support plane.
	LowerSupport func(ctx context.Context, result, a, b *topo.Body) error
	// MatchingFaces is decad.WithMatchingFaces: the caller intends the
	// operands' faces to meet exactly where they meet within rounding.
	MatchingFaces bool
}

// Analytic runs every route Body tries before the mesh pipeline, in Body's order.
// ok=false with a nil error is a miss. A non-nil error is the route's refusal, already
// mapped through hooks.AsBooleanError where Body maps it. It writes nothing to the
// document. a and b must belong to one document.
func Analytic(ctx context.Context, op meshbool.OperationKind, env topo.Env, a, b *topo.Body,
	hooks Hooks) (*topo.Body, bool, error) {
	if n, ok, err := loftmirror.Union(ctx, op, env, hooks.Verified,
		a, b); err != nil {
		return nil, false, err
	} else if ok {
		return n, true, nil
	}
	if n, ok, err := loftcylinder.MirrorUnion(ctx, op, env, hooks.Verified,
		a, b); err != nil {
		return nil, false, err
	} else if ok {
		return n, true, nil
	}
	if n, ok, err := loftcylinder.JoinUnion(ctx, op, env, hooks.Verified,
		a, b); err != nil {
		return nil, false, err
	} else if ok {
		return n, true, nil
	}
	if n, ok, err := revolveshell.SectorCylinderCut(ctx, op, env,
		a, b); err != nil {
		return nil, false, err
	} else if ok {
		return n, true, nil
	}
	if n, ok, err := loftcylinder.MirrorBoreCut(ctx, op, env, hooks.Verified,
		a, b); err != nil {
		return nil, false, err
	} else if ok {
		return n, true, nil
	}
	if n, ok, err := loftcylinder.JoinBoreCut(ctx, op, env, hooks.Verified,
		a, b); err != nil {
		return nil, false, err
	} else if ok {
		return n, true, nil
	}
	if n, ok, err := revolveshell.NextSectorUnion(ctx, op, env, a, b); err != nil {
		return nil, false, err
	} else if ok {
		return n, true, nil
	}
	if n, ok, err := revolveshell.SectorUnion(ctx, op, env, a, b); err != nil {
		return nil, false, err
	} else if ok {
		return n, true, nil
	}
	if n, ok, err := revolveplan.PointConeBoolean(ctx, op, env, a, b, hooks.MatchingFaces); err != nil {
		if _, ok := errors.AsType[*meshbool.BooleanExpectedError](err); ok {
			err = hooks.AsBooleanError(op, err)
		}
		return nil, false, err
	} else if ok {
		return n, true, nil
	}
	if n, ok, err := loftfacestrim.ConeHeelBoolean(ctx, op, env, hooks.Mesher,
		a, b); err != nil {
		return nil, false, hooks.AsBooleanError(op, err)
	} else if ok {
		return n, true, nil
	}
	// Table X (agent-docs/surface-design.md §11): a sheet operand in either position
	// is a plain decaderr.ErrUnsupported, never a BooleanError — without this a sheet
	// falls through to prismboolean.Try below, which type-asserts prismPayload
	// and a sheet still carries one, producing a confidently-wrong solid.
	if err := refuseSheetOperand(a, op.String()); err != nil {
		return nil, false, err
	}
	if err := refuseSheetOperand(b, op.String()); err != nil {
		return nil, false, err
	}
	// agent-docs/axial-section-boolean-design.md: a Union of axial extrusions
	// between the same two planes is the extrusion of the 2D union of their
	// sections, when some section holds a free-form segment the analytic prism
	// routes below do not read. ok=false is a miss, never an error.
	if op == meshbool.OpUnion {
		if n, ok, err := axialsection.UnionMany(ctx, env, []*topo.Body{a, b}); err != nil {
			return nil, false, err
		} else if ok {
			return n, true, nil
		}
	}
	if pp, ok, err := prismboolean.TryAxialLoftUnion(ctx, op, a.Payload, b.Payload); err != nil {
		if errors.Is(err, decaderr.ErrUnsupported) {
			return nil, false, hooks.AsBooleanError(op, meshbool.ExpectedBoolean(meshbool.BooleanExpectedUnsupported, err))
		}
		return nil, false, err
	} else if ok {
		return found(prismboolean.AnalyticBody(ctx, env, pp))
	}
	// agent-docs/prism-boolean-design.md: a reject-only analytic reduction for a
	// co-directional coplanar prism pair, dispatched ahead of the mesh path
	// for Union's select-all/merge/chain path (§4.2) and Cut/Intersect's
	// clean-nesting structural match (§4.2). ok=false is never an error —
	// the pair falls back to the unchanged mesh path below exactly as it did
	// before this design existed. A non-nil err here is a genuine, typed
	// analytic-resolution refusal (§3.4) and must propagate rather than
	// reroute to the mesh path: an decaderr.ErrUnsupported refusal becomes the public
	// BooleanUnsupportedContact below, preserving errors.Is(err,
	// decaderr.ErrUnsupported); ErrDegenerate and ErrUnrecordableProfile pass through
	// unwrapped, keeping their own documented sentinels.
	if pp, ok, err := prismboolean.Try(ctx, op, a.Payload, b.Payload); err != nil {
		if errors.Is(err, decaderr.ErrUnsupported) {
			return nil, false, hooks.AsBooleanError(op, meshbool.ExpectedBoolean(meshbool.BooleanExpectedUnsupported, err))
		}
		return nil, false, err
	} else if ok {
		body, err := prismrecord.Build(ctx, env, pp, freeform.NewFreeformWork())
		if err != nil {
			return nil, false, err
		}
		if err := ctx.Err(); err != nil {
			return nil, false, err
		}
		return body, true, nil
	}
	if op == meshbool.OpUnion {
		if payload, ok, err := prismboolean.TryFacePatchUnion(ctx, a.Payload, b.Payload); err != nil {
			if errors.Is(err, decaderr.ErrUnsupported) {
				return nil, false, hooks.AsBooleanError(op, meshbool.ExpectedBoolean(meshbool.BooleanExpectedUnsupported, err))
			}
			return nil, false, err
		} else if ok {
			return found(prismboolean.AnalyticBody(ctx, env, payload))
		}
		// agent-docs/general-boolean-design.md §3 A1: unequal sweep intervals that
		// overlap or touch build a stacked prism, or a brep where an
		// interface is flush or crossing. Same contract as above.
		if payload, ok, err := prismboolean.TryStackedUnion(ctx, a.Payload, b.Payload); err != nil {
			if errors.Is(err, decaderr.ErrUnsupported) {
				return nil, false, hooks.AsBooleanError(op, meshbool.ExpectedBoolean(meshbool.BooleanExpectedUnsupported, err))
			}
			return nil, false, err
		} else if ok {
			body, err := prismboolean.AnalyticBody(ctx, env, payload)
			if err != nil {
				return nil, false, err
			}
			if err := ctx.Err(); err != nil {
				return nil, false, err
			}
			return body, true, nil
		}
		// agent-docs/general-boolean-design.md §3 A5: a prism-group operand, or
		// survivors that close into several disjoint loops.
		if payload, ok, err := prismboolean.TryGroupUnion(ctx, a.Payload, b.Payload); err != nil {
			if errors.Is(err, decaderr.ErrUnsupported) {
				return nil, false, hooks.AsBooleanError(op, meshbool.ExpectedBoolean(meshbool.BooleanExpectedUnsupported, err))
			}
			return nil, false, err
		} else if ok {
			return found(prismboolean.AnalyticBody(ctx, env, payload))
		}
	}
	if op == meshbool.OpCut {
		if node, ok, err := loftrecord.ThroughBoreCut(ctx, env, a.Payload, b.Payload); err != nil {
			return nil, false, err
		} else if ok {
			return node, true, nil
		}
		if sp, ok, err := prismboolean.TryStackedThroughCut(ctx, a.Payload, b.Payload); err != nil {
			if errors.Is(err, decaderr.ErrUnsupported) {
				return nil, false, hooks.AsBooleanError(op, meshbool.ExpectedBoolean(meshbool.BooleanExpectedUnsupported, err))
			}
			return nil, false, err
		} else if ok {
			body, err := stackedrecord.Build(ctx, env, sp, stackedrecord.OwnPlan{})
			if err != nil {
				return nil, false, err
			}
			if err := ctx.Err(); err != nil {
				return nil, false, err
			}
			return body, true, nil
		}
		if sp, ok, err := prismboolean.TryStackedBlindCut(ctx, a.Payload, b.Payload); err != nil {
			if errors.Is(err, decaderr.ErrUnsupported) {
				return nil, false, hooks.AsBooleanError(op, meshbool.ExpectedBoolean(meshbool.BooleanExpectedUnsupported, err))
			}
			return nil, false, err
		} else if ok {
			body, err := stackedrecord.Build(ctx, env, sp, stackedrecord.OwnPlan{})
			if err != nil {
				return nil, false, err
			}
			if err := ctx.Err(); err != nil {
				return nil, false, err
			}
			return body, true, nil
		}
		// agent-docs/general-boolean-design.md §3 A8: opposed pockets that
		// overlap in plan at the interface the tool ends at.
		if bp, ok, err := prismboolean.TryOpposedBlindCut(ctx, a.Payload, b.Payload); err != nil {
			return nil, false, err
		} else if ok {
			return found(prismboolean.AnalyticBody(ctx, env, bp))
		}
		if bp, ok, err := prismboolean.TryCrossingBlindCut(ctx, a.Payload, b.Payload); err != nil {
			return nil, false, err
		} else if ok {
			return found(prismboolean.AnalyticBody(ctx, env, bp))
		}
		if sp, ok, err := prismboolean.TryBlindStackedCut(ctx, a.Payload, b.Payload); err != nil {
			if errors.Is(err, decaderr.ErrUnsupported) {
				return nil, false, hooks.AsBooleanError(op, meshbool.ExpectedBoolean(meshbool.BooleanExpectedUnsupported, err))
			}
			return nil, false, err
		} else if ok {
			body, err := stackedrecord.Build(ctx, env, sp, stackedrecord.OwnPlan{})
			if err != nil {
				return nil, false, err
			}
			if err := ctx.Err(); err != nil {
				return nil, false, err
			}
			return body, true, nil
		}
		// agent-docs/general-boolean-design.md §3 A5: a prism-group tool.
		if pp, ok, err := prismboolean.TryGroupCut(ctx, a.Payload, b.Payload); err != nil {
			if errors.Is(err, decaderr.ErrUnsupported) {
				return nil, false, hooks.AsBooleanError(op, meshbool.ExpectedBoolean(meshbool.BooleanExpectedUnsupported, err))
			}
			return nil, false, err
		} else if ok {
			return found(prismboolean.AnalyticBody(ctx, env, pp))
		}
	}
	// agent-docs/general-boolean-design.md §3 class B: a perpendicular pair with a
	// prism, stacked or brep operand builds a brep body (or, for Intersect, a
	// prism). Same contract as above.
	if payload, ok, err := classB(ctx, op, a, b); err != nil {
		if errors.Is(err, decaderr.ErrUnsupported) {
			return nil, false, hooks.AsBooleanError(op, meshbool.ExpectedBoolean(meshbool.BooleanExpectedUnsupported, err))
		}
		return nil, false, err
	} else if ok {
		return found(prismboolean.AnalyticBody(ctx, env, payload))
	}
	if body, ok, err := commonAxisRevolve(ctx, op, env, a, b, hooks); err != nil {
		if errors.Is(err, decaderr.ErrUnsupported) {
			return nil, false, hooks.AsBooleanError(op, meshbool.ExpectedBoolean(meshbool.BooleanExpectedUnsupported, err))
		}
		return nil, false, err
	} else if ok {
		return body, true, nil
	}
	if body, ok, err := hooks.DirectShellUnion(ctx, op, env, a, b); err != nil {
		return nil, false, err
	} else if ok {
		return body, true, nil
	}

	return nil, false, nil
}

// found adapts a builder's (body, error) result to a route hit.
func found(b *topo.Body, err error) (*topo.Body, bool, error) {
	if err != nil {
		return nil, false, err
	}
	return b, true, nil
}

// Body builds a Boolean's result body under env's producer identity without
// touching the document: no operand is retired and nothing is registered. The
// caller commits what it returns; Patterned's Union fallback chains it over
// instances not yet registered and commits only the last result. a and b must
// belong to one document.
func Body(ctx context.Context, op meshbool.OperationKind, env topo.Env, a, b *topo.Body, hooks Hooks) (*topo.Body, error) {
	if n, ok, err := Analytic(ctx, op, env, a, b, hooks); err != nil {
		return nil, err
	} else if ok {
		return n, nil
	}

	eval, err := booleaneval.Evaluate(ctx, op, a, b, hooks.Mesher, 1, nil, nil)
	if err != nil {
		return nil, hooks.AsBooleanError(op, err)
	}
	if op == meshbool.OpUnion {
		eval.Payload.ObliqueBoss = obliqueboss.Recognize(a, b)
		eval.Payload.PrismUnionFillet = facetedrecord.RecognizePrismUnionFillet(a, b)
	}
	if op == meshbool.OpCut {
		eval.Payload.CrossCylinderBore = crossbore.RecognizeBore(a, b)
	}

	volume := measurement.Measurement{Value: units.CubicMillimeters(eval.Volume),
		Exactness: measurement.ExactnessOf(eval.VolumeBound), Bound: units.CubicMillimeters(eval.VolumeBound)}
	body, err := facetedrecord.BuildWithProof(ctx, env, eval.Payload, eval.Audit, volume, eval.VolumeRat)
	if err != nil {
		return nil, hooks.AsBooleanError(op, err)
	}
	body.OperationWarnings = append(body.OperationWarnings, eval.Diagnostics(op)...)
	if op == meshbool.OpCut {
		if err := loftfacestrim.RecordConeToe(ctx, a, b, body); err != nil {
			return nil, err
		}
	}
	if op == meshbool.OpUnion {
		if err := hooks.LowerSupport(ctx, body, a, b); err != nil {
			return nil, err
		}
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	return body, nil
}

// refuseSheetOperand reports ErrUnsupported when b is a sheet, for an operation
// agent-docs/surface-design.md Table X does not admit a sheet to.
func refuseSheetOperand(b *topo.Body, operation string) error {
	if b != nil && b.Kind == topo.BodySheet {
		return fmt.Errorf(`%w: %s does not accept a sheet body (agent-docs/surface-design.md Table X)`, decaderr.ErrUnsupported, operation)
	}
	return nil
}

// classB attempts op over the pair of bodies. ok=false (err nil) is a silent
// miss: the caller takes the mesh path unchanged. It is classbgeom.OfPayloads
// over the operands' payloads, placed as the first operand is.
func classB(ctx context.Context, op meshbool.OperationKind, a, b *topo.Body) (topo.Payload, bool, error) {
	// A body this evaluator did not build has no payload and no class.
	if a.Payload == nil || b.Payload == nil {
		return nil, false, nil
	}
	// A turned shaft's Cut reads the shaft as the prism, or the stack of
	// coaxial prisms, it denotes exactly
	// (agent-docs/general-boolean-design.md §3 "Revolved shaft as a prism").
	if rp, ok := a.Payload.(revolveplan.Payload); ok && op == meshbool.OpCut {
		steps, ok := rp.CylinderSteps()
		if !ok {
			return nil, false, nil
		}
		if len(steps) == 1 {
			return classbgeom.CutPrismView(ctx, steps[0], b.Payload)
		}
		return classbgeom.CutSteppedView(ctx, steps, b.Payload)
	}
	return classbgeom.OfPayloads(ctx, op, a.Payload, b.Payload, a.Payload.Transform(), 0)
}

// commonAxisRevolve keeps the typed error boundary after the recorded meridians
// have been admitted by revolveplan.
func commonAxisRevolve(ctx context.Context, op meshbool.OperationKind, env topo.Env,
	a, b *topo.Body, hooks Hooks) (*topo.Body, bool, error) {
	result, admitted, empty, err := revolveplan.PrepareCommonAxis(ctx, op, a, b)
	if err != nil || !admitted {
		return nil, false, err
	}
	if empty {
		return nil, true, hooks.AsBooleanError(op, meshbool.ExpectedBoolean(meshbool.BooleanExpectedEmpty,
			fmt.Errorf("%w: the common-axis result contains no volume", decaderr.ErrBooleanFailed)))
	}
	body, err := revolveplan.Build(ctx, env, result, freeform.NewFreeformWork())
	if err != nil {
		return nil, false, err
	}
	return body, true, nil
}

// AnalyticIntersect is the read-only twin of Body's analytic dispatch for
// Intersect (agent-docs/prism-boolean-design.md §14 PR4; agent-docs/interference-design.md
// §5.2, §12): it runs the same admission and resolution, writes nothing to the
// document and consumes neither operand. ok=false (err nil) means the pair is
// not admitted by the analytic path and the caller falls back to the mesh
// path unchanged. A non-nil err is a genuine analytic-resolution refusal,
// returned as a *meshbool.BooleanExpectedError so the caller can share the mesh
// path's expected-outcome classification.
func AnalyticIntersect(ctx context.Context, env topo.Env, a, b *topo.Body) (*topo.Body, bool, error) {
	pp, ok, err := prismboolean.Try(ctx, meshbool.OpIntersect, a.Payload, b.Payload)
	if err != nil {
		if errors.Is(err, decaderr.ErrUnsupported) {
			return nil, false, meshbool.ExpectedBoolean(meshbool.BooleanExpectedUnsupported, err)
		}
		return nil, false, err
	}
	if !ok {
		return nil, false, nil
	}
	body, err := prismrecord.Build(ctx, env, pp, freeform.NewFreeformWork())
	if err != nil {
		return nil, false, err
	}
	return body, true, nil
}
