package decad

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/verifyrun"
	"github.com/lestrrat-3d/decad/report"

	"github.com/lestrrat-3d/decad/internal/verifyoption"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// This file is the Verify of agent-docs/evaluator-design.md §10/§11 and
// agent-docs/verification-design.md: one non-mutating call returning a rich report
// with a Status at both levels, aggregated by a fixed severity precedence,
// and one bit — Passed() — an agent gates on. The wall, undercut and
// concave-radius questions are answered outright by the analytic surveys of
// internal/verifyrun/survey.go on the bodies this evaluator builds; the staging rule of
// evaluator §11 still governs everything else — an option this evaluator
// cannot ANSWER is accepted, its parameters validated, and the
// asked-but-unanswered question reads Suspect, never a silent pass.
//
// Two sibling files carry the rest, each with its own doc comment: report.go
// the vocabulary the report is written in, and internal/verifyrun/tolerance.go the gate
// that decides which readings are trustworthy. The proof of the reference
// diameter that gate is anchored on is internal/clearance/pairkernel/gate.go.

// VerifyOption configures Verify.
type VerifyOption interface {
	verifyOption()
}

// WallOption parameterizes WithMinWallThickness.
type WallOption interface {
	wallOption()
}

type verifyOptionValue struct{ encoded verifyoption.Token }
type wallOptionValue struct{ encoded verifyoption.WallToken }

func (verifyOptionValue) verifyOption() {}
func (wallOptionValue) wallOption()     {}

type verifyConfig = verifyoption.Config

// WithTolerance sets the relative tolerance gate of verification §2.
func WithTolerance(rel units.Value) VerifyOption {
	return verifyOptionValue{verifyoption.WithTolerance(rel)}
}

// WithMinWallThickness asks whether a wall is thinner than minimum.
func WithMinWallThickness(minimum units.Value, opts ...WallOption) VerifyOption {
	encoded := make([]verifyoption.WallToken, len(opts))
	for i, opt := range opts {
		if value, ok := opt.(wallOptionValue); ok {
			encoded[i] = value.encoded
		}
	}
	return verifyOptionValue{verifyoption.WithMinWallThickness(minimum, encoded...)}
}

// WithDraftAllowance sets the angle separating walls from edges.
func WithDraftAllowance(a units.Value) WallOption {
	return wallOptionValue{verifyoption.WithDraftAllowance(a)}
}

// WithPullDirection asks for undercuts against the stated direction.
func WithPullDirection(v r3.Vec) VerifyOption {
	return verifyOptionValue{verifyoption.WithPullDirection(v)}
}

// WithConcaveRadius asks for the tightest concave radius.
func WithConcaveRadius() VerifyOption {
	return verifyOptionValue{verifyoption.WithConcaveRadius()}
}

// WithClearances asks for minimum gaps between disjoint pairs.
func WithClearances() VerifyOption {
	return verifyOptionValue{verifyoption.WithClearances()}
}

func resolveVerifyOptions(opts []VerifyOption) (verifyConfig, error) {
	encoded := make([]verifyoption.Token, len(opts))
	for i, opt := range opts {
		if opt == nil {
			continue
		}
		value, ok := opt.(verifyOptionValue)
		if !ok {
			return verifyConfig{}, fmt.Errorf("%w: the verification option is not a decad option (%T)", ErrDegenerate, opt)
		}
		encoded[i] = value.encoded
	}
	return verifyoption.Resolve(encoded)
}

// Verify is one non-mutating call over the live model: solidity and boundary
// validity per body, every quantity judged by the tolerance gate, and the
// pairwise partition — proven overlap, proven disjointness, or Suspect
// (verification §1/§6). It mirrors sketch.Verify (core §10).
//
// Verify checks interference even when no options are passed. For each pair of
// proven-valid bodies that cheaper proofs do not settle, its mesh fallback
// checks every pair of operand facet boxes before pruning exact predicates. A
// pair holding a sheet operand never reaches that fallback: it is decided by
// box separation, or, when the boxes meet, the sheet decision procedure of
// agent-docs/surface-design.md §9.3. One pair's
// work can therefore grow with the two facet counts multiplied together, and
// total work also grows with the number of unresolved body pairs. Verify
// proves those pairs on up to GOMAXPROCS goroutines (at most 12) and builds
// the report in pair order, so the report and any returned error match a
// one-pair-at-a-time walk (agent-docs/interference-design.md §2). Large-model
// callers should pass a context with a deadline chosen from representative
// inputs. For a document with live bodies, after document and option
// validation, cancellation returns ctx.Err() and a nil report; validation
// errors take precedence even when ctx is already canceled. An empty document
// retains its Sound result even when the context is already canceled. The
// document remains unchanged. Verify reads the live set once, as it stands
// when the call begins, so a concurrent Remove never changes the bodies one
// call reports on.
func (d *Document) Verify(ctx context.Context, opts ...VerifyOption) (*Report, error) {
	if ctx == nil {
		return nil, fmt.Errorf(`%w: a nil context cannot control verification`, ErrDegenerate)
	}
	if d == nil {
		return nil, fmt.Errorf(`%w: a nil document owns no model`, ErrDegenerate)
	}
	cfg, err := resolveVerifyOptions(opts)
	if err != nil {
		return nil, err
	}
	ctx, err = d.evalContext(ctx)
	if err != nil {
		return nil, err
	}
	found, err := verifyrun.Run(ctx, verifyHooks(), d.n.Live(), cfg)
	if err != nil {
		return nil, err
	}
	return report.LiftVerifyReport(&verifyLift, found), nil
}

// verifyHooks are the services Verify takes from the document: the verified
// mesher, the public Booleans' operand mesher and the handle allocator.
func verifyHooks() verifyrun.Hooks {
	return verifyrun.Hooks{Mesher: verifyAllMesher{}, Draft: verifyAllMesher{}, Operand: pairMeshes{}, Alloc: handleAlloc{}}
}

// verifyLift maps the node form of a verification report to the root handles.
var verifyLift = report.RefLift[*topo.Body, *topo.Face, *Body, *Face]{Body: bodyHandle, Face: faceHandle}
