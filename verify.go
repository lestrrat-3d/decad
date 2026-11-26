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

// WithSkippedPair leaves the unordered pair of a and b out of Verify's
// pairwise partition, for a caller that proves that pair by other means, such
// as ContactPair, or whose bodies share space by design (verification §1.3).
// Verify proves nothing about a skipped pair: it emits no Interference,
// Clearance or pair Diagnostic for it, the pair adds nothing to Status, and
// Report.SkippedPairs names it. Each body's own checks still run. a and b
// MUST be distinct, non-nil live bodies of the verified document; Verify
// rejects any other pair before it does any work, with ErrDegenerate,
// ErrForeignBody or ErrRetiredBody. Naming a pair more than once, in either
// order, skips it once.
func WithSkippedPair(a, b *Body) VerifyOption {
	return skippedPairOption{a: a, b: b}
}

// skippedPairOption carries body handles, which internal/verifyoption cannot
// name: the root gate checks them against the document and hands their nodes
// to internal/verifyrun.
type skippedPairOption struct{ a, b *Body }

func (skippedPairOption) verifyOption() {}

// resolveVerifyOptions validates opts for a Verify call on d. It stays in
// root because WithSkippedPair's bodies are root handles: their document and
// liveness checks (Document.requireLive) read the document's live set.
func (d *Document) resolveVerifyOptions(opts []VerifyOption) (verifyConfig, error) {
	encoded := make([]verifyoption.Token, 0, len(opts))
	var skipped []verifyoption.SkippedPair
	for _, opt := range opts {
		switch value := opt.(type) {
		case nil:
			// verifyoption.Resolve rejects the nil option.
			encoded = append(encoded, nil)
		case verifyOptionValue:
			encoded = append(encoded, value.encoded)
		case skippedPairOption:
			if value.a == value.b && value.a != nil {
				return verifyConfig{}, fmt.Errorf("%w: a skipped pair needs two distinct bodies", ErrDegenerate)
			}
			if err := d.requireLive(value.a); err != nil {
				return verifyConfig{}, err
			}
			if err := d.requireLive(value.b); err != nil {
				return verifyConfig{}, err
			}
			skipped = append(skipped, verifyoption.SkippedPair{A: bodyNode(value.a), B: bodyNode(value.b)})
		default:
			return verifyConfig{}, fmt.Errorf("%w: the verification option is not a decad option (%T)", ErrDegenerate, opt)
		}
	}
	cfg, err := verifyoption.Resolve(encoded)
	if err != nil {
		return verifyConfig{}, err
	}
	cfg.SkippedPairs = skipped
	return cfg, nil
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
	cfg, err := d.resolveVerifyOptions(opts)
	if err != nil {
		return nil, err
	}
	ctx = d.queryContext(ctx)
	ctx, err = d.evalContext(ctx)
	if err != nil {
		return nil, err
	}
	found, err := verifyrun.Run(ctx, verifyHooks(d), d.n.Live(), cfg)
	if err != nil {
		return nil, err
	}
	return report.LiftVerifyReport(&verifyLift, found), nil
}

// verifyHooks are the services Verify takes from the document: the verified
// mesher, the public Booleans' operand mesher, operation history and handle allocator.
func verifyHooks(d *Document) verifyrun.Hooks {
	return verifyrun.Hooks{Mesher: verifyAllMesher{}, Draft: verifyAllMesher{}, Operand: pairMeshes{}, Alloc: handleAlloc{}, History: historyProducers{d}}
}

// verifyLift maps the node form of a verification report to the root handles.
var verifyLift = report.RefLift[*topo.Body, *topo.Face, *Body, *Face]{Body: bodyHandle, Face: faceHandle}
