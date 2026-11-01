// Package booleaneval is the mesh-boolean pipeline of
// agent-docs/evaluator-design.md §9: the two operands' tessellations meet in exact
// triangle/triangle intersection segments, each cut facet is subdivided along
// them (internal/meshbool/boolean_cut.go), every resulting piece is classified
// against the other solid by an exact local side-of-contact test (or exact ray
// parity for the uncut regions), the kept pieces are stitched by shared exact
// vertices, and a conforming pass plus a closure audit prove the result
// watertight by construction — an audit failure is an error, never a wrong
// mesh.
package booleaneval

import (
	"context"
	"errors"
	"fmt"
	"math"
	"math/big"
	"slices"
	"unsafe"

	"github.com/lestrrat-3d/r3"

	"github.com/lestrrat-3d/decad/internal/booloperand"
	"github.com/lestrrat-3d/decad/internal/chaintol"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/facetproof"
	"github.com/lestrrat-3d/decad/internal/gaptol"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/opdiagnostic"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
)

// OperandMesher supplies the VerifyAll mesh the evaluation composes for one
// operand, and whether the operand restates a held mesh (booloperand.HeldFloor),
// which arms the local chain-depth gate at the pair tolerance. pairTol is the
// pair's own chord tolerance (booloperand.PairChord). Every implementation
// returns a mesh tessellated at VerifyAll, so the mesh carries its own proven
// bounds whatever tolerance it was asked for; the composition reads only those
// bounds, never the request.
type OperandMesher interface {
	TessellateOperand(ctx context.Context, b *topo.Body, pairTol float64) (*topo.Mesh, bool, error)
}

// Evaluation is the geometry-only result of a mesh Boolean: the faceted
// payload with the occupied volume, its proven bound and exact value, and the
// closure audit. It holds no document reference or public model state.
type Evaluation struct {
	circleSeal       *circleCandidateSeal
	Payload          facetedrecord.Payload
	Volume           float64
	VolumeBound      float64
	VolumeRat        *big.Rat
	Audit            *facetproof.MeshAudit
	Shift            *ShiftSummary
	ShiftCertificate *ShiftCertificate
	Chain            *ChainAllowanceSummary
	Chains           []ChainAllowanceSummary
	EmbeddingReach   *meshbool.EmbeddingReach
}

// ChainAllowanceSummary records a local gate that the configured allowance
// raised above the pair's ordinary chord limit.
type ChainAllowanceSummary struct {
	Operand        int
	TouchedBound   float64
	PairChordLimit float64
	ChainAllowance float64
}

// ShiftSummary describes the accepted shifted retry, including coordinate
// rounding in the charge on the moved mesh pieces.
type ShiftSummary struct {
	DisplacementBound float64
	AllowedReach      float64
}

// Diagnostics returns copied summaries from this accepted evaluation. Callers
// that combine private steps must aggregate these records at their public
// boundary and discard them if the enclosing operation fails.
func (e Evaluation) Diagnostics(op meshbool.OperationKind) []opdiagnostic.Record {
	var records []opdiagnostic.Record
	if e.Shift != nil {
		records = append(records, opdiagnostic.Record{
			Operation: int(op), Code: opdiagnostic.CodeGapShiftApplied, Operand: 1,
			Change:            opdiagnostic.ChangeGapShift,
			DisplacementBound: e.Shift.DisplacementBound,
			AllowedReach:      e.Shift.AllowedReach,
		})
	}
	chains := e.Chains
	if chains == nil && e.Chain != nil {
		chains = []ChainAllowanceSummary{*e.Chain}
	}
	for _, chain := range chains {
		allowance := chain.ChainAllowance
		records = append(records, opdiagnostic.Record{
			Operation: int(op), Code: opdiagnostic.CodeChainAllowanceApplied,
			Operand: chain.Operand, TouchedBound: chain.TouchedBound,
			PairChordLimit: chain.PairChordLimit, ChainAllowance: &allowance,
		})
	}
	return append(records, e.EmbeddingReach.Diagnostics()...)
}

// VolumeReading reads the audited mesh's exact component volumes and composes
// the symmetric-difference allowance with final float rounding.
func VolumeReading(ctx context.Context, audit *facetproof.MeshAudit, volSymDiff float64) (float64, float64, *big.Rat, error) {
	if err := ctx.Err(); err != nil {
		return 0, 0, nil, err
	}
	volF, bound, volRat, err := facetproof.VolumeFromAudit(audit, volSymDiff)
	if ctxErr := ctx.Err(); ctxErr != nil {
		return 0, 0, nil, ctxErr
	}
	if err != nil {
		return 0, 0, nil, err
	}
	return volF, bound, volRat, nil
}

// Evaluate runs the mesh Boolean of a and b under op. Both operands are meshed
// at the pair's own chord, scaled by chordScale; capFace and coneFace name the
// cap and cone faces whose contact the proximity gate exempts for a
// cone-tangent tooth (nil for every other Boolean).
//
// An operand pair whose contact the exact predicates cannot classify is tried
// again with b moved by up to the document's gap tolerance
// (agent-docs/gap-tolerance-design.md §5.2).
func Evaluate(ctx context.Context, op meshbool.OperationKind, a, b *topo.Body,
	meshes OperandMesher, chordScale float64, capFace, coneFace *topo.Face) (Evaluation, error) {
	twins := &operandTwins{}
	ev, err := evaluate(ctx, op, a, b, meshes, chordScale, capFace, coneFace, twins, nil, nil, nil, nil, 0)
	if err == nil {
		return ev, nil
	}
	var touch *meshbool.TouchContactError
	if !errors.As(err, &touch) || !(gaptol.Reach(ctx) > 0) {
		return ev, err
	}
	tolMM, dPair, perr := booloperand.PairChord(ctx, a, b)
	if perr != nil {
		return ev, err
	}
	mb, _, terr := meshes.TessellateOperand(ctx, b, operandChord(ctx, b, tolMM*chordScale, dPair, chordScale))
	if terr != nil {
		return ev, err
	}
	return retryShifted(ctx, err, op, piecesOf(mb), func(shifts map[int]pieceShift) (Evaluation, error) {
		return evaluate(ctx, op, a, b, meshes, chordScale, capFace, coneFace, twins, shifts, nil, nil, nil, 0)
	})
}

// EvaluateReplay runs the private mesh-only attempt and its shifted retries
// on one counter. Unsupported operand and placement branches refuse.
func EvaluateReplay(ctx context.Context, op meshbool.OperationKind, a, b *topo.Body, chordScale float64,
	budget *proofbound.ReplayBudget) (Evaluation, error) {
	return evaluateReplay(ctx, op, a, b, chordScale, 0, 0, nil, budget)
}

// EvaluateReplayClamped binds every attempt to the authenticated pair chord.
func EvaluateReplayClamped(ctx context.Context, op meshbool.OperationKind, a, b *topo.Body, chordScale, candidate, original float64, budget *proofbound.ReplayBudget) (Evaluation, error) {
	if op != meshbool.OpUnion || !(candidate > 0) || !(original > 0) || !(chordScale > 0) || chordScale > 1 || proofbound.IsNonFinite(candidate) || proofbound.IsNonFinite(original) || proofbound.IsNonFinite(chordScale) || candidate*chordScale > original {
		return Evaluation{}, fmt.Errorf("%w: invalid replay chord clamp", decaderr.ErrUnsupported)
	}
	return evaluateReplay(ctx, op, a, b, chordScale, candidate, original, nil, budget)
}

// EvaluateReplayRetainedClamped uses the authenticated original target mesh.
func EvaluateReplayRetainedClamped(ctx context.Context, a, b *topo.Body, chordScale, candidate, original float64, retained *RetainedTarget, budget *proofbound.ReplayBudget) (Evaluation, error) {
	if retained == nil || retained.body != a || !(candidate > 0) || !(original > 0) || !(chordScale > 0) || chordScale > 1 || proofbound.IsNonFinite(candidate) || proofbound.IsNonFinite(original) || proofbound.IsNonFinite(chordScale) || candidate*chordScale > original {
		return Evaluation{}, fmt.Errorf("%w: invalid retained replay", decaderr.ErrUnsupported)
	}
	return evaluateReplay(ctx, meshbool.OpUnion, a, b, chordScale, candidate, original, retained, budget)
}

func evaluateReplay(ctx context.Context, op meshbool.OperationKind, a, b *topo.Body, chordScale, candidate, original float64, retained *RetainedTarget, budget *proofbound.ReplayBudget) (Evaluation, error) {
	if budget == nil || a == nil || b == nil {
		return Evaluation{}, fmt.Errorf("%w: bounded evaluation needs two operands and a budget", decaderr.ErrUnsupported)
	}
	check := func() error {
		if cert, ok := ctx.Value(preparedTagShiftKey{}).(*BrakeCandidateCertificate); ok {
			if err := cert.validate(ctx, a, b, budget); err != nil {
				return err
			}
		}
		if candidate == 0 {
			return nil
		}
		got, _, err := booloperand.PairChordReplay(ctx, a, b, budget)
		if err != nil {
			return err
		}
		if math.Float64bits(got) != math.Float64bits(candidate) || got*chordScale > original {
			return fmt.Errorf("%w: replay pair chord changed", decaderr.ErrUnsupported)
		}
		return nil
	}
	if err := check(); err != nil {
		return Evaluation{}, err
	}
	meshes := &retainedReplayMesher{replayMesher: replayMesher{budget: budget}, target: retained} //nolint:modernize // Named fields keep the embedded budget explicit.
	twins := &operandTwins{}
	ev, err := evaluate(ctx, op, a, b, meshes, chordScale, nil, nil, twins, nil, nil, budget, nil, 0)
	if err == nil {
		return ev, nil
	}
	var touch *meshbool.TouchContactError
	if !errors.As(err, &touch) || !(gaptol.Reach(ctx) > 0) {
		return ev, err
	}
	tol, dPair, perr := booloperand.PairChordReplay(ctx, a, b, budget)
	if perr != nil {
		return Evaluation{}, perr
	}
	chord, perr := operandChordReplay(ctx, b, tol*chordScale, dPair, chordScale, budget)
	if perr != nil {
		return Evaluation{}, perr
	}
	mb, _, perr := meshes.TessellateOperand(ctx, b, chord)
	if perr != nil {
		return Evaluation{}, perr
	}
	pc, perr := piecesOfReplay(ctx, mb, budget)
	if perr != nil {
		return Evaluation{}, perr
	}
	return replayRetryShifted(ctx, err, op, pc, func(shifts *ReplayShifts) (Evaluation, error) {
		if err := check(); err != nil {
			return Evaluation{}, err
		}
		return evaluate(ctx, op, a, b, meshes, chordScale, nil, nil, twins, nil, shifts, budget, nil, 0)
	}, budget)
}

// EvaluateReplayFixedCut uses the authenticated moved tool exactly once.
func EvaluateReplayFixedCut(ctx context.Context, a, b *topo.Body, tool *FixedShiftedTool, targetAllowance float64, budget *proofbound.ReplayBudget) (Evaluation, error) {
	if budget == nil || a == nil || b == nil || tool == nil || tool.tool != b || targetAllowance < 0 || proofbound.IsNonFinite(targetAllowance) || chaintol.Of(ctx) != 0 {
		return Evaluation{}, fmt.Errorf("%w: fixed shifted Cut lacks authenticated operands or allowance", decaderr.ErrUnsupported)
	}
	base, _, err := booloperand.PairChordReplay(ctx, a, b, budget)
	if err != nil {
		return Evaluation{}, err
	}
	ev, err := evaluate(ctx, meshbool.OpCut, a, b, replayMesher{budget: budget}, 1, nil, nil, &operandTwins{}, nil, nil, budget, tool, targetAllowance)
	if err != nil {
		return ev, err
	}
	after, _, err := booloperand.PairChordReplay(ctx, a, b, budget)
	if err != nil {
		return Evaluation{}, err
	}
	if !(base > 0) || proofbound.IsNonFinite(base) || math.Float64bits(base) != math.Float64bits(after) {
		return Evaluation{}, fmt.Errorf("%w: final Cut chord changed", decaderr.ErrUnsupported)
	}
	return ev, nil
}

// evaluate is Evaluate's single attempt, with the pieces of b's mesh moved by
// shifts when there are any. twins caches the operands' identity rebuilds
// across attempts.
func evaluate(ctx context.Context, op meshbool.OperationKind, a, b *topo.Body,
	meshes OperandMesher, chordScale float64, capFace, coneFace *topo.Face, twins *operandTwins,
	shifts map[int]pieceShift, ordered *ReplayShifts, replay *proofbound.ReplayBudget, fixed *FixedShiftedTool, targetAllowance float64) (Evaluation, error) {
	hasShifts := len(shifts) > 0 || (ordered != nil && len(ordered.entries) > 0)
	if err := ctx.Err(); err != nil {
		return Evaluation{}, err
	}

	var tolMM, dPair float64
	var err error
	if replay != nil {
		tolMM, dPair, err = booloperand.PairChordReplay(ctx, a, b, replay)
	} else {
		tolMM, dPair, err = booloperand.PairChord(ctx, a, b)
	}
	if err != nil {
		return Evaluation{}, err
	}
	tolMM *= chordScale
	if err := ctx.Err(); err != nil {
		return Evaluation{}, err
	}
	// A payload whose mesh cannot carry an occupied-volume proof is refused
	// BEFORE it is meshed. meshbool.OperandSymDiff is still the gate that decides
	// it — this only asks the same question of the payload class, where the
	// answer is already known, so a refusal costs a map lookup instead of a
	// complete tessellation and its facet-pair audit.
	if err := booloperand.RequireVolumeProof(ctx, a, 0); err != nil {
		return Evaluation{}, err
	}
	if err := booloperand.RequireVolumeProof(ctx, b, 1); err != nil {
		return Evaluation{}, err
	}
	// VerifyAll, passed explicitly rather than taken from Tessellate's default
	// (agent-docs/tessellation-design.md §11 step 1) by every operandMeshes: the
	// composition below reads every proof a mesh can carry, and the one-entry
	// cache keys on the level, so a caller's own earlier unverified mesh at
	// this same internal tolerance is a different entry and is never handed
	// back here.
	chordA := 0.0
	if replay != nil {
		chordA, err = operandChordReplay(ctx, a, tolMM, dPair, chordScale, replay)
		if err != nil {
			return Evaluation{}, err
		}
	} else {
		chordA = operandChord(ctx, a, tolMM, dPair, chordScale)
	}
	ma, restatingA, err := meshes.TessellateOperand(ctx, a, chordA)
	if err != nil {
		if _, ok := errors.AsType[*tessellation.ExpectedError](err); ok {
			err = meshbool.ExpectedBoolean(meshbool.BooleanExpectedCoarseTessellation, err)
		} else if errors.Is(err, decaderr.ErrUnsupported) {
			// A tessellation decaderr.ErrUnsupported is a capability/staging limit on the
			// operand itself, reached before any contact is examined — never a
			// contact refusal (see meshbool.BooleanExpectedStaging).
			err = meshbool.ExpectedBooleanForOperand(meshbool.BooleanExpectedStaging, 0, err)
		}
		return Evaluation{}, err
	}
	if err := ctx.Err(); err != nil {
		return Evaluation{}, err
	}
	chordB := 0.0
	if replay != nil {
		chordB, err = operandChordReplay(ctx, b, tolMM, dPair, chordScale, replay)
		if err != nil {
			return Evaluation{}, err
		}
	} else {
		chordB = operandChord(ctx, b, tolMM, dPair, chordScale)
	}
	var mb *topo.Mesh
	var restatingB bool
	if fixed != nil {
		mb = fixed.charged
		restatingB = true
	} else {
		mb, restatingB, err = meshes.TessellateOperand(ctx, b, chordB)
	}
	if err != nil {
		if _, ok := errors.AsType[*tessellation.ExpectedError](err); ok {
			err = meshbool.ExpectedBoolean(meshbool.BooleanExpectedCoarseTessellation, err)
		} else if errors.Is(err, decaderr.ErrUnsupported) {
			// A tessellation decaderr.ErrUnsupported is a capability/staging limit on the
			// operand itself, reached before any contact is examined — never a
			// contact refusal (see meshbool.BooleanExpectedStaging).
			err = meshbool.ExpectedBooleanForOperand(meshbool.BooleanExpectedStaging, 1, err)
		}
		return Evaluation{}, err
	}
	if err := ctx.Err(); err != nil {
		return Evaluation{}, err
	}

	// Global source-face ids: the operands' faces in order, so the payload
	// records provenance without holding live pointers.
	var facesA, facesB []*topo.Face
	if replay != nil {
		facesA, err = facesReplay(ctx, a, replay)
		if err != nil {
			return Evaluation{}, err
		}
		facesB, err = facesReplay(ctx, b, replay)
		if err != nil {
			return Evaluation{}, err
		}
		for _, faces := range [2][]*topo.Face{facesA, facesB} {
			for _, f := range faces {
				if f == nil {
					return Evaluation{}, fmt.Errorf("%w: nil source face", decaderr.ErrUnsupported)
				}
				if err := replay.SpendWork(ctx, 1); err != nil {
					return Evaluation{}, err
				}
				if err := replay.ReserveRecords(ctx, uint64(len(f.Origins)+len(f.Upstream))); err != nil {
					return Evaluation{}, err
				}
			}
		}
		if err := replay.ReserveRecords(ctx, 4*uint64(len(facesA)+len(facesB))+uint64(len(ma.Source)+len(mb.Source))+4); err != nil {
			return Evaluation{}, err
		}
		if err := replay.SpendWork(ctx, 2*uint64(len(facesA)+len(facesB))+uint64(len(ma.Source)+len(mb.Source))); err != nil {
			return Evaluation{}, err
		}
	} else {
		facesA = topobuild.Faces(a)
		facesB = topobuild.Faces(b)
	}
	targetFaces := len(facesA)
	totalFaces := targetFaces + len(facesB)
	if replay != nil {
		if err := replay.ReserveArray(ctx, uint64(totalFaces), uint64(unsafe.Sizeof(facetedrecord.Group{}))); err != nil {
			return Evaluation{}, err
		}
	}
	groups := make([]facetedrecord.Group, 0, totalFaces)
	var sourceID map[*topo.Face]int
	var fixedSourceID *topo.FaceBoundTable
	if replay != nil {
		fixedSourceID, err = topo.NewFaceBoundTable(ctx, totalFaces, replay)
		if err != nil {
			return Evaluation{}, err
		}
	} else {
		sourceID = make(map[*topo.Face]int, totalFaces)
	}
	var (
		piecesB        pieces
		facePiece      map[*topo.Face]int
		fixedFacePiece *topo.FaceBoundTable
		reachB         float64
	)
	if hasShifts {
		if replay != nil {
			work, ok := proofbound.WallCheckedMul(uint64(len(facesB)), uint64(len(mb.Vertices)))
			if !ok {
				return Evaluation{}, replay.SpendWork(ctx, proofbound.MaxMultiwayWorkPerCall+1)
			}
			if err := replay.SpendWork(ctx, work); err != nil {
				return Evaluation{}, err
			}
		}
		if replay != nil {
			piecesB, err = piecesOfReplay(ctx, mb, replay)
			if err != nil {
				return Evaluation{}, err
			}

			if err := replay.SpendWork(ctx, 4*uint64(len(mb.Triangles))+2*uint64(len(mb.Vertices))); err != nil {
				return Evaluation{}, err
			}
			if err := replay.ReserveRecords(ctx, uint64(len(mb.Vertices)+len(mb.Triangles)+len(mb.FaceBound))); err != nil {
				return Evaluation{}, err
			}
		} else {
			piecesB = piecesOf(mb)
		}
		if replay != nil {
			fixedFacePiece, err = replayFacePieces(ctx, replay, mb, piecesB)
			if err != nil {
				return Evaluation{}, err
			}
			var beta []float64
			beta, err = topobuild.VertexBoundsReplay(ctx, mb, replay)
			if err != nil {
				return Evaluation{}, err
			}
			reachB = mb.Bound
			for _, v := range beta {
				if err := replay.SpendWork(ctx, 1); err != nil {
					return Evaluation{}, err
				}
				reachB = max(reachB, v)
			}
			reachB = proofbound.AbsSumUpper(reachB, mb.Bound)
		} else {
			facePiece = facePieces(mb, piecesB)
			reachB, err = tagReach(mb)
		}
		if err != nil {
			return Evaluation{}, err
		}
	}
	// A placed analytic operand's tags are float images of its record's true
	// surfaces; each states how far it may lie off its own.
	var placed [2]placedDepartures
	for k, m := range [2]*topo.Mesh{ma, mb} {
		owner := a
		if k == 1 {
			owner = b
		}
		if fixed != nil && k == 1 {
			continue
		}
		if replay != nil {
			if _, faceted := owner.Payload.(facetedrecord.Payload); !faceted && owner.Payload.Transform() != r3.Identity() {
				if err := replay.SpendWork(ctx, 8); err != nil {
					return Evaluation{}, err
				}
				faces := facesA
				if k == 1 {
					faces = facesB
				}
				candidate, candidateOK := meshes.(brakeCircleCandidateMesher)
				_, retainedOK := meshes.(*retainedReplayMesher)
				pp, prismOK := owner.Payload.(prismrecord.Payload)
				circle := false
				if prismOK && len(pp.Profile.Outer.Segments) == 1 {
					_, circle = pp.Profile.Outer.Segments[0].(sectionrecord.CircleSeg)
				}
				if candidateOK && candidate.pin == owner || retainedOK && k == 0 && circle && replay.IsBrakeReplay() {
					placed[k], err = replayPlacedCircleDepartures(ctx, owner, m, faces, replay)
				} else {
					placed[k], err = replayPlacedRectangleDepartures(ctx, owner, m, faces, replay)
				}
				if err != nil {
					return Evaluation{}, err
				}
				continue
			}
		}
		twin, err := twins.of(ctx, k, owner)
		if err != nil {
			return Evaluation{}, err
		}
		if placed[k], err = placedOperandDepartures(owner, twin, m); err != nil {
			return Evaluation{}, err
		}
	}
	var prepared *preparedTagShift
	if cert, _ := ctx.Value(preparedTagShiftKey{}).(*BrakeCandidateCertificate); cert != nil && replay != nil && ordered != nil && hasShifts {
		prepared, err = prepareTagShift(ctx, a, b, mb, facesB, piecesB, fixedFacePiece, placed[1], reachB, ordered, replay)
		if err != nil {
			return Evaluation{}, err
		}
	}
	for i := range totalFaces {
		var f *topo.Face
		if i < targetFaces {
			f = facesA[i]
		} else {
			f = facesB[i-targetFaces]
		}
		if i%256 == 0 {
			if err := ctx.Err(); err != nil {
				return Evaluation{}, err
			}
		}
		if replay != nil {
			if err := fixedSourceID.Set(ctx, f, float64(len(groups)), replay); err != nil {
				return Evaluation{}, err
			}
			for _, n := range []int{len(f.Origins), len(f.Upstream)} {
				if err := replay.ReserveArray(ctx, uint64(n), uint64(unsafe.Sizeof(provenance.Ref{}))); err != nil {
					return Evaluation{}, err
				}
			}
		} else {
			sourceID[f] = len(groups)
		}
		operand, ordinal, owner := 0, i, a
		if i >= targetFaces {
			operand, ordinal, owner = 1, i-targetFaces, b
		}
		// The tool of a Cut (every face of b) leaves its natural normal
		// pointing into the result (agent-docs/faceted-source-retention-design.md §3.1).
		if replay != nil {
			if err := replay.ReserveArray(ctx, 1, uint64(unsafe.Sizeof(facetedrecord.SourcePatch{}))); err != nil {
				return Evaluation{}, err
			}
		}
		group := sourceGroup(owner, operand, ordinal, f, op == meshbool.OpCut && i >= targetFaces, placed[operand])
		if fixed != nil && i >= targetFaces {
			if ordinal >= len(fixed.groups) {
				return Evaluation{}, fmt.Errorf("%w: shifted tool source groups changed", decaderr.ErrUnsupported)
			}
			group = fixed.groups[ordinal]
		}
		if hasShifts && i >= targetFaces {
			// A moved piece's true faces lie on its tags translated by the
			// shift, never on the unmoved tags.
			piece, ok := facePiece[f]
			if replay != nil {
				var v float64
				v, ok, err = fixedFacePiece.Lookup(ctx, f, replay)
				if err != nil {
					return Evaluation{}, err
				}
				piece = int(v)
			}
			if ordered != nil {
				sh, moved, e := ordered.get(ctx, piece)
				if e != nil {
					return Evaluation{}, e
				}
				if prepared != nil {
					group = prepared.groups[ordinal]
				} else if moved && ok && piece != spanningPieces && sh.Grow == 0 && group.Surface != nil && group.Retained && group.Denoted == nil {
					return Evaluation{}, fmt.Errorf("%w: replay moved retained-tag proof is not bounded", decaderr.ErrUnsupported)
				} else {
					group = movedGroupWithShift(group, piece, ok, sh, moved, mb.Vertices, reachB)
				}
			} else {
				group = movedGroup(group, piece, ok, shifts, mb.Vertices, reachB)
			}
		}
		groups = append(groups, group)
	}
	var srcA, srcB []int
	if replay != nil {
		srcA, err = replaySourceIDs(ctx, replay, ma.Source, fixedSourceID)
	} else {
		srcA, err = meshbool.SourceIDs(ctx, ma.Source, sourceID)
	}
	if err != nil {
		return Evaluation{}, err
	}
	if replay != nil {
		srcB, err = replaySourceIDs(ctx, replay, mb.Source, fixedSourceID)
	} else {
		srcB, err = meshbool.SourceIDs(ctx, mb.Source, sourceID)
	}
	if err != nil {
		return Evaluation{}, err
	}
	if err := ctx.Err(); err != nil {
		return Evaluation{}, err
	}
	// mbDecide is the mesh the contact gate reads: mb itself, unless b moved.
	mbDecide := mb
	// unmoved keeps b's own mesh: the chain gate's allowance is how far the
	// charged copy's bound grew past it.
	unmoved := mb
	if fixed != nil {
		mbDecide = fixed.decision
		unmoved = fixed.selected
	}
	shiftDisplacement := 0.0
	var shrink uniformShrink
	if hasShifts {
		// The cap and cone certificates authenticate the operands' own
		// geometry, which a moved operand no longer has.
		if capFace != nil || coneFace != nil {
			return Evaluation{}, errShiftDeclined
		}
		if prepared != nil {
			if err := prepared.recheck(ctx, a, b, mb, facesB, piecesB, fixedFacePiece, ordered, replay); err != nil {
				return Evaluation{}, err
			}
			mbDecide, mb, shiftDisplacement = prepared.decision, prepared.charged, prepared.displacement
		} else if replay != nil {
			if ordered != nil {
				mbDecide, mb, shiftDisplacement, err = shiftOperandMeshReplay(ctx, mb, piecesB, ordered, &shrink, replay)
			} else {
				mbDecide, mb, shiftDisplacement, err = shiftOperandMeshWithBudget(ctx, mb, piecesB, shifts, &shrink, replay)
			}
		} else {
			mbDecide, mb, shiftDisplacement, err = shiftOperandMesh(ctx, mb, piecesB, shifts, &shrink)
		}
		if err != nil {
			return Evaluation{}, err
		}
	}
	var bmA *meshbool.BoolMesh
	if replay != nil {
		if cached, ok := meshes.(*retainedReplayMesher); ok && hasShifts && cached.target != nil && cached.target.body == a {
			bmA, err = cached.prepareTarget(ctx, a, ma, chordA, srcA, !retainedPlaneSlabDisabled(ctx), replay)
		} else {
			bmA, err = PrepBoolMeshReplay(ctx, ma, srcA, !retainedPlaneSlabDisabled(ctx), replay)
		}
	} else {
		bmA, err = PrepBoolMesh(ctx, ma, srcA)
	}
	if err != nil {
		if errors.Is(err, decaderr.ErrUnsupported) {
			err = meshbool.ExpectedBoolean(meshbool.BooleanExpectedUnsupported, err)
		}
		return Evaluation{}, err
	}
	var bmB *meshbool.BoolMesh
	if replay != nil {
		bmB, err = PrepBoolMeshReplay(ctx, mb, srcB, !hasShifts && !retainedPlaneSlabDisabled(ctx), replay)
	} else if hasShifts {
		bmB, err = prepBoolMesh(ctx, mb, srcB, func(int) (r3.Frame, float64, bool) { return r3.Frame{}, 0, false })
	} else {
		bmB, err = PrepBoolMesh(ctx, mb, srcB)
	}
	if err != nil {
		if errors.Is(err, decaderr.ErrUnsupported) {
			err = meshbool.ExpectedBoolean(meshbool.BooleanExpectedUnsupported, err)
		}
		return Evaluation{}, err
	}
	// A restating operand's facets where the pair meets must hold within the
	// pair tolerance; the gate reads them on the contact classification below
	// (agent-docs/faceted-vertex-bounds-design.md §5). Any other operand is ungated.
	chain := chaintol.Of(ctx)
	if restatingA {
		if fixed != nil {
			configureChainGate(bmA, tolMM, proofbound.AbsSumUpper(tolMM, targetAllowance))
		} else {
			configureChainGate(bmA, tolMM, chain)
		}
	}
	if restatingB {
		// A moved operand's facets carry the move in their bounds, which the
		// gate would read as a coarse mesh. The move is what the gap tolerance
		// permits and the composition of the result's bounds charges it, so the
		// gate allows the growth of the mesh bound the move charged.
		if fixed != nil || replay != nil && hasShifts {
			configureChainGate(bmB, tolMM, proofbound.AbsSumUpper(tolMM, mb.Bound-unmoved.Bound))
		} else {
			configureChainGate(bmB, tolMM+(mb.Bound-unmoved.Bound), chain)
		}
	}
	if err := ctx.Err(); err != nil {
		return Evaluation{}, err
	}
	// One memo per call, over exactly the two prepared operand tessellations
	// the gate and the mesh pass both walk; it is dropped when the call
	// returns (meshbool.ContactMemo).
	var memo *meshbool.ContactMemo
	if replay != nil {
		memo, err = meshbool.NewReplayContactMemo(ctx, bmA, bmB, replay)
		if err != nil {
			return Evaluation{}, err
		}
	} else {
		memo = meshbool.NewContactMemo(bmA, bmB)
	}
	var proximityErr error
	if replay != nil {
		proximityErr = meshbool.RefuseUndecidableProximityReplay(ctx, ma, mbDecide, bmA, bmB, memo, capFace, coneFace, replay)
	} else {
		proximityErr = meshbool.RefuseUndecidableProximity(ctx, ma, mbDecide, bmA, bmB, memo, capFace, coneFace)
	}
	if err := proximityErr; err != nil {
		err = meshbool.StagingError(op, err)
		if errors.Is(err, decaderr.ErrUnsupported) {
			err = meshbool.ExpectedBoolean(meshbool.BooleanExpectedContact, err)
		}
		return Evaluation{}, err
	}
	if err := ctx.Err(); err != nil {
		return Evaluation{}, err
	}
	var kept []meshbool.KeptFacet
	var maxRim float64
	var replayKeys *meshbool.ReplayKeys
	if replay != nil {
		replayKeys, err = meshbool.NewReplayKeysBounded(ctx, replay)
		if err != nil {
			return Evaluation{}, err
		}
		kept, maxRim, err = meshbool.MeshBooleanReplayWithKeys(ctx, op, bmA, bmB, memo, replay, replayKeys)
	} else {
		kept, maxRim, err = meshbool.MeshBoolean(ctx, op, bmA, bmB, memo)
	}
	if err != nil {
		return Evaluation{}, meshbool.StagingError(op, err)
	}
	if len(kept) == 0 {
		err := fmt.Errorf(`%w: the %s result is empty`, decaderr.ErrBooleanFailed, op)
		if hasShifts {
			return Evaluation{}, meshbool.ExpectedBooleanAfterShift(err, shiftDisplacement, gaptol.Reach(ctx))
		}
		return Evaluation{}, meshbool.ExpectedBoolean(meshbool.BooleanExpectedEmpty, err)
	}
	if err := ctx.Err(); err != nil {
		return Evaluation{}, err
	}
	var stitched *meshbool.StitchedMesh
	if replay != nil {
		stitched, err = meshbool.StitchFacetsReplayWithKeys(ctx, kept, replay, replayKeys)
	} else {
		stitched, err = meshbool.StitchFacetsContext(ctx, kept)
	}
	if err != nil {
		if errors.Is(err, decaderr.ErrUnsupported) {
			err = meshbool.ExpectedBoolean(meshbool.BooleanExpectedUnsupported, err)
		}
		return Evaluation{}, err
	}
	if err := ctx.Err(); err != nil {
		return Evaluation{}, err
	}

	// Bound composition (§9, the verification-design shapes; the helpers own
	// every mechanism — internal/proofbound/bounds.go). The volume error obeys the
	// symmetric-difference identity |1_{A∘B} − 1_{A'∘B'}| ≤ |1_A − 1_A'| +
	// |1_B − 1_B'| for all three ops, so it is the sum of the operands' own
	// symmetric-difference bounds — each δ · (that operand's held area) — plus
	// what the final float rounding can move. The BOUNDARY bound is a different
	// question, answered per vertex (agent-docs/faceted-vertex-bounds-design.md §3):
	// a vertex the boolean itself creates sits at the crossing of two chord
	// PLANES, so it is displaced by its own facet pair's trim-amplified
	// (δ(t_A) + δ(t_B))/sin θ, not by δ, and the stitch adds each vertex's own
	// weld. The amplification is refused where it reaches the pair diameter.
	if err := meshbool.RefuseRimPastPair(maxRim, dPair); err != nil {
		return Evaluation{}, meshbool.ExpectedBoolean(meshbool.BooleanExpectedUnsupported, err)
	}
	symA, err := meshbool.OperandSymDiff(ma.BoundaryOK, ma.SymDiffOK, ma.VolSymDiff)
	if err != nil {
		return Evaluation{}, meshbool.ExpectedBooleanForOperand(meshbool.BooleanExpectedVolumeProof, 0, err)
	}
	symB, err := meshbool.OperandSymDiff(mb.BoundaryOK, mb.SymDiffOK, mb.VolSymDiff)
	if err != nil {
		return Evaluation{}, meshbool.ExpectedBooleanForOperand(meshbool.BooleanExpectedVolumeProof, 1, err)
	}
	// The final rounding's own volume error is what its vertex displacement
	// sweeps out over the surface it acted on — the stitched surface BEFORE the
	// weld dropped any collapsed facet from it (StitchedMesh.RoundingVolumeBound).
	// The held mesh's area is the WRONG yardstick here: a dropped facet is
	// missing from it, and its swept volume would go uncharged. The area the
	// weld dropped is likewise missing from every area the result reports, so
	// it joins the operands' own chord deficit in areaSlack.
	roundVol := stitched.RoundingVolumeBound()
	volSymDiff, areaSlack := booleanProofBounds(
		symA, symB, roundVol,
		ma.AreaSlack, mb.AreaSlack, stitched.DropArea,
	)
	if replay != nil {
		if err := replay.SpendWork(ctx, uint64(len(stitched.Tris))); err != nil {
			return Evaluation{}, err
		}
	}
	payload := facetedrecord.Payload{
		Verts:       stitched.Verts,
		VertexBound: stitched.VertexBound,
		Tris:        stitched.Tris,
		Src:         stitched.Src,
		Groups:      groups,
		MeshBound:   facetproof.FacetBoundMax(stitched.Tris, stitched.VertexBound),
		VolSymDiff:  volSymDiff,
		AreaSlack:   areaSlack,
		DPair:       dPair,
		Xform:       r3.Identity(),
	}

	if err := ctx.Err(); err != nil {
		return Evaluation{}, err
	}
	var audit *facetproof.MeshAudit
	if replay != nil {
		var ok bool
		audit, ok, err = facetproof.AuditFacetedMeshReplay(ctx, replay.WorkBudget(ctx), payload.Verts, payload.Tris)
		if err == nil && !ok {
			err = fmt.Errorf("%w: bounded result audit refused", decaderr.ErrUnsupported)
		}
	} else {
		audit, err = facetproof.AuditFacetedMesh(ctx, payload.Verts, payload.Tris)
	}
	if err != nil {
		return Evaluation{}, err
	}
	var volume, volumeBound float64
	var volumeRat *big.Rat
	if replay != nil {
		var ok bool
		volume, volumeBound, volumeRat, ok, err = facetproof.VolumeFromAuditReplay(ctx, replay.WorkBudget(ctx), audit, payload.VolSymDiff)
		if err == nil && !ok {
			err = fmt.Errorf("%w: bounded volume proof refused", decaderr.ErrUnsupported)
		}
	} else {
		volume, volumeBound, volumeRat, err = VolumeReading(ctx, audit, payload.VolSymDiff)
	}
	if err != nil {
		return Evaluation{}, err
	}
	// The mesh audit and occupied-volume reading certify this Boolean result
	// as a source for a later positive similarity. No unbuilt result is marked.
	payload.BooleanSimilaritySource = true
	if err := ctx.Err(); err != nil {
		return Evaluation{}, err
	}
	ev := Evaluation{Payload: payload, Volume: volume, VolumeBound: volumeBound, VolumeRat: volumeRat, Audit: audit}
	ev.EmbeddingReach = stitched.EmbeddingReach
	if hasShifts {
		if replay != nil {
			if err := replay.ReserveArray(ctx, 1, uint64(unsafe.Sizeof(ShiftSummary{}))); err != nil {
				return Evaluation{}, err
			}
		}
		ev.Shift = &ShiftSummary{DisplacementBound: shiftDisplacement, AllowedReach: gaptol.Reach(ctx)}
		if replay != nil {
			ev.ShiftCertificate, err = captureReplayShift(ctx, a, b, unmoved, mbDecide, mb, piecesB, groups[targetFaces:], ordered, tolMM, chordScale, shiftDisplacement, replay)
			if err != nil {
				return Evaluation{}, err
			}
		} else if op == meshbool.OpCut {
			ev.ShiftCertificate = captureShiftCertificate(ctx, a, b, unmoved, mbDecide, mb, piecesB, groups[targetFaces:], shifts, tolMM, chordScale, shiftDisplacement)
		}
		if prepared != nil && (ev.ShiftCertificate == nil ||
			prepared.shift.Grow == 0 && !ev.ShiftCertificate.IsTranslation() ||
			prepared.shift.Grow > 0 && !ev.ShiftCertificate.IsPureGrowth()) {
			return Evaluation{}, fmt.Errorf("%w: prepared shift certificate unavailable", decaderr.ErrUnsupported)
		}
		if op == meshbool.OpCut && ev.ShiftCertificate != nil && len(ma.Vertices) <= 65536 && len(ma.Triangles) <= 65536 {
			if replay != nil {
				ev.ShiftCertificate.targetMesh, err = snapshotShiftMesh(ctx, ma, replay)
				if err != nil {
					return Evaluation{}, err
				}
			} else {
				var captureErr error
				ev.ShiftCertificate.targetMesh, captureErr = captureOrdinaryShiftMesh(ctx, ma, proofbound.NewReplayTrialBudget())
				if captureErr != nil {
					ev.ShiftCertificate = nil
				}
			}
			if ev.ShiftCertificate != nil {
				floor, _ := booloperand.HeldFloor(a)
				ev.ShiftCertificate.targetChord = max(chordA, floor)
			}
		}
	}
	ev.Chain = summarizeChainAllowance(bmA, 0, bmB, 1)
	if replay != nil {
		if err := replay.ReserveRecords(ctx, 2); err != nil {
			return Evaluation{}, err
		}
		if err := replay.ReserveArray(ctx, 2, uint64(unsafe.Sizeof(ChainAllowanceSummary{}))); err != nil {
			return Evaluation{}, err
		}
		for i, m := range [2]*meshbool.BoolMesh{bmA, bmB} {
			if m.ChainAllowanceUsed {
				if ev.Chains == nil {
					ev.Chains = make([]ChainAllowanceSummary, 0, 2)
				}
				ev.Chains = append(ev.Chains, ChainAllowanceSummary{Operand: i, TouchedBound: m.ChainTouchedBound, PairChordLimit: m.PairChordLimit, ChainAllowance: m.ChainAllowance})
			}
		}
	}
	return ev, nil
}

// sourceGroup is the result group a mesh Boolean copies from face f of
// operand owner, at position ordinal of topobuild.Faces(owner): the face's
// retained tag (facetedrecord.SourceTag, flipped for the tool of a Cut) charged
// with the placed operand's departure, its roles and its identity.
func sourceGroup(owner *topo.Body, operand, ordinal int, f *topo.Face, flip bool,
	placed placedDepartures) facetedrecord.Group {
	group, _ := facetedrecord.SourceTag(f, flip)
	group = placed.charge(group, f, ordinal)
	// A mesh result face descends from f, so it carries every role f is
	// selected by: its own as Origins, which recognizers match faces by,
	// and those an analytic Boolean attributed to it as Upstream.
	group.Origins = slices.Clone(f.Origins)
	group.Upstream = slices.Clone(f.Upstream)
	// Identity only: no reader derives a plane or a bound from Source.
	group.Source = facetedrecord.FaceSource(owner.Origin.Producer, operand, ordinal, f)
	group.Planar = f.Surface.Kind() == surfacegeom.KindPlane || f.HeldPlanar
	return group
}

func configureChainGate(bm *meshbool.BoolMesh, pairLimit, allowance float64) {
	bm.PairChordLimit = pairLimit
	bm.ChainAllowance = allowance
	bm.Gate = max(pairLimit, allowance)
}

func summarizeChainAllowance(a *meshbool.BoolMesh, operandA int, b *meshbool.BoolMesh, operandB int) *ChainAllowanceSummary {
	var chosen *ChainAllowanceSummary
	for _, candidate := range []struct {
		mesh    *meshbool.BoolMesh
		operand int
	}{{a, operandA}, {b, operandB}} {
		m := candidate.mesh
		if !m.ChainAllowanceUsed {
			continue
		}
		summary := &ChainAllowanceSummary{Operand: candidate.operand,
			TouchedBound: m.ChainTouchedBound, PairChordLimit: m.PairChordLimit, ChainAllowance: m.ChainAllowance}
		if chosen == nil || summary.TouchedBound > chosen.TouchedBound {
			chosen = summary
		}
	}
	return chosen
}

// PrepBoolMesh lifts a tessellation into the exact domain and attaches its
// per-vertex and per-facet bounds. A non-finite vertex has no exact form, so
// it is rejected outright.
//
// A COLLAPSED operand facet — one whose exact normal is zero, which a rigid
// placement's own rounding can produce on an already-faceted body — is refused
// (decaderr.ErrUnsupported). A collapsed facet has no plane and no interior, so every
// contact predicate in this file is blind to it: a point or tangent contact the
// other operand makes THERE would be classified by nothing at all, and the
// facet would ride silently through the boolean with its component's verdict.
// The contact is exactly what this pipeline must not miss, so the operand is
// refused rather than partly examined. Loud beats silently wrong.
func PrepBoolMesh(ctx context.Context, m *topo.Mesh, src []int) (*meshbool.BoolMesh, error) {
	if retainedPlaneSlabDisabled(ctx) {
		return prepBoolMesh(ctx, m, src, func(int) (r3.Frame, float64, bool) { return r3.Frame{}, 0, false })
	}
	return prepBoolMesh(ctx, m, src, retainedPlaneOf(m))
}

// prepBoolMesh is PrepBoolMesh with the retained-plane reading as a parameter:
// a moved operand's facets lie on no retained plane.
func prepBoolMesh(ctx context.Context, m *topo.Mesh, src []int,
	retained func(facet int) (r3.Frame, float64, bool)) (*meshbool.BoolMesh, error) {
	bm, err := meshbool.PrepContext(ctx, m.Vertices, m.Triangles, src)
	if err != nil {
		return nil, err
	}
	// Per-vertex bounds and their facet maxima (agent-docs/faceted-vertex-bounds-
	// design.md §2, §4.5): the mesh's own record, or §2.1's reading derived
	// from its face bounds.
	beta, err := topobuild.VertexBounds(m)
	if err != nil {
		return nil, err
	}
	bm.SetVertexBounds(beta)
	bm.SetChargedSlabBounds(retained)
	return bm, nil
}

// booleanProofBounds composes the independent, non-negative volume and area
// proof terms that survive the mesh boolean into its faceted result.
func booleanProofBounds(symA, symB, roundVol, slackA, slackB, dropArea float64) (
	volSymDiff, areaSlack float64,
) {
	return proofbound.AbsSumUpper(symA, symB, roundVol),
		proofbound.AbsSumUpper(slackA, slackB, dropArea)
}

// operandChord is the chord request for operand b of a pair whose tolerance,
// already scaled by chordScale, is pairTol (meshbool.OperandChord).
func operandChord(ctx context.Context, b *topo.Body, pairTol, dPair, chordScale float64) float64 {
	_, restating := booloperand.HeldFloor(b)
	return meshbool.OperandChord(pairTol, dPair*chordScale, booloperand.ChordShare(ctx, b).Floor*chordScale, restating)
}
