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
	"math/big"

	"github.com/lestrrat-3d/r3"

	"github.com/lestrrat-3d/decad/internal/booloperand"
	"github.com/lestrrat-3d/decad/internal/chaintol"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/facetproof"
	"github.com/lestrrat-3d/decad/internal/gaptol"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/opdiagnostic"
	"github.com/lestrrat-3d/decad/internal/proofbound"
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
	Payload        facetedrecord.Payload
	Volume         float64
	VolumeBound    float64
	VolumeRat      *big.Rat
	Audit          *facetproof.MeshAudit
	Shift          *ShiftSummary
	Chain          *ChainAllowanceSummary
	EmbeddingReach *meshbool.EmbeddingReach
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
	if e.Chain != nil {
		allowance := e.Chain.ChainAllowance
		records = append(records, opdiagnostic.Record{
			Operation: int(op), Code: opdiagnostic.CodeChainAllowanceApplied,
			Operand: e.Chain.Operand, TouchedBound: e.Chain.TouchedBound,
			PairChordLimit: e.Chain.PairChordLimit, ChainAllowance: &allowance,
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
	ev, err := evaluate(ctx, op, a, b, meshes, chordScale, capFace, coneFace, nil)
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
		return evaluate(ctx, op, a, b, meshes, chordScale, capFace, coneFace, shifts)
	})
}

// evaluate is Evaluate's single attempt, with the pieces of b's mesh moved by
// shifts when there are any.
func evaluate(ctx context.Context, op meshbool.OperationKind, a, b *topo.Body,
	meshes OperandMesher, chordScale float64, capFace, coneFace *topo.Face, shifts map[int]pieceShift) (Evaluation, error) {
	if err := ctx.Err(); err != nil {
		return Evaluation{}, err
	}

	tolMM, dPair, err := booloperand.PairChord(ctx, a, b)
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
	ma, restatingA, err := meshes.TessellateOperand(ctx, a, operandChord(ctx, a, tolMM, dPair, chordScale))
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
	mb, restatingB, err := meshes.TessellateOperand(ctx, b, operandChord(ctx, b, tolMM, dPair, chordScale))
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
	facesA := topobuild.Faces(a)
	facesB := topobuild.Faces(b)
	targetFaces := len(facesA)
	totalFaces := targetFaces + len(facesB)
	groups := make([]facetedrecord.Group, 0, totalFaces)
	sourceID := make(map[*topo.Face]int, totalFaces)
	var (
		piecesB   pieces
		facePiece map[*topo.Face]int
		reachB    float64
	)
	if len(shifts) > 0 {
		piecesB = piecesOf(mb)
		facePiece = facePieces(mb, piecesB)
		if reachB, err = tagReach(mb); err != nil {
			return Evaluation{}, err
		}
	}
	for i, f := range append(facesA, facesB...) {
		if i%256 == 0 {
			if err := ctx.Err(); err != nil {
				return Evaluation{}, err
			}
		}
		sourceID[f] = len(groups)
		// The tool of a Cut (every face of b) leaves its natural normal
		// pointing into the result (agent-docs/faceted-source-retention-design.md §3.1).
		group, _ := facetedrecord.SourceTag(f, op == meshbool.OpCut && i >= targetFaces)
		// A mesh result face descends from f, so it carries every role f is
		// selected by: its own and those an analytic Boolean attributed to it.
		group.Origins = f.AllOrigins()
		// Identity only: no reader derives a plane or a bound from Source.
		operand, ordinal, owner := 0, i, a
		if i >= targetFaces {
			operand, ordinal, owner = 1, i-targetFaces, b
		}
		group.Source = facetedrecord.FaceSource(owner.Origin.Producer, operand, ordinal, f)
		group.Planar = f.Surface.Kind() == surfacegeom.KindPlane || f.HeldPlanar
		if len(shifts) > 0 && i >= targetFaces {
			// A moved piece's true faces lie on its tags translated by the
			// shift, never on the unmoved tags.
			piece, ok := facePiece[f]
			group = movedGroup(group, piece, ok, shifts, mb.Vertices, reachB)
		}
		groups = append(groups, group)
	}
	srcA, err := meshbool.SourceIDs(ctx, ma.Source, sourceID)
	if err != nil {
		return Evaluation{}, err
	}
	srcB, err := meshbool.SourceIDs(ctx, mb.Source, sourceID)
	if err != nil {
		return Evaluation{}, err
	}
	if err := ctx.Err(); err != nil {
		return Evaluation{}, err
	}
	// mbDecide is the mesh the contact gate reads: mb itself, unless b moved.
	mbDecide := mb
	shiftDisplacement := 0.0
	if len(shifts) > 0 {
		// The cap and cone certificates authenticate the operands' own
		// geometry, which a moved operand no longer has.
		if capFace != nil || coneFace != nil {
			return Evaluation{}, errShiftDeclined
		}
		mbDecide, mb, shiftDisplacement, err = shiftOperandMesh(ctx, mb, piecesB, shifts)
		if err != nil {
			return Evaluation{}, err
		}
	}
	bmA, err := PrepBoolMesh(ctx, ma, srcA)
	if err != nil {
		if errors.Is(err, decaderr.ErrUnsupported) {
			err = meshbool.ExpectedBoolean(meshbool.BooleanExpectedUnsupported, err)
		}
		return Evaluation{}, err
	}
	var bmB *meshbool.BoolMesh
	if len(shifts) > 0 {
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
		configureChainGate(bmA, tolMM, chain)
	}
	if restatingB {
		// A moved operand's facets carry the move in their bounds, which the
		// gate would read as a coarse mesh. The move is what the gap tolerance
		// permits and the composition of the result's bounds charges it, so the
		// gate allows it.
		configureChainGate(bmB, tolMM+(mb.Bound-mbDecide.Bound), chain)
	}
	if err := ctx.Err(); err != nil {
		return Evaluation{}, err
	}
	// One memo per call, over exactly the two prepared operand tessellations
	// the gate and the mesh pass both walk; it is dropped when the call
	// returns (meshbool.ContactMemo).
	memo := meshbool.NewContactMemo(bmA, bmB)
	if err := meshbool.RefuseUndecidableProximity(ctx, ma, mbDecide, bmA, bmB, memo,
		capFace, coneFace); err != nil {
		err = meshbool.StagingError(op, err)
		if errors.Is(err, decaderr.ErrUnsupported) {
			err = meshbool.ExpectedBoolean(meshbool.BooleanExpectedContact, err)
		}
		return Evaluation{}, err
	}
	if err := ctx.Err(); err != nil {
		return Evaluation{}, err
	}
	kept, maxRim, err := meshbool.MeshBoolean(ctx, op, bmA, bmB, memo)
	if err != nil {
		return Evaluation{}, meshbool.StagingError(op, err)
	}
	if len(kept) == 0 {
		err := fmt.Errorf(`%w: the %s result is empty`, decaderr.ErrBooleanFailed, op)
		if len(shifts) > 0 {
			return Evaluation{}, meshbool.ExpectedBooleanAfterShift(err, shiftDisplacement, gaptol.Reach(ctx))
		}
		return Evaluation{}, meshbool.ExpectedBoolean(meshbool.BooleanExpectedEmpty, err)
	}
	if err := ctx.Err(); err != nil {
		return Evaluation{}, err
	}
	stitched, err := meshbool.StitchFacetsContext(ctx, kept)
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
	audit, err := facetproof.AuditFacetedMesh(ctx, payload.Verts, payload.Tris)
	if err != nil {
		return Evaluation{}, err
	}
	volume, volumeBound, volumeRat, err := VolumeReading(ctx, audit, payload.VolSymDiff)
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
	if len(shifts) > 0 {
		ev.Shift = &ShiftSummary{DisplacementBound: shiftDisplacement, AllowedReach: gaptol.Reach(ctx)}
	}
	ev.Chain = summarizeChainAllowance(bmA, 0, bmB, 1)
	return ev, nil
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
