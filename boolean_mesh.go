package decad

import (
	"context"
	"errors"
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/booloperand"
	"github.com/lestrrat-3d/decad/internal/facetproof"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/spiralsector"
	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// This file is the mesh-boolean pipeline of docs/evaluator-design.md §9: the
// two operands' tessellations meet in exact triangle/triangle intersection
// segments, each cut facet is subdivided along them (internal/meshbool/boolean_cut.go), every
// resulting piece is classified against the other solid by an exact local
// side-of-contact test (or exact ray parity for the uncut regions), the kept
// pieces are stitched by shared exact vertices, and a conforming pass plus a
// closure audit prove the result watertight by construction — an audit
// failure is an error, never a wrong mesh.

// prepBoolMesh lifts a tessellation into the exact domain and attaches its
// per-vertex and per-facet bounds. A non-finite vertex has no exact form, so
// it is rejected outright.
//
// A COLLAPSED operand facet — one whose exact normal is zero, which a rigid
// placement's own rounding can produce on an already-faceted body — is refused
// (ErrUnsupported). A collapsed facet has no plane and no interior, so every
// contact predicate in this file is blind to it: a point or tangent contact the
// other operand makes THERE would be classified by nothing at all, and the
// facet would ride silently through the boolean with its component's verdict.
// The contact is exactly what this pipeline must not miss, so the operand is
// refused rather than partly examined. Loud beats silently wrong.
func prepBoolMeshContext(ctx context.Context, m *Mesh, src []int) (*meshbool.BoolMesh, error) {
	bm, err := meshbool.PrepContext(ctx, m.n.Vertices, m.n.Triangles, src)
	if err != nil {
		return nil, err
	}
	// Per-vertex bounds and their facet maxima (docs/faceted-vertex-bounds-
	// design.md §2, §4.5): the mesh's own record, or §2.1's reading derived
	// from its face bounds.
	beta, err := m.vertexBounds()
	if err != nil {
		return nil, err
	}
	bm.SetVertexBounds(beta)
	return bm, nil
}

func evaluateBooleanMeshesWithConeCap(ctx context.Context, op meshbool.OperationKind, a, b *Body,
	meshes operandMeshes, chordScale float64, capFace, coneFace *topo.Face) (booleanEvaluation, error) {
	if err := ctx.Err(); err != nil {
		return booleanEvaluation{}, err
	}

	tolMM, dPair, err := booloperand.PairChord(ctx, &a.n, &b.n)
	if err != nil {
		return booleanEvaluation{}, err
	}
	tolMM *= chordScale
	if err := ctx.Err(); err != nil {
		return booleanEvaluation{}, err
	}
	// A payload whose mesh cannot carry an occupied-volume proof is refused
	// BEFORE it is meshed. meshbool.OperandSymDiff is still the gate that decides
	// it — this only asks the same question of the payload class, where the
	// answer is already known, so a refusal costs a map lookup instead of a
	// complete tessellation and its facet-pair audit.
	if err := booloperand.RequireVolumeProof(ctx, &a.n, 0); err != nil {
		return booleanEvaluation{}, err
	}
	if err := booloperand.RequireVolumeProof(ctx, &b.n, 1); err != nil {
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
	sourceID := map[*topo.Face]int{}
	for i, f := range append(a.Faces(), b.Faces()...) {
		if i%256 == 0 {
			if err := ctx.Err(); err != nil {
				return booleanEvaluation{}, err
			}
		}
		sourceID[&f.n] = len(groups)
		groups = append(groups, facetGroup{
			Origins: f.originRefs(),
			Planar:  f.isPlanar(),
		})
	}
	srcA, err := meshbool.SourceIDs(ctx, ma.n.Source, sourceID)
	if err != nil {
		return booleanEvaluation{}, err
	}
	srcB, err := meshbool.SourceIDs(ctx, mb.n.Source, sourceID)
	if err != nil {
		return booleanEvaluation{}, err
	}
	if err := ctx.Err(); err != nil {
		return booleanEvaluation{}, err
	}
	spiralHeld, spiralJoin, err := spiralsector.PrepareFirstUnion(ctx, op, revolveRootCandidates(a),
		bodyNode(a), bodyNode(b), &mb.n, srcB)
	if err != nil {
		return booleanEvaluation{}, err
	}
	if spiralHeld != nil {
		mb = bornMesh(&Mesh{n: *spiralHeld})
	}
	mb, spiralCombine, err := spiralPrepareCombinedUnion(ctx, op, a, b, mb, tolMM)
	if err != nil {
		return booleanEvaluation{}, err
	}
	if spiralJoin != nil {
		allowed, ok := spiralsector.FirstUnionPairs(bodyNode(a), bodyNode(b), spiralJoin, sourceID)
		if !ok {
			return booleanEvaluation{}, meshbool.ExpectedBoolean(meshbool.BooleanExpectedUnsupported,
				fmt.Errorf("%w: first spiral join source faces", ErrUnsupported))
		}
		spiralJoin.Allowed = allowed
	}
	var combinedAdmission *spiralsector.CombinedAdmission
	if spiralCombine != nil {
		combinedAdmission, err = spiralsector.CombineFaceRoles(ctx, bodyNode(a), bodyNode(b), spiralCombine, sourceID)
		if err != nil {
			return booleanEvaluation{}, err
		}
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
	if spiralJoin != nil {
		// The temporary curved cap carries its actual moved-facet departure.
		// Complete source-pair and output-curve gates below scope this one
		// allowance to the authenticated first tooth join.
		bmB.Gate = max(bmB.Gate, mb.n.Bound)
	}
	if spiralCombine != nil {
		bmB.Gate = max(bmB.Gate, mb.n.Bound)
	}
	if err := ctx.Err(); err != nil {
		return booleanEvaluation{}, err
	}
	// One memo per call, over exactly the two prepared operand tessellations
	// the gate and the mesh pass both walk; it is dropped when the call
	// returns (boolean_mesh.go, meshbool.ContactMemo).
	memo := meshbool.NewContactMemo(bmA, bmB)
	var spiralPairs map[[2]*topo.Face]meshbool.NearPairRole
	if spiralJoin != nil {
		spiralPairs = spiralJoin.Roles
	}
	if combinedAdmission != nil {
		spiralPairs = combinedAdmission.Roles
	}
	if err := meshbool.RefuseUndecidableProximity(ctx, &ma.n, &mb.n, bmA, bmB, memo,
		capFace, coneFace, spiralPairs); err != nil {
		err = meshbool.StagingError(op, err)
		if errors.Is(err, ErrUnsupported) {
			err = meshbool.ExpectedBoolean(meshbool.BooleanExpectedContact, err)
		}
		return booleanEvaluation{}, err
	}
	if spiralJoin != nil {
		ok, err := spiralsector.ScopeWidenedContacts(ctx, bmA, bmB, memo,
			spiralJoin.Allowed, tolMM)
		if err != nil {
			return booleanEvaluation{}, err
		}
		if !ok {
			return booleanEvaluation{}, meshbool.ExpectedBoolean(meshbool.BooleanExpectedUnsupported,
				fmt.Errorf("%w: first spiral join contact faces", ErrUnsupported))
		}
	}
	if combinedAdmission != nil {
		ok, err := spiralsector.ScopeWidenedContacts(ctx, bmA, bmB, memo,
			combinedAdmission.Allowed, tolMM, 24*len(combinedAdmission.Source.Tools))
		if err != nil {
			return booleanEvaluation{}, err
		}
		if !ok {
			return booleanEvaluation{}, meshbool.ExpectedBoolean(meshbool.BooleanExpectedUnsupported,
				fmt.Errorf("%w: combined spiral join contact faces", ErrUnsupported))
		}
	}
	if err := ctx.Err(); err != nil {
		return booleanEvaluation{}, err
	}
	kept, maxRim, err := meshbool.MeshBoolean(ctx, op, bmA, bmB, memo)
	if err != nil {
		return booleanEvaluation{}, meshbool.StagingError(op, err)
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
	if err := meshbool.RefuseRimPastPair(maxRim, dPair); err != nil {
		return booleanEvaluation{}, meshbool.ExpectedBoolean(meshbool.BooleanExpectedUnsupported, err)
	}
	symA, err := meshbool.OperandSymDiff(ma.n.BoundaryOK, ma.n.SymDiffOK, ma.n.VolSymDiff)
	if err != nil {
		return booleanEvaluation{}, meshbool.ExpectedBooleanForOperand(meshbool.BooleanExpectedVolumeProof, 0, err)
	}
	symB, err := meshbool.OperandSymDiff(mb.n.BoundaryOK, mb.n.SymDiffOK, mb.n.VolSymDiff)
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
		ma.n.AreaSlack, mb.n.AreaSlack, stitched.DropArea,
	)
	if spiralJoin != nil || combinedAdmission != nil {
		allowance, ok, err := spiralsector.JoinAreaAllowance(ctx, bodyNode(a), bodyNode(b), stitched)
		if err != nil {
			return booleanEvaluation{}, err
		}
		if !ok {
			return booleanEvaluation{}, meshbool.ExpectedBoolean(meshbool.BooleanExpectedUnsupported,
				fmt.Errorf("%w: spiral join area bound", ErrUnsupported))
		}
		areaSlack = proofbound.AbsSumUpper(areaSlack, allowance)
	}
	payload := facetedPayload{
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
		return booleanEvaluation{}, err
	}
	audit, err := facetproof.AuditFacetedMesh(ctx, payload.Verts, payload.Tris)
	if err != nil {
		return booleanEvaluation{}, err
	}
	if spiralJoin != nil {
		ok, err := spiralsector.ScopeOutputSwitches(ctx, stitched,
			len(a.Faces()), spiralJoin.Allowed)
		if err != nil {
			return booleanEvaluation{}, err
		}
		if !ok {
			return booleanEvaluation{}, meshbool.ExpectedBoolean(meshbool.BooleanExpectedUnsupported,
				fmt.Errorf("%w: first spiral join output faces", ErrUnsupported))
		}
		ok, err = spiralsector.ValidateFirstUnion(ctx, bodyNode(a), bodyNode(b), bmA, stitched, spiralJoin)
		if err != nil {
			return booleanEvaluation{}, err
		}
		if !ok {
			return booleanEvaluation{}, meshbool.ExpectedBoolean(meshbool.BooleanExpectedUnsupported,
				fmt.Errorf("%w: first spiral join boundary proof", ErrUnsupported))
		}
	}
	if combinedAdmission != nil {
		ok, err := spiralsector.ScopeOutputSwitches(ctx, stitched,
			len(a.Faces()), combinedAdmission.Allowed, 24*len(combinedAdmission.Source.Tools))
		if err != nil {
			return booleanEvaluation{}, err
		}
		if !ok {
			return booleanEvaluation{}, meshbool.ExpectedBoolean(meshbool.BooleanExpectedUnsupported,
				fmt.Errorf("%w: combined spiral join output faces", ErrUnsupported))
		}
		ok, err = spiralsector.ValidateCombinedUnion(ctx, bodyNode(a), bmA, stitched, combinedAdmission)
		if err != nil {
			return booleanEvaluation{}, err
		}
		if !ok {
			return booleanEvaluation{}, meshbool.ExpectedBoolean(meshbool.BooleanExpectedUnsupported,
				fmt.Errorf("%w: combined spiral join boundary proof", ErrUnsupported))
		}
	}
	volume, volumeRat, err := meshVolumeMeasurement(ctx, audit, payload.VolSymDiff)
	if err != nil {
		return booleanEvaluation{}, err
	}
	// The mesh audit and occupied-volume reading certify this Boolean result
	// as a source for a later positive similarity. No unbuilt result is marked.
	payload.BooleanSimilaritySource = true
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

// meshVolumeMeasurement reads the audited mesh's exact component volumes and
// composes the symmetric-difference allowance with final float rounding.
func meshVolumeMeasurement(ctx context.Context, audit *facetproof.MeshAudit, volSymDiff float64) (Measurement, *big.Rat, error) {
	if err := ctx.Err(); err != nil {
		return Measurement{}, nil, err
	}
	volF, bound, volRat, err := facetproof.VolumeFromAudit(audit, volSymDiff)
	if ctxErr := ctx.Err(); ctxErr != nil {
		return Measurement{}, nil, ctxErr
	}
	if err != nil {
		return Measurement{}, nil, err
	}
	return Measurement{
		Value:     units.CubicMillimeters(volF),
		Exactness: exactnessOf(bound),
		Bound:     units.CubicMillimeters(bound),
	}, volRat, nil
}

// exactnessOf keys a measurement's exactness off its proven bound: zero is the
// truth, anything else is Approximate.
//
// A zero bound is therefore a CLAIM that the reported value is exactly
// representable — and only a value proven so may reach it: analytic features
// with explicit rational/error evaluation, and boolean rational integrals
// carrying their own rounding term. A value a float loop COMPUTED is never among
// them: its bound comes from internal/proofbound/bounds.go, whose helpers are never zero for a
// nonzero float-computed quantity (proofbound.SumSlop, proofbound.ChainLengthBound), so it can never
// arrive here claiming Exact.
func exactnessOf(bound float64) Exactness {
	if bound == 0 {
		return Exact
	}
	return Approximate
}
