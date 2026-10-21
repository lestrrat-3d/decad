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
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/facetproof"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/revolveshell"
	"github.com/lestrrat-3d/decad/internal/spiralsector"
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
	Payload     facetedrecord.Payload
	Volume      float64
	VolumeBound float64
	VolumeRat   *big.Rat
	Audit       *facetproof.MeshAudit
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
func Evaluate(ctx context.Context, op meshbool.OperationKind, a, b *topo.Body,
	meshes OperandMesher, chordScale float64, capFace, coneFace *topo.Face) (Evaluation, error) {
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
	ma, restatingA, err := meshes.TessellateOperand(ctx, a, tolMM)
	if err != nil {
		var coarse *tessellation.ExpectedError
		if errors.As(err, &coarse) {
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
	mb, restatingB, err := meshes.TessellateOperand(ctx, b, tolMM)
	if err != nil {
		var coarse *tessellation.ExpectedError
		if errors.As(err, &coarse) {
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
	var groups []facetedrecord.Group
	sourceID := map[*topo.Face]int{}
	targetFaces := len(topobuild.Faces(a))
	for i, f := range append(topobuild.Faces(a), topobuild.Faces(b)...) {
		if i%256 == 0 {
			if err := ctx.Err(); err != nil {
				return Evaluation{}, err
			}
		}
		sourceID[f] = len(groups)
		// The tool of a Cut (every face of b) leaves its natural normal
		// pointing into the result (agent-docs/faceted-source-retention-design.md §3.1).
		group, _ := facetedrecord.SourceTag(f, op == meshbool.OpCut && i >= targetFaces)
		group.Origins = append([]provenance.Ref(nil), f.Origins...)
		group.Planar = f.Surface.Kind() == surfacegeom.KindPlane || f.HeldPlanar
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
	spiralHeld, spiralJoin, err := spiralsector.PrepareFirstUnion(ctx, op, revolveshell.RootCandidates(a),
		a, b, mb, srcB)
	if err != nil {
		return Evaluation{}, err
	}
	if spiralHeld != nil {
		mb = spiralHeld
	}
	mb, spiralCombine, err := spiralsector.PrepareCombinedUnion(ctx, op, meshes, a, b, mb, tolMM)
	if err != nil {
		return Evaluation{}, err
	}
	if spiralJoin != nil {
		allowed, ok := spiralsector.FirstUnionPairs(a, b, spiralJoin, sourceID)
		if !ok {
			return Evaluation{}, meshbool.ExpectedBoolean(meshbool.BooleanExpectedUnsupported,
				fmt.Errorf("%w: first spiral join source faces", decaderr.ErrUnsupported))
		}
		spiralJoin.Allowed = allowed
	}
	var combinedAdmission *spiralsector.CombinedAdmission
	if spiralCombine != nil {
		combinedAdmission, err = spiralsector.CombineFaceRoles(ctx, a, b, spiralCombine, sourceID)
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
	bmB, err := PrepBoolMesh(ctx, mb, srcB)
	if err != nil {
		if errors.Is(err, decaderr.ErrUnsupported) {
			err = meshbool.ExpectedBoolean(meshbool.BooleanExpectedUnsupported, err)
		}
		return Evaluation{}, err
	}
	// A restating operand's facets where the pair meets must hold within the
	// pair tolerance; the gate reads them on the contact classification below
	// (agent-docs/faceted-vertex-bounds-design.md §5). Any other operand is ungated.
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
		bmB.Gate = max(bmB.Gate, mb.Bound)
	}
	if spiralCombine != nil {
		bmB.Gate = max(bmB.Gate, mb.Bound)
	}
	if err := ctx.Err(); err != nil {
		return Evaluation{}, err
	}
	// One memo per call, over exactly the two prepared operand tessellations
	// the gate and the mesh pass both walk; it is dropped when the call
	// returns (meshbool.ContactMemo).
	memo := meshbool.NewContactMemo(bmA, bmB)
	var spiralPairs map[[2]*topo.Face]meshbool.NearPairRole
	if spiralJoin != nil {
		spiralPairs = spiralJoin.Roles
	}
	if combinedAdmission != nil {
		spiralPairs = combinedAdmission.Roles
	}
	if err := meshbool.RefuseUndecidableProximity(ctx, ma, mb, bmA, bmB, memo,
		capFace, coneFace, spiralPairs); err != nil {
		err = meshbool.StagingError(op, err)
		if errors.Is(err, decaderr.ErrUnsupported) {
			err = meshbool.ExpectedBoolean(meshbool.BooleanExpectedContact, err)
		}
		return Evaluation{}, err
	}
	if spiralJoin != nil {
		ok, err := spiralsector.ScopeWidenedContacts(ctx, bmA, bmB, memo,
			spiralJoin.Allowed, tolMM)
		if err != nil {
			return Evaluation{}, err
		}
		if !ok {
			return Evaluation{}, meshbool.ExpectedBoolean(meshbool.BooleanExpectedUnsupported,
				fmt.Errorf("%w: first spiral join contact faces", decaderr.ErrUnsupported))
		}
	}
	if combinedAdmission != nil {
		ok, err := spiralsector.ScopeWidenedContacts(ctx, bmA, bmB, memo,
			combinedAdmission.Allowed, tolMM, 24*len(combinedAdmission.Source.Tools))
		if err != nil {
			return Evaluation{}, err
		}
		if !ok {
			return Evaluation{}, meshbool.ExpectedBoolean(meshbool.BooleanExpectedUnsupported,
				fmt.Errorf("%w: combined spiral join contact faces", decaderr.ErrUnsupported))
		}
	}
	if err := ctx.Err(); err != nil {
		return Evaluation{}, err
	}
	kept, maxRim, err := meshbool.MeshBoolean(ctx, op, bmA, bmB, memo)
	if err != nil {
		return Evaluation{}, meshbool.StagingError(op, err)
	}
	if len(kept) == 0 {
		return Evaluation{}, meshbool.ExpectedBoolean(meshbool.BooleanExpectedEmpty,
			fmt.Errorf(`%w: the %s result is empty`, decaderr.ErrBooleanFailed, op))
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
	// weld dropped any collapsed facet from it (internal/proofbound/bounds.go, proofbound.SweptVolumeAllow).
	// The held mesh's area is the WRONG yardstick here: a dropped facet is
	// missing from it, and its swept volume would go uncharged. The area the
	// weld dropped is likewise missing from every area the result reports, so
	// it joins the operands' own chord deficit in areaSlack.
	roundVol := proofbound.SweptVolumeAllow(stitched.Round, stitched.PreArea)
	volSymDiff, areaSlack := booleanProofBounds(
		symA, symB, roundVol,
		ma.AreaSlack, mb.AreaSlack, stitched.DropArea,
	)
	if spiralJoin != nil || combinedAdmission != nil {
		allowance, ok, err := spiralsector.JoinAreaAllowance(ctx, a, b, stitched)
		if err != nil {
			return Evaluation{}, err
		}
		if !ok {
			return Evaluation{}, meshbool.ExpectedBoolean(meshbool.BooleanExpectedUnsupported,
				fmt.Errorf("%w: spiral join area bound", decaderr.ErrUnsupported))
		}
		areaSlack = proofbound.AbsSumUpper(areaSlack, allowance)
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
	audit, err := facetproof.AuditFacetedMesh(ctx, payload.Verts, payload.Tris)
	if err != nil {
		return Evaluation{}, err
	}
	if spiralJoin != nil {
		ok, err := spiralsector.ScopeOutputSwitches(ctx, stitched,
			len(topobuild.Faces(a)), spiralJoin.Allowed)
		if err != nil {
			return Evaluation{}, err
		}
		if !ok {
			return Evaluation{}, meshbool.ExpectedBoolean(meshbool.BooleanExpectedUnsupported,
				fmt.Errorf("%w: first spiral join output faces", decaderr.ErrUnsupported))
		}
		ok, err = spiralsector.ValidateFirstUnion(ctx, a, b, bmA, stitched, spiralJoin)
		if err != nil {
			return Evaluation{}, err
		}
		if !ok {
			return Evaluation{}, meshbool.ExpectedBoolean(meshbool.BooleanExpectedUnsupported,
				fmt.Errorf("%w: first spiral join boundary proof", decaderr.ErrUnsupported))
		}
	}
	if combinedAdmission != nil {
		ok, err := spiralsector.ScopeOutputSwitches(ctx, stitched,
			len(topobuild.Faces(a)), combinedAdmission.Allowed, 24*len(combinedAdmission.Source.Tools))
		if err != nil {
			return Evaluation{}, err
		}
		if !ok {
			return Evaluation{}, meshbool.ExpectedBoolean(meshbool.BooleanExpectedUnsupported,
				fmt.Errorf("%w: combined spiral join output faces", decaderr.ErrUnsupported))
		}
		ok, err = spiralsector.ValidateCombinedUnion(ctx, a, bmA, stitched, combinedAdmission)
		if err != nil {
			return Evaluation{}, err
		}
		if !ok {
			return Evaluation{}, meshbool.ExpectedBoolean(meshbool.BooleanExpectedUnsupported,
				fmt.Errorf("%w: combined spiral join boundary proof", decaderr.ErrUnsupported))
		}
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
	return Evaluation{Payload: payload, Volume: volume, VolumeBound: volumeBound, VolumeRat: volumeRat, Audit: audit}, nil
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
	bm.SetSlabBounds(retainedPlaneOf(m))
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
