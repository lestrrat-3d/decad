package booloperand

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/brepbandmesh"
	"github.com/lestrrat-3d/decad/internal/breprecord"
	"github.com/lestrrat-3d/decad/internal/capblend"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/draftrecord"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/stitchrecord"
	"github.com/lestrrat-3d/decad/internal/stitchtess"
	"github.com/lestrrat-3d/decad/internal/stitchweld"
	"github.com/lestrrat-3d/decad/internal/topo"
)

// RequireVolumeProof refuses an operand whose payload class publishes no
// occupied-volume proof on its mesh, before that mesh is built. index is the
// operand's position in the pair (0 or 1). A refusal is a
// meshbool.BooleanExpectedVolumeProof error for that operand wrapping
// ErrUnsupported: a staging refusal reached before any contact is examined,
// which reaches the same public error as an untessellatable operand and a
// distinct Verify diagnostic message.
//
// It is meshbool.OperandSymDiff's question asked one step earlier. That
// function reads the volume of the symmetric difference between the operand's
// held tessellation and the body it stands for from the MESH's own proof
// record (agent-docs/tessellation-design.md §2's volSymDiff), and it has no
// fallback: `Mesh.Bound × held area` is NOT an occupied-volume proof — a
// two-sided Hausdorff bound alone does not bound it, because a doubly-curved
// cell can gain material where another loses it and the signed error cancels
// while the symmetric difference does not — and agent-docs/tessellation-design.md
// §11 forbids the substitution outright. A payload whose occupied-volume proof
// has not landed publishes symDiffOK false, and its mesh serves export only.
//
// A cap-loop chamfer, and a draft body through the same band, narrows rather
// than refuses outright: both arms read the SAME predicate,
// capblend.OccupiedVolumeAdmission — this one before the mesh is built, and
// the cap-blend tessellation through symDiffOK after it — so a payload whose
// every band agent-docs/tessellation-reach-design.md §7 admits (whole turns,
// line-line miters, exactly G1 joins) passes here and publishes its proof
// there, and every other one refuses here with the loop and corner it fails
// on. Meshing a refused operand for a boolean only to refuse it afterwards
// buys nothing and costs the whole tessellation and its facet-pair audit —
// which the evaluator's own internal tolerance makes the most expensive part
// of the call. This and meshbool.OperandSymDiff must name the same payload
// classes, and each proof retires both arms of its own row together. The
// matching axial loft's held-polygon cap band has a separate slab proof for
// its reflex corners; the cap-blend tessellation publishes that proof for this
// marked payload, so it passes this precheck too (loft §18).
//
// A sheet operand refuses HERE too, ahead of the payload-class switch:
// agent-docs/surface-design.md §10 states that a sheet mesh carries no
// occupied-volume proof, on the same terms as a refused cap-loop chamfer's.
// In the ordinary public boolean path the root's sheet-operand refusal fires
// first (Table X); this arm keeps the invariant that it and
// meshbool.OperandSymDiff name the same set of payload classes for a caller
// that reaches this function by some other path.
//
// A stitched operand narrows rather than refuses outright: a CLOSED,
// all-planar body (Tris != nil; an open one is a sheet and refused above)
// whose every vertex carries a proven bound of exactly zero
// (stitchweld.ZeroBound over stitchtess.VertexBounds) passes here, because
// that is the vertex gate stitchtess.Restate applies before it publishes the
// occupied-volume proof. Restate also requires its lumps proven separate
// (stitchrecord.FaceLumpsProvenSeparate), so a body passing here may still be
// refused by meshbool.OperandSymDiff after it is meshed. Every other stitched
// body — curved/mixed, placed, or certificate-welded — refuses here, before
// it is ever meshed.
func RequireVolumeProof(ctx context.Context, b *topo.Body, index int) error {
	if b.Kind == topo.BodySheet {
		return meshbool.ExpectedBooleanForOperand(meshbool.BooleanExpectedVolumeProof, index,
			fmt.Errorf(`%w: a sheet's mesh carries no proof of the volume it and the body it stands for differ by, so no boolean may compose it`, decaderr.ErrUnsupported))
	}
	var err error
	switch pl := b.Payload.(type) {
	case capblend.Payload:
		if pl.LoftSource != nil {
			// The loft-only cap band proves its occupied volume with
			// a bounded reflex-corner slab in its tessellation.
			return nil
		}
		refusal, aErr := capblend.OccupiedVolumeAdmission(proofbound.NewWorkBudget(ctx), pl)
		if aErr != nil {
			return aErr
		}
		if refusal == nil {
			return nil
		}
		err = refusal
	case breprecord.Payload:
		// A route L body's mesh proves its occupied volume only for bands
		// the cap-loop chamfer's rule admits (modify-general Table DG's
		// DG4); a plain brep's proof is unconditional.
		if len(pl.LoopBands) == 0 {
			return nil
		}
		refusal, aErr := brepbandmesh.OccupiedVolumeAdmission(proofbound.NewWorkBudget(ctx), pl)
		if aErr != nil {
			return aErr
		}
		if refusal == nil {
			return nil
		}
		err = refusal
	case draftrecord.Payload:
		refusal, aErr := capblend.OccupiedVolumeAdmission(proofbound.NewWorkBudget(ctx), pl.Band())
		if aErr != nil {
			return aErr
		}
		if refusal == nil {
			return nil
		}
		err = refusal
	case draftrecord.TwoSidedPayload:
		for _, half := range []draftrecord.Payload{pl.Negative, pl.Positive} {
			refusal, aErr := capblend.OccupiedVolumeAdmission(proofbound.NewWorkBudget(ctx), half.Band())
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
	case stitchrecord.Payload:
		if pl.Tris != nil {
			zeroBound, zErr := stitchweld.ZeroBound(proofbound.NewWorkBudget(ctx), stitchtess.VertexBounds(b))
			if zErr != nil {
				return zErr
			}
			if zeroBound {
				return nil
			}
		}
		err = fmt.Errorf(`%w: a stitched body's mesh carries no proof of the volume it and the body it stands for differ by, so no boolean may compose it`, decaderr.ErrUnsupported)
	default:
		return nil
	}
	return meshbool.ExpectedBooleanForOperand(meshbool.BooleanExpectedVolumeProof, index, err)
}
