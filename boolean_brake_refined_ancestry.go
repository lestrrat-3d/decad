package decad

import (
	"context"
	"slices"
	"unsafe"

	"github.com/lestrrat-3d/decad/internal/booleaneval"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/opdiagnostic"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/topo"
)

// brakeRefinedPlacementPayload checks the live proved operands before removing
// the private U-prime parent from the groups that placement will rebuild.
func brakeRefinedPlacementPayload(ctx context.Context, refined *brakeRefinedTool, result *topo.Body, budget *proofbound.ReplayBudget) (facetedrecord.Payload, error) {
	var zero facetedrecord.Payload
	if refined != nil {
		refined.growthVerified = false
	}
	if refined == nil || result == nil || budget == nil || refined.budget != budget || refined.cutBody != result || refined.blank == nil || refined.original == nil || refined.slot == nil || refined.pin == nil || refined.body == nil || refined.cert == nil {
		return zero, errBrakeMissingRefinedClevisEvidence
	}
	if refined.blank.Origin.Producer != refined.blankID || refined.original.Origin.Producer != refined.originalID || refined.slot.Origin.Producer != refined.slotID || refined.pin.Origin.Producer != refined.pinID || len(refined.original.OperationWarnings) != 0 {
		return zero, errBrakeRefinedClevisSourceChanged
	}
	shift, ok := result.PrivateShift.(*booleaneval.ShiftCertificate)
	if !ok {
		return zero, errBrakeRefinedClevisShiftEvidenceMissing
	}
	var warning opdiagnostic.Record
	count := 0
	for _, w := range result.OperationWarnings {
		if err := budget.SpendWork(ctx, 1); err != nil {
			return zero, err
		}
		if w.Code == opdiagnostic.CodeGapShiftApplied {
			count++
			warning = w
		}
	}
	if count != 1 {
		return zero, errBrakeRefinedClevisShiftWarningMismatch
	}
	growth, err := booleaneval.AuthenticateStoredGrowth(ctx, shift, result, refined.blank, refined.body, warning, budget)
	if err != nil {
		return zero, err
	}
	if err := booleaneval.RevalidateBrakeCandidateForPlacement(ctx, refined.cert, refined.blank, refined.body, budget); err != nil {
		return zero, err
	}
	resultPayload, ok := result.Payload.(facetedrecord.Payload)
	if !ok {
		return zero, errBrakeRefinedClevisPayloadMissing
	}
	unchanged, err := refined.cutSnapshot.Unchanged(ctx, resultPayload, budget)
	if err != nil {
		return zero, err
	}
	if !unchanged {
		return zero, errBrakeRefinedClevisPayloadChanged
	}
	candidatePayload, ok := refined.body.Payload.(facetedrecord.Payload)
	if !ok {
		return zero, errBrakeRefinedToolPayloadMissing
	}
	originalPayload, ok := refined.original.Payload.(facetedrecord.Payload)
	if !ok {
		return zero, errBrakeOriginalToolPayloadMissing
	}
	if len(resultPayload.Groups) > brakeSourceNodeLimit || len(candidatePayload.Groups) > brakeSourceNodeLimit || len(originalPayload.Groups) > brakeSourceNodeLimit {
		return zero, errBrakeRefinedAncestryGroupLimit
	}
	blankFaces, err := brakeOrderedFaces(ctx, refined.blank, budget)
	if err != nil {
		return zero, err
	}
	toolFaces, err := brakeOrderedFaces(ctx, refined.body, budget)
	if err != nil {
		return zero, err
	}
	resultFaces, err := brakeOrderedFaces(ctx, result, budget)
	if err != nil {
		return zero, err
	}
	for _, item := range []struct {
		payload facetedrecord.Payload
		faces   []*topo.Face
	}{{candidatePayload, toolFaces}, {resultPayload, resultFaces}} {
		p := item.payload
		if len(p.Src) != len(p.Tris) || len(p.FaceOf) != len(p.Tris) {
			return zero, errBrakeRefinedFacetAncestryLengthMismatch
		}
		for i := range p.Tris {
			if err := budget.SpendWork(ctx, 1); err != nil {
				return zero, err
			}
			group, face := p.Src[i], p.FaceOf[i]
			if group < 0 || group >= len(p.Groups) || face < 0 || face >= len(item.faces) ||
				p.Groups[group].Source != item.faces[face].Source {
				return zero, newBrakeAncestryError(ctx, budget, brakeFacetAncestryMismatch, i, 0, 0)
			}
		}
	}
	sources := []*topo.Body{refined.slot, refined.pin}
	var leafFaces [2][]*topo.Face
	for i, body := range sources {
		leafFaces[i], err = brakeOrderedFaces(ctx, body, budget)
		if err != nil {
			return zero, err
		}
	}
	// Revalidate each original and refined U leaf against the same actual source
	// face. Identity is used only after candidate geometry was authenticated.
	checkLeaf := func(g facetedrecord.Group) error {
		s := g.Source
		if s == nil || s.Blend || s.Operand < 0 || s.Operand > 1 {
			return errBrakeRefinedToolLeafMissing
		}
		fs, owner := leafFaces[s.Operand], sources[s.Operand]
		if s.Ordinal < 0 || s.Ordinal >= len(fs) || s.Producer != owner.Origin.Producer || s.Parent != fs[s.Ordinal].Source {
			return errBrakeRefinedToolLeafIdentityChanged
		}
		if err := budget.SpendWork(ctx, uint64(len(g.Origins))+uint64(len(g.Upstream))+1); err != nil {
			return err
		}
		if !slices.Equal(g.Origins, fs[s.Ordinal].Origins) || !slices.Equal(g.Upstream, fs[s.Ordinal].Upstream) {
			return errBrakeRefinedToolLeafOriginsChanged
		}
		return nil
	}
	for _, g := range originalPayload.Groups {
		if err := checkLeaf(g); err != nil {
			return zero, err
		}
	}
	for _, g := range candidatePayload.Groups {
		if err := checkLeaf(g); err != nil {
			return zero, err
		}
		found := false
		for _, old := range originalPayload.Groups {
			if err := budget.SpendWork(ctx, uint64(len(old.Origins))+uint64(len(g.Origins))+uint64(len(old.Upstream))+uint64(len(g.Upstream))+1); err != nil {
				return zero, err
			}
			if old.Source != nil && *old.Source == *g.Source && slices.Equal(old.Origins, g.Origins) && slices.Equal(old.Upstream, g.Upstream) {
				found = true
				break
			}
		}
		if !found {
			return zero, errBrakeRefinedToolLeafAbsentFromOriginalU
		}
	}
	if err := budget.ReserveRecords(ctx, uint64(len(resultPayload.Groups))); err != nil {
		return zero, err
	}
	if err := budget.ReserveArray(ctx, uint64(len(resultPayload.Groups)), uint64(unsafe.Sizeof(facetedrecord.Group{}))); err != nil {
		return zero, err
	}
	groups := slices.Clone(resultPayload.Groups)
	for i, g := range resultPayload.Groups {
		if err := budget.SpendWork(ctx, uint64(len(g.Origins))+uint64(len(g.Upstream))+1); err != nil {
			return zero, err
		}
		s := g.Source
		if s == nil || s.Blend {
			return zero, newBrakeAncestryError(ctx, budget, brakeSourceGroupMissing, i, 0, 0)
		}
		var faces []*topo.Face
		switch {
		case s.Producer == refined.blankID && s.Operand == 0:
			faces = blankFaces
		case s.Producer == refined.body.Origin.Producer && s.Operand == 1:
			faces = toolFaces
		default:
			return zero, newBrakeAncestryError(ctx, budget, brakeSourceGroupProducer, i, int(s.Producer), s.Operand)
		}
		if s.Ordinal < 0 || s.Ordinal >= len(faces) || faces[s.Ordinal].Source != s.Parent || !slices.Equal(g.Origins, faces[s.Ordinal].Origins) || !slices.Equal(g.Upstream, faces[s.Ordinal].Upstream) {
			return zero, newBrakeAncestryError(ctx, budget, brakeSourceGroupFace, i, 0, 0)
		}
		if s.Operand == 1 {
			if s.Parent == nil {
				return zero, errBrakeRefinedToolLeafMissingParent
			}
			groups[i].Source = s.Parent
		}
	}
	for _, face := range resultFaces {
		found := false
		for _, g := range resultPayload.Groups {
			if err := budget.SpendWork(ctx, uint64(len(g.Origins))+1); err != nil {
				return zero, err
			}
			if face.Source == g.Source && slices.Equal(face.Origins, g.Origins) {
				found = true
				break
			}
		}
		if !found {
			return zero, errBrakeRefinedClevisFaceAncestryMissing
		}
	}
	resultPayload.Groups = groups
	refined.verifiedGrowth, refined.growthVerified = growth, true
	return resultPayload, nil
}
