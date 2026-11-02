package prismboolean

import (
	"context"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/topo"
)

// ReplayPrismCap finds the unique start or end cap and authenticates it against
// the body's sealed prism record. A missing, duplicate or unproved cap declines.
// ctx must not be nil; a nil body or budget declines without work.
func ReplayPrismCap(ctx context.Context, body *topo.Body, end bool,
	budget *proofbound.ReplayBudget) (*topo.Face, bool, error) {
	if body == nil || budget == nil {
		return nil, false, nil
	}
	role := provenance.RoleCapStart
	if end {
		role = provenance.RoleCapEnd
	}
	var found *topo.Face
	for _, lump := range body.Lumps {
		if err := budget.SpendWork(ctx, 1); err != nil {
			return nil, false, err
		}
		if lump == nil {
			return nil, false, nil
		}
		for _, shell := range lump.Shells {
			if err := budget.SpendWork(ctx, 1); err != nil {
				return nil, false, err
			}
			if shell == nil {
				return nil, false, nil
			}
			for _, face := range shell.Faces {
				if err := budget.SpendWork(ctx, 1); err != nil {
					return nil, false, err
				}
				if face == nil || len(face.Origins) != 1 || face.Origins[0].Role != role {
					continue
				}
				if found != nil {
					return nil, false, nil
				}
				found = face
			}
		}
	}
	if found == nil {
		return nil, false, nil
	}
	ok, err := ReplayPrismCapRecord(ctx, body, found, end, budget)
	return found, ok, err
}
