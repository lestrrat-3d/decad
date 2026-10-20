package loftcontact

import (
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/topo"
)

// SourceFaces maps each source role the body's own producer stamped on a face
// to that face, charging the budget for every lump, shell, face and origin it
// reads. It reports false, with no error, when the origin count passes
// VolumeOutputSteps or one role names two different faces. The face nodes are
// read directly because the copying accessors would allocate before the budget
// has charged for them.
func SourceFaces(body *topo.Body, budget *proofbound.WorkBudget) (map[string]*topo.Face, bool, error) {
	var roles int
	for _, lump := range body.Lumps {
		if err := budget.Step(); err != nil {
			return nil, false, err
		}
		for _, shell := range lump.Shells {
			if err := budget.Step(); err != nil {
				return nil, false, err
			}
			for _, face := range shell.Faces {
				if err := budget.Step(); err != nil {
					return nil, false, err
				}
				if len(face.Origins) > VolumeOutputSteps ||
					roles > VolumeOutputSteps-len(face.Origins) {
					return nil, false, nil
				}
				roles += len(face.Origins)
				if _, err := budget.StepN(uint64(len(face.Origins))); err != nil {
					return nil, false, err
				}
			}
		}
	}
	// Walk the born nodes directly after charging for the map and its source-role entries.
	if err := budget.Err(); err != nil {
		return nil, false, err
	}
	if err := budget.Step(); err != nil {
		return nil, false, err
	}
	faces := make(map[string]*topo.Face, roles)
	for _, lump := range body.Lumps {
		if err := budget.Step(); err != nil {
			return nil, false, err
		}
		for _, shell := range lump.Shells {
			if err := budget.Step(); err != nil {
				return nil, false, err
			}
			for _, node := range shell.Faces {
				for _, origin := range node.Origins {
					if err := budget.Step(); err != nil {
						return nil, false, err
					}
					if origin.Producer != body.Origin.Producer {
						continue
					}
					if previous := faces[origin.Role]; previous != nil && previous != node {
						return nil, false, nil
					}
					faces[origin.Role] = node
				}
			}
		}
	}
	return faces, true, nil
}
