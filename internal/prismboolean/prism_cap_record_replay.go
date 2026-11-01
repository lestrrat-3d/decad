package prismboolean

import (
	"context"

	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismplacement"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
)

// ReplayPrismCapRecord checks a present body's cap against its prism record.
// The caller must separately bind body and face to the original producer.
func ReplayPrismCapRecord(ctx context.Context, body *topo.Body, face *topo.Face, top bool, budget proof.ReplayKeyBudget) (bool, error) {
	if budget == nil {
		return false, nil
	}
	if err := budget.SpendWork(ctx, 1); err != nil {
		return false, err
	}
	if body == nil || face == nil || face.Body != body || len(face.Origins) != 1 || face.Origins[0].Producer != body.Origin.Producer {
		return false, nil
	}
	built, err := replayBuiltFace(ctx, body, face, budget)
	if err != nil || !built {
		return false, err
	}
	pp, ok := body.Payload.(prismrecord.Payload)
	if !ok {
		return false, nil
	}
	if err := budget.ReserveRecords(ctx, 1); err != nil {
		return false, err
	}
	if err := budget.ReserveBytes(ctx, 512); err != nil {
		return false, err
	}
	sealed, err := body.ReplayPrismBuildPayload(ctx, pp.Frame, pp.Xform, pp.Z0, pp.Z1,
		pp.Z0Delta, pp.Z1Delta, pp.SectionDelta, pp.Z0Denoted, pp.Z1Denoted,
		[]momentinput.Profile{pp.Profile}, budget)
	if err != nil || !sealed {
		return false, err
	}
	role, z, delta, denoted := provenance.RoleCapStart, pp.Z0, pp.Z0Delta, pp.Z0Denoted
	if top {
		role, z, delta, denoted = provenance.RoleCapEnd, pp.Z1, pp.Z1Delta, pp.Z1Denoted
	}
	if face.Origins[0].Role != role || face.Reversed || !face.HasAxialDelta || face.AxialDelta != delta {
		return false, nil
	}
	record := face.CapRecord
	if record == nil || record.Frame != pp.Frame || record.Xform != pp.Xform || record.Z != z || record.ZDenoted == nil {
		return false, nil
	}
	exact, ok, err := prismplacement.ReplayDenotedLevel(ctx, z, delta, denoted, budget)
	if err != nil || !ok {
		return false, err
	}
	cmp, err := proof.ReplayRatCompare(ctx, record.ZDenoted, exact, budget)
	if err != nil || cmp != 0 {
		return false, err
	}
	plane, ok := face.Surface.(surfacegeom.Plane)
	if !ok {
		return false, nil
	}
	if err := budget.SpendWork(ctx, 1024); err != nil {
		return false, err
	}
	if err := budget.ReserveRecords(ctx, 8); err != nil {
		return false, err
	}
	if err := budget.ReserveBytes(ctx, 4096); err != nil {
		return false, err
	}
	frame, err := pp.CapFrame(z, !top)
	if err != nil {
		return false, err
	}
	if plane.Frame != frame {
		return false, nil
	}
	found := false
	for _, lump := range body.Lumps {
		if err := budget.SpendWork(ctx, 1); err != nil {
			return false, err
		}
		if lump == nil {
			return false, nil
		}
		for _, shell := range lump.Shells {
			if err := budget.SpendWork(ctx, 1); err != nil {
				return false, err
			}
			if shell == nil {
				return false, nil
			}
			for _, candidate := range shell.Faces {
				if err := budget.SpendWork(ctx, 1); err != nil {
					return false, err
				}
				if candidate == face {
					if found {
						return false, nil
					}
					found = true
				}
			}
		}
	}
	return found, nil
}
