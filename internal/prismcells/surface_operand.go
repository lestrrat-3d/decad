package prismcells

import (
	"context"

	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// SurfaceOperandOf adapts a prism record to the neutral surface gate's input.
func SurfaceOperandOf(p prismrecord.Payload) SurfaceOperand {
	return SurfaceOperand{Profile: p.Profile, Placement: p.PlacementOperand()}
}

// SplitPieces asks sketch for the bounded cells of the private scene, keeps
// precisely the cells on the target's material side, and records each selected
// cell from that arrangement as a prism record over the target's sweep fields,
// carrying the cell's own δ_cut. The revolve arm passes MERIDIAN views, whose
// sweep fields are zero, and reads only each cell's profile and displacement.
func SplitPieces(ctx context.Context, budget *proofbound.WorkBudget, target, tool prismrecord.Payload) ([]prismrecord.Payload, error) {
	selected, err := ResolveSurfaceSplit(ctx, budget,
		SurfaceOperandOf(target), SurfaceOperandOf(tool))
	if err != nil {
		return nil, err
	}
	result := make([]prismrecord.Payload, len(selected))
	for i, cell := range selected {
		result[i] = prismrecord.Payload{
			Profile: cell.Profile, Frame: target.Frame, Xform: target.Xform,
			Z0: target.Z0, Z1: target.Z1,
			Z0Delta: target.Z0Delta, Z1Delta: target.Z1Delta,
			Z0Denoted: target.Z0Denoted, Z1Denoted: target.Z1Denoted,
			SectionDelta: cell.CutDelta,
		}
	}
	return result, nil
}
