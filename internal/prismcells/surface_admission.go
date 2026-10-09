package prismcells

import (
	"fmt"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismplacement"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// SurfaceOperation selects the Trim, Extend or Split surface gate.
type SurfaceOperation uint8

const (
	SurfaceExtend SurfaceOperation = iota
	SurfaceTrim
	SurfaceSplit
)

// SurfaceOperand holds the section and placement needed by the common gates.
type SurfaceOperand struct {
	Profile   momentinput.Profile
	Placement prismplacement.Operand
}

// ProfileAnalytic reports G4: every recorded segment is a line, circle or arc.
// A free-form segment blinds sketch's whole-scene TExact gate. The scan
// charges work even when it finds an unsupported segment.
func ProfileAnalytic(budget *proofbound.WorkBudget, p momentinput.Profile) (bool, error) {
	for _, loop := range append([]momentinput.LoopRecord{p.Outer}, p.Holes...) {
		for _, seg := range loop.Segments {
			if err := budget.Step(); err != nil {
				return false, err
			}
			switch seg.(type) {
			case momentinput.LineSeg, momentinput.CircleSeg, momentinput.ArcSeg:
			default:
				return false, nil
			}
		}
	}
	return true, nil
}

// AdmitSurfacePair applies the common S2-S7 gates after the caller has checked
// body family, section shape and section displacement in its required order.
func AdmitSurfacePair(budget *proofbound.WorkBudget, op SurfaceOperation, receiver, tool SurfaceOperand) error {
	if receiver.Placement.Xform.IsReflection() || tool.Placement.Xform.IsReflection() {
		return fmt.Errorf(`%w: %s does not admit a reflected operand`, decaderr.ErrUnsupported, surfaceName(op))
	}
	rcvAnalytic, err := ProfileAnalytic(budget, receiver.Profile)
	if err != nil {
		return err
	}
	tlAnalytic, err := ProfileAnalytic(budget, tool.Profile)
	if err != nil {
		return err
	}
	if !rcvAnalytic || !tlAnalytic {
		if op == SurfaceTrim {
			return fmt.Errorf(`%w: Trim admits only line, circle and arc segments; a free-form segment blinds sketch's whole-scene TExact gate`, decaderr.ErrUnsupported)
		}
		return fmt.Errorf(`%w: %s admits only line, circle and arc segments`, decaderr.ErrUnsupported, surfaceName(op))
	}
	rcv, tl := receiver.Placement, tool.Placement
	worldNormalRcv := rcv.Xform.ApplyDir(rcv.Frame.N())
	worldNormalTool := tl.Xform.ApplyDir(tl.Frame.N())
	if worldNormalRcv != worldNormalTool {
		return surfaceGeneratorError(op)
	}
	worldOriginRcv := rcv.Xform.Apply(rcv.Frame.Origin())
	worldOriginTool := tl.Xform.Apply(tl.Frame.Origin())
	if worldOriginTool.Sub(worldOriginRcv).Dot(worldNormalRcv) != 0.0 {
		return surfaceGeneratorError(op)
	}
	if op == SurfaceTrim && len(tool.Profile.Holes) != 0 {
		return fmt.Errorf(`%w: Trim's tool section carries a hole, whose interior this evaluator cannot yet distinguish as a side`, decaderr.ErrUnsupported)
	}
	if !prismplacement.CutZIntervalSpans(rcv, tl) {
		if op == SurfaceSplit {
			return fmt.Errorf(`%w: the tool does not span the target over the sweep parameter`, decaderr.ErrUnsupported)
		}
		return fmt.Errorf(`%w: the tool does not span the receiver over the sweep parameter`, decaderr.ErrUnsupported)
	}
	reexpress, err := NewReexpression(rcv, tl)
	if err != nil {
		return err
	}
	if !reexpress.Identity {
		if op == SurfaceSplit {
			return fmt.Errorf(`%w: the target and tool do not share one frame and placement, so their re-expression is not the identity`, decaderr.ErrUnsupported)
		}
		return fmt.Errorf(`%w: the receiver and tool do not share one frame and placement, so their re-expression is not the identity`, decaderr.ErrUnsupported)
	}
	if op == SurfaceExtend {
		whole, err := TrimProfileFullyWhole(budget, tool.Profile.Outer, tool.Profile.Holes)
		if err != nil {
			return err
		}
		if !whole {
			return fmt.Errorf(`%w: every segment the tool consumes must span its entity's own natural domain`, decaderr.ErrUnsupported)
		}
		return nil
	}
	rcvWhole, err := TrimProfileFullyWhole(budget, receiver.Profile.Outer, receiver.Profile.Holes)
	if err != nil {
		return err
	}
	tlWhole, err := TrimProfileFullyWhole(budget, tool.Profile.Outer, tool.Profile.Holes)
	if err != nil {
		return err
	}
	if !rcvWhole || !tlWhole {
		if op == SurfaceSplit {
			return fmt.Errorf(`%w: every segment the target or tool consumes must span its entity's own natural domain`, decaderr.ErrUnsupported)
		}
		return fmt.Errorf(`%w: every segment the receiver or tool consumes must span its entity's own natural domain`, decaderr.ErrUnsupported)
	}
	return nil
}

func surfaceGeneratorError(op SurfaceOperation) error {
	if op == SurfaceSplit {
		return fmt.Errorf(`%w: the target and tool do not sweep along the same generator, exactly`, decaderr.ErrUnsupported)
	}
	return fmt.Errorf(`%w: the receiver and tool do not sweep along the same generator, exactly`, decaderr.ErrUnsupported)
}

func surfaceName(op SurfaceOperation) string {
	switch op {
	case SurfaceExtend:
		return "Extend"
	case SurfaceTrim:
		return "Trim"
	default:
		return "Split"
	}
}
