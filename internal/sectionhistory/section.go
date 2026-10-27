// Package sectionhistory sections a mesh Boolean result through the two
// operands its public operation record names: the section of A op B is the
// planar op of the operands' exact sections, arranged in a private Sketch
// scene (agent-docs/section-design.md, "Mesh Boolean result read through its
// operation history"). It reads no document; the caller supplies the operation
// and the operand payloads.
package sectionhistory

import (
	"context"
	"errors"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismcells"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionclip"
	"github.com/lestrrat-3d/decad/internal/stackedunion"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
)

// Refusal messages, one per gate, so a caller and its tests can tell them apart.
const (
	msgOperand  = `section needs an exact line section of each Boolean operand`
	msgRegion   = `the Boolean section is not one exact region`
	msgJunction = `the Boolean section has a junction Sketch does not certify exact`
)

// Section returns the exact section of op(a, b) on plane, where a and b are
// the payloads of the two operands the result's public record names (a Cut's
// target then its tool). The caller has proved the result denotes op over
// exactly these operands. Every miss returns an error wrapping
// decaderr.ErrUnsupported; cancellation returns ctx.Err().
func Section(ctx context.Context, op meshbool.OperationKind, a, b topo.Payload, plane r3.Frame) (
	*sketch.Sketch, []*sketch.Profile, error,
) {
	keep, err := keeper(op)
	if err != nil {
		return nil, nil, err
	}
	pa, err := operandSection(ctx, a, plane)
	if err != nil {
		return nil, nil, err
	}
	pb, err := operandSection(ctx, b, plane)
	if err != nil {
		return nil, nil, err
	}
	region, err := combine(ctx, pa, pb, keep)
	if err != nil {
		return nil, nil, err
	}
	return sectionclip.ReconstructExactSection(ctx, region, plane)
}

// keeper is the cell selection of op: Union keeps material of either operand,
// Cut of the target and not the tool, Intersect of both.
func keeper(op meshbool.OperationKind) (func(a, b bool) bool, error) {
	switch op {
	case meshbool.OpUnion:
		return func(a, b bool) bool { return a || b }, nil
	case meshbool.OpCut:
		return func(a, b bool) bool { return a && !b }, nil
	case meshbool.OpIntersect:
		return func(a, b bool) bool { return a && b }, nil
	}
	return nil, fmt.Errorf(`%w: section history reads only Union, Cut and Intersect`, decaderr.ErrUnsupported)
}

// combine arranges the two exact sections in one private Sketch scene and
// keeps the cells keep selects (agent-docs/prism-boolean-design.md §4.1-§4.2).
// The result must be one loop of whole lines whose every cut end Sketch
// certified as an exact line crossing. Classify admits only hole-free cells,
// so a single surviving loop is the outer boundary of the selected cells,
// walked counter-clockwise as Sketch walks them.
func combine(ctx context.Context, pa, pb momentinput.Profile, keep func(a, b bool) bool) (momentinput.Profile, error) {
	budget := proofbound.NewWorkBudget(ctx)
	if _, within, err := prismcells.RegionsWithinWorkCap(budget, pa, pb); err != nil || !within {
		if err != nil {
			return momentinput.Profile{}, err
		}
		return momentinput.Profile{}, fmt.Errorf(`%w: %s`, decaderr.ErrUnsupported, msgRegion)
	}
	// Whole lines enter the scene at their recorded points, so the scene's
	// walk charge is zero.
	s, tags, delta, err := prismcells.BuildScene(budget, pa, pb, &prismcells.Reexpression{Identity: true})
	if err != nil {
		return momentinput.Profile{}, err
	}
	profiles, err := prismcells.CellProfiles(ctx, budget, s)
	if err != nil {
		return momentinput.Profile{}, err
	}
	// Both inputs are exact, so the only charge left is the gap across a span
	// two near-coincident carriers share. An exact section allows none.
	if ok, err := delta.ChargeCrossings(budget, tags, profiles, 0, 0, 0); err != nil || !ok || delta.Crossing != 0 {
		if err != nil {
			return momentinput.Profile{}, err
		}
		return momentinput.Profile{}, fmt.Errorf(`%w: %s`, decaderr.ErrUnsupported, msgRegion)
	}
	matterA, matterB, resolved, err := prismcells.Classify(budget, tags, profiles)
	if err != nil {
		return momentinput.Profile{}, err
	}
	if !resolved {
		return momentinput.Profile{}, fmt.Errorf(`%w: %s`, decaderr.ErrUnsupported, msgRegion)
	}
	selected, err := prismcells.Select(budget, profiles, matterA, matterB, keep)
	if err != nil {
		return momentinput.Profile{}, err
	}
	loops, cutDelta, resolved, err := prismcells.MergeLoopsExactLineCuts(budget, s, selected, "section")
	if err != nil {
		if errors.Is(err, context.Canceled) || errors.Is(err, context.DeadlineExceeded) {
			return momentinput.Profile{}, err
		}
		return momentinput.Profile{}, fmt.Errorf(`%w: %s: %w`, decaderr.ErrUnsupported, msgRegion, err)
	}
	if !resolved || len(loops) != 1 {
		return momentinput.Profile{}, fmt.Errorf(`%w: %s`, decaderr.ErrUnsupported, msgRegion)
	}
	if cutDelta != 0 {
		return momentinput.Profile{}, fmt.Errorf(`%w: %s`, decaderr.ErrUnsupported, msgJunction)
	}
	return momentinput.Profile{Outer: loops[0]}, nil
}

// operandSection finds an exact all-line section of one operand in a signed
// axis permutation of plane and maps it into plane's own coordinates.
func operandSection(ctx context.Context, p topo.Payload, plane r3.Frame) (momentinput.Profile, error) {
	for _, perm := range permutations {
		if err := ctx.Err(); err != nil {
			return momentinput.Profile{}, err
		}
		frame, ok := perm.frame(plane)
		if !ok {
			continue
		}
		record, ok, err := exactRecord(ctx, p, frame)
		if err != nil {
			return momentinput.Profile{}, err
		}
		if !ok {
			continue
		}
		lines, ok := wholeLines(record)
		if !ok {
			return momentinput.Profile{}, fmt.Errorf(`%w: %s`, decaderr.ErrUnsupported, msgOperand)
		}
		return perm.mapBack(lines), nil
	}
	return momentinput.Profile{}, fmt.Errorf(`%w: %s`, decaderr.ErrUnsupported, msgOperand)
}

// exactRecord tries the exact section classes Body.Section admits for a
// prism, a stacked prism and a stack-backed BRep, in frame's coordinates.
// Each class proves its own record; ok=false is a miss.
func exactRecord(ctx context.Context, p topo.Payload, frame r3.Frame) (momentinput.Profile, bool, error) {
	pp, isPrism := p.(prismrecord.Payload)
	if isPrism && (pp.SectionDelta != 0 || pp.Z0Delta != 0 || pp.Z1Delta != 0) {
		return momentinput.Profile{}, false, nil
	}
	if operand, ok := stackedunion.OperandFromPayload(p); ok {
		record, err := sectionclip.SectionBrepStackRecord(ctx, operand.Proxy, operand.Slabs, frame)
		switch {
		case err == nil:
			return record, true, nil
		case !errors.Is(err, decaderr.ErrUnsupported):
			return momentinput.Profile{}, false, err
		}
	}
	if !isPrism {
		return momentinput.Profile{}, false, nil
	}
	if record, ok := sectionclip.CardinalRectangleSectionRecord(pp, frame); ok {
		return record, true, nil
	}
	if record, ok := sectionclip.CardinalCircleSectionRecord(pp, frame); ok {
		return record, true, nil
	}
	return momentinput.Profile{}, false, nil
}
