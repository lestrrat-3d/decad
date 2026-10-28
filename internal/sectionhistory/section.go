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
	"slices"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismcells"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/revolveplan"
	"github.com/lestrrat-3d/decad/internal/sectionclip"
	"github.com/lestrrat-3d/decad/internal/stackedunion"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
)

// Refusal messages, one per gate, so a caller and its tests can tell them apart.
const (
	msgOperand  = `section needs an exact section of whole lines, arcs and circles of each Boolean operand`
	msgRegion   = `the Boolean section is not a set of exact regions`
	msgJunction = `the Boolean section has a junction Sketch does not certify exact`
	msgCurveCut = `the Boolean section cuts an arc or circle, and Sketch certifies no exact arc junction`
)

// Section returns the exact section of op(a, b) on plane, where a and b are
// the payloads of the two operands the result's public record names (a Cut's
// target then its tool). The caller has proved the result denotes op over
// exactly these operands. It returns one profile per region. Every miss
// returns an error wrapping decaderr.ErrUnsupported; cancellation returns
// ctx.Err().
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
	regions, err := combine(ctx, pa, pb, keep)
	if err != nil {
		return nil, nil, err
	}
	return sectionclip.ReconstructExactSections(ctx, regions, plane)
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

// combine arranges the two exact sections, each a list of regions, in one
// private Sketch scene and keeps the cells keep selects
// (agent-docs/prism-boolean-design.md §4.1-§4.2, agent-docs/section-design.md
// "The 2D operation"). Every recorded loop is declared to Sketch, and each
// cell's membership is Sketch's containment verdict against those loops; the
// edge-direction reading may only falsify it. The kept cells merge into one
// record per connected region, with its holes, whose every cut end Sketch
// certified as an exact line crossing and whose every arc and circle is whole.
func combine(ctx context.Context, pa, pb []momentinput.Profile, keep func(a, b bool) bool) ([]momentinput.Profile, error) {
	budget := proofbound.NewWorkBudget(ctx)
	if _, within, err := prismcells.RegionsWithinWorkCap(budget, slices.Concat(pa, pb)...); err != nil || !within {
		if err != nil {
			return nil, err
		}
		return nil, fmt.Errorf(`%w: %s`, decaderr.ErrUnsupported, msgRegion)
	}
	s, tags, delta, err := prismcells.BuildProfileScene(budget, pa, pb, &prismcells.Reexpression{Identity: true})
	if err != nil {
		return nil, err
	}
	// A narrowed segment enters the scene at a computed walked end. No fixture
	// reaches this refusal: operandSection passes only whole segments, whose
	// walk charge is zero, and a nonzero charge fed to combine directly also
	// enters ChargeCrossings below as incoming displacement, which refuses every
	// cut it touches. It keeps the zero-charge claim checked rather than assumed.
	if delta.A != 0 || delta.B != 0 {
		return nil, fmt.Errorf(`%w: %s`, decaderr.ErrUnsupported, msgRegion)
	}
	loops, declared, err := prismcells.DeclareOperandLoops(budget, s, tags)
	if err != nil {
		return nil, err
	}
	if !declared {
		return nil, fmt.Errorf(`%w: %s`, decaderr.ErrUnsupported, msgRegion)
	}
	profiles, err := prismcells.CellProfiles(ctx, budget, s)
	if err != nil {
		return nil, err
	}
	// Both inputs are exact, so the only charge left is the gap across a span
	// two near-coincident carriers share. An exact section allows none.
	if ok, err := delta.ChargeCrossings(budget, tags, profiles, 0, 0, 0); err != nil || !ok || delta.Crossing != 0 {
		if err != nil {
			return nil, err
		}
		return nil, fmt.Errorf(`%w: %s`, decaderr.ErrUnsupported, msgRegion)
	}
	matterA, matterB, resolved, err := prismcells.ContainedMembership(budget, profiles, loops)
	if err != nil {
		return nil, err
	}
	if !resolved {
		return nil, fmt.Errorf(`%w: %s: Sketch does not prove each cell's containment`, decaderr.ErrUnsupported, msgRegion)
	}
	// A recorded loop Sketch dropped below its area floor has edges no cell
	// walks, while its verdicts still read as proven.
	if walked, err := prismcells.LoopEntitiesWalked(budget, tags, profiles); err != nil || !walked {
		if err != nil {
			return nil, err
		}
		return nil, fmt.Errorf(`%w: %s: Sketch publishes no cell along part of a recorded loop`, decaderr.ErrUnsupported, msgRegion)
	}
	reading, read, err := prismcells.ClassifyNested(budget, tags, profiles)
	if err != nil {
		return nil, err
	}
	// Sketch's verdicts and the edge directions describe one arrangement, so no
	// consistent arrangement reaches a disagreement and no fixture does; the
	// check keeps the verdicts falsifiable rather than trusted.
	if !read || !agrees(reading, matterA, matterB) {
		return nil, fmt.Errorf(`%w: %s`, decaderr.ErrUnsupported, msgRegion)
	}
	selected := make([]bool, len(profiles))
	for i := range profiles {
		selected[i] = keep(matterA[i], matterB[i])
	}
	merged, resolved, err := prismcells.MergeRegionsExact(ctx, budget, s, profiles, selected)
	if err != nil {
		if errors.Is(err, context.Canceled) || errors.Is(err, context.DeadlineExceeded) {
			return nil, err
		}
		return nil, fmt.Errorf(`%w: %s: %w`, decaderr.ErrUnsupported, msgRegion, err)
	}
	if !resolved {
		return nil, fmt.Errorf(`%w: %s`, decaderr.ErrUnsupported, msgRegion)
	}
	if merged.CurvedCut {
		return nil, fmt.Errorf(`%w: %s`, decaderr.ErrUnsupported, msgCurveCut)
	}
	if merged.CutDelta != 0 {
		return nil, fmt.Errorf(`%w: %s`, decaderr.ErrUnsupported, msgJunction)
	}
	return merged.Regions, nil
}

// agrees reports whether every membership the edge-direction reading reached
// matches Sketch's containment verdict. A cell the reading did not reach
// proves nothing either way.
func agrees(reading prismcells.NestedReading, matterA, matterB []bool) bool {
	for i := range matterA {
		if reading.KnownA[i] && reading.A[i] != matterA[i] {
			return false
		}
		if reading.KnownB[i] && reading.B[i] != matterB[i] {
			return false
		}
	}
	return true
}

// operandSection finds an exact section of one operand, made of whole lines,
// arcs and circles, in a signed axis permutation of plane and maps every
// region into plane's own coordinates.
func operandSection(ctx context.Context, p topo.Payload, plane r3.Frame) ([]momentinput.Profile, error) {
	for _, perm := range permutations {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		frame, ok := perm.frame(plane)
		if !ok {
			continue
		}
		records, ok, err := exactRecords(ctx, p, frame)
		if err != nil {
			return nil, err
		}
		if !ok {
			continue
		}
		regions := make([]momentinput.Profile, len(records))
		for i, record := range records {
			whole, ok := wholeCurves(record)
			if !ok {
				return nil, fmt.Errorf(`%w: %s`, decaderr.ErrUnsupported, msgOperand)
			}
			regions[i] = perm.mapBack(whole)
		}
		return regions, nil
	}
	return nil, fmt.Errorf(`%w: %s`, decaderr.ErrUnsupported, msgOperand)
}

// exactRecords tries the exact section classes Body.Section admits for a
// prism, a stacked prism, a stack-backed BRep and a full Revolve, in frame's
// coordinates. Each class proves its own records; ok=false is a miss.
func exactRecords(ctx context.Context, p topo.Payload, frame r3.Frame) ([]momentinput.Profile, bool, error) {
	if rp, ok := p.(revolveplan.Payload); ok {
		records, exact := sectionclip.RevolveAxialSection(rp, frame)
		if !exact {
			records, exact = sectionclip.RevolveNormalSection(rp, frame)
		}
		return records, exact, nil
	}
	pp, isPrism := p.(prismrecord.Payload)
	if isPrism && (pp.SectionDelta != 0 || pp.Z0Delta != 0 || pp.Z1Delta != 0) {
		return nil, false, nil
	}
	if operand, ok := stackedunion.OperandFromPayload(p); ok {
		record, err := sectionclip.SectionBrepStackRecord(ctx, operand.Proxy, operand.Slabs, frame)
		switch {
		case err == nil:
			return []momentinput.Profile{record}, true, nil
		case !errors.Is(err, decaderr.ErrUnsupported):
			return nil, false, err
		}
	}
	if !isPrism {
		return nil, false, nil
	}
	if record, ok := sectionclip.CardinalRectangleSectionRecord(pp, frame); ok {
		return []momentinput.Profile{record}, true, nil
	}
	if record, ok := sectionclip.CardinalCircleSectionRecord(pp, frame); ok {
		return []momentinput.Profile{record}, true, nil
	}
	return nil, false, nil
}
