package prismrecord

import (
	"math"

	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
)

// ExactCarriers names the loops of a displaced section whose carriers are
// exact (agent-docs/prism-boolean-design.md §7, "Exact carriers"). A loop is
// listed when it is, segment for segment, a loop of an operand whose own
// record states that loop's carriers exactly: an operand with no section
// displacement, or one whose own ExactCarriers lists the loop. Each listed
// segment's carrier, the line through its recorded Start and End or the
// circle its recorded Center and radius state, is then a carrier of the
// denoted section's boundary, and every point the denoted boundary takes from
// that loop lies on it.
//
// The listing says nothing about where a listed loop's walk ends: another
// loop's displaced set may still reach it within the payload's SectionDelta,
// so every coordinate reading keeps charging SectionDelta. Only a reader of a
// loop's carriers, such as a retained surface tag, may take a listed loop as
// exact.
//
// SectionDelta is the payload's section displacement when the listing was
// made. A copy that changes the displacement or any loop record leaves the
// listing stale, and Payload.CarrierExact then answers false for every loop:
// a stale listing withholds a claim and never states one.
type ExactCarriers struct {
	SectionDelta float64
	Loops        []sectionrecord.LoopRecord
}

// CarrierExact reports whether loop li of the section (0 is the outer loop,
// i+1 is hole i) has exact carriers. A payload with no section displacement
// states every carrier exactly. A displaced payload answers true only for a
// loop its ExactCarriers lists, bit for bit, made at its current displacement.
func (pp Payload) CarrierExact(budget *proofbound.WorkBudget, li int) (bool, error) {
	loops := pp.sectionLoops()
	if li < 0 || li >= len(loops) {
		return false, nil
	}
	if pp.SectionDelta == 0 {
		return true, nil
	}
	ec := pp.ExactCarriers
	if ec == nil || math.Float64bits(ec.SectionDelta) != math.Float64bits(pp.SectionDelta) {
		return false, nil
	}
	for _, listed := range ec.Loops {
		same, err := momentinput.ExactLoopEqual(budget, listed, loops[li])
		if err != nil || same {
			return same, err
		}
	}
	return false, nil
}

// ListExactCarriers is the listing a construction gives its result when the
// result's section keeps operand loops verbatim: every loop of result that
// is, bit for bit, a loop of an operand whose carriers that operand states
// exactly. Only an operand on result's own frame and placement is read, since
// a loop's coordinates name the same points only in the same plane frame.
// It returns nil when result has no section displacement (every carrier is
// then exact without a listing) or when no loop qualifies.
func ListExactCarriers(budget *proofbound.WorkBudget, result Payload, operands ...Payload) (*ExactCarriers, error) {
	if result.SectionDelta == 0 {
		return nil, nil //nolint:nilnil // an undisplaced section states every carrier without a listing
	}
	var same []Payload
	for _, op := range operands {
		if op.Frame == result.Frame && op.Xform == result.Xform {
			same = append(same, op)
		}
	}
	var listed []sectionrecord.LoopRecord
	for _, loop := range result.sectionLoops() {
		exact, err := operandCarrierExact(budget, loop, same)
		if err != nil {
			return nil, err
		}
		if exact {
			listed = append(listed, loop)
		}
	}
	if len(listed) == 0 {
		return nil, nil //nolint:nilnil // no loop qualifies, so there is no listing
	}
	return &ExactCarriers{SectionDelta: result.SectionDelta, Loops: listed}, nil
}

// operandCarrierExact reports whether loop is, bit for bit, a loop of one of
// operands whose carriers that operand states exactly.
func operandCarrierExact(budget *proofbound.WorkBudget, loop sectionrecord.LoopRecord, operands []Payload) (bool, error) {
	for _, op := range operands {
		for li, candidate := range op.sectionLoops() {
			same, err := momentinput.ExactLoopEqual(budget, candidate, loop)
			if err != nil {
				return false, err
			}
			if !same {
				continue
			}
			exact, err := op.CarrierExact(budget, li)
			if err != nil || exact {
				return exact, err
			}
		}
	}
	return false, nil
}

// sectionLoops is the section's loops in record order: the outer, then each
// hole.
func (pp Payload) sectionLoops() []sectionrecord.LoopRecord {
	return append([]sectionrecord.LoopRecord{pp.Profile.Outer}, pp.Profile.Holes...)
}
