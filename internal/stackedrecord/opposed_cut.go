package stackedrecord

import (
	"context"

	"github.com/lestrrat-3d/decad/internal/prismcells"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/r3"
)

// OpposedCut is a blind cut on a stack whose tool ends exactly at an existing
// interface and leaves exclusive holes on both sides of it: pockets from
// opposite faces that overlap in plan at one level
// (agent-docs/stacked-prism-design.md §7 stage 3). Its slabs are proven as
// CutSlabs proves them, but the payload's whole column rings cannot hold the
// partial floor and ceiling, so the caller states the body as a BRep
// (agent-docs/general-boolean-design.md §3 A8).
type OpposedCut struct {
	Slabs []Slab
	// Interfaces holds every other interface's derived record. Interfaces[At]
	// is empty: its faces are cells of a private scene over LowerOnly and
	// UpperOnly.
	Interfaces           []Interface
	At                   int
	LowerOnly, UpperOnly []sectionrecord.LoopRecord
	Frame                r3.Frame
	Xform                r3.Transform
}

// ApplyOpposedBlindCut admits the interface arm of ApplyBlindCut that CutSlabs
// leaves to the mesh path because its opposed exclusive holes are not apart.
// The dispatch tries ApplyBlindCut first, so opposed holes the stacked payload
// records never reach this arm. Every reached slab still needs its own
// whole-loop cut proof, the section displacement must be zero, and every
// exclusive hole at the interface must be a loop of axis-aligned lines, whose
// crossings are exact. ok=false is a silent miss.
func ApplyOpposedBlindCut(ctx context.Context, budget *proofbound.WorkBudget, sp Payload,
	outer, tool prismrecord.Payload) (OpposedCut, bool, error) {
	reach, ok, err := reachBlindCut(budget, sp, outer, tool)
	if err != nil || !ok || reach.interfaceAt < 0 {
		return OpposedCut{}, false, err
	}
	// The interface arm admits equal denoted levels that carry a
	// displacement; this arm keeps to undisplaced ones, whose held floats are
	// the levels the BRep faces state.
	at := reach.interfaceAt
	if reach.innerDelta != 0 || sp.Slabs[at].Z1Delta != 0 || sp.Slabs[at+1].Z0Delta != 0 {
		return OpposedCut{}, false, nil
	}
	active := reach.interfaceActive(len(sp.Slabs))
	reexpress, err := prismcells.NewReexpression(outer.PlacementOperand(), tool.PlacementOperand())
	if err != nil {
		return OpposedCut{}, false, err
	}
	result, ok, err := cutActiveSlabs(ctx, budget, sp, outer, tool, reexpress, active, sp.Interfaces)
	if err != nil || !ok || result.SectionDelta != 0 {
		return OpposedCut{}, false, err
	}
	lowerOnly, upperOnly := ExclusiveHoles(result.Slabs[at].Regions[0], result.Slabs[at+1].Regions[0])
	if len(lowerOnly) == 0 || len(upperOnly) == 0 {
		return OpposedCut{}, false, nil
	}
	for _, hole := range append(append([]sectionrecord.LoopRecord(nil), lowerOnly...), upperOnly...) {
		if !opposedLoop(hole) {
			return OpposedCut{}, false, nil
		}
	}
	interfaces := make([]Interface, len(result.Slabs)-1)
	for i := range interfaces {
		if i == at {
			continue
		}
		interfaces[i], err = deriveInterface(ctx, budget, result.Slabs, sp.Interfaces, i)
		if err != nil {
			return OpposedCut{}, false, err
		}
	}
	return OpposedCut{Slabs: result.Slabs, Interfaces: interfaces, At: at,
		LowerOnly: lowerOnly, UpperOnly: upperOnly, Frame: sp.Frame, Xform: sp.Xform}, true, nil
}

// opposedLoop reports whether every segment of loop is a line along one
// section axis, a circle or an arc. Two axis-aligned lines meet at their
// exact levels; a line crossing a circle is a cut sketch certifies, which the
// A1 engine places as A7 places it. An oblique line, an ellipse or a
// free-form segment is refused: the engine states no exact crossing for it.
func opposedLoop(loop sectionrecord.LoopRecord) bool {
	if len(loop.Segments) == 0 {
		return false
	}
	for _, segment := range loop.Segments {
		switch s := segment.(type) {
		case sectionrecord.LineSeg:
			if s.Start.U != s.End.U && s.Start.V != s.End.V {
				return false
			}
		case sectionrecord.CircleSeg, sectionrecord.ArcSeg:
		default:
			return false
		}
	}
	return true
}
