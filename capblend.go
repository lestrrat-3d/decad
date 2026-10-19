package decad

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/capband"
	"github.com/lestrrat-3d/decad/internal/capblend"
	"github.com/lestrrat-3d/decad/internal/modifyselect"
	"github.com/lestrrat-3d/decad/internal/offset2d"

	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// This file is the root entry of docs/modify-reach-design.md PR E (§14), which
// lands §8.3's cap-loop chamfer: the receiver/selection classification RX1's
// second class needs (every geometric edge of one or more COMPLETE prism cap
// loops, never mixed with lateral edges — S4), gates
// SX4/SX6/SX7/SX10/SX12, and the hand-off to internal/capblend, which owns the
// payload (capBlendPayload), the setback-span audit, SX13's axial half, SX14's
// corner-locus half, the BX3 roles and the build. The cap-loop FILLET
// (§8.2, Cylinder/Torus/Sphere patches) is in row E's staged column and is
// not implemented here. Each chamfered cap carries its own two setbacks
// (capSetback): dc = ds = d for an equal chamfer, and the pair §8.3.1 assigns
// from WithAsymmetricChamfer's reference face for a two-distance one.

// classifyChamferSelection is RX1's second class plus SX4 (Table SX):
// distinguishes a lateral-edge selection (the base modify-design §7 case,
// left to the existing code path unchanged) from a complete-cap-loop
// selection, and refuses a selection that mixes the two classes or covers
// only part of a cap loop. It returns the loop indices selected per cap when
// the selection is a clean cap-loop selection; lateral is true when every
// selected edge is instead an ordinary lateral edge (the caller then runs the
// base path). caps names the two cap faces whose loops are the cap loops: the
// receiver's own capStart/capEnd faces, or a brep receiver's route P caps
// (docs/brep-modify-design.md §4.2), each face loop mapped to its section
// loop.
func classifyChamferSelection(ctx context.Context, pp prismPayload, caps prismCaps, sel EdgeSelector, edges []*Edge) (startLoops, endLoops map[int]bool, lateral bool, err error) {
	budget := proofbound.NewWorkBudget(ctx)
	cornerLoops, err := offset2d.ProfileCornerLoopsBudget(budget, pp.Profile)
	if err != nil {
		return nil, nil, false, err
	}

	// Index every rim edge of every complete cap loop by (cap, loop).
	type capKey struct {
		start bool
		loop  int
	}
	capEdgeOf := map[*Edge]capKey{}
	capLoopSize := map[capKey]int{}
	for _, c := range []struct {
		face  *Face
		start bool
	}{{caps.start, true}, {caps.end, false}} {
		if c.face == nil {
			continue
		}
		for li, l := range c.face.Loops() {
			key := capKey{start: c.start, loop: caps.sectionLoop(c.start, li)}
			for _, e := range l.Edges() {
				capEdgeOf[e] = key
				capLoopSize[key]++
			}
		}
	}

	lateralCount, capCount := 0, 0
	touched := map[capKey]map[*Edge]bool{}
	var unmatched *Edge
	for ei, e := range edges {
		if _, _, found, mErr := modifyselect.MatchCorner(budget, pp, cornerLoops, edgeNode(e)); mErr != nil {
			return nil, nil, false, mErr
		} else if found {
			lateralCount++
			continue
		}
		if key, ok := capEdgeOf[e]; ok {
			capCount++
			if touched[key] == nil {
				touched[key] = map[*Edge]bool{}
			}
			touched[key][e] = true
			continue
		}
		if unmatched == nil {
			unmatched = edges[ei]
		}
	}

	switch {
	case lateralCount > 0 && capCount == 0 && unmatched == nil:
		return nil, nil, true, nil
	case lateralCount > 0 && capCount > 0:
		return nil, nil, false, fmt.Errorf(`%w: the selection mixes lateral edges with prism cap-loop edges in one call; selector %s`, ErrUnsupported, sel)
	case capCount == 0:
		if unmatched != nil {
			return nil, nil, false, fmt.Errorf(`%w: a chamfer of a cap edge is the vertex-blend problem, not yet supported; selector %s`, ErrUnsupported, sel)
		}
		return nil, nil, false, fmt.Errorf(`%w: the selector matched no edges to chamfer`, ErrNoMatch)
	}

	startLoops = map[int]bool{}
	endLoops = map[int]bool{}
	for key, set := range touched {
		if len(set) != capLoopSize[key] {
			return nil, nil, false, fmt.Errorf(`%w: the selection covers only part of a cap loop; every geometric edge of a complete loop must be selected; selector %s`, ErrUnsupported, sel)
		}
		if key.start {
			startLoops[key.loop] = true
		} else {
			endLoops[key.loop] = true
		}
	}
	return startLoops, endLoops, false, nil
}

// buildCapBlend runs the existence and constructed-geometry gates (SX6, SX7,
// SX12, and SX13's axial half) and, once every gate passes, builds the body. It
// is the shared entry Chamfer calls once a clean cap-loop selection is
// classified. SX13's radial half is decided per circular wall as the band is
// constructed, in capband.BandRadius. start and end are the two
// caps' own setbacks (§8.3.1); a cap with no selected loop reads neither.
func buildCapBlend(ctx context.Context, doc *Document, ref producerID, pp prismPayload, start, end capSetback, startLoops, endLoops map[int]bool) (*Body, error) {
	return buildCapBlendWithLoftSource(ctx, doc, ref, pp, start, end, startLoops, endLoops, nil)
}

func buildCapBlendWithLoftSource(ctx context.Context, doc *Document, ref producerID, pp prismPayload,
	start, end capSetback, startLoops, endLoops map[int]bool, source *loftCapBandSource) (*Body, error) {
	height := pp.Z1 - pp.Z0
	loops := append([]loopRecord{pp.Profile.Outer}, pp.Profile.Holes...)
	cbp := capBlendPayload{
		Profile:    pp.Profile,
		Frame:      pp.Frame,
		Z0:         pp.Z0,
		Z1:         pp.Z1,
		Z0Delta:    pp.Z0Delta,
		Z1Delta:    pp.Z1Delta,
		Xform:      pp.Xform,
		Start:      start,
		End:        end,
		StartLoops: startLoops,
		EndLoops:   endLoops,
		LoftSource: source,
	}

	// SX6 + SX7/SX12: build the "mixed" profile — every selected loop offset
	// its own dc into the material (the cap contour at whichever cap it is
	// selected on; when a loop is selected on BOTH caps the two contours are
	// equal — resolveCapSetbacks gives both caps of such a loop one dc — so
	// one offset serves both),
	// every unselected loop left unchanged — and run the existing exact
	// offset + §5 audit machinery on it. SX6 is the offset's own drop
	// refusal, re-sentinelled here (capband.ChamferDropError); the audit's own
	// refusals keep the sentinel their §4 stage-6 row decided
	// (capband.ChamferAuditError) — SX7/SX12 for a crossing or contact, the base
	// S8/S9 ErrDegenerate for broken nesting. SX12 audits the exact offset
	// FAMILY, not the ruled patch the body builds: at axial fraction s, the
	// denoted miter locus is the parallel section offset by s*dc
	// (docs/modify-reach-design.md §8.3) — a fact about that family alone,
	// independent of which surface later reports the body's volume — and the
	// offset distance to any fixed feature is monotone non-increasing as the
	// offset grows from 0, so a crossing anywhere in the family occurs no
	// later than it occurs at the full offset dc — proving the family
	// disjoint at s=1 certifies every s in [0, 1].
	budget := proofbound.NewWorkBudget(ctx)
	mixed, err := capblend.MixedOffsetProfile(budget, cbp)
	if err != nil {
		return nil, capband.ChamferDropError(err)
	}

	// SX7 (band-meeting): a loop chamfered on both caps needs both bands to
	// fit without meeting; a loop chamfered on one cap needs its own band to
	// fit within the sweep. A band reaches its own cap's ds along the sweep
	// (§8.3.1), whatever its dc.
	//
	// It runs AFTER SX6, which docs/modify-reach-design.md puts in stage 5 while
	// this row is stage 6 ("SX6 precedes SX7"). The two rows answer the same §4
	// existence test in opposite directions, and stage order is how §4 keeps one
	// input from having two sentinels: a setback that empties the cap contour
	// names a body that does not exist at any sweep height, so it is SX6's
	// ErrDegenerate whether the sweep is short enough for the bands to meet or
	// not. Deciding the band reach first would let the sweep height alone pick
	// which sentinel that one nonexistent body reports.
	for li := range loops {
		reach := 0.0
		if startLoops[li] {
			reach += start.Ds
		}
		if endLoops[li] {
			reach += end.Ds
		}
		if reach >= height {
			return nil, fmt.Errorf(`%w: the chamfer band(s) on loop %d reach or pass the opposite end of the sweep; a merging kernel is not available`, ErrUnsupported, li)
		}
	}

	if err := offset2d.AuditOffsetSectionBudget(budget, pp.Profile, mixed); err != nil {
		return nil, capband.ChamferAuditError(err)
	}
	if err := capblend.AuditSetbackSpan(budget, pp.Profile, cbp); err != nil {
		return nil, err
	}
	if err := capblend.RequireLevelsSeparate(cbp); err != nil {
		return nil, err
	}
	if err := capblend.RequireCornerLoci(budget, cbp); err != nil {
		return nil, err
	}
	return bodyResult(capblend.Build(ctx, envOf(documentNode(doc), ref), cbp))
}
