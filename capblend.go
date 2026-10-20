package decad

import (
	"context"
	"fmt"

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
