package classbgeom

import (
	"context"

	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/topo"
)

// CutPrismView is class B's Cut of a target x that is a prism view of
// another payload denoting exactly the same solid, such as a turned shaft's
// (agent-docs/general-boolean-design.md §3 "Revolved shaft as a prism"), by a
// prism tool y. It runs AdmitPair under x's placement and then the crossing
// reach, as OfPayloads does for a prism target. Beside those gates it misses
// unless every straight tool wall across x's sweep lies at a level strictly
// outside x's sweep interval. Inside x's slab the tool's boundary then runs
// along x's sweep only, so every surviving piece of x's wall runs from one cap
// to the other, and a tool that ends inside the shaft keeps the mesh path the
// source payload took before this view, rather than reaching §4.2's refusal of
// a rim two swept walls would share. ok=false (err nil) is a silent miss.
func CutPrismView(ctx context.Context, x prismrecord.Payload, y topo.Payload) (topo.Payload, bool, error) {
	cp, ok, err := AdmitPair(ctx, x, y, x.Xform, 0)
	if err != nil || !ok {
		return nil, false, err
	}
	if !toolWallsOutsideSweep(cp, x.Z0, x.Z1) {
		return nil, false, nil
	}
	bp, ok, err := tryClassBCrossingCut(ctx, x, cp)
	if err != nil || !ok {
		return nil, false, err
	}
	return bp, true, nil
}

// toolWallsOutsideSweep reports whether every straight wall of the tool
// that lies across the reference axis 2 (x's sweep) sits at a level strictly
// outside [z0, z1]. Under B6 a tool line either runs along that axis or
// across it, and its level is one recorded coordinate, signed exactly.
func toolWallsOutsideSweep(cp Pair, z0, z1 float64) bool {
	// k is the tool's local section axis that lands on reference axis 2;
	// AdmitPair's B3 leaves it in the section (d is not axis 2).
	k := -1
	for i := range 2 {
		if cp.axis[i] == 2 {
			k = i
		}
	}
	if k < 0 {
		return false
	}
	coord := func(p sectionrecord.Point2) float64 {
		if k == 0 {
			return p.U
		}
		return p.V
	}
	for _, seg := range cp.tool.Profile.Outer.Segments {
		l, ok := seg.(sectionrecord.LineSeg)
		if !ok || coord(l.Start) != coord(l.End) {
			continue
		}
		level := cp.sign[k] * coord(l.Start)
		if level >= z0 && level <= z1 {
			return false
		}
	}
	return true
}
