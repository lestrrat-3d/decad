package decad

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/brepgeom"
	"github.com/lestrrat-3d/decad/internal/offset2d"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/stackedrecord"
)

// shellThroughRoundPort shells a rectangular prism cut by one whole-circle
// through tool when the tool's cylindrical wall is the sole removed face.
// The tool wall is replaced by the two short cylindrical rims through the
// exterior wall thickness. Between them the inset rectangular cavity opens
// into the port. All three sections are exact stack records, so the stack's
// analytic mass and mesh proofs apply without a new surface kind.
func shellThroughRoundPort(ctx context.Context, source *Body, bp brepPayload, tc throughCut,
	call brepShellCall) (*Body, bool, error) {
	if len(call.removed) != 1 || len(tc.tools) != 1 || len(tc.tools[0].walls) != 1 {
		return nil, false, nil
	}
	tool := tc.tools[0]
	wall := bp.faces[tool.walls[0]]
	if call.removed[0] != facesByRole(source)[wall.role] {
		return nil, false, nil
	}
	refuse := func(reason string) (*Body, bool, error) {
		return nil, true, fmt.Errorf(`%w: the removed circular through wall %s %s (modify-general SG4)`,
			ErrUnsupported, wall.role, reason)
	}
	// Seven source faces and the two rectangular cap readings certify that
	// the one tool passes through a box. No other hole or curved outer wall
	// can be discarded by the three-section rewrite.
	if len(bp.faces) != 7 || len(tc.caps.Section.Holes) != 0 || len(bp.faces[tool.w0].region.Holes) != 1 ||
		len(bp.faces[tool.w1].region.Holes) != 1 {
		return refuse("needs one rectangular box and one through hole")
	}
	if _, ok := recordedBossRect(tc.caps.Section); !ok {
		return refuse("needs a rectangular box section")
	}
	for _, face := range bp.faces {
		if face.delta != 0 || face.z0Delta != 0 || face.z1Delta != 0 {
			return refuse("needs exact source faces and levels")
		}
	}
	if call.tDelta != 0 {
		return refuse("needs an exactly converted thickness")
	}
	outer, ok := brepgeom.NewPrismMap(tc.embeds[tool.w0], tool.frameEmb).Loop(bp.faces[tool.w0].region.Outer)
	if !ok {
		return refuse("cannot restate the pierced rectangle in the tool frame")
	}
	rect, ok := recordedBossRect(profileRecord{Outer: outer})
	if !ok {
		return refuse("needs a rectangular pierced wall")
	}
	circle, ok := recordedBossCircle(tool.prism.profile)
	if !ok {
		return refuse("needs one exact whole-circle through tool")
	}
	t := call.tmm
	inner := rect.inset(t)
	loInner, hiInner := tool.lo+t, tool.hi-t
	if !(loInner < hiInner && inner.width() > 0 && inner.height() > 0 &&
		circle.center.U-circle.radius > inner.u0 && circle.center.U+circle.radius < inner.u1 &&
		circle.center.V-circle.radius > inner.v0 && circle.center.V+circle.radius < inner.v1) {
		return refuse("has too little wall thickness or port clearance")
	}
	if !bossRectArithmeticExact(rect, t) ||
		proofbound.ExactSumRound(loInner, tool.lo, t) != 0 ||
		proofbound.ExactSumRound(hiInner, tool.hi, -t) != 0 ||
		proofbound.ExactSumRound(tool.hi-tool.lo, tool.hi, -tool.lo) != 0 {
		return refuse("needs exactly represented offset coordinates")
	}
	for _, c := range []float64{circle.center.U, circle.center.V} {
		if proofbound.ExactSumRound(c-circle.radius, c, -circle.radius) != 0 ||
			proofbound.ExactSumRound(c+circle.radius, c, circle.radius) != 0 {
			return refuse("needs exactly represented circle extents")
		}
	}
	// The box around the circle is a strict recorded-coordinate proof that
	// the circle is inside the inset rectangle. The interface ring has no
	// crossings or tangency, so its outer and inner loops bound one face.
	port, err := offset2d.ReverseLoopRecordContext(ctx, circle.outer.Outer)
	if err != nil {
		return nil, true, err
	}
	cavity, err := offset2d.ReverseLoopRecordContext(ctx, inner.profile().Outer)
	if err != nil {
		return nil, true, err
	}
	end := profileRecord{Outer: outer, Holes: []loopRecord{port}}
	middle := profileRecord{Outer: outer, Holes: []loopRecord{cavity}}
	ring, err := stackedrecord.EnclosingExposed(ctx, cavity, []loopRecord{port})
	if err != nil {
		return nil, true, err
	}
	sp := stackedPrismPayload{frame: tool.prism.frame, xform: bp.xform,
		slabs: []stackedrecord.Slab{
			{Regions: []profileRecord{end}, Z0: tool.lo, Z1: loInner},
			{Regions: []profileRecord{middle}, Z0: loInner, Z1: hiInner},
			{Regions: []profileRecord{end}, Z0: hiInner, Z1: tool.hi},
		},
		interfaces: []stackedrecord.Interface{{LowerExposed: ring}, {UpperExposed: ring}},
	}
	result, err := evalStackedContext(ctx, source.doc, source.doc.nextProducerID(), sp)
	return result, true, err
}
