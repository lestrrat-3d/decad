package brepmodify

import (
	"context"
	"fmt"
	"math"

	"github.com/lestrrat-3d/r3"

	"github.com/lestrrat-3d/decad/internal/breprecord"
	"github.com/lestrrat-3d/decad/internal/capblend"
	"github.com/lestrrat-3d/decad/internal/cupwall"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/draftrecord"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/offset2d"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/prismshell"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/revolveplan"
	"github.com/lestrrat-3d/decad/internal/revolveshell"
	"github.com/lestrrat-3d/decad/internal/stackedrecord"
	"github.com/lestrrat-3d/decad/internal/stackedshell"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/sweeprecord"
	"github.com/lestrrat-3d/decad/internal/throughshell"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/twistsweep"
	"github.com/lestrrat-3d/units"
)

// ShellCall is what Shell hands the node-level shell once it has resolved the
// options, the selector and the thickness: the faces to remove (empty for
// NoOpenings), the wall sense and the thickness as given, in millimetres, with
// its conversion bound.
type ShellCall struct {
	Removed     []*topo.Face
	NoOpenings  bool
	Inward      bool
	T           units.Value
	Tmm, TDelta float64
	// CurveName names a curve in the public API's vocabulary, for the
	// refusals that print one.
	CurveName func(surfacegeom.Curve) string
}

// Shell builds the shell of b from the stage-2 gates on: a receiver payload
// class, then the removed faces. The result is not yet committed.
func Shell(ctx context.Context, env topo.Env, b *topo.Body, call ShellCall) (*topo.Body, error) {
	t, tmm, tDelta := call.T, call.Tmm, call.TDelta
	s := 1.0 // inward: erode
	if !call.Inward {
		s = -1.0 // outward: dilate
	}
	if call.NoOpenings {
		// The one nil-selector shell: no face is removed, so there is no
		// selection to resolve. SX10 still leads for a cap-blend receiver.
		if err := capblend.RequireNotReceiver(b.Payload, "shells"); err != nil {
			return nil, err
		}
		// Reach §9.3: a full turn over a hole-free meridian keeps every face
		// and sweeps its closed wall region over the whole turn.
		if rp, ok := b.Payload.(revolveplan.Payload); ok && rp.Full && len(rp.Profile.Holes) == 0 {
			return shellRevolve(ctx, env, b, rp, nil, s, t, tmm, tDelta)
		}
		// Reach BX5: a hole-free straight prism keeps both caps and lines
		// them with a wall, closing a cavity.
		if pp, ok := b.Payload.(prismrecord.Payload); ok && len(pp.Profile.Holes) == 0 {
			return shellClosedPrism(ctx, env, pp, s, t, tmm, tDelta)
		}
		if pp, ok := sweeprecord.StraightPrismReduction(b.Payload); ok && len(pp.Profile.Holes) == 0 {
			return shellClosedPrism(ctx, env, pp, s, t, tmm, tDelta)
		}
		if sp, ok := b.Payload.(stackedrecord.Payload); ok && call.Inward {
			n, matched, err := stackedshell.Closed(ctx, env, sp, stackedshell.Call{Inward: true, T: tmm, TDelta: tDelta})
			if err != nil {
				return nil, err
			}
			if matched {
				return n, nil
			}
		}
		return nil, stackedshell.RefuseClosed(b.Payload)
	}
	removed := call.Removed

	// SX10: a capBlendPayload receiver is staged before the generic "not a
	// prism" refusal, so the more specific reason leads.
	if err := capblend.RequireNotReceiver(b.Payload, "shells"); err != nil {
		return nil, err
	}
	if out, taken, err := draftCircleShell(ctx, env, b, removed, call.Inward, tmm, tDelta); taken {
		return out, err
	}
	if err := draftrecord.RequireNotReceiver(b.Payload, "shells"); err != nil {
		return nil, err
	}
	// Reach RX2 (agent-docs/modify-reach-design.md §9.3): a revolve receiver shells
	// its meridian and sweeps the wall over its own angular interval.
	if rp, ok := b.Payload.(revolveplan.Payload); ok {
		return shellRevolve(ctx, env, b, rp, removed, s, t, tmm, tDelta)
	}
	if tp, ok := b.Payload.(twistsweep.Payload); ok {
		return shellTwistedSweep(ctx, env, b, tp, removed, call.Inward, tmm, tDelta)
	}
	if fp, ok := b.Payload.(facetedrecord.Payload); ok && fp.PrismUnionFillet != nil {
		if node, matched, err := prismshell.PrismUnionBossShell(ctx, env, b, fp, removed, call.Inward, tmm, tDelta); matched || err != nil {
			return node, err
		}
	}
	// A brep or stacked receiver takes the brep route
	// (agent-docs/brep-modify-design.md §2), ahead of the generic refusal.
	route, err := Receiver(ctx, env, b, Request{Op: "shells", Shell: true,
		Admits: func(_ prismrecord.Payload, caps PrismCaps) error {
			_, _, err := classifyRemovedCaps(caps, removed)
			return err
		},
		ShellCall: throughshell.Call{Removed: removed, Inward: call.Inward, T: t, Tmm: tmm, TDelta: tDelta, Tol: offset2d.ShellTol},
		CurveName: call.CurveName})
	if err != nil {
		return nil, err
	}
	if route.Body != nil {
		return route.Body, nil
	}

	// Stage 2 (§4): the receiver's payload class (S3), then the removed faces:
	// caps by role, and side faces by the receiver's exact prism segment,
	// which routes the call to the side opening (shell-opening §5). A brep
	// receiver's route P takes caps only (SB3).
	pp, ok := b.Payload.(prismrecord.Payload)
	caps := CapsOf(b)
	var sweepStraight bool
	if route.Prism != nil {
		pp, ok, caps = *route.Prism, true, route.Caps
	} else if sweepPrism, admitted := sweeprecord.StraightPrismReduction(b.Payload); admitted {
		pp, ok, sweepStraight = sweepPrism, true, true
	}
	if !ok {
		return nil, fmt.Errorf(`%w: this evaluator shells a straight prism or its one-line Sweep reduction only`, decaderr.ErrUnsupported)
	}
	if err := pp.RequireExactSection("shells"); err != nil {
		return nil, err
	}
	var removedStart, removedEnd bool
	var sides map[int]struct{}
	switch {
	case route.Prism != nil:
		removedStart, removedEnd, err = classifyRemovedCaps(caps, removed)
	case sweepStraight:
		removedStart, removedEnd, sides, err = classifyRemovedSweepFaces(b, caps, pp, removed)
	default:
		removedStart, removedEnd, sides, err = classifyRemovedFaces(b, caps, removed)
	}
	if err == nil && len(sides) > 0 {
		return shellSideOpening(ctx, env, pp, removedStart, removedEnd, sides, s, t, tmm, tDelta)
	}
	if err != nil {
		return nil, err
	}
	bothCaps := removedStart && removedEnd

	in := prismshell.CapShellInput{Prism: pp, KeptCaps: 1, Sense: s, Thickness: t, HeldThickness: tmm, ThicknessDelta: tDelta}
	if bothCaps {
		in.KeptCaps = 0
	}
	capShell, err := cupwall.PlanCapShell(ctx, in, removedEnd)
	if err != nil {
		return nil, err
	}
	return capShell.Build(ctx, env)
}

// shellClosedPrism is reach BX5 (agent-docs/modify-reach-design.md §9.2), built by
// prismshell.BuildClosed: WithNoOpenings on a hole-free straight prism keeps
// every face and lines it with a wall of thickness t, closing a cavity whose
// walls form one void shell (FaceShell.IsVoid).
func shellClosedPrism(ctx context.Context, env topo.Env, pp prismrecord.Payload, s float64, t units.Value, tmm, tDelta float64) (*topo.Body, error) {
	return prismshell.BuildClosed(ctx, env, prismshell.CapShellInput{
		Prism: pp, KeptCaps: 2, Sense: s, Thickness: t, HeldThickness: tmm, ThicknessDelta: tDelta,
	})
}

// classifyRemovedCaps decides which caps a removed-face set names on a brep
// receiver's route P (agent-docs/brep-modify-design.md §4.2): every face must be one
// of the recognised prism's two cap faces, else ErrUnsupported, which the
// route reports as SB3. It reports whether the start cap, the end cap, or
// both were removed. A prism receiver classifies its removed faces with
// classifyRemovedFaces instead, which routes side faces to the side opening.
func classifyRemovedCaps(caps PrismCaps, removed []*topo.Face) (bool, bool, error) {
	var start, end bool
	for _, f := range removed {
		switch {
		case caps.Start != nil && f == caps.Start:
			start = true
		case caps.End != nil && f == caps.End:
			end = true
		default:
			// Route P maps removed faces to the recognised prism's caps only
			// (shell-opening SO7).
			return false, false, fmt.Errorf(`%w: a shell that removes a side wall is not supported by this evaluator`, decaderr.ErrUnsupported)
		}
	}
	return start, end, nil
}

// shellRevolve shells a revolve receiver through the faces in removed (nil for a
// WithNoOpenings full turn). s is +1 inward, -1 outward.
func shellRevolve(ctx context.Context, env topo.Env, b *topo.Body, rp revolveplan.Payload, removed []*topo.Face,
	s float64, t units.Value, tmm, tDelta float64) (*topo.Body, error) {
	caps := CapsOf(b)
	return revolveshell.Shell(ctx, env, b, rp, caps.Start, caps.End, removed, s, t, tmm, tDelta, offset2d.ShellTol)
}

// classifyRemovedFaces sorts a prism shell's removed faces (§5, stage 2):
// the caps by their roles, and every other face by the side(0,j) roles of the
// receiver it carries, j being the recorded outer-loop segment the face
// sweeps. It reports whether each cap is removed and the removed outer-loop
// segments. A face that is neither a cap nor a side face of the outer loop —
// a hole loop's wall — is SO6, decaderr.ErrUnsupported.
func classifyRemovedFaces(b *topo.Body, caps PrismCaps, removed []*topo.Face) (bool, bool, map[int]struct{}, error) {
	var start, end bool
	sides := map[int]struct{}{}
	for _, f := range removed {
		switch {
		case caps.Start != nil && f == caps.Start:
			start = true
			continue
		case caps.End != nil && f == caps.End:
			end = true
			continue
		}
		named := false
		for _, o := range f.Origins {
			var li, j int
			if o.Producer != b.Origin.Producer {
				continue
			}
			if n, _ := fmt.Sscanf(o.Role, "side(%d,%d)", &li, &j); n != 2 {
				continue
			}
			if li != 0 {
				return false, false, nil, fmt.Errorf(`%w: a side opening removes faces of the section's outer loop only, and this face lines a hole (modify-reach SX8, shell-opening SO6)`, decaderr.ErrUnsupported)
			}
			sides[j] = struct{}{}
			named = true
		}
		if !named {
			return false, false, nil, fmt.Errorf(`%w: a removed face is neither a cap nor a side face of this prism (shell-opening SO6)`, decaderr.ErrUnsupported)
		}
	}
	return start, end, sides, nil
}

// classifyRemovedSweepFaces maps the published one-line Sweep wall role
// side(0,0,j) to segment j of its stored exact prism. The first zero names
// the only path span; the second names the section's outer loop. No role from
// another span or a hole loop is a side-opening target.
func classifyRemovedSweepFaces(b *topo.Body, caps PrismCaps, pp prismrecord.Payload, removed []*topo.Face) (bool, bool, map[int]struct{}, error) {
	var start, end bool
	sides := map[int]struct{}{}
	for _, f := range removed {
		switch {
		case caps.Start != nil && f == caps.Start:
			start = true
			continue
		case caps.End != nil && f == caps.End:
			end = true
			continue
		}
		if len(f.Origins) != 1 || f.Origins[0].Producer != b.Origin.Producer {
			return false, false, nil, fmt.Errorf(`%w: a selected Sweep wall has no single source segment (shell-opening SO6)`, decaderr.ErrUnsupported)
		}
		role := f.Origins[0].Role
		var span, loop, segment int
		n, err := fmt.Sscanf(role, "side(%d,%d,%d)", &span, &loop, &segment)
		if err != nil || n != 3 || fmt.Sprintf("side(%d,%d,%d)", span, loop, segment) != role ||
			span != 0 || segment < 0 || segment >= len(pp.Profile.Outer.Segments) {
			return false, false, nil, fmt.Errorf(`%w: a selected face does not map to this Sweep's stored prism (shell-opening SO6)`, decaderr.ErrUnsupported)
		}
		if loop != 0 {
			return false, false, nil, fmt.Errorf(`%w: a side opening removes faces of the section's outer loop only (shell-opening SO6)`, decaderr.ErrUnsupported)
		}
		sides[segment] = struct{}{}
	}
	return start, end, sides, nil
}

// shellSideOpening is Shell's side opening on a hole-free straight
// prism (agent-docs/shell-opening-design.md): sides names the removed outer-loop
// segments, and removedStart/removedEnd the removed caps. The regions are
// built and audited first (prismshell.SideOpeningRegions); both caps removed records W
// as a prism over the receiver's sweep (BO1), and otherwise the slabs are
// stated as a breprecord.Payload (BO2, breprecord.SideOpening).
func shellSideOpening(ctx context.Context, env topo.Env, pp prismrecord.Payload, removedStart, removedEnd bool, sides map[int]struct{}, s float64, t units.Value, tmm, tDelta float64) (*topo.Body, error) {
	budget := proofbound.NewWorkBudget(ctx)
	if err := budget.Err(); err != nil {
		return nil, err
	}
	keptCaps := 0
	for _, removed := range []bool{removedStart, removedEnd} {
		if !removed {
			keptCaps++
		}
	}
	sec, err := prismshell.SideOpeningRegions(budget, prismshell.SideOpeningInput{
		Profile: pp.Profile, Height: pp.Z1 - pp.Z0, Sides: sides,
		KeptCaps: keptCaps, Sense: s, Thickness: t, HeldThickness: tmm,
		ThicknessDelta: tDelta, Tolerance: offset2d.ShellTol,
	}, offset2d.AuditOffsetSectionBudget)
	if err != nil {
		return nil, prismshell.CancelCause(err)
	}
	var body *topo.Body
	if keptCaps == 0 {
		body, err = prismshell.SideOpeningPrism(ctx, env, pp, sec)
	} else {
		var bp breprecord.Payload
		bp, err = breprecord.SideOpening(ctx, budget, pp, sec, removedStart, removedEnd, s, tmm, tDelta)
		if err == nil {
			body, err = breprecord.Build(ctx, env, bp)
		}
	}
	if err != nil {
		return nil, prismshell.CancelCause(err)
	}
	return body, nil
}

// shellTwistedSweep admits a top opening on a smooth, centrally symmetric,
// strictly convex one-line twisted section. The true cavity is the inward
// normal offset of the swept wall; its held faceted boundary is separately
// certified against that denoted source.
func shellTwistedSweep(ctx context.Context, env topo.Env, b *topo.Body, p twistsweep.Payload,
	removed []*topo.Face, inward bool, thickness, thicknessDelta float64) (*topo.Body, error) {
	if !inward || len(removed) != 1 || removed[0] != CapsOf(b).End ||
		p.Path == nil || len(p.Path.Records) != 1 || p.Path.Records[0].Arc != nil ||
		p.Path.Start != (r3.Vec{}) || p.Path.End.X != 0 || p.Path.End.Y != 0 ||
		p.Path.End.Z <= 0 || p.Plane.Origin != (r3.Vec{}) ||
		p.Plane.U != r3.NewVec(1, 0, 0) || p.Plane.V != r3.NewVec(0, 1, 0) ||
		thicknessDelta != 0 {
		return nil, fmt.Errorf(`%w: twisted shell needs one straight axial sweep, a top opening, and exact inward thickness`, decaderr.ErrUnsupported)
	}
	height := p.Path.End.Z
	source, err := twistsweep.TwistOffsetSource(p.Profile, height, p.Angle, thickness, thicknessDelta)
	if err != nil {
		return nil, err
	}
	stations, err := twistsweep.TwistOffsetSampleStations(p.Profile, height, p.Angle,
		thickness, thicknessDelta, 8, 16)
	if err != nil {
		return nil, err
	}
	poly, err := twistsweep.TwistPolygonOf(p.Profile)
	if err != nil {
		return nil, err
	}
	held, err := twistsweep.TwistShellSampledMesh(ctx, stations, height, thickness,
		p.Angle, poly.AreaInterval)
	if err != nil {
		return nil, err
	}
	node, err := twistsweep.ShellFacetedBody(ctx, env, held,
		poly.AreaInterval, source.Area, height, thickness)
	if err != nil {
		return nil, err
	}
	out := node
	if p.Xform != r3.Identity() {
		gap, proofErr := twistsweep.TwistShellPlacedProof(p.Profile, height, thickness,
			p.Angle, source, p.Xform)
		if proofErr != nil {
			return nil, proofErr
		}
		facets, ok := out.Payload.(facetedrecord.Payload)
		if !ok {
			return nil, fmt.Errorf(`%w: twisted shell lost its held cup`, decaderr.ErrUnsupported)
		}
		facets.VertexBound = append([]float64(nil), facets.VertexBound...)
		for i := range facets.VertexBound {
			facets.VertexBound[i] = proofbound.AbsSumUpper(facets.VertexBound[i], gap.Boundary)
		}
		facets.MeshBound = proofbound.AbsSumUpper(facets.MeshBound, gap.Boundary)
		facets.VolSymDiff = proofbound.AbsSumUpper(facets.VolSymDiff, gap.Volume)
		facets.AreaSlack = proofbound.AbsSumUpper(facets.AreaSlack, gap.Area)
		out, err = facets.Placed(ctx, env, p.Xform)
		if err != nil {
			return nil, err
		}
	}
	return out, nil
}

// fullTurnRadians is the end angle of a full revolution.
const fullTurnRadians = 2 * math.Pi
