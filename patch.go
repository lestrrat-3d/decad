package decad

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/momentinput"

	"github.com/lestrrat-3d/decad/internal/survey2d"

	"github.com/lestrrat-3d/decad/internal/proofbound"

	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// This file is Document.Patch of docs/surface-design.md §5.1: a single planar
// face built from a recorded profile, on the sketch plane's own frame. It is
// the seam call (recordProfile) plus one evaluation, evalPatchContext, that
// mirrors evalPrismContext's order without ever building a solid.
//
// patchPayload records what the body was built from — profile, frame and
// placement — exactly as prismPayload does for an extrude, so Placed,
// Duplicate and PlacedCopy re-evaluate it the same way (docs/evaluator-design.md
// §8). It is its own type rather than a zero-height prismPayload: roughly
// sixteen call sites across the package type-assert prismPayload, each
// ok-guarded or carrying a default arm, so a patch falls through to the
// existing refusal there instead of being silently handed a zero-height
// prism. prism() below is the one bridge — a *view*, never routed through
// evalPrismContext, which rejects h <= 0 with ErrDegenerate — that lets a
// patch reuse capFrame, prismPayload.point and prismBoundsContext, each of
// which is correct at zero height.
type patchPayload struct {
	profile ProfileRecord
	frame   r3.Frame
	xform   r3.Transform
	walks   *momentinput.ProfileWalks
}

// transform is the accumulated rigid placement.
func (pp patchPayload) transform() r3.Transform { return pp.xform }

// placed re-evaluates the same record under the composed motion (core §8).
func (pp patchPayload) placed(ctx context.Context, d *Document, ref producerID, composed r3.Transform) (*Body, error) {
	pp.xform = composed
	return evalPatchContext(ctx, d, ref, pp, freeform.NewFreeformWork())
}

// prism is a *view* of pp as a zero-height prismPayload — never a body this
// evaluator builds — used only to feed capFrame, prismPayload.point and
// prismBoundsContext, each of which reads a section and a placement and is
// correct at z0 == z1 == 0. Every prismPayload field this view leaves at its
// zero value (z0Delta, z1Delta, sectionDelta, blend roles, surfaceResult)
// is meaningless for a patch, which denotes its own recorded section exactly
// as a plain extrude does.
func (pp patchPayload) prism() prismPayload {
	return prismPayload{profile: pp.profile, frame: pp.frame, xform: pp.xform, walks: pp.walks}
}

// Patch records p through the seam exactly as Extrude does — the same
// authentication, staleness, foreign-entity and TExact admission gates of
// docs/api-design.md §7 and docs/sketch-seam-design.md, with no relaxation —
// and builds a single planar face on s.Plane().Frame(), carrying the
// profile's outer loop and every hole loop (docs/surface-design.md §5.1). The
// result is a one-face sheet body: Kind() == BodySheet, one lump, one open
// shell, and one free edge per coalesced boundary walk. p MUST be a profile
// of s (ErrForeignProfile) and a current, unaltered snapshot (ErrStaleProfile
// or ErrInvalidProfile); an invalid profile is also ErrInvalidProfile, and a
// boundary decad cannot record exactly is ErrUnrecordableProfile. A recorded
// boundary carrying a free-form segment this evaluator cannot integrate is
// ErrUnsupported (Table R row R3). A failed evaluation leaves the document
// untouched.
func (d *Document) Patch(ctx context.Context, s *sketch.Sketch, p *sketch.Profile) (*Body, error) {
	if ctx == nil {
		return nil, fmt.Errorf(`%w: a nil context cannot control a patch`, ErrDegenerate)
	}
	if d == nil {
		return nil, fmt.Errorf(`%w: a nil document owns no model`, ErrDegenerate)
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	profile, plane, profileArea, err := recordProfile(s, p)
	if err != nil {
		return nil, err
	}
	// ONE free-form work counter for this whole call, same convention as
	// Extrude: the area falsifier's preflight opens it and evalPatchContext's
	// own walk resolution spends what is left (docs/spline-design.md §5.2).
	work := freeform.NewFreeformWork()
	if _, err := falsifyRecordedArea(profile, profileArea, work); err != nil {
		return nil, err
	}
	frame, err := r3.NewFrame(plane.Origin, plane.U, plane.V)
	if err != nil {
		return nil, fmt.Errorf(`%w: the recorded plane is degenerate: %s`, ErrDegenerate, err)
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	ref := d.nextProducerID()
	body, err := evalPatchContext(ctx, d, ref, patchPayload{
		profile: profile,
		frame:   frame,
		xform:   r3.Identity(),
	}, work)
	if err != nil {
		return nil, err
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	d.commit(body)
	return body, nil
}

// evalPatchContext builds the patch body from the payload: one planar face
// carrying one loop per recorded loop, the free edges that bound it, and the
// measurements the finished body publishes. It mirrors evalPrismContext's own
// order (prism_build.go) without ever building a solid.
func evalPatchContext(ctx context.Context, d *Document, ref producerID, pp patchPayload, work *freeform.FreeformWork) (*Body, error) {
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	ig, err := pp.profile.EvaluatorIntegralsContext(ctx, freeform.MomentAreaOrder, work)
	if err != nil {
		return nil, err
	}
	if ig.Area <= 0 {
		return nil, fmt.Errorf(`%w: the recorded region encloses no area`, ErrDegenerate)
	}

	body := &Body{doc: d, origin: FeatureRef{producer: ref, Role: roleBody}, solid: false, kind: BodySheet}
	prismView := pp.prism()

	// pw resolves every boundary segment's walk exactly once for this whole
	// build (internal/momentinput/profile_walks.go), read back by every
	// buildPatchLoop call below instead of each one resolving afresh — the
	// same reuse evalPrismContext runs for its own loops.
	pw := pp.walks
	if pw.Reusable(pp.profile) {
		if err := pw.Charge(work); err != nil {
			return nil, err
		}
	} else {
		resolved, err := momentinput.ResolveProfileWalks(pp.profile, work)
		if err != nil {
			return nil, err
		}
		pw = resolved
	}

	loops := append([]LoopRecord{pp.profile.Outer}, pp.profile.Holes...)
	var faceLoops []*Loop
	for li, loop := range loops {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		coedges, err := buildPatchLoop(ctx, prismView, li, loop, work, pw)
		if err != nil {
			return nil, err
		}
		faceLoops = append(faceLoops, &Loop{coedges: coedges, outer: li == 0})
	}

	// The positive side is the sketch plane's own normal — the sense
	// Direction.Along names for that plane (docs/surface-design.md §5.1) —
	// so the face's frame is capFrame at zero height with no flip, unlike a
	// prism's start cap.
	frame, err := capFrame(prismView, 0, false)
	if err != nil {
		return nil, err
	}
	face := &Face{
		surface:       Plane{Frame: frame},
		origins:       []FeatureRef{{producer: ref, Role: rolePatch}},
		body:          body,
		loops:         faceLoops,
		area:          ig.Area,
		areaBound:     ig.AreaBound,
		axialDelta:    0,
		hasAxialDelta: true,
	}
	if err := attachFaceLoopsContext(ctx, []*Face{face}); err != nil {
		return nil, err
	}

	shell := &Shell{faces: []*Face{face}, open: true}
	body.lumps = []*Lump{{shells: []*Shell{shell}}}

	body.area = Measurement{
		Value:     units.SquareMillimeters(ig.Area),
		Exactness: exactnessOf(ig.AreaBound),
		Bound:     units.SquareMillimeters(ig.AreaBound),
	}
	// volume and centroid stay at their zero value: finite, so
	// validateAnalyticBodyMeasurements below passes, and neither is
	// reachable through Body.Volume/Body.Centroid while solid is false
	// (docs/surface-design.md §8).
	bounds, err := prismBoundsContext(ctx, prismView, work, pw)
	if err != nil {
		return nil, err
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	body.bounds = bounds
	if err := chargePrismMap(body, pp.frame, pp.xform); err != nil {
		return nil, err
	}
	if err := validateAnalyticBodyMeasurements(body); err != nil {
		return nil, err
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	pp.walks = pw
	body.payload = pp
	return body, nil
}

// buildPatchLoop builds one recorded loop's free boundary: one vertex per
// junction, one edge per coalesced walk, and exactly one coedge per edge
// (forward: true) — a patch has no side wall and no cap to share an edge
// with, so every edge it mints is free by construction. It reuses
// buildLoopSidesAs's own per-kind curve construction and its circular
// edgeSign rule (prism_build.go), which buildLoopSidesAs's own doc comment
// notes is a template rather than a callable for exactly this reason: a
// patch's loop has no bottom/top pair, no vertical edge and no per-walk face.
//
// pp is patchPayload.prism()'s zero-height view. holeLoop is li != 0, the
// same convention buildLoopSides derives it by. resolved is pp.profile's
// pre-resolved segment walks, or nil to resolve each segment through walkOf
// as before (internal/momentinput/profile_walks.go).
func buildPatchLoop(ctx context.Context, pp prismPayload, li int, loop LoopRecord, work *freeform.FreeformWork, resolved *momentinput.ProfileWalks) ([]coedge, error) {
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	if len(loop.Segments) == 0 {
		return nil, fmt.Errorf(`%w: a recorded loop holds no segments`, ErrDegenerate)
	}
	holeLoop := li != 0

	var loopWalks []survey2d.SegmentWalk
	if resolved != nil {
		if !resolved.LoopMatches(li, loop) {
			return nil, momentinput.ErrResolvedWalksMismatch
		}
		loopWalks = resolved.LoopWalks(li)
	}

	raw := make([]survey2d.SideWalk, len(loop.Segments))
	for i, seg := range loop.Segments {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		seg, err := normalizeSegment(seg)
		if err != nil {
			return nil, err
		}
		var w survey2d.SegmentWalk
		if loopWalks != nil {
			w = loopWalks[i]
		} else {
			w, err = boundarywalk.WalkOf(seg, work)
			if err != nil {
				return nil, err
			}
		}
		raw[i] = survey2d.SideWalk{SegmentWalk: w, Segs: []int{i}}
	}
	walks, err := boundarywalk.CoalesceWalksContext(ctx, raw)
	if err != nil {
		return nil, err
	}
	n := len(walks)
	singleClosed := n == 1 && walks[0].Closed

	// A patch's own record is its own denotation (patchPayload carries no
	// section displacement), so the section and level terms below are always
	// zero; they are composed anyway, exactly as buildLoopSidesAs composes its
	// rim vertices, so a later field this payload gains is charged the same way
	// rather than silently skipped. Beside them every rim vertex carries its
	// own exact frame lift and placement rounding (prismPayload.liftedVertex),
	// zero wherever that lift is exact for the coordinates at hand.
	rimVertex := func(u, v, extra float64) *Vertex {
		held, lift := pp.liftedVertex(u, v, pp.z0)
		base := proofbound.AbsSumUpper(pp.sectionDelta, pp.z0Delta, lift)
		return &Vertex{position: held, bound: units.Millimeters(proofbound.AbsSumUpper(base, extra))}
	}
	var seam *Vertex
	var verts []*Vertex
	if singleClosed {
		w := walks[0]
		seam = rimVertex(junctionVertexAt(loop.Segments, w, w))
	} else {
		verts = make([]*Vertex, n)
		for i, w := range walks {
			if err := ctx.Err(); err != nil {
				return nil, err
			}
			prev := walks[(i+n-1)%n]
			verts[i] = rimVertex(junctionVertexAt(loop.Segments, prev, w))
		}
	}

	coedges := make([]coedge, 0, n)
	for i, w := range walks {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		var start, end *Vertex
		if singleClosed {
			start, end = seam, seam
		} else {
			start, end = verts[i], verts[(i+1)%n]
		}
		convex, err := rimConvexity(ctx, w, holeLoop, work)
		if err != nil {
			return nil, err
		}
		var curve Curve
		switch w.Kind {
		case survey2d.WalkCircular:
			// The same circular edgeSign rule buildLoopSidesAs's survey2d.WalkCircular
			// arm applies to its own rim edges: an Arc3/Circle3 is CCW from
			// start to end about its axis, and a clockwise walk (or a
			// reflected placement) inverts that sense.
			clockwise := w.Th1 < w.Th0
			edgeSign := 1.0
			if clockwise {
				edgeSign = -1
			}
			if pp.reflected() {
				edgeSign = -edgeSign
			}
			edgeAxis := pp.dir(0, 0, 1).Scale(edgeSign)
			center := pp.point(w.CU, w.CV, pp.z0)
			radius := units.Millimeters(w.Radius)
			if singleClosed {
				curve = Circle3{Center: center, Axis: edgeAxis, Radius: radius}
			} else {
				curve = Arc3{Center: center, Axis: edgeAxis, Radius: radius}
			}
		case survey2d.WalkFreeform:
			curve = NURBSCurve{}
		default:
			curve = Line3{}
		}
		edge := &Edge{curve: curve, start: start, end: end, convex: convex, length: w.Length, lengthBound: w.LengthBound}
		switch c := curve.(type) {
		case Circle3:
			edge.curveBound, edge.curveBounded = pp.circleCurveBound(w.CU, w.CV, pp.z0, pp.z0Delta, 0, w.Radius, w.RadiusBound, c.Center, c.Axis)
		case Arc3:
			edge.curveBound, edge.curveBounded = pp.circleCurveBound(w.CU, w.CV, pp.z0, pp.z0Delta, 0, w.Radius, w.RadiusBound, c.Center, c.Axis)
		}
		coedges = append(coedges, coedge{edge: edge, forward: true})
	}
	return coedges, nil
}
