package decad

import (
	"context"
	"fmt"
	"math"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// This file builds a straight extrude's body from its prismPayload: the two
// cap faces, one side face per recorded segment, and the topology that joins
// them, together with the measurements the finished body publishes.
//
// evalPrismContext is the whole build and states the order; buildLoopSidesAs
// is the per-loop walk it runs once per outer loop and once per hole, with
// the hole's orientation reversed. Every face this file stamps carries the
// displacement its own surface was built from. See
// docs/evaluator-design.md §5.

// evalPrism builds the analytic prism body from the payload: side faces per
// boundary segment, two caps, shared edges and vertices, and Exact
// measurements (docs/evaluator-design.md §5). The payload's segment kinds
// are line, circle and arc; anything else has already been rejected by the
// mass-property integrals it runs first.
//
// work is the record's ONE free-form work counter: the preflight this build runs
// and every walkOf under it charge the same ceiling, and a caller that already
// spent part of it on this record passes it in rather than opening a second one.
func evalPrism(d *Document, ref StepRef, pp prismPayload, work *freeformWork) (*Body, error) {
	return evalPrismContext(context.Background(), d, ref, pp, work)
}

func evalPrismContext(ctx context.Context, d *Document, ref StepRef, pp prismPayload, work *freeformWork) (*Body, error) {
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	ig, err := pp.profile.evaluatorIntegralsUncheckedContext(ctx, momentFirstOrder, work)
	if err != nil {
		return nil, err
	}
	if ig.area <= 0 {
		return nil, fmt.Errorf(`%w: the recorded region encloses no area`, ErrDegenerate)
	}
	height := boundedSub(pp.z1Scalar(), pp.z0Scalar())
	h := height.value
	if h <= 0 {
		return nil, fmt.Errorf(`%w: the sweep interval is empty`, ErrDegenerate)
	}

	body := &Body{doc: d, origin: FeatureRef{Step: ref, Role: roleBody}, solid: true}

	// Topology: one shell over every loop's side faces plus the two caps.
	var faces []*Face
	// The start cap faces −N (its outward normal leaves the material at the
	// bottom), so its frame swaps the in-plane axes; the end cap keeps them.
	startFrame, err := capFrame(pp, pp.z0, true)
	if err != nil {
		return nil, err
	}
	endFrame, err := capFrame(pp, pp.z1, false)
	if err != nil {
		return nil, err
	}
	capStart := &Face{
		surface:       Plane{Frame: startFrame},
		origins:       []FeatureRef{{Step: ref, Role: roleCapStart}},
		body:          body,
		area:          ig.area,
		areaBound:     ig.areaBound,
		axialDelta:    pp.z0Delta,
		hasAxialDelta: true,
	}
	capEnd := &Face{
		surface:       Plane{Frame: endFrame},
		origins:       []FeatureRef{{Step: ref, Role: roleCapEnd}},
		body:          body,
		area:          ig.area,
		areaBound:     ig.areaBound,
		axialDelta:    pp.z1Delta,
		hasAxialDelta: true,
	}

	perimeter := boundedScalar{}
	walks := 0
	loops := append([]LoopRecord{pp.profile.Outer}, pp.profile.Holes...)
	for _, loop := range loops {
		walks += len(loop.Segments)
	}
	// pw resolves every boundary segment's walk exactly ONCE for this whole
	// build (segment_walk.go's profileWalks doc comment): buildLoopSides below,
	// prismCentroidGeometryBound and prismBoundsContext's three per-axis
	// extentBoundedAlong calls all read it back instead of each calling
	// walkOf itself, which is what let one free-form segment's §5.2 charge be
	// spent eight times over in a single evalPrismContext call.
	//
	// A payload that already carries the resolution of THIS record hands it
	// straight over — a rigid re-evaluation (placed) is the case, since a
	// placement changes no plane-local walk — and this call is charged what
	// that resolution cost rather than being handed it free. reusable is what
	// decides, and it accepts only a set resolved from a bit-identical record
	// that measured its own charge; anything else resolves here as before.
	pw := pp.walks
	if pw.reusable(pp.profile) {
		if err := pw.charge(work); err != nil {
			return nil, err
		}
	} else {
		resolved, err := resolveProfileWalks(pp.profile, work)
		if err != nil {
			return nil, err
		}
		pw = resolved
	}
	for li, loop := range loops {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		sideFaces, bottom, top, loopLen, err := buildLoopSides(ctx, body, ref, pp, li, loop, work, pw)
		if err != nil {
			return nil, err
		}
		faces = append(faces, sideFaces...)
		perimeter = boundedAdd(perimeter, loopLen)
		capStart.loops = append(capStart.loops, &Loop{coedges: bottom, outer: li == 0})
		capEnd.loops = append(capEnd.loops, &Loop{coedges: top, outer: li == 0})
	}
	faces = append(faces, capStart, capEnd)
	if err := attachFaceLoopsContext(ctx, []*Face{capStart, capEnd}); err != nil {
		return nil, err
	}

	shell := &Shell{faces: faces}
	body.lumps = []*Lump{{shells: []*Shell{shell}}}

	// Measurements carry the closed forms' proven float bounds
	// (docs/evaluator-design.md §5), plus — where the payload carries one — the
	// section's own displacement from the section it denotes
	// (docs/prism-boolean-design.md §7). The area a displaced boundary moves is
	// bounds.go's sectionDisplacementArea, charged once into the region area, so
	// the volume takes it through the height and each cap takes it once; the
	// walls take it through the perimeter, which every walk already charged its
	// own length displacement into (buildLoopSidesAs).
	delta := pp.sectionDelta
	regionArea := measuredScalar(ig.area, absSumUpper(
		ig.areaBound,
		sectionDisplacementArea(delta, walks, absSumUpper(perimeter.value, perimeter.bound)),
	))
	capStart.areaBound = regionArea.bound
	capEnd.areaBound = regionArea.bound
	volume := boundedMul(regionArea, height)
	caps := boundedMul(exactScalar(2), regionArea)
	sides := boundedMul(perimeter, height)
	area := boundedAdd(caps, sides)
	body.volume = Measurement{
		Value:     units.CubicMillimeters(volume.value),
		Exactness: exactnessOf(volume.bound),
		Bound:     units.CubicMillimeters(volume.bound),
	}
	body.area = Measurement{
		Value:     units.SquareMillimeters(area.value),
		Exactness: exactnessOf(area.bound),
		Bound:     units.SquareMillimeters(area.bound),
	}
	cu := boundedQuotient(ig.mu, ig.muBound, ig.area, ig.areaBound)
	cv := boundedQuotient(ig.mv, ig.mvBound, ig.area, ig.areaBound)
	zc := boundedDiv(boundedAdd(pp.z0Scalar(), pp.z1Scalar()), exactScalar(2))
	centroidValue := pp.point(cu.value, cv.value, zc.value)
	// A displaced section moves its own centroid, so the displacement enters the
	// plane-local source term prismPointBound already carries through the frame
	// and placement (docs/prism-boolean-design.md §7). The geometry envelope is a
	// separate proof — the centroid lies inside the prism — so it takes the same
	// displacement rather than being allowed to undercut the formula bound with a
	// figure proven only for the recorded section.
	cu.bound = absSumUpper(cu.bound, delta)
	cv.bound = absSumUpper(cv.bound, delta)
	centroidBound := prismPointBound(pp, cu, cv, zc)
	geometryBound, err := prismCentroidGeometryBound(pp, pp.profile, centroidValue, work, pw)
	if err != nil {
		return nil, err
	}
	centroidBound = math.Min(centroidBound, absSumUpper(geometryBound, delta))
	body.centroid = VecMeasurement{
		Value:     centroidValue,
		Exactness: exactnessOf(centroidBound),
		Bound:     units.Millimeters(centroidBound),
	}
	bounds, err := prismBoundsContext(ctx, pp, work, pw)
	if err != nil {
		return nil, err
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	body.bounds = bounds
	if err := validateAnalyticBodyMeasurements(body); err != nil {
		return nil, err
	}
	// A prism-derived modify body re-mints its blend roles from its own record
	// under this build's ref, so a copy or placement reproduces them (modify §9).
	// A plain extrude carries no descriptors and this is a no-op.
	if err := addBlendRoles(ctx, body, ref, pp.blendSegs, pp.blendKind); err != nil {
		return nil, err
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	// The build succeeded, so pw is a resolution of a record this body now owns
	// and every walk in it held. Publishing it onto the payload is what lets a
	// later rigid re-evaluation read it back; the record itself is unchanged, so
	// the payload the body carries denotes exactly what it did before.
	pp.walks = pw
	body.payload = pp
	return body, nil
}

// attachFaceLoopsContext links each coedge's shared edge to its face. The
// assembly is private until the caller commits the body, but it can be large
// for a recorded region with many loops, so every nesting level polls ctx.
func attachFaceLoopsContext(ctx context.Context, faces []*Face) error {
	for _, f := range faces {
		if err := ctx.Err(); err != nil {
			return err
		}
		for _, l := range f.loops {
			if err := ctx.Err(); err != nil {
				return err
			}
			for _, ce := range l.coedges {
				if err := ctx.Err(); err != nil {
					return err
				}
				ce.edge.faces = append(ce.edge.faces, f)
			}
		}
	}
	return nil
}

// capFrame is the plane frame of a cap at height z, under the placement;
// flip swaps the in-plane axes so the frame's normal is the cap's OUTWARD
// normal, and a reflected placement swaps once more (a reflection flips the
// cross product's handedness). The payload frame is orthonormal and the
// placement rigid, so the transformed axes cannot be degenerate; NewFrame
// re-validates anyway and the error propagates rather than being discarded.
func capFrame(pp prismPayload, z float64, flip bool) (r3.Frame, error) {
	if pp.reflected() {
		flip = !flip
	}
	u, v := pp.dir(1, 0, 0), pp.dir(0, 1, 0)
	if flip {
		u, v = v, u
	}
	f, err := r3.NewFrame(pp.point(0, 0, z), u, v)
	if err != nil {
		return r3.Frame{}, fmt.Errorf(`%w: the placed cap frame is degenerate: %s`, ErrDegenerate, err)
	}
	return f, nil
}

// freeformVertexAllow folds a junction vertex's bound with a FREE-FORM walk's
// own endpoint bound (bounds.go's walkEndBoundAllow), and answers zero for
// every other kind. A vertex the payload recorded is exact only where every
// coordinate feeding it is; a free-form walk's endpoint is the converted
// chain's own control point (§5.1), read into float64 by the one rounding
// walkEndBoundAllow measures, so a vertex it touches must carry that rounding
// too (topology.go's Vertex.Position contract). An analytic walk's own
// endpoint bound is a separate question buildLoopSidesAs does not answer here.
func freeformVertexAllow(w segmentWalk, bound walkEndBound) float64 {
	if w.kind != walkFreeform {
		return 0
	}
	return walkEndBoundAllow(bound)
}

// buildLoopSides builds one loop's side faces with shared vertices and
// edges, returning the faces, the bottom and top cap coedges in walk order,
// and the loop's perimeter length. A loop's index is both its role index and,
// via li != 0, its orientation: loop 0 is an outer loop (material inside),
// every other a hole (material outside).
//
// resolved is pp.profile's pre-resolved segment walks, or nil to resolve each
// segment through walkOf as before (this file's profileWalks doc comment).
// buildLoopSides's own li IS resolved's loop index here — li walks the same
// append([]LoopRecord{pp.profile.Outer}, pp.profile.Holes...) order a
// *profileWalks was resolved from — so it is passed straight through as
// buildLoopSidesAs's roleLoop, which resolved is read against.
func buildLoopSides(ctx context.Context, body *Body, ref StepRef, pp prismPayload, li int, loop LoopRecord, work *freeformWork, resolved *profileWalks) ([]*Face, []coedge, []coedge, boundedScalar, error) {
	return buildLoopSidesAs(ctx, body, ref, pp, li, li != 0, loop, work, resolved)
}

// buildLoopSidesAs is buildLoopSides with the role index and the orientation
// decoupled: roleLoop names the walls' side(roleLoop,j) role, and holeLoop
// picks the material side of a straight wall independently. A cup's cavity
// walks each loop of the void the SOLID wraps — the void's outer boundary is a
// hole in the solid (holeLoop true), each of the void's own holes a solid post
// (holeLoop false) — a pairing the natural li != 0 rule cannot express
// (docs/modify-design.md §9).
//
// resolved is a *profileWalks whose loop index roleLoop holds this loop's
// pre-resolved walks, or nil to resolve each segment through walkOf as
// before. A non-nil resolved whose loop at roleLoop was not resolved from
// exactly this loop's recorded segments (loopMatches — the segments
// themselves, not their count) is a plumbing bug and refuses rather than
// silently resolving anyway — the only caller that ever passes non-nil is
// buildLoopSides from evalPrismContext, where roleLoop already IS the loop
// index the *profileWalks was resolved at.
func buildLoopSidesAs(ctx context.Context, body *Body, ref StepRef, pp prismPayload, roleLoop int, holeLoop bool, loop LoopRecord, work *freeformWork, resolved *profileWalks) ([]*Face, []coedge, []coedge, boundedScalar, error) {
	if err := ctx.Err(); err != nil {
		return nil, nil, nil, boundedScalar{}, err
	}
	if len(loop.Segments) == 0 {
		return nil, nil, nil, boundedScalar{}, fmt.Errorf(`%w: a recorded loop holds no segments`, ErrDegenerate)
	}
	var loopWalks []segmentWalk
	if resolved != nil {
		if !resolved.loopMatches(roleLoop, loop) {
			return nil, nil, nil, boundedScalar{}, errResolvedWalksMismatch
		}
		loopWalks = resolved.loopWalks(roleLoop)
	}
	// Every coordinate this loop's walks read sits within the payload's own
	// section displacement of the section it denotes, so each walk's length, each
	// junction vertex and each side face carries that displacement too
	// (docs/prism-boolean-design.md §7). It is zero for every payload a caller
	// draws, and the arithmetic below is then the unchanged one.
	delta := pp.sectionDelta
	walkLenAllow := sectionDisplacementLength(delta, 1)
	raw := make([]sideWalk, len(loop.Segments))
	total := boundedScalar{}
	for i, seg := range loop.Segments {
		if err := ctx.Err(); err != nil {
			return nil, nil, nil, boundedScalar{}, err
		}
		seg, err := normalizeSegment(seg)
		if err != nil {
			return nil, nil, nil, boundedScalar{}, err
		}
		// A Tier B or Tier C free-form kind (a conic, a whole ellipse, an
		// unequal-weight NURBS, or an elliptical arc) refuses inside walkOf
		// itself (freeformBezierSpans, Table R R2/R10) — this build stages no
		// gate of its own ahead of it any more (§10 P4b retires R6). A
		// resolved walk was already through walkOf once (resolveProfileWalks),
		// so it carries the same refusal already surfaced there.
		var w segmentWalk
		if loopWalks != nil {
			w = loopWalks[i]
		} else {
			w, err = walkOf(seg, work)
			if err != nil {
				return nil, nil, nil, boundedScalar{}, err
			}
		}
		w.lengthBound = absSumUpper(w.lengthBound, walkLenAllow)
		raw[i] = sideWalk{segmentWalk: w, segs: []int{i}}
		total = boundedAdd(total, measuredScalar(w.length, w.lengthBound))
	}
	walks, err := coalesceWalksContext(ctx, raw)
	if err != nil {
		return nil, nil, nil, boundedScalar{}, err
	}
	n := len(walks)

	// Junction vertices, shared between neighbors: junction i sits at walk
	// i's start (== walk i−1's end). A single whole closed curve has none.
	singleClosed := n == 1 && walks[0].closed
	// The sweep height is read from the two BOUNDED levels, so a level the
	// evaluator computed carries its own displacement into the vertical edge
	// lengths and the side face areas built from it below — a ToFace or
	// ThroughAll stop's float arithmetic (stops.go), a non-base unit's rescale,
	// and a chamfered end's setback alike.
	height := boundedSub(pp.z1Scalar(), pp.z0Scalar())
	// A side vertex sits at one recorded boundary coordinate and one sweep level,
	// so it carries both displacements: the section's, which moves it in the
	// plane, and its own end's, which moves it along the normal. Each is zero for
	// a coordinate the payload recorded from what the caller stated, which is what
	// keeps an ordinary extrude's vertices Exact; neither is a claim about the
	// other's axis, so they compose rather than one standing in for the other.
	// A junction touching a FREE-FORM walk's own end also folds in that walk's
	// own endpoint bound (freeformVertexAllow, bounds.go's walkEndBoundAllow) —
	// the one rounding §5.1's exact-rational Bézier conversion committed taking
	// the endpoint into float64 (topology.go's Vertex.Position contract: a
	// COMPUTED coordinate carries its own computation's proven displacement).
	// An analytic walk contributes nothing here: widening a trimmed circular
	// walk's vertex is a separate question this build does not answer by
	// accident.
	bottomBoundBase := absSumUpper(delta, pp.z0Delta)
	topBoundBase := absSumUpper(delta, pp.z1Delta)
	var bottomV, topV []*Vertex
	// seamBottom/seamTop are the SINGLE seam vertex a lone closed walk's rim
	// edges share at each cap — one per cap, no junction vertex at all — the
	// free-form twin of a whole CircleSeg's own seam vertex. Hoisted out of the
	// per-kind switch below (§10 P4b): a closed free-form walk reaches
	// singleClosed on the same terms a whole circle does, and the switch's
	// walkFreeform arm needs the SAME seam pair the walkCircular arm builds.
	var seamBottom, seamTop *Vertex
	if singleClosed {
		w := walks[0]
		extra := math.Max(freeformVertexAllow(w.segmentWalk, w.startBound), freeformVertexAllow(w.segmentWalk, w.endBound))
		seamBottom = &Vertex{position: pp.point(w.startU, w.startV, pp.z0), bound: units.Millimeters(absSumUpper(bottomBoundBase, extra))}
		seamTop = &Vertex{position: pp.point(w.startU, w.startV, pp.z1), bound: units.Millimeters(absSumUpper(topBoundBase, extra))}
	} else {
		bottomV = make([]*Vertex, n)
		topV = make([]*Vertex, n)
		for i, w := range walks {
			if err := ctx.Err(); err != nil {
				return nil, nil, nil, boundedScalar{}, err
			}
			prev := walks[(i+n-1)%n]
			extra := math.Max(freeformVertexAllow(w.segmentWalk, w.startBound), freeformVertexAllow(prev.segmentWalk, prev.endBound))
			bottomV[i] = &Vertex{position: pp.point(w.startU, w.startV, pp.z0), bound: units.Millimeters(absSumUpper(bottomBoundBase, extra))}
			topV[i] = &Vertex{position: pp.point(w.startU, w.startV, pp.z1), bound: units.Millimeters(absSumUpper(topBoundBase, extra))}
		}
	}

	// Vertical edges at junctions, shared between side face i−1 and i.
	// Convexity from the 2D turn: a positive cross of the incoming and
	// outgoing tangents is a left turn — interior angle < π — which is a
	// convex edge on the outer loop and works out identically for hole
	// loops walked clockwise. A free-form walk's end tangent is the
	// hodograph's own exact-rational leg (§5.1), rounded once into float64
	// with its own stated bound (tanInBound/tanOutBound) — at least as well
	// founded as the circular walk's own tangent, whose bound is +Inf by
	// segmentWalk's convention because it comes from trig at a computed
	// angle. This cross needs no change for either kind.
	var vertical []*Edge
	if !singleClosed {
		vertical = make([]*Edge, n)
		for i := range walks {
			if err := ctx.Err(); err != nil {
				return nil, nil, nil, boundedScalar{}, err
			}
			prev := walks[(i+n-1)%n]
			cross := prev.tanOutU*walks[i].tanInV - prev.tanOutV*walks[i].tanInU
			vertical[i] = &Edge{
				curve:       Line3{},
				start:       bottomV[i],
				end:         topV[i],
				convex:      cross > 0,
				length:      pp.z1 - pp.z0,
				lengthBound: height.bound,
			}
		}
	}

	// Side faces with bottom/top edges; cap coedges accumulate in walk order.
	faces := make([]*Face, 0, n)
	bottomCo := make([]coedge, 0, n)
	topCo := make([]coedge, 0, n)
	for i, w := range walks {
		if err := ctx.Err(); err != nil {
			return nil, nil, nil, boundedScalar{}, err
		}
		var bStart, bEnd, tStart, tEnd *Vertex
		if singleClosed {
			bStart, bEnd = seamBottom, seamBottom
			tStart, tEnd = seamTop, seamTop
		} else {
			bStart, tStart = bottomV[i], topV[i]
			bEnd, tEnd = bottomV[(i+1)%n], topV[(i+1)%n]
		}
		var bottomEdge, topEdge *Edge
		var surf Surface
		faceReversed := false
		switch w.kind {
		case walkCircular:
			axis := pp.dir(0, 0, 1)
			// The material's side of a circular wall is decided by the WALK,
			// not by the loop's role: the outward normal is the walk tangent
			// turned a quarter turn against the walk's sense, which is the
			// radial direction away from the centre for a counter-clockwise
			// walk and toward the centre for a clockwise one. A hole is
			// walked clockwise — which is why its wall reverses — but so is a
			// CONCAVE round on an outer loop (a rounded bite out of the
			// boundary), whose material also lies outside the cylinder.
			clockwise := w.th1 < w.th0
			// An Arc3/Circle3 is CCW from start to end about its axis. A
			// clockwise walk, and a reflected placement (which flips
			// handedness), each invert that sense, so the EDGE axis carries
			// the corrected sign; the cylinder surface keeps the plain ruling
			// direction.
			edgeSign := 1.0
			if clockwise {
				edgeSign = -1
			}
			if pp.reflected() {
				edgeSign = -edgeSign
			}
			edgeAxis := axis.Scale(edgeSign)
			center0 := pp.point(w.cU, w.cV, pp.z0)
			center1 := pp.point(w.cU, w.cV, pp.z1)
			radius := units.Millimeters(w.radius)
			var curve0, curve1 Curve
			if singleClosed {
				// A full circle's edge closes on itself: one vertex per cap
				// edge, start == end (topology.go's Circle3 contract) — the
				// seamBottom/seamTop pair the switch's caller already built.
				curve0, curve1 = Circle3{Center: center0, Axis: edgeAxis, Radius: radius}, Circle3{Center: center1, Axis: edgeAxis, Radius: radius}
			} else {
				curve0, curve1 = Arc3{Center: center0, Axis: edgeAxis, Radius: radius}, Arc3{Center: center1, Axis: edgeAxis, Radius: radius}
			}
			// The rim edges read the same WALK: a clockwise wall's material
			// lies outside its cylinder, so the boundary turns into the metal
			// there and the rim is concave — a hole's rim (clockwise) and a
			// concave outer bite's (also clockwise) alike, while a
			// counter-clockwise round keeps its convex rim. The loop's role
			// decides nothing here.
			capConvex := !clockwise
			bottomEdge = &Edge{curve: curve0, start: bStart, end: bEnd, convex: capConvex, length: w.length, lengthBound: w.lengthBound}
			topEdge = &Edge{curve: curve1, start: tStart, end: tEnd, convex: capConvex, length: w.length, lengthBound: w.lengthBound}
			surf = Cylinder{Origin: center0, Axis: axis, Radius: radius}
			// A clockwise-walked wall has its material OUTSIDE the cylinder,
			// so its outward normal is the radial direction negated.
			faceReversed = clockwise
		case walkFreeform:
			// §6.5's wall-edge convexity certificate, wired here for the first
			// time (docs/spline-design.md §6.5, Table R R19). It applies the
			// one reversal negation internally and states its verdict in the
			// LOOP'S OWN WALK direction, so the mapping below reads it
			// verbatim — a counter-clockwise turn convex, clockwise concave,
			// the identical convention the circular wall's own turn test
			// fixes above. NEVER negate again for a hole loop: a hole rim's
			// concavity falls out of the clockwise walk itself, exactly as it
			// does for a circular hole wall. An error is R19: propagate it so
			// the build refuses and no step commits.
			verdict, err := freeformWallConvexityContext(ctx, w.spans, w.closed, w.reversed, w.fitInterpolated, work)
			if err != nil {
				return nil, nil, nil, boundedScalar{}, err
			}
			var convex bool
			switch verdict {
			case freeformConvexityPositive:
				convex = true
			case freeformConvexityNegative:
				convex = false
			case freeformConvexityStraight:
				// Every live span lies on one line and no joint turns off it
				// (§6.5's Table K): the chain has no turn of its own, so
				// evaluator §3's straight-wall rule decides it by its loop's
				// role — the same expression the walkLine arm below uses.
				convex = !holeLoop
			}
			bottomEdge = &Edge{curve: NURBSCurve{}, start: bStart, end: bEnd, convex: convex, length: w.length, lengthBound: w.lengthBound}
			topEdge = &Edge{curve: NURBSCurve{}, start: tStart, end: tEnd, convex: convex, length: w.length, lengthBound: w.lengthBound}
			surf = NURBSSurface{}
			// faceReversed stays false: an opaque NURBSSurface publishes no
			// normal at all (§7's NormalAt refusal, topology.go), so
			// Face.reversed — "the outward normal is the surface's geometric
			// normal negated" — names nothing for this variant.
		default:
			// A straight wall has no turn of its own to disagree with the
			// loop's: which side its material lies on is decided by the sense
			// the whole loop is walked, and that sense IS the loop's role —
			// the outer loop counter-clockwise, holes clockwise (moments.go).
			bottomEdge = &Edge{curve: Line3{}, start: bStart, end: bEnd, convex: !holeLoop, length: w.length, lengthBound: w.lengthBound}
			topEdge = &Edge{curve: Line3{}, start: tStart, end: tEnd, convex: !holeLoop, length: w.length, lengthBound: w.lengthBound}
			mid := pp.point((w.startU+w.endU)/2, (w.startV+w.endV)/2, pp.z0)
			// tangent × N is the outward normal for a CCW outer walk (and a
			// CW hole walk); a reflection flips the cross product, so the
			// tangent is negated to keep the frame's normal outward.
			tu, tv := w.tanInU, w.tanInV
			if pp.reflected() {
				tu, tv = -tu, -tv
			}
			f, err := r3.NewFrame(mid, pp.dir(tu, tv, 0), pp.dir(0, 0, 1))
			if err != nil {
				return nil, nil, nil, boundedScalar{}, fmt.Errorf(`%w: a boundary segment has no direction`, ErrDegenerate)
			}
			surf = Plane{Frame: f}
		}

		origins, err := sideOriginsContext(ctx, ref, roleLoop, w.segs)
		if err != nil {
			return nil, nil, nil, boundedScalar{}, err
		}
		faceArea := boundedMul(measuredScalar(w.length, w.lengthBound), height)
		face := &Face{
			surface:   surf,
			origins:   origins,
			body:      body,
			area:      faceArea.value,
			areaBound: faceArea.bound,
			reversed:  faceReversed,
		}
		if singleClosed {
			// A closed cylindrical band has TWO boundary loops — one circle
			// per cap — not one loop holding both.
			face.loops = []*Loop{
				{coedges: []coedge{{edge: bottomEdge, forward: true}}, outer: true},
				{coedges: []coedge{{edge: topEdge, forward: false}}, outer: true},
			}
		} else {
			face.loops = []*Loop{{coedges: []coedge{
				{edge: bottomEdge, forward: true},
				{edge: vertical[(i+1)%n], forward: true},
				{edge: topEdge, forward: false},
				{edge: vertical[i], forward: false},
			}, outer: true}}
		}

		bottomEdge.faces = append(bottomEdge.faces, face)
		topEdge.faces = append(topEdge.faces, face)
		if !singleClosed {
			vertical[i].faces = append(vertical[i].faces, face)
			vertical[(i+1)%n].faces = append(vertical[(i+1)%n].faces, face)
		}

		faces = append(faces, face)
		bottomCo = append(bottomCo, coedge{edge: bottomEdge, forward: true})
		topCo = append(topCo, coedge{edge: topEdge, forward: true})
	}
	return faces, bottomCo, topCo, total, nil
}

func sideOriginsContext(ctx context.Context, ref StepRef, roleLoop int, segs []int) ([]FeatureRef, error) {
	origins := make([]FeatureRef, len(segs))
	for oi, si := range segs {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		origins[oi] = FeatureRef{Step: ref, Role: fmt.Sprintf("side(%d,%d)", roleLoop, si)}
	}
	return origins, nil
}
