package prismrecord

import (
	"context"
	"fmt"
	"math"
	"unsafe"

	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/denotation"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// This file builds a straight extrude's body from its Payload: the two
// cap faces, one side face per recorded segment, and the topology that joins
// them, together with the measurements the finished body publishes.
//
// Build is the whole build and states the order; BuildLoopSidesAs
// is the per-loop walk it runs once per outer loop and once per hole, with
// the hole's orientation reversed. Every face this file stamps carries the
// displacement its own surface was built from. See
// agent-docs/evaluator-design.md §5.

// OuterFirstSideRole names the wall built from the first segment of a
// prism's outer profile. A single-circle extrusion has only this side wall.
const OuterFirstSideRole = "side(0,0)"

// Build evaluates pp into a new body allocated through env.Alloc, minting
// its level and curve identities through env.Doc and its roles under
// env.Ref. It never commits: the caller maps the result to its handle and
// commits it.
func Build(ctx context.Context, env topo.Env, pp Payload, work *freeform.FreeformWork) (*topo.Body, error) {
	var witness *PrismSourceWitness
	if capture, ok := ctx.Value(prismWitnessKey{}).(func(*PrismSourceWitness)); ok && capture != nil {
		witness = newPrismSourceWitness(pp)
		defer func() {
			if witness != nil && witness.Body != nil {
				capture(witness)
			}
		}()
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	// A fit-spline section costs about 70000 units per cubic piece, so the
	// ceiling scales with the piece count whoever opened the counter.
	if work != nil {
		loops := append([]sectionrecord.LoopRecord{pp.Profile.Outer}, pp.Profile.Holes...)
		work.RaiseLimit(freeform.SpanWorkLimit(sectionrecord.FitSplinePieces(loops...)))
	}
	var ig momentinput.Integrals
	var captured *momentinput.ProfileWalks
	var err error
	if pp.Walks == nil {
		ig, captured, err = pp.Profile.EvaluatorIntegralsWithAnalyticWalksContext(ctx, freeform.MomentFirstOrder, work)
	} else {
		ig, err = pp.Walks.EvaluatorIntegralsUncheckedContext(ctx, pp.Profile, freeform.MomentFirstOrder, work)
	}
	if err != nil {
		return nil, err
	}
	if ig.Area <= 0 {
		return nil, fmt.Errorf(`%w: the recorded region encloses no area`, decaderr.ErrDegenerate)
	}
	height := proofbound.BoundedSub(pp.Z1Scalar(), pp.Z0Scalar())
	h := height.Value
	if h <= 0 {
		return nil, fmt.Errorf(`%w: the sweep interval is empty`, decaderr.ErrDegenerate)
	}

	// A surface result publishes a sheet, never a solid: solid false is what
	// makes Volume() and Centroid() answer ErrNotSolid through their existing
	// guards, and kind BodySheet is what Kind() reports
	// (agent-docs/surface-design.md §4.3).
	kind := topo.BodySolid
	if pp.SurfaceResult {
		kind = topo.BodySheet
	}
	body := env.Alloc.NewBody()
	body.Solid = !pp.SurfaceResult
	body.Origin = provenance.Ref{Producer: env.Ref, Role: provenance.RoleBody}
	body.Kind = kind
	body.Doc = env.Doc
	if body.BeginPrismConstruction() {
		defer body.FinishPrismConstruction()
	}

	// The LEVEL half of the shared-denotation certificate (internal/denotation):
	// minted only when this payload's own section is drawn straight from its
	// record — a section the analytic prism boolean re-expresses sets
	// sectionDelta nonzero and mints neither — so every vertex and rim edge
	// this build places at z0/z1 can later prove PatchEdges's bounded-chain
	// gate coplanar by shared identity alone, never by comparing a
	// coordinate (agent-docs/surface-design.md §5.2). The token's own origin/normal
	// are read off the SAME frame-derived construction Payload.CapFrame uses below
	// for the solid's own caps (pp.point/pp.dir), never fitted to held vertex
	// coordinates — the two ends share one normal (a prism does not tilt
	// between them), so it is computed once.
	var levelZ0, levelZ1 denotation.LevelToken
	if pp.SectionDelta == 0 {
		levelNormal := pp.Dir(0, 0, 1)
		levelZ0 = env.Doc.MintLevel(pp.Point(0, 0, pp.Z0), levelNormal)
		levelZ1 = env.Doc.MintLevel(pp.Point(0, 0, pp.Z1), levelNormal)
	}

	// Topology: one shell over every loop's side faces plus the two caps.
	var faces []*topo.Face
	// The start cap faces −N (its outward normal leaves the material at the
	// bottom), so its frame swaps the in-plane axes; the end cap keeps them.
	startFrame, err := pp.CapFrame(pp.Z0, true)
	if err != nil {
		return nil, err
	}
	endFrame, err := pp.CapFrame(pp.Z1, false)
	if err != nil {
		return nil, err
	}
	capStart := topobuild.NewFace(env.Alloc, topo.Face{
		Area:          ig.Area,
		AreaBound:     ig.AreaBound,
		AxialDelta:    pp.Z0Delta,
		HasAxialDelta: true,
		Surface:       surfacegeom.Plane{Frame: startFrame},
		CapRecord:     &topo.CapRecord{Frame: pp.Frame, Xform: pp.Xform, Z: pp.Z0, ZDenoted: pp.DenotedZ0()},
		Origins:       []provenance.Ref{{Producer: env.Ref, Role: provenance.RoleCapStart}},
		Body:          body,
	})
	capEnd := topobuild.NewFace(env.Alloc, topo.Face{
		Area:          ig.Area,
		AreaBound:     ig.AreaBound,
		AxialDelta:    pp.Z1Delta,
		HasAxialDelta: true,
		Surface:       surfacegeom.Plane{Frame: endFrame},
		CapRecord:     &topo.CapRecord{Frame: pp.Frame, Xform: pp.Xform, Z: pp.Z1, ZDenoted: pp.DenotedZ1()},
		Origins:       []provenance.Ref{{Producer: env.Ref, Role: provenance.RoleCapEnd}},
		Body:          body,
	})

	perimeter := proofbound.BoundedScalar{}
	walks := 0
	loops := append([]sectionrecord.LoopRecord{pp.Profile.Outer}, pp.Profile.Holes...)
	for _, loop := range loops {
		walks += len(loop.Segments)
	}
	// pw resolves every boundary segment's walk exactly ONCE for this whole
	// build (internal/momentinput/profile_walks.go): BuildLoopSides below,
	// prismrecord.Payload.CentroidGeometryBound and Bounds's three per-axis
	// ExtentBoundedAlong calls all read it back instead of each calling
	// walkOf itself, which is what let one free-form segment's §5.2 charge be
	// spent eight times over in a single Build call.
	//
	// A payload that already carries the resolution of THIS record hands it
	// straight over — a rigid re-evaluation (placed) is the case, since a
	// placement changes no plane-local walk — and this call is charged what
	// that resolution cost rather than being handed it free. reusable is what
	// decides, and it accepts only a set resolved from a bit-identical record
	// that measured its own charge; anything else resolves here as before.
	pw := pp.Walks
	if pw.Reusable(pp.Profile) {
		if err := pw.Charge(work); err != nil {
			return nil, err
		}
	} else if captured.Reusable(pp.Profile) {
		// Exact record identity preserves normalized-type and unit mismatches
		// as fallbacks. The preflight measured zero charges, so adopting its
		// walks adds no Step call at this formerly uncharged resolution.
		captured.Profile = pp.Profile
		pw = captured
	} else {
		resolved, err := momentinput.ResolveProfileWalks(pp.Profile, work)
		if err != nil {
			return nil, err
		}
		pw = resolved
	}
	for li, loop := range loops {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		sideFaces, bottom, top, loopLen, err := buildLoopSidesAs(ctx, env, body, pp, li, li != 0,
			loop, work, pw, levelZ0, levelZ1, true, true, nil, witness)
		if err != nil {
			return nil, err
		}
		faces = append(faces, sideFaces...)
		perimeter = proofbound.BoundedAdd(perimeter, loopLen)
		capStart.Loops = append(capStart.Loops, topobuild.NewLoop(env.Alloc, topo.Loop{Outer: li == 0, CoEdges: bottom}))
		capEnd.Loops = append(capEnd.Loops, topobuild.NewLoop(env.Alloc, topo.Loop{Outer: li == 0, CoEdges: top}))
	}
	// A surface result omits both caps from the shell (Table W,
	// agent-docs/surface-design.md §4.1-§4.2): capStart/capEnd stay constructed above
	// so the area accumulation below can still read their area fields, but
	// neither joins faces nor gets its loops attached, which is what leaves
	// each rim edge with only its wall face — both rims of every loop become
	// free edges with no rim-specific code.
	if !pp.SurfaceResult {
		capStart.RecordPrismFace(topo.PrismFaceCapStart, -1, -1)
		capEnd.RecordPrismFace(topo.PrismFaceCapEnd, -1, -1)
		faces = append(faces, capStart, capEnd)
		if err := topobuild.AttachFaceLoops(ctx, []*topo.Face{capStart, capEnd}); err != nil {
			return nil, err
		}
	}

	// topobuild.SheetLumps derives IsOpen from the faces' own edge adjacency rather than
	// the option that built them, and splits the face set by connectivity
	// rather than assuming one shell holds the whole body regardless of it
	// (agent-docs/surface-design.md §2.2). A solid's faces are always one connected
	// piece — its caps join every loop's wall tube to every other's — so this
	// reproduces the previous single-shell, single-lump body for a solid
	// unchanged; a surface result's holed profile, whose loops touch nowhere
	// once their caps are omitted, is what actually needs more than one.
	body.Lumps = topobuild.SheetLumps(env.Alloc, faces)

	// Measurements carry the closed forms' proven float bounds
	// (agent-docs/evaluator-design.md §5), plus — where the payload carries one — the
	// section's own displacement from the section it denotes
	// (agent-docs/prism-boolean-design.md §7). The area a displaced boundary moves is
	// internal/proofbound/bounds.go's proofbound.SectionDisplacementArea, charged once into the region area, so
	// the volume takes it through the height and each cap takes it once; the
	// walls take it through the perimeter, which every walk already charged its
	// own length displacement into (BuildLoopSidesAs).
	delta := pp.SectionDelta
	regionArea := proofbound.MeasuredScalar(ig.Area, proofbound.AbsSumUpper(
		ig.AreaBound,
		proofbound.SectionDisplacementArea(delta, walks, proofbound.AbsSumUpper(perimeter.Value, perimeter.Bound)),
	))
	capStart.AreaBound = regionArea.Bound
	capEnd.AreaBound = regionArea.Bound
	volume := proofbound.BoundedMul(regionArea, height)
	caps := proofbound.BoundedMul(proofbound.ExactScalar(2), regionArea)
	sides := proofbound.BoundedMul(perimeter, height)
	area := proofbound.BoundedAdd(caps, sides)
	if pp.SurfaceResult {
		// §4.3: a surface result's area is the solid's area minus the two
		// omitted caps'. Each cap's area and bound are read back from the Face
		// fields rather than assumed to equal regionArea's, and the
		// subtraction runs here — after capStart/capEnd.areaBound were
		// re-stamped from regionArea.bound just above — so it composes the
		// POST-displacement bound rather than the one the caps carried before
		// that re-stamp.
		area = proofbound.BoundedSub(area, proofbound.MeasuredScalar(capStart.Area, capStart.AreaBound))
		area = proofbound.BoundedSub(area, proofbound.MeasuredScalar(capEnd.Area, capEnd.AreaBound))
	}
	// volume and centroid are always computed and stored — a sheet needs them
	// finite for topobuild.ValidateAnalyticMeasurements below — but Body.Volume and
	// Body.Centroid gate on solid and answer ErrNotSolid for a sheet, so these
	// two stay held and unpublished then (agent-docs/surface-design.md §8).
	body.Volume = measurement.Measurement{
		Value:     units.CubicMillimeters(volume.Value),
		Exactness: measurement.ExactnessOf(volume.Bound),
		Bound:     units.CubicMillimeters(volume.Bound),
	}
	body.Area = measurement.Measurement{
		Value:     units.SquareMillimeters(area.Value),
		Exactness: measurement.ExactnessOf(area.Bound),
		Bound:     units.SquareMillimeters(area.Bound),
	}
	cu := proofbound.BoundedQuotient(ig.Mu, ig.MuBound, ig.Area, ig.AreaBound)
	cv := proofbound.BoundedQuotient(ig.Mv, ig.MvBound, ig.Area, ig.AreaBound)
	zc := proofbound.BoundedDiv(proofbound.BoundedAdd(pp.Z0Scalar(), pp.Z1Scalar()), proofbound.ExactScalar(2))
	centroidValue := pp.Point(cu.Value, cv.Value, zc.Value)
	// A displaced section moves its own centroid, so the displacement enters the
	// plane-local source term prismrecord.Payload.PointBound already carries through the frame
	// and placement (agent-docs/prism-boolean-design.md §7). The geometry envelope is a
	// separate proof — the centroid lies inside the prism — so it takes the same
	// displacement rather than being allowed to undercut the formula bound with a
	// figure proven only for the recorded section.
	cu.Bound = proofbound.AbsSumUpper(cu.Bound, delta)
	cv.Bound = proofbound.AbsSumUpper(cv.Bound, delta)
	centroidBound := pp.PointBound(cu, cv, zc)
	geometryBound, err := pp.CentroidGeometryBound(pp.Profile, centroidValue, work, pw)
	if err != nil {
		return nil, err
	}
	centroidBound = math.Min(centroidBound, proofbound.AbsSumUpper(geometryBound, delta))
	body.Centroid = measurement.VecMeasurement{
		Value:     centroidValue,
		Exactness: measurement.ExactnessOf(centroidBound),
		Bound:     units.Millimeters(centroidBound),
	}
	bounds, err := Bounds(ctx, pp, work, pw)
	if err != nil {
		return nil, err
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	body.Bounds = bounds
	if err := ChargeMap(body, pp.Frame, pp.Xform); err != nil {
		return nil, err
	}
	if err := topobuild.ValidateAnalyticMeasurements(body); err != nil {
		return nil, err
	}
	// A prism-derived modify body re-mints its blend roles from its own record
	// under this build's ref, so a copy or placement reproduces them (modify §9).
	// A plain extrude carries no descriptors and this is a no-op.
	if err := topobuild.AddBlendRoles(ctx, body, env.Ref, pp.BlendSegs, pp.BlendKind); err != nil {
		return nil, err
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	// The build succeeded, so pw is a resolution of a record this body now owns
	// and every walk in it held. Publishing it onto the payload is what lets a
	// later rigid re-evaluation read it back; the record itself is unchanged, so
	// the payload the body carries denotes exactly what it did before.
	if pw != pp.Walks {
		pw = pw.WithIntegrals(ig, freeform.MomentFirstOrder)
	}
	if !pp.SurfaceResult && len(pp.Profile.Holes) == 0 && len(pp.Profile.Outer.Segments) == 1 {
		if _, ok := pp.Profile.Outer.Segments[0].(sectionrecord.CircleSeg); ok {
			for _, f := range faces {
				f.SealPrismFace()
			}
		}
	}
	pp.Walks = pw
	pp.Upstream = nil
	witnessBudget := proofbound.NewReplayTrialBudget()
	if sealed, _ := body.SealPrismBuildFacesReplay(ctx, faces, witnessBudget); sealed {
		if sealed, _ := body.SealPrismBuildPayloadReplay(ctx, pp.Frame, pp.Xform,
			pp.Z0, pp.Z1, pp.Z0Delta, pp.Z1Delta, pp.SectionDelta,
			pp.Z0Denoted, pp.Z1Denoted,
			[]momentinput.Profile{pp.Profile}, witnessBudget); sealed {
			// This optional seal only enables a later bounded replay; ordinary
			// construction remains valid if its witness cannot be recorded.
			_, _ = body.SealPrismBuildWalksReplay(ctx, pw, witnessBudget)
		}
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	body.Payload = pp
	if witness != nil && witness.valid && len(witness.Walls) == len(pp.Profile.Outer.Segments) {
		witness.Body, witness.CapStart, witness.CapEnd = body, capStart, capEnd
	}
	return body, nil
}

// BuildWallGeometry builds one wall's two rim curves and the surface it
// sweeps, from its walk alone, allocating both edges through alloc: convex is boundarywalk.RimConvexity's own answer for this
// walk, and closed marks a single whole closed walk sharing one seam vertex
// at each cap (bStart==bEnd, tStart==tEnd) — never true for a chain, since
// RecordChain admits no whole closed segment
// (agent-docs/sketch-seam-design.md §2.2). It is the one per-kind construction a
// closed loop's own wall (BuildLoopSidesAs) and an open chain's own wall
// (chainrecord's buildSides) share; the two differ only in the TOPOLOGY
// around it — wraparound junctions and cap coedges for a loop, neither for a
// chain (agent-docs/surface-design.md §13.4).
func BuildWallGeometry(alloc topo.Alloc, pp Payload, w survey2d.SideWalk, convex, closed bool, bStart, bEnd, tStart, tEnd *topo.Vertex) (*topo.Edge, *topo.Edge, surfacegeom.Surface, bool, error) {
	return buildWallGeometry(alloc, pp, w, convex, closed, bStart, bEnd, tStart, tEnd, nil, nil)
}

// BuildWallGeometryReplay uses the same topology construction with a charged
// signed-basis circular bound. The caller charges its node allocations.
func BuildWallGeometryReplay(ctx context.Context, alloc topo.Alloc, pp Payload, w survey2d.SideWalk,
	convex, closed bool, bStart, bEnd, tStart, tEnd *topo.Vertex, budget *proofbound.ReplayBudget,
) (*topo.Edge, *topo.Edge, surfacegeom.Surface, bool, error) {
	if budget == nil {
		return nil, nil, nil, false, decaderr.ErrUnsupported
	}
	return buildWallGeometry(alloc, pp, w, convex, closed, bStart, bEnd, tStart, tEnd, ctx, budget)
}

func buildWallGeometry(alloc topo.Alloc, pp Payload, w survey2d.SideWalk, convex, closed bool,
	bStart, bEnd, tStart, tEnd *topo.Vertex, ctx context.Context, budget *proofbound.ReplayBudget,
) (*topo.Edge, *topo.Edge, surfacegeom.Surface, bool, error) {
	var err error
	switch w.Kind {
	case survey2d.WalkCircular:
		axis := pp.Dir(0, 0, 1)
		// The material's side of a circular wall is decided by the WALK, not
		// by the loop's role: the outward normal is the walk tangent turned a
		// quarter turn against the walk's sense, which is the radial
		// direction away from the centre for a counter-clockwise walk and
		// toward the centre for a clockwise one. A hole is walked clockwise —
		// which is why its wall reverses — but so is a CONCAVE round on an
		// outer loop (a rounded bite out of the boundary), whose material
		// also lies outside the cylinder.
		clockwise := w.Th1 < w.Th0
		// An Arc3/Circle3 is CCW from start to end about its axis. A
		// clockwise walk, and a reflected placement (which flips
		// handedness), each invert that sense, so the EDGE axis carries
		// the corrected sign; the cylinder surface keeps the plain ruling
		// direction.
		edgeSign := 1.0
		if clockwise {
			edgeSign = -1
		}
		if pp.Reflected() {
			edgeSign = -edgeSign
		}
		edgeAxis := axis.Scale(edgeSign)
		center0 := pp.Point(w.CU, w.CV, pp.Z0)
		center1 := pp.Point(w.CU, w.CV, pp.Z1)
		radius := units.Millimeters(w.Radius)
		var curve0, curve1 surfacegeom.Curve
		if closed {
			// A full circle's edge closes on itself: one vertex per cap
			// edge, start == end (root topology.go's Circle3 contract) — the
			// seamBottom/seamTop pair the caller already built.
			curve0, curve1 = surfacegeom.Circle3{Center: center0, Axis: edgeAxis, Radius: radius}, surfacegeom.Circle3{Center: center1, Axis: edgeAxis, Radius: radius}
		} else {
			curve0, curve1 = surfacegeom.Arc3{Center: center0, Axis: edgeAxis, Radius: radius}, surfacegeom.Arc3{Center: center1, Axis: edgeAxis, Radius: radius}
		}
		// The rim edges read the same WALK: a clockwise wall's material
		// lies outside its cylinder, so the boundary turns into the metal
		// there and the rim is concave — a hole's rim (clockwise) and a
		// concave outer bite's (also clockwise) alike, while a
		// counter-clockwise round keeps its convex rim. The loop's role
		// decides nothing here. convex is boundarywalk.RimConvexity's own answer for
		// this walk, computed by the caller.
		bottomEdge := topobuild.NewEdge(alloc, topo.Edge{Convex: convex, Length: w.Length, LengthBound: w.LengthBound, Curve: curve0, Start: bStart, End: bEnd})
		topEdge := topobuild.NewEdge(alloc, topo.Edge{Convex: convex, Length: w.Length, LengthBound: w.LengthBound, Curve: curve1, Start: tStart, End: tEnd})
		if budget == nil {
			bottomEdge.CurveBound, bottomEdge.CurveBounded = pp.CircleCurveBound(w.CU, w.CV, pp.Z0, pp.Z0Delta, 0, w.Radius, w.RadiusBound, center0, edgeAxis)
			topEdge.CurveBound, topEdge.CurveBounded = pp.CircleCurveBound(w.CU, w.CV, pp.Z1, pp.Z1Delta, 0, w.Radius, w.RadiusBound, center1, edgeAxis)
		} else {
			bottomEdge.CurveBound, bottomEdge.CurveBounded, err = pp.CircleCurveBoundSignedReplay(ctx, w.CU, w.CV, pp.Z0, pp.Z0Delta, 0, w.Radius, w.RadiusBound, center0, edgeAxis, budget)
			if err != nil {
				return nil, nil, nil, false, err
			}
			topEdge.CurveBound, topEdge.CurveBounded, err = pp.CircleCurveBoundSignedReplay(ctx, w.CU, w.CV, pp.Z1, pp.Z1Delta, 0, w.Radius, w.RadiusBound, center1, edgeAxis, budget)
			if err != nil {
				return nil, nil, nil, false, err
			}
		}
		surf := surfacegeom.Cylinder{Origin: center0, Axis: axis, Radius: radius}
		// A clockwise-walked wall has its material OUTSIDE the cylinder,
		// so its outward normal is the radial direction negated.
		return bottomEdge, topEdge, surf, clockwise, nil
	case survey2d.WalkFreeform:
		// §6.5's wall-edge convexity certificate, wired here for the first
		// time (agent-docs/spline-design.md §6.5, Table R R19). It applies the
		// one reversal negation internally and states its verdict in the
		// LOOP'S OWN WALK direction, so the mapping below reads it
		// verbatim — a counter-clockwise turn convex, clockwise concave,
		// the identical convention the circular wall's own turn test
		// fixes above. NEVER negate again for a hole loop: a hole rim's
		// concavity falls out of the clockwise walk itself, exactly as it
		// does for a circular hole wall. An R19 refusal already returned
		// from boundarywalk.RimConvexity, before this switch ever ran.
		bottomEdge := topobuild.NewEdge(alloc, topo.Edge{Convex: convex, Length: w.Length, LengthBound: w.LengthBound, Curve: surfacegeom.NURBSCurve{}, Start: bStart, End: bEnd})
		topEdge := topobuild.NewEdge(alloc, topo.Edge{Convex: convex, Length: w.Length, LengthBound: w.LengthBound, Curve: surfacegeom.NURBSCurve{}, Start: tStart, End: tEnd})
		// faceReversed stays false: an opaque NURBSSurface publishes no
		// normal at all (§7's NormalAt refusal, root topology.go), so
		// topo.Face.Reversed — "the outward normal is the surface's geometric
		// normal negated" — names nothing for this variant.
		return bottomEdge, topEdge, surfacegeom.NURBSSurface{}, false, nil
	default:
		// A straight wall has no turn of its own to disagree with the
		// loop's: which side its material lies on is decided by the sense
		// the whole loop is walked, and that sense IS the loop's role —
		// the outer loop counter-clockwise, holes clockwise (internal/momentregion).
		bottomEdge := topobuild.NewEdge(alloc, topo.Edge{Convex: convex, Length: w.Length, LengthBound: w.LengthBound, Curve: surfacegeom.Line3{}, Start: bStart, End: bEnd})
		topEdge := topobuild.NewEdge(alloc, topo.Edge{Convex: convex, Length: w.Length, LengthBound: w.LengthBound, Curve: surfacegeom.Line3{}, Start: tStart, End: tEnd})
		mid := pp.Point((w.StartU+w.EndU)/2, (w.StartV+w.EndV)/2, pp.Z0)
		// tangent × N is the outward normal for a CCW outer walk (and a
		// CW hole walk); a reflection flips the cross product, so the
		// tangent is negated to keep the frame's normal outward.
		tu, tv := w.TanInU, w.TanInV
		if pp.Reflected() {
			tu, tv = -tu, -tv
		}
		f, err := r3.NewFrame(mid, pp.Dir(tu, tv, 0), pp.Dir(0, 0, 1))
		if err != nil {
			return nil, nil, nil, false, fmt.Errorf(`%w: a boundary segment has no direction`, decaderr.ErrDegenerate)
		}
		return bottomEdge, topEdge, surfacegeom.Plane{Frame: f}, false, nil
	}
}

// BuildLoopSides builds one loop's side faces with shared vertices and
// edges, returning the faces, the bottom and top cap coedges in walk order,
// and the loop's perimeter length. A loop's index is both its role index and,
// via li != 0, its orientation: loop 0 is an outer loop (material inside),
// every other a hole (material outside).
//
// resolved is pp.profile's pre-resolved segment walks, or nil to resolve each
// segment through walkOf as before (internal/momentinput/profile_walks.go).
// BuildLoopSides's own li IS resolved's loop index here — li walks the same
// append([]LoopRecord{pp.profile.Outer}, pp.profile.Holes...) order a
// *momentinput.ProfileWalks was resolved from — so it is passed straight through as
// BuildLoopSidesAs's roleLoop, which resolved is read against.
//
// mintCurveTokens is the CURVE half of the shared-denotation certificate's
// own opt-in, read the same way levelZ0/levelZ1's zero value already opts a
// caller out of the LEVEL half: true only from Build's own
// straight-prism build, false from every other caller (root shell_cup.go,
// root capblend_moments.go), which mint no curve identity for their own rim and
// so keep every certificate check refusing by default.
func BuildLoopSides(ctx context.Context, env topo.Env, body *topo.Body, pp Payload, li int, loop sectionrecord.LoopRecord, work *freeform.FreeformWork, resolved *momentinput.ProfileWalks, levelZ0, levelZ1 denotation.LevelToken, mintCurveTokens bool) ([]*topo.Face, []topo.CoEdge, []topo.CoEdge, proofbound.BoundedScalar, error) {
	return BuildLoopSidesAs(ctx, env, body, pp, li, li != 0, loop, work, resolved, levelZ0, levelZ1, mintCurveTokens)
}

// BuildLoopSidesAs is BuildLoopSides with the role index and the orientation
// decoupled: roleLoop names the walls' side(roleLoop,j) role, and holeLoop
// picks the material side of a straight wall independently. A cup's cavity
// walks each loop of the void the SOLID wraps — the void's outer boundary is a
// hole in the solid (holeLoop true), each of the void's own holes a solid post
// (holeLoop false) — a pairing the natural li != 0 rule cannot express
// (agent-docs/modify-design.md §9).
//
// resolved is a *momentinput.ProfileWalks whose loop index roleLoop holds this loop's
// pre-resolved walks, or nil to resolve each segment through walkOf as
// before. A non-nil resolved whose loop at roleLoop was not resolved from
// exactly this loop's recorded segments (LoopMatches — the segments
// themselves, not their count) is a plumbing bug and refuses rather than
// silently resolving anyway — the only caller that ever passes non-nil is
// BuildLoopSides from Build, where roleLoop already IS the loop
// index the *momentinput.ProfileWalks was resolved at.
//
// levelZ0 and levelZ1 are the LEVEL half of the shared-denotation certificate
// (internal/denotation), minted once per build by the caller's own Build
// and stamped here onto every vertex and rim edge this loop places at its
// respective end — the zero value declines for every OTHER caller of this
// function (root shell_cup.go, root capblend_moments.go), which mint neither and so
// leave every certificate check refusing by default.
//
// mintCurveTokens is the CURVE half of the shared-denotation certificate:
// true mints a fresh curveID (body.Doc.MintCurve) for every rim vertex and
// rim edge this loop places, at EITHER end, unconditionally — unlike the
// LEVEL half, minting a curve identity claims nothing about the section
// itself, so it needs no sectionDelta==0 precondition; freshness alone is
// what keeps two separate builds from ever sharing one (internal/denotation).
// False for every OTHER caller of this function, which mints no curve
// identity for their own rim.
func BuildLoopSidesAs(ctx context.Context, env topo.Env, body *topo.Body, pp Payload, roleLoop int, holeLoop bool, loop sectionrecord.LoopRecord, work *freeform.FreeformWork, resolved *momentinput.ProfileWalks, levelZ0, levelZ1 denotation.LevelToken, mintCurveTokens bool) ([]*topo.Face, []topo.CoEdge, []topo.CoEdge, proofbound.BoundedScalar, error) {
	return buildLoopSidesAs(ctx, env, body, pp, roleLoop, holeLoop, loop, work, resolved, levelZ0, levelZ1, mintCurveTokens, false, nil, nil)
}

// BuildLoopSidesWithWitness records the actual face created for each unmerged
// source segment. The caller must authenticate this construction witness.
func BuildLoopSidesWithWitness(ctx context.Context, env topo.Env, body *topo.Body, pp Payload,
	roleLoop int, holeLoop bool, loop sectionrecord.LoopRecord, work *freeform.FreeformWork,
	resolved *momentinput.ProfileWalks, levelZ0, levelZ1 denotation.LevelToken,
	mintCurveTokens bool,
) ([]*topo.Face, []topo.CoEdge, []topo.CoEdge, proofbound.BoundedScalar, []*topo.Face, error) {
	witness := &PrismSourceWitness{Segments: loop.Segments}
	faces, bottom, top, length, err := buildLoopSidesAs(ctx, env, body, pp, roleLoop,
		holeLoop, loop, work, resolved, levelZ0, levelZ1, mintCurveTokens, false, nil, witness)
	if err != nil {
		return nil, nil, nil, proofbound.BoundedScalar{}, nil, err
	}
	if !witness.valid {
		return faces, bottom, top, length, nil, nil
	}
	return faces, bottom, top, length, witness.Walls, nil
}

// BuildLoopSidesAsReplay charges the exact lifted rim points to one replay
// meter. The remaining side-building stages still need accounting.
func BuildLoopSidesAsReplay(ctx context.Context, env topo.Env, body *topo.Body, pp Payload, roleLoop int, holeLoop bool,
	loop sectionrecord.LoopRecord, work *freeform.FreeformWork, resolved *momentinput.ProfileWalks,
	levelZ0, levelZ1 denotation.LevelToken, mintCurveTokens bool, budget *proofbound.ReplayBudget,
) ([]*topo.Face, []topo.CoEdge, []topo.CoEdge, proofbound.BoundedScalar, error) {
	if budget == nil {
		return nil, nil, nil, proofbound.BoundedScalar{}, fmt.Errorf("%w: missing side-build budget", decaderr.ErrUnsupported)
	}
	return buildLoopSidesAs(ctx, env, body, pp, roleLoop, holeLoop, loop, work, resolved, levelZ0, levelZ1, mintCurveTokens, false, budget, nil)
}

func buildLoopSidesAs(ctx context.Context, env topo.Env, body *topo.Body, pp Payload, roleLoop int, holeLoop bool,
	loop sectionrecord.LoopRecord, work *freeform.FreeformWork, resolved *momentinput.ProfileWalks,
	levelZ0, levelZ1 denotation.LevelToken, mintCurveTokens, recordPrismFace bool, budget *proofbound.ReplayBudget,
	witness *PrismSourceWitness,
) ([]*topo.Face, []topo.CoEdge, []topo.CoEdge, proofbound.BoundedScalar, error) {
	mintCurve := func() denotation.CurveToken {
		if !mintCurveTokens {
			return denotation.CurveToken{}
		}
		return body.Doc.MintCurve()
	}
	if err := ctx.Err(); err != nil {
		return nil, nil, nil, proofbound.BoundedScalar{}, err
	}
	if len(loop.Segments) == 0 {
		return nil, nil, nil, proofbound.BoundedScalar{}, fmt.Errorf(`%w: a recorded loop holds no segments`, decaderr.ErrDegenerate)
	}
	var loopWalks []survey2d.SegmentWalk
	if resolved != nil {
		matches := false
		if budget == nil {
			matches = resolved.LoopMatches(roleLoop, loop)
		} else {
			var err error
			matches, err = resolved.LoopMatchesReplay(ctx, roleLoop, loop, budget)
			if err != nil {
				return nil, nil, nil, proofbound.BoundedScalar{}, err
			}
		}
		if !matches {
			return nil, nil, nil, proofbound.BoundedScalar{}, momentinput.ErrResolvedWalksMismatch
		}
		loopWalks = resolved.LoopWalks(roleLoop)
	}
	// Every coordinate this loop's walks read sits within the payload's own
	// section displacement of the section it denotes, so each walk's length, each
	// junction vertex and each side face carries that displacement too
	// (agent-docs/prism-boolean-design.md §7). It is zero for every payload a caller
	// draws, and the arithmetic below is then the unchanged one.
	delta := pp.SectionDelta
	walkLenAllow := proofbound.SectionDisplacementLength(delta, 1)
	if budget != nil {
		if err := budget.ReserveRecords(ctx, uint64(len(loop.Segments))); err != nil {
			return nil, nil, nil, proofbound.BoundedScalar{}, err
		}
		if err := budget.ReserveArray(ctx, uint64(len(loop.Segments)), uint64(unsafe.Sizeof(survey2d.SideWalk{}))); err != nil {
			return nil, nil, nil, proofbound.BoundedScalar{}, err
		}
	}
	raw := make([]survey2d.SideWalk, len(loop.Segments))
	total := proofbound.BoundedScalar{}
	for i, seg := range loop.Segments {
		if err := ctx.Err(); err != nil {
			return nil, nil, nil, proofbound.BoundedScalar{}, err
		}
		if budget != nil {
			switch seg.(type) {
			case sectionrecord.LineSeg, sectionrecord.CircleSeg, sectionrecord.ArcSeg:
			default:
				return nil, nil, nil, proofbound.BoundedScalar{}, decaderr.ErrUnsupported
			}
			if err := budget.SpendWork(ctx, 16); err != nil {
				return nil, nil, nil, proofbound.BoundedScalar{}, err
			}
		}
		seg, err := sectionrecord.NormalizeSegment(seg)
		if err != nil {
			return nil, nil, nil, proofbound.BoundedScalar{}, err
		}
		// A Tier B or Tier C free-form kind (a conic, a whole ellipse, an
		// unequal-weight NURBS, or an elliptical arc) refuses inside walkOf
		// itself (freeformBezierSpans, Table R R2/R10) — this build stages no
		// gate of its own ahead of it any more (§10 P4b retires R6). A
		// resolved walk was already through walkOf once (momentinput.ResolveProfileWalks),
		// so it carries the same refusal already surfaced there.
		var w survey2d.SegmentWalk
		if loopWalks != nil {
			w = loopWalks[i]
		} else if budget != nil {
			circle, ok := seg.(sectionrecord.CircleSeg)
			if !ok {
				return nil, nil, nil, proofbound.BoundedScalar{}, decaderr.ErrUnsupported
			}
			w, err = boundarywalk.WholeCircleWalkReplay(ctx, circle, budget)
			if err != nil {
				return nil, nil, nil, proofbound.BoundedScalar{}, err
			}
		} else {
			w, err = boundarywalk.WalkOf(seg, work)
			if err != nil {
				return nil, nil, nil, proofbound.BoundedScalar{}, err
			}
		}
		w.LengthBound = proofbound.AbsSumUpper(w.LengthBound, walkLenAllow)
		if budget != nil {
			if err := budget.ReserveRecords(ctx, 1); err != nil {
				return nil, nil, nil, proofbound.BoundedScalar{}, err
			}
			if err := budget.ReserveArray(ctx, 1, uint64(unsafe.Sizeof(int(0)))); err != nil {
				return nil, nil, nil, proofbound.BoundedScalar{}, err
			}
		}
		raw[i] = survey2d.SideWalk{SegmentWalk: w, Segs: []int{i}}
		if budget == nil {
			total = proofbound.BoundedAdd(total, proofbound.MeasuredScalar(w.Length, w.LengthBound))
		} else {
			total, err = proofbound.BoundedAddReplay(ctx, total, proofbound.MeasuredScalar(w.Length, w.LengthBound), budget)
			if err != nil {
				return nil, nil, nil, proofbound.BoundedScalar{}, err
			}
		}
	}
	var walks []survey2d.SideWalk
	var err error
	if budget == nil {
		walks, err = boundarywalk.CoalesceWalksContext(ctx, raw)
	} else {
		walks, err = boundarywalk.CoalesceWalksNoMergeReplay(ctx, raw, budget)
	}
	if err != nil {
		return nil, nil, nil, proofbound.BoundedScalar{}, err
	}
	n := len(walks)

	// Junction vertices, shared between neighbors: junction i sits at walk
	// i's start (== walk i−1's end). A single whole closed curve has none.
	singleClosed := n == 1 && walks[0].Closed
	// The sweep height is read from the two BOUNDED levels, so a level the
	// evaluator computed carries its own displacement into the vertical edge
	// lengths and the side face areas built from it below — a ToFace or
	// ThroughAll stop's float arithmetic (root stops.go), a non-base unit's rescale,
	// and a chamfered end's setback alike.
	var height proofbound.BoundedScalar
	if budget == nil {
		height = proofbound.BoundedSub(pp.Z1Scalar(), pp.Z0Scalar())
	} else {
		height, err = proofbound.BoundedSubReplay(ctx, pp.Z1Scalar(), pp.Z0Scalar(), budget)
		if err != nil {
			return nil, nil, nil, proofbound.BoundedScalar{}, err
		}
	}
	// A side vertex sits at one recorded boundary coordinate and one sweep level,
	// so it carries both displacements: the section's, which moves it in the
	// plane, and its own end's, which moves it along the normal. Each is zero for
	// a coordinate the payload recorded from what the caller stated. Beside them
	// every rim vertex carries its OWN frame lift and accumulated placement
	// rounding, measured exactly for that vertex (Payload.LiftedVertex;
	// root topology.go's Vertex.Position contract) — zero wherever the lift is exact
	// for the coordinates at hand, which is what keeps an ordinary extrude's
	// vertices Exact. None of the three is a claim about another's axis, so
	// they compose rather than one standing in for another.
	// rimVertex places one rim vertex at (u, v) on level z, whose own axial
	// displacement is zDelta, charging the section's displacement, that level's,
	// the vertex's own exact lift rounding and the walk-end term extra.
	rimVertex := func(u, v, z, zDelta, extra float64, level denotation.LevelToken) (*topo.Vertex, error) {
		var held r3.Vec
		var lift float64
		if budget == nil {
			held, lift = pp.LiftedVertex(u, v, z)
		} else {
			var err error
			held, lift, err = pp.LiftedVertexReplay(ctx, u, v, z, budget)
			if err != nil {
				return nil, err
			}
			if err := budget.ReserveRecords(ctx, 1); err != nil {
				return nil, err
			}
			if err := budget.ReserveArray(ctx, 1, uint64(unsafe.Sizeof(topo.Vertex{}))+128); err != nil {
				return nil, err
			}
		}
		base := proofbound.AbsSumUpper(delta, zDelta, lift)
		return topobuild.NewVertex(env.Alloc, topo.Vertex{Position: held, Bound: units.Millimeters(proofbound.AbsSumUpper(base, extra)), Level: level, Denot: mintCurve()}), nil
	}
	var bottomV, topV []*topo.Vertex
	// seamBottom/seamTop are the SINGLE seam vertex a lone closed walk's rim
	// edges share at each cap — one per cap, no junction vertex at all — the
	// free-form twin of a whole CircleSeg's own seam vertex. Hoisted out of the
	// per-kind switch below (§10 P4b): a closed free-form walk reaches
	// singleClosed on the same terms a whole circle does, and the switch's
	// survey2d.WalkFreeform arm needs the SAME seam pair the survey2d.WalkCircular arm builds.
	var seamBottom, seamTop *topo.Vertex
	junction := func(prev, next survey2d.SideWalk) (float64, float64, float64, error) {
		if budget != nil {
			return boundarywalk.JunctionVertexAtReplay(ctx, loop.Segments, prev, next, budget)
		}
		u, v, extra := boundarywalk.JunctionVertexAt(loop.Segments, prev, next)
		return u, v, extra, nil
	}
	if singleClosed {
		w := walks[0]
		u, v, extra, err := junction(w, w)
		if err != nil {
			return nil, nil, nil, proofbound.BoundedScalar{}, err
		}
		seamBottom, err = rimVertex(u, v, pp.Z0, pp.Z0Delta, extra, levelZ0)
		if err != nil {
			return nil, nil, nil, proofbound.BoundedScalar{}, err
		}
		seamTop, err = rimVertex(u, v, pp.Z1, pp.Z1Delta, extra, levelZ1)
		if err != nil {
			return nil, nil, nil, proofbound.BoundedScalar{}, err
		}
	} else {
		if budget != nil {
			if err := budget.ReserveArray(ctx, uint64(n), uint64(unsafe.Sizeof((*topo.Vertex)(nil)))); err != nil {
				return nil, nil, nil, proofbound.BoundedScalar{}, err
			}
			if err := budget.ReserveArray(ctx, uint64(n), uint64(unsafe.Sizeof((*topo.Vertex)(nil)))); err != nil {
				return nil, nil, nil, proofbound.BoundedScalar{}, err
			}
		}
		bottomV = make([]*topo.Vertex, n)
		topV = make([]*topo.Vertex, n)
		for i, w := range walks {
			if err := ctx.Err(); err != nil {
				return nil, nil, nil, proofbound.BoundedScalar{}, err
			}
			prev := walks[(i+n-1)%n]
			u, v, extra, err := junction(prev, w)
			if err != nil {
				return nil, nil, nil, proofbound.BoundedScalar{}, err
			}
			bottomV[i], err = rimVertex(u, v, pp.Z0, pp.Z0Delta, extra, levelZ0)
			if err != nil {
				return nil, nil, nil, proofbound.BoundedScalar{}, err
			}
			topV[i], err = rimVertex(u, v, pp.Z1, pp.Z1Delta, extra, levelZ1)
			if err != nil {
				return nil, nil, nil, proofbound.BoundedScalar{}, err
			}
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
	// survey2d.SegmentWalk's convention because it comes from trig at a computed
	// angle. This cross needs no change for either kind.
	var vertical []*topo.Edge
	if !singleClosed {
		if budget != nil {
			if err := budget.ReserveArray(ctx, uint64(n), uint64(unsafe.Sizeof((*topo.Edge)(nil)))); err != nil {
				return nil, nil, nil, proofbound.BoundedScalar{}, err
			}
		}
		vertical = make([]*topo.Edge, n)
		for i := range walks {
			if err := ctx.Err(); err != nil {
				return nil, nil, nil, proofbound.BoundedScalar{}, err
			}
			prev := walks[(i+n-1)%n]
			cross := prev.TanOutU*walks[i].TanInV - prev.TanOutV*walks[i].TanInU
			if budget != nil {
				if err := budget.SpendWork(ctx, 32); err != nil {
					return nil, nil, nil, proofbound.BoundedScalar{}, err
				}
				if err := budget.ReserveRecords(ctx, 1); err != nil {
					return nil, nil, nil, proofbound.BoundedScalar{}, err
				}
				if err := budget.ReserveArray(ctx, 1, uint64(unsafe.Sizeof(topo.Edge{}))); err != nil {
					return nil, nil, nil, proofbound.BoundedScalar{}, err
				}
			}
			vertical[i] = topobuild.NewEdge(env.Alloc, topo.Edge{
				Convex:      cross > 0,
				Length:      pp.Z1 - pp.Z0,
				LengthBound: height.Bound,
				Curve:       surfacegeom.Line3{},
				Start:       bottomV[i],
				End:         topV[i],
			})
		}
	}

	// Side faces with bottom/top edges; cap coedges accumulate in walk order.
	if budget != nil {
		if err := budget.ReserveArray(ctx, uint64(n), uint64(unsafe.Sizeof((*topo.Face)(nil)))); err != nil {
			return nil, nil, nil, proofbound.BoundedScalar{}, err
		}
		if err := budget.ReserveArray(ctx, uint64(n), uint64(unsafe.Sizeof(topo.CoEdge{}))); err != nil {
			return nil, nil, nil, proofbound.BoundedScalar{}, err
		}
		if err := budget.ReserveArray(ctx, uint64(n), uint64(unsafe.Sizeof(topo.CoEdge{}))); err != nil {
			return nil, nil, nil, proofbound.BoundedScalar{}, err
		}
	}
	faces := make([]*topo.Face, 0, n)
	bottomCo := make([]topo.CoEdge, 0, n)
	topCo := make([]topo.CoEdge, 0, n)
	for i, w := range walks {
		if err := ctx.Err(); err != nil {
			return nil, nil, nil, proofbound.BoundedScalar{}, err
		}
		var bStart, bEnd, tStart, tEnd *topo.Vertex
		if singleClosed {
			bStart, bEnd = seamBottom, seamBottom
			tStart, tEnd = seamTop, seamTop
		} else {
			bStart, tStart = bottomV[i], topV[i]
			bEnd, tEnd = bottomV[(i+1)%n], topV[(i+1)%n]
		}
		if budget != nil {
			if w.Kind != survey2d.WalkLine && w.Kind != survey2d.WalkCircular {
				return nil, nil, nil, proofbound.BoundedScalar{}, decaderr.ErrUnsupported
			}
			if err := budget.SpendWork(ctx, 64); err != nil {
				return nil, nil, nil, proofbound.BoundedScalar{}, err
			}
			if err := budget.ReserveRecords(ctx, 2); err != nil {
				return nil, nil, nil, proofbound.BoundedScalar{}, err
			}
			if err := budget.ReserveArray(ctx, 2, uint64(unsafe.Sizeof(topo.Edge{}))); err != nil {
				return nil, nil, nil, proofbound.BoundedScalar{}, err
			}
			if err := budget.ReserveBytes(ctx, 1024); err != nil {
				return nil, nil, nil, proofbound.BoundedScalar{}, err
			}
		}
		convex, err := boundarywalk.RimConvexity(ctx, w, holeLoop, work)
		if err != nil {
			return nil, nil, nil, proofbound.BoundedScalar{}, err
		}
		var bottomEdge, topEdge *topo.Edge
		var surf surfacegeom.Surface
		var faceReversed bool
		if budget == nil {
			bottomEdge, topEdge, surf, faceReversed, err = BuildWallGeometry(env.Alloc, pp, w, convex, singleClosed, bStart, bEnd, tStart, tEnd) //nolint:contextcheck // The ordinary wall builder has no replay context or budget.
		} else {
			bottomEdge, topEdge, surf, faceReversed, err = BuildWallGeometryReplay(ctx, env.Alloc, pp, w, convex, singleClosed, bStart, bEnd, tStart, tEnd, budget)
		}
		if err != nil {
			return nil, nil, nil, proofbound.BoundedScalar{}, err
		}
		// The LEVEL certificate travels uniformly, whatever curve kind this
		// rim edge carries: every rim edge at one end is stamped by the SAME
		// Build call over the SAME recorded frame and level.
		bottomEdge.Level = levelZ0
		topEdge.Level = levelZ1
		// The CURVE certificate is minted fresh per rim edge, never shared:
		// each rim edge denotes its OWN curve (a different wall's own top or
		// bottom boundary), unlike the level, which many rim edges share.
		bottomEdge.Denot = mintCurve()
		topEdge.Denot = mintCurve()

		if budget != nil {
			if err := budget.ReserveRecords(ctx, uint64(len(w.Segs))); err != nil {
				return nil, nil, nil, proofbound.BoundedScalar{}, err
			}
			if err := budget.ReserveArray(ctx, uint64(len(w.Segs)), uint64(unsafe.Sizeof(provenance.Ref{}))); err != nil {
				return nil, nil, nil, proofbound.BoundedScalar{}, err
			}
			for range w.Segs {
				if err := budget.ReserveBytes(ctx, 512); err != nil {
					return nil, nil, nil, proofbound.BoundedScalar{}, err
				}
				if err := budget.SpendWork(ctx, 128); err != nil {
					return nil, nil, nil, proofbound.BoundedScalar{}, err
				}
			}
		}
		origins, err := SideOrigins(ctx, env.Ref, roleLoop, w.Segs)
		if err != nil {
			return nil, nil, nil, proofbound.BoundedScalar{}, err
		}
		var faceArea proofbound.BoundedScalar
		if budget == nil {
			faceArea = proofbound.BoundedMul(proofbound.MeasuredScalar(w.Length, w.LengthBound), height)
		} else {
			faceArea, err = proofbound.BoundedMulReplay(ctx, proofbound.MeasuredScalar(w.Length, w.LengthBound), height, budget)
			if err != nil {
				return nil, nil, nil, proofbound.BoundedScalar{}, err
			}
		}
		if budget != nil {
			loops := uint64(1)
			coedges := uint64(4)
			if singleClosed {
				loops, coedges = 2, 2
			}
			if err := budget.ReserveRecords(ctx, 1+loops+coedges); err != nil {
				return nil, nil, nil, proofbound.BoundedScalar{}, err
			}
			if err := budget.ReserveArray(ctx, 1, uint64(unsafe.Sizeof(topo.Face{}))); err != nil {
				return nil, nil, nil, proofbound.BoundedScalar{}, err
			}
			if err := budget.ReserveArray(ctx, loops, uint64(unsafe.Sizeof(topo.Loop{}))+uint64(unsafe.Sizeof((*topo.Loop)(nil)))); err != nil {
				return nil, nil, nil, proofbound.BoundedScalar{}, err
			}
			if err := budget.ReserveArray(ctx, coedges, uint64(unsafe.Sizeof(topo.CoEdge{}))); err != nil {
				return nil, nil, nil, proofbound.BoundedScalar{}, err
			}
			if err := budget.ReserveArray(ctx, 4, uint64(unsafe.Sizeof((*topo.Face)(nil)))); err != nil {
				return nil, nil, nil, proofbound.BoundedScalar{}, err
			}
		}
		face := topobuild.NewFace(env.Alloc, topo.Face{
			Area:      faceArea.Value,
			AreaBound: faceArea.Bound,
			Reversed:  faceReversed,
			Surface:   surf,
			Origins:   origins,
			Body:      body,
		})
		if recordPrismFace && len(w.Segs) == 1 {
			face.RecordPrismFace(topo.PrismFaceWall, roleLoop, w.Segs[0])
		}
		if singleClosed {
			// A closed cylindrical band has TWO boundary loops — one circle
			// per cap — not one loop holding both.
			face.Loops = []*topo.Loop{
				topobuild.NewLoop(env.Alloc, topo.Loop{Outer: true, CoEdges: []topo.CoEdge{{Edge: bottomEdge, Forward: true}}}),
				topobuild.NewLoop(env.Alloc, topo.Loop{Outer: true, CoEdges: []topo.CoEdge{{Edge: topEdge, Forward: false}}}),
			}
		} else {
			face.Loops = []*topo.Loop{topobuild.NewLoop(env.Alloc, topo.Loop{Outer: true, CoEdges: []topo.CoEdge{
				{Edge: bottomEdge, Forward: true},
				{Edge: vertical[(i+1)%n], Forward: true},
				{Edge: topEdge, Forward: false},
				{Edge: vertical[i], Forward: false},
			},
			})}
		}

		bottomEdge.Faces = append(bottomEdge.Faces, face)
		topEdge.Faces = append(topEdge.Faces, face)
		if !singleClosed {
			vertical[i].Faces = append(vertical[i].Faces, face)
			vertical[(i+1)%n].Faces = append(vertical[(i+1)%n].Faces, face)
		}

		faces = append(faces, face)
		bottomCo = append(bottomCo, topo.CoEdge{Edge: bottomEdge, Forward: true})
		topCo = append(topCo, topo.CoEdge{Edge: topEdge, Forward: true})
	}
	if witness != nil && roleLoop == 0 && !holeLoop {
		witness.captureWalks(walks, faces)
	}
	return faces, bottomCo, topCo, total, nil
}

// SideOrigins names the side(roleLoop,j) role of each recorded segment j a
// wall was built from, under ref.
func SideOrigins(ctx context.Context, ref provenance.Producer, roleLoop int, segs []int) ([]provenance.Ref, error) {
	origins := make([]provenance.Ref, len(segs))
	for oi, si := range segs {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		origins[oi] = provenance.Ref{Producer: ref, Role: fmt.Sprintf("side(%d,%d)", roleLoop, si)}
	}
	return origins, nil
}
