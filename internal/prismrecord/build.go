package prismrecord

import (
	"context"
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/denotation"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/massmoment"
	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismextent"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/surfacegroup"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// This file is the straight prism's builder over topology nodes: root's
// evalPrismContext (prism_build.go) for a payload with no blend roles, with
// every node allocated through env.Alloc and every certificate minted through
// env.Doc. See docs/evaluator-design.md §5.

// Transform is the accumulated rigid placement.
func (pp Payload) Transform() r3.Transform { return pp.Xform }

// Placed re-evaluates the same record under the composed motion. It is a
// re-evaluation path: no moments preflight has run on this record within the
// call, so the build opens the record's one counter itself. It never commits.
func (pp Payload) Placed(ctx context.Context, env topo.Env, composed r3.Transform) (*topo.Body, error) {
	pp.Xform = composed
	return Build(ctx, env, pp, freeform.NewFreeformWork())
}

// Build builds the straight prism p records: walls, caps, shared edges and
// vertices, and the body's measurements, allocating every node through
// env.Alloc and minting through env.Doc. It never commits. A payload with blend
// roles is ErrUnsupported: those roles are minted by root (fillet.go).
//
// work is the record's ONE free-form work counter: the preflight this build
// runs and every walk under it charge the same ceiling.
func Build(ctx context.Context, env topo.Env, p Payload, work *freeform.FreeformWork) (*topo.Body, error) {
	if len(p.BlendSegs) != 0 {
		return nil, fmt.Errorf(`%w: the prism builder does not mint blend roles`, decaderr.ErrUnsupported)
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	ig, err := p.Profile.EvaluatorIntegralsUncheckedContext(ctx, freeform.MomentFirstOrder, work)
	if err != nil {
		return nil, err
	}
	if ig.Area <= 0 {
		return nil, fmt.Errorf(`%w: the recorded region encloses no area`, decaderr.ErrDegenerate)
	}
	height := proofbound.BoundedSub(p.Z1Scalar(), p.Z0Scalar())
	if height.Value <= 0 {
		return nil, fmt.Errorf(`%w: the sweep interval is empty`, decaderr.ErrDegenerate)
	}

	// A surface result publishes a sheet, never a solid
	// (docs/surface-design.md §4.3).
	kind := topo.BodySolid
	if p.SurfaceResult {
		kind = topo.BodySheet
	}
	body := env.Alloc.NewBody()
	body.Doc = env.Doc
	body.Origin = provenance.Ref{Producer: env.Ref, Role: provenance.RoleBody}
	body.Solid = !p.SurfaceResult
	body.Kind = kind

	// The LEVEL half of the shared-denotation certificate, minted only when
	// this payload's own section is drawn straight from its record.
	var levelZ0, levelZ1 denotation.LevelToken
	if p.SectionDelta == 0 {
		levelNormal := p.Dir(0, 0, 1)
		levelZ0 = env.Doc.MintLevel(p.Point(0, 0, p.Z0), levelNormal)
		levelZ1 = env.Doc.MintLevel(p.Point(0, 0, p.Z1), levelNormal)
	}

	var faces []*topo.Face
	startFrame, err := p.CapFrame(p.Z0, true)
	if err != nil {
		return nil, err
	}
	endFrame, err := p.CapFrame(p.Z1, false)
	if err != nil {
		return nil, err
	}
	capStart := env.Alloc.NewFace()
	capStart.Surface = surfacegeom.Plane{Frame: startFrame}
	capStart.Origins = []provenance.Ref{{Producer: env.Ref, Role: provenance.RoleCapStart}}
	capStart.Body = body
	capStart.Area, capStart.AreaBound = ig.Area, ig.AreaBound
	capStart.AxialDelta, capStart.HasAxialDelta = p.Z0Delta, true
	capEnd := env.Alloc.NewFace()
	capEnd.Surface = surfacegeom.Plane{Frame: endFrame}
	capEnd.Origins = []provenance.Ref{{Producer: env.Ref, Role: provenance.RoleCapEnd}}
	capEnd.Body = body
	capEnd.Area, capEnd.AreaBound = ig.Area, ig.AreaBound
	capEnd.AxialDelta, capEnd.HasAxialDelta = p.Z1Delta, true

	perimeter := proofbound.BoundedScalar{}
	walks := 0
	loops := append([]sectionrecord.LoopRecord{p.Profile.Outer}, p.Profile.Holes...)
	for _, loop := range loops {
		walks += len(loop.Segments)
	}
	// pw resolves every boundary segment's walk exactly ONCE for this whole
	// build; a payload that already carries the resolution of THIS record hands
	// it over and is charged what it cost.
	pw := p.Walks
	if pw.Reusable(p.Profile) {
		if err := pw.Charge(work); err != nil {
			return nil, err
		}
	} else {
		resolved, err := momentinput.ResolveProfileWalks(p.Profile, work)
		if err != nil {
			return nil, err
		}
		pw = resolved
	}
	b := builder{env: env, body: body, p: p, work: work, resolved: pw, levelZ0: levelZ0, levelZ1: levelZ1}
	for li, loop := range loops {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		sideFaces, bottom, top, loopLen, err := b.loopSides(ctx, li, loop)
		if err != nil {
			return nil, err
		}
		faces = append(faces, sideFaces...)
		perimeter = proofbound.BoundedAdd(perimeter, loopLen)
		capStart.Loops = append(capStart.Loops, b.newLoop(bottom, li == 0))
		capEnd.Loops = append(capEnd.Loops, b.newLoop(top, li == 0))
	}
	// A surface result omits both caps from the shell (Table W,
	// docs/surface-design.md §4.1-§4.2).
	if !p.SurfaceResult {
		faces = append(faces, capStart, capEnd)
		if err := attachFaceLoopsContext(ctx, []*topo.Face{capStart, capEnd}); err != nil {
			return nil, err
		}
	}
	body.Lumps = sheetLumps(env.Alloc, faces)

	// Measurements carry the closed forms' proven float bounds plus the
	// section's own displacement (docs/prism-boolean-design.md §7).
	delta := p.SectionDelta
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
	if p.SurfaceResult {
		area = proofbound.BoundedSub(area, proofbound.MeasuredScalar(capStart.Area, capStart.AreaBound))
		area = proofbound.BoundedSub(area, proofbound.MeasuredScalar(capEnd.Area, capEnd.AreaBound))
	}
	body.Volume = measurement.Measurement{
		Value:     units.CubicMillimeters(volume.Value),
		Exactness: exactnessOf(volume.Bound),
		Bound:     units.CubicMillimeters(volume.Bound),
	}
	body.Area = measurement.Measurement{
		Value:     units.SquareMillimeters(area.Value),
		Exactness: exactnessOf(area.Bound),
		Bound:     units.SquareMillimeters(area.Bound),
	}
	cu := proofbound.BoundedQuotient(ig.Mu, ig.MuBound, ig.Area, ig.AreaBound)
	cv := proofbound.BoundedQuotient(ig.Mv, ig.MvBound, ig.Area, ig.AreaBound)
	zc := proofbound.BoundedDiv(proofbound.BoundedAdd(p.Z0Scalar(), p.Z1Scalar()), proofbound.ExactScalar(2))
	centroidValue := p.Point(cu.Value, cv.Value, zc.Value)
	cu.Bound = proofbound.AbsSumUpper(cu.Bound, delta)
	cv.Bound = proofbound.AbsSumUpper(cv.Bound, delta)
	centroidBound := p.PointBound(cu, cv, zc)
	geometryBound, err := p.CentroidGeometryBound(p.Profile, centroidValue, work, pw)
	if err != nil {
		return nil, err
	}
	centroidBound = math.Min(centroidBound, proofbound.AbsSumUpper(geometryBound, delta))
	body.Centroid = measurement.VecMeasurement{
		Value:     centroidValue,
		Exactness: exactnessOf(centroidBound),
		Bound:     units.Millimeters(centroidBound),
	}
	bounds, err := p.boundsContext(ctx, work, pw)
	if err != nil {
		return nil, err
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	body.Bounds = bounds
	if err := chargePrismMap(body, p.Frame, p.Xform); err != nil {
		return nil, err
	}
	if err := validateMeasurements(body); err != nil {
		return nil, err
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	// The build succeeded, so pw is a resolution of a record this body now
	// owns; publishing it lets a later rigid re-evaluation read it back.
	p.Walks = pw
	body.Payload = p
	return body, nil
}

// builder carries one Build call's fixed inputs through its per-loop walk.
type builder struct {
	env      topo.Env
	body     *topo.Body
	p        Payload
	work     *freeform.FreeformWork
	resolved *momentinput.ProfileWalks
	levelZ0  denotation.LevelToken
	levelZ1  denotation.LevelToken
}

func (b *builder) newLoop(coedges []topo.CoEdge, outer bool) *topo.Loop {
	l := b.env.Alloc.NewLoop()
	l.CoEdges = coedges
	l.Outer = outer
	return l
}

func (b *builder) newEdge(curve surfacegeom.Curve, start, end *topo.Vertex, convex bool, length, lengthBound float64) *topo.Edge {
	e := b.env.Alloc.NewEdge()
	e.Curve = curve
	e.Start, e.End = start, end
	e.Convex = convex
	e.Length, e.LengthBound = length, lengthBound
	return e
}

// attachFaceLoopsContext links each coedge's shared edge to its face. The
// assembly can be large for a recorded region with many loops, so every
// nesting level polls ctx (root attachFaceLoopsContext; the name is kept
// because cancellation tests target it by frame).
func attachFaceLoopsContext(ctx context.Context, faces []*topo.Face) error {
	for _, f := range faces {
		if err := ctx.Err(); err != nil {
			return err
		}
		for _, l := range f.Loops {
			if err := ctx.Err(); err != nil {
				return err
			}
			for _, ce := range l.CoEdges {
				if err := ctx.Err(); err != nil {
					return err
				}
				ce.Edge.Faces = append(ce.Edge.Faces, f)
			}
		}
	}
	return nil
}

// wallGeometry builds one wall's two rim edges and the surface it sweeps, from
// its walk alone (root buildWallGeometry).
func (b *builder) wallGeometry(w survey2d.SideWalk, convex, closed bool, bStart, bEnd, tStart, tEnd *topo.Vertex) (*topo.Edge, *topo.Edge, surfacegeom.Surface, bool, error) {
	p := b.p
	switch w.Kind {
	case survey2d.WalkCircular:
		axis := p.Dir(0, 0, 1)
		// The material's side of a circular wall is decided by the WALK: a
		// clockwise walk has its material outside the cylinder.
		clockwise := w.Th1 < w.Th0
		edgeSign := 1.0
		if clockwise {
			edgeSign = -1
		}
		if p.Reflected() {
			edgeSign = -edgeSign
		}
		edgeAxis := axis.Scale(edgeSign)
		center0 := p.Point(w.CU, w.CV, p.Z0)
		center1 := p.Point(w.CU, w.CV, p.Z1)
		radius := units.Millimeters(w.Radius)
		var curve0, curve1 surfacegeom.Curve
		if closed {
			curve0 = surfacegeom.Circle3{Center: center0, Axis: edgeAxis, Radius: radius}
			curve1 = surfacegeom.Circle3{Center: center1, Axis: edgeAxis, Radius: radius}
		} else {
			curve0 = surfacegeom.Arc3{Center: center0, Axis: edgeAxis, Radius: radius}
			curve1 = surfacegeom.Arc3{Center: center1, Axis: edgeAxis, Radius: radius}
		}
		bottomEdge := b.newEdge(curve0, bStart, bEnd, convex, w.Length, w.LengthBound)
		topEdge := b.newEdge(curve1, tStart, tEnd, convex, w.Length, w.LengthBound)
		bottomEdge.CurveBound, bottomEdge.CurveBounded = p.CircleCurveBound(w.CU, w.CV, p.Z0, p.Z0Delta, 0, w.Radius, w.RadiusBound, center0, edgeAxis)
		topEdge.CurveBound, topEdge.CurveBounded = p.CircleCurveBound(w.CU, w.CV, p.Z1, p.Z1Delta, 0, w.Radius, w.RadiusBound, center1, edgeAxis)
		surf := surfacegeom.Cylinder{Origin: center0, Axis: axis, Radius: radius}
		return bottomEdge, topEdge, surf, clockwise, nil
	case survey2d.WalkFreeform:
		bottomEdge := b.newEdge(surfacegeom.NURBSCurve{}, bStart, bEnd, convex, w.Length, w.LengthBound)
		topEdge := b.newEdge(surfacegeom.NURBSCurve{}, tStart, tEnd, convex, w.Length, w.LengthBound)
		return bottomEdge, topEdge, surfacegeom.NURBSSurface{}, false, nil
	default:
		bottomEdge := b.newEdge(surfacegeom.Line3{}, bStart, bEnd, convex, w.Length, w.LengthBound)
		topEdge := b.newEdge(surfacegeom.Line3{}, tStart, tEnd, convex, w.Length, w.LengthBound)
		mid := p.Point((w.StartU+w.EndU)/2, (w.StartV+w.EndV)/2, p.Z0)
		tu, tv := w.TanInU, w.TanInV
		if p.Reflected() {
			tu, tv = -tu, -tv
		}
		f, err := r3.NewFrame(mid, p.Dir(tu, tv, 0), p.Dir(0, 0, 1))
		if err != nil {
			return nil, nil, nil, false, fmt.Errorf(`%w: a boundary segment has no direction`, decaderr.ErrDegenerate)
		}
		return bottomEdge, topEdge, surfacegeom.Plane{Frame: f}, false, nil
	}
}

// loopSides builds one loop's side faces with shared vertices and edges,
// returning the faces, the bottom and top cap coedges in walk order, and the
// loop's perimeter length (root buildLoopSidesAs with roleLoop li, holeLoop
// li != 0 and curve tokens minted).
func (b *builder) loopSides(ctx context.Context, li int, loop sectionrecord.LoopRecord) ([]*topo.Face, []topo.CoEdge, []topo.CoEdge, proofbound.BoundedScalar, error) {
	p := b.p
	holeLoop := li != 0
	if len(loop.Segments) == 0 {
		return nil, nil, nil, proofbound.BoundedScalar{}, fmt.Errorf(`%w: a recorded loop holds no segments`, decaderr.ErrDegenerate)
	}
	if !b.resolved.LoopMatches(li, loop) {
		return nil, nil, nil, proofbound.BoundedScalar{}, momentinput.ErrResolvedWalksMismatch
	}
	loopWalks := b.resolved.LoopWalks(li)
	delta := p.SectionDelta
	walkLenAllow := proofbound.SectionDisplacementLength(delta, 1)
	raw := make([]survey2d.SideWalk, len(loop.Segments))
	total := proofbound.BoundedScalar{}
	for i, seg := range loop.Segments {
		if err := ctx.Err(); err != nil {
			return nil, nil, nil, proofbound.BoundedScalar{}, err
		}
		if _, err := sectionrecord.NormalizeSegment(seg); err != nil {
			return nil, nil, nil, proofbound.BoundedScalar{}, err
		}
		w := loopWalks[i]
		w.LengthBound = proofbound.AbsSumUpper(w.LengthBound, walkLenAllow)
		raw[i] = survey2d.SideWalk{SegmentWalk: w, Segs: []int{i}}
		total = proofbound.BoundedAdd(total, proofbound.MeasuredScalar(w.Length, w.LengthBound))
	}
	walks, err := boundarywalk.CoalesceWalksContext(ctx, raw)
	if err != nil {
		return nil, nil, nil, proofbound.BoundedScalar{}, err
	}
	n := len(walks)
	singleClosed := n == 1 && walks[0].Closed
	height := proofbound.BoundedSub(p.Z1Scalar(), p.Z0Scalar())
	// rimVertex places one rim vertex at (u, v) on level z, charging the
	// section's displacement, that level's, the vertex's own exact lift
	// rounding and the walk-end term extra.
	rimVertex := func(u, v, z, zDelta, extra float64, level denotation.LevelToken) *topo.Vertex {
		held, lift := p.LiftedVertex(u, v, z)
		base := proofbound.AbsSumUpper(delta, zDelta, lift)
		vx := b.env.Alloc.NewVertex()
		vx.Position = held
		vx.Bound = units.Millimeters(proofbound.AbsSumUpper(base, extra))
		vx.Level = level
		vx.Denot = b.env.Doc.MintCurve()
		return vx
	}
	var bottomV, topV []*topo.Vertex
	var seamBottom, seamTop *topo.Vertex
	if singleClosed {
		w := walks[0]
		u, v, extra := boundarywalk.JunctionVertexAt(loop.Segments, w, w)
		seamBottom = rimVertex(u, v, p.Z0, p.Z0Delta, extra, b.levelZ0)
		seamTop = rimVertex(u, v, p.Z1, p.Z1Delta, extra, b.levelZ1)
	} else {
		bottomV = make([]*topo.Vertex, n)
		topV = make([]*topo.Vertex, n)
		for i, w := range walks {
			if err := ctx.Err(); err != nil {
				return nil, nil, nil, proofbound.BoundedScalar{}, err
			}
			prev := walks[(i+n-1)%n]
			u, v, extra := boundarywalk.JunctionVertexAt(loop.Segments, prev, w)
			bottomV[i] = rimVertex(u, v, p.Z0, p.Z0Delta, extra, b.levelZ0)
			topV[i] = rimVertex(u, v, p.Z1, p.Z1Delta, extra, b.levelZ1)
		}
	}

	// Vertical edges at junctions, shared between side face i−1 and i; a left
	// turn of the walk is a convex edge.
	var vertical []*topo.Edge
	if !singleClosed {
		vertical = make([]*topo.Edge, n)
		for i := range walks {
			if err := ctx.Err(); err != nil {
				return nil, nil, nil, proofbound.BoundedScalar{}, err
			}
			prev := walks[(i+n-1)%n]
			cross := prev.TanOutU*walks[i].TanInV - prev.TanOutV*walks[i].TanInU
			vertical[i] = b.newEdge(surfacegeom.Line3{}, bottomV[i], topV[i], cross > 0, p.Z1-p.Z0, height.Bound)
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
		convex, err := boundarywalk.RimConvexity(ctx, w, holeLoop, b.work)
		if err != nil {
			return nil, nil, nil, proofbound.BoundedScalar{}, err
		}
		bottomEdge, topEdge, surf, faceReversed, err := b.wallGeometry(w, convex, singleClosed, bStart, bEnd, tStart, tEnd)
		if err != nil {
			return nil, nil, nil, proofbound.BoundedScalar{}, err
		}
		bottomEdge.Level = b.levelZ0
		topEdge.Level = b.levelZ1
		bottomEdge.Denot = b.env.Doc.MintCurve()
		topEdge.Denot = b.env.Doc.MintCurve()

		origins, err := sideOriginsContext(ctx, b.env.Ref, li, w.Segs)
		if err != nil {
			return nil, nil, nil, proofbound.BoundedScalar{}, err
		}
		faceArea := proofbound.BoundedMul(proofbound.MeasuredScalar(w.Length, w.LengthBound), height)
		face := b.env.Alloc.NewFace()
		face.Surface = surf
		face.Origins = origins
		face.Body = b.body
		face.Area, face.AreaBound = faceArea.Value, faceArea.Bound
		face.Reversed = faceReversed
		if singleClosed {
			// A closed cylindrical band has TWO boundary loops, one per cap.
			face.Loops = []*topo.Loop{
				b.newLoop([]topo.CoEdge{{Edge: bottomEdge, Forward: true}}, true),
				b.newLoop([]topo.CoEdge{{Edge: topEdge, Forward: false}}, true),
			}
		} else {
			face.Loops = []*topo.Loop{b.newLoop([]topo.CoEdge{
				{Edge: bottomEdge, Forward: true},
				{Edge: vertical[(i+1)%n], Forward: true},
				{Edge: topEdge, Forward: false},
				{Edge: vertical[i], Forward: false},
			}, true)}
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
	return faces, bottomCo, topCo, total, nil
}

// sideOriginsContext mints the side roles of one wall (root
// sideOriginsContext; the name is kept because cancellation tests target it
// by frame).
func sideOriginsContext(ctx context.Context, ref provenance.Producer, roleLoop int, segs []int) ([]provenance.Ref, error) {
	origins := make([]provenance.Ref, len(segs))
	for oi, si := range segs {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		origins[oi] = provenance.Ref{Producer: ref, Role: fmt.Sprintf("side(%d,%d)", roleLoop, si)}
	}
	return origins, nil
}

// sheetLumps builds one Lump per connected piece of faces, each holding one
// Shell whose Open is read from its edges (root sheetLumps). It runs after
// every face is registered on its edges.
func sheetLumps(alloc topo.Alloc, faces []*topo.Face) []*topo.Lump {
	byEdge := map[*topo.Edge][]*topo.Face{}
	for _, f := range faces {
		for _, l := range f.Loops {
			for _, ce := range l.CoEdges {
				byEdge[ce.Edge] = append(byEdge[ce.Edge], f)
			}
		}
	}
	groups := surfacegroup.Connected(faces, byEdge)
	lumps := make([]*topo.Lump, len(groups))
	for i, g := range groups {
		s := alloc.NewShell()
		s.Faces = g
		s.Open = shellIsOpen(g)
		l := alloc.NewLump()
		l.Shells = []*topo.Shell{s}
		lumps[i] = l
	}
	return lumps
}

func shellIsOpen(faces []*topo.Face) bool {
	for _, f := range faces {
		for _, l := range f.Loops {
			for _, ce := range l.CoEdges {
				if len(ce.Edge.Faces) == 1 {
					return true
				}
			}
		}
	}
	return false
}

func exactnessOf(bound float64) measurement.Exactness {
	if bound == 0 {
		return measurement.Exact
	}
	return measurement.Approximate
}

// boundsContext is the axis-aligned box of the prism (root prismBoundsContext).
func (pp Payload) boundsContext(ctx context.Context, work *freeform.FreeformWork, walks *momentinput.ProfileWalks) (measurement.Box, error) {
	axialDelta := math.Max(pp.Z0Delta, pp.Z1Delta)
	low, high, bound, err := prismextent.Bounds(ctx, work, walks, pp.SectionDelta, axialDelta, pp.extentBoundedAlong)
	if err != nil {
		return measurement.Box{}, err
	}
	return measurement.Box{Min: low, Max: high, Exactness: exactnessOf(bound), Bound: units.Millimeters(bound)}, nil
}

// extentBoundedAlong passes the payload's placement coefficients to
// prismextent.ExtentAlong (root prismPayload.extentBoundedAlong).
func (pp Payload) extentBoundedAlong(ctx context.Context, g r3.Vec, work *freeform.FreeformWork, walks *momentinput.ProfileWalks) (float64, float64, float64, error) {
	base := pp.Xform.Apply(pp.Frame.Origin()).Dot(g)
	gu := pp.Dir(1, 0, 0).Dot(g)
	gv := pp.Dir(0, 1, 0).Dot(g)
	gz := pp.Dir(0, 0, 1).Dot(g)
	return prismextent.ExtentAlong(ctx, prismextent.ExtentInput{
		Profile: pp.Profile, Frame: pp.Frame, Transform: pp.Xform,
		Z0: pp.Z0, Z1: pp.Z1, Base: base, GU: gu, GV: gv, GZ: gz,
		Direction: g,
	}, work, walks, prismextent.ProfileBoundaryExtremes)
}

// chargePrismMap widens every reading a plane-coordinate build published to
// cover the image under the map the prism denotes through (root
// chargePrismMap and chargeBodyMap).
func chargePrismMap(body *topo.Body, frame r3.Frame, xform r3.Transform) error {
	l, err := massmoment.PrismRotation(frame, xform)
	if err != nil {
		return err
	}
	c, err := massmoment.MapChargeOf(l)
	if err != nil {
		return err
	}
	if c == (massmoment.MapCharge{}) {
		return nil
	}
	if body.Solid {
		v := c.VolumeOf(proofbound.MeasuredScalar(body.Volume.Value.Base(), body.Volume.Bound.Base()))
		body.Volume.Bound = units.CubicMillimeters(v.Bound)
		body.Volume.Exactness = exactnessOf(v.Bound)
	}
	a := c.AreaOf(proofbound.MeasuredScalar(body.Area.Value.Base(), body.Area.Bound.Base()))
	body.Area.Bound = units.SquareMillimeters(a.Bound)
	body.Area.Exactness = exactnessOf(a.Bound)
	for _, f := range body.Faces() {
		f.AreaBound = c.AreaOf(proofbound.MeasuredScalar(f.Area, f.AreaBound)).Bound
	}
	for _, e := range body.Edges() {
		e.LengthBound = c.LengthBound(e.Length, e.LengthBound)
	}
	return nil
}

// validateMeasurements is the last gate before a prism commits: every check is
// finiteness only (root validateAnalyticBodyMeasurements).
func validateMeasurements(body *topo.Body) error {
	finite := func(values ...float64) bool {
		for _, value := range values {
			if math.IsNaN(value) || math.IsInf(value, 0) {
				return false
			}
		}
		return true
	}
	checks := []struct {
		name string
		ok   bool
	}{
		{name: "volume", ok: finite(body.Volume.Value.Base(), body.Volume.Bound.Base())},
		{name: "area", ok: finite(body.Area.Value.Base(), body.Area.Bound.Base())},
		{name: "centroid", ok: finite(body.Centroid.Value.X, body.Centroid.Value.Y, body.Centroid.Value.Z, body.Centroid.Bound.Base())},
		{name: "bounds", ok: finite(body.Bounds.Min.X, body.Bounds.Min.Y, body.Bounds.Min.Z,
			body.Bounds.Max.X, body.Bounds.Max.Y, body.Bounds.Max.Z, body.Bounds.Bound.Base())},
	}
	for _, check := range checks {
		if !check.ok {
			return fmt.Errorf(`%w: the analytic body's %s measurement is not finite`, decaderr.ErrNotFinite, check.name)
		}
	}
	return nil
}
