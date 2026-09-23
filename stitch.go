package decad

import (
	"context"
	"fmt"
	"math"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// This file is Stitch of docs/surface-design.md §6: the one operation that
// turns a boundary into a solid. stitch_weld.go owns Table J's admission
// (which free edges join); this file owns the public entry point, the fresh
// topology rebuild over the weld plan, the derived orientation, the closure
// and manifold leg, Table C's outcome, and the measurements a stitched body
// publishes.
//
// evalStitchContext is the whole evaluator and is shared, unchanged, by the
// initial build and by every later Placed/Duplicate/PlacedCopy
// re-evaluation: the payload records the operand faces and the weld plan,
// never a built topology, and placed() replays this same function under a
// composed transform (docs/surface-design.md's "recorded weld" decision).
// Table J's admission is never re-run on a placed (rounded) coordinate; the
// crossing audit IS re-run, because a rigid motion can make two placed
// triangles meet that the unplaced ones did not.

// Stitch calls [StitchContext] with [context.Background].
func Stitch(bodies ...*Body) (*Body, error) {
	return StitchContext(context.Background(), bodies...)
}

// StitchContext welds every free-edge pair it can prove coincident
// (docs/surface-design.md §6.2's Table J) across bodies, assembles the
// result, and returns one new body, retiring every operand. One operand is
// meaningful: it re-audits that body's own boundary, which is how a closed
// sheet becomes a solid (§2.1, §4.1).
//
// Every operand must carry an evaluator payload and be a live [BodySheet] of
// this call's document: a nil or payload-less operand is [ErrDegenerate]
// (Table R row R12), an operand owned by a different document is
// [ErrForeignBody] (R13), a retired operand is [ErrRetiredBody] (R14), and a
// [BodySolid] operand is [ErrUnsupported] (R17) — it has no free edge, so
// stitching cannot change it, and a caller combining two solids means
// [Union]. Table C decides the result: a residual free edge is a
// [BodySheet] and never an error; a fully welded, all-planar, non-crossing
// boundary is a [BodySolid]; a fully welded boundary holding a curved face is
// [ErrUnsupported] (R8); a welded set with no consistent orientation is
// [ErrDegenerate] (R7); and a proven self-contact or an exhausted audit
// budget surfaces the crossing audit's own [ErrDegenerate]/[ErrUnsupported]
// (R9/R10) unchanged. A failed call leaves the document and every operand
// unchanged.
func StitchContext(ctx context.Context, bodies ...*Body) (*Body, error) {
	if ctx == nil {
		return nil, fmt.Errorf(`%w: a nil context cannot control a stitch`, ErrDegenerate)
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	if len(bodies) == 0 {
		return nil, fmt.Errorf(`%w: Stitch requires at least one body`, ErrDegenerate)
	}
	for _, b := range bodies {
		if b == nil || b.payload == nil {
			return nil, fmt.Errorf(`%w: Stitch requires every operand to carry an evaluator payload`, ErrDegenerate)
		}
	}
	d := bodies[0].doc
	for _, b := range bodies {
		if err := d.requireLive(b); err != nil {
			return nil, err
		}
	}
	for _, b := range bodies {
		if b.Kind() == BodySolid {
			return nil, fmt.Errorf(`%w: Stitch does not accept a solid body operand (docs/surface-design.md Table R row R17)`, ErrUnsupported)
		}
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}

	var srcFaces []*Face
	for _, b := range bodies {
		srcFaces = append(srcFaces, b.Faces()...)
	}
	plan := buildStitchWeldPlan(srcFaces)

	ref := d.nextProducerID()
	body, err := evalStitchContext(ctx, d, ref, srcFaces, plan, r3.Identity())
	if err != nil {
		return nil, err
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	d.commit(body, bodies...)
	return body, nil
}

// stitchPayload is Stitch's own record. It carries the operand faces
// (readable even after the operands retire — a Body is never mutated or
// freed on retirement, core §6) and the weld plan buildStitchWeldPlan
// derived from them, never a previously built topology: placed() replays
// evalStitchContext from these same two values under the composed
// transform, so the admission Table J proved is REPLAYED, never re-derived,
// while the crossing audit and the global orientation sign are re-decided
// fresh from the placed triangle set every time, exactly as a rigid motion
// can change which pairs of placed triangles meet.
type stitchPayload struct {
	xform r3.Transform
	// delta is the proven displacement rigidRoundAllow charges against the
	// placed shared vertex table — zero exactly when xform is the identity
	// transform, an exact struct comparison (docs/loft-design.md §5's
	// identical fast path).
	delta float64
	faces []*Face
	plan  *stitchWeldPlan
	// verts is the shared vertex table actually built for THIS body, in
	// world coordinates — verify_gate.go's own diameter arm reads it back,
	// modelled on loftPayload's identical field.
	verts []r3.Vec
	// vertBound is verts' own per-vertex proven bound (millimetres), the
	// same widened class bound rebuildStitchTopology's vertexForClass
	// stamps onto the live Vertex it builds for that class — read back here
	// so tessellate_stitch.go can publish a per-face bound without
	// recomputing it. Populated only alongside tris/triFaces below.
	vertBound []float64
	// tris is the final outward-wound triangle set (indices into verts) a
	// CLOSED, all-planar (stitchAllTetrahedronEligible) build assembled and
	// audited — the exact set tessellate_stitch.go restates with no
	// chording. Nil for a curved, mixed, or OPEN stitched body: none of
	// those has a triangle set to restate.
	tris [][3]int
	// triFaces is tris' own per-triangle LIVE rebuilt face, parallel to
	// tris. It is never stitchPayload.faces (the retired operand faces) and
	// never attributed by role, since rebuildStitchTopology copies a welded
	// operand's origins verbatim and two welded operands can share one role
	// string.
	triFaces []*Face
	// auditClean records whether the crossing audit ran and passed for this
	// body: true for a solid (Table C requires it) and for an open sheet
	// whose audit was also run and admitted (the open-case decision,
	// docs/surface-design.md §9.1's sheet validity leg 4). It is false for a
	// non-planar body (no audit runs at all) and for an open sheet whose
	// audit declined without erroring.
	auditClean bool
}

func (sp stitchPayload) transform() r3.Transform { return sp.xform }

func (sp stitchPayload) placed(ctx context.Context, d *Document, ref producerID, composed r3.Transform) (*Body, error) {
	return evalStitchContext(ctx, d, ref, sp.faces, sp.plan, composed)
}

// evalStitchContext is Stitch's whole evaluator, shared by the initial build
// and every later placement. It places the shared vertex table exactly once
// (never per operand), rebuilds fresh topology over the weld plan, derives
// one consistent orientation, checks closure and manifoldness, decides
// Table C, and publishes the resulting body's measurements.
func evalStitchContext(ctx context.Context, d *Document, ref producerID, srcFaces []*Face, plan *stitchWeldPlan, xform r3.Transform) (*Body, error) {
	if err := ctx.Err(); err != nil {
		return nil, err
	}

	// The single held vertex table, placed under the composed motion ONCE —
	// never per operand (docs/surface-design.md's "recorded weld" decision).
	maxInputAbs := 0.0
	for _, p := range plan.table.verts {
		maxInputAbs = max(maxInputAbs, vecMaxAbs(p))
	}
	verts := make([]r3.Vec, len(plan.table.verts))
	for i, p := range plan.table.verts {
		v := xform.Apply(p)
		if !finiteVec(v) {
			return nil, fmt.Errorf(`%w: a placed stitch vertex is not representable`, ErrUnsupported)
		}
		verts[i] = v
	}
	delta := 0.0
	if xform != r3.Identity() {
		delta = rigidRoundAllow(maxInputAbs, vecMaxAbs(xform.Translation()))
	}
	// massDelta is what the mass accumulator charges instead of delta alone:
	// a welded vertex the shared-denotation certificate admitted (J5's
	// bounded-pair lift, docs/surface-design.md §6.2's amendment) is no
	// longer zero-bound, so the triangle set the tetrahedron sum runs over is
	// no longer the body's own true vertices — it is only within
	// maxClassBound of them. sweptVolumeAllow already bounds exactly this
	// shape of error (a uniform per-vertex displacement), so folding the
	// widest class bound into the same delta the placement rounding uses is
	// a reuse, not new proof machinery (§6.4's amendment: a bounded weld's
	// volume is never Exact). It stays equal to delta whenever every welded
	// class is zero-bound, which is every case this evaluator admitted
	// before this increment.
	maxClassBound := 0.0
	for _, b := range plan.table.boundByClass {
		maxClassBound = max(maxClassBound, b)
	}
	massDelta := delta
	if maxClassBound > 0 {
		massDelta = absSumUpper(delta, maxClassBound)
	}

	newFaces, classOf, welded, err := rebuildStitchTopology(ctx, plan, xform, verts, delta, srcFaces)
	if err != nil {
		return nil, err
	}
	// vertBound reads back, per vertex CLASS index (the same index tris and
	// verts share), the widened bound rebuildStitchTopology's vertexForClass
	// already stamped onto the live Vertex it built for that class —
	// tessellate_stitch.go's own per-face bound is the largest of these over
	// the vertices a face's triangles touch, so this is a read of an
	// existing proof, never a second one.
	vertBound := make([]float64, len(verts))
	for v, class := range classOf {
		vertBound[class] = v.bound.Base()
	}
	if err := attachFaceLoopsContext(ctx, newFaces); err != nil {
		return nil, err
	}
	if err := deriveStitchOrientation(newFaces); err != nil {
		return nil, err
	}
	if err := checkStitchClosure(newFaces); err != nil {
		return nil, err
	}

	open := stitchHasFreeEdge(newFaces)
	allTetra := stitchAllTetrahedronEligible(newFaces)

	kind, solid := BodySheet, false
	auditClean := false
	var tris [][3]int
	var triFaces []*Face
	var acc *loftMassAccumulator
	var curvedVolume Measurement
	var curvedCentroid VecMeasurement
	haveCurved := false

	switch {
	case allTetra:
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		tris, triFaces, err = triangulateStitchFaces(ctx, newFaces, classOf)
		if err != nil {
			return nil, err
		}

		budget := newWorkBudget(ctx)
		auditErr := loftCrossingAudit(budget, verts, tris)
		switch {
		case auditErr != nil && !open:
			// Closed plus a refusing audit surfaces R9/R10 unchanged — the
			// decision at docs/surface-design.md §6.3/§6.4.
			return nil, auditErr
		case auditErr == nil:
			auditClean = true
		}
		// auditErr != nil && open: leg 4 stays undecided and no error is
		// raised — the open-case decision preserving §6.3's rule that a
		// residual free edge is never an error.

		if !open {
			acc = newLoftMassAccumulator(verts[0], massDelta, 0, 0)
			for _, t := range tris {
				acc.add(verts[t[0]], verts[t[1]], verts[t[2]], false)
			}
			if acc.vol6.Sign() < 0 {
				// §5's whole-shell orientation step, re-decided fresh from
				// this placed triangle set every time: the derived
				// combinatorial orientation is consistent either way it
				// lands, so a negative signed volume means the whole
				// assembly is the globally reversed (inward) choice, fixed
				// by reversing every face and re-triangulating.
				for _, f := range newFaces {
					reverseFaceOrientation(f)
				}
				tris, triFaces, err = triangulateStitchFaces(ctx, newFaces, classOf)
				if err != nil {
					return nil, err
				}
				acc = newLoftMassAccumulator(verts[0], massDelta, 0, 0)
				for _, t := range tris {
					acc.add(verts[t[0]], verts[t[1]], verts[t[2]], false)
				}
			}
			solid, kind = true, BodySolid
		} else if delta > 0 {
			// This accumulator instance only ever feeds perturbAreaSum below
			// (open means solid stays false, so body.volume/centroid never
			// read it): Area needs no massDelta charge of its own — each
			// face's own areaBound already covers a bounded weld
			// (docs/surface-design.md §6.4's amendment) — so this stays the
			// placement's own delta, unwidened.
			acc = newLoftMassAccumulator(verts[0], delta, 0, 0)
			for _, t := range tris {
				acc.add(verts[t[0]], verts[t[1]], verts[t[2]], false)
			}
		}

		// The welded edges' own convexity is a genuine dihedral-angle
		// question once two faces from different operands share a seam
		// (topology.go's Edge.IsConvex doc: "a JUNCTION edge... is also the
		// material angle"), computed here — AFTER any global sign fix above
		// — from the two adjacent faces' own outward normals rather than
		// carried over from either original operand's now-meaningless walk.
		fixWeldedEdgeConvexity(welded)

	case !open:
		// Table C's curved-closure arm (docs/surface-design.md §6.4): every
		// edge is welded and at least one face is not tetrahedron-eligible,
		// so the volume needs stitch_flux.go's per-surface flux integral
		// rather than the exact triangle sum. Rule S gates admission on the
		// single source body's own construction proof; the vertex-link audit
		// gates the curved path's own claim of manifoldness, since the
		// reused crossing audit consumes triangles this set has none of.
		if !stitchRuleSAdmits(ctx, srcFaces) {
			return nil, fmt.Errorf(`%w: Stitch closes a boundary this evaluator cannot prove simple by construction (docs/surface-design.md Table R row R8)`, ErrUnsupported)
		}
		if err := auditVertexLinksForStitchFaces(ctx, newFaces); err != nil {
			return nil, err
		}
		// verts[0] anchors the flux sum exactly as newLoftMassAccumulator
		// anchors the tetrahedron sum — but a fully boundary-less analytic
		// face (a complete Sphere or Torus, which mints no edge at all) can
		// leave the shared vertex table empty, and every admitted arm this
		// increment lands (Plane, Cylinder) is bounded by at least one full
		// circle and so always has a vertex. An empty table therefore means
		// the flux path is about to refuse on the sealed switch's own
		// default arm regardless of the anchor's value, so a zero anchor is
		// safe rather than a special-cased refusal here.
		anchor := r3.Vec{}
		if len(verts) > 0 {
			anchor = verts[0]
		}
		vol, cen, err := stitchCurvedMass(ctx, newFaces, anchor, delta)
		if err != nil {
			return nil, err
		}
		curvedVolume, curvedCentroid, haveCurved = vol, cen, true
		solid, kind = true, BodySolid

	// open && !allTetra: an open curved (or mixed) sheet earns no volume
	// attempt at all — Table C's first row, a residual free edge, and
	// never an error (docs/surface-design.md §6.3).
	default:
	}

	body := &Body{doc: d, origin: FeatureRef{producer: ref, Role: roleBody}, solid: solid, kind: kind}
	for _, f := range newFaces {
		f.body = body
	}
	body.lumps = sheetLumps(newFaces)

	// Area is the sum of the constituent faces' own area/areaBound through
	// boundedAdd (correction 2, docs/surface-design.md §6.4/§8): never a
	// triangle-sum reading, since a general planar triangle's area is a
	// square root of a rational and so is never Exact the way a face's own
	// analytically integrated area already is.
	areaAcc := boundedScalar{}
	for _, f := range newFaces {
		areaAcc = boundedAdd(areaAcc, measuredScalar(f.area, f.areaBound))
	}
	if acc != nil && delta > 0 {
		areaAcc.bound = absSumUpper(areaAcc.bound, acc.perturbAreaSum)
	}
	body.area = Measurement{
		Value:     units.SquareMillimeters(areaAcc.value),
		Exactness: exactnessOf(areaAcc.bound),
		Bound:     units.SquareMillimeters(areaAcc.bound),
	}

	bounds, err := stitchBounds(srcFaces, xform, delta)
	if err != nil {
		return nil, err
	}
	body.bounds = bounds

	if solid {
		switch {
		case acc != nil:
			body.volume = acc.volume(verts, tris)
			cen, err := acc.centroid(verts, tris)
			if err != nil {
				return nil, err
			}
			body.centroid = cen
		case haveCurved:
			body.volume = curvedVolume
			body.centroid = curvedCentroid
		}
	}

	if err := validateAnalyticBodyMeasurements(body); err != nil {
		return nil, err
	}

	// tessellate_stitch.go's exact restatement is scoped to the CLOSED,
	// all-planar case only (docs/surface-design.md §14 Table D row 5): the
	// curved-closure arm's own faces have no triangle set to restate, and an
	// OPEN all-planar sheet's mesh is a later increment even though this
	// arm did triangulate it to check its own perturbed area sum above.
	payloadTris, payloadTriFaces, payloadVertBound := tris, triFaces, vertBound
	if !allTetra || open {
		payloadTris, payloadTriFaces, payloadVertBound = nil, nil, nil
	}
	body.payload = stitchPayload{
		xform:      xform,
		delta:      delta,
		faces:      srcFaces,
		plan:       plan,
		verts:      verts,
		vertBound:  payloadVertBound,
		tris:       payloadTris,
		triFaces:   payloadTriFaces,
		auditClean: auditClean,
	}
	return body, nil
}

// rebuildStitchTopology deep-copies every operand face's surface, origins,
// area, area bound and flags into a fresh Face, over fresh vertices and
// edges built from the weld plan: a welded pair of free edges (plan.group)
// shares ONE new edge across both faces, and every other edge gets its own,
// shared across whichever coedges of the SAME operand already pointed at
// it. Reusing an operand face outright would corrupt the retired operand
// through its body back-pointer and populated edge list (docs/surface-design.md
// §6, correction 1's construction note), so nothing here aliases an operand's
// own Face, Loop, CoEdge, Edge or Vertex.
func rebuildStitchTopology(ctx context.Context, plan *stitchWeldPlan, xform r3.Transform, verts []r3.Vec, delta float64, srcFaces []*Face) ([]*Face, map[*Vertex]int, map[*Edge]struct{}, error) {
	newVertByClass := map[int]*Vertex{}
	classOf := map[*Vertex]int{}
	// vertexForClass restates the class's own curve token (denotByClass,
	// stitch_weld.go — the zero value for a class no member carries one for)
	// under xform, composing rather than overwriting so a token nested
	// through more than one placement still states the true accumulated
	// motion (denotation.go's curveToken.compose).
	vertexForClass := func(class int) *Vertex {
		if nv, ok := newVertByClass[class]; ok {
			return nv
		}
		bound := plan.table.boundByClass[class]
		if delta > 0 {
			bound = absSumUpper(bound, delta)
		}
		nv := &Vertex{position: verts[class], bound: units.Millimeters(bound), denot: plan.table.denotByClass[class].compose(xform)}
		newVertByClass[class] = nv
		classOf[nv] = class
		return nv
	}

	type edgeBuild struct {
		edge                 *Edge
		startClass, endClass int
	}
	buildByOld := map[*Edge]*edgeBuild{}
	buildByGroup := map[int]*edgeBuild{}
	welded := map[*Edge]struct{}{}

	edgeFor := func(old *Edge) (*edgeBuild, error) {
		if eb, ok := buildByOld[old]; ok {
			return eb, nil
		}
		startClass := plan.table.classOf(old.start)
		endClass := plan.table.classOf(old.end)
		gid, isWelded := plan.group[old]
		if isWelded {
			if eb, ok := buildByGroup[gid]; ok {
				buildByOld[old] = eb
				return eb, nil
			}
		}
		curve, err := transformCurve(old.curve, xform)
		if err != nil {
			return nil, err
		}
		lengthBound := old.lengthBound
		if delta > 0 {
			lengthBound = absSumUpper(lengthBound, delta)
		}
		ne := &Edge{
			curve:           curve,
			start:           vertexForClass(startClass),
			end:             vertexForClass(endClass),
			convex:          old.convex,
			length:          old.length,
			lengthBound:     lengthBound,
			lengthUnbounded: old.lengthUnbounded,
			// A welded pair's two edges are already proven to denote the same
			// curve (Table J), so the FIRST one reached here — the only one
			// that ever builds a new Edge for the group, per buildByGroup
			// above — is a sound representative for the other's identity too.
			denot: old.denot.compose(xform),
		}
		eb := &edgeBuild{edge: ne, startClass: startClass, endClass: endClass}
		buildByOld[old] = eb
		if isWelded {
			buildByGroup[gid] = eb
			welded[ne] = struct{}{}
		}
		return eb, nil
	}

	newFaces := make([]*Face, len(srcFaces))
	for i, f := range srcFaces {
		if err := ctx.Err(); err != nil {
			return nil, nil, nil, err
		}
		surface, err := transformSurface(f.surface, xform)
		if err != nil {
			return nil, nil, nil, err
		}
		nf := &Face{
			surface:    surface,
			origins:    append([]FeatureRef(nil), f.origins...),
			area:       f.area,
			areaBound:  f.areaBound,
			reversed:   f.reversed,
			heldPlanar: f.heldPlanar,
		}
		for _, l := range f.loops {
			coedges := make([]coedge, len(l.coedges))
			for j, ce := range l.coedges {
				eb, err := edgeFor(ce.edge)
				if err != nil {
					return nil, nil, nil, err
				}
				// The old edge's own (start, end) may run either the same
				// way as the canonical build's or opposite to it — a weld
				// pairs two free edges whose two operands typically walked
				// their shared rim in opposite senses — so the coedge's
				// forward flag is restated against the CANONICAL edge's own
				// direction rather than copied verbatim.
				oldStartClass := plan.table.classOf(ce.edge.start)
				forward := ce.forward
				if oldStartClass != eb.startClass {
					forward = !forward
				}
				coedges[j] = coedge{edge: eb.edge, forward: forward}
			}
			nf.loops = append(nf.loops, &Loop{coedges: coedges, outer: l.outer})
		}
		newFaces[i] = nf
	}
	return newFaces, classOf, welded, nil
}

// transformSurface transforms every exported vector a Surface variant
// carries under a rigid motion, without re-deriving the surface from any
// record: Stitch's payload holds already-built geometry, so a placement
// transforms that geometry's own descriptive vectors directly (r3.Transform's
// Apply for a position, ApplyDir for a direction — CLAUDE.md's rule against
// hand-rolled coordinate math). NURBSSurface and Faceted are opaque markers
// carrying no exported geometry of their own and pass through unchanged.
func transformSurface(s Surface, xf r3.Transform) (Surface, error) {
	switch v := s.(type) {
	case Plane:
		origin := xf.Apply(v.Frame.Origin())
		u := xf.ApplyDir(v.Frame.U())
		vv := xf.ApplyDir(v.Frame.V())
		f, err := r3.NewFrame(origin, u, vv)
		if err != nil {
			return nil, fmt.Errorf(`%w: a placed stitch face's plane is degenerate: %s`, ErrUnsupported, err)
		}
		return Plane{Frame: f}, nil
	case Cylinder:
		return Cylinder{Origin: xf.Apply(v.Origin), Axis: xf.ApplyDir(v.Axis), Radius: v.Radius}, nil
	case Cone:
		return Cone{Origin: xf.Apply(v.Origin), Axis: xf.ApplyDir(v.Axis), Radius: v.Radius, HalfAngle: v.HalfAngle}, nil
	case Sphere:
		return Sphere{Center: xf.Apply(v.Center), Radius: v.Radius}, nil
	case Torus:
		return Torus{Center: xf.Apply(v.Center), Axis: xf.ApplyDir(v.Axis), Major: v.Major, Minor: v.Minor}, nil
	case NURBSSurface:
		return v, nil
	case Faceted:
		return v, nil
	default:
		return nil, fmt.Errorf(`%w: Stitch cannot transform surface kind %T`, ErrUnsupported, s)
	}
}

// transformCurve is transformSurface's one-dimensional analog for Curve.
func transformCurve(c Curve, xf r3.Transform) (Curve, error) {
	switch v := c.(type) {
	case Line3:
		return Line3{}, nil
	case Circle3:
		return Circle3{Center: xf.Apply(v.Center), Axis: xf.ApplyDir(v.Axis), Radius: v.Radius}, nil
	case Arc3:
		return Arc3{Center: xf.Apply(v.Center), Axis: xf.ApplyDir(v.Axis), Radius: v.Radius}, nil
	case NURBSCurve:
		return v, nil
	case FacetedCurve:
		return v, nil
	default:
		return nil, fmt.Errorf(`%w: Stitch cannot transform curve kind %T`, ErrUnsupported, c)
	}
}

// coedgeDirectionFor returns f's own forward flag for its use of e, and
// whether f actually uses e at all.
func coedgeDirectionFor(f *Face, e *Edge) (bool, bool) {
	for _, l := range f.loops {
		for _, ce := range l.coedges {
			if ce.edge == e {
				return ce.forward, true
			}
		}
	}
	return false, false
}

// reverseFaceOrientation reverses f's whole boundary walk in place: every
// loop's coedges are reordered back to front with their forward flag
// negated, and f.reversed toggles with it — the outward side flips together
// with the walk that names it.
func reverseFaceOrientation(f *Face) {
	for _, l := range f.loops {
		n := len(l.coedges)
		rev := make([]coedge, n)
		for i, ce := range l.coedges {
			rev[n-1-i] = coedge{edge: ce.edge, forward: !ce.forward}
		}
		l.coedges = rev
	}
	f.reversed = !f.reversed
}

// deriveStitchOrientation derives one consistent orientation across the
// welded set by walking every two-face edge and requiring the two adjacent
// faces to traverse it in opposite senses, flipping a whole face (never a
// single coedge) whenever its component's arbitrary root choice disagrees
// with it. A face reached twice with contradicting flips proves the
// component non-orientable — [ErrDegenerate] (Table R row R7); the
// three-triangle Möbius fixture in stitch_test.go exercises exactly this
// path directly, on a hand-built face set, because Stitch's own public
// gates admit no way to reach a non-orientable assembly through the seam.
func deriveStitchOrientation(faces []*Face) error {
	flip := map[*Face]bool{}
	for _, root := range faces {
		if _, ok := flip[root]; ok {
			continue
		}
		flip[root] = false
		queue := []*Face{root}
		for len(queue) > 0 {
			f := queue[0]
			queue = queue[1:]
			for _, l := range f.loops {
				for _, ce := range l.coedges {
					e := ce.edge
					if len(e.faces) != 2 {
						continue
					}
					other := e.faces[0]
					if other == f {
						other = e.faces[1]
					}
					if other == f {
						continue
					}
					dirOther, ok := coedgeDirectionFor(other, e)
					if !ok {
						continue
					}
					effectiveF := ce.forward != flip[f]
					wantOther := !effectiveF
					flipOther := dirOther != wantOther
					if existing, seen := flip[other]; seen {
						if existing != flipOther {
							return fmt.Errorf(`%w: Stitch's welded set cannot be consistently oriented (docs/surface-design.md Table R row R7)`, ErrDegenerate)
						}
						continue
					}
					flip[other] = flipOther
					queue = append(queue, other)
				}
			}
		}
	}
	for f, fl := range flip {
		if fl {
			reverseFaceOrientation(f)
		}
	}
	return nil
}

// checkStitchClosure is docs/surface-design.md §6.4's closure and manifold
// leg, the explicit directed-edge parity leg the reused loftCrossingAudit
// alone does not run (correction 1): every edge must be adjacent to one or
// two faces, and a two-face edge must be traversed by exactly one forward
// and one backward coedge, with a coedge-use count that disagrees with the
// edge's own adjacent-face count also a violation. One face is an open
// boundary — Table C's first row, and never an error here. Three or more,
// or a disagreeing use count, is [ErrDegenerate] (Table R row R7): three
// triangles sharing one edge would pass every pairwise contact test the
// crossing audit alone runs, which is exactly the gap this leg closes.
func checkStitchClosure(faces []*Face) error {
	uses := map[*Edge]int{}
	forward := map[*Edge]int{}
	backward := map[*Edge]int{}
	var order []*Edge
	seen := map[*Edge]struct{}{}
	for _, f := range faces {
		for _, l := range f.loops {
			for _, ce := range l.coedges {
				e := ce.edge
				if _, ok := seen[e]; !ok {
					seen[e] = struct{}{}
					order = append(order, e)
				}
				uses[e]++
				if ce.forward {
					forward[e]++
				} else {
					backward[e]++
				}
			}
		}
	}
	for _, e := range order {
		switch len(e.faces) {
		case 1:
			if uses[e] != 1 {
				return fmt.Errorf(`%w: a stitched edge's coedge-use count disagrees with its one adjacent face (docs/surface-design.md Table R row R7)`, ErrDegenerate)
			}
		case 2:
			if uses[e] != 2 || forward[e] != 1 || backward[e] != 1 {
				return fmt.Errorf(`%w: a stitched edge is not traversed by exactly one forward and one backward coedge (docs/surface-design.md Table R row R7)`, ErrDegenerate)
			}
		default:
			return fmt.Errorf(`%w: a stitched edge is adjacent to more than two faces (docs/surface-design.md Table R row R7)`, ErrDegenerate)
		}
	}
	return nil
}

func stitchHasFreeEdge(faces []*Face) bool {
	for _, f := range faces {
		for _, l := range f.loops {
			for _, ce := range l.coedges {
				if ce.edge.IsFree() {
					return true
				}
			}
		}
	}
	return false
}

// triangulateStitchFaces triangulates each planar face in its own plane
// frame through triangulate.go's existing cap triangulator and maps every
// resulting index back through the shared vertex table (docs/surface-design.md
// §6.4): ear clipping mints no new point, so no coordinate enters that the
// body does not already hold, and the plane-local projection may round the
// CHOICE of triangulation but the crossing audit that follows runs on the
// exact 3D coordinates the table already carries — never on a
// re-approximated one.
//
// It returns the LIVE face each returned triangle belongs to, parallel to
// the triangle set, so a caller that must attribute a triangle to its face
// (tessellate_stitch.go) reads it back directly rather than reconstructing
// the mapping afterward — stitchPayload.faces holds only the retired
// operand faces, and a stitched body's faces carry no per-body-unique role
// to recover it from either (docs/tessellation-design.md §4).
func triangulateStitchFaces(ctx context.Context, faces []*Face, classOf map[*Vertex]int) ([][3]int, []*Face, error) {
	var tris [][3]int
	var triFaces []*Face
	for _, f := range faces {
		if err := ctx.Err(); err != nil {
			return nil, nil, err
		}
		pl, ok := f.surface.(Plane)
		if !ok {
			return nil, nil, fmt.Errorf(`%w: Stitch cannot triangulate a non-planar face`, ErrUnsupported)
		}
		frame := pl.Frame
		var pts []Point2
		var verts []*Vertex
		loopIdx := make([][]int, len(f.loops))
		for li, l := range f.loops {
			idxs := make([]int, len(l.coedges))
			for i, ce := range l.coedges {
				v := ce.Start()
				local := frame.ToLocal(v.Position().Value)
				idxs[i] = len(pts)
				pts = append(pts, Point2{U: local.X, V: local.Y})
				verts = append(verts, v)
			}
			loopIdx[li] = idxs
		}
		for li, l := range f.loops {
			wantCCW := l.outer
			isCCW := shoelace(pts, loopIdx[li]) > 0
			if wantCCW != isCCW {
				reverseIntSlice(loopIdx[li])
			}
		}
		tris2D, err := triangulate2DContext(ctx, pts, loopIdx)
		if err != nil {
			return nil, nil, err
		}
		for _, t := range tris2D {
			a, b, c := classOf[verts[t[0]]], classOf[verts[t[1]]], classOf[verts[t[2]]]
			if f.reversed {
				b, c = c, b
			}
			tris = append(tris, [3]int{a, b, c})
			triFaces = append(triFaces, f)
		}
	}
	return tris, triFaces, nil
}

// shoelace returns twice the signed area of the polygon idx walks over pts —
// its sign alone is what triangulateStitchFaces reads, to decide whether a
// loop's stored walk is counter-clockwise in the face's own local frame.
func shoelace(pts []Point2, idx []int) float64 {
	n := len(idx)
	sum := 0.0
	for i := range n {
		a := pts[idx[i]]
		b := pts[idx[(i+1)%n]]
		sum += a.U*b.V - b.U*a.V
	}
	return sum
}

func reverseIntSlice(s []int) {
	for i, j := 0, len(s)-1; i < j; i, j = i+1, j-1 {
		s[i], s[j] = s[j], s[i]
	}
}

// fixWeldedEdgeConvexity computes IsConvex for every newly welded edge from
// its two adjacent faces' own outward normals, rather than carrying over
// either original operand's now-meaningless walk-based flag: a welded edge
// is a genuine interior junction between two independently built faces, and
// topology.go's own doc comment on Edge.IsConvex states that a junction
// edge's convexity IS the material dihedral angle. Both adjacent faces are
// guaranteed Plane: only a Line3 rim edge is ever welded (stitch_weld.go),
// and only a planar face's straight boundary segment ever records one. It
// must run AFTER any global orientation-sign fix, since that fix can invert
// which side either face calls outward.
func fixWeldedEdgeConvexity(welded map[*Edge]struct{}) {
	for e := range welded {
		if len(e.faces) != 2 {
			continue
		}
		var fwd, bwd *Face
		for _, f := range e.faces {
			dir, ok := coedgeDirectionFor(f, e)
			if !ok {
				continue
			}
			if dir {
				fwd = f
			} else {
				bwd = f
			}
		}
		if fwd == nil || bwd == nil {
			continue
		}
		pa, okA := fwd.surface.(Plane)
		pb, okB := bwd.surface.(Plane)
		if !okA || !okB {
			continue
		}
		na := pa.Frame.N()
		if fwd.reversed {
			na = na.Scale(-1)
		}
		nb := pb.Frame.N()
		if bwd.reversed {
			nb = nb.Scale(-1)
		}
		d := e.end.position.Sub(e.start.position)
		e.convex = na.Cross(nb).Dot(d) > 0
	}
}

// stitchOperandBodies returns the distinct original operand bodies srcFaces
// were read from, in first-seen order.
func stitchOperandBodies(srcFaces []*Face) []*Body {
	seen := map[*Body]struct{}{}
	var bodies []*Body
	for _, f := range srcFaces {
		if f.body == nil {
			continue
		}
		if _, ok := seen[f.body]; ok {
			continue
		}
		seen[f.body] = struct{}{}
		bodies = append(bodies, f.body)
	}
	return bodies
}

// stitchBounds publishes the assembled body's box as the union of every
// operand's own already-proven Bounds, each inflated by its own Bound before
// the union so no per-operand slack has to be tracked separately, then
// carried through xform by its 8 corners — a valid enclosing box under any
// rigid motion, since the image of a box's convex hull under an affine map is
// the convex hull of the transformed corners. This is sound for a curved
// operand exactly as it is for a planar one, because it reuses each
// operand's OWN extent proof rather than re-deriving one from vertices alone,
// which would understate a curved wall's true box. The published Bound is
// exactly delta: every corner is exact once inflated and transformed, so the
// only remaining uncertainty is the placement's own rounding.
func stitchBounds(srcFaces []*Face, xform r3.Transform, delta float64) (Box, error) {
	bodies := stitchOperandBodies(srcFaces)
	if len(bodies) == 0 {
		return Box{}, fmt.Errorf(`%w: Stitch has no operand geometry to bound`, ErrDegenerate)
	}
	have := false
	var lo, hi r3.Vec
	for _, b := range bodies {
		box := b.bounds
		inflate := box.Bound.Base()
		for _, c := range stitchBoxCorners(box.Min, box.Max, inflate) {
			p := xform.Apply(c)
			if !finiteVec(p) {
				return Box{}, fmt.Errorf(`%w: a placed stitch bound is not representable`, ErrUnsupported)
			}
			if !have {
				lo, hi = p, p
				have = true
				continue
			}
			lo = r3.Vec{X: math.Min(lo.X, p.X), Y: math.Min(lo.Y, p.Y), Z: math.Min(lo.Z, p.Z)}
			hi = r3.Vec{X: math.Max(hi.X, p.X), Y: math.Max(hi.Y, p.Y), Z: math.Max(hi.Z, p.Z)}
		}
	}
	return Box{Min: lo, Max: hi, Exactness: exactnessOf(delta), Bound: units.Millimeters(delta)}, nil
}

func stitchBoxCorners(boxMin, boxMax r3.Vec, inflate float64) [8]r3.Vec {
	lo := r3.Vec{X: boxMin.X - inflate, Y: boxMin.Y - inflate, Z: boxMin.Z - inflate}
	hi := r3.Vec{X: boxMax.X + inflate, Y: boxMax.Y + inflate, Z: boxMax.Z + inflate}
	return [8]r3.Vec{
		{X: lo.X, Y: lo.Y, Z: lo.Z}, {X: hi.X, Y: lo.Y, Z: lo.Z},
		{X: lo.X, Y: hi.Y, Z: lo.Z}, {X: lo.X, Y: lo.Y, Z: hi.Z},
		{X: hi.X, Y: hi.Y, Z: lo.Z}, {X: hi.X, Y: lo.Y, Z: hi.Z},
		{X: lo.X, Y: hi.Y, Z: hi.Z}, {X: hi.X, Y: hi.Y, Z: hi.Z},
	}
}
