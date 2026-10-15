package stitchrecord

import (
	"context"
	"fmt"
	"math"
	"slices"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/loftmesh"
	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/stitchweld"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/surfaceplace"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/decad/internal/triangulation"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// This file is Stitch's evaluator (docs/surface-design.md §6). plan.go owns
// Table J's admission (which free edges join); this file owns the fresh
// topology rebuild over the weld plan, the derived orientation, the closure
// and manifold leg, Table C's outcome, and the measurements a stitched body
// publishes.
//
// Build is the whole evaluator and is shared, unchanged, by the
// initial build and by every later Transform/Copy
// re-evaluation: the payload records the operand faces and the weld plan,
// never a built topology, and Placed() replays this same function under a
// composed transform (docs/surface-design.md's "recorded weld" decision).
// Table J's admission is never re-run on a placed (rounded) coordinate; the
// crossing audit IS re-run, because a rigid motion can make two placed
// triangles meet that the unplaced ones did not.

// Build is Stitch's whole evaluator, shared by the initial build
// and every later placement. It places the shared vertex table exactly once
// (never per operand), rebuilds fresh topology over the weld plan, derives
// one consistent orientation, checks closure and manifoldness, decides
// Table C, and publishes the resulting body's measurements.
func Build(ctx context.Context, env topo.Env, srcFaces []*topo.Face, plan *Plan, xform r3.Transform) (*topo.Body, error) {
	if err := ctx.Err(); err != nil {
		return nil, err
	}

	// The single held vertex table, placed under the composed motion ONCE —
	// never per operand (docs/surface-design.md's "recorded weld" decision).
	maxInputAbs := 0.0
	for _, p := range plan.Table.Vertices {
		maxInputAbs = max(maxInputAbs, proofbound.VecMaxAbs(p))
	}
	verts := make([]r3.Vec, len(plan.Table.Vertices))
	for i, p := range plan.Table.Vertices {
		v := xform.Apply(p)
		if !proofbound.FiniteVec(v) {
			return nil, fmt.Errorf(`%w: a placed stitch vertex is not representable`, decaderr.ErrUnsupported)
		}
		verts[i] = v
	}
	delta := 0.0
	if xform != r3.Identity() {
		delta = proofbound.RigidRoundAllow(maxInputAbs, proofbound.VecMaxAbs(xform.Translation()))
	}
	// massDelta is what the mass accumulator charges instead of delta alone:
	// a welded vertex the shared-denotation certificate admitted (J5's
	// bounded-pair lift, docs/surface-design.md §6.2's amendment) is no
	// longer zero-bound, so the triangle set the tetrahedron sum runs over is
	// no longer the body's own true vertices — it is only within
	// maxClassBound of them. proofbound.SweptVolumeAllow already bounds exactly this
	// shape of error (a uniform per-vertex displacement), so folding the
	// widest class bound into the same delta the placement rounding uses is
	// a reuse, not new proof machinery (§6.4's amendment: a bounded weld's
	// volume is never Exact). It stays equal to delta whenever every welded
	// class is zero-bound, which is every case this evaluator admitted
	// before this increment.
	maxClassBound := 0.0
	for _, b := range plan.Table.Bounds {
		maxClassBound = max(maxClassBound, b)
	}
	massDelta := delta
	if maxClassBound > 0 {
		massDelta = proofbound.AbsSumUpper(delta, maxClassBound)
	}

	newFaces, classOf, welded, err := rebuildTopology(ctx, env.Alloc, plan, xform, verts, delta, srcFaces)
	if err != nil {
		return nil, err
	}
	// vertBound reads back, per vertex CLASS index (the same index tris and
	// verts share), the widened bound rebuildTopology's vertexForClass
	// already stamped onto the live Vertex it built for that class —
	// tessellate_stitch.go's own per-face bound is the largest of these over
	// the vertices a face's triangles touch, so this is a read of an
	// existing proof, never a second one.
	vertBound := make([]float64, len(verts))
	for v, class := range classOf {
		vertBound[class] = v.Bound.Base()
	}
	if err := topobuild.AttachFaceLoops(ctx, newFaces); err != nil {
		return nil, err
	}
	if err := DeriveOrientation(newFaces); err != nil {
		return nil, err
	}
	if err := CheckClosure(newFaces); err != nil {
		return nil, err
	}
	// The vertex-link audit runs unconditionally, on every build arm below —
	// all-planar or curved, open or closed alike — never the curved-closed
	// arm alone (docs/surface-design.md §6.4's vertex-link paragraph): a
	// pinch at a shared-vertex-table entry with no connecting edge is
	// reachable through the all-planar path exactly as it is through a
	// curved one, and CheckClosure's own directed-edge parity leg does
	// not see it either way.
	if err := AuditVertexLinks(ctx, newFaces); err != nil {
		return nil, err
	}

	open := hasFreeEdge(newFaces)
	allTetra := AllTetrahedronEligible(newFaces)

	// A fully welded multi-lump assembly whose components are not PROVEN
	// mutually separate is refused outright, on both remaining Table C arms
	// below — the all-planar tetrahedron sum and the curved flux integral
	// alike sum one additive mass/flux term over the whole welded face set
	// with no notion of which component a triangle or face belongs to, so a
	// nested or interlocking pair double-counts the same way regardless of
	// which arm Table C reaches. An open assembly earns no such check: it
	// publishes no volume at all (Table C's first row), so there is nothing
	// for a nested or interlocking pair of components to double-count. This
	// evaluator cannot express a cavity at all — FaceShell.void is hardcoded
	// false and topobuild.SheetLumps builds one shell per lump, so admitting a nested
	// pair would need a new recorded fact, a new shell-assembly rule and a
	// per-component orientation derivation this increment does not add —
	// and a refusal needs none of that. Strict axis-aligned bounding-box
	// separation on the held (placed) vertex coordinates, each widened on
	// both sides by that vertex's OWN proven class bound, is the one
	// question asked instead: it is exact and tolerance-free, unlike the
	// clearance kernel's pointInBody ray cast, which settles crossings
	// through a tolerance and a near-parallel guard and so may never serve
	// as the ADMISSION gate for publishing an exact volume (CLAUDE.md's
	// reject-only rule). Two lumps that are genuinely disjoint but whose
	// boxes interlock in all three axes — a holed-profile extrude capped
	// with annular patches, stitched with a plug sitting in its hole — also
	// refuse here: for axis-aligned boxes that shape cannot arise, no
	// fixture in this tree builds it, and a refusal is the sound answer
	// until this evaluator can decide the case on its own terms.
	if !open {
		if components := topobuild.SplitConnectedFaces(newFaces); len(components) > 1 && !FaceLumpsProvenSeparate(components) {
			return nil, fmt.Errorf(`%w: Stitch refuses a multi-lump assembly whose lumps are not proven mutually separate (docs/surface-design.md Table R row R20)`, decaderr.ErrUnsupported)
		}
	}

	kind, solid := topo.BodySheet, false
	auditClean := false
	var tris [][3]int
	var triFaces []*topo.Face
	var acc *loftmesh.MassAccumulator
	var curvedVolume measurement.Measurement
	var curvedCentroid measurement.VecMeasurement
	haveCurved := false

	switch {
	case allTetra:
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		tris, triFaces, err = TriangulateFaces(ctx, newFaces, classOf)
		if err != nil {
			return nil, err
		}

		budget := proofbound.NewWorkBudget(ctx)
		auditErr := loftmesh.LoftCrossingAudit(budget, verts, tris)
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
			acc = loftmesh.NewMassAccumulator(verts[0], massDelta, 0, 0)
			for _, t := range tris {
				acc.Add(verts[t[0]], verts[t[1]], verts[t[2]], false)
			}
			if acc.Vol6.Sign() < 0 {
				// §5's whole-shell orientation step, re-decided fresh from
				// this placed triangle set every time: the derived
				// combinatorial orientation is consistent either way it
				// lands, so a negative signed volume means the whole
				// assembly is the globally reversed (inward) choice, fixed
				// by reversing every face and re-triangulating.
				for _, f := range newFaces {
					ReverseFaceOrientation(f)
				}
				tris, triFaces, err = TriangulateFaces(ctx, newFaces, classOf)
				if err != nil {
					return nil, err
				}
				acc = loftmesh.NewMassAccumulator(verts[0], massDelta, 0, 0)
				for _, t := range tris {
					acc.Add(verts[t[0]], verts[t[1]], verts[t[2]], false)
				}
			}
			solid, kind = true, topo.BodySolid
		} else if delta > 0 {
			// This accumulator instance only ever feeds perturbAreaSum below
			// (open means solid stays false, so body.Volume/Centroid never
			// read it): Area needs no massDelta charge of its own — each
			// face's own areaBound already covers a bounded weld
			// (docs/surface-design.md §6.4's amendment) — so this stays the
			// placement's own delta, unwidened.
			acc = loftmesh.NewMassAccumulator(verts[0], delta, 0, 0)
			for _, t := range tris {
				acc.Add(verts[t[0]], verts[t[1]], verts[t[2]], false)
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
		// so the volume needs flux.go's per-surface flux integral
		// rather than the exact triangle sum. Rule S gates admission on the
		// single source body's own construction proof; manifoldness at every
		// vertex is already proven above, unconditionally, since the reused
		// crossing audit consumes triangles this set has none of.
		if !RuleSAdmits(ctx, srcFaces) {
			return nil, fmt.Errorf(`%w: Stitch closes a boundary this evaluator cannot prove simple by construction (docs/surface-design.md Table R row R8)`, decaderr.ErrUnsupported)
		}
		// The flux arms integrate each face's tag, so a revolve face is
		// admitted only where its tag is exactly the surface its record
		// denotes. srcFaces are the unplaced operand faces; this stitch's own
		// placement rounding is delta, charged below.
		if !fluxTagsDenoted(srcFaces) {
			return nil, fmt.Errorf(`%w: Stitch's flux path integrates each face's tag, and a revolve face's tag here is not proven to be the surface its record denotes (docs/surface-design.md Table R row R8)`, decaderr.ErrUnsupported)
		}
		// verts[0] anchors the flux sum exactly as loftmesh.NewMassAccumulator
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
		vol, cen, err := CurvedMass(ctx, newFaces, anchor, delta)
		if err != nil {
			return nil, err
		}
		curvedVolume, curvedCentroid, haveCurved = vol, cen, true
		solid, kind = true, topo.BodySolid

	// open && !allTetra: an open curved (or mixed) sheet earns no volume
	// attempt at all — Table C's first row, a residual free edge, and
	// never an error (docs/surface-design.md §6.3).
	default:
	}

	body := env.Alloc.NewBody()
	body.Solid = solid
	body.Origin = provenance.Ref{Producer: env.Ref, Role: provenance.RoleBody}
	body.Kind = kind
	body.Doc = env.Doc
	for _, f := range newFaces {
		f.Body = body
	}
	body.Lumps = topobuild.SheetLumps(env.Alloc, newFaces)

	// Area is the sum of the constituent faces' own area/areaBound through
	// proofbound.BoundedAdd (correction 2, docs/surface-design.md §6.4/§8): never a
	// triangle-sum reading, since a general planar triangle's area is a
	// square root of a rational and so is never Exact the way a face's own
	// analytically integrated area already is.
	areaAcc := proofbound.BoundedScalar{}
	for _, f := range newFaces {
		areaAcc = proofbound.BoundedAdd(areaAcc, proofbound.MeasuredScalar(f.Area, f.AreaBound))
	}
	if acc != nil && delta > 0 {
		areaAcc.Bound = proofbound.AbsSumUpper(areaAcc.Bound, acc.PerturbAreaSum)
	}
	body.Area = measurement.Measurement{
		Value:     units.SquareMillimeters(areaAcc.Value),
		Exactness: measurement.ExactnessOf(areaAcc.Bound),
		Bound:     units.SquareMillimeters(areaAcc.Bound),
	}

	box, err := bounds(srcFaces, xform, delta)
	if err != nil {
		return nil, err
	}
	body.Bounds = box

	if solid {
		switch {
		case acc != nil:
			value, bound := acc.Volume(verts, tris)
			body.Volume = measurement.Measurement{
				Value: units.CubicMillimeters(value), Exactness: measurement.ExactnessOf(bound),
				Bound: units.CubicMillimeters(bound),
			}
			cen, cenBound, err := acc.Centroid(verts, tris)
			if err != nil {
				return nil, err
			}
			body.Centroid = measurement.VecMeasurement{
				Value: cen, Exactness: measurement.ExactnessOf(cenBound), Bound: units.Millimeters(cenBound),
			}
		case haveCurved:
			body.Volume = curvedVolume
			body.Centroid = curvedCentroid
		}
	}

	if err := topobuild.ValidateAnalyticMeasurements(body); err != nil {
		return nil, err
	}

	// tessellate_stitch.go's exact restatement is scoped to the all-planar
	// case, CLOSED or OPEN (docs/surface-design.md §14 Table D row 5): the
	// curved-closure arm's own faces have no triangle set to restate, but an
	// OPEN all-planar sheet's own triangle set is exactly the one this arm
	// already built (and audited via its own perturbed area sum) above, so
	// it is recorded rather than discarded.
	payloadTris, payloadTriFaces, payloadVertBound := tris, triFaces, vertBound
	if !allTetra {
		payloadTris, payloadTriFaces, payloadVertBound = nil, nil, nil
	}
	body.Payload = Payload{
		Xform:      xform,
		Delta:      delta,
		Faces:      srcFaces,
		Plan:       plan,
		LiveFaces:  newFaces,
		Verts:      verts,
		VertBound:  payloadVertBound,
		Tris:       payloadTris,
		TriFaces:   payloadTriFaces,
		AuditClean: auditClean,
	}
	return body, nil
}

// rebuildTopology deep-copies every operand face's surface, origins,
// area, area bound and flags into a fresh Face, over fresh vertices and
// edges built from the weld plan: a welded pair of free edges (plan.Group)
// shares ONE new edge across both faces, and every other edge gets its own,
// shared across whichever coedges of the SAME operand already pointed at
// it. Reusing an operand face outright would corrupt the retired operand
// through its body back-pointer and populated edge list (docs/surface-design.md
// §6, correction 1's construction note), so nothing here aliases an operand's
// own Face, Loop, CoEdge, Edge or Vertex.
//
// AxialDelta, hasAxialDelta and normalBound copy the same way
// unstitch.go's copyFaceUnder does, and for the same reason: the copy
// carries the operand's identical surface and identical tag, so both
// readings stay exactly as true of the copy as they were of the operand.
// hasAxialDelta and normalBound copy verbatim; AxialDelta additionally
// widens by delta under a non-identity placement. A placed copy of an
// operand face whose normalBound is nonzero refuses with [decaderr.ErrUnsupported]
// rather than invent the dimensionless term a sound composition would need
// (unstitch.go's copyFaceUnder doc comment). A revolve face's denoted
// surface (topo.Face.Denoted) carries over as its exact image under xform.
func rebuildTopology(ctx context.Context, alloc topo.Alloc, plan *Plan, xform r3.Transform, verts []r3.Vec, delta float64, srcFaces []*topo.Face) ([]*topo.Face, map[*topo.Vertex]int, map[*topo.Edge]struct{}, error) {
	if xform != r3.Identity() {
		for _, f := range srcFaces {
			if f.NormalBound != 0 {
				return nil, nil, nil, fmt.Errorf(`%w: a placed copy of a face whose normalBound is nonzero has no dimensionless term to bound the placement's own rotation of the tag frame off the true rotation, so this evaluator refuses rather than guess one`, decaderr.ErrUnsupported)
			}
		}
	}
	newVertByClass := map[int]*topo.Vertex{}
	classOf := map[*topo.Vertex]int{}
	// vertexForClass restates the class's own curve token (denotByClass,
	// plan.go — the zero value for a class no member carries one for)
	// under xform, composing rather than overwriting so a token nested
	// through more than one placement still states the true accumulated
	// motion (internal/denotation's CurveToken.Compose).
	vertexForClass := func(class int) *topo.Vertex {
		if nv, ok := newVertByClass[class]; ok {
			return nv
		}
		bound := plan.Table.Bounds[class]
		if delta > 0 {
			bound = proofbound.AbsSumUpper(bound, delta)
		}
		nv := topobuild.NewVertex(alloc, topo.Vertex{Position: verts[class], Bound: units.Millimeters(bound), Denot: plan.Table.Tokens[class].Compose(xform)})
		newVertByClass[class] = nv
		classOf[nv] = class
		return nv
	}

	type edgeBuild struct {
		edge                 *topo.Edge
		startClass, endClass int
	}
	buildByOld := map[*topo.Edge]*edgeBuild{}
	buildByGroup := map[int]*edgeBuild{}
	welded := map[*topo.Edge]struct{}{}

	edgeFor := func(old *topo.Edge) (*edgeBuild, error) {
		if eb, ok := buildByOld[old]; ok {
			return eb, nil
		}
		startClass := plan.Table.classOf(old.Start)
		endClass := plan.Table.classOf(old.End)
		gid, isWelded := plan.Group[old]
		if isWelded {
			if eb, ok := buildByGroup[gid]; ok {
				buildByOld[old] = eb
				return eb, nil
			}
		}
		curve, err := surfaceplace.TransformCurve(old.Curve, xform)
		if err != nil {
			return nil, err
		}
		lengthBound := old.LengthBound
		if delta > 0 {
			lengthBound = proofbound.AbsSumUpper(lengthBound, delta)
		}
		ne := topobuild.NewEdge(alloc, topo.Edge{
			Convex:          old.Convex,
			Length:          old.Length,
			LengthBound:     lengthBound,
			LengthUnbounded: old.LengthUnbounded,
			// A welded pair's two edges are already proven to denote the same
			// curve (Table J), so the FIRST one reached here — the only one
			// that ever builds a new Edge for the group, per buildByGroup
			// above — is a sound representative for the other's identity too.
			Denot: old.Denot.Compose(xform),
			Curve: curve,
			Start: vertexForClass(startClass),
			End:   vertexForClass(endClass),
		})
		ne.CurveBound, ne.CurveBounded = surfaceplace.PlacedCurveBound(
			old.Curve, old.CurveBound, old.CurveBounded, curve, xform)
		eb := &edgeBuild{edge: ne, startClass: startClass, endClass: endClass}
		buildByOld[old] = eb
		if isWelded {
			buildByGroup[gid] = eb
			welded[ne] = struct{}{}
		}
		return eb, nil
	}

	newFaces := make([]*topo.Face, len(srcFaces))
	for i, f := range srcFaces {
		if err := ctx.Err(); err != nil {
			return nil, nil, nil, err
		}
		transformed, err := surfaceplace.TransformSurface(f.Surface, xform)
		if err != nil {
			return nil, nil, nil, err
		}
		axialDelta := f.AxialDelta
		if delta > 0 {
			axialDelta = proofbound.AbsSumUpper(axialDelta, delta)
		}
		nf := topobuild.NewFace(alloc, topo.Face{
			Area:          f.Area,
			AreaBound:     f.AreaBound,
			Reversed:      f.Reversed,
			HeldPlanar:    f.HeldPlanar,
			AxialDelta:    axialDelta,
			HasAxialDelta: f.HasAxialDelta,
			NormalBound:   f.NormalBound,
			Denoted:       f.DenotedUnder(xform),
			Surface:       transformed,
			CapRecord:     f.CapRecord.Under(xform),
			Origins:       append([]provenance.Ref(nil), f.Origins...),
		})
		for _, l := range f.Loops {
			coedges := make([]topo.CoEdge, len(l.CoEdges))
			for j, ce := range l.CoEdges {
				eb, err := edgeFor(ce.Edge)
				if err != nil {
					return nil, nil, nil, err
				}
				// The old edge's own (start, end) may run either the same
				// way as the canonical build's or opposite to it — a weld
				// pairs two free edges whose two operands typically walked
				// their shared rim in opposite senses — so the coedge's
				// forward flag is restated against the CANONICAL edge's own
				// direction rather than copied verbatim.
				oldStartClass := plan.Table.classOf(ce.Edge.Start)
				forward := ce.Forward
				if oldStartClass != eb.startClass {
					forward = !forward
				}
				coedges[j] = topo.CoEdge{Edge: eb.edge, Forward: forward}
			}
			nf.Loops = append(nf.Loops, topobuild.NewLoop(alloc, topo.Loop{Outer: l.Outer, CoEdges: coedges}))
		}
		newFaces[i] = nf
	}
	return newFaces, classOf, welded, nil
}

// ReverseFaceOrientation reverses f's whole boundary walk in place: every
// loop's coedges are reordered back to front with their forward flag
// negated, and f.Reversed toggles with it — the outward side flips together
// with the walk that names it.
func ReverseFaceOrientation(f *topo.Face) {
	for _, l := range f.Loops {
		n := len(l.CoEdges)
		rev := make([]topo.CoEdge, n)
		for i, ce := range l.CoEdges {
			rev[n-1-i] = topo.CoEdge{Edge: ce.Edge, Forward: !ce.Forward}
		}
		l.CoEdges = rev
	}
	f.Reversed = !f.Reversed
}

func topology(faces []*topo.Face) stitchweld.Topology[*topo.Face, *topo.Edge] {
	btopo := stitchweld.Topology[*topo.Face, *topo.Edge]{
		Faces: faces, Uses: map[*topo.Face][]stitchweld.Use[*topo.Edge]{}, Adjacent: map[*topo.Edge][]*topo.Face{},
	}
	queue := append([]*topo.Face(nil), faces...)
	seen := map[*topo.Face]struct{}{}
	for len(queue) > 0 {
		f := queue[0]
		queue = queue[1:]
		if _, ok := seen[f]; ok {
			continue
		}
		seen[f] = struct{}{}
		var uses []stitchweld.Use[*topo.Edge]
		for _, l := range f.Loops {
			for _, ce := range l.CoEdges {
				uses = append(uses, stitchweld.Use[*topo.Edge]{Edge: ce.Edge, Forward: ce.Forward})
				adjacent := ce.Edge.Faces
				btopo.Adjacent[ce.Edge] = adjacent
				queue = append(queue, adjacent...)
			}
		}
		btopo.Uses[f] = uses
	}
	return btopo
}

// DeriveOrientation derives one consistent orientation across the
// welded set by walking every two-face edge and requiring the two adjacent
// faces to traverse it in opposite senses, flipping a whole face (never a
// single coedge) whenever its component's arbitrary root choice disagrees
// with it. A face reached twice with contradicting flips proves the
// component non-orientable — [decaderr.ErrDegenerate] (Table R row R7); the
// three-triangle Möbius fixture in apitest/stitch_test.go exercises exactly this
// path directly, on a hand-built face set, because Stitch's own public
// gates admit no way to reach a non-orientable assembly through the seam.
func DeriveOrientation(faces []*topo.Face) error {
	flip, ok := topology(faces).Orientation()
	if !ok {
		return fmt.Errorf(`%w: Stitch's welded set cannot be consistently oriented (docs/surface-design.md Table R row R7)`, decaderr.ErrDegenerate)
	}
	for f, fl := range flip {
		if fl {
			ReverseFaceOrientation(f)
		}
	}
	return nil
}

// CheckClosure is docs/surface-design.md §6.4's closure and manifold
// leg, the explicit directed-edge parity leg the reused loftmesh.LoftCrossingAudit
// alone does not run (correction 1): every edge must be adjacent to one or
// two faces, and a two-face edge must be traversed by exactly one forward
// and one backward coedge, with a coedge-use count that disagrees with the
// edge's own adjacent-face count also a violation. One face is an open
// boundary — Table C's first row, and never an error here. Three or more,
// or a disagreeing use count, is [decaderr.ErrDegenerate] (Table R row R7): three
// triangles sharing one edge would pass every pairwise contact test the
// crossing audit alone runs, which is exactly the gap this leg closes.
func CheckClosure(faces []*topo.Face) error {
	switch topology(faces).CheckClosure() {
	case stitchweld.ClosureOneFaceUses:
		return fmt.Errorf(`%w: a stitched edge's coedge-use count disagrees with its one adjacent face (docs/surface-design.md Table R row R7)`, decaderr.ErrDegenerate)
	case stitchweld.ClosureTwoFaceParity:
		return fmt.Errorf(`%w: a stitched edge is not traversed by exactly one forward and one backward coedge (docs/surface-design.md Table R row R7)`, decaderr.ErrDegenerate)
	case stitchweld.ClosureTooManyFaces:
		return fmt.Errorf(`%w: a stitched edge is adjacent to more than two faces (docs/surface-design.md Table R row R7)`, decaderr.ErrDegenerate)
	}
	return nil
}

func hasFreeEdge(faces []*topo.Face) bool {
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

// FaceLumpsProvenSeparate compares connected-face components using each
// distinct vertex's held position and proven bound. A touching or nested pair
// cannot prove separation (docs/surface-design.md Table C/R20).
func FaceLumpsProvenSeparate(components [][]*topo.Face) bool {
	return stitchweld.LumpsProvenSeparate(components,
		func(f *topo.Face, visit func(*topo.Vertex)) {
			for _, l := range f.Loops {
				for _, ce := range l.CoEdges {
					visit(ce.Edge.Start)
					visit(ce.Edge.End)
				}
			}
		},
		func(v *topo.Vertex) (r3.Vec, float64, bool) {
			if v == nil {
				return r3.Vec{}, 0, false
			}
			return v.Position, v.Bound.Base(), true
		})
}

// TriangulateFaces triangulates each planar face in its own plane
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
// the mapping afterward — Payload.Faces holds only the retired
// operand faces, and a stitched body's faces carry no per-body-unique role
// to recover it from either (docs/tessellation-design.md §4).
func TriangulateFaces(ctx context.Context, faces []*topo.Face, classOf map[*topo.Vertex]int) ([][3]int, []*topo.Face, error) {
	var tris [][3]int
	var triFaces []*topo.Face
	for _, f := range faces {
		if err := ctx.Err(); err != nil {
			return nil, nil, err
		}
		pl, ok := f.Surface.(surfacegeom.Plane)
		if !ok {
			return nil, nil, fmt.Errorf(`%w: Stitch cannot triangulate a non-planar face`, decaderr.ErrUnsupported)
		}
		frame := pl.Frame
		var pts []sectionrecord.Point2
		var verts []*topo.Vertex
		loopIdx := make([][]int, len(f.Loops))
		for li, l := range f.Loops {
			idxs := make([]int, len(l.CoEdges))
			for i, ce := range l.CoEdges {
				v := topobuild.CoEdgeStart(ce)
				local := frame.ToLocal(v.Position)
				idxs[i] = len(pts)
				pts = append(pts, sectionrecord.Point2{U: local.X, V: local.Y})
				verts = append(verts, v)
			}
			loopIdx[li] = idxs
		}
		for li, l := range f.Loops {
			wantCCW := l.Outer
			isCCW := shoelace(pts, loopIdx[li]) > 0
			if wantCCW != isCCW {
				reverseIntSlice(loopIdx[li])
			}
		}
		tris2D, err := triangulation.Triangulate(ctx, slices.Clone(pts), loopIdx)
		if err != nil {
			return nil, nil, err
		}
		for _, t := range tris2D {
			a, b, c := classOf[verts[t[0]]], classOf[verts[t[1]]], classOf[verts[t[2]]]
			if f.Reversed {
				b, c = c, b
			}
			tris = append(tris, [3]int{a, b, c})
			triFaces = append(triFaces, f)
		}
	}
	return tris, triFaces, nil
}

// shoelace returns twice the signed area of the polygon idx walks over pts —
// its sign alone is what TriangulateFaces reads, to decide whether a
// loop's stored walk is counter-clockwise in the face's own local frame.
func shoelace(pts []sectionrecord.Point2, idx []int) float64 {
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
// guaranteed Plane: only a Line3 rim edge is ever welded (plan.go),
// and only a planar face's straight boundary segment ever records one. It
// must run AFTER any global orientation-sign fix, since that fix can invert
// which side either face calls outward.
func fixWeldedEdgeConvexity(welded map[*topo.Edge]struct{}) {
	for e := range welded {
		if len(e.Faces) != 2 {
			continue
		}
		var fwd, bwd *topo.Face
		for _, f := range e.Faces {
			dir, ok := f.CoEdgeDirection(e)
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
		pa, okA := fwd.Surface.(surfacegeom.Plane)
		pb, okB := bwd.Surface.(surfacegeom.Plane)
		if !okA || !okB {
			continue
		}
		na := pa.Frame.N()
		if fwd.Reversed {
			na = na.Scale(-1)
		}
		nb := pb.Frame.N()
		if bwd.Reversed {
			nb = nb.Scale(-1)
		}
		d := e.End.Position.Sub(e.Start.Position)
		e.Convex = na.Cross(nb).Dot(d) > 0
	}
}

// bounds publishes the assembled body's box as the union of every
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
func bounds(srcFaces []*topo.Face, xform r3.Transform, delta float64) (measurement.Box, error) {
	bodies := topobuild.OperandBodies(srcFaces)
	if len(bodies) == 0 {
		return measurement.Box{}, fmt.Errorf(`%w: Stitch has no operand geometry to bound`, decaderr.ErrDegenerate)
	}
	have := false
	var lo, hi r3.Vec
	for _, b := range bodies {
		box := b.Bounds
		inflate := box.Bound.Base()
		for _, c := range stitchweld.BoxCorners(box.Min, box.Max, inflate) {
			p := xform.Apply(c)
			if !proofbound.FiniteVec(p) {
				return measurement.Box{}, fmt.Errorf(`%w: a placed stitch bound is not representable`, decaderr.ErrUnsupported)
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
	return measurement.Box{Min: lo, Max: hi, Exactness: measurement.ExactnessOf(delta), Bound: units.Millimeters(delta)}, nil
}
