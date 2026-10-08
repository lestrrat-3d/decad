package decad

import (
	"context"
	"errors"
	"math"

	"github.com/lestrrat-3d/decad/internal/clearance"

	"github.com/lestrrat-3d/decad/internal/survey2d"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/revolvemesh"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// This file is the clearance kernel's boundary model (docs/clearance-design.md
// §2/§3): every proven-solid body reduces to trimmed carrier faces read off
// the evaluator's own payload (the same source the surveys read), plus the
// topology's exact edges and vertices. Trim membership on the shipped faces
// is closed form — an angular interval × a meridian/axial range on the
// revolution faces, loop containment with exact crossing parity on the planar
// ones — which is what makes §3's admission exact. It also carries the §2
// nesting-exclusion ray casts: crossings counted against every face,
// closed-form for plane, sphere, cylinder and cone, Sturm-certified for the
// torus quartic, with a deterministic direction ladder retried whenever a
// cast grazes an edge, a vertex or a tangency.

// bodyGeom is one body's kernel boundary model.
//
// delta is a proven upper bound on how far every carrier point, direction and
// axis this model reads may sit from the exact boundary the body's payload
// denotes (docs/clearance-design.md §2's payload-adapter displacement,
// docs/payload-verification-design.md §2.3's delta(X)). addPrismFaces and
// addRevolveFaces fold in the payload's own frame/placement point-rounding
// (the largest clearance.CFace.LiftRound its carriers recorded, each measured
// exactly per lifted point, the same charge topology.go's Vertex.Position
// takes), the payload's own proven axial or angular displacement, and the
// per-face tilt a rounded carrier normal commits across that face's own extent
// (clearance.BodyFaceTiltDelta, below); addRevolveFaces adds how far its
// carriers sit off the recorded meridian swept about the recorded axis
// (clearance.RevolveCarrierResult's AxisGap). addStitchFaces folds in the
// body's own largest proven vertex bound instead. It is exactly zero for an unplaced,
// axis-aligned, feature-built body whose lifts are exact for its own
// coordinates, which is what keeps every such body's Clearance rows Exact.
type bodyGeom struct {
	body     *Body
	faces    []*clearance.CFace
	edges    []*clearance.CEdge
	verts    []r3.Vec
	shellWit []r3.Vec // one witness per shell, void shells included (§2)
	supports []r3.Vec // support points for the pair-D reading (§7)
	delta    float64
	// carrierDelta is the part of delta that can move a carrier the ruling
	// certificates read. It equals delta except on a full revolve, whose
	// angular term charges end angles that no carrier of a full turn reads.
	carrierDelta float64
	// vertexDelta is the largest proven bound any topology vertex the kernel
	// reads carries (Vertex.Position's own Bound). A vertex is lifted through
	// its own float construction — a partial revolve's cap vertex through the
	// held cosine and sine of its end angle, times a radius that can be the
	// whole distance to a far axis — so it can sit farther from the record
	// than any carrier does. widenDelta folds it in; the exact certificates of
	// §6 read carriers alone and keep reading delta and carrierDelta.
	vertexDelta float64
}

// widenDelta is the displacement clearanceDeltaWiden charges this body once
// candidate aggregation completes: every candidate pairs carrier points,
// edge points or vertices, so the larger of delta and vertexDelta bounds how
// far any of them sits from the boundary the payload denotes. It is delta
// itself wherever no vertex bound exceeds it, which keeps every row that did
// not read an uncovered vertex exactly as it was.
func (g *bodyGeom) widenDelta() float64 { return math.Max(g.delta, g.vertexDelta) }

// newBodyGeom builds the kernel model for shipped single-lump analytic
// payloads. Faceted and other multi-lump bodies bypass analytic containment
// and proceed to read-only intersection; ok is false, never a partial model.
func newBodyGeom(b *Body) (*bodyGeom, bool) {
	g, ok, err := newBodyGeomBudget(proofbound.NewWorkBudget(context.Background()), b)
	if err != nil {
		// A background budget never cancels, so this is unreachable.
		return nil, false
	}
	return g, ok
}

// newBodyGeomBudget is the cancellable form. Building a body's carrier faces,
// edges, vertices and support points is linear in the body but is not free, and
// §7.2 carries the context through the entire read-only path — so the whole
// build steps the caller's budget rather than making cancellation wait for both
// operands to finish.
func newBodyGeomBudget(budget *proofbound.WorkBudget, b *Body) (*bodyGeom, bool, error) {
	if err := budget.Err(); err != nil {
		return nil, false, err
	}
	if len(b.lumps) != 1 {
		return nil, false, nil
	}
	g := &bodyGeom{body: b}
	var ok bool
	var err error
	switch pl := b.payload.(type) {
	case prismPayload:
		ok, err = g.addPrismFaces(budget, pl)
	case revolvePayload:
		if pl.surfaceResult {
			// A revolve sheet reaches no pair today (§9.3 stops it on box
			// separation before clearancePair ever sees one), but
			// addRevolveFaces still hands back a closed model for a partial
			// sweep: it unconditionally builds the two cap faces
			// Table W says a surface-result revolve omits. Building that
			// wrong model now would bless a future caller's clearance against
			// caps the body does not have, so this arm refuses one outright —
			// the reject-only answer (CLAUDE.md) where building a model is
			// not. Only a plain, solid revolve reaches addRevolveFaces.
			return nil, false, nil
		}
		ok, err = g.addRevolveFaces(budget, pl)
	case brepPayload:
		ok, err = g.addBrepFaces(budget, pl)
	case stitchPayload:
		// Refuse outright unless the recorded triangle set exists (an
		// all-planar body, open or closed — tessellate_stitch.go's own gate).
		// A bounded body — placed (stitch.go's proofbound.RigidRoundAllow widens every
		// vertex bound) or closed by the CURVE weld certificate at a nonzero
		// class bound — no longer refuses on that alone: bodyGeom.delta now
		// exists for it to charge its own worst vertex bound against
		// (stitchMaxVertexBound, stitch.go), the identical charge a placed
		// prism's frame/placement rounding takes (bodyGeom's own doc
		// comment), so this arm needs no standing narrower than addPrismFaces'
		// own anymore.
		if pl.tris == nil {
			return nil, false, nil
		}
		maxVertexBound, mErr := stitchMaxVertexBound(budget, b)
		if mErr != nil {
			return nil, false, mErr
		}
		if ok, err = g.addStitchFaces(budget, b, pl); ok && err == nil {
			g.delta = maxVertexBound
			g.carrierDelta = maxVertexBound
		}
	default:
		return nil, false, nil
	}
	if err != nil || !ok {
		return nil, false, err
	}
	if ok, err = g.addTopology(budget, b); err != nil || !ok {
		return nil, false, err
	}
	g.supports = append([]r3.Vec{}, g.verts...)
	for _, f := range g.faces {
		if err := budget.Step(); err != nil {
			return nil, false, err
		}
		g.supports = append(g.supports, f.Wit...)
	}
	return g, true, nil
}

// addTopology reads exact edges and vertices and picks one witness per shell
// for the §2 nesting casts. Every shell earns a witness, void shells included:
// a void shell of one body can lie wholly inside the other body's material, so
// its membership is not implied by any other shell's.
func (g *bodyGeom) addTopology(budget *proofbound.WorkBudget, b *Body) (bool, error) {
	for _, e := range b.Edges() {
		if err := budget.Step(); err != nil {
			return false, err
		}
		ce, ok := newCEdge(e)
		if !ok {
			return false, nil
		}
		if ce != nil {
			g.edges = append(g.edges, ce)
		}
	}
	for _, v := range b.Vertices() {
		if err := budget.Step(); err != nil {
			return false, err
		}
		g.verts = append(g.verts, v.position)
		if m := v.Position().Bound.Mag(); m > g.vertexDelta {
			g.vertexDelta = m
		}
	}
	for _, sh := range b.Shells() {
		if err := budget.Step(); err != nil {
			return false, err
		}
		w, ok := shellWitness(sh)
		if !ok {
			return false, nil
		}
		g.shellWit = append(g.shellWit, w)
	}
	return true, nil
}

func newCEdge(e *Edge) (*clearance.CEdge, bool) {
	switch c := e.curve.(type) {
	case Line3:
		if e.start.position.Sub(e.end.position).Len() == 0 {
			return nil, true
		}
		ce := &clearance.CEdge{Line: true, A: e.start.position, B: e.end.position}
		ce.Box = clearance.BoxOf(ce.A, ce.B)
		return ce, true
	case Circle3:
		r, err := c.Radius.In(units.Millimeter)
		if err != nil {
			return nil, false
		}
		u := clearance.PerpTo(c.Axis)
		v := c.Axis.Cross(u)
		ce := &clearance.CEdge{Center: c.Center, Axis: c.Axis, RefU: u, RefV: v, Radius: r, Ang: clearance.AngWindow{Full: true}}
		ce.Box = clearance.CircleBox(c.Center, c.Axis, r)
		return ce, true
	case Arc3:
		r, err := c.Radius.In(units.Millimeter)
		if err != nil {
			return nil, false
		}
		rel := e.start.position.Sub(c.Center)
		u, ok := rel.Normalize()
		if !ok {
			return nil, false
		}
		v := c.Axis.Cross(u)
		relEnd := e.end.position.Sub(c.Center)
		sweep := math.Atan2(relEnd.Dot(v), relEnd.Dot(u))
		if sweep <= 0 {
			sweep += 2 * math.Pi
		}
		ce := &clearance.CEdge{Center: c.Center, Axis: c.Axis, RefU: u, RefV: v, Radius: r, Ang: clearance.NewAngWindow(0, sweep)}
		ce.Box = clearance.CircleBox(c.Center, c.Axis, r)
		return ce, true
	default:
		return nil, false
	}
}

// shellWitness returns a point provenly on the shell: a vertex where one
// exists, else the loop-less closed face's own synthesized point off stored
// surface data (§2).
func shellWitness(sh *Shell) (r3.Vec, bool) {
	for _, f := range sh.faces {
		for _, e := range f.Edges() {
			if e.start != nil {
				return e.start.position, true
			}
		}
	}
	for _, f := range sh.faces {
		switch s := f.surface.(type) {
		case Sphere:
			r, err := s.Radius.In(units.Millimeter)
			if err != nil {
				return r3.Vec{}, false
			}
			return s.Center.Add(clearance.PerpTo(r3.NewVec(0, 0, 1)).Scale(r)), true
		case Torus:
			major, err1 := s.Major.In(units.Millimeter)
			minor, err2 := s.Minor.In(units.Millimeter)
			if err1 != nil || err2 != nil {
				return r3.Vec{}, false
			}
			return s.Center.Add(clearance.PerpTo(s.Axis).Scale(major + minor)), true
		}
	}
	return r3.Vec{}, false
}

// addPrismFaces builds the prism's faces from its own payload, mirroring the
// walk decomposition the evaluator built the topology from.
//
// A surface-result payload (docs/surface-design.md §4.1) omits both cap
// faces, unconditionally and with no caller flag: a surface-result prism's
// walls ARE its whole boundary, so a cap face in the model is geometry the
// body does not have, and no caller ever wants a sheet modelled as the closed
// solid it is not. The walk validity gate (walkElem, below) still runs for
// every wall regardless — it is not cap-only construction, it is the shared
// check that a wall's own segment kind is one this kernel can model at all —
// only the cap-only region built from its result is skipped.
func (g *bodyGeom) addPrismFaces(budget *proofbound.WorkBudget, pp prismPayload) (bool, error) {
	if pp.sectionDelta != 0 {
		// The kernel's certificates are exact statements about the carriers it
		// reads, so a section the payload only holds within a displacement of
		// (docs/prism-boolean-design.md §7) is a payload it cannot model: no
		// model, which leaves the pair undecided rather than certifying a gap
		// against the wrong boundary.
		return false, nil
	}
	loops, err := recordLoops(budget, pp.profile)
	if err != nil {
		if errors.Is(err, context.Canceled) || errors.Is(err, context.DeadlineExceeded) {
			return false, err
		}
		// A section this kernel cannot decompose is an unsupported payload, not
		// a failure: newBodyGeom answers with no model rather than a partial
		// one. The error return is reserved for cancellation.
		return false, nil
	}
	built, ok, err := clearance.BuildPrismCarriers(budget, prismCarrierFrame(pp), loops, pp.surfaceResult)
	if err != nil || !ok {
		return false, err
	}
	g.faces = append(g.faces, built.Faces...)

	// g.delta charges the three terms clearance_geom.go's package doc comment
	// and bodyGeom's own doc comment name: the payload's own frame/placement
	// point rounding (every anchor, axis point and witness came from the
	// frame's point lift, each carrier recording the largest exact rounding
	// its own lifts committed in CFace.LiftRound), its proven axial
	// displacement (pp.axialDelta — the section's own sectionDelta is already
	// refused above), and the worst per-face tilt a rounded carrier normal
	// commits (every u, v and n came from the frame's direction lift). It is
	// exactly zero for an unplaced, feature-built prism whose lifts are exact
	// for its own coordinates, which keeps its Clearance rows Exact.
	pointTerm := carrierLiftRound(built.Faces)
	g.delta = proofbound.AbsSumUpper(pointTerm, pp.axialDelta(), clearance.BodyFaceTiltDelta(g.faces, pp.frame, pp.xform))
	g.carrierDelta = g.delta
	return true, nil
}

// carrierLiftRound is the largest frame/placement lift rounding any of faces
// recorded (clearance.CFace.LiftRound): the point term of bodyGeom.delta.
func carrierLiftRound(faces []*clearance.CFace) float64 {
	round := 0.0
	for _, f := range faces {
		round = math.Max(round, f.LiftRound)
	}
	return round
}

func prismCarrierFrame(pp prismPayload) clearance.PrismCarrierFrame {
	return clearance.PrismCarrierFrame{Frame: pp.frame, Transform: pp.xform, Z0: pp.z0, Z1: pp.z1}
}

// addBrepFaces builds a brep body's carrier faces from its own record
// (docs/general-boolean-design.md §4.5's clearance row), each face over its
// own frame through the prism builders addPrismFaces uses: a swept face is
// prismWallCFace over its wall walk and interval, a planar face a plane at
// its level whose trim region is its own loops' (outer and holes, crossing
// parity decides membership), its normal the frame's N turned outward.
//
// A record carrying any section displacement has no model, as addPrismFaces
// refuses a displaced prism: the certificates are exact statements about the
// carriers read. g.delta charges, per face, the same three terms a prism's
// model does — the exact frame and placement rounding of that face's own
// lifted points (CFace.LiftRound), the record's largest level displacement,
// and the tilt a rounded carrier normal commits across that face's extent —
// and keeps the largest.
func (g *bodyGeom) addBrepFaces(budget *proofbound.WorkBudget, bp brepPayload) (bool, error) {
	if bp.sectionDelta() != 0 {
		return false, nil
	}
	pointTerm, tiltTerm := 0.0, 0.0
	for _, f := range bp.faces {
		if err := budget.Step(); err != nil {
			return false, err
		}
		pp := f.view(bp.xform)
		var face *clearance.CFace
		if f.planar() {
			loops, err := recordLoops(budget, *f.region)
			if err != nil {
				if errors.Is(err, context.Canceled) || errors.Is(err, context.DeadlineExceeded) {
					return false, err
				}
				return false, nil
			}
			var elems []survey2d.SurveyElem
			for _, loop := range loops {
				for _, w := range loop {
					el, ok := walkElem(w.SegmentWalk)
					if !ok {
						return false, nil
					}
					elems = append(elems, el)
				}
			}
			sign := -1.0
			if f.outward {
				sign = 1
			}
			face = clearance.PrismPlaneCarrier(prismCarrierFrame(pp), f.z0, sign, clearance.NewRegion2(elems))
		} else {
			w, ok := brepCarrierWalk(f.wall)
			if !ok {
				return false, nil
			}
			if face, ok = clearance.PrismWallCarrier(prismCarrierFrame(pp), w); !ok {
				return false, nil
			}
		}
		g.faces = append(g.faces, face)
		pointTerm = math.Max(pointTerm, face.LiftRound)
		tiltTerm = math.Max(tiltTerm, clearance.BodyFaceTiltDelta([]*clearance.CFace{face}, f.frame, bp.xform))
	}
	g.delta = proofbound.AbsSumUpper(pointTerm, bp.axialDelta(), tiltTerm)
	g.carrierDelta = g.delta
	return true, nil
}

// brepCarrierWalk walks one brep wall for the clearance model. ok is false
// for a wall this kernel cannot carry, which leaves the body with no model.
func brepCarrierWalk(seg CurveSegment) (survey2d.SegmentWalk, bool) {
	w, err := walkOf(seg, nil)
	if err != nil {
		return survey2d.SegmentWalk{}, false
	}
	_, ok := walkElem(w)
	return w, ok
}

// addRevolveFaces builds the revolved body's faces from its own payload. Its
// cap planes, clearance.AngWindow and witnesses all read rp.phi0/rp.phi1 as the held
// sweep angle rather than the angle the record denotes, and folds
// rp.angularDelta() — scaled by the radial envelope every witness can carry
// it at, verify_gate.go's own reading of the same displacement — into
// bodyGeom.delta below, beside the payload's own frame/placement point
// rounding, the per-face tilt term every plane carrier here can commit
// (bodyGeom's own doc comment) and how far the carriers sit off the recorded
// meridian swept about the recorded axis (clearance.RevolveCarrierResult's
// AxisGap).
func (g *bodyGeom) addRevolveFaces(budget *proofbound.WorkBudget, rp revolvePayload) (bool, error) {
	in, ok, err := revolveCarrierInput(budget, rp)
	if err != nil || !ok {
		return false, err
	}
	built := clearance.BuildRevolveCarriers(in)
	g.faces = append(g.faces, built.Faces...)
	g.verts = append(g.verts, built.Vertices...)

	// g.delta mirrors addPrismFaces' own three terms, over this payload's own
	// axis-symmetric construction: the exact frame/placement point rounding
	// every anchor, axis point and witness took through the placed basis
	// (CFace.LiftRound), the angular
	// displacement scaled by the radial envelope every such point can carry it
	// at (the same reading verify_gate.go's own bodyGateDiameter arm takes for
	// the identical displacement), and the worst per-face tilt a rounded
	// wallPlane/cap carrier normal commits. It is exactly zero for an
	// axis-aligned, unplaced, full-turn revolve whose lifts are exact for its
	// own coordinates, which keeps an ordinary revolve's Clearance rows Exact.
	pointTerm := carrierLiftRound(built.Faces)
	angularTerm := proofbound.ProductUpper(built.AxisRadiusUpper, rp.angularDelta())
	tiltTerm := clearance.BodyFaceTiltDelta(g.faces, rp.frame, rp.xform)
	g.delta = proofbound.AbsSumUpper(pointTerm, angularTerm, tiltTerm)
	g.carrierDelta = g.delta
	if rp.full {
		// A full turn builds no cap faces and gives every carrier the full
		// angular window, so the end angles the angular term charges place
		// only seams and witnesses on the complete surface of revolution.
		g.carrierDelta = proofbound.AbsSumUpper(pointTerm, tiltTerm)
	}
	// The carriers read the walk's float axis coordinates, so each sits off
	// the record by what the re-expression, an axis snap and the axis's own
	// error put between them (built.AxisGap). That moves the carrier itself,
	// so it widens both figures. An absent gap folds nothing: AbsSumUpper
	// up-rounds every term, zero included, and an exact revolve must read as
	// it did before the charge existed.
	if built.AxisGap > 0 {
		g.delta = proofbound.AbsSumUpper(g.delta, built.AxisGap)
		g.carrierDelta = proofbound.AbsSumUpper(g.carrierDelta, built.AxisGap)
	}
	return true, nil
}

// revolveCarrierInput resolves rp's meridian walks, each beside the recorded
// plane-local walks it was re-expressed from, into the carrier builder's
// input. ok is false for a meridian this kernel cannot decompose, which
// leaves the body with no model.
func revolveCarrierInput(budget *proofbound.WorkBudget, rp revolvePayload) (clearance.RevolveCarrierInput, bool, error) {
	loops, planes, err := revolveLoopsPlane(budget, rp)
	if err != nil {
		if errors.Is(err, context.Canceled) || errors.Is(err, context.DeadlineExceeded) {
			return clearance.RevolveCarrierInput{}, false, err
		}
		if errors.Is(err, ErrUnsupported) {
			// An undecomposable meridian is an unsupported payload, not a
			// failure: newBodyGeom answers with no model rather than a partial
			// one. Other errors remain visible to the caller.
			return clearance.RevolveCarrierInput{}, false, nil
		}
		return clearance.RevolveCarrierInput{}, false, err
	}
	var walls []clearance.RevolveWall
	for li, loop := range loops {
		for _, w := range loop {
			if err := budget.Step(); err != nil {
				return clearance.RevolveCarrierInput{}, false, err
			}
			wall := clearance.RevolveWall{
				Walk: w, Kind: rp.ax.classify(w.SegmentWalk),
				First: planes[li][w.Segs[0]], Last: planes[li][w.Segs[len(w.Segs)-1]],
			}
			for _, seg := range w.Segs[1:] {
				rec := planes[li][seg]
				at := rp.ax.walk(rec)
				wall.Joints = append(wall.Joints, clearance.RevolveJoint{
					Z: at.StartU, Rho: at.StartV,
					Rec: revolvemesh.RecordedMeridian{U: rec.StartU, V: rec.StartV, UV: rec.StartBound},
				})
			}
			walls = append(walls, wall)
		}
	}
	return clearance.RevolveCarrierInput{
		Lift: rp.lift(), Axis: rp.axisBound(), Transform: rp.xform, Full: rp.full,
		Phi0: rp.phi0, Phi1: rp.phi1, Walls: walls,
	}, true, nil
}

// addStitchFaces builds one exact planar carrier per live face of a CLOSED,
// all-planar stitched body (docs/clearance-design.md §2). Every coordinate it
// reads comes straight off the body's own topology, so this arm introduces no
// rounding of its own; the caller (newBodyGeomBudget) charges the body's own
// worst proven vertex bound into bodyGeom.delta separately
// (stitchMaxVertexBound, stitch.go) rather than refusing a bounded body a
// model outright.
//
// The plane frame's origin and axes come straight off the face's own Plane
// tag, negating the normal when the face is reversed — the identical
// construction fixWeldedEdgeConvexity (stitch.go) already uses for a welded
// edge's own convexity. The trim region reads each loop's own coedge start
// vertices, outer loop and holes together, exactly as addPrismFaces builds
// its cap region. The interior witnesses are the payload's own recorded
// triangle centroids for that face, filtered by the region's own classify
// call as a reject-only confirmation (CLAUDE.md's reject-only rule): ear
// clipping tiles the polygon, so every centroid is interior by construction,
// and this filter costs nothing more than a proof it never fires wrong.
func (g *bodyGeom) addStitchFaces(budget *proofbound.WorkBudget, b *Body, sp stitchPayload) (bool, error) {
	for _, f := range b.Faces() {
		if err := budget.Step(); err != nil {
			return false, err
		}
		pl, ok := f.surface.(Plane)
		if !ok {
			// Every face of a closed, all-planar stitched body is a Plane
			// (tessellate_stitch.go's own gate); anything else reaching here
			// means the payload should never have offered this arm a model.
			return false, nil
		}
		normal := pl.Frame.N()
		if f.reversed {
			normal = normal.Scale(-1)
		}
		cf := &clearance.CFace{Kind: clearance.CkPlane, O: pl.Frame.Origin(), U: pl.Frame.U(), V: pl.Frame.V(), N: normal}

		var elems []survey2d.SurveyElem
		var pts []r3.Vec
		for _, l := range f.loops {
			cnt := len(l.coedges)
			if cnt == 0 {
				return false, nil
			}
			local := make([]r3.Vec, cnt)
			for i, ce := range l.coedges {
				if err := budget.Step(); err != nil {
					return false, err
				}
				v := ce.Start().position
				pts = append(pts, v)
				local[i] = pl.Frame.ToLocal(v)
			}
			for i := range local {
				a, next := local[i], local[(i+1)%cnt]
				if el, ok := survey2d.LineElem(a.X, a.Y, next.X, next.Y); ok {
					elems = append(elems, el)
				}
			}
		}
		region, err := clearance.NewRegion2Budget(budget, elems)
		if err != nil {
			return false, err
		}
		cf.Region = region
		cf.Box = clearance.BoxOf(pts...)

		for i, tri := range sp.tris {
			if sp.triFaces[i] != f {
				continue
			}
			if err := budget.Step(); err != nil {
				return false, err
			}
			a, b2, c := sp.verts[tri[0]], sp.verts[tri[1]], sp.verts[tri[2]]
			cen := a.Add(b2).Add(c).Scale(1.0 / 3.0)
			lx, ly := cf.PlaneCoords(cen)
			if cf.Region.Classify(lx, ly, cf.Region.Tol()) > 0 {
				cf.Wit = append(cf.Wit, cen)
			}
		}
		g.faces = append(g.faces, cf)
	}
	return true, nil
}

// pointInBody decides strict membership of a point in the body's material by
// a certified ray cast (§2): crossings counted against every face, each
// crossing certified transversal and interior to its trim; a grazing cast
// retries the next ladder direction. ok is false when every direction stays
// ambiguous.

func (g *bodyGeom) pointInBody(ctx context.Context, p r3.Vec, tol float64) (bool, bool, error) {
	budget := proofbound.NewWorkBudget(ctx)
	for _, dir := range clearance.ClrLadder() {
		if err := ctx.Err(); err != nil {
			return false, false, err
		}
		total, ok := 0, true
		for _, f := range g.faces {
			if err := budget.Step(); err != nil {
				return false, false, err
			}
			n, good, err := f.RayCrossings(ctx, p, dir, tol)
			if err != nil {
				return false, false, err
			}
			if !good {
				ok = false
				break
			}
			total += n
		}
		if ok {
			return total%2 == 1, true, nil
		}
	}
	return false, false, nil
}
