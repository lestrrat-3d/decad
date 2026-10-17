package decad

import (
	"context"
	"errors"
	"math"

	"github.com/lestrrat-3d/decad/internal/clearance"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"

	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/lestrrat-3d/decad/internal/wallsurvey"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/stitchweld"

	"github.com/lestrrat-3d/decad/internal/boundarywalk"
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

// newBodyGeomBudget is the cancellable form. Building a body's carrier faces,
// edges and vertices is linear in the body but is not free, and
// §7.2 carries the context through the entire read-only path — so the whole
// build steps the caller's budget rather than making cancellation wait for both
// operands to finish.
func newBodyGeomBudget(budget *proofbound.WorkBudget, b *Body) (*bodyGeom, bool, error) {
	if err := budget.Err(); err != nil {
		return nil, false, err
	}
	if len(b.n.Lumps) != 1 {
		return nil, false, nil
	}
	g := &bodyGeom{body: b}
	var ok bool
	var err error
	switch pl := b.n.Payload.(type) {
	case prismPayload:
		ok, err = g.addPrismFaces(budget, pl)
	case revolvePayload:
		if pl.SurfaceResult {
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
		// A bounded body — placed (internal/stitchrecord/build.go's proofbound.RigidRoundAllow widens every
		// vertex bound) or closed by the CURVE weld certificate at a nonzero
		// class bound — no longer refuses on that alone: bodyGeom.delta now
		// exists for it to charge its own worst vertex bound against
		// (stitchweld.MaxBound over stitchVertexBounds, stitch.go), the
		// identical charge a placed prism's frame/placement rounding takes
		// (bodyGeom's own doc comment), so this arm needs no standing narrower than addPrismFaces'
		// own anymore.
		if pl.Tris == nil {
			return nil, false, nil
		}
		maxVertexBound, mErr := stitchweld.MaxBound(budget, stitchVertexBounds(b))
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
		g.verts = append(g.verts, v.n.Position)
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
	switch c := e.n.Curve.(type) {
	case surfacegeom.Line3:
		if e.n.Start.Position.Sub(e.n.End.Position).Len() == 0 {
			return nil, true
		}
		ce := &clearance.CEdge{Line: true, A: e.n.Start.Position, B: e.n.End.Position}
		ce.Box = clearance.BoxOf(ce.A, ce.B)
		return ce, true
	case surfacegeom.Circle3:
		r, err := c.Radius.In(units.Millimeter)
		if err != nil {
			return nil, false
		}
		u := clearance.PerpTo(c.Axis)
		v := c.Axis.Cross(u)
		ce := &clearance.CEdge{Center: c.Center, Axis: c.Axis, RefU: u, RefV: v, Radius: r, Ang: clearance.AngWindow{Full: true}}
		ce.Box = clearance.CircleBox(c.Center, c.Axis, r)
		return ce, true
	case surfacegeom.Arc3:
		r, err := c.Radius.In(units.Millimeter)
		if err != nil {
			return nil, false
		}
		rel := e.n.Start.Position.Sub(c.Center)
		u, ok := rel.Normalize()
		if !ok {
			return nil, false
		}
		v := c.Axis.Cross(u)
		relEnd := e.n.End.Position.Sub(c.Center)
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
func shellWitness(sh *FaceShell) (r3.Vec, bool) {
	for _, f := range sh.n.Faces {
		for _, l := range f.Loops {
			for _, ce := range l.CoEdges {
				if ce.Edge.Start != nil {
					return ce.Edge.Start.Position, true
				}
			}
		}
	}
	for _, f := range sh.n.Faces {
		switch s := f.Surface.(type) {
		case surfacegeom.Sphere:
			r, err := s.Radius.In(units.Millimeter)
			if err != nil {
				return r3.Vec{}, false
			}
			return s.Center.Add(clearance.PerpTo(r3.NewVec(0, 0, 1)).Scale(r)), true
		case surfacegeom.Torus:
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
// solid it is not. The walk validity gate (survey2d.WalkElem, below) still runs for
// every wall regardless — it is not cap-only construction, it is the shared
// check that a wall's own segment kind is one this kernel can model at all —
// only the cap-only region built from its result is skipped.
func (g *bodyGeom) addPrismFaces(budget *proofbound.WorkBudget, pp prismPayload) (bool, error) {
	if pp.SectionDelta != 0 {
		// The kernel's certificates are exact statements about the carriers it
		// reads, so a section the payload only holds within a displacement of
		// (docs/prism-boolean-design.md §7) is a payload it cannot model: no
		// model, which leaves the pair undecided rather than certifying a gap
		// against the wrong boundary.
		return false, nil
	}
	loops, err := boundarywalk.SurveyLoops(budget, boundarywalk.Profile(pp.Profile))
	if err != nil {
		if errors.Is(err, context.Canceled) || errors.Is(err, context.DeadlineExceeded) {
			return false, err
		}
		// A section this kernel cannot decompose is an unsupported payload, not
		// a failure: newBodyGeom answers with no model rather than a partial
		// one. The error return is reserved for cancellation.
		return false, nil
	}
	built, ok, err := clearance.BuildPrismCarriers(budget, prismCarrierFrame(pp), loops, pp.SurfaceResult)
	if err != nil || !ok {
		return false, err
	}
	g.faces = append(g.faces, built.Faces...)

	// g.delta charges the three terms clearance_geom.go's package doc comment
	// and bodyGeom's own doc comment name: the payload's own frame/placement
	// point rounding (every anchor, axis point and witness came from the
	// frame's point lift, each carrier recording the largest exact rounding
	// its own lifts committed in CFace.LiftRound), its proven axial
	// displacement (pp.AxialDelta — the section's own sectionDelta is already
	// refused above), and the worst per-face tilt a rounded carrier normal
	// commits (every u, v and n came from the frame's direction lift). It is
	// exactly zero for an unplaced, feature-built prism whose lifts are exact
	// for its own coordinates, which keeps its Clearance rows Exact.
	pointTerm := carrierLiftRound(built.Faces)
	g.delta = proofbound.AbsSumUpper(pointTerm, pp.AxialDelta(), clearance.BodyFaceTiltDelta(g.faces, pp.Frame, pp.Xform))
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
	return clearance.PrismCarrierFrame{Frame: pp.Frame, Transform: pp.Xform, Z0: pp.Z0, Z1: pp.Z1}
}

// addBrepFaces builds a brep body's carrier faces from its own record
// (docs/general-boolean-design.md §4.5's clearance row), each face over its
// own frame through the prism builders addPrismFaces uses: a swept face is
// clearance.PrismWallCarrier over its wall walk and interval, a planar face a plane at
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
	// A record carrying route L chamfer bands has no model: its faces omit
	// the band patches (docs/modify-general-design.md Table DG's DG6, PR L-2).
	if bp.SectionDelta() != 0 || len(bp.LoopBands) > 0 || bp.BossShell != nil || bp.PocketShell != nil {
		return false, nil
	}
	pointTerm, tiltTerm := 0.0, 0.0
	for _, f := range bp.Faces {
		if err := budget.Step(); err != nil {
			return false, err
		}
		pp := f.View(bp.Xform)
		var face *clearance.CFace
		if f.Planar() {
			loops, err := boundarywalk.SurveyLoops(budget, boundarywalk.Profile(*f.Region))
			if err != nil {
				if errors.Is(err, context.Canceled) || errors.Is(err, context.DeadlineExceeded) {
					return false, err
				}
				return false, nil
			}
			var elems []survey2d.SurveyElem
			for _, loop := range loops {
				for _, w := range loop {
					el, ok := survey2d.WalkElem(w.SegmentWalk)
					if !ok {
						return false, nil
					}
					elems = append(elems, el)
				}
			}
			sign := -1.0
			if f.Outward {
				sign = 1
			}
			face = clearance.PrismPlaneCarrier(prismCarrierFrame(pp), f.Z0, sign, clearance.NewRegion2(elems))
		} else {
			w, ok := brepCarrierWalk(f.Wall)
			if !ok {
				return false, nil
			}
			if face, ok = clearance.PrismWallCarrier(prismCarrierFrame(pp), w); !ok {
				return false, nil
			}
		}
		g.faces = append(g.faces, face)
		pointTerm = math.Max(pointTerm, face.LiftRound)
		tiltTerm = math.Max(tiltTerm, clearance.BodyFaceTiltDelta([]*clearance.CFace{face}, f.Frame, bp.Xform))
	}
	g.delta = proofbound.AbsSumUpper(pointTerm, bp.AxialDelta(), tiltTerm)
	g.carrierDelta = g.delta
	return true, nil
}

// brepCarrierWalk walks one brep wall for the clearance model. ok is false
// for a wall this kernel cannot carry, which leaves the body with no model.
func brepCarrierWalk(seg curveSegment) (survey2d.SegmentWalk, bool) {
	w, err := boundarywalk.WalkOf(seg, nil)
	if err != nil {
		return survey2d.SegmentWalk{}, false
	}
	_, ok := survey2d.WalkElem(w)
	return w, ok
}

// addRevolveFaces builds the revolved body's faces from its own payload. Its
// cap planes, clearance.AngWindow and witnesses all read rp.Phi0/rp.Phi1 as the held
// sweep angle rather than the angle the record denotes, and folds
// rp.AngularDelta() — scaled by the radial envelope every witness can carry
// it at, verify_gate.go's own reading of the same displacement — into
// bodyGeom.delta below, beside the payload's own frame/placement point
// rounding, the per-face tilt term every plane carrier here can commit
// (bodyGeom's own doc comment) and how far the carriers sit off the recorded
// meridian swept about the recorded axis (clearance.RevolveCarrierResult's
// AxisGap).
func (g *bodyGeom) addRevolveFaces(budget *proofbound.WorkBudget, rp revolvePayload) (bool, error) {
	if rp.SectionDelta != 0 {
		// addPrismFaces' own refusal over the meridian: the carriers would be
		// exact statements about a section the payload only holds within its
		// displacement of (docs/surface-intersection-design.md §7.2). No model,
		// so the pair stays undecided.
		return false, nil
	}
	in, ok, err := wallsurvey.RevolveCarrierInput(budget, rp.Profile, rp.Ax, rp.Lift(), rp.AxisBound(),
		rp.Xform, rp.Full, rp.Phi0, rp.Phi1)
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
	angularTerm := proofbound.ProductUpper(built.AxisRadiusUpper, rp.AngularDelta())
	tiltTerm := clearance.BodyFaceTiltDelta(g.faces, rp.Frame, rp.Xform)
	g.delta = proofbound.AbsSumUpper(pointTerm, angularTerm, tiltTerm)
	g.carrierDelta = g.delta
	if rp.Full {
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

// addStitchFaces builds one exact planar carrier per live face of a CLOSED,
// all-planar stitched body (docs/clearance-design.md §2). Every coordinate it
// reads comes straight off the body's own topology, so this arm introduces no
// rounding of its own; the caller (newBodyGeomBudget) charges the body's own
// worst proven vertex bound into bodyGeom.delta separately
// (stitchweld.MaxBound over stitchVertexBounds, stitch.go) rather than
// refusing a bounded body a model outright.
//
// The plane frame's origin and axes come straight off the face's own Plane
// tag, negating the normal when the face is reversed — the identical
// construction fixWeldedEdgeConvexity (internal/stitchrecord/build.go) already uses for a welded
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
		pl, ok := f.n.Surface.(surfacegeom.Plane)
		if !ok {
			// Every face of a closed, all-planar stitched body is a Plane
			// (tessellate_stitch.go's own gate); anything else reaching here
			// means the payload should never have offered this arm a model.
			return false, nil
		}
		normal := pl.Frame.N()
		if f.n.Reversed {
			normal = normal.Scale(-1)
		}
		cf := &clearance.CFace{Kind: clearance.CkPlane, O: pl.Frame.Origin(), U: pl.Frame.U(), V: pl.Frame.V(), N: normal}

		var elems []survey2d.SurveyElem
		var pts []r3.Vec
		for _, l := range f.n.Loops {
			cnt := len(l.CoEdges)
			if cnt == 0 {
				return false, nil
			}
			local := make([]r3.Vec, cnt)
			for i, ce := range l.CoEdges {
				if err := budget.Step(); err != nil {
					return false, err
				}
				v := coedgeStart(ce).Position
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
		// The box holds the region as the cells read it, mapped back through
		// the plane's frame (CapBox), and the topology's own vertices.
		cf.Box = clearance.BoxUnion(clearance.BoxOf(pts...), clearance.CapBox(cf))

		fn := faceNode(f)
		for i, tri := range sp.Tris {
			if sp.TriFaces[i] != fn {
				continue
			}
			if err := budget.Step(); err != nil {
				return false, err
			}
			a, b2, c := sp.Verts[tri[0]], sp.Verts[tri[1]], sp.Verts[tri[2]]
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
