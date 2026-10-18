package decad

import (
	"math"

	"github.com/lestrrat-3d/decad/internal/clearance"
	"github.com/lestrrat-3d/decad/internal/wallsurvey"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/stitchweld"
	"github.com/lestrrat-3d/decad/internal/topobuild"

	"github.com/lestrrat-3d/r3"
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
		var built clearance.BodyCarriers
		built, ok, err = clearance.BuildPrismBodyCarriers(budget, pl)
		if ok && err == nil {
			g.faces = append(g.faces, built.Faces...)
			g.delta, g.carrierDelta = built.Delta, built.Delta
		}
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
		var built clearance.BodyCarriers
		built, ok, err = clearance.BuildBrepBodyCarriers(budget, pl)
		if ok && err == nil {
			g.faces = append(g.faces, built.Faces...)
			g.delta, g.carrierDelta = built.Delta, built.Delta
		}
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
		var faces []*clearance.CFace
		faces, ok, err = clearance.BuildStitchBodyCarriers(budget, topobuild.Faces(&b.n), pl)
		if ok && err == nil {
			g.faces = append(g.faces, faces...)
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
		ce, ok := clearance.NewCEdge(edgeNode(e))
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
		w, ok := clearance.ShellWitness(faceShellNode(sh))
		if !ok {
			return false, nil
		}
		g.shellWit = append(g.shellWit, w)
	}
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
