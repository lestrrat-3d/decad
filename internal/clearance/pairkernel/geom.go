package pairkernel

import (
	"math"

	"github.com/lestrrat-3d/decad/internal/breprecord"
	"github.com/lestrrat-3d/decad/internal/clearance"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/revolveplan"
	"github.com/lestrrat-3d/decad/internal/stitchrecord"
	"github.com/lestrrat-3d/decad/internal/stitchtess"
	"github.com/lestrrat-3d/decad/internal/stitchweld"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/decad/internal/wallsurvey"
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
// docs/payload-verification-design.md §2.3's delta(X)). The prism and revolve
// carrier builders fold in the payload's own frame/placement point-rounding
// (the largest clearance.CFace.LiftRound its carriers recorded, each measured
// exactly per lifted point, the same charge the root Vertex.Position takes),
// the payload's own proven axial or angular displacement, and the per-face
// tilt a rounded carrier normal commits across that face's own extent
// (clearance.BodyFaceTiltDelta); the revolve carrier builder adds how far its
// carriers sit off the recorded meridian swept about the recorded axis
// (clearance.RevolveCarrierResult's AxisGap). The stitch arm folds in the
// body's own largest proven vertex bound instead. It is exactly zero for an
// unplaced, axis-aligned, feature-built body whose lifts are exact for its own
// coordinates, which is what keeps every such body's Clearance rows Exact.
type bodyGeom struct {
	body     *topo.Body
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
	// reads carries (the vertex node's own Bound). A vertex is lifted through
	// its own float construction — a partial revolve's cap vertex through the
	// held cosine and sine of its end angle, times a radius that can be the
	// whole distance to a far axis — so it can sit farther from the record
	// than any carrier does. widenDelta folds it in; the exact certificates of
	// §6 read carriers alone and keep reading delta and carrierDelta.
	vertexDelta float64
}

// widenDelta is the displacement DeltaWiden charges this body once candidate
// aggregation completes: every candidate pairs carrier points, edge points or
// vertices, so the larger of delta and vertexDelta bounds how far any of them
// sits from the boundary the payload denotes. It is delta itself wherever no
// vertex bound exceeds it, which keeps every row that did not read an
// uncovered vertex exactly as it was.
func (g *bodyGeom) widenDelta() float64 { return math.Max(g.delta, g.vertexDelta) }

// newBodyGeomBudget is the cancellable form. Building a body's carrier faces,
// edges and vertices is linear in the body but is not free, and
// §7.2 carries the context through the entire read-only path — so the whole
// build steps the caller's budget rather than making cancellation wait for both
// operands to finish.
func newBodyGeomBudget(budget *proofbound.WorkBudget, b *topo.Body) (*bodyGeom, bool, error) {
	if err := budget.Err(); err != nil {
		return nil, false, err
	}
	if len(b.Lumps) != 1 {
		return nil, false, nil
	}
	g := &bodyGeom{body: b}
	var ok bool
	var err error
	switch pl := b.Payload.(type) {
	case prismrecord.Payload:
		var built clearance.BodyCarriers
		built, ok, err = clearance.BuildPrismBodyCarriers(budget, pl)
		if ok && err == nil {
			g.faces = append(g.faces, built.Faces...)
			g.delta, g.carrierDelta = built.Delta, built.Delta
		}
	case revolveplan.Payload:
		if pl.SurfaceResult {
			// A revolve sheet reaches no pair today (§9.3 stops it on box
			// separation before Pair ever sees one), but
			// the revolve carrier builder still hands back a closed model for a partial
			// sweep: it unconditionally builds the two cap faces
			// Table W says a surface-result revolve omits. Building that
			// wrong model now would bless a future caller's clearance against
			// caps the body does not have, so this arm refuses one outright —
			// the reject-only answer (CLAUDE.md) where building a model is
			// not. Only a plain, solid revolve reaches the builder.
			return nil, false, nil
		}
		var built wallsurvey.RevolveClearanceCarriers
		built, ok, err = wallsurvey.BuildRevolveClearanceCarriers(budget, pl)
		if ok && err == nil {
			g.faces = append(g.faces, built.Faces...)
			g.verts = append(g.verts, built.Vertices...)
			g.delta, g.carrierDelta = built.Delta, built.CarrierDelta
		}
	case breprecord.Payload:
		var built clearance.BodyCarriers
		built, ok, err = clearance.BuildBrepBodyCarriers(budget, pl)
		if ok && err == nil {
			g.faces = append(g.faces, built.Faces...)
			g.delta, g.carrierDelta = built.Delta, built.Delta
		}
	case stitchrecord.Payload:
		// Refuse outright unless the recorded triangle set exists (an
		// all-planar body, open or closed — internal/stitchtess/stitch.go's own gate).
		// A bounded body — placed (internal/stitchrecord/build.go's proofbound.RigidRoundAllow widens every
		// vertex bound) or closed by the CURVE weld certificate at a nonzero
		// class bound — does not refuse on that alone: bodyGeom.delta
		// exists for it to charge its own worst vertex bound against
		// (stitchweld.MaxBound over stitchtess.VertexBounds), the
		// identical charge a placed prism's frame/placement rounding takes
		// (bodyGeom's own doc comment), so this arm needs no standing narrower
		// than the prism arm's.
		if pl.Tris == nil {
			return nil, false, nil
		}
		maxVertexBound, mErr := stitchweld.MaxBound(budget, stitchtess.VertexBounds(b))
		if mErr != nil {
			return nil, false, mErr
		}
		var faces []*clearance.CFace
		faces, ok, err = clearance.BuildStitchBodyCarriers(budget, topobuild.Faces(b), pl)
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
func (g *bodyGeom) addTopology(budget *proofbound.WorkBudget, b *topo.Body) (bool, error) {
	for _, e := range topobuild.Edges(b) {
		if err := budget.Step(); err != nil {
			return false, err
		}
		ce, ok := clearance.NewCEdge(e)
		if !ok {
			return false, nil
		}
		if ce != nil {
			g.edges = append(g.edges, ce)
		}
	}
	for _, v := range topobuild.Vertices(b) {
		if err := budget.Step(); err != nil {
			return false, err
		}
		g.verts = append(g.verts, v.Position)
		if m := v.Bound.Mag(); m > g.vertexDelta {
			g.vertexDelta = m
		}
	}
	for _, l := range b.Lumps {
		for _, sh := range l.Shells {
			if err := budget.Step(); err != nil {
				return false, err
			}
			w, ok := clearance.ShellWitness(sh)
			if !ok {
				return false, nil
			}
			g.shellWit = append(g.shellWit, w)
		}
	}
	return true, nil
}
