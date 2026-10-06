package decad

import (
	"context"
	"errors"
	"math"

	"github.com/lestrrat-3d/decad/internal/clearance"

	"github.com/lestrrat-3d/decad/internal/survey2d"

	"github.com/lestrrat-3d/decad/internal/proofbound"

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
// (internal/proofbound/bounds.go's proofbound.FrameAndPlacementRoundAllow, the identical charge
// topology.go's Vertex.Position already takes), the payload's own proven
// axial or angular displacement, and the per-face tilt a rounded carrier
// normal commits across that face's own extent (clearance.BodyFaceTiltDelta, below).
// addStitchFaces folds in the body's own largest proven vertex bound instead.
// It is exactly zero for an axis-aligned, unplaced, feature-built body, which
// is what keeps every such body's Clearance rows Exact as before.
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
}

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
	nDir := pp.dir(0, 0, 1)
	h := pp.z1 - pp.z0

	var capElems []survey2d.SurveyElem
	maxCoordUpper := 0.0
	for _, loop := range loops {
		for _, w := range loop {
			if err := budget.Step(); err != nil {
				return false, err
			}
			el, ok := walkElem(w.SegmentWalk)
			if !ok {
				return false, nil
			}
			maxCoordUpper = math.Max(maxCoordUpper, w.CoordUpper)
			if !pp.surfaceResult {
				capElems = append(capElems, el)
			}

			if w.IsCircular() {
				f := &clearance.CFace{
					Kind:   clearance.CkCylinder,
					Anchor: pp.point(w.CU, w.CV, pp.z0),
					Axis:   nDir,
					RefU:   pp.dir(1, 0, 0),
					RefV:   pp.dir(0, 1, 0),
					Radius: w.Radius,
					ZWin:   clearance.NewLinWindow(0, h),
					Sweep:  clearance.AngWindow{Full: w.Closed},
				}
				if !w.Closed {
					f.Sweep = clearance.NewAngWindow(w.Th0, w.Th1)
				}
				top := pp.point(w.CU, w.CV, pp.z1)
				f.Box = clearance.BoxUnion(clearance.CircleBox(f.Anchor, nDir, w.Radius), clearance.CircleBox(top, nDir, w.Radius))
				midTh := (w.Th0 + w.Th1) / 2
				f.Wit = append(f.Wit,
					pp.point(w.CU+w.Radius*math.Cos(midTh), w.CV+w.Radius*math.Sin(midTh), (pp.z0+pp.z1)/2),
					pp.point(w.CU+w.Radius*math.Cos(w.Th0), w.CV+w.Radius*math.Sin(w.Th0), pp.z0))
				g.faces = append(g.faces, f)
				continue
			}

			l := math.Hypot(w.EndU-w.StartU, w.EndV-w.StartV)
			if l == 0 {
				return false, nil
			}
			e1, ok := pp.dir(w.EndU-w.StartU, w.EndV-w.StartV, 0).Normalize()
			if !ok {
				return false, nil
			}
			tu, tv := w.TanInU, w.TanInV
			if pp.reflected() {
				tu, tv = -tu, -tv
			}
			nOut, ok := pp.dir(tu, tv, 0).Cross(nDir).Normalize()
			if !ok {
				return false, nil
			}
			f := &clearance.CFace{
				Kind: clearance.CkPlane,
				O:    pp.point(w.StartU, w.StartV, pp.z0),
				U:    e1,
				V:    nDir,
				N:    nOut,
			}
			le0, _ := survey2d.LineElem(0, 0, l, 0)
			le1, _ := survey2d.LineElem(l, 0, l, h)
			le2, _ := survey2d.LineElem(l, h, 0, h)
			le3, _ := survey2d.LineElem(0, h, 0, 0)
			f.Region = clearance.NewRegion2([]survey2d.SurveyElem{le0, le1, le2, le3})
			f.Box = clearance.BoxOf(f.O, f.O.Add(e1.Scale(l)), f.O.Add(nDir.Scale(h)), f.O.Add(e1.Scale(l)).Add(nDir.Scale(h)))
			f.Wit = append(f.Wit, f.O.Add(e1.Scale(l/2)).Add(nDir.Scale(h/2)), f.O)
			g.faces = append(g.faces, f)
		}
	}

	if !pp.surfaceResult {
		region := clearance.NewRegion2(capElems)
		for _, cap := range []struct {
			z    float64
			sign float64
		}{{z: pp.z0, sign: -1}, {z: pp.z1, sign: 1}} {
			f := &clearance.CFace{
				Kind:   clearance.CkPlane,
				O:      pp.point(0, 0, cap.z),
				U:      pp.dir(1, 0, 0),
				V:      pp.dir(0, 1, 0),
				N:      nDir.Scale(cap.sign),
				Region: region,
			}
			f.Box = clearance.CapBox(f)
			f.Wit = clearance.CapWitnesses(f)
			g.faces = append(g.faces, f)
		}
	}
	// g.delta charges the three terms clearance_geom.go's package doc comment
	// and bodyGeom's own doc comment name: the payload's own frame/placement
	// point rounding (every anchor, axis point and witness above came from
	// pp.point), its proven axial displacement (pp.axialDelta — the section's
	// own sectionDelta is already refused above, but the sweep levels' own
	// displacement was never charged here before), and the worst per-face
	// tilt a rounded carrier normal commits (every u, v and n above came from
	// pp.dir). It is exactly zero for an axis-aligned, unplaced, feature-built
	// prism, which is what keeps an ordinary extrude's Clearance rows Exact.
	pointTerm := proofbound.FrameAndPlacementRoundAllow(pp.frame, pp.xform, math.Max(maxCoordUpper, math.Max(math.Abs(pp.z0), math.Abs(pp.z1))))
	g.delta = proofbound.AbsSumUpper(pointTerm, pp.axialDelta(), clearance.BodyFaceTiltDelta(g.faces, pp.frame, pp.xform))
	g.carrierDelta = g.delta
	return true, nil
}

// addRevolveFaces builds the revolved body's faces from its own payload. Its
// cap planes, clearance.AngWindow and witnesses all read rp.phi0/rp.phi1 as the held
// sweep angle rather than the angle the record denotes, and folds
// rp.angularDelta() — scaled by the radial envelope every witness can carry
// it at, verify_gate.go's own reading of the same displacement — into
// bodyGeom.delta below, beside the payload's own frame/placement point
// rounding and the per-face tilt term every plane carrier here can commit
// (bodyGeom's own doc comment).
func (g *bodyGeom) addRevolveFaces(budget *proofbound.WorkBudget, rp revolvePayload) (bool, error) {
	loops, err := revolveLoops(budget, rp)
	if err != nil {
		if errors.Is(err, context.Canceled) || errors.Is(err, context.DeadlineExceeded) {
			return false, err
		}
		if errors.Is(err, ErrUnsupported) {
			// An undecomposable meridian is an unsupported payload, not a
			// failure: newBodyGeom answers with no model rather than a partial
			// one. Other errors remain visible to the caller.
			return false, nil
		}
		return false, err
	}
	b := rp.basis()
	a3p := rp.xform.Apply(b.A3)
	wp := rp.xform.ApplyDir(b.W)
	e0p := rp.xform.ApplyDir(b.E0)
	e1p := rp.xform.ApplyDir(b.E1)
	sweep := clearance.AngWindow{Full: rp.full}
	if !rp.full {
		sweep = clearance.NewAngWindow(rp.phi0, rp.phi1)
	}
	midPhi := (rp.phi0 + rp.phi1) / 2
	onAxis := func(z float64) r3.Vec { return a3p.Add(wp.Scale(z)) }

	var capElems []survey2d.SurveyElem
	maxAxisRadiusUpper := 0.0
	for _, loop := range loops {
		for _, w := range loop {
			if err := budget.Step(); err != nil {
				return false, err
			}
			kind := rp.ax.classify(w.SegmentWalk)
			if el, ok := walkElem(w.SegmentWalk); ok {
				capElems = append(capElems, el)
			}
			maxAxisRadiusUpper = math.Max(maxAxisRadiusUpper, w.AxisRadiusUpper)
			switch kind {
			case wallAxis:
				continue
			case wallCylinder:
				r := (w.StartV + w.EndV) / 2
				f := &clearance.CFace{
					Kind: clearance.CkCylinder, Anchor: a3p, Axis: wp, RefU: e0p, RefV: e1p,
					Radius: r, ZWin: clearance.NewLinWindow(w.StartU, w.EndU), Sweep: sweep,
				}
				f.Box = clearance.BoxUnion(clearance.CircleBox(onAxis(w.StartU), wp, r), clearance.CircleBox(onAxis(w.EndU), wp, r))
				f.Wit = append(f.Wit, rp.point(b, (w.StartU+w.EndU)/2, r, midPhi), rp.point(b, w.StartU, r, rp.phi0))
				g.faces = append(g.faces, f)
			case wallPlane:
				rlo := math.Min(w.StartV, w.EndV)
				rhi := math.Max(w.StartV, w.EndV)
				sign := 1.0
				if w.TanInV < 0 {
					sign = -1
				}
				f := &clearance.CFace{
					Kind: clearance.CkPlane,
					O:    onAxis(w.StartU),
					U:    e0p, V: e1p,
					N: wp.Scale(sign),
				}
				f.Region = clearance.AnnularRegion(rlo, rhi, rp.phi0, rp.phi1, rp.full)
				f.Box = clearance.CapBox(f)
				f.Wit = clearance.CapWitnesses(f)
				g.faces = append(g.faces, f)
			case wallCone:
				dz, dr := w.EndU-w.StartU, w.EndV-w.StartV
				apexZ := w.StartU - w.StartV*dz/dr
				growth := 1.0
				if dz*dr < 0 {
					growth = -1
				}
				f := &clearance.CFace{
					Kind:   clearance.CkCone,
					Anchor: onAxis(apexZ),
					Axis:   wp.Scale(growth),
					RefU:   e0p, RefV: e1p,
					Half:  math.Atan2(math.Abs(dr), math.Abs(dz)),
					ZWin:  clearance.NewLinWindow(math.Abs(w.StartU-apexZ), math.Abs(w.EndU-apexZ)),
					Sweep: sweep,
				}
				f.Box = clearance.BoxUnion(clearance.CircleBox(onAxis(w.StartU), wp, w.StartV), clearance.CircleBox(onAxis(w.EndU), wp, w.EndV))
				f.Wit = append(f.Wit, rp.point(b, (w.StartU+w.EndU)/2, (w.StartV+w.EndV)/2, midPhi))
				g.faces = append(g.faces, f)
				if w.StartV <= 0 || w.EndV <= 0 {
					// The apex sits on the trimmed face: a surface singular
					// point, synthesized as a vertex-like candidate (§3).
					g.verts = append(g.verts, onAxis(apexZ))
				}
			case wallSphere:
				f := &clearance.CFace{
					Kind:   clearance.CkSphere,
					Anchor: onAxis(w.CU),
					Axis:   wp,
					RefU:   e0p, RefV: e1p,
					Radius: w.Radius,
					Merid:  clearance.AngWindow{Full: w.Closed},
					Sweep:  sweep,
				}
				if !w.Closed {
					f.Merid = clearance.NewAngWindow(w.Th0, w.Th1)
				}
				f.Box = [2]r3.Vec{
					f.Anchor.Sub(r3.NewVec(w.Radius, w.Radius, w.Radius)),
					f.Anchor.Add(r3.NewVec(w.Radius, w.Radius, w.Radius)),
				}
				midTh := (w.Th0 + w.Th1) / 2
				f.Wit = append(f.Wit, rp.point(b, w.CU+w.Radius*math.Cos(midTh), math.Max(0, w.Radius*math.Sin(midTh)), midPhi))
				g.faces = append(g.faces, f)
			case wallTorus:
				f := &clearance.CFace{
					Kind:   clearance.CkTorus,
					Anchor: onAxis(w.CU),
					Axis:   wp,
					RefU:   e0p, RefV: e1p,
					Radius:  w.Radius,
					Major:   w.CV,
					Merid:   clearance.AngWindow{Full: w.Closed},
					Sweep:   sweep,
					Spindle: w.Radius >= w.CV-clearance.ClrAngTol*math.Max(1, w.CV),
				}
				if !w.Closed {
					f.Merid = clearance.NewAngWindow(w.Th0, w.Th1)
				}
				spineBox := clearance.CircleBox(f.Anchor, wp, f.Major)
				pad := r3.NewVec(w.Radius, w.Radius, w.Radius)
				f.Box = [2]r3.Vec{spineBox[0].Sub(pad), spineBox[1].Add(pad)}
				midTh := (w.Th0 + w.Th1) / 2
				f.Wit = append(f.Wit, rp.point(b, w.CU+w.Radius*math.Cos(midTh), w.CV+w.Radius*math.Sin(midTh), midPhi))
				g.faces = append(g.faces, f)
				// A spindle patch reaching the axis at a walk endpoint has a
				// singular axis-collapse point there, synthesized like a cone
				// apex (§3).
				if !w.Closed && w.StartV == 0 {
					g.verts = append(g.verts, onAxis(w.CU+w.Radius*math.Cos(w.Th0)))
				}
				if !w.Closed && w.EndV == 0 {
					g.verts = append(g.verts, onAxis(w.CU+w.Radius*math.Cos(w.Th1)))
				}
			}
		}
	}

	if !rp.full {
		region := clearance.NewRegion2(capElems)
		for _, cap := range []struct {
			phi  float64
			sign float64
		}{{phi: rp.phi0, sign: -1}, {phi: rp.phi1, sign: 1}} {
			sin, cos := math.Sincos(cap.phi)
			radial := e0p.Scale(cos).Add(e1p.Scale(sin))
			vel := e0p.Scale(-sin).Add(e1p.Scale(cos))
			f := &clearance.CFace{
				Kind:   clearance.CkPlane,
				O:      a3p,
				U:      wp,
				V:      radial,
				N:      vel.Scale(cap.sign),
				Region: region,
			}
			f.Box = clearance.CapBox(f)
			f.Wit = clearance.CapWitnesses(f)
			g.faces = append(g.faces, f)
		}
	}
	// g.delta mirrors addPrismFaces' own three terms, over this payload's own
	// axis-symmetric construction: the frame/placement point rounding every
	// anchor, axis point and witness above took through rp.point/onAxis, the
	// angular displacement scaled by the radial envelope every such point can
	// carry it at (the same reading verify_gate.go's own bodyGateDiameter
	// arm takes for the identical displacement), and the worst per-face tilt
	// a rounded wallPlane/cap carrier normal commits. It is exactly zero for
	// an axis-aligned, unplaced, full-turn revolve, which is what keeps an
	// ordinary revolve's Clearance rows Exact.
	pointTerm := revolveVertexFrameLiftAllow(rp, maxAxisRadiusUpper)
	angularTerm := proofbound.ProductUpper(maxAxisRadiusUpper, rp.angularDelta())
	tiltTerm := clearance.BodyFaceTiltDelta(g.faces, rp.frame, rp.xform)
	g.delta = proofbound.AbsSumUpper(pointTerm, angularTerm, tiltTerm)
	g.carrierDelta = g.delta
	if rp.full {
		// A full turn builds no cap faces and gives every carrier the full
		// angular window, so the end angles the angular term charges place
		// only seams and witnesses on the complete surface of revolution.
		g.carrierDelta = proofbound.AbsSumUpper(pointTerm, tiltTerm)
	}
	return true, nil
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
