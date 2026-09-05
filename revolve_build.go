package decad

import (
	"context"
	"fmt"
	"math"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// This file builds a revolve's body from its revolvePayload: the wall surface
// each recorded segment sweeps, the two cap faces a partial sweep closes with,
// the poles and seams a full sweep instead joins, and the measurements the
// finished body publishes.
//
// evalRevolveContextWork is the whole build and states the order.
// buildRevolveLoop is the per-loop walk, and revLoopParts is what it hands
// back so a full sweep and a partial one assemble the same parts differently
// rather than through two builds. Every face carries the bound its own
// surface was built from. See docs/evaluator-design.md §6.

// revolvePayload is the evaluator's own record of a revolved body: the
// recorded region, the plane frame, the oriented plane-local axis, the sweep
// interval (right-handed about the axis, phi1 > phi0; exactly 2π apart when
// full), and the accumulated rigid placement. Every measurement and the
// whole topology derive from it, which is what makes Placed exact: it
// re-evaluates the same payload under the composed motion
// (docs/evaluator-design.md §8).
type revolvePayload struct {
	profile    ProfileRecord
	frame      r3.Frame
	ax         axisFrame
	phi0, phi1 float64
	full       bool
	xform      r3.Transform
}

// transform is the accumulated rigid placement.
func (rp revolvePayload) transform() r3.Transform { return rp.xform }

// placed re-evaluates the same record under the composed motion.
func (rp revolvePayload) placed(ctx context.Context, d *Document, ref StepRef, composed r3.Transform) (*Body, error) {
	rp.xform = composed
	return evalRevolveContext(ctx, d, ref, rp)
}

// revolveBasis is the unplaced world anchor of the sweep: a3 the axis
// origin, w the unit axis direction, e0 the in-plane radial direction at
// sweep angle zero, and e1 = w × e0 the sweep-velocity direction at zero —
// so a rotation by +φ about w carries e0 toward e1, the right-handed sense
// Along means.
type revolveBasis struct {
	a3, w, e0, e1 r3.Vec
}

// basis derives the sweep basis from the plane frame and the axis frame.
func (rp revolvePayload) basis() revolveBasis {
	a3 := rp.frame.ToWorldUV(rp.ax.aU, rp.ax.aV)
	w := rp.frame.U().Scale(rp.ax.dU).Add(rp.frame.V().Scale(rp.ax.dV))
	e0 := rp.frame.U().Scale(-rp.ax.dV).Add(rp.frame.V().Scale(rp.ax.dU))
	return revolveBasis{a3: a3, w: w, e0: e0, e1: w.Cross(e0)}
}

// revolveCentroidGeometryBound bounds the centroid independently of the
// Pappus quotient. Every material point starts in the recorded profile plane,
// rotates about the resolved axis, then passes through a rigid placement. The
// L1 envelopes use three times an input L1 norm for any orthogonal map.
func revolveCentroidGeometryBound(rp revolvePayload, held r3.Vec, work *freeformWork) (float64, error) {
	coordUpper, err := profileCoordinateUpper(rp.profile, work, nil)
	if err != nil {
		return 0, err
	}
	originUpper := vecL1(rp.frame.Origin())
	profileUpper := absSumUpper(
		originUpper,
		productUpper(vecL1(rp.frame.U()), coordUpper),
		productUpper(vecL1(rp.frame.V()), coordUpper),
	)
	aUUpper := absSumUpper(rp.ax.aU, rp.ax.aUBound)
	aVUpper := absSumUpper(rp.ax.aV, rp.ax.aVBound)
	axisUpper := absSumUpper(
		originUpper,
		productUpper(vecL1(rp.frame.U()), aUUpper),
		productUpper(vecL1(rp.frame.V()), aVUpper),
	)
	rotatedUpper := absSumUpper(productUpper(3, profileUpper), productUpper(4, axisUpper))
	placedUpper := absSumUpper(productUpper(3, rotatedUpper), vecL1(rp.xform.Translation()))
	return absSumUpper(vecL1(held), placedUpper), nil
}

// point places the axis-frame point (z, ρ) at sweep angle φ into placed
// world space.
func (rp revolvePayload) point(b revolveBasis, z, rho, phi float64) r3.Vec {
	sin, cos := math.Sincos(phi)
	radial := b.e0.Scale(cos).Add(b.e1.Scale(sin))
	return rp.xform.Apply(b.a3.Add(b.w.Scale(z)).Add(radial.Scale(rho)))
}

// reflected reports whether the accumulated placement flips handedness — a
// reflected solid's rotational senses invert with it, and every swept edge's
// axis carries the corrected sign.
func (rp revolvePayload) reflected() bool { return rp.xform.IsReflection() }

// axisMoments re-references the plane-origin region integrals into the axis
// frame: q = ∫ρ dA (Pappus's second theorem reads volume off it), mzr =
// ∫zρ dA (the solid centroid's axial position), and mrr = ∫ρ² dA (the
// partial sweep's in-plane centroid term) — the §4 first, second and mixed
// moments with their source and arithmetic bounds (docs/evaluator-design.md
// §6).
func axisMoments(ig regionIntegrals, ax axisFrame) (boundedScalar, boundedScalar, boundedScalar) {
	aU, aV := measuredScalar(ax.aU, ax.aUBound), measuredScalar(ax.aV, ax.aVBound)
	dU, dV := measuredScalar(ax.dU, ax.dUBound), measuredScalar(ax.dV, ax.dVBound)
	nU, nV := measuredScalar(-ax.dV, ax.dVBound), measuredScalar(ax.dU, ax.dUBound)
	area := measuredScalar(ig.area, ig.areaBound)
	mu := measuredScalar(ig.mu, ig.muBound)
	mv := measuredScalar(ig.mv, ig.mvBound)
	muu := measuredScalar(ig.muu, ig.muuBound)
	muv := measuredScalar(ig.muv, ig.muvBound)
	mvv := measuredScalar(ig.mvv, ig.mvvBound)

	iuu := boundedAdd(
		boundedSub(muu, boundedMul(boundedMul(exactScalar(2), aU), mu)),
		boundedMul(boundedMul(aU, aU), area),
	)
	iuv := boundedAdd(
		boundedSub(boundedSub(muv, boundedMul(aU, mv)), boundedMul(aV, mu)),
		boundedMul(boundedMul(aU, aV), area),
	)
	ivv := boundedAdd(
		boundedSub(mvv, boundedMul(boundedMul(exactScalar(2), aV), mv)),
		boundedMul(boundedMul(aV, aV), area),
	)
	q := boundedAdd(
		boundedMul(nU, boundedSub(mu, boundedMul(aU, area))),
		boundedMul(nV, boundedSub(mv, boundedMul(aV, area))),
	)
	mzr := boundedAdd(
		boundedAdd(
			boundedMul(boundedMul(dU, nU), iuu),
			boundedMul(boundedAdd(boundedMul(dU, nV), boundedMul(dV, nU)), iuv),
		),
		boundedMul(boundedMul(dV, nV), ivv),
	)
	mrr := boundedAdd(
		boundedAdd(
			boundedMul(boundedMul(nU, nU), iuu),
			boundedMul(boundedMul(exactScalar(2), boundedMul(nU, nV)), iuv),
		),
		boundedMul(boundedMul(nV, nV), ivv),
	)
	return q, mzr, mrr
}

func boundedRevolveSweep(phi0, phi1 float64) boundedScalar {
	sweep := boundedSub(exactScalar(phi1), exactScalar(phi0))
	sweep.bound = math.Max(sweep.bound, conservativeValueError(sweep.value, twoPiUpper()))
	return sweep
}

// evalRevolve builds the analytic revolved body from the payload: side
// surfaces of revolution per boundary segment, caps only for a partial
// sweep, shared edges and vertices, and bounded mass measurements
// (docs/evaluator-design.md §6). The payload's segment kinds are line, circle
// and arc; anything else has already been rejected by the mass-property
// integrals it runs first.
func evalRevolve(d *Document, ref StepRef, rp revolvePayload) (*Body, error) {
	return evalRevolveWork(d, ref, newFreeformWork(), rp)
}

// evalRevolveWork is the build an operation that already holds this record's
// free-form work counter runs: the preflight below and every walkOf under it
// continue that counter rather than open a second ceiling on the same record.
func evalRevolveWork(d *Document, ref StepRef, work *freeformWork, rp revolvePayload) (*Body, error) {
	return evalRevolveContextWork(context.Background(), d, ref, rp, work)
}

func evalRevolveContext(ctx context.Context, d *Document, ref StepRef, rp revolvePayload) (*Body, error) {
	return evalRevolveContextWork(ctx, d, ref, rp, newFreeformWork())
}

func evalRevolveContextWork(ctx context.Context, d *Document, ref StepRef, rp revolvePayload, work *freeformWork) (*Body, error) {
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	ig, err := rp.profile.evaluatorIntegralsUncheckedContext(ctx, momentSecondOrder, work)
	if err != nil {
		return nil, err
	}
	if ig.area <= 0 {
		return nil, fmt.Errorf(`%w: the recorded region encloses no area`, ErrDegenerate)
	}
	sweep := boundedRevolveSweep(rp.phi0, rp.phi1)
	dphi := sweep.value
	if dphi <= 0 {
		return nil, fmt.Errorf(`%w: the sweep interval is empty`, ErrDegenerate)
	}
	q, mzr, mrr := axisMoments(ig, rp.ax)
	if q.value <= 0 {
		return nil, fmt.Errorf(`%w: the region has no material off the revolve axis`, ErrDegenerate)
	}

	b := rp.basis()
	body := &Body{doc: d, origin: FeatureRef{Step: ref, Role: roleBody}, solid: true}

	// Partial sweeps get two planar cap faces; a full revolution has none.
	var capStart, capEnd *Face
	if !rp.full {
		startFrame, err := rp.capFrame(b, rp.phi0, true)
		if err != nil {
			return nil, err
		}
		endFrame, err := rp.capFrame(b, rp.phi1, false)
		if err != nil {
			return nil, err
		}
		capStart = &Face{
			surface:   Plane{Frame: startFrame},
			origins:   []FeatureRef{{Step: ref, Role: roleCapStart}},
			body:      body,
			area:      ig.area,
			areaBound: ig.areaBound,
		}
		capEnd = &Face{
			surface:   Plane{Frame: endFrame},
			origins:   []FeatureRef{{Step: ref, Role: roleCapEnd}},
			body:      body,
			area:      ig.area,
			areaBound: ig.areaBound,
		}
	}

	sideArea := boundedScalar{}
	loops := append([]LoopRecord{rp.profile.Outer}, rp.profile.Holes...)
	perLoop := make([][]*Face, len(loops))
	for li, loop := range loops {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		parts, err := buildRevolveLoop(ctx, body, ref, rp, b, li, loop, work)
		if err != nil {
			return nil, err
		}
		perLoop[li] = parts.faces
		sideArea = boundedAdd(sideArea, parts.area)
		if !rp.full {
			capStart.loops = append(capStart.loops, &Loop{coedges: parts.startCo, outer: li == 0})
			capEnd.loops = append(capEnd.loops, &Loop{coedges: parts.endCo, outer: li == 0})
		}
	}

	// Shells: a partial sweep's caps connect every loop's walls into one
	// boundary, but a full revolution encloses each profile hole as its own
	// toroidal void — a separate shell, and a void one (evaluator §3).
	var shells []*Shell
	if rp.full {
		var err error
		shells, err = fullRevolveShellsContext(ctx, perLoop)
		if err != nil {
			return nil, err
		}
	} else {
		var faces []*Face
		for _, group := range perLoop {
			faces = append(faces, group...)
		}
		faces = append(faces, capStart, capEnd)
		if err := attachFaceLoopsContext(ctx, []*Face{capStart, capEnd}); err != nil {
			return nil, err
		}
		shells = []*Shell{{faces: faces}}
	}
	body.lumps = []*Lump{{shells: shells}}

	// Measurements — Pappus with the profile and float-evaluation bounds
	// carried through (docs/evaluator-design.md §6).
	area := sideArea
	if !rp.full {
		area = boundedAdd(area, boundedMul(exactScalar(2), measuredScalar(ig.area, ig.areaBound)))
	}
	volume := boundedMul(q, sweep)
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

	axial := boundedDiv(mzr, q)
	cen := b.a3.Add(b.w.Scale(axial.value))
	centroidScale := absSumUpper(vecMaxAbs(b.a3), axial.value)
	centroidBound := absSumUpper(axial.bound, radius3D(analyticRoundBound(centroidScale)))
	if !rp.full {
		// The in-plane term is the swept radial direction integrated over
		// the interval — closed form in the sweep angle; a full turn's is
		// identically zero, which is what puts its centroid on the axis.
		sin1 := boundedSin(exactScalar(rp.phi1))
		cos1 := boundedCos(exactScalar(rp.phi1))
		sin0 := boundedSin(exactScalar(rp.phi0))
		cos0 := boundedCos(exactScalar(rp.phi0))
		rx := boundedSub(sin1, sin0)
		ry := boundedSub(cos0, cos1)
		radial := b.e0.Scale(rx.value).Add(b.e1.Scale(ry.value))
		radialBound := radius2D(rx.bound, ry.bound)
		radialScale := boundedDiv(mrr, boundedMul(sweep, q))
		cen = cen.Add(radial.Scale(radialScale.value))
		radialUpper := vecL1(radial)
		centroidBound = absSumUpper(
			centroidBound,
			productUpper(radialScale.value, radialBound),
			productUpper(radialUpper, radialScale.bound),
			radius3D(analyticRoundBound(productUpper(radialScale.value, radialUpper))),
		)
	}
	centroidBound = absSumUpper(
		centroidBound,
		rigidRoundAllow(vecMaxAbs(cen), vecMaxAbs(rp.xform.Translation())),
	)
	centroidValue := rp.xform.Apply(cen)
	geometryBound, err := revolveCentroidGeometryBound(rp, centroidValue, work)
	if err != nil {
		return nil, err
	}
	centroidBound = math.Min(centroidBound, geometryBound)
	body.centroid = VecMeasurement{
		Value:     centroidValue,
		Exactness: exactnessOf(centroidBound),
		Bound:     units.Millimeters(centroidBound),
	}

	bounds, err := revolveBoundsContext(ctx, rp, work)
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
	body.payload = rp
	return body, nil
}

// capFrame is the plane frame of a cap at sweep angle φ, under the
// placement. The start cap's outward (material-leaving) normal points
// against the sweep, the end cap's along it; the axes are ordered so the
// frame's normal IS the outward normal, and a reflected placement swaps them
// once more (a reflection flips the cross product's handedness).
func (rp revolvePayload) capFrame(b revolveBasis, phi float64, start bool) (r3.Frame, error) {
	sin, cos := math.Sincos(phi)
	radial := b.e0.Scale(cos).Add(b.e1.Scale(sin))
	// (w, radial) has w × radial = the rotated plane normal — the sweep
	// velocity direction, the END cap's outward normal.
	u3, v3 := b.w, radial
	if start {
		u3, v3 = v3, u3
	}
	if rp.reflected() {
		u3, v3 = v3, u3
	}
	f, err := r3.NewFrame(rp.xform.Apply(b.a3), rp.xform.ApplyDir(u3), rp.xform.ApplyDir(v3))
	if err != nil {
		return r3.Frame{}, fmt.Errorf(`%w: the placed cap frame is degenerate: %s`, ErrDegenerate, err)
	}
	return f, nil
}

// revLoopParts is what one recorded loop contributes to the revolved body.
type revLoopParts struct {
	faces   []*Face
	startCo []coedge      // the loop's start-cap coedges, walk order (partial only)
	endCo   []coedge      // the loop's end-cap coedges, walk order (partial only)
	area    boundedScalar // the loop's side-face area
}

// revJunction is one junction between consecutive walks: the shared point in
// axis coordinates and what it sweeps to — nothing on the axis, a full
// latitude circle for a full revolution, an arc between the two caps for a
// partial sweep.
type revJunction struct {
	z, rho float64
	onAxis bool
	v0, v1 *Vertex // partial: start-/end-cap vertices (one shared on the axis)
	arc    *Edge   // partial: the swept junction arc; nil on the axis
	lat    *Edge   // full: the latitude circle; nil on the axis
}

// revolveWalks is one recorded loop resolved the way a revolve reads it: the
// coalesced walks in AXIS coordinates, what each of them sweeps, the same
// walks still in PLANE-local coordinates indexed by recorded segment, and
// whether the loop is a single whole closed curve.
//
// plane is kept beside walks because the two answer different questions. A
// build needs only the axis coordinates; a proof about how far a held sample
// sits from the point the RECORD denotes needs the recorded plane coordinates
// the axis re-expression consumed, since axisFrame.walk states no bound on the
// axial coordinate it computes and snaps a near-axis radial one to zero
// outright (docs/tessellation-design.md §8's deltaC).
type revolveWalks struct {
	walks        []sideWalk
	kinds        []wallKind
	plane        []segmentWalk
	singleClosed bool
}

// revolveLoopWalks resolves one recorded loop into the walks a revolve reads,
// so the builder and the tessellator consume the SAME resolution rather than
// two copies of it (docs/tessellation-design.md §3: the mesh reads the
// evaluator's payload, never live sketch input). what names the caller in the
// free-form refusal.
func revolveLoopWalks(ctx context.Context, rp revolvePayload, loop LoopRecord, work *freeformWork, what string) (revolveWalks, error) {
	if err := ctx.Err(); err != nil {
		return revolveWalks{}, err
	}
	if len(loop.Segments) == 0 {
		return revolveWalks{}, fmt.Errorf(`%w: a recorded loop holds no segments`, ErrDegenerate)
	}
	raw := make([]sideWalk, len(loop.Segments))
	plane := make([]segmentWalk, len(loop.Segments))
	for i, seg := range loop.Segments {
		if err := ctx.Err(); err != nil {
			return revolveWalks{}, err
		}
		w, err := walkOf(seg, work)
		if err != nil {
			return revolveWalks{}, err
		}
		if err := requireAnalyticWalk(w, what); err != nil {
			return revolveWalks{}, err
		}
		plane[i] = w
		raw[i] = sideWalk{segmentWalk: rp.ax.walk(w), segs: []int{i}}
	}
	walks, err := coalesceWalksContext(ctx, raw)
	if err != nil {
		return revolveWalks{}, err
	}
	kinds := make([]wallKind, len(walks))
	for i, w := range walks {
		kinds[i] = rp.ax.classify(w.segmentWalk)
	}
	return revolveWalks{
		walks:        walks,
		kinds:        kinds,
		plane:        plane,
		singleClosed: len(walks) == 1 && walks[0].closed,
	}, nil
}

// buildRevolveLoop builds one loop's side faces with shared vertices and
// edges, returning the faces, the two caps' coedges in walk order, and the
// loop's side area.
func buildRevolveLoop(ctx context.Context, body *Body, ref StepRef, rp revolvePayload, b revolveBasis, li int, loop LoopRecord, work *freeformWork) (revLoopParts, error) {
	resolved, err := revolveLoopWalks(ctx, rp, loop, work, "the revolve wall build")
	if err != nil {
		return revLoopParts{}, err
	}
	walks, kinds, singleClosed := resolved.walks, resolved.kinds, resolved.singleClosed
	n := len(walks)
	sweep := boundedRevolveSweep(rp.phi0, rp.phi1)
	dphi := sweep.value
	sweepSign := 1.0
	if rp.reflected() {
		sweepSign = -1
	}
	wDir := rp.xform.ApplyDir(b.w)

	// Junction vertices and swept edges: junction i sits at walk i's start
	// (== walk i−1's end). A single whole closed curve has none; a junction
	// on the axis sweeps to a single point — no edge, and for a partial
	// sweep one vertex shared by both caps. Convexity comes from the 2D
	// turn, exactly as the prism's vertical edges: a positive cross of the
	// incoming and outgoing tangents is a convex edge on the outer loop and
	// works out identically for hole loops walked clockwise.
	var js []revJunction
	if !singleClosed {
		js = make([]revJunction, n)
		for i, w := range walks {
			if err := ctx.Err(); err != nil {
				return revLoopParts{}, err
			}
			j := revJunction{z: w.startU, rho: w.startV, onAxis: w.startV == 0}
			prev := walks[(i+n-1)%n]
			turn := prev.tanOutU*w.tanInV - prev.tanOutV*w.tanInU
			center := rp.point(b, j.z, 0, 0)
			switch {
			case rp.full && !j.onAxis:
				seam := &Vertex{position: rp.point(b, j.z, j.rho, rp.phi0), bound: units.Millimeters(0)}
				latitudeLength := 2 * math.Pi * j.rho
				j.lat = &Edge{
					curve:       Circle3{Center: center, Axis: wDir.Scale(sweepSign), Radius: units.Millimeters(j.rho)},
					start:       seam,
					end:         seam,
					convex:      turn > 0,
					length:      latitudeLength,
					lengthBound: conservativeValueError(latitudeLength, productUpper(w.axisRadiusUpper, twoPiUpper())),
				}
			case !rp.full:
				j.v0 = &Vertex{position: rp.point(b, j.z, j.rho, rp.phi0), bound: units.Millimeters(0)}
				j.v1 = j.v0
				if !j.onAxis {
					j.v1 = &Vertex{position: rp.point(b, j.z, j.rho, rp.phi1), bound: units.Millimeters(0)}
					arcLength := j.rho * dphi
					dphiUpper := absSumUpper(math.Abs(dphi), sweep.bound)
					j.arc = &Edge{
						curve:       Arc3{Center: center, Axis: wDir.Scale(sweepSign), Radius: units.Millimeters(j.rho)},
						start:       j.v0,
						end:         j.v1,
						convex:      turn > 0,
						length:      arcLength,
						lengthBound: conservativeValueError(arcLength, productUpper(w.axisRadiusUpper, dphiUpper)),
					}
				}
			}
			js[i] = j
		}
	}

	// Cap edges (partial sweeps only): each walk's copy in each cap plane.
	// A line lying on the axis does not move when swept, so its two copies
	// coincide: ONE edge, shared by both caps — that is how the caps of a
	// solid wedge meet along the axis. Its dihedral is the sweep angle
	// itself.
	holeLoop := li != 0
	var cap0, cap1 []*Edge
	if !rp.full {
		cap0 = make([]*Edge, n)
		cap1 = make([]*Edge, n)
		for i, w := range walks {
			if err := ctx.Err(); err != nil {
				return revLoopParts{}, err
			}
			if kinds[i] == wallAxis {
				shared := &Edge{
					curve:       Line3{},
					start:       js[i].v0,
					end:         js[(i+1)%n].v0,
					convex:      dphi < math.Pi,
					length:      w.length,
					lengthBound: w.lengthBound,
				}
				cap0[i], cap1[i] = shared, shared
				continue
			}
			var vs0, ve0, vs1, ve1 *Vertex
			if !singleClosed {
				vs0, ve0 = js[i].v0, js[(i+1)%n].v0
				vs1, ve1 = js[i].v1, js[(i+1)%n].v1
			}
			cap0[i] = rp.capEdge(b, w.segmentWalk, singleClosed, vs0, ve0, rp.phi0, holeLoop)
			cap1[i] = rp.capEdge(b, w.segmentWalk, singleClosed, vs1, ve1, rp.phi1, holeLoop)
		}
	}

	parts := revLoopParts{}
	for i, w := range walks {
		if err := ctx.Err(); err != nil {
			return revLoopParts{}, err
		}
		if kinds[i] == wallAxis {
			if !rp.full {
				parts.startCo = append(parts.startCo, coedge{edge: cap0[i], forward: true})
				parts.endCo = append(parts.endCo, coedge{edge: cap1[i], forward: true})
			}
			continue
		}
		surf, reversed, err := rp.wallSurface(b, w.segmentWalk, kinds[i])
		if err != nil {
			return revLoopParts{}, err
		}
		origins, err := sideOriginsContext(ctx, ref, li, w.segs)
		if err != nil {
			return revLoopParts{}, err
		}
		faceArea := boundedMul(walkAxisMoment(w.segmentWalk, kinds[i]), sweep)
		face := &Face{
			surface:   surf,
			origins:   origins,
			body:      body,
			area:      faceArea.value,
			areaBound: faceArea.bound,
			reversed:  reversed,
		}
		switch {
		case rp.full && singleClosed:
			// A whole closed generator swept a full turn bounds a closed
			// surface — a torus needs no boundary loop at all.
		case rp.full:
			face.loops = fullRevLoops(js[i], js[(i+1)%n], kinds[i])
		case singleClosed:
			// A closed generator's partial sweep is bounded by its two cap
			// copies — two loops, one whole closed edge each, mirroring the
			// prism's whole-circle two-loop discipline.
			face.loops = []*Loop{
				{coedges: []coedge{{edge: cap0[i], forward: true}}, outer: true},
				{coedges: []coedge{{edge: cap1[i], forward: false}}, outer: true},
			}
		default:
			co := []coedge{{edge: cap0[i], forward: true}}
			if a := js[(i+1)%n].arc; a != nil {
				co = append(co, coedge{edge: a, forward: true})
			}
			co = append(co, coedge{edge: cap1[i], forward: false})
			if a := js[i].arc; a != nil {
				co = append(co, coedge{edge: a, forward: false})
			}
			face.loops = []*Loop{{coedges: co, outer: true}}
		}
		if err := attachFaceLoopsContext(ctx, []*Face{face}); err != nil {
			return revLoopParts{}, err
		}
		parts.faces = append(parts.faces, face)
		parts.area = boundedAdd(parts.area, faceArea)
		if !rp.full {
			parts.startCo = append(parts.startCo, coedge{edge: cap0[i], forward: true})
			parts.endCo = append(parts.endCo, coedge{edge: cap1[i], forward: true})
		}
	}
	return parts, nil
}

func fullRevolveShellsContext(ctx context.Context, perLoop [][]*Face) ([]*Shell, error) {
	var shells []*Shell
	for li, group := range perLoop {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		if len(group) == 0 {
			continue
		}
		shells = append(shells, &Shell{faces: group, void: li != 0})
	}
	return shells, nil
}

// fullRevLoops assembles a full-revolution side face's boundary loops from
// its junctions' latitude circles — none on the axis (a pole or an apex
// bounds nothing), and for a planar annulus the inner circle is genuinely a
// hole; every other face's circles are both outer, like the prism's whole
// cylinder.
func fullRevLoops(j0, j1 revJunction, kind wallKind) []*Loop {
	outer0, outer1 := true, true
	if kind == wallPlane && !j0.onAxis && !j1.onAxis {
		if j0.rho < j1.rho {
			outer0 = false
		} else {
			outer1 = false
		}
	}
	var loops []*Loop
	if j0.lat != nil {
		loops = append(loops, &Loop{coedges: []coedge{{edge: j0.lat, forward: true}}, outer: outer0})
	}
	if j1.lat != nil {
		loops = append(loops, &Loop{coedges: []coedge{{edge: j1.lat, forward: false}}, outer: outer1})
	}
	if len(loops) == 2 && !loops[0].outer {
		loops[0], loops[1] = loops[1], loops[0]
	}
	return loops
}

// capEdge is one boundary walk's copy in the cap plane at sweep angle φ: a
// straight walk stays a line, a circular one an arc — or a whole circle
// with a seam vertex, closing on itself. The cap edge sits between the cap
// and the side face it bounds; the material across it is a quarter-turn wedge
// everywhere, so the material angle decides nothing — a rim reports the
// WALKED-BOUNDARY convexity instead (topology.go, Edge.IsConvex), which is the
// side the wall's material lies on. A CIRCULAR walk decides that by its own
// sense (a clockwise arc keeps the
// material outside the sphere/torus it sweeps, exactly as wallSurface reads
// it, so its cap edge is concave — a hole's arc and a concave bite in the
// outer boundary alike), while a STRAIGHT walk has no sense of its own and
// takes the loop's: outer convex, hole concave.
func (rp revolvePayload) capEdge(b revolveBasis, w segmentWalk, closed bool, vs, ve *Vertex, phi float64, holeLoop bool) *Edge {
	convex := !holeLoop
	if w.isCircular() {
		convex = w.th0 < w.th1
	}
	e := &Edge{convex: convex, length: w.length, lengthBound: w.lengthBound}
	if !w.isCircular() {
		e.curve = Line3{}
		e.start, e.end = vs, ve
		return e
	}
	// The cap plane at φ is the profile plane rotated about the axis, so
	// its normal is the rotated sweep-velocity direction; the walk's own
	// range order is the arc's CCW sense about it, inverted once by a
	// reflected placement.
	sin, cos := math.Sincos(phi)
	normal := b.e0.Scale(-sin).Add(b.e1.Scale(cos))
	sign := 1.0
	if w.th1 < w.th0 {
		sign = -1
	}
	if rp.reflected() {
		sign = -sign
	}
	axis := rp.xform.ApplyDir(normal).Scale(sign)
	center := rp.point(b, w.cU, w.cV, phi)
	radius := units.Millimeters(w.radius)
	if closed {
		seam := &Vertex{position: rp.point(b, w.startU, w.startV, phi), bound: units.Millimeters(0)}
		e.curve = Circle3{Center: center, Axis: axis, Radius: radius}
		e.start, e.end = seam, seam
		return e
	}
	e.curve = Arc3{Center: center, Axis: axis, Radius: radius}
	e.start, e.end = vs, ve
	return e
}

// wallSurface is the placed surface of revolution one off-axis walk sweeps,
// and whether the face's outward (material-leaving) normal is the surface's
// geometric normal negated. The walk runs with the region's material on its
// left in (z, ρ), so its outward direction is the tangent rotated a quarter
// turn right: (t_ρ, −t_z). A cylinder's and a cone's geometric normal has a
// positive ρ component, so those reverse exactly when the tangent climbs
// (t_z > 0 — a hole wall, or the inner wall of an annular section); a
// sphere's and a torus's geometric normal points away from the generating
// circle's center, which a counter-clockwise walk's outward direction is —
// so those reverse exactly when the walk runs clockwise. Radial directions
// are reflection-equivariant, so a reflected placement changes none of
// this; only the plane frames (built from cross products) correct for it.
func (rp revolvePayload) wallSurface(b revolveBasis, w segmentWalk, kind wallKind) (Surface, bool, error) {
	place := func(z float64) r3.Vec { return rp.xform.Apply(b.a3.Add(b.w.Scale(z))) }
	axis := rp.xform.ApplyDir(b.w)
	switch kind {
	case wallCylinder:
		return Cylinder{
			Origin: place(w.startU),
			Axis:   axis,
			Radius: units.Millimeters((w.startV + w.endV) / 2),
		}, w.tanInU > 0, nil
	case wallPlane:
		// The outward normal is ±axis by the walk's radial heading; the
		// frame's axes are ordered so its normal is outward, swapped once
		// more under a reflected placement.
		u3, v3 := b.e0, b.e1
		if w.tanInV < 0 {
			u3, v3 = v3, u3
		}
		if rp.reflected() {
			u3, v3 = v3, u3
		}
		f, err := r3.NewFrame(place(w.startU), rp.xform.ApplyDir(u3), rp.xform.ApplyDir(v3))
		if err != nil {
			return nil, false, fmt.Errorf(`%w: the placed wall frame is degenerate: %s`, ErrDegenerate, err)
		}
		return Plane{Frame: f}, false, nil
	case wallCone:
		// The apex is where the wall meets the axis; the cone's radius
		// grows along its stored Axis direction.
		dz, dr := w.endU-w.startU, w.endV-w.startV
		apex := w.startU - w.startV*dz/dr
		growth := 1.0
		if dz*dr < 0 {
			growth = -1
		}
		return Cone{
			Origin:    place(apex),
			Axis:      axis.Scale(growth),
			Radius:    units.Millimeters(0),
			HalfAngle: units.Radians(math.Atan2(math.Abs(dr), math.Abs(dz))),
		}, w.tanInU > 0, nil
	case wallSphere:
		return Sphere{
			Center: place(w.cU),
			Radius: units.Millimeters(w.radius),
		}, w.th1 < w.th0, nil
	case wallTorus:
		return Torus{
			Center: place(w.cU),
			Axis:   axis,
			Major:  units.Millimeters(w.cV),
			Minor:  units.Millimeters(w.radius),
		}, w.th1 < w.th0, nil
	default:
		return nil, false, fmt.Errorf(`%w: a wall on the axis sweeps no surface`, ErrDegenerate)
	}
}

// walkAxisMoment is the first moment ∫ρ ds of one boundary walk about the
// axis — Pappus's first theorem reads the side face's area from it:
// a straight walk's is its length times its mean radius (ρ is linear along
// it), a circular walk's is the closed-form antiderivative over its angular
// range, and an on-axis walk sweeps nothing.
func walkAxisMoment(w segmentWalk, kind wallKind) boundedScalar {
	if kind == wallAxis {
		return boundedScalar{}
	}
	if !w.isCircular() {
		meanRadius := boundedDiv(
			boundedAdd(exactScalar(w.startV), exactScalar(w.endV)),
			exactScalar(2),
		)
		result := boundedMul(measuredScalar(w.length, w.lengthBound), meanRadius)
		result.bound = math.Max(result.bound, conservativeValueError(result.value, w.axisMomentUpper))
		return result
	}
	lo, hi := math.Min(w.th0, w.th1), math.Max(w.th0, w.th1)
	dtheta := boundedSub(exactScalar(hi), exactScalar(lo))
	cosDelta := boundedSub(boundedCos(exactScalar(lo)), boundedCos(exactScalar(hi)))
	result := boundedMul(
		exactScalar(w.radius),
		boundedAdd(
			boundedMul(exactScalar(w.cV), dtheta),
			boundedMul(exactScalar(w.radius), cosDelta),
		),
	)
	result.bound = math.Max(result.bound, conservativeValueError(result.value, w.axisMomentUpper))
	return result
}
