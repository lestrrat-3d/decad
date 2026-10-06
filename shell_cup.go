package decad

import (
	"context"
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/proofbound"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// This file is the cup payload of docs/modify-design.md §9 (Table B, B5/B6),
// the payload the shell op introduces. A cup is two co-directional
// prisms over the same plane — the outer region O on its interval and the
// cavity region C on its own, sharing a rim at the open end and a floor at the
// closed one. It re-evaluates under Body.Placed (evaluator §8) and holds every
// measurement §10 specifies with its proven numerical bound.
//
// A holed section (k ≥ 1 posts in the pocket) builds too: the outer region and
// the cavity region each carry k holes, so the cup wraps a wall around each
// post. The cavity is a VOID the solid encloses, so its loops invert — the
// void's outer boundary is a hole in the solid, each of the void's own holes a
// solid post — and every band still hangs off the one floor slab, so the result
// is one lump (Table B, B5/B6). A both-caps holed shell keeps no floor and is
// 1 + k disjoint lumps (B4), which is why THAT case stays refused (S12), not
// this one.

// cupPayload is the evaluator's own record of a cup: the outer region O and the
// cavity region C (each walked in its natural sense — the outer loop
// counter-clockwise), the plane frame, the three sweep planes, the private
// shell-morphology certificate and the accumulated rigid placement. The open
// end (the removed cap, where the rim is) is zOpen; the outer prism's floor is
// at zOuter, the cavity's floor (shellCap) at zCav, which lies between the two
// so the floor slab is [zOuter, zCav] (docs/modify-design.md §9).
//
// zOpenDelta, zOuterDelta and zCavDelta are the axial displacement of their
// matching levels. zOpen and the unstepped floor level retain the displacement
// of the receiver end they came from. A derived floor level additionally carries
// the thickness conversion and float-sum rounding. Every level-derived cup
// reading uses its matching displacement — the two heights and the volume, area
// and centroid built on them, the box, and the wall vertices and vertical edges
// the shared prism build stamps.
//
// thicknessDelta is the shell thickness's OWN conversion displacement — the
// bound magnitudeInBounded proved when it converted the caller's stated
// thickness into millimetres (shell.go). It is distinct from zOuterDelta and
// zCavDelta: those cover the derived FLOOR LEVEL's own float-sum rounding
// (which already folds thicknessDelta in — cupPayloadFor's step closure), and
// this one is the reading cupWall publishes when the shell-wall theorem holds
// exactly on the payload's own morphology — the number the theorem's t is
// held in, not the theorem itself (docs/payload-verification-design.md §4.1).
//
// offsetDelta is the OFFSET region's section displacement — the cavity
// inward, the outer region outward: the proven upper bound offsetSectionDelta
// (shell_offset.go) states on how far any recorded boundary point of that
// region sits from the boundary of P ⊖ t* or P ⊕ t* the shell denotes, t* the
// caller's thickness in exact millimetres. It covers the thickness conversion
// and the offset solve's own rounding together. The other region is the
// receiver's own section and carries none. outerPrism and cavityPrism hand it
// to the offset region's prism as its sectionDelta, so every prism reading
// built on that region — its walls' vertices and lengths, Bounds, the mass
// path — charges it there, and evalCup charges it into the areas and moments
// it composes itself (docs/modify-design.md §9, §10).
type cupPayload struct {
	outer          ProfileRecord
	cavity         ProfileRecord
	frame          r3.Frame
	zOpen          float64
	zOuter         float64
	zCav           float64
	zOpenDelta     float64
	zOuterDelta    float64
	zCavDelta      float64
	thickness      float64
	thicknessDelta float64
	offsetDelta    float64
	sense          ShellSense
	xform          r3.Transform
}

// transform is the accumulated rigid placement.
func (cp cupPayload) transform() r3.Transform { return cp.xform }

// axialDelta reports the largest held-level displacement to a body-relative
// stop resolved against this cup's faces (stops.go).
func (cp cupPayload) axialDelta() float64 {
	return math.Max(cp.zOpenDelta, math.Max(cp.zOuterDelta, cp.zCavDelta))
}

// openScalar, outerScalar and cavityScalar are the cup's three sweep levels as
// bounded readings, each carrying its own axial displacement.
func (cp cupPayload) openScalar() proofbound.BoundedScalar {
	return proofbound.MeasuredScalar(cp.zOpen, cp.zOpenDelta)
}

func (cp cupPayload) outerScalar() proofbound.BoundedScalar {
	return proofbound.MeasuredScalar(cp.zOuter, cp.zOuterDelta)
}

func (cp cupPayload) cavityScalar() proofbound.BoundedScalar {
	return proofbound.MeasuredScalar(cp.zCav, cp.zCavDelta)
}

// placed re-evaluates the same cup under the composed motion (evaluator §8).
func (cp cupPayload) placed(ctx context.Context, d *Document, ref producerID, composed r3.Transform) (*Body, error) {
	cp.xform = composed
	return evalCupContext(ctx, d, ref, cp)
}

// basePrism shares the cup's frame and placement with the prism helpers that
// need no axial level.
func (cp cupPayload) basePrism() prismPayload {
	return prismPayload{frame: cp.frame, xform: cp.xform}
}

// prismBetween builds an ordered prism from two cup levels, preserving each
// level's own axial displacement at the matching prism end.
func (cp cupPayload) prismBetween(a, b proofbound.BoundedScalar) prismPayload {
	if a.Value <= b.Value {
		return prismPayload{
			frame: cp.frame, z0: a.Value, z1: b.Value,
			z0Delta: a.Bound, z1Delta: b.Bound,
			xform: cp.xform,
		}
	}
	return prismPayload{
		frame: cp.frame, z0: b.Value, z1: a.Value,
		z0Delta: b.Bound, z1Delta: a.Bound,
		xform: cp.xform,
	}
}

// outerPrism and cavityPrism are the cup's two prisms, without their
// profiles. The offset region's prism carries offsetDelta as its section
// displacement: the outer one outward, the cavity inward.
func (cp cupPayload) outerPrism() prismPayload {
	pp := cp.prismBetween(cp.outerScalar(), cp.openScalar())
	if cp.sense == Outward {
		pp.sectionDelta = cp.offsetDelta
	}
	return pp
}

func (cp cupPayload) cavityPrism() prismPayload {
	pp := cp.prismBetween(cp.cavityScalar(), cp.openScalar())
	if cp.sense != Outward {
		pp.sectionDelta = cp.offsetDelta
	}
	return pp
}

// extentAlong is the cup's extent interval along an arbitrary world direction
// g, beside the displacement its ends carry — the OUTER prism's reading
// (docs/modify-design.md §10, Table D, D5). The cavity is interior and reaches
// no farther than the outer region, so the outward extent is the solid outer
// prism's, read by the same prismPayload.extentAlong the outer prism's bounds
// already use. This is what a through-all stop consults when a cup is a live
// body in the sweep's path. An outward cup's outer region is the offset one:
// its recorded extent is read, and its offsetDelta joins the displacement the
// ends carry — moving every boundary point by at most offsetDelta moves an
// extreme along g by at most offsetDelta·|g|, and |g| ≤ its L1 norm.
func (cp cupPayload) extentAlong(g r3.Vec) (float64, float64, float64, error) {
	outer := cp.outerPrism()
	outer.profile = cp.outer
	displaced := outer.sectionDelta
	outer.sectionDelta = 0
	lo, hi, delta, err := outer.extentAlong(g)
	if err != nil || displaced == 0 {
		return lo, hi, delta, err
	}
	return lo, hi, proofbound.AbsSumUpper(delta, proofbound.ProductUpper(displaced, vecL1(g))), nil
}

// cupPayloadFor assembles the cup record from the receiver prism, its offset
// section with that section's proven displacement, and the shell
// sense/opening (docs/modify-design.md §9). removedEnd
// opens the cup at the top (z1); a removed start opens it at the bottom (z0),
// its mirror. Inward, O is the original section P and C the erosion Q; outward,
// O is the dilation Q and C the original P.
func cupPayloadFor(pp prismPayload, offset ProfileRecord, s, t, tDelta, offsetDelta float64, removedEnd bool) cupPayload {
	z0, z1 := pp.z0, pp.z1
	o, c := pp.profile, offset
	if s < 0 {
		o, c = offset, pp.profile
	}
	sense := Inward
	if s < 0 {
		sense = Outward
	}
	cp := cupPayload{
		outer:          o,
		cavity:         c,
		frame:          pp.frame,
		thickness:      t,
		thicknessDelta: tDelta,
		offsetDelta:    offsetDelta,
		sense:          sense,
		xform:          pp.xform,
	}
	// step is the one derived floor level. It carries the source end's own
	// displacement, the thickness conversion, and this float sum's rounding.
	step := func(from, delta, by float64) (float64, float64) {
		to := from + by
		return to, proofbound.AbsSumUpper(delta, tDelta, proofarith.AddRoundError(from, by, to))
	}
	if removedEnd { // open at the top
		cp.zOpen = z1
		cp.zOpenDelta = pp.z1Delta
		if s > 0 {
			cp.zOuter, cp.zOuterDelta = z0, pp.z0Delta
			cp.zCav, cp.zCavDelta = step(z0, pp.z0Delta, t)
		} else {
			cp.zOuter, cp.zOuterDelta = step(z0, pp.z0Delta, -t)
			cp.zCav, cp.zCavDelta = z0, pp.z0Delta
		}
	} else { // open at the bottom — the mirror
		cp.zOpen = z0
		cp.zOpenDelta = pp.z0Delta
		if s > 0 {
			cp.zOuter, cp.zOuterDelta = z1, pp.z1Delta
			cp.zCav, cp.zCavDelta = step(z1, pp.z1Delta, -t)
		} else {
			cp.zOuter, cp.zOuterDelta = step(z1, pp.z1Delta, t)
			cp.zCav, cp.zCavDelta = z1, pp.z1Delta
		}
	}
	return cp
}

// evalCup builds the analytic cup body (Table B, B5/B6): outer walls over every
// loop of O, cavity walls over every loop of the reversed C, the kept cap
// capStart, the pocket floor shellCap, and one rim per loop (1 + k of them) —
// one manifold, watertight shell (every edge bounds two faces), with Exact
// measurements per §10. The cavity region is a VOID: its loops invert, so the
// void's outer boundary is walked as a hole in the solid (its wall's material
// lies outside it) and each of the void's own holes as a solid post (material
// inside) — the pairing buildLoopSidesAs's explicit holeLoop expresses.
func evalCup(d *Document, ref producerID, cp cupPayload) (*Body, error) {
	return evalCupContext(context.Background(), d, ref, cp)
}

func evalCupContext(ctx context.Context, d *Document, ref producerID, cp cupPayload) (*Body, error) {
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	// ONE free-form counter for the whole cup build. The outer region and the
	// cavity are the two halves of one recorded section and no preflight has run
	// on either, so the ceiling opens here and both preflights and every walkOf
	// below spend it (docs/spline-design.md §5.2).
	work := newFreeformWork()
	igO, err := cp.outer.evaluatorIntegralsContext(ctx, momentFirstOrder, work)
	if err != nil {
		return nil, err
	}
	igC, err := cp.cavity.evaluatorIntegralsContext(ctx, momentFirstOrder, work)
	if err != nil {
		return nil, err
	}
	if igO.area <= 0 || igC.area <= 0 {
		return nil, fmt.Errorf(`%w: a cup region encloses no area`, ErrDegenerate)
	}
	oLoops := append([]LoopRecord{cp.outer.Outer}, cp.outer.Holes...)
	cLoops := append([]LoopRecord{cp.cavity.Outer}, cp.cavity.Holes...)
	if len(oLoops) != len(cLoops) {
		// The offset preserves the loop count (a dropped loop is S11a, caught
		// before here); a mismatch would leave a rim with no partner loop.
		return nil, fmt.Errorf(`%w: the cup's outer and cavity regions have different loop counts`, ErrDegenerate)
	}
	// Each height is read from BOUNDED levels, so a cup built on a computed
	// sweep carries that computation into the volume, area and centroid below
	// rather than publishing them as the levels they denote.
	heightO := proofbound.BoundedAbs(proofbound.BoundedSub(cp.openScalar(), cp.outerScalar()))
	heightC := proofbound.BoundedAbs(proofbound.BoundedSub(cp.openScalar(), cp.cavityScalar()))
	hO, hC := heightO.Value, heightC.Value
	if hO <= 0 || hC <= 0 {
		return nil, fmt.Errorf(`%w: a cup interval is empty`, ErrDegenerate)
	}
	openIsMax := cp.zOpen > cp.zOuter

	body := &Body{doc: d, origin: FeatureRef{producer: ref, Role: roleBody}, solid: true}

	// floorOpen splits a wall's (bottom, top) cap coedges into the floor-side
	// set (kept cap / pocket floor) and the open-side set (the rim).
	floorOpen := func(bottom, top []coedge) (floor, open []coedge) {
		if openIsMax {
			return bottom, top
		}
		return top, bottom
	}

	// Outer walls: every loop of O in its natural sense (outer counter-clockwise,
	// holes clockwise), role side(i,j). A hole of O is a tunnel through the whole
	// body — its wall runs the full outer interval.
	ppO := cp.outerPrism()
	var faces []*Face
	perimO := proofbound.BoundedScalar{}
	loopPerimO := make([]proofbound.BoundedScalar, len(oLoops))
	oFloor := make([][]coedge, len(oLoops))
	oOpen := make([][]coedge, len(oLoops))
	for i, loop := range oLoops {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		sf, bottom, top, ll, err := buildLoopSides(ctx, body, ref, ppO, i, loop, work, nil, levelToken{}, levelToken{}, false)
		if err != nil {
			return nil, err
		}
		faces = append(faces, sf...)
		perimO = proofbound.BoundedAdd(perimO, ll)
		loopPerimO[i] = ll
		oFloor[i], oOpen[i] = floorOpen(bottom, top)
	}

	// Cavity walls: every loop of C reversed and built over the cavity interval.
	// The reversed outer walks clockwise (a hole in the solid), each reversed
	// hole counter-clockwise (a post) — holeLoop is (i == 0). Roles are
	// shellSide(i,j) via renameCavityRoles.
	ppC := cp.cavityPrism()
	perimC := proofbound.BoundedScalar{}
	loopPerimC := make([]proofbound.BoundedScalar, len(cLoops))
	cFloor := make([][]coedge, len(cLoops))
	cOpen := make([][]coedge, len(cLoops))
	var cavFaces []*Face
	for i, loop := range cLoops {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		rev, err := reverseLoopRecordContext(ctx, loop)
		if err != nil {
			return nil, err
		}
		sf, bottom, top, ll, err := buildLoopSidesAs(ctx, body, ref, ppC, i, i == 0, rev, work, nil, levelToken{}, levelToken{}, false)
		if err != nil {
			return nil, err
		}
		cavFaces = append(cavFaces, sf...)
		perimC = proofbound.BoundedAdd(perimC, ll)
		loopPerimC[i] = ll
		cFloor[i], cOpen[i] = floorOpen(bottom, top)
	}
	if err := renameCavityRoles(ctx, cavFaces, ref); err != nil {
		return nil, err
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	faces = append(faces, cavFaces...)

	// The offset region's recorded boundary sits within offsetDelta of the one
	// the cup denotes, so its area and first moments move by the displacement
	// area that boundary can sweep (internal/proofbound/bounds.go's proofbound.SectionDisplacementArea), the
	// moments by that area times the largest coordinate it can reach. Its walls
	// already took the displacement through their prism's sectionDelta.
	if cp.offsetDelta > 0 {
		if cp.sense == Outward {
			igO, err = displacedRegionIntegrals(igO, cp.outer, perimO, cp.offsetDelta, work)
		} else {
			igC, err = displacedRegionIntegrals(igC, cp.cavity, perimC, cp.offsetDelta, work)
		}
		if err != nil {
			return nil, err
		}
	}

	// The planar faces. capFrame orients each normal outward via its flip;
	// capStart faces away from the material, shellCap into the pocket, the rim
	// away from the material at the open end.
	base := cp.basePrism()
	capStartFrame, err := capFrame(base, cp.zOuter, openIsMax)
	if err != nil {
		return nil, err
	}
	shellCapFrame, err := capFrame(base, cp.zCav, !openIsMax)
	if err != nil {
		return nil, err
	}
	rimFrame, err := capFrame(base, cp.zOpen, !openIsMax)
	if err != nil {
		return nil, err
	}

	// The kept cap and the pocket floor each carry one loop per region loop: the
	// outer boundary (outer true) and each hole (a tunnel through the kept cap, a
	// post through the pocket floor).
	capStart := &Face{
		surface:       Plane{Frame: capStartFrame},
		origins:       []FeatureRef{{producer: ref, Role: roleCapStart}},
		body:          body,
		area:          igO.area,
		areaBound:     igO.areaBound,
		axialDelta:    cp.zOuterDelta,
		hasAxialDelta: true,
	}
	shellCap := &Face{
		surface:       Plane{Frame: shellCapFrame},
		origins:       []FeatureRef{{producer: ref, Role: "shellCap"}},
		body:          body,
		area:          igC.area,
		areaBound:     igC.areaBound,
		axialDelta:    cp.zCavDelta,
		hasAxialDelta: true,
	}
	for i := range oLoops {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		capStart.loops = append(capStart.loops, &Loop{coedges: oFloor[i], outer: i == 0})
		shellCap.loops = append(shellCap.loops, &Loop{coedges: cFloor[i], outer: i == 0})
	}

	// Rims: one per loop, the removed cap's plane trimmed to the band between
	// loop i of O and loop i of C. The bigger boundary is O for the outer region
	// (the cavity sits inside it) and C for a hole loop (a post rim is wider than
	// the tunnel it wraps), so the outer/hole roles of the two loops swap at i≥1.
	// Loops() lists the outer loop FIRST (topology.go), so the outer boundary
	// heads the slice: O for the outer region, C for a post rim.
	rims := make([]*Face, len(oLoops))
	for i := range oLoops {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		oIsOuter := i == 0
		aO, err := loopEnclosedAreaContext(ctx, oLoops[i])
		if err != nil {
			return nil, err
		}
		aC, err := loopEnclosedAreaContext(ctx, cLoops[i])
		if err != nil {
			return nil, err
		}
		oLoop := &Loop{coedges: oOpen[i], outer: oIsOuter}
		cLoop := &Loop{coedges: cOpen[i], outer: !oIsOuter}
		outerLoop, holeLoop := oLoop, cLoop
		if !oIsOuter {
			outerLoop, holeLoop = cLoop, oLoop
		}
		// The offset loop's own enclosed area moves with its boundary.
		if cp.sense == Outward {
			aO.Bound = proofbound.AbsSumUpper(aO.Bound, loopDisplacementArea(cp.offsetDelta, oLoops[i], loopPerimO[i]))
		} else {
			aC.Bound = proofbound.AbsSumUpper(aC.Bound, loopDisplacementArea(cp.offsetDelta, cLoops[i], loopPerimC[i]))
		}
		rimArea := proofbound.BoundedAbs(proofbound.BoundedSub(aO, aC))
		rims[i] = &Face{
			surface:       Plane{Frame: rimFrame},
			origins:       []FeatureRef{{producer: ref, Role: fmt.Sprintf("rim(%d)", i)}},
			body:          body,
			area:          rimArea.Value,
			areaBound:     rimArea.Bound,
			loops:         []*Loop{outerLoop, holeLoop},
			axialDelta:    cp.zOpenDelta,
			hasAxialDelta: true,
		}
	}

	// Attach each planar face to its boundary edges (two faces per edge).
	planarFaces := make([]*Face, 0, 2+len(rims))
	planarFaces = append(planarFaces, capStart, shellCap)
	planarFaces = append(planarFaces, rims...)
	if err := attachFaceLoopsContext(ctx, planarFaces); err != nil {
		return nil, err
	}

	faces = append(faces, capStart, shellCap)
	faces = append(faces, rims...)
	body.lumps = []*Lump{{shells: []*Shell{{faces: faces}}}}

	// Measurements carry the composed profile, length, and arithmetic bounds
	// (docs/modify-design.md §10).
	areaO := proofbound.MeasuredScalar(igO.area, igO.areaBound)
	areaC := proofbound.MeasuredScalar(igC.area, igC.areaBound)
	massO := proofbound.BoundedMul(areaO, heightO)
	massC := proofbound.BoundedMul(areaC, heightC)
	volume := proofbound.BoundedSub(massO, massC)
	body.volume = Measurement{
		Value:     units.CubicMillimeters(volume.Value),
		Exactness: exactnessOf(volume.Bound),
		Bound:     units.CubicMillimeters(volume.Bound),
	}
	area := proofbound.BoundedAdd(
		proofbound.BoundedAdd(proofbound.BoundedMul(proofbound.ExactScalar(2), areaO), proofbound.BoundedMul(perimO, heightO)),
		proofbound.BoundedMul(perimC, heightC),
	)
	body.area = Measurement{
		Value:     units.SquareMillimeters(area.Value),
		Exactness: exactnessOf(area.Bound),
		Bound:     units.SquareMillimeters(area.Bound),
	}

	// Centroid: each region's centroid lifted to its own interval midpoint, the
	// two combined with the cavity's mass subtracted (§10).
	zMidO := proofbound.BoundedDiv(proofbound.BoundedAdd(cp.outerScalar(), cp.openScalar()), proofbound.ExactScalar(2))
	zMidC := proofbound.BoundedDiv(proofbound.BoundedAdd(cp.cavityScalar(), cp.openScalar()), proofbound.ExactScalar(2))
	cuO := proofbound.BoundedQuotient(igO.mu, igO.muBound, igO.area, igO.areaBound)
	cvO := proofbound.BoundedQuotient(igO.mv, igO.mvBound, igO.area, igO.areaBound)
	cuC := proofbound.BoundedQuotient(igC.mu, igC.muBound, igC.area, igC.areaBound)
	cvC := proofbound.BoundedQuotient(igC.mv, igC.mvBound, igC.area, igC.areaBound)
	pp := cp.basePrism()
	cO := pp.point(cuO.Value, cvO.Value, zMidO.Value)
	cC := pp.point(cuC.Value, cvC.Value, zMidC.Value)
	cOBound := prismPointBound(pp, cuO, cvO, zMidO)
	cCBound := prismPointBound(pp, cuC, cvC, zMidC)
	denom := proofbound.BoundedSub(massO, massC)
	if denom.Value <= 0 {
		return nil, fmt.Errorf(`%w: the cup cavity is not smaller than its outer solid`, ErrDegenerate)
	}
	weightO := proofbound.BoundedDiv(massO, denom)
	weightC := proofbound.BoundedDiv(massC, denom)
	centroidValue := cO.Scale(weightO.Value).Sub(cC.Scale(weightC.Value))
	centroidBound := proofbound.AbsSumUpper(
		proofbound.ProductUpper(weightO.Value, cOBound),
		proofbound.ProductUpper(vecL1(cO), weightO.Bound),
		proofbound.ProductUpper(weightO.Bound, cOBound),
		proofbound.ProductUpper(weightC.Value, cCBound),
		proofbound.ProductUpper(vecL1(cC), weightC.Bound),
		proofbound.ProductUpper(weightC.Bound, cCBound),
		exactWeightedPointRound(cO, weightO.Value, cC, weightC.Value, centroidValue),
	)
	outerPrism := cp.outerPrism()
	geometryBound, err := prismCentroidGeometryBound(outerPrism, cp.outer, centroidValue, work, nil)
	if err != nil {
		return nil, err
	}
	// That envelope is read off the RECORDED outer region and levels; the
	// denoted body reaches up to the outer region's section displacement
	// farther in the plane and each level's axial one along the normal, and
	// the rigid map carries each through the same 3·L1 factor the envelope
	// uses.
	geometryBound = proofbound.AbsSumUpper(geometryBound, proofbound.ProductUpper(3, proofbound.AbsSumUpper(
		proofbound.ProductUpper(proofbound.AbsSumUpper(vecL1(outerPrism.frame.U()), vecL1(outerPrism.frame.V())), outerPrism.sectionDelta),
		proofbound.ProductUpper(vecL1(outerPrism.frame.N()), cp.axialDelta()),
	)))
	centroidBound = math.Min(centroidBound, geometryBound)
	body.centroid = VecMeasurement{
		Value:     centroidValue,
		Exactness: exactnessOf(centroidBound),
		Bound:     units.Millimeters(centroidBound),
	}

	// Bounds: the outer prism's — the cavity lies within it in both senses (§10).
	outerBox := cp.outerPrism()
	outerBox.profile = cp.outer
	bounds, err := prismBoundsContext(ctx, outerBox, work, nil)
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
	body.payload = cp
	return body, nil
}

func exactWeightedPointRound(a r3.Vec, wa float64, b r3.Vec, wb float64, held r3.Vec) float64 {
	coordinateError := func(av, bv, hv float64) float64 {
		ra, rwa := proofarith.FloatRat(av), proofarith.FloatRat(wa)
		rb, rwb := proofarith.FloatRat(bv), proofarith.FloatRat(wb)
		if ra == nil || rwa == nil || rb == nil || rwb == nil {
			return math.Inf(1)
		}
		exact := new(big.Rat).Sub(
			new(big.Rat).Mul(ra, rwa),
			new(big.Rat).Mul(rb, rwb),
		)
		return proofarith.RationalFloatError(exact, hv)
	}
	return proofbound.Radius3D(max(
		coordinateError(a.X, b.X, held.X),
		coordinateError(a.Y, b.Y, held.Y),
		coordinateError(a.Z, b.Z, held.Z),
	))
}

// renameCavityRoles rewrites the cavity walls' provenance from the
// buildLoopSidesAs default side(i,j) to shellSide(i,j) — the role Table B
// (B5/B6) gives a cavity wall, indexing loop i of the cavity region Q in the
// result's own record (§11). The cavity walls are built with roleLoop equal to
// their Q loop index, so the rename keeps the index and only swaps the tag.
func renameCavityRoles(ctx context.Context, faces []*Face, ref producerID) error {
	if err := ctx.Err(); err != nil {
		return err
	}
	for _, f := range faces {
		if err := ctx.Err(); err != nil {
			return err
		}
		for i, o := range f.origins {
			if err := ctx.Err(); err != nil {
				return err
			}
			if o.producer != ref {
				continue
			}
			var li, j int
			if n, _ := fmt.Sscanf(o.Role, "side(%d,%d)", &li, &j); n == 2 {
				f.origins[i] = FeatureRef{producer: ref, Role: fmt.Sprintf("shellSide(%d,%d)", li, j)}
			}
		}
	}
	return nil
}

// loopEnclosedAreaContext is the absolute area a single loop encloses — the magnitude
// of its Green's-theorem signed area, so a hole (clockwise) and an outer loop
// (counter-clockwise) both report a positive area. A rim band's area is the
// difference of the two loops it spans, and both are strictly nested (the audit
// proved the cavity simple and inside the outer region), so the absolute
// difference is the band's analytic area, with both source bounds carried.
func loopEnclosedAreaContext(ctx context.Context, l LoopRecord) (proofbound.BoundedScalar, error) {
	ig, err := loopRegionIntegralsContext(ctx, l)
	if err != nil {
		return proofbound.BoundedScalar{}, err
	}
	return proofbound.MeasuredScalar(math.Abs(ig.area), ig.areaBound), nil
}

// loopRegionIntegralsContext runs the regionIntegrals accumulator over one
// loop's own segments about the plane origin — the shared walk
// loopEnclosedAreaContext and loopEnclosedMomentsContext both read, so the
// two never disagree about which boundary they integrated.
func loopRegionIntegralsContext(ctx context.Context, l LoopRecord) (regionIntegrals, error) {
	var ig regionIntegrals
	for _, seg := range l.Segments {
		if err := ctx.Err(); err != nil {
			return regionIntegrals{}, err
		}
		// Integrated about the plane origin itself; the band's area is a
		// difference of two loop areas, so no walk anchor is involved.
		if err := ig.addAnalytic(seg, Point2{}); err != nil {
			return regionIntegrals{}, err
		}
	}
	return ig, nil
}

// loopEnclosedMomentsContext is a signed first-moment sibling of
// loopEnclosedAreaContext (docs/modify-reach-design.md §8.4's slab term): the
// SAME regionIntegrals accumulator over the loop's own segments about the
// plane origin, returning the loop's own area together with its first
// moments (∫u dA, ∫v dA) — each canonicalized the SAME way
// loopEnclosedAreaContext's |area| already is, via orient = sign(the loop's
// own raw signed area). Reversing a walk's direction negates every one of
// Green's theorem's contour integrals identically — area and first moments
// alike, since they are all contour integrals of the same shape — so the
// SAME orient factor that turns a clockwise hole's negative raw area into
// loopEnclosedAreaContext's positive reading turns its first moments
// consistently too, and orient*ig.area equals math.Abs(ig.area) exactly, so
// this function's own area field always matches loopEnclosedAreaContext's.
// The caller (evalCapBlendContext) applies its own per-loop sign
// (outer/hole, by loop index) on top of this canonicalized triple, exactly as
// it already does to loopEnclosedAreaContext's |area| for the slab volume.
func loopEnclosedMomentsContext(ctx context.Context, l LoopRecord) (area, mu, mv proofbound.BoundedScalar, err error) {
	ig, err := loopRegionIntegralsContext(ctx, l)
	if err != nil {
		return proofbound.BoundedScalar{}, proofbound.BoundedScalar{}, proofbound.BoundedScalar{}, err
	}
	orient := 1.0
	if ig.area < 0 {
		orient = -1
	}
	return proofbound.MeasuredScalar(orient*ig.area, ig.areaBound),
		proofbound.MeasuredScalar(orient*ig.mu, ig.muBound),
		proofbound.MeasuredScalar(orient*ig.mv, ig.mvBound), nil
}

// displacedRegionIntegrals widens a region's area and first-moment bounds by
// what a boundary displaced by at most delta can move them: the displacement
// area proofbound.SectionDisplacementArea proves over the region's segments and proven
// perimeter, and that area times the largest coordinate magnitude any point
// of the symmetric difference can have — the region's own envelope plus
// delta.
func displacedRegionIntegrals(ig regionIntegrals, profile ProfileRecord, perim proofbound.BoundedScalar, delta float64, work *freeformWork) (regionIntegrals, error) {
	segments := len(profile.Outer.Segments)
	for _, hole := range profile.Holes {
		segments += len(hole.Segments)
	}
	area := proofbound.SectionDisplacementArea(delta, segments, proofbound.AbsSumUpper(perim.Value, perim.Bound))
	coord, err := profileCoordinateEnvelope(profile, work, nil)
	if err != nil {
		return regionIntegrals{}, err
	}
	moment := proofbound.ProductUpper(area, proofbound.AbsSumUpper(coord, delta))
	ig.areaBound = proofbound.AbsSumUpper(ig.areaBound, area)
	ig.muBound = proofbound.AbsSumUpper(ig.muBound, moment)
	ig.mvBound = proofbound.AbsSumUpper(ig.mvBound, moment)
	return ig, nil
}

// loopDisplacementArea is proofbound.SectionDisplacementArea over one loop: how far the
// area that loop encloses can move when its boundary moves by at most delta.
func loopDisplacementArea(delta float64, loop LoopRecord, perim proofbound.BoundedScalar) float64 {
	return proofbound.SectionDisplacementArea(delta, len(loop.Segments), proofbound.AbsSumUpper(perim.Value, perim.Bound))
}
