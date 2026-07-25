package decad

import (
	"context"
	"fmt"
	"math"
	"math/big"

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
type cupPayload struct {
	outer     ProfileRecord
	cavity    ProfileRecord
	frame     r3.Frame
	zOpen     float64
	zOuter    float64
	zCav      float64
	thickness float64
	sense     ShellSense
	xform     r3.Transform
}

// transform is the accumulated rigid placement.
func (cp cupPayload) transform() r3.Transform { return cp.xform }

// placed re-evaluates the same cup under the composed motion (evaluator §8).
func (cp cupPayload) placed(ctx context.Context, d *Document, ref StepRef, composed r3.Transform) (*Body, error) {
	cp.xform = composed
	return evalCupContext(ctx, d, ref, cp)
}

// prismLike is a prismPayload sharing the cup's frame and placement, so the
// point/dir/reflected/capFrame machinery serves the cup too.
func (cp cupPayload) prismLike(z0, z1 float64) prismPayload {
	return prismPayload{frame: cp.frame, z0: z0, z1: z1, xform: cp.xform}
}

// extentAlong is the cup's exact extent interval along an arbitrary world
// direction g — the OUTER prism's extent (docs/modify-design.md §10, Table D,
// D5). The cavity is interior and reaches no farther than the outer region, so
// the outward extent is the solid outer prism's, read by the same
// prismPayload.extentAlong the outer prism's bounds already use. This is what a
// through-all stop consults when a cup is a live body in the sweep's path.
func (cp cupPayload) extentAlong(g r3.Vec) (float64, float64, error) {
	oLo, oHi := math.Min(cp.zOuter, cp.zOpen), math.Max(cp.zOuter, cp.zOpen)
	outer := prismPayload{profile: cp.outer, frame: cp.frame, z0: oLo, z1: oHi, xform: cp.xform}
	return outer.extentAlong(g)
}

// cupPayloadFor assembles the cup record from the receiver prism, its offset
// section and the shell sense/opening (docs/modify-design.md §9). removedEnd
// opens the cup at the top (z1); a removed start opens it at the bottom (z0),
// its mirror. Inward, O is the original section P and C the erosion Q; outward,
// O is the dilation Q and C the original P.
func cupPayloadFor(pp prismPayload, offset ProfileRecord, s, t float64, removedEnd bool) cupPayload {
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
		outer:     o,
		cavity:    c,
		frame:     pp.frame,
		thickness: t,
		sense:     sense,
		xform:     pp.xform,
	}
	if removedEnd { // open at the top
		cp.zOpen = z1
		if s > 0 {
			cp.zOuter, cp.zCav = z0, z0+t
		} else {
			cp.zOuter, cp.zCav = z0-t, z0
		}
	} else { // open at the bottom — the mirror
		cp.zOpen = z0
		if s > 0 {
			cp.zOuter, cp.zCav = z1, z1-t
		} else {
			cp.zOuter, cp.zCav = z1+t, z1
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
func evalCup(d *Document, ref StepRef, cp cupPayload) (*Body, error) {
	return evalCupContext(context.Background(), d, ref, cp)
}

func evalCupContext(ctx context.Context, d *Document, ref StepRef, cp cupPayload) (*Body, error) {
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	igO, err := cp.outer.evaluatorIntegralsContext(ctx, momentFirstOrder)
	if err != nil {
		return nil, err
	}
	igC, err := cp.cavity.evaluatorIntegralsContext(ctx, momentFirstOrder)
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
	heightO := boundedAbs(boundedSub(exactScalar(cp.zOpen), exactScalar(cp.zOuter)))
	heightC := boundedAbs(boundedSub(exactScalar(cp.zOpen), exactScalar(cp.zCav)))
	hO, hC := heightO.value, heightC.value
	if hO <= 0 || hC <= 0 {
		return nil, fmt.Errorf(`%w: a cup interval is empty`, ErrDegenerate)
	}
	openIsMax := cp.zOpen > cp.zOuter

	body := &Body{doc: d, origin: FeatureRef{Step: ref, Role: "body"}, solid: true}

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
	oLo, oHi := math.Min(cp.zOuter, cp.zOpen), math.Max(cp.zOuter, cp.zOpen)
	ppO := cp.prismLike(oLo, oHi)
	var faces []*Face
	perimO := boundedScalar{}
	oFloor := make([][]coedge, len(oLoops))
	oOpen := make([][]coedge, len(oLoops))
	for i, loop := range oLoops {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		sf, bottom, top, ll, err := buildLoopSides(ctx, body, ref, ppO, i, loop)
		if err != nil {
			return nil, err
		}
		faces = append(faces, sf...)
		perimO = boundedAdd(perimO, ll)
		oFloor[i], oOpen[i] = floorOpen(bottom, top)
	}

	// Cavity walls: every loop of C reversed and built over the cavity interval.
	// The reversed outer walks clockwise (a hole in the solid), each reversed
	// hole counter-clockwise (a post) — holeLoop is (i == 0). Roles are
	// shellSide(i,j) via renameCavityRoles.
	cLo, cHi := math.Min(cp.zCav, cp.zOpen), math.Max(cp.zCav, cp.zOpen)
	ppC := cp.prismLike(cLo, cHi)
	perimC := boundedScalar{}
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
		sf, bottom, top, ll, err := buildLoopSidesAs(ctx, body, ref, ppC, i, i == 0, rev)
		if err != nil {
			return nil, err
		}
		cavFaces = append(cavFaces, sf...)
		perimC = boundedAdd(perimC, ll)
		cFloor[i], cOpen[i] = floorOpen(bottom, top)
	}
	if err := renameCavityRoles(ctx, cavFaces, ref); err != nil {
		return nil, err
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	faces = append(faces, cavFaces...)

	// The planar faces. capFrame orients each normal outward via its flip;
	// capStart faces away from the material, shellCap into the pocket, the rim
	// away from the material at the open end.
	base := cp.prismLike(0, 0)
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
		surface:   Plane{Frame: capStartFrame},
		origins:   []FeatureRef{{Step: ref, Role: roleCapStart}},
		body:      body,
		area:      igO.area,
		areaBound: igO.areaBound,
	}
	shellCap := &Face{
		surface:   Plane{Frame: shellCapFrame},
		origins:   []FeatureRef{{Step: ref, Role: "shellCap"}},
		body:      body,
		area:      igC.area,
		areaBound: igC.areaBound,
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
		rimArea := boundedAbs(boundedSub(aO, aC))
		rims[i] = &Face{
			surface:   Plane{Frame: rimFrame},
			origins:   []FeatureRef{{Step: ref, Role: fmt.Sprintf("rim(%d)", i)}},
			body:      body,
			area:      rimArea.value,
			areaBound: rimArea.bound,
			loops:     []*Loop{outerLoop, holeLoop},
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
	areaO := measuredScalar(igO.area, igO.areaBound)
	areaC := measuredScalar(igC.area, igC.areaBound)
	massO := boundedMul(areaO, heightO)
	massC := boundedMul(areaC, heightC)
	volume := boundedSub(massO, massC)
	body.volume = Measurement{
		Value:     units.CubicMillimeters(volume.value),
		Exactness: exactnessOf(volume.bound),
		Bound:     units.CubicMillimeters(volume.bound),
	}
	area := boundedAdd(
		boundedAdd(boundedMul(exactScalar(2), areaO), boundedMul(perimO, heightO)),
		boundedMul(perimC, heightC),
	)
	body.area = Measurement{
		Value:     units.SquareMillimeters(area.value),
		Exactness: exactnessOf(area.bound),
		Bound:     units.SquareMillimeters(area.bound),
	}

	// Centroid: each region's centroid lifted to its own interval midpoint, the
	// two combined with the cavity's mass subtracted (§10).
	zMidO := boundedDiv(boundedAdd(exactScalar(cp.zOuter), exactScalar(cp.zOpen)), exactScalar(2))
	zMidC := boundedDiv(boundedAdd(exactScalar(cp.zCav), exactScalar(cp.zOpen)), exactScalar(2))
	cuO := boundedQuotient(igO.mu, igO.muBound, igO.area, igO.areaBound)
	cvO := boundedQuotient(igO.mv, igO.mvBound, igO.area, igO.areaBound)
	cuC := boundedQuotient(igC.mu, igC.muBound, igC.area, igC.areaBound)
	cvC := boundedQuotient(igC.mv, igC.mvBound, igC.area, igC.areaBound)
	pp := cp.prismLike(0, 0)
	cO := pp.point(cuO.value, cvO.value, zMidO.value)
	cC := pp.point(cuC.value, cvC.value, zMidC.value)
	cOBound := prismPointBound(pp, cuO, cvO, zMidO)
	cCBound := prismPointBound(pp, cuC, cvC, zMidC)
	denom := boundedSub(massO, massC)
	if denom.value <= 0 {
		return nil, fmt.Errorf(`%w: the cup cavity is not smaller than its outer solid`, ErrDegenerate)
	}
	weightO := boundedDiv(massO, denom)
	weightC := boundedDiv(massC, denom)
	centroidValue := cO.Scale(weightO.value).Sub(cC.Scale(weightC.value))
	centroidBound := absSumUpper(
		productUpper(weightO.value, cOBound),
		productUpper(vecL1(cO), weightO.bound),
		productUpper(weightO.bound, cOBound),
		productUpper(weightC.value, cCBound),
		productUpper(vecL1(cC), weightC.bound),
		productUpper(weightC.bound, cCBound),
		exactWeightedPointRound(cO, weightO.value, cC, weightC.value, centroidValue),
	)
	outerPrism := cp.prismLike(oLo, oHi)
	geometryBound, err := prismCentroidGeometryBound(outerPrism, cp.outer, centroidValue)
	if err != nil {
		return nil, err
	}
	centroidBound = math.Min(centroidBound, geometryBound)
	body.centroid = VecMeasurement{
		Value:     centroidValue,
		Exactness: exactnessOf(centroidBound),
		Bound:     units.Millimeters(centroidBound),
	}

	// Bounds: the outer prism's — the cavity lies within it in both senses (§10).
	bounds, err := prismBoundsContext(ctx, prismPayload{profile: cp.outer, frame: cp.frame, z0: oLo, z1: oHi, xform: cp.xform})
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
		ra, rwa := floatRat(av), floatRat(wa)
		rb, rwb := floatRat(bv), floatRat(wb)
		if ra == nil || rwa == nil || rb == nil || rwb == nil {
			return math.Inf(1)
		}
		exact := new(big.Rat).Sub(
			new(big.Rat).Mul(ra, rwa),
			new(big.Rat).Mul(rb, rwb),
		)
		return rationalFloatError(exact, hv)
	}
	return radius3D(max(
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
func renameCavityRoles(ctx context.Context, faces []*Face, ref StepRef) error {
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
			if o.Step != ref {
				continue
			}
			var li, j int
			if n, _ := fmt.Sscanf(o.Role, "side(%d,%d)", &li, &j); n == 2 {
				f.origins[i] = FeatureRef{Step: ref, Role: fmt.Sprintf("shellSide(%d,%d)", li, j)}
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
func loopEnclosedAreaContext(ctx context.Context, l LoopRecord) (boundedScalar, error) {
	var ig regionIntegrals
	for _, seg := range l.Segments {
		if err := ctx.Err(); err != nil {
			return boundedScalar{}, err
		}
		if err := ig.add(seg); err != nil {
			return boundedScalar{}, err
		}
	}
	return measuredScalar(math.Abs(ig.area), ig.areaBound), nil
}
