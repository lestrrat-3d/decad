package decad

import (
	"context"
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	"github.com/lestrrat-3d/decad/internal/freeform"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// This file measures a brepPayload (docs/general-boolean-design.md §4.3):
// every reading is a sum over faces of a closed form in that face's own
// record. The divergence-theorem sums run in exact rational intervals in the
// reference frame (brepEmbeds): a line contributes an exact rational, a
// circular wall or region its proven enclosure, and the published float is
// the sum rounded once. The terms may cancel in value, never in bound — every
// enclosure's width is carried through the sum. The undercut and
// minimum-radius surveys over the same faces (§4.5) close the file.

// brepRegion is a planar face's region read once: its area enclosure, the
// published area with its bound, and the upper bound on its area.
type brepRegion struct {
	area      proofbound.RatInterval
	published proofbound.BoundedScalar
	upper     float64
	// displacement is the area its section displacement moves
	// (proofbound.SectionDisplacementArea over the region's own walks).
	displacement float64
}

// brepRegionOf integrates a planar face's region. The published area carries
// the region's own integration bound plus the area its section displacement
// moves, as evalPrism composes a cap's.
func brepRegionOf(ctx context.Context, f brepFace, walks [][]survey2d.SegmentWalk) (brepRegion, error) {
	ig, err := f.region.EvaluatorIntegralsContext(ctx, freeform.MomentFirstOrder, freeform.NewFreeformWork())
	if err != nil {
		return brepRegion{}, err
	}
	area, err := brepEnclosure(ig.Area, ig.AreaBound, ig.ExactArea())
	if err != nil {
		return brepRegion{}, err
	}
	perimeter := proofbound.BoundedScalar{}
	count := 0
	for _, loop := range walks {
		for _, w := range loop {
			perimeter = proofbound.BoundedAdd(perimeter, proofbound.MeasuredScalar(w.Length,
				proofbound.AbsSumUpper(w.LengthBound, proofbound.SectionDisplacementLength(f.delta, 1))))
			count++
		}
	}
	displacement := proofbound.SectionDisplacementArea(f.delta, count, proofbound.AbsSumUpper(perimeter.Value, perimeter.Bound))
	published := proofbound.MeasuredScalar(ig.Area, proofbound.AbsSumUpper(ig.AreaBound, displacement))
	return brepRegion{
		area: area, published: published, displacement: displacement,
		upper: proofbound.AbsSumUpper(ig.Area, ig.AreaBound),
	}, nil
}

// brepEnclosure is a rational enclosure of a reading: the exact rational where
// there is one, otherwise the held float widened by its proven bound. A
// non-finite reading has no enclosure and is ErrNotFinite.
func brepEnclosure(value, bound float64, exact *big.Rat) (proofbound.RatInterval, error) {
	if exact != nil {
		return proofbound.PointInterval(exact), nil
	}
	v, b := proofarith.FloatRat(value), proofarith.FloatRat(bound)
	if v == nil || b == nil {
		return proofbound.RatInterval{}, fmt.Errorf(`%w: a brep face's integral is not finite`, ErrNotFinite)
	}
	return proofbound.IntervalWiden(proofbound.PointInterval(v), b), nil
}

// brepSegmentIntegrals encloses one wall segment's Green's-theorem
// contributions about its frame origin, in the per-segment forms moments.go
// accumulates: g = ½∫(u dv − v du), mu = ½∫u² dv and mv = −½∫v² du. A line's
// are exact rationals; a circular segment's carry its proven bounds.
func brepSegmentIntegrals(seg CurveSegment) ([3]proofbound.RatInterval, error) {
	var ig regionIntegrals
	if err := ig.AddFor(seg, freeformPlan{}, Point2{}, freeform.MomentFirstOrder); err != nil {
		return [3]proofbound.RatInterval{}, err
	}
	var exact [3]*big.Rat
	if !ig.ExactDead && ig.Exact.Complete() {
		exact = [3]*big.Rat{ig.Exact.Area, ig.Exact.Mu, ig.Exact.Mv}
	}
	var out [3]proofbound.RatInterval
	for i, field := range [3][2]float64{{ig.Area, ig.AreaBound}, {ig.Mu, ig.MuBound}, {ig.Mv, ig.MvBound}} {
		iv, err := brepEnclosure(field[0], field[1], exact[i])
		if err != nil {
			return [3]proofbound.RatInterval{}, err
		}
		out[i] = iv
	}
	return out, nil
}

// brepHeld is the float an enclosure publishes: its exact value rounded once
// when it is a point, otherwise its midpoint rounded once.
func brepHeld(iv proofbound.RatInterval) float64 {
	if iv.Lo.Cmp(iv.Hi) == 0 {
		held, _ := iv.Lo.Float64()
		return held
	}
	mid := new(big.Rat).Add(iv.Lo, iv.Hi)
	held, _ := mid.Quo(mid, big.NewRat(2, 1)).Float64()
	return held
}

// brepFaceBody builds one face's own geometry, role and area; the caller
// attaches its loops. A planar face's plane is the cap frame its view states
// at its level, turned to its outward normal; a swept face's surface is the
// prism wall's own (buildWallGeometry).
func brepFaceBody(ctx context.Context, bp brepPayload, topo *brepTopology, fi int, body *Body, ref producerID) (*Face, error) {
	f := bp.faces[fi]
	pp := f.view(bp.xform)
	origins := []FeatureRef{{producer: ref, Role: f.role}}
	if f.blend != "" {
		origins = append(origins, FeatureRef{producer: ref, Role: fmt.Sprintf("%s(%d)", f.blend, fi)})
	}
	if f.planar() {
		frame, err := capFrame(pp, f.z0, !f.outward)
		if err != nil {
			return nil, err
		}
		region, err := topo.region(ctx, bp, fi)
		if err != nil {
			return nil, err
		}
		return &Face{surface: Plane{Frame: frame}, origins: origins, body: body,
			area: region.published.Value, areaBound: region.published.Bound,
			axialDelta: f.z0Delta, hasAxialDelta: true}, nil
	}
	w := topo.walls[fi]
	_, _, surf, reversed, err := buildWallGeometry(pp, survey2d.SideWalk{SegmentWalk: w, Segs: []int{0}},
		false, w.Closed, nil, nil, nil, nil)
	if err != nil {
		return nil, err
	}
	area := brepWallArea(f, w)
	return &Face{surface: surf, origins: origins, body: body, area: area.Value, areaBound: area.Bound,
		reversed: reversed}, nil
}

// brepWallArea is a swept face's L·h: its walk's length, carrying the length
// its section displacement moves, times its bounded height.
func brepWallArea(f brepFace, w survey2d.SegmentWalk) proofbound.BoundedScalar {
	length := proofbound.MeasuredScalar(w.Length,
		proofbound.AbsSumUpper(w.LengthBound, proofbound.SectionDisplacementLength(f.delta, 1)))
	height := proofbound.BoundedSub(proofbound.MeasuredScalar(f.z1, f.z1Delta), proofbound.MeasuredScalar(f.z0, f.z0Delta))
	return proofbound.BoundedMul(length, height)
}

// region reads a planar face's region once per topology.
func (topo *brepTopology) region(ctx context.Context, bp brepPayload, fi int) (brepRegion, error) {
	if r, ok := topo.regions[fi]; ok {
		return r, nil
	}
	r, err := brepRegionOf(ctx, bp.faces[fi], topo.planar[fi])
	if err != nil {
		return brepRegion{}, err
	}
	if topo.regions == nil {
		topo.regions = map[int]brepRegion{}
	}
	topo.regions[fi] = r
	return r, nil
}

// brepRestoredRegion integrates a planar face the measurement reads restored
// (brepPayload.filletRestored), walking its region afresh.
func brepRestoredRegion(ctx context.Context, f brepFace) (brepRegion, error) {
	work := freeform.NewFreeformWork()
	var walks [][]survey2d.SegmentWalk
	for _, loop := range append([]LoopRecord{f.region.Outer}, f.region.Holes...) {
		var ws []survey2d.SegmentWalk
		for _, seg := range loop.Segments {
			w, err := boundarywalk.WalkOf(seg, work)
			if err != nil {
				return brepRegion{}, err
			}
			ws = append(ws, w)
		}
		walks = append(walks, ws)
	}
	return brepRegionOf(ctx, f, walks)
}

// measureBrepContext publishes the body's volume, area, centroid and box.
//
// In reference coordinates a planar face at level z with outward sign s
// contributes s·z·A to 3V, and ½·s·σ·z²·A to the first moment along the
// reference axis its normal lands on (σ that axis's sign); a swept face over
// height h contributes 2·h·g to 3V and σ·h·mu, σ·h·mv to the moments along the
// axes its u and v land on (brepSegmentIntegrals). Over a closed boundary
// these are ∮(p·n)/3 and ½∮x_i²·n_i, the divergence theorem's volume and
// first moments. Each face's displacements enter as evalPrism composes them
// for a prism: a swept face's section band times its height, and a planar
// face's level displacement times its area, bound the volume the denoted body
// can differ by; that volume times the coordinate envelope bounds each moment's.
//
// bands is what a route L record's band patches add to the same three sums
// (docs/modify-general-design.md §4.3, brepBandMassOf): the record's faces
// and the patches together are the body's whole boundary. The box needs no
// band term: every patch is ruled between its cap contour, a loop of a
// record face, and its side contour, a rim or segment of the faces beside it,
// so a linear functional over a patch is extremized on those two directrices,
// which the record's faces already hold within their own displacements.
//
// A fillet band's patches are never integrated (docs/loop-fillet-design.md
// §5.3): bands carries its strip terms σ·V_strip and σ·M_strip, and the
// volume and moment sums read every face the band rewrote restored to the
// receiver's (filletRestored), whose difference from the rewritten faces is
// exactly the strip terms the patches' flux would add. Area still reads the
// rewritten faces, and the box adds each fillet band's own extents
// (brepBoundsContext).
func measureBrepContext(ctx context.Context, bp brepPayload, topo *brepTopology, body *Body, bands brepBandMass) error {
	vol3 := bands.vol3
	moments := bands.moments
	area := bands.area
	displaced := 0.0
	envelope := topo.coordUpper
	restored, err := bp.filletRestored(topo)
	if err != nil {
		return err
	}
	for fi, f := range bp.faces {
		if err := ctx.Err(); err != nil {
			return err
		}
		e := topo.embeds[fi]
		rf := restored[fi]
		z0, z1 := proofarith.FloatRat(rf.z0), proofarith.FloatRat(rf.z1)
		if f.planar() {
			region, err := topo.region(ctx, bp, fi)
			if err != nil {
				return err
			}
			volRegion := region
			if rf.region != f.region {
				if volRegion, err = brepRestoredRegion(ctx, rf); err != nil {
					return err
				}
			}
			s := big.NewRat(-1, 1)
			if f.outward {
				s = big.NewRat(1, 1)
			}
			vol3 = proofbound.IntervalAdd(vol3, proofbound.IntervalScale(volRegion.area, proofbound.RatMul(s, z0)))
			k := e.Axis[2]
			half := proofbound.RatMul(big.NewRat(1, 2), s, big.NewRat(int64(e.Sign[2]), 1), z0, z0)
			moments[k] = proofbound.IntervalAdd(moments[k], proofbound.IntervalScale(volRegion.area, half))
			area = proofbound.BoundedAdd(area, region.published)
			displaced = proofbound.AbsSumUpper(displaced,
				proofbound.ProductUpper(f.z0Delta, proofbound.AbsSumUpper(volRegion.upper, volRegion.displacement)))
			continue
		}
		w := topo.walls[fi]
		terms, err := brepSegmentIntegrals(f.wall)
		if err != nil {
			return err
		}
		h := new(big.Rat).Sub(z1, z0)
		vol3 = proofbound.IntervalAdd(vol3, proofbound.IntervalScale(terms[0], proofbound.RatMul(big.NewRat(2, 1), h)))
		for i, mom := range [2]proofbound.RatInterval{terms[1], terms[2]} {
			scale := proofbound.RatMul(big.NewRat(int64(e.Sign[i]), 1), h)
			moments[e.Axis[i]] = proofbound.IntervalAdd(moments[e.Axis[i]], proofbound.IntervalScale(mom, scale))
		}
		area = proofbound.BoundedAdd(area, brepWallArea(f, w))
		heightUpper := proofbound.AbsSumUpper(proofbound.UpRound(rf.z1-rf.z0), f.z0Delta, f.z1Delta)
		band := proofbound.SectionDisplacementArea(f.delta, 1, proofbound.AbsSumUpper(w.Length, w.LengthBound))
		displaced = proofbound.AbsSumUpper(displaced, proofbound.ProductUpper(heightUpper, band))
	}
	envelope = proofbound.AbsSumUpper(envelope, bp.sectionDelta(), bp.axialDelta())
	volume := proofbound.IntervalScale(vol3, big.NewRat(1, 3))
	heldVolume := brepHeld(volume)
	if !(heldVolume > 0) {
		return fmt.Errorf(`%w: a brep body encloses no volume`, ErrDegenerate)
	}
	volumeScalar := proofbound.MeasuredScalar(heldVolume,
		proofbound.AbsSumUpper(proofbound.IntervalFloatError(volume, heldVolume), displaced))
	body.volume = Measurement{Value: units.CubicMillimeters(volumeScalar.Value),
		Exactness: exactnessOf(volumeScalar.Bound), Bound: units.CubicMillimeters(volumeScalar.Bound)}
	body.area = Measurement{Value: units.SquareMillimeters(area.Value), Exactness: exactnessOf(area.Bound),
		Bound: units.SquareMillimeters(area.Bound)}

	var c [3]proofbound.BoundedScalar
	exact := displaced == 0 && volume.Lo.Cmp(volume.Hi) == 0
	for i := range moments {
		exact = exact && moments[i].Lo.Cmp(moments[i].Hi) == 0
	}
	for i, mom := range moments {
		if exact {
			q := new(big.Rat).Quo(mom.Lo, volume.Lo)
			held, _ := q.Float64()
			c[i] = proofbound.MeasuredScalar(held, proofarith.RationalFloatError(q, held))
			continue
		}
		heldMoment := brepHeld(mom)
		momBound := proofbound.AbsSumUpper(proofbound.IntervalFloatError(mom, heldMoment),
			proofbound.ProductUpper(displaced, envelope))
		c[i] = proofbound.BoundedDiv(proofbound.MeasuredScalar(heldMoment, momBound), volumeScalar)
	}
	refView := bp.refView()
	centroidBound := prismPointBound(refView, c[0], c[1], c[2])
	body.centroid = VecMeasurement{Value: refView.point(c[0].Value, c[1].Value, c[2].Value),
		Exactness: exactnessOf(centroidBound), Bound: units.Millimeters(centroidBound)}

	box, err := brepBoundsContext(ctx, bp)
	if err != nil {
		return err
	}
	body.bounds = box
	return validateAnalyticBodyMeasurements(body)
}

// brepBoundsContext is the union of every face's own box (§4.3): each face's
// prism view read along the three world axes, composed with the largest
// section and level displacement as prismBoundsContext composes a prism's. A
// fillet band's patches bulge past their directrices along an oblique axis,
// so each adds its own extents (filletBandExtent).
func brepBoundsContext(ctx context.Context, bp brepPayload) (Box, error) {
	axes := []r3.Vec{r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0), r3.NewVec(0, 0, 1)}
	minC := [3]float64{math.Inf(1), math.Inf(1), math.Inf(1)}
	maxC := [3]float64{math.Inf(-1), math.Inf(-1), math.Inf(-1)}
	extremeBound := 0.0
	work := freeform.NewFreeformWork()
	for _, f := range bp.faces {
		pp := f.view(bp.xform)
		for i, axis := range axes {
			if err := ctx.Err(); err != nil {
				return Box{}, err
			}
			lo, hi, bound, err := pp.extentBoundedAlong(ctx, axis, work, nil)
			if err != nil {
				return Box{}, err
			}
			minC[i], maxC[i] = math.Min(minC[i], lo), math.Max(maxC[i], hi)
			extremeBound = math.Max(extremeBound, bound)
		}
	}
	for _, b := range bp.loopBands {
		if b.kind != brepBandFillet {
			continue
		}
		for i, axis := range axes {
			if err := ctx.Err(); err != nil {
				return Box{}, err
			}
			lo, hi, bound, err := filletBandExtent(ctx, b, bp.faces[b.face], bp.xform, axis, work)
			if err != nil {
				return Box{}, err
			}
			minC[i], maxC[i] = math.Min(minC[i], lo), math.Max(maxC[i], hi)
			extremeBound = math.Max(extremeBound, bound)
		}
	}
	terms := make([]float64, 0, 3)
	for _, term := range []float64{bp.sectionDelta(), extremeBound, bp.axialDelta()} {
		if term != 0 {
			terms = append(terms, term)
		}
	}
	bound := 0.0
	switch len(terms) {
	case 0:
	case 1:
		bound = terms[0]
	default:
		bound = proofbound.AbsSumUpper(terms...)
	}
	return Box{
		Min:       r3.NewVec(minC[0], minC[1], minC[2]),
		Max:       r3.NewVec(maxC[0], maxC[1], maxC[2]),
		Exactness: exactnessOf(bound),
		Bound:     units.Millimeters(bound),
	}, nil
}

// extentAlong is the through-all stop's reading (stops.go): the union of every
// face's own extent along g, and every fillet band's (filletBandExtent,
// docs/loop-fillet-design.md §5.4), beside the largest of their bounds. Like
// a prism's, it refuses a record carrying a section displacement, which moves
// a coordinate the interval is stated over.
func (bp brepPayload) extentAlong(g r3.Vec) (float64, float64, float64, error) {
	if bp.sectionDelta() != 0 {
		return 0, 0, 0, fmt.Errorf(`%w: a through-all stop cannot use a brep body with a proven section displacement`, ErrUnsupported)
	}
	lo, hi, bound := math.Inf(1), math.Inf(-1), 0.0
	work := freeform.NewFreeformWork()
	for _, f := range bp.faces {
		l, h, b, err := f.view(bp.xform).extentBoundedAlong(context.Background(), g, work, nil)
		if err != nil {
			return 0, 0, 0, err
		}
		lo, hi, bound = math.Min(lo, l), math.Max(hi, h), math.Max(bound, b)
	}
	for _, band := range bp.loopBands {
		if band.kind != brepBandFillet {
			continue
		}
		l, h, b, err := filletBandExtent(context.Background(), band, bp.faces[band.face], bp.xform, g, work)
		if err != nil {
			return 0, 0, 0, err
		}
		lo, hi, bound = math.Min(lo, l), math.Max(hi, h), math.Max(bound, b)
	}
	return lo, hi, bound, nil
}

// brepUndercuts surveys a brep body's faces against the pull
// (docs/general-boolean-design.md §4.5's undercut row, DX7's reading): a
// planar face carries one normal, its frame's N turned outward, and a swept
// face sweeps its wall walk's normal range exactly as a prism's side does.
// Both readings go through the exact three-valued decisions prismUndercuts
// uses (survey2d.CapNormalDecision, survey2d.WallNormalDecision) over each
// face's own placed frame, so a straddling face sets undecided without
// discarding a face already proven to oppose. Every outward normal maps
// through the placement's linear part, which a reflection maps correctly. A
// fillet band's pipe patches are read by brepFilletUndercuts (Table DF's DF7).
// The reading is a normal-direction membership, unaffected by a face's
// displacements, as a prism's is (docs/prism-boolean-design.md §12).
func brepUndercuts(budget *proofbound.WorkBudget, b *Body, bp brepPayload, pull r3.Vec) undercutOutcome {
	p, ok := pull.Normalize()
	if !ok {
		return undercutOutcome{}
	}
	roles := facesByRole(b)
	faces := []*Face{}
	undecided := false
	work := freeform.NewFreeformWork()
	for _, f := range bp.faces {
		face := roles[f.role]
		if face == nil {
			return undercutOutcome{}
		}
		view := f.view(bp.xform)
		m, ok := survey2d.NewPlacedFrameMap(view.frame, view.xform)
		if !ok {
			return undercutOutcome{}
		}
		var verdict survey2d.PullVerdict
		if f.planar() {
			sign := -1.0
			if f.outward {
				sign = 1
			}
			verdict, ok = survey2d.CapNormalDecision(m, pull, sign)
		} else {
			w, err := boundarywalk.WalkOf(f.wall, work)
			if err != nil {
				return undercutOutcome{}
			}
			verdict, ok = survey2d.WallNormalDecision(survey2d.SideWalk{SegmentWalk: w, Segs: []int{0}}, m, pull)
		}
		if !listVerdict(&faces, &undecided, face, verdict, ok) {
			return undercutOutcome{}
		}
	}
	// A route L body's band patches are no face of the record: each is read
	// through its own built Face.NormalAt in its band's face frame, by the
	// reader a cap-blend body's patches take (modify-general Table DG's DG7).
	for bi, band := range bp.loopBands {
		if bi >= len(bp.loopPatches) {
			return undercutOutcome{}
		}
		if band.kind == brepBandFillet {
			if !brepFilletUndercuts(budget, roles, bp, band, pull, &faces, &undecided, work) {
				return undercutOutcome{}
			}
			continue
		}
		pl := bp.faces[band.face].view(bp.xform)
		if !capPatchUndercuts(roles, pl, bp.loopPatches[bi], p, &faces, &undecided) {
			return undercutOutcome{}
		}
	}
	if undecided && len(faces) == 0 {
		// Keep an entirely undecided result distinct from a proven all-clear.
		faces = nil
	}
	return undercutOutcome{faces: faces, ok: true, undecided: undecided}
}

// brepMinRadius is the tightest concave radius over a brep body's faces
// (§4.5's minimum-radius row, DX8's reading): only a swept face whose wall is
// a circle or an arc walked clockwise in its own frame, with the material on
// its left, curves away from the material — a hole's wall, a groove — and
// its radius is the walk's own, entering the §9.2 aggregate under its own
// proven radius bound, as prismMinRadius takes a prism side's. Planar faces
// carry no radius. A record carrying any section displacement leaves the
// question undecided, as prismMinRadius does: the radius is read off the
// recorded wall, which the face only denotes within that displacement.
//
// A route L body's band patches add no radius the record's walls do not
// already read, on capBlendMinRadius's reduction (capblend_survey.go): a
// Plane is flat, a Cone's tightest azimuthal radius is its side wall's own,
// and an apex Cone shrinks to zero only at a boundary vertex. That holds only
// for a patch the build proves is the Cone it publishes, so a band whose
// patch carries a skew or a non-zero stamped departure leaves the survey
// undecided (modify-general Table DG's DG8). A fillet band that fills a
// concave corner adds its tube radius exactly; one that removes material is
// convex in its tube direction and adds none (loop-fillet Table DF's DF8).
func brepMinRadius(b *Body, bp brepPayload) (radiusOutcome, bool) {
	if bp.sectionDelta() != 0 {
		return radiusOutcome{}, false
	}
	if len(bp.loopPatches) != len(bp.loopBands) {
		return radiusOutcome{}, false
	}
	roles := facesByRole(b)
	for _, patches := range bp.loopPatches {
		for _, patch := range patches {
			f := roles[patch.role]
			if f == nil || capPatchWindowSkew(patch.geom) > 0 || f.normalBound != 0 {
				return radiusOutcome{}, false
			}
		}
	}
	agg := survey2d.MinAggregate()
	work := freeform.NewFreeformWork()
	for _, band := range bp.loopBands {
		if band.kind == brepBandFillet && band.sigma > 0 {
			// The tag's Minor or Radius, which a placement leaves unchanged,
			// within the radius's unit conversion.
			agg.Take(band.setback.dc, band.setback.dcDelta)
		}
	}
	for _, f := range bp.faces {
		if f.planar() {
			continue
		}
		w, err := boundarywalk.WalkOf(f.wall, work)
		if err != nil {
			return radiusOutcome{}, false
		}
		if w.IsCircular() && w.Th1 < w.Th0 {
			agg.Take(w.Radius, w.RadiusBound)
		}
	}
	return radiusOutcomeOf(agg)
}
