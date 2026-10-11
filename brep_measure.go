package decad

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/brepgeom"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/radiussurvey"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
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

type brepRegion = brepgeom.Region

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
			area: region.Published.Value, areaBound: region.Published.Bound,
			axialDelta: f.z0Delta, hasAxialDelta: true}, nil
	}
	w := topo.walls[fi]
	_, _, surf, reversed, err := buildWallGeometry(pp, survey2d.SideWalk{SegmentWalk: w, Segs: []int{0}},
		false, w.Closed, nil, nil, nil, nil)
	if err != nil {
		return nil, err
	}
	area := brepgeom.WallArea(f.delta, f.z0, f.z1, f.z0Delta, f.z1Delta, w)
	return &Face{surface: surf, origins: origins, body: body, area: area.Value, areaBound: area.Bound,
		reversed: reversed}, nil
}

// region reads a planar face's region once per topology.
func (topo *brepTopology) region(ctx context.Context, bp brepPayload, fi int) (brepRegion, error) {
	if r, ok := topo.regions[fi]; ok {
		return r, nil
	}
	f := bp.faces[fi]
	r, err := brepgeom.RegionOf(ctx, f.region, f.delta, topo.planar[fi])
	if err != nil {
		return brepRegion{}, err
	}
	if topo.regions == nil {
		topo.regions = map[int]brepRegion{}
	}
	topo.regions[fi] = r
	return r, nil
}

// measureBrepContext adapts the body's face records to brepgeom.IntegrateFaces
// and publishes its readings, topology bounds and final validation.
func measureBrepContext(ctx context.Context, bp brepPayload, topo *brepTopology, body *Body, bands brepBandMass) error {
	restored, err := bp.filletRestored(topo)
	if err != nil {
		return err
	}
	readings, err := brepgeom.IntegrateFaces(ctx, len(bp.faces), func(fi int) (brepgeom.FaceMeasure, error) {
		f, rf := bp.faces[fi], restored[fi]
		record := brepgeom.FaceMeasure{
			Embed: topo.embeds[fi], Planar: f.planar(), Outward: f.outward,
			Z0: f.z0, Z1: f.z1, RestoredZ0: rf.z0, RestoredZ1: rf.z1,
			Z0Delta: f.z0Delta, Z1Delta: f.z1Delta, Delta: f.delta,
		}
		if !record.Planar {
			record.Wall, record.Walk = f.wall, topo.walls[fi]
			return record, nil
		}
		region, err := topo.region(ctx, bp, fi)
		if err != nil {
			return brepgeom.FaceMeasure{}, err
		}
		record.Region, record.RestoredRegion = region, region
		if rf.region != f.region {
			record.RestoredRegion, err = brepgeom.RestoredRegion(ctx, rf.region, rf.delta)
		}
		return record, err
	}, brepgeom.FaceMass{Vol3: bands.vol3, Moments: bands.moments, Area: bands.area},
		topo.coordUpper, bp.sectionDelta(), bp.axialDelta())
	if err != nil {
		return err
	}
	body.volume = Measurement{Value: units.CubicMillimeters(readings.Volume.Value),
		Exactness: exactnessOf(readings.Volume.Bound), Bound: units.CubicMillimeters(readings.Volume.Bound)}
	body.area = Measurement{Value: units.SquareMillimeters(readings.Area.Value),
		Exactness: exactnessOf(readings.Area.Bound), Bound: units.SquareMillimeters(readings.Area.Bound)}
	refView := bp.refView()
	c := readings.Centroid
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
	minC, maxC, bound, err := brepgeom.Bounds(ctx, len(bp.faces)+len(bp.loopBands), bp.extentAt,
		bp.sectionDelta(), bp.axialDelta())
	if err != nil {
		return Box{}, err
	}
	return Box{
		Min:       minC,
		Max:       maxC,
		Exactness: exactnessOf(bound),
		Bound:     units.Millimeters(bound),
	}, nil
}

// extentAt adapts one face or band to the shared BRep extent reader.
func (bp brepPayload) extentAt(ctx context.Context, item int, axis r3.Vec,
	work *freeform.FreeformWork) (float64, float64, float64, bool, error) {
	if item < len(bp.faces) {
		lo, hi, bound, err := bp.faces[item].view(bp.xform).extentBoundedAlong(ctx, axis, work, nil)
		return lo, hi, bound, true, err
	}
	band := bp.loopBands[item-len(bp.faces)]
	if band.kind != brepBandFillet {
		return 0, 0, 0, false, nil
	}
	lo, hi, bound, err := filletBandExtent(ctx, band, bp.faces[band.face], bp.xform, axis, work)
	return lo, hi, bound, true, err
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
	return brepgeom.ExtentAlong(context.Background(), len(bp.faces)+len(bp.loopBands), g, bp.extentAt)
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
	if bp.bossShell != nil || bp.pocketShell != nil {
		return undercutOutcome{reason: surveyPayloadStaged}
	}
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
		verdict, ok := brepgeom.PullDecision(brepgeom.PullFace{
			Frame: f.frame, Transform: bp.xform, Wall: f.wall,
			Planar: f.planar(), Outward: f.outward,
		}, pull, work)
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
// proven radius bound, as radiussurvey.Prism takes a prism side's. Planar faces
// carry no radius. A record carrying any section displacement leaves the
// question undecided, as radiussurvey.Prism does: the radius is read off the
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
	fillets := make([]radiussurvey.BrepFilletRadius, 0, len(bp.loopBands))
	if bp.bossShell != nil {
		fillets = append(fillets, radiussurvey.BrepFilletRadius{Radius: bp.bossShell.t})
	}
	if bp.pocketShell != nil {
		fillets = append(fillets, radiussurvey.BrepFilletRadius{Radius: bp.pocketShell.t})
	}
	for _, band := range bp.loopBands {
		if band.kind == brepBandFillet && band.sigma > 0 {
			// The tag's Minor or Radius, which a placement leaves unchanged,
			// within the radius's unit conversion.
			fillets = append(fillets, radiussurvey.BrepFilletRadius{
				Radius: band.setback.dc, Bound: band.setback.dcDelta,
			})
		}
	}
	walls := make([]sectionrecord.CurveSegment, 0, len(bp.faces))
	for _, f := range bp.faces {
		if !f.planar() {
			walls = append(walls, f.wall)
		}
	}
	reading := radiussurvey.Brep(walls, fillets, bp.sectionDelta())
	return radiusOutcome{reading: reading.Reading, bound: reading.Bound, ok: reading.OK}, reading.OK
}
