package decad

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/brepgeom"
	"github.com/lestrrat-3d/decad/internal/capblend"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/radiussurvey"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
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
// prism wall's own (prismrecord.BuildWallGeometry).
func brepFaceBody(ctx context.Context, bp brepPayload, btopo *brepTopology, fi int, body *Body, ref producerID) (*Face, error) {
	f := bp.Faces[fi]
	pp := f.View(bp.Xform)
	origins := []provenance.Ref{{Producer: ref, Role: f.Role}}
	if f.Blend != "" {
		origins = append(origins, provenance.Ref{Producer: ref, Role: fmt.Sprintf("%s(%d)", f.Blend, fi)})
	}
	if f.Planar() {
		frame, err := pp.CapFrame(f.Z0, !f.Outward)
		if err != nil {
			return nil, err
		}
		region, err := btopo.Region(ctx, bp, fi)
		if err != nil {
			return nil, err
		}
		return bornFace(&Face{
			n: topo.Face{
				Area:          region.Published.Value,
				AreaBound:     region.Published.Bound,
				AxialDelta:    f.Z0Delta,
				HasAxialDelta: true,
				Surface:       surfacegeom.Plane{Frame: frame},
				CapRecord:     &topo.CapRecord{Frame: pp.Frame, Xform: pp.Xform, Z: f.Z0},
				Origins:       origins,
				Body:          bodyNode(body),
			}}), nil
	}
	w := btopo.Walls[fi]
	_, _, surf, reversed, err := prismrecord.BuildWallGeometry(handleAlloc{}, pp, survey2d.SideWalk{SegmentWalk: w, Segs: []int{0}}, false, w.Closed, nil, nil, nil, nil)
	if err != nil {
		return nil, err
	}
	area := brepgeom.WallArea(f.Delta, f.Z0, f.Z1, f.Z0Delta, f.Z1Delta, w)
	return bornFace(&Face{n: topo.Face{
		Area:      area.Value,
		AreaBound: area.Bound,
		Reversed:  reversed,
		Surface:   surf,
		Origins:   origins,
		Body:      bodyNode(body),
	}}), nil
}

// region reads a planar face's region once per topology.
func (btopo *brepTopology) Region(ctx context.Context, bp brepPayload, fi int) (brepRegion, error) {
	if r, ok := btopo.Regions[fi]; ok {
		return r, nil
	}
	f := bp.Faces[fi]
	r, err := brepgeom.RegionOf(ctx, f.Region, f.Delta, btopo.Planar[fi])
	if err != nil {
		return brepRegion{}, err
	}
	if btopo.Regions == nil {
		btopo.Regions = map[int]brepRegion{}
	}
	btopo.Regions[fi] = r
	return r, nil
}

// measureBrepContext adapts the body's face records to brepgeom.IntegrateFaces
// and publishes its readings, topology bounds and final validation.
func measureBrepContext(ctx context.Context, bp brepPayload, btopo *brepTopology, body *Body, bands brepBandMass) error {
	restored, err := bp.FilletRestored(btopo)
	if err != nil {
		return err
	}
	readings, err := brepgeom.IntegrateFaces(ctx, len(bp.Faces), func(fi int) (brepgeom.FaceMeasure, error) {
		f, rf := bp.Faces[fi], restored[fi]
		record := brepgeom.FaceMeasure{
			Embed: btopo.Embeds[fi], Planar: f.Planar(), Outward: f.Outward,
			Z0: f.Z0, Z1: f.Z1, RestoredZ0: rf.Z0, RestoredZ1: rf.Z1,
			Z0Delta: f.Z0Delta, Z1Delta: f.Z1Delta, Delta: f.Delta,
		}
		if !record.Planar {
			record.Wall, record.Walk = f.Wall, btopo.Walls[fi]
			return record, nil
		}
		region, err := btopo.Region(ctx, bp, fi)
		if err != nil {
			return brepgeom.FaceMeasure{}, err
		}
		record.Region, record.RestoredRegion = region, region
		if rf.Region != f.Region {
			record.RestoredRegion, err = brepgeom.RestoredRegion(ctx, rf.Region, rf.Delta)
		}
		return record, err
	}, brepgeom.FaceMass{Vol3: bands.vol3, Moments: bands.moments, Area: bands.area},
		btopo.CoordUpper, bp.SectionDelta(), bp.AxialDelta())
	if err != nil {
		return err
	}
	body.n.Volume = measurement.Measurement{Value: units.CubicMillimeters(readings.Volume.Value),
		Exactness: measurement.ExactnessOf(readings.Volume.Bound), Bound: units.CubicMillimeters(readings.Volume.Bound)}
	body.n.Area = measurement.Measurement{Value: units.SquareMillimeters(readings.Area.Value),
		Exactness: measurement.ExactnessOf(readings.Area.Bound), Bound: units.SquareMillimeters(readings.Area.Bound)}
	refView := bp.RefView()
	c := readings.Centroid
	centroidBound := refView.PointBound(c[0], c[1], c[2])
	body.n.Centroid = measurement.VecMeasurement{Value: refView.Point(c[0].Value, c[1].Value, c[2].Value),
		Exactness: measurement.ExactnessOf(centroidBound), Bound: units.Millimeters(centroidBound)}

	box, err := brepBoundsContext(ctx, bp)
	if err != nil {
		return err
	}
	body.n.Bounds = boxToInternal(box)
	return topobuild.ValidateAnalyticMeasurements(bodyNode(body))
}

// brepBoundsContext is the union of every face's own box (§4.3): each face's
// prism view read along the three world axes, composed with the largest
// section and level displacement as prismrecord.Bounds composes a prism's. A
// fillet band's patches bulge past their directrices along an oblique axis,
// so each adds its own extents (filletBandExtent).
func brepBoundsContext(ctx context.Context, bp brepPayload) (Box, error) {
	minC, maxC, bound, err := brepgeom.Bounds(ctx, len(bp.Faces)+len(bp.LoopBands), bp.ExtentAt,
		bp.SectionDelta(), bp.AxialDelta())
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
func (bp brepPayload) ExtentAt(ctx context.Context, item int, axis r3.Vec,
	work *freeform.FreeformWork) (float64, float64, float64, bool, error) {
	if item < len(bp.Faces) {
		lo, hi, bound, err := bp.Faces[item].View(bp.Xform).ExtentBoundedAlong(ctx, axis, work, nil)
		return lo, hi, bound, true, err
	}
	band := bp.LoopBands[item-len(bp.Faces)]
	if band.Kind != brepBandFillet {
		return 0, 0, 0, false, nil
	}
	lo, hi, bound, err := filletBandExtent(ctx, band, bp.Faces[band.Face], bp.Xform, axis, work)
	return lo, hi, bound, true, err
}

// ExtentAlong is the through-all stop's reading (stops.go): the union of every
// face's own extent along g, and every fillet band's (filletBandExtent,
// docs/loop-fillet-design.md §5.4), beside the largest of their bounds. Like
// a prism's, it refuses a record carrying a section displacement, which moves
// a coordinate the interval is stated over.
func (bp brepPayload) ExtentAlong(g r3.Vec) (float64, float64, float64, error) {
	if bp.SectionDelta() != 0 {
		return 0, 0, 0, fmt.Errorf(`%w: a through-all stop cannot use a brep body with a proven section displacement`, ErrUnsupported)
	}
	return brepgeom.ExtentAlong(context.Background(), len(bp.Faces)+len(bp.LoopBands), g, bp.ExtentAt)
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
	if bp.BossShell != nil || bp.PocketShell != nil {
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
	for _, f := range bp.Faces {
		face := roles[f.Role]
		if face == nil {
			return undercutOutcome{}
		}
		verdict, ok := brepgeom.PullDecision(brepgeom.PullFace{
			Frame: f.Frame, Transform: bp.Xform, Wall: f.Wall,
			Planar: f.Planar(), Outward: f.Outward,
		}, pull, work)
		if !listVerdict(&faces, &undecided, face, verdict, ok) {
			return undercutOutcome{}
		}
	}
	// A route L body's band patches are no face of the record: each is read
	// through its own built Face.NormalAt in its band's face frame, by the
	// reader a cap-blend body's patches take (modify-general Table DG's DG7).
	for bi, band := range bp.LoopBands {
		if bi >= len(bp.LoopPatches) {
			return undercutOutcome{}
		}
		if band.Kind == brepBandFillet {
			if !brepFilletUndercuts(budget, roles, bp, band, pull, &faces, &undecided, work) {
				return undercutOutcome{}
			}
			continue
		}
		pl := bp.Faces[band.Face].View(bp.Xform)
		if !capPatchUndercuts(roles, pl, bp.LoopPatches[bi], p, &faces, &undecided) {
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
	if bp.SectionDelta() != 0 {
		return radiusOutcome{}, false
	}
	if len(bp.LoopPatches) != len(bp.LoopBands) {
		return radiusOutcome{}, false
	}
	roles := facesByRole(b)
	for _, patches := range bp.LoopPatches {
		for _, patch := range patches {
			f := roles[patch.Role]
			if f == nil || capblend.PatchWindowSkew(patch.Geom) > 0 || f.n.NormalBound != 0 {
				return radiusOutcome{}, false
			}
		}
	}
	fillets := make([]radiussurvey.BrepFilletRadius, 0, len(bp.LoopBands))
	if bp.BossShell != nil {
		fillets = append(fillets, radiussurvey.BrepFilletRadius{Radius: bp.BossShell.T})
	}
	if bp.PocketShell != nil {
		fillets = append(fillets, radiussurvey.BrepFilletRadius{Radius: bp.PocketShell.T})
	}
	for _, band := range bp.LoopBands {
		if band.Kind == brepBandFillet && band.Sigma > 0 {
			// The tag's Minor or Radius, which a placement leaves unchanged,
			// within the radius's unit conversion.
			fillets = append(fillets, radiussurvey.BrepFilletRadius{
				Radius: band.Setback.Dc, Bound: band.Setback.DcDelta,
			})
		}
	}
	walls := make([]sectionrecord.CurveSegment, 0, len(bp.Faces))
	for _, f := range bp.Faces {
		if !f.Planar() {
			walls = append(walls, f.Wall)
		}
	}
	reading := radiussurvey.Brep(walls, fillets, bp.SectionDelta())
	return radiusOutcome{reading: reading.Reading, bound: reading.Bound, ok: reading.OK}, reading.OK
}
