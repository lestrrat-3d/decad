package contactbox

import (
	"math"

	vocab "github.com/lestrrat-3d/decad/report"

	"github.com/lestrrat-3d/decad/internal/clearance"
	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/decad/internal/pair"
	"github.com/lestrrat-3d/decad/internal/pair/box"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/units"

	"github.com/lestrrat-3d/decad/internal/proofbound"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
)

// SourceBox is the complete exact occupied interval of a source
// rectangular prism under a read pose. The sweep uses the full intervals for
// one-sided departure and persistent-contact proofs.
type SourceBox struct {
	Lo, Hi [3]proofarith.Dyadic
	// faces[axis][0] is the original minimum-support face; [1] is maximum.
	Faces [3][2]*topo.Face
}

func (b SourceBox) AxisBox() box.AxisBox {
	return box.AxisBox{Lo: b.Lo, Hi: b.Hi}
}

// SourceBoxAtPose never infers occupancy from a held bounding box. It maps all
// eight corners of a source-certified rectangle through its recorded frame,
// accumulated placement, and query pose with exact dyadic operations.
func SourceBoxAtPose(b *topo.Body, pose r3.Transform) (SourceBox, bool) {
	pp, ok := b.Payload.(prismrecord.Payload)
	if !ok || !b.Solid || b.Kind != topo.BodySolid || pp.SurfaceResult ||
		pp.SectionDelta != 0 || pp.Z0Delta != 0 || pp.Z1Delta != 0 ||
		!box.RectangularProfile(pp.Profile) ||
		!box.CardinalBasis(pp.Frame.U(), pp.Frame.V(), pp.Frame.N()) ||
		!box.SignedAxisTransform(pp.Xform) || !box.SignedAxisTransform(pose) {
		return SourceBox{}, false
	}
	if !proofbound.FiniteVec(pp.Frame.Origin()) || !measurement.FiniteValues(pp.Z0, pp.Z1) {
		return SourceBox{}, false
	}
	var umin, umax, vmin, vmax float64
	for i, seg := range pp.Profile.Outer.Segments {
		line, ok := seg.(sectionrecord.LineSeg)
		if !ok {
			return SourceBox{}, false
		}
		p := line.Start
		if i == 0 {
			umin, umax, vmin, vmax = p.U, p.U, p.V, p.V
		}
		umin, umax = math.Min(umin, p.U), math.Max(umax, p.U)
		vmin, vmax = math.Min(vmin, p.V), math.Max(vmax, p.V)
	}
	if !measurement.FiniteValues(umin, umax, vmin, vmax) || pp.Z0 >= pp.Z1 {
		return SourceBox{}, false
	}
	numeric, ok := box.SourceAxisBox(pp.Frame, pp.Xform, pose,
		[2]float64{umin, umax}, [2]float64{vmin, vmax}, [2]float64{pp.Z0, pp.Z1})
	if !ok {
		return SourceBox{}, false
	}
	box := SourceBox{Lo: numeric.Lo, Hi: numeric.Hi}
	faces := topobuild.Faces(b)
	if len(faces) != 6 {
		return SourceBox{}, false
	}
	for _, face := range faces {
		plane, ok := face.Surface.(surfacegeom.Plane)
		if !ok || face.NormalBound != 0 {
			return SourceBox{}, false
		}
		normal := plane.Frame.N()
		if face.Reversed {
			normal = normal.Scale(-1)
		}
		normal = pose.ApplyDir(normal)
		axis, side, ok := clearance.SignedAxis(normal)
		if !ok || box.Faces[axis][side] != nil {
			return SourceBox{}, false
		}
		box.Faces[axis][side] = face
	}
	for axis := range 3 {
		if box.Faces[axis][0] == nil || box.Faces[axis][1] == nil {
			return SourceBox{}, false
		}
	}
	return box, true
}

// The root package maps neutral pair readings to report units and topology.
func PublishSourceBoxPatch(report *contactReport, a, b SourceBox, axis, sign int,
	separation proofarith.Dyadic) {
	patch, reason := box.FacePatch(
		box.AxisBox{Lo: a.Lo, Hi: a.Hi}, box.AxisBox{Lo: b.Lo, Hi: b.Hi},
		axis, sign, separation,
		box.AxisBoxRequest{PointResolutionMM: report.Request.PointResolution.Base()},
	)
	report.Reason = pair.ContactReasonOf(reason)
	if patch != nil {
		publishAxisBoxPatch(report, a, b, patch)
	}
}

func ClassifySourceBoxes(report *contactReport, a, b SourceBox) {
	result := box.ClassifyAxisBoxes(
		box.AxisBox{Lo: a.Lo, Hi: a.Hi},
		box.AxisBox{Lo: b.Lo, Hi: b.Hi},
		box.AxisBoxRequest{PointResolutionMM: report.Request.PointResolution.Base()},
	)
	switch result.Relation {
	case pair.Separated:
		report.Relation = vocab.ContactSeparated
	case pair.Touching:
		report.Relation = vocab.ContactTouching
	case pair.Overlapping:
		report.Relation = vocab.ContactOverlapping
	default:
		report.Relation = vocab.ContactUndecided
	}
	report.Reason = pair.ContactReasonOf(result.Reason)
	if result.Gap != nil {
		gap := result.Gap.Measurement()
		report.Gap = &gap
	}
	if result.Patch == nil {
		return
	}
	publishAxisBoxPatch(report, a, b, result.Patch)
}

func publishAxisBoxPatch(report *contactReport, a, b SourceBox, patch *box.AxisBoxPatch) {
	faceA := a.Faces[patch.FaceA.Axis][patch.FaceA.Side]
	faceB := b.Faces[patch.FaceB.Axis][patch.FaceB.Side]
	var normal r3.Vec
	switch patch.NormalAxis {
	case 0:
		normal.X = float64(patch.NormalSign)
	case 1:
		normal.Y = float64(patch.NormalSign)
	case 2:
		normal.Z = float64(patch.NormalSign)
	}
	separation := patch.Separation.Measurement()
	points := make([]contactPoint, 0, len(patch.Points))
	for _, point := range patch.Points {
		points = append(points, contactPoint{
			OnA:         point.OnA.Measurement(),
			OnB:         point.OnB.Measurement(),
			Normal:      measurement.VecMeasurement{Value: normal, Exactness: measurement.Exact, Bound: units.Scalar(0)},
			NormalAngle: units.Radians(0),
			Separation:  separation,
			FaceA:       faceA, FaceB: faceB,
			FeatureA: contactFeature{Face: faceA},
			FeatureB: contactFeature{Face: faceB},
		})
	}
	report.Manifold = &contactManifold{Points: points}
}
