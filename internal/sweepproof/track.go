package sweepproof

import (
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/topo"
	vocab "github.com/lestrrat-3d/decad/report"

	"github.com/lestrrat-3d/decad/internal/contactbox"
	"github.com/lestrrat-3d/decad/internal/contactfaceted"
	"github.com/lestrrat-3d/decad/internal/contactround"
	"github.com/lestrrat-3d/decad/internal/measurement"
	pairbox "github.com/lestrrat-3d/decad/internal/pair/box"
	"github.com/lestrrat-3d/decad/internal/planarsweep"
	"github.com/lestrrat-3d/decad/internal/sweeppath"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// Track owns the exact source geometry and affine motion of a certified
// touching prefix, as the public vocab.SweepContactTrackOf reads it. Its
// source faces are the original faces.
type Track struct {
	A, B          contactbox.SourceBox
	DeltaA        [3]proofarith.Dyadic
	DeltaB        [3]proofarith.Dyadic
	Sphere        *contactround.SphereProof
	SpherePair    *[2]contactround.SphereProof
	SphereFirst   bool
	Cylinder      *contactround.CylinderProof // disk-on-face track (internal/sweeprun/cylinder.go)
	CylinderFirst bool
	From, To      *big.Rat
	Duration      *big.Rat
	Request       vocab.ContactRequest
	Sources       [2]contactFeature
	NormalRead    vocab.VecMeasurement
	PointCount    int
	OrientedA     *contactbox.OrientedSourceBox
	OrientedB     *contactbox.OrientedSourceBox
	OrientedDelta [3]proofarith.Dyadic
	Planar        *planarsweep.TrackProof   // general planar touch or band (internal/sweeprun/planar.go)
	Rolling       *planarsweep.RollingTrack // rolling cylinder touch or band (internal/sweeprun/rolling.go)
}

func (t *Track) Start() vocab.SweepInstant { return sweeppath.Instant(t.From, t.Duration) }

func (t *Track) End() vocab.SweepInstant { return sweeppath.Instant(t.To, t.Duration) }

func (t *Track) Features() (contactFeature, contactFeature) {
	return (t.Sources[0]), (t.Sources[1])
}

func (t *Track) Normal() vocab.VecMeasurement { return t.NormalRead }

// Band returns the depth of a band track (docs/multibody-dynamics-design.md
// §10.3), a Length Measurement whose Value plus Bound bounds every contact
// point's separation in both directions and the interiors' overlap along
// Normal(). It is nil for an exact touch track.
func (t *Track) Band() *vocab.Measurement {
	var band *vocab.Measurement
	switch {
	case t.Planar != nil:
		band = t.Planar.Band
	case t.Rolling != nil:
		band = t.Rolling.Band
	}
	if band == nil {
		return nil
	}
	out := *band
	return &out
}

// BandAt returns the depth of a band track over its prefix through fraction
// (docs/multibody-dynamics-design.md §10.3): a Length Measurement whose Value
// plus Bound bounds every contact point's separation in both directions and
// the interiors' overlap along Normal() at every instant from the track's
// start through fraction. At the track's end it is at most Band(); it grows
// with fraction, so a caller can end a slice where the band reaches its own
// residual. It is nil for an exact touch track.
func (t *Track) BandAt(fraction units.Value) (*vocab.Measurement, error) {
	if fraction.Kind() != units.Dimensionless {
		return nil, fmt.Errorf("%w: contact-track fraction must be dimensionless", decaderr.ErrUnitKind)
	}
	f, ok := sweeppath.ExactBaseValue(fraction)
	if !ok || !measurement.FiniteValues(fraction.Base()) {
		return nil, fmt.Errorf("%w: nonfinite contact-track fraction", decaderr.ErrNotFinite)
	}
	if f.Cmp(t.From) < 0 || f.Cmp(t.To) > 0 {
		return nil, fmt.Errorf("%w: fraction is outside contact track", decaderr.ErrDegenerate)
	}
	var depth *big.Rat
	switch {
	case t.Planar != nil && t.Planar.Band != nil:
		if depth, ok = t.Planar.DepthThrough(f); !ok {
			return nil, fmt.Errorf("%w: band depth has no finite bound", decaderr.ErrUnsupported)
		}
	case t.Rolling != nil && t.Rolling.Band != nil:
		depth = t.Rolling.DepthThrough(f)
	default:
		return nil, nil //nolint:nilnil // an exact touch track has no band, as Band() reports
	}
	value := sweeppath.RatFloatNearest(depth)
	bound := proofarith.RationalFloatError(depth, value)
	if !measurement.FiniteValues(value, bound) {
		return nil, fmt.Errorf("%w: band depth has no finite bound", decaderr.ErrUnsupported)
	}
	return &vocab.Measurement{Value: units.Millimeters(value), Bound: units.Millimeters(bound),
		Exactness: measurement.ExactnessOf(bound)}, nil
}

// ManifoldAt returns a fresh reduction of the complete exact touching set at
// a fraction in this track's certified interval.
func (t *Track) ManifoldAt(fraction units.Value) (*contactManifold, error) {
	if fraction.Kind() != units.Dimensionless {
		return nil, fmt.Errorf("%w: contact-track fraction must be dimensionless", decaderr.ErrUnitKind)
	}
	f, ok := sweeppath.ExactBaseValue(fraction)
	if !ok || !measurement.FiniteValues(fraction.Base()) {
		return nil, fmt.Errorf("%w: nonfinite contact-track fraction", decaderr.ErrNotFinite)
	}
	if f.Cmp(t.From) < 0 || f.Cmp(t.To) > 0 {
		return nil, fmt.Errorf("%w: fraction is outside contact track", decaderr.ErrDegenerate)
	}
	if t.Planar != nil {
		return planarManifoldAt(t.Planar, f, t.Request)
	}
	if t.Rolling != nil {
		manifold, err := t.Rolling.ManifoldAt(f, t.Request)
		return (manifold), err
	}
	if t.Cylinder != nil {
		return t.cylinderTrackManifold(f)
	}
	if t.OrientedA != nil && t.OrientedB != nil {
		// Translation keeps both boxes' source face identities.
		a, b := *t.OrientedA, *t.OrientedB
		var okA, okB bool
		a.OrientedBox, okA = pairbox.TranslateOriented(a.OrientedBox, t.OrientedDelta, f)
		b.OrientedBox, okB = pairbox.TranslateOriented(b.OrientedBox, t.OrientedDelta, f)
		if !okA || !okB {
			return nil, fmt.Errorf("%w: oriented contact-track fraction cannot be represented", decaderr.ErrUnsupported)
		}
		report := &contactReport{Request: t.Request}
		report.Relation, _, _ = contactbox.OrientedBoxRelation(a, b)
		if report.Relation == vocab.ContactTouching {
			contactbox.PublishOrientedBoxPatch(report, a, b)
			if report.Manifold == nil {
				contactbox.PublishClippedHorizontalPatch(report, a, b)
			}
		}
		if report.Relation != vocab.ContactTouching || report.Manifold == nil ||
			len(report.Manifold.Points) != t.PointCount {
			return nil, fmt.Errorf("%w: oriented contact track lost its face patch", decaderr.ErrUnsupported)
		}
		return (report.Manifold), nil
	}
	if t.Sphere != nil {
		sphereDelta, boxDelta := t.DeltaB, t.DeltaA
		box := t.A
		if t.SphereFirst {
			sphereDelta, boxDelta, box = t.DeltaA, t.DeltaB, t.B
		}
		sphere, okSphere := contactround.TranslatedSphere(*t.Sphere, sphereDelta, f)
		box, okBox := TranslatedContactBox(box, boxDelta, f)
		if !okSphere || !okBox {
			return nil, fmt.Errorf("%w: contact-track fraction cannot be represented", decaderr.ErrUnsupported)
		}
		report := &contactReport{Request: t.Request}
		contactround.ClassifySphereBox(report, sphere, box, t.SphereFirst)
		if report.Relation != vocab.ContactTouching || report.Manifold == nil {
			return nil, fmt.Errorf("%w: contact track has no bounded sphere point", decaderr.ErrUnsupported)
		}
		return (report.Manifold), nil
	}
	if t.SpherePair != nil {
		a, okA := contactround.TranslatedSphere(t.SpherePair[0], t.DeltaA, f)
		b, okB := contactround.TranslatedSphere(t.SpherePair[1], t.DeltaB, f)
		if !okA || !okB {
			return nil, fmt.Errorf("%w: sphere-pair contact-track fraction cannot be represented", decaderr.ErrUnsupported)
		}
		report := &contactReport{Request: t.Request}
		contactround.ClassifySpherePair(report, a, b)
		if report.Relation != vocab.ContactTouching || report.Manifold == nil ||
			len(report.Manifold.Points) != 1 ||
			report.Manifold.Points[0].FeatureA != t.Sources[0] ||
			report.Manifold.Points[0].FeatureB != t.Sources[1] {
			return nil, fmt.Errorf("%w: sphere-pair contact track lost its point", decaderr.ErrUnsupported)
		}
		return (report.Manifold), nil
	}
	a, b, ok := TranslatedSourceBoxes(t.A, t.B, t.DeltaA, t.DeltaB, f)
	if !ok {
		return nil, fmt.Errorf("%w: fraction is not representable as a source-box translation", decaderr.ErrUnsupported)
	}
	report := &contactReport{Request: t.Request}
	contactbox.ClassifySourceBoxes(report, a, b)
	if report.Relation != vocab.ContactTouching || report.Manifold == nil {
		return nil, fmt.Errorf("%w: contact track has no bounded face patch", decaderr.ErrUnsupported)
	}
	return (report.Manifold), nil
}

// cylinderTrackManifold reduces the track's exact cylinder and box at one
// fraction and requires the track's single disk-face point.
func (t *Track) cylinderTrackManifold(f *big.Rat) (*contactManifold, error) {
	cylinderBox, box := t.B, t.A
	cylinderDelta, boxDelta := t.DeltaB, t.DeltaA
	if t.CylinderFirst {
		cylinderBox, box = t.A, t.B
		cylinderDelta, boxDelta = t.DeltaA, t.DeltaB
	}
	movedCylinder, okCylinder := TranslatedContactBox(cylinderBox, cylinderDelta, f)
	movedBox, okBox := TranslatedContactBox(box, boxDelta, f)
	if !okCylinder || !okBox {
		return nil, fmt.Errorf("%w: cylinder contact-track fraction cannot be represented", decaderr.ErrUnsupported)
	}
	cylinder := *t.Cylinder
	cylinder.Box = movedCylinder
	report := &contactReport{Request: t.Request}
	contactround.ClassifyCylinderBox(report, cylinder, movedBox, t.CylinderFirst)
	if report.Relation != vocab.ContactTouching || report.Manifold == nil || len(report.Manifold.Points) != 1 ||
		report.Manifold.Points[0].FeatureA != t.Sources[0] ||
		report.Manifold.Points[0].FeatureB != t.Sources[1] {
		return nil, fmt.Errorf("%w: cylinder contact track lost its disk point", decaderr.ErrUnsupported)
	}
	return (report.Manifold), nil
}

func OuterBoxGapExceeds(a, b contactbox.SourceBox, axis, sign int, minimum *big.Rat) bool {
	if sign > 0 {
		return new(big.Rat).Sub(b.Lo[axis].Rat(), a.Hi[axis].Rat()).Cmp(minimum) > 0
	}
	return new(big.Rat).Sub(a.Lo[axis].Rat(), b.Hi[axis].Rat()).Cmp(minimum) > 0
}

// planarManifoldAt publishes a planar track's manifold at an exact fraction
// (planarsweep.TrackProof.Points). The separation is the band's two-sided
// bound.
func planarManifoldAt(p *planarsweep.TrackProof, f *big.Rat, req vocab.ContactRequest) (*contactManifold, error) {
	verts, eta, ok := p.RoundedVertices(f)
	if !ok {
		return nil, fmt.Errorf("%w: planar contact track pose has no finite bound", decaderr.ErrUnsupported)
	}
	resolution, okResolution := sweeppath.ExactBaseValue(req.PointResolution)
	if !okResolution {
		return nil, fmt.Errorf("%w: planar contact track resolution is invalid", decaderr.ErrUnsupported)
	}
	readings, ok := p.Points(verts, eta, resolution)
	if !ok {
		return nil, fmt.Errorf("%w: planar contact track point exceeds point resolution", decaderr.ErrUnsupported)
	}
	separation := vocab.Measurement{Value: units.Millimeters(0), Bound: units.Millimeters(p.DepthUp),
		Exactness: measurement.ExactnessOf(p.DepthUp)}
	points := make([]contactPoint, 0, len(readings))
	for slot, reading := range readings {
		onM, onS := reading.OnM, reading.OnS
		point := contactPoint{Normal: p.Direction, NormalAngle: p.Angle,
			Separation: separation}
		featureM, featureS := (p.FeatureM[slot]), (p.FeatureS)
		if p.M == 0 {
			point.OnA, point.OnB = onM, onS
			point.FeatureA, point.FeatureB = featureM, featureS
		} else {
			point.OnA, point.OnB = onS, onM
			point.FeatureA, point.FeatureB = featureS, featureM
		}
		point.FaceA, point.FaceB = point.FeatureA.Face, point.FeatureB.Face
		points = append(points, point)
	}
	return &contactManifold{Points: points}, nil
}
func TranslatedContactBox(box contactbox.SourceBox, delta [3]proofarith.Dyadic,
	f *big.Rat) (contactbox.SourceBox, bool) {
	fraction, ok := proofarith.DyOfRat(f)
	if !ok {
		return contactbox.SourceBox{}, false
	}
	for i := range 3 {
		move := proofarith.DyMul(delta[i], fraction)
		box.Lo[i], box.Hi[i] = proofarith.DyAdd(box.Lo[i], move), proofarith.DyAdd(box.Hi[i], move)
	}
	return box, true
}

// BoxPoseDeviation bounds the L1 distance between a float-pose source box and
// the ideal affine box. Each support endpoint is compared as an exact rational.
func BoxPoseDeviation(start, observed contactbox.SourceBox, delta [3]proofarith.Dyadic, f *big.Rat) *big.Rat {
	return pairbox.PoseDeviation(start.AxisBox(), observed.AxisBox(), delta, f)
}

func TranslatedSourceBoxes(a, b contactbox.SourceBox, da, db [3]proofarith.Dyadic,
	f *big.Rat) (contactbox.SourceBox, contactbox.SourceBox, bool) {
	translatedA, translatedB, ok := pairbox.TranslatePair(a.AxisBox(), b.AxisBox(), da, db, f)
	if !ok {
		return contactbox.SourceBox{}, contactbox.SourceBox{}, false
	}
	a.Lo, a.Hi = translatedA.Lo, translatedA.Hi
	b.Lo, b.Hi = translatedB.Lo, translatedB.Hi
	return a, b, true
}

func BoundedFacetedReplayBoxes(floor contactbox.SourceBox, extent contactfaceted.BoundedExtent,
	pa, pb sweeppath.AffinePath, poseA, poseB r3.Transform, f *big.Rat,
	facetedFirst bool) (contactbox.SourceBox, contactfaceted.BoundedExtent, *big.Rat, bool) {
	facetedPath, floorPath := pb, pa
	facetedPose, floorPose := poseB, poseA
	if facetedFirst {
		facetedPath, floorPath, facetedPose, floorPose = pa, pb, poseA, poseB
	}
	observedFloor, okFloor := TranslatedReplayBox(floor, floorPath.From, floorPose)
	observedFacet, okFacet := TranslatedReplayBox(extent.Box, facetedPath.From, facetedPose)
	if !okFloor || !okFacet {
		return contactbox.SourceBox{}, contactfaceted.BoundedExtent{}, nil, false
	}
	deviation := BoxPoseDeviation(floor, observedFloor, floorPath.Delta, f)
	deviation.Add(deviation, BoxPoseDeviation(extent.Box, observedFacet, facetedPath.Delta, f))
	extent.Box = observedFacet
	return observedFloor, extent, deviation, true
}

var _ vocab.SweepTrack[*topo.Face, *topo.Edge, *topo.Vertex] = (*Track)(nil)
