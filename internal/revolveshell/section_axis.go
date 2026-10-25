package revolveshell

import (
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/coneaxis"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/massmoment"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/revolvemesh"
	"github.com/lestrrat-3d/decad/internal/revolveplan"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
)

// rootCandidatesAbout is RootCandidates for a record outside the frame
// predicate (agent-docs/revolve-axis-design.md §4.1, §4.2, §5.2). It reads
// each cone side from the exact plane-scaled span ends, never from the
// rounded surfacegeom.Cone tag, which only binds the face:
//
//   - the slope k* = (R1−R0)/(Z1−Z0) must be finite and positive, which
//     puts the apex scalar Z_a = Z0 − R0/k* strictly below both span ends;
//   - the apex point is P* = (Z_a / A·A)·A;
//   - the reported cone is {float(P*), A, float(k*)}, and ConeGap is the
//     §5.2 bound over the span's own axial reach from P*.
func rootCandidatesAbout(blank *topo.Body,
	section recordedSectionReading) []facetedrecord.RevolveSectionCertificate {
	axis, frame := section.axis, section.frame
	basis := section.payload.Basis()
	normSq := axis.NormSq().Rat()
	var candidates []facetedrecord.RevolveSectionCertificate
	for i, span := range section.spans {
		if span.Face == nil {
			continue
		}
		tag, ok := span.Face.Surface.(surfacegeom.Cone)
		if !ok {
			continue
		}
		start, end := span.Start, span.End
		z0, r0 := planeScaledRat(axis, frame, start)
		z1, r1 := planeScaledRat(axis, frame, end)
		if z0.Cmp(z1) > 0 {
			start, end = end, start
			z0, z1, r0, r1 = z1, z0, r1, r0
		}
		if z0.Cmp(z1) == 0 || r0.Sign() == 0 || r1.Sign() == 0 {
			continue
		}
		slope := new(big.Rat).Quo(new(big.Rat).Sub(r1, r0), new(big.Rat).Sub(z1, z0))
		if slope.Sign() <= 0 {
			continue
		}
		apexScalar := new(big.Rat).Sub(z0, new(big.Rat).Quo(r0, slope))
		cone, apexGap, ok := reportedCone(axis, normSq, apexScalar, slope)
		if !ok {
			continue
		}
		slopeGap := proofbound.RatFloatUp(new(big.Rat).Abs(
			new(big.Rat).Sub(slope, proofarith.FloatRat(cone.Slope))))
		reach, ok := axis.ScaledUp(new(big.Rat).Sub(z1, apexScalar))
		if !ok {
			continue
		}
		general := &facetedrecord.RevolveSectionAxis{Cone: cone, Slope: slope, Apex: apexScalar,
			RootZ: [2]*big.Rat{z0, z1}, RootR: [2]*big.Rat{r0, r1},
			ApexGap: apexGap, SlopeGap: slopeGap, W: basis.W, E0: basis.E0, E1: basis.E1}
		gap := general.ConeGap(reach)
		if proofbound.IsNonFinite(gap) || proofbound.IsNonFinite(slopeGap) {
			continue
		}
		candidates = append(candidates, facetedrecord.RevolveSectionCertificate{
			Blank: blank, Spans: section.spans, Frame: frame,
			AxisSpan: section.axisSpan, RootSpan: i, RootStart: start, RootEnd: end,
			RootCone: tag, ConeGap: gap, Axis: axis, General: general})
	}
	return candidates
}

// planeScaledRat returns the exact plane-scaled (Z, R) of a plane-local point.
func planeScaledRat(axis coneaxis.Axis, frame r3.Frame, p sectionrecord.Point2) (*big.Rat, *big.Rat) {
	z, r := axis.PlaneScaled(frame, p.U, p.V)
	return z.Rat(), r.Rat()
}

// reportedCone rounds the exact apex P* = (Z_a / A·A)·A and the slope k* to
// the nearest floats. It returns the cone and δ_a, an upper bound on
// |float(P*) − P*|, and reports false when a reading is not finite or the
// rounded slope is not positive.
func reportedCone(axis coneaxis.Axis, normSq, apexScalar, slope *big.Rat) (sketch.ConeSection, float64, bool) {
	scale := new(big.Rat).Quo(apexScalar, normSq)
	a := axis.Vec()
	var apex [3]float64
	distanceSq := new(big.Rat)
	for i, component := range [3]float64{a.X, a.Y, a.Z} {
		exact := new(big.Rat).Mul(scale, proofarith.FloatRat(component))
		apex[i], _ = exact.Float64()
		gap := new(big.Rat).Sub(proofarith.FloatRat(apex[i]), exact)
		distanceSq.Add(distanceSq, new(big.Rat).Mul(gap, gap))
	}
	s, _ := slope.Float64()
	point := r3.Vec{X: apex[0], Y: apex[1], Z: apex[2]}
	apexGap := proofbound.RatSqrtUp(distanceSq)
	if !proofbound.FiniteVec(point) || proofbound.IsNonFinite(s) || !(s > 0) ||
		proofbound.IsNonFinite(apexGap) {
		return sketch.ConeSection{}, 0, false
	}
	return sketch.ConeSection{Apex: point, Axis: a, Slope: s}, apexGap, true
}

// sectionStation is how the section ring reads one span's start point: on the
// axis, on the negative radial side, and an upper bound on its radius in
// millimetres.
type sectionStation struct {
	onAxis, negative bool
	radius           float64
}

// sectionStations reads every span's start point. A +X certificate reads the
// plane-local V as the radius. A general one reads the exact plane-scaled R:
// the point is on the axis exactly when R = 0, and its radius bound is
// R/|A|↓ (agent-docs/revolve-axis-design.md §3.4, §5.7). It reports false
// when a general radius has no finite bound.
func sectionStations(section facetedrecord.RevolveSectionCertificate) ([]sectionStation, bool) {
	stations := make([]sectionStation, len(section.Spans))
	for i, span := range section.Spans {
		p := span.Start
		if section.General == nil {
			stations[i] = sectionStation{onAxis: p.V == 0, negative: p.V < 0, radius: p.V}
			continue
		}
		_, r := planeScaledRat(section.Axis, section.Frame, p)
		radius, ok := section.Axis.ScaledUp(new(big.Rat).Abs(r))
		if !ok {
			return nil, false
		}
		stations[i] = sectionStation{onAxis: r.Sign() == 0, negative: r.Sign() < 0, radius: radius}
	}
	return stations, true
}

// ringLiftAbout places a general certificate's ring vertices with the blank
// Revolve's own construction (agent-docs/revolve-axis-design.md §5.7): a
// vertex is RevolveLift.Point at the plane point's axis coordinates
// (revolveaxis.Frame.ToAxis) and the station angle, and an axis vertex is the
// same point at radius and angle zero. Its gap is RevolveLift.SweptPointGap
// against the recorded plane point rotated about the record's axis within
// the axis's own bound.
type ringLiftAbout struct {
	payload revolveplan.Payload
	lift    revolvemesh.RevolveLift
	basis   revolvemesh.RevolveBasis
	bound   revolvemesh.AxisBound
}

func newRingLiftAbout(section facetedrecord.RevolveSectionCertificate) (ringLiftAbout, bool) {
	if section.Blank == nil {
		return ringLiftAbout{}, false
	}
	rp, ok := section.Blank.Payload.(revolveplan.Payload)
	if !ok || rp.Frame != section.Frame || rp.Xform != r3.Identity() {
		return ringLiftAbout{}, false
	}
	lift := rp.Lift()
	return ringLiftAbout{payload: rp, lift: lift, basis: lift.Basis(), bound: rp.AxisBound()}, true
}

// vertex returns the held ring vertex of plane point p at the angle whose
// held value is angle and whose exact sine and cosine sin and cos enclose,
// and the proven distance from it to the point it denotes. An axis vertex
// takes radius and angle zero.
func (l ringLiftAbout) vertex(p sectionrecord.Point2, onAxis bool, angle float64,
	sin, cos proofbound.RatInterval) (r3.Vec, float64) {
	z, rho := l.payload.Ax.ToAxis(p.U, p.V)
	if onAxis {
		rho, angle = 0, 0
		zero, one := new(big.Rat), big.NewRat(1, 1)
		sin, cos = proofbound.Interval(zero, zero), proofbound.Interval(one, one)
	}
	held := l.lift.Point(l.basis, r3.Identity(), z, rho, angle)
	gap := l.lift.SweptPointGap(l.bound, r3.Identity(), p.U, p.V, proofbound.WalkEndBound{}, sin, cos, held)
	return held, gap
}

// GeneralPlanInput sets the angular inputs of a general certificate's sector
// plan (agent-docs/revolve-axis-design.md §3.7, §5.6): MinRadius is
// min(R0, R1)/|A|↑, a lower bound on the root ends' radii in millimetres,
// the plan charges τ from FrameTilt, the δ_B of the blank's held frame
// (W, E0, E1) about the axis, and measures the placed root nodes' angles in
// that frame. It refuses a frame that massmoment.FrameTilt
// declines. section must come from the general record branch: its General
// must not be nil.
func GeneralPlanInput(input *SectorJoinInput, section facetedrecord.RevolveSectionCertificate) error {
	general := section.General
	rootR := general.RootR[0]
	if general.RootR[1].Cmp(rootR) < 0 {
		rootR = general.RootR[1]
	}
	minRadius, ok := section.Axis.ScaledDown(rootR)
	if !ok {
		return fmt.Errorf("%w: root sector has no radius bound about the axis", decaderr.ErrUnsupported)
	}
	tilt, ok := massmoment.FrameTilt(general.W, general.E0, general.E1, section.Axis)
	if !ok {
		return fmt.Errorf("%w: the blank's angular frame departs from the axis", decaderr.ErrUnsupported)
	}
	input.MinRadius, input.Tilted, input.FrameTilt = minRadius, true, tilt
	input.Angles = section.AngleBasis()
	return nil
}

// sectionToolGapAbout is RevolveSectionToolGap for a general certificate. It
// reads each trimming cone's meridian Z + k·R = c and the root meridian
// R = k*·(Z − Z_a) in A-scaled coordinates, and converts each residual and
// crossing distance to millimetres: a residual divides by |A|↓, and a squared
// distance by A·A exactly (agent-docs/revolve-axis-design.md §3.4).
func sectionToolGapAbout(section facetedrecord.RevolveSectionCertificate,
	record *facetedrecord.PointConeTrimRecord) (float64, bool) {
	general, axis := section.General, section.Axis
	if record.Lower.Axis != axis || record.Upper.Axis != axis {
		return 0, false
	}
	root := section.Spans[section.RootSpan]
	startZ, _ := planeScaledRat(axis, section.Frame, root.Start)
	endZ, _ := planeScaledRat(axis, section.Frame, root.End)
	var neighbors [2]sectionrecord.Point2
	if startZ.Cmp(endZ) < 0 {
		neighbors[0] = section.Spans[(section.RootSpan+len(section.Spans)-1)%len(section.Spans)].Start
		neighbors[1] = section.Spans[(section.RootSpan+1)%len(section.Spans)].End
	} else {
		neighbors[0] = section.Spans[(section.RootSpan+1)%len(section.Spans)].End
		neighbors[1] = section.Spans[(section.RootSpan+len(section.Spans)-1)%len(section.Spans)].Start
	}
	m := general.Slope
	b := new(big.Rat).Neg(new(big.Rat).Mul(m, general.Apex))
	normSq := axis.NormSq().Rat()
	maxGap := 0.0
	for i, item := range []struct {
		limit *facetedrecord.PointConeLimit
		root  sectionrecord.Point2
	}{{record.Lower, section.RootStart}, {record.Upper, section.RootEnd}} {
		outer := neighbors[i]
		outerZ, outerR := planeScaledRat(axis, section.Frame, outer)
		if outerR.Sign() <= 0 || outer == item.root {
			return 0, false
		}
		residual := func(z, r *big.Rat) (float64, bool) {
			value := new(big.Rat).Sub(new(big.Rat).Add(z, new(big.Rat).Mul(item.limit.Slope, r)),
				item.limit.Apex)
			return axis.ScaledUp(value.Abs(value))
		}
		rootGap, rootOK := residual(general.RootZ[i], general.RootR[i])
		outerGap, outerOK := residual(outerZ, outerR)
		if !rootOK || !outerOK {
			return 0, false
		}
		maxGap = max(maxGap, rootGap, outerGap)
		den := new(big.Rat).Add(big.NewRat(1, 1), new(big.Rat).Mul(item.limit.Slope, m))
		if den.Sign() <= 0 {
			return 0, false
		}
		crossZ := new(big.Rat).Quo(new(big.Rat).Sub(item.limit.Apex,
			new(big.Rat).Mul(item.limit.Slope, b)), den)
		zGap := new(big.Rat).Sub(crossZ, general.RootZ[i])
		rGap := new(big.Rat).Mul(m, zGap)
		distance2 := new(big.Rat).Add(new(big.Rat).Mul(zGap, zGap), new(big.Rat).Mul(rGap, rGap))
		maxGap = max(maxGap, proofbound.RatSqrtUp(distance2.Quo(distance2, normSq)))
	}
	return maxGap, !proofbound.IsNonFinite(maxGap)
}
