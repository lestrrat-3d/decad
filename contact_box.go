package decad

import (
	"math"

	"github.com/lestrrat-3d/decad/internal/clearance"
	"github.com/lestrrat-3d/decad/internal/pair/box"

	"github.com/lestrrat-3d/decad/internal/proofbound"

	"github.com/lestrrat-3d/decad/internal/pair"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// sourceBoxContactProof is the complete exact occupied interval of a source
// rectangular prism under a read pose. The sweep uses the full intervals for
// one-sided departure and persistent-contact proofs.
type sourceBoxContactProof struct {
	lo, hi [3]proofarith.Dyadic
	// faces[axis][0] is the original minimum-support face; [1] is maximum.
	faces [3][2]*Face
}

func (b sourceBoxContactProof) axisBox() box.AxisBox {
	return box.AxisBox{Lo: b.lo, Hi: b.hi}
}

// sourceBoxAtPose never infers occupancy from a held bounding box. It maps all
// eight corners of a source-certified rectangle through its recorded frame,
// accumulated placement, and query pose with exact dyadic operations.
func sourceBoxAtPose(b *Body, pose r3.Transform) (sourceBoxContactProof, bool) {
	pp, ok := b.payload.(prismPayload)
	if !ok || !b.solid || b.kind != BodySolid || pp.surfaceResult ||
		pp.sectionDelta != 0 || pp.z0Delta != 0 || pp.z1Delta != 0 ||
		!rectangularProfile(pp.profile) ||
		!cardinalBasis(pp.frame.U(), pp.frame.V(), pp.frame.N()) ||
		!signedAxisTransform(pp.xform) || !signedAxisTransform(pose) {
		return sourceBoxContactProof{}, false
	}
	if !proofbound.FiniteVec(pp.frame.Origin()) || !finiteMeasurementValues(pp.z0, pp.z1) {
		return sourceBoxContactProof{}, false
	}
	var umin, umax, vmin, vmax float64
	for i, seg := range pp.profile.Outer.Segments {
		line, ok := seg.(lineSeg)
		if !ok {
			return sourceBoxContactProof{}, false
		}
		p := line.Start
		if i == 0 {
			umin, umax, vmin, vmax = p.U, p.U, p.V, p.V
		}
		umin, umax = math.Min(umin, p.U), math.Max(umax, p.U)
		vmin, vmax = math.Min(vmin, p.V), math.Max(vmax, p.V)
	}
	if !finiteMeasurementValues(umin, umax, vmin, vmax) || pp.z0 >= pp.z1 {
		return sourceBoxContactProof{}, false
	}
	u := [2]proofarith.Dyadic{proofarith.MustDyOf(umin), proofarith.MustDyOf(umax)}
	v := [2]proofarith.Dyadic{proofarith.MustDyOf(vmin), proofarith.MustDyOf(vmax)}
	z := [2]proofarith.Dyadic{proofarith.MustDyOf(pp.z0), proofarith.MustDyOf(pp.z1)}
	origin := proofarith.DyVec(pp.frame.Origin())
	fu, fv, fn := proofarith.DyVec(pp.frame.U()), proofarith.DyVec(pp.frame.V()), proofarith.DyVec(pp.frame.N())
	var box sourceBoxContactProof
	first := true
	for iu := range u {
		for iv := range v {
			for iz := range z {
				p := proofarith.DvAdd(origin, proofarith.DvAdd(dyScaleVec(fu, u[iu]),
					proofarith.DvAdd(dyScaleVec(fv, v[iv]), dyScaleVec(fn, z[iz]))))
				p = exactContactTransform(pp.xform, p)
				p = exactContactTransform(pose, p)
				for axis := range 3 {
					if first || proofarith.DyCmp(p[axis], box.lo[axis]) < 0 {
						box.lo[axis] = p[axis]
					}
					if first || proofarith.DyCmp(p[axis], box.hi[axis]) > 0 {
						box.hi[axis] = p[axis]
					}
				}
				first = false
			}
		}
	}
	for i := range 3 {
		if proofarith.DyCmp(box.lo[i], box.hi[i]) >= 0 {
			return sourceBoxContactProof{}, false
		}
	}
	faces := b.Faces()
	if len(faces) != 6 {
		return sourceBoxContactProof{}, false
	}
	for _, face := range faces {
		plane, ok := face.surface.(Plane)
		if !ok || face.normalBound != 0 {
			return sourceBoxContactProof{}, false
		}
		normal := plane.Frame.N()
		if face.reversed {
			normal = normal.Scale(-1)
		}
		normal = pose.ApplyDir(normal)
		axis, side, ok := clearance.SignedAxis(normal)
		if !ok || box.faces[axis][side] != nil {
			return sourceBoxContactProof{}, false
		}
		box.faces[axis][side] = face
	}
	for axis := range 3 {
		if box.faces[axis][0] == nil || box.faces[axis][1] == nil {
			return sourceBoxContactProof{}, false
		}
	}
	return box, true
}

func dyScaleVec(v proofarith.DyV3, s proofarith.Dyadic) proofarith.DyV3 {
	return proofarith.DyV3{proofarith.DyMul(v[0], s), proofarith.DyMul(v[1], s), proofarith.DyMul(v[2], s)}
}

func exactContactTransform(t r3.Transform, p proofarith.DyV3) proofarith.DyV3 {
	b := t.Basis()
	return proofarith.DvAdd(proofarith.DyVec(t.Translation()), proofarith.DvAdd(dyScaleVec(proofarith.DyVec(b.EX), p[0]),
		proofarith.DvAdd(dyScaleVec(proofarith.DyVec(b.EY), p[1]), dyScaleVec(proofarith.DyVec(b.EZ), p[2]))))
}

// exactContactMap is exactContactTransform with the transform's translation
// and basis lifted once, for a caller mapping many points through one pose.
// apply returns exactly what exactContactTransform returns.
type exactContactMap struct {
	translation, ex, ey, ez proofarith.DyV3
}

func newExactContactMap(t r3.Transform) exactContactMap {
	b := t.Basis()
	return exactContactMap{translation: proofarith.DyVec(t.Translation()),
		ex: proofarith.DyVec(b.EX), ey: proofarith.DyVec(b.EY), ez: proofarith.DyVec(b.EZ)}
}

func (m exactContactMap) apply(p proofarith.DyV3) proofarith.DyV3 {
	return proofarith.DvAdd(m.translation, proofarith.DvAdd(dyScaleVec(m.ex, p[0]),
		proofarith.DvAdd(dyScaleVec(m.ey, p[1]), dyScaleVec(m.ez, p[2]))))
}

func signedAxisTransform(t r3.Transform) bool {
	if !t.IsValid() || !proofbound.FiniteVec(t.Translation()) {
		return false
	}
	b := t.Basis()
	return cardinalBasis(b.EX, b.EY, b.EZ)
}

// The root package maps neutral pair readings to public units and topology.
func publishSourceBoxPatch(report *ContactReport, a, b sourceBoxContactProof, axis, sign int,
	separation proofarith.Dyadic) {
	patch, reason := box.FacePatch(
		box.AxisBox{Lo: a.lo, Hi: a.hi}, box.AxisBox{Lo: b.lo, Hi: b.hi},
		axis, sign, separation,
		box.AxisBoxRequest{PointResolutionMM: report.Request.PointResolution.Base()},
	)
	report.Reason = sourceBoxReason(reason)
	if patch != nil {
		publishAxisBoxPatch(report, a, b, patch)
	}
}

func sourceBoxScalar(reading pair.ScalarReading) Measurement {
	return Measurement{Value: units.Millimeters(reading.ValueMM),
		Bound: units.Millimeters(reading.BoundMM), Exactness: exactnessOf(reading.BoundMM)}
}

func sourceBoxPointMeasurement(reading box.PointReading) VecMeasurement {
	return VecMeasurement{Value: reading.Value, Bound: units.Millimeters(reading.BoundMM),
		Exactness: exactnessOf(reading.BoundMM)}
}

func sourceBoxGap(gaps [3]proofarith.Dyadic) (Measurement, bool) {
	reading, ok := box.AxisGap(gaps)
	if !ok {
		return Measurement{}, false
	}
	return sourceBoxScalar(reading), true
}

func sourceBoxPoint(point proofarith.DyV3) (VecMeasurement, bool) {
	return sourceBoxPointAt(&point)
}

func sourceBoxPointAt(point *proofarith.DyV3) (VecMeasurement, bool) {
	reading, ok := box.ReadPointAt(point)
	if !ok {
		return VecMeasurement{}, false
	}
	return sourceBoxPointMeasurement(reading), true
}

func sourceBoxSignedReading(value proofarith.Dyadic) (Measurement, bool) {
	reading, ok := box.SignedReading(value)
	if !ok {
		return Measurement{}, false
	}
	return sourceBoxScalar(reading), true
}

func dyMax(a, b proofarith.Dyadic) proofarith.Dyadic {
	if proofarith.DyCmp(a, b) >= 0 {
		return a
	}
	return b
}

func dyMin(a, b proofarith.Dyadic) proofarith.Dyadic {
	if proofarith.DyCmp(a, b) <= 0 {
		return a
	}
	return b
}
