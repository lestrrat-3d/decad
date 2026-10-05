package decad

import (
	"math"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// sourceBoxContactProof is the complete exact occupied interval of a source
// rectangular prism under a read pose. The sweep uses the full intervals for
// one-sided departure and persistent-contact proofs.
type sourceBoxContactProof struct {
	lo, hi [3]dyadic
	// faces[axis][0] is the original minimum-support face; [1] is maximum.
	faces [3][2]*Face
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
	if !finiteVec(pp.frame.Origin()) || !finiteMeasurementValues(pp.z0, pp.z1) {
		return sourceBoxContactProof{}, false
	}
	var umin, umax, vmin, vmax float64
	for i, seg := range pp.profile.Outer.Segments {
		line, ok := seg.(LineSeg)
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
	u := [2]dyadic{mustDyOf(umin), mustDyOf(umax)}
	v := [2]dyadic{mustDyOf(vmin), mustDyOf(vmax)}
	z := [2]dyadic{mustDyOf(pp.z0), mustDyOf(pp.z1)}
	origin := dyVec(pp.frame.Origin())
	fu, fv, fn := dyVec(pp.frame.U()), dyVec(pp.frame.V()), dyVec(pp.frame.N())
	var box sourceBoxContactProof
	first := true
	for iu := range u {
		for iv := range v {
			for iz := range z {
				p := dvAdd(origin, dvAdd(dyScaleVec(fu, u[iu]),
					dvAdd(dyScaleVec(fv, v[iv]), dyScaleVec(fn, z[iz]))))
				p = exactContactTransform(pp.xform, p)
				p = exactContactTransform(pose, p)
				for axis := range 3 {
					if first || dyCmp(p[axis], box.lo[axis]) < 0 {
						box.lo[axis] = p[axis]
					}
					if first || dyCmp(p[axis], box.hi[axis]) > 0 {
						box.hi[axis] = p[axis]
					}
				}
				first = false
			}
		}
	}
	for i := range 3 {
		if dyCmp(box.lo[i], box.hi[i]) >= 0 {
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
		axis, side, ok := signedAxis(normal)
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

func dyScaleVec(v dyV3, s dyadic) dyV3 {
	return dyV3{dyMul(v[0], s), dyMul(v[1], s), dyMul(v[2], s)}
}

func exactContactTransform(t r3.Transform, p dyV3) dyV3 {
	b := t.Basis()
	return dvAdd(dyVec(t.Translation()), dvAdd(dyScaleVec(dyVec(b.EX), p[0]),
		dvAdd(dyScaleVec(dyVec(b.EY), p[1]), dyScaleVec(dyVec(b.EZ), p[2]))))
}

func signedAxisTransform(t r3.Transform) bool {
	if !t.IsValid() || !finiteVec(t.Translation()) {
		return false
	}
	b := t.Basis()
	return cardinalBasis(b.EX, b.EY, b.EZ)
}

func signedAxis(v r3.Vec) (int, int, bool) {
	switch v {
	case r3.Vec{X: -1}:
		return 0, 0, true
	case r3.Vec{X: 1}:
		return 0, 1, true
	case r3.Vec{Y: -1}:
		return 1, 0, true
	case r3.Vec{Y: 1}:
		return 1, 1, true
	case r3.Vec{Z: -1}:
		return 2, 0, true
	case r3.Vec{Z: 1}:
		return 2, 1, true
	default:
		return 0, 0, false
	}
}

func publishSourceBoxPatch(report *ContactReport, a, b sourceBoxContactProof, axis, sign int,
	separation dyadic) {
	var sideA, sideB int
	if sign > 0 {
		sideA, sideB = 1, 0
	} else {
		sideA, sideB = 0, 1
	}
	faceA, faceB := a.faces[axis][sideA], b.faces[axis][sideB]
	var projected [2]int
	n := 0
	for i := range 3 {
		if i != axis {
			projected[n] = i
			n++
		}
	}
	var low, high [2]dyadic
	for i, ax := range projected {
		low[i], high[i] = dyMax(a.lo[ax], b.lo[ax]), dyMin(a.hi[ax], b.hi[ax])
		if dyCmp(low[i], high[i]) >= 0 {
			report.Reason = ContactAmbiguousFeature
			return
		}
	}
	sep, ok := sourceBoxSignedReading(separation)
	if !ok {
		report.Reason = ContactPointTooCoarse
		return
	}
	var normal r3.Vec
	switch axis {
	case 0:
		normal.X = float64(sign)
	case 1:
		normal.Y = float64(sign)
	case 2:
		normal.Z = float64(sign)
	}
	points := make([]ContactPoint, 0, 4)
	for _, corner := range [][2]int{{0, 0}, {1, 0}, {1, 1}, {0, 1}} {
		var pA, pB dyV3
		for i, ax := range projected {
			coord := low[i]
			if corner[i] == 1 {
				coord = high[i]
			}
			pA[ax], pB[ax] = coord, coord
		}
		if sign > 0 {
			pA[axis], pB[axis] = a.hi[axis], b.lo[axis]
		} else {
			pA[axis], pB[axis] = a.lo[axis], b.hi[axis]
		}
		ma, oka := sourceBoxPoint(pA)
		mb, okb := sourceBoxPoint(pB)
		if !oka || !okb || ma.Bound.Base() > report.Request.PointResolution.Base() ||
			mb.Bound.Base() > report.Request.PointResolution.Base() {
			report.Reason = ContactPointTooCoarse
			return
		}
		points = append(points, ContactPoint{
			OnA: ma, OnB: mb,
			Normal:      VecMeasurement{Value: normal, Exactness: Exact, Bound: units.Scalar(0)},
			NormalAngle: units.Radians(0), Separation: sep,
			FaceA: faceA, FaceB: faceB,
			FeatureA: ContactFeature{Face: faceA}, FeatureB: ContactFeature{Face: faceB},
		})
	}
	report.Manifold = &ContactManifold{Points: points}
}

func sourceBoxPoint(p dyV3) (VecMeasurement, bool) {
	return sourceBoxPointAt(&p)
}

// sourceBoxPointAt reads an exact point without copying its pointer-bearing
// dyadic components across the call boundary.
func sourceBoxPointAt(p *dyV3) (VecMeasurement, bool) {
	var coords [3]float64
	bound := 0.0
	for i := range 3 {
		coords[i], _ = p[i].Float64()
		if !finiteMeasurementValues(coords[i]) {
			return VecMeasurement{}, false
		}
		bound = math.Max(bound, dyadicFloatError(p[i], coords[i]))
	}
	bound = radius3D(bound)
	if !finiteMeasurementValues(bound) {
		return VecMeasurement{}, false
	}
	return VecMeasurement{Value: r3.Vec{X: coords[0], Y: coords[1], Z: coords[2]},
		Exactness: exactnessOf(bound), Bound: units.Millimeters(bound)}, true
}

func sourceBoxSignedReading(v dyadic) (Measurement, bool) {
	held, _ := v.Float64()
	if !finiteMeasurementValues(held) {
		return Measurement{}, false
	}
	bound := dyadicFloatError(v, held)
	return Measurement{Value: units.Millimeters(held), Exactness: exactnessOf(bound),
		Bound: units.Millimeters(bound)}, finiteMeasurementValues(bound)
}

func dyMax(a, b dyadic) dyadic {
	if dyCmp(a, b) >= 0 {
		return a
	}
	return b
}

func dyMin(a, b dyadic) dyadic {
	if dyCmp(a, b) <= 0 {
		return a
	}
	return b
}
