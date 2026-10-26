package decad

import (
	"fmt"
	"math"

	"github.com/lestrrat-3d/r3"
)

// dihedralTolerance is the relative difference, in the setback or radius two
// edges' dihedral angles give, within which ChamferAngle and FilletChord treat
// them as one value. Equal dihedrals read from different face pairs differ only
// in rounding, far below this, and the one value is far below the bound of the
// result a chamfer or fillet publishes.
const dihedralTolerance = 1e-12

// planarDihedral returns the angle, in radians, between the two planar faces of
// the edge e on the side a bevel or a round cuts across: strictly between 0 and
// π. On a convex edge that is the interior angle of the material. On a concave
// edge it is the angle of the open wedge the faces make in the air, which a
// chamfer or fillet fills; it is π less the turn between the faces' normals
// either way. what names the operation in the error. The angle is read from the
// unit outward normals of the two planes by the angle between them,
// atan2(|n1 × n2|, n1 · n2), which is as accurate near a flat edge or a knife
// edge as at a right angle. It is a method because it reads the edge's own
// topology.
//
// The faces are planes, or on a circular edge (a Circle3 or an Arc3) a plane, cylinder,
// cone or sphere: a circle on such a surface of revolution about its own axis
// meets the other face at one angle all round, so one angle states the edge. A
// curved face beside a straight or free-form edge, or a torus, meets the other
// at an angle that varies along it. An edge that is not shared by exactly two
// such faces is ErrUnsupported: a chamfer or fillet of it is not a bevel or a
// round of a corner at one angle.
func (e *Edge) planarDihedral(what string) (float64, error) {
	faces := e.Faces()
	if len(faces) != 2 {
		return 0, fmt.Errorf(`%w: %s needs an edge between two faces, got %d`, ErrUnsupported, what, len(faces))
	}
	at := e.Start().Position().Value
	var circular bool
	switch e.Curve().(type) {
	case Circle3, Arc3:
		circular = true
	}
	var normals [2]r3.Vec
	for i, f := range faces {
		switch f.Surface().(type) {
		case Plane:
		case Cylinder, Cone, Sphere:
			if !circular {
				return 0, fmt.Errorf(`%w: %s needs planar faces beside a straight or free-form edge, as a curved face meets the other at a changing angle`,
					ErrUnsupported, what)
			}
		default:
			return 0, fmt.Errorf(`%w: %s needs planar faces, or a plane, cylinder, cone or sphere beside a circular edge`, ErrUnsupported, what)
		}
		n, err := f.NormalAt(at)
		if err != nil {
			return 0, err
		}
		normals[i] = n.Value
	}
	turn := math.Atan2(normals[0].Cross(normals[1]).Len(), normals[0].Dot(normals[1]))
	phi := math.Pi - turn
	if !(phi > 0 && phi < math.Pi) {
		return 0, fmt.Errorf(`%w: %s needs faces that meet at an angle between 0 and π`, ErrUnsupported, what)
	}
	return phi, nil
}

// sameWithinDihedralTolerance reports whether a and b, two setbacks or radii in
// millimetres, are equal within dihedralTolerance of the larger.
func sameWithinDihedralTolerance(a, b float64) bool {
	return math.Abs(a-b) <= dihedralTolerance*math.Max(math.Abs(a), math.Abs(b))
}
