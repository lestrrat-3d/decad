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

// convexDihedral returns the interior angle, in radians, between the two planar
// faces of the convex edge e: strictly between 0 and π. what names the operation
// in the error. The angle is read from the unit outward normals of the two
// planes by the angle between them, atan2(|n1 × n2|, n1 · n2), which is as
// accurate near a flat edge or a knife edge as at a right angle; the interior
// angle is π less that. It is a method because it reads the edge's own topology.
//
// An edge that is not shared by exactly two planar faces, or is not convex, is
// ErrUnsupported: a chamfer or fillet of it is not a bevel or a round of a
// corner between two planes.
func (e *Edge) convexDihedral(what string) (float64, error) {
	faces := e.Faces()
	if len(faces) != 2 {
		return 0, fmt.Errorf(`%w: %s needs an edge between two faces, got %d`, ErrUnsupported, what, len(faces))
	}
	if !e.IsConvex() {
		return 0, fmt.Errorf(`%w: %s needs a convex edge`, ErrUnsupported, what)
	}
	at := e.Start().Position().Value
	var normals [2]r3.Vec
	for i, f := range faces {
		if _, ok := f.Surface().(Plane); !ok {
			return 0, fmt.Errorf(`%w: %s needs planar faces beside the edge`, ErrUnsupported, what)
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
