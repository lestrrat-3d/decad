package decad

import "fmt"

// requireRightAngle refuses an edge that is not shared by exactly two planar
// faces whose normals are exactly perpendicular. what names the operation in
// the error. Both ChamferAngle and FilletChord convert a measure that only
// reads as a setback or radius on such an edge, and neither admits a dihedral
// that is merely close to a right angle (CLAUDE.md "Reject-only"). It is a
// method because it reads the edge's own topology.
func (e *Edge) requireRightAngle(what string) error {
	faces := e.Faces()
	if len(faces) != 2 {
		return fmt.Errorf(`%w: %s needs an edge between two faces, got %d`, ErrUnsupported, what, len(faces))
	}
	var normals [2]Plane
	for i, f := range faces {
		p, ok := f.Surface().(Plane)
		if !ok {
			return fmt.Errorf(`%w: %s needs planar faces beside the edge`, ErrUnsupported, what)
		}
		normals[i] = p
	}
	if normals[0].Frame.N().Dot(normals[1].Frame.N()) != 0 {
		return fmt.Errorf(`%w: %s needs faces that meet at exactly a right angle`, ErrUnsupported, what)
	}
	return nil
}
