package surfacegeom

// DiagnosticPlaneKey is a canonicalized (normal, signed offset) key for one
// planar face. Verify uses it only to choose unsupported-contact message text;
// it is not a contact admission or general plane-equivalence proof.
type DiagnosticPlaneKey struct {
	nx, ny, nz, off float64
}

// CanonicalDiagnosticPlaneKey builds p's DiagnosticPlaneKey: the normal's sign
// is fixed by its first nonzero component being positive (flipping the offset
// along with it), so a pair of opposite-facing coplanar faces produces the
// same key, and every component is passed through zeroSign so -0.0 and 0.0
// never key differently.
func CanonicalDiagnosticPlaneKey(p Plane) DiagnosticPlaneKey {
	n := p.Frame.N()
	off := n.Dot(p.Frame.Origin())
	flip := n.X < 0 || (n.X == 0 && (n.Y < 0 || (n.Y == 0 && n.Z < 0)))
	if flip {
		n = n.Scale(-1)
		off = -off
	}
	return DiagnosticPlaneKey{zeroSign(n.X), zeroSign(n.Y), zeroSign(n.Z), zeroSign(off)}
}

// zeroSign normalizes a negative zero to a positive zero so it never keys a
// map entry differently from 0.0.
func zeroSign(f float64) float64 {
	if f == 0 {
		return 0
	}
	return f
}
