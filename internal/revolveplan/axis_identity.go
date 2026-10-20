package revolveplan

import "github.com/lestrrat-3d/decad/internal/revolveaxis"

// AxisIdentical is S4's revolve comparison: the resolved axis's own
// anchor, direction and each of those four fields' proven bound, under Go ==
// on the stored floats and never a tolerance
// (agent-docs/surface-intersection-design.md §2.1 S4).
//
// It compares EIGHT fields and not the whole revolveaxis.Frame. The remaining
// four — snapTol, radialAdmitAllow, axialExtentUpper and snap — are each
// operand's own admission allowances, derived from ITS OWN section rather
// than from the generator the two share, so two genuinely co-axial operands
// differ in them by construction and comparing them would refuse every
// admissible pair.
func AxisIdentical(a, b revolveaxis.Frame) bool {
	return a.AU == b.AU && a.AV == b.AV &&
		a.AUBound == b.AUBound && a.AVBound == b.AVBound &&
		a.DU == b.DU && a.DV == b.DV &&
		a.DUBound == b.DUBound && a.DVBound == b.DVBound
}

// CommonAxisPair checks whether two revolve records can use the analytic
// common-axis Boolean path.
func CommonAxisPair(a, b Payload) bool {
	if !a.Full || !b.Full || a.SurfaceResult || b.SurfaceResult ||
		a.SectionDelta != 0 || b.SectionDelta != 0 || a.SectionWhole || b.SectionWhole ||
		a.Frame != b.Frame || a.Xform != b.Xform || a.Phi0 != b.Phi0 || a.Phi1 != b.Phi1 {
		return false
	}
	return a.Ax.AU == b.Ax.AU && a.Ax.AV == b.Ax.AV &&
		a.Ax.DU == b.Ax.DU && a.Ax.DV == b.Ax.DV &&
		a.Ax.AUBound == b.Ax.AUBound && a.Ax.AVBound == b.Ax.AVBound &&
		a.Ax.DUBound == b.Ax.DUBound && a.Ax.DVBound == b.Ax.DVBound
}
