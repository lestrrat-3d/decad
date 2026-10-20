package decad

import (
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/stackedshell"
	"github.com/lestrrat-3d/units"
)

// This file is route S of docs/modify-general-design.md ("modify-general §N"
// below): the shell of a brep or stacked receiver that reads as a prism A
// along a reference axis k cut by through tools T₁…Tₙ along other axes
// (Table TC, §3.1), with one or both of A's caps, one run of A's walls, or
// both removed (§3.2), or with one circular tool wall removed (§3.1d).
// For cap and outer-wall removals, the shell is the erosion
// (A \ ⋃Tᵢ) ⊖ t = (A ⊖ t) \ ⋃(Tᵢ ⊕ t): A ⊖ t is the eroded section — S ⊖ t,
// or with a removed wall run the side opening's cavity section C
// (shell_opening.go) — swept over the cup's interval, each Tᵢ ⊕ t the tool's
// dilated section swept t past both walls it pierces, and the difference is
// class B's own Cut (classbgeom.OfPayloads), run privately. The result record is
// the receiver's kept faces, the cavity's faces reversed, and the rim at each
// removed face (§3.3, internal/throughshell/). The circular tool-wall route
// builds exact stacked sections (brep_shell.go). Every test below
// either compares recorded floats exactly or refuses; none admits on a residual.

// brepShellCall is what Shell hands route S beside the brep request: the
// faces to remove, the wall sense, and the thickness as given, in
// millimetres, with its conversion bound (extent.MagnitudeInBounded).
type brepShellCall struct {
	removed     []*Face
	sense       ShellSense
	t           units.Value
	tmm, tDelta float64
}

// stackedShellCall is call as the plain data internal/stackedshell reads. It
// stays in root because it matches the live removed face against source's
// cap-end and cap-start faces (facesByRole).
func stackedShellCall(source *Body, call brepShellCall) stackedshell.Call {
	sc := stackedshell.Call{Inward: call.sense == Inward, Removed: len(call.removed), T: call.tmm, TDelta: call.tDelta}
	if len(call.removed) == 1 {
		roles := facesByRole(source)
		sc.CapEnd = call.removed[0] == roles[provenance.RoleCapEnd]
		sc.CapStart = call.removed[0] == roles[provenance.RoleCapStart]
	}
	return sc
}
