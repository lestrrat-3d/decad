package apitest_test

import (
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// The tests in this file stop a tool at a level a shell derives: a cup's
// floor, kept cap or rim, and a closed shell's cavity floor or ceiling
// (agent-docs/stacked-prism-design.md §2.1). A derived level denotes its
// source end's rational plus or minus the exact thickness, so a tool that
// stops there spans a plate whose cap denotes the same rational. 0.2 in
// denotes a rational r about 7.3e-17 below its held float h = 5.08, so a
// 5.08 mm thickness or plate stands for another plane at the same float.

// roleShellCap is a cup's pocket floor role.
const roleShellCap = "shellCap"

// roleQuery selects the one face of b that carries role.
func roleQuery(t *testing.T, b *decad.Body, role string) *decad.FaceQuery {
	t.Helper()
	for _, f := range b.Faces() {
		for _, o := range f.Origins() {
			if o.Role == role {
				return decad.Faces(decad.FaceCreatedBy(o)).Exactly(1)
			}
		}
	}
	require.FailNow(t, "the body has no face with role "+role)
	return nil
}

// shellStop is the 4x4 stopTool over 8..12 stopping at the face of shell
// that carries role, offset by offset, and running 20 mm the other way. up
// says the stop is the upward side.
func shellStop(t *testing.T, doc *decad.Document, shell *decad.Body, role string, offset units.Value, up bool) *decad.Body {
	t.Helper()
	stop := decad.ToFace{Body: shell, Face: roleQuery(t, shell, role), Offset: offset}
	other := decad.DistanceSide{D: units.Millimeters(20)}
	if up {
		return stopTool(t, doc, decad.TwoSided{One: stop, Two: other})
	}
	return stopTool(t, doc, decad.TwoSided{One: other, Two: stop})
}

// shellOf shells the 20x20 block over 30..50 x 0..20 that rises by up and
// sinks by down. A nil sel keeps every face.
func shellOf(t *testing.T, doc *decad.Document, up, down units.Value, sel func(*decad.Body) decad.FaceSelector,
	th units.Value, opts ...decad.ShellOption) *decad.Body {
	t.Helper()
	block := twoSidedRect(t, doc, 30, 0, 50, 20, up, down)
	var faces decad.FaceSelector
	if sel != nil {
		faces = sel(block)
	} else {
		opts = append(opts, decad.WithNoOpenings())
	}
	shell, err := decad.Shell(t.Context(), block, faces, th, opts...)
	require.NoError(t, err)
	return shell
}

func removeEnd(b *decad.Body) decad.FaceSelector {
	return decad.Faces(decad.FaceCreatedBy(decad.CapEnd(b)))
}

func removeStart(b *decad.Body) decad.FaceSelector {
	return decad.Faces(decad.FaceCreatedBy(decad.CapStart(b)))
}

// denotedSum is the sum of the values' denoted rationals in millimetres.
func denotedSum(vs ...units.Value) *big.Rat {
	out := new(big.Rat)
	for _, v := range vs {
		out.Add(out, denotedDepth(v))
	}
	return out
}

// TestShellCupDenotedLevel cuts a 20x20 plate with a 4x4 tool that stops at a
// cup's face. Each case names the face, the plate cap it should meet and the
// thickness or offset that makes the stop denote that cap's rational: the cut
// is then the analytic through hole, 384 mm² over the plate's height. Each
// control changes one value so that the stop holds the plate cap's float but
// denotes a plane just inside the plate, and the cut takes the mesh path.
func TestShellCupDenotedLevel(t *testing.T) {
	t.Parallel()
	inch, mm := units.Inches(0.2), units.Millimeters(5.08)
	zero, ten, three := units.Millimeters(0), units.Millimeters(10), units.Millimeters(3)
	two, minusInch := units.Millimeters(2), units.Inches(-0.2)
	for _, tc := range []struct {
		name string
		// The cup: the block's rise and sink, the removed cap, the thickness
		// and sense.
		up, down units.Value
		sel      func(*decad.Body) decad.FaceSelector
		th       units.Value
		outward  bool
		// The stop: the face's role, its offset and whether it is the upward
		// side.
		role   string
		offset units.Value
		upStop bool
		// The plate rises by plateUp and sinks by plateDown.
		plateUp, plateDown units.Value
		// The control replaces th, offset or the plate's stop-side cap.
		controlTh, controlOffset, controlPlate units.Value
	}{
		// The floor is the block's bottom at 0 plus the thickness, r.
		{name: "inward floor", up: ten, down: zero, sel: removeEnd, th: inch, role: roleShellCap,
			upStop: true, plateUp: inch, plateDown: three, controlPlate: mm},
		// The rim keeps the block's 0.2 in top.
		{name: "inward rim", up: inch, down: zero, sel: removeEnd, th: two, role: roleRim0,
			upStop: true, plateUp: inch, plateDown: three, controlPlate: mm},
		// The kept cap keeps the block's bottom at -r.
		{name: "inward kept cap", up: ten, down: inch, sel: removeEnd, th: two, role: roleCapStart,
			plateUp: three, plateDown: inch, controlPlate: mm},
		// The outward floor is the block's bottom at 0 less the thickness, -r.
		{name: "outward floor", up: ten, down: zero, sel: removeEnd, th: inch, outward: true, role: roleCapStart,
			plateUp: three, plateDown: inch, controlPlate: mm},
		// Open at the bottom, the floor is 10 - r and the offset adds r back.
		{name: "inward floor open at the bottom", up: ten, down: zero, sel: removeStart, th: inch,
			role: roleShellCap, offset: inch, upStop: true, plateUp: ten, plateDown: three, controlTh: mm},
		// Open at the bottom outward, the kept cap is 10 + r and the offset
		// takes r off.
		{name: "outward floor open at the bottom", up: ten, down: zero, sel: removeStart, th: inch,
			outward: true, role: roleCapStart, offset: minusInch, upStop: true, plateUp: ten, plateDown: three,
			controlOffset: units.Millimeters(-5.08)},
	} {
		run := func(t *testing.T, th, offset, plateUp, plateDown units.Value) *decad.Body {
			t.Helper()
			doc := decad.New()
			var opts []decad.ShellOption
			if tc.outward {
				opts = append(opts, decad.WithShellSense(decad.Outward))
			}
			cup := shellOf(t, doc, tc.up, tc.down, tc.sel, th, opts...)
			plate := twoSidedRect(t, doc, 0, 0, 20, 20, plateUp, plateDown)
			part, err := decad.Cut(t.Context(), plate, shellStop(t, doc, cup, tc.role, offset, tc.upStop))
			require.NoError(t, err)
			return part
		}
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			part := run(t, tc.th, tc.offset, tc.plateUp, tc.plateDown)
			requireDenotedThroughHole(t, part, denotedSum(tc.plateUp, tc.plateDown))
		})
		t.Run(tc.name+" control", func(t *testing.T) {
			t.Parallel()
			th, offset, plateUp, plateDown := tc.th, tc.offset, tc.plateUp, tc.plateDown
			// toolEnd is how far the tool's stop end reaches past the sketch
			// plane, the plate's own cap on that side being reach.
			var reach, toolEnd *big.Rat
			switch {
			case tc.controlPlate != (units.Value{}) && tc.upStop:
				plateUp, reach, toolEnd = tc.controlPlate, denotedDepth(tc.controlPlate), denotedDepth(tc.plateUp)
			case tc.controlPlate != (units.Value{}):
				plateDown, reach, toolEnd = tc.controlPlate, denotedDepth(tc.controlPlate), denotedDepth(tc.plateDown)
			case tc.controlTh != (units.Value{}):
				// 10 - h + r, just below the 10 mm plate top.
				th, reach = tc.controlTh, denotedDepth(ten)
				toolEnd = new(big.Rat).Sub(denotedSum(ten, offset), denotedDepth(th))
			default:
				// 10 + r - h, just below the 10 mm plate top.
				offset, reach = tc.controlOffset, denotedDepth(ten)
				toolEnd = new(big.Rat).Add(denotedSum(ten, th), denotedDepth(offset))
			}
			require.Negative(t, toolEnd.Cmp(reach), "the control's stop denotes a plane inside the plate")
			part := run(t, th, offset, plateUp, plateDown)
			require.True(t, tookMeshPath(part), "a stop denoting a plane inside the plate proves no span")
			// The plate less the 16 mm² tool over the part of the plate the
			// tool reaches: from the far cap, 3 mm away, to the stop end.
			requireRationalVolumeCovered(t, part, skinnedPlateVolume(reach, toolEnd, 1))
		})
	}
}

// TestShellClosedDenotedLevel stops a tool at a closed shell's cavity floor,
// ceiling and top cap. Over a block on 0..20 mm shelled 0.2 in thick, the
// floor denotes r and the ceiling 20 - r, so a stop at the floor spans a 0.2
// in plate and a stop 0.2 in past the ceiling spans a 20 mm plate. A block on
// 0..0.2 in shelled 1 mm thick keeps its top cap at r. The controls hold the
// same floats and denote a plane just inside the plate.
func TestShellClosedDenotedLevel(t *testing.T) {
	t.Parallel()
	inch, mm := units.Inches(0.2), units.Millimeters(5.08)
	twenty, three := units.Millimeters(20), units.Millimeters(3)
	for _, tc := range []struct {
		name string
		// up is the block's top; zero is 20 mm.
		up      units.Value
		th      units.Value
		role    string
		offset  units.Value
		plateUp units.Value
		// analytic is false for a control; toolEnd is then where the tool's
		// top denotes.
		analytic bool
		toolEnd  *big.Rat
	}{
		{name: "cavity floor", th: inch, role: "floor(0,0)", plateUp: inch, analytic: true},
		{name: "cavity floor control", th: inch, role: "floor(0,0)", plateUp: mm, toolEnd: denotedDepth(inch)},
		{name: "cavity ceiling", th: inch, role: "ceiling(1,0)", offset: inch, plateUp: twenty, analytic: true},
		{name: "cavity ceiling control", th: mm, role: "ceiling(1,0)", offset: inch, plateUp: twenty,
			toolEnd: new(big.Rat).Sub(denotedSum(twenty, inch), denotedDepth(mm))},
		{name: "closed top cap", up: inch, th: units.Millimeters(1), role: roleCapEnd, plateUp: inch, analytic: true},
		{name: "closed top cap control", up: inch, th: units.Millimeters(1), role: roleCapEnd, plateUp: mm,
			toolEnd: denotedDepth(inch)},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			doc := decad.New()
			up := tc.up
			if up == (units.Value{}) {
				up = twenty
			}
			shell := shellOf(t, doc, up, units.Millimeters(0), nil, tc.th)
			plate := twoSidedRect(t, doc, 0, 0, 20, 20, tc.plateUp, three)
			part, err := decad.Cut(t.Context(), plate, shellStop(t, doc, shell, tc.role, tc.offset, true))
			require.NoError(t, err)
			if tc.analytic {
				requireDenotedThroughHole(t, part, denotedSum(tc.plateUp, three))
				return
			}
			require.Negative(t, tc.toolEnd.Cmp(denotedDepth(tc.plateUp)), "the control's stop denotes a plane inside the plate")
			requireDeclinedSpan(t, part, nil, skinnedPlateVolume(denotedDepth(tc.plateUp), tc.toolEnd, 1))
		})
	}
}
