package clearance

import (
	"math"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/lestrrat-3d/r3"
)

// PrismCarrierFrame places section coordinates and sweep levels in world space.
type PrismCarrierFrame struct {
	Frame     r3.Frame
	Transform r3.Transform
	Z0, Z1    float64
}

// sample lifts plane-local (u, v) at level z into placed world space.
func (p PrismCarrierFrame) sample(u, v, z float64) r3.Vec {
	local := p.Frame.ToWorldUV(u, v)
	n := p.Frame.N()
	return p.Transform.Apply(local.Add(n.Scale(z)))
}

// point lifts a recorded plane-local (u, v) at level z — a carrier's anchor,
// origin or axis end, a point the record states — and widens f.LiftRound by
// the exact rounding that lift committed. A witness is a float sample the
// kernel reads as an approximate surface point, never as a recorded one, so
// it lifts through sample and records nothing.
func (p PrismCarrierFrame) point(f *CFace, u, v, z float64) r3.Vec {
	held := p.sample(u, v, z)
	f.LiftRound = math.Max(f.LiftRound, proofbound.ExactFrameLiftRound(p.Frame, p.Transform, u, v, z, held))
	return held
}

func (p PrismCarrierFrame) dir(du, dv, dz float64) r3.Vec {
	world := p.Frame.U().Scale(du).Add(p.Frame.V().Scale(dv)).Add(p.Frame.N().Scale(dz))
	return p.Transform.ApplyDir(world)
}

// PrismCarrierResult holds the ordered carrier faces.
type PrismCarrierResult struct {
	Faces []*CFace
}

// BuildPrismCarriers builds wall and, for a solid, cap carriers from validated walks.
// A surface result contributes only its walls; both cap-only regions are omitted.
// budget MUST NOT be nil. A walk this kernel cannot carry returns ok=false.
func BuildPrismCarriers(budget *proofbound.WorkBudget, p PrismCarrierFrame,
	loops [][]survey2d.SideWalk, surfaceResult bool,
) (PrismCarrierResult, bool, error) {
	var out PrismCarrierResult
	var capElems []survey2d.SurveyElem
	for _, loop := range loops {
		for _, w := range loop {
			if err := budget.Step(); err != nil {
				return PrismCarrierResult{}, false, err
			}
			el, ok := survey2d.WalkElem(w.SegmentWalk)
			if !ok {
				return PrismCarrierResult{}, false, nil
			}
			if !surfaceResult {
				capElems = append(capElems, el)
			}
			f, ok := PrismWallCarrier(p, w.SegmentWalk)
			if !ok {
				return PrismCarrierResult{}, false, nil
			}
			out.Faces = append(out.Faces, f)
		}
	}
	if !surfaceResult {
		region := NewRegion2(capElems)
		for _, cap := range []struct {
			z    float64
			sign float64
		}{{z: p.Z0, sign: -1}, {z: p.Z1, sign: 1}} {
			out.Faces = append(out.Faces, PrismPlaneCarrier(p, cap.z, cap.sign, region))
		}
	}
	return out, true, nil
}

// PrismWallCarrier builds one prism side's carrier: a cylinder for a circular
// walk, or a plane rectangle for a line walk with its outward normal the
// walk's right, turned back for a reflected placement. It refuses a
// zero-length line or a direction that does not normalize.
func PrismWallCarrier(p PrismCarrierFrame, w survey2d.SegmentWalk) (*CFace, bool) {
	nDir := p.dir(0, 0, 1)
	h := p.Z1 - p.Z0
	if w.IsCircular() {
		f := &CFace{
			Kind:   CkCylinder,
			Axis:   nDir,
			RefU:   p.dir(1, 0, 0),
			RefV:   p.dir(0, 1, 0),
			Radius: w.Radius,
			ZWin:   NewLinWindow(0, h),
			Sweep:  AngWindow{Full: w.Closed},
		}
		f.Anchor = p.point(f, w.CU, w.CV, p.Z0)
		if !w.Closed {
			f.Sweep = NewAngWindow(w.Th0, w.Th1)
		}
		top := p.point(f, w.CU, w.CV, p.Z1)
		f.Box = BoxUnion(CircleBox(f.Anchor, nDir, w.Radius), CircleBox(top, nDir, w.Radius))
		midTh := (w.Th0 + w.Th1) / 2
		f.Wit = append(f.Wit,
			p.sample(w.CU+w.Radius*math.Cos(midTh), w.CV+w.Radius*math.Sin(midTh), (p.Z0+p.Z1)/2),
			p.sample(w.CU+w.Radius*math.Cos(w.Th0), w.CV+w.Radius*math.Sin(w.Th0), p.Z0))
		return f, true
	}

	l := math.Hypot(w.EndU-w.StartU, w.EndV-w.StartV)
	if l == 0 {
		return nil, false
	}
	e1, ok := p.dir(w.EndU-w.StartU, w.EndV-w.StartV, 0).Normalize()
	if !ok {
		return nil, false
	}
	tu, tv := w.TanInU, w.TanInV
	if p.Transform.IsReflection() {
		tu, tv = -tu, -tv
	}
	nOut, ok := p.dir(tu, tv, 0).Cross(nDir).Normalize()
	if !ok {
		return nil, false
	}
	f := &CFace{
		Kind: CkPlane,
		U:    e1,
		V:    nDir,
		N:    nOut,
	}
	f.O = p.point(f, w.StartU, w.StartV, p.Z0)
	le0, _ := survey2d.LineElem(0, 0, l, 0)
	le1, _ := survey2d.LineElem(l, 0, l, h)
	le2, _ := survey2d.LineElem(l, h, 0, h)
	le3, _ := survey2d.LineElem(0, h, 0, 0)
	f.Region = NewRegion2([]survey2d.SurveyElem{le0, le1, le2, le3})
	f.Box = BoxOf(f.O, f.O.Add(e1.Scale(l)), f.O.Add(nDir.Scale(h)), f.O.Add(e1.Scale(l)).Add(nDir.Scale(h)))
	f.Wit = append(f.Wit, f.O.Add(e1.Scale(l/2)).Add(nDir.Scale(h/2)), f.O)
	return f, true
}

// PrismPlaneCarrier builds a planar cap at level z with the given trim region.
func PrismPlaneCarrier(p PrismCarrierFrame, z, sign float64, region Region2) *CFace {
	f := &CFace{
		Kind:   CkPlane,
		U:      p.dir(1, 0, 0),
		V:      p.dir(0, 1, 0),
		N:      p.dir(0, 0, 1).Scale(sign),
		Region: region,
	}
	f.O = p.point(f, 0, 0, z)
	f.Box = CapBox(f)
	f.Wit = CapWitnesses(f)
	return f
}
