package clearance

import (
	"math"

	"github.com/lestrrat-3d/decad/internal/revolveaxis"
	"github.com/lestrrat-3d/decad/internal/revolvemesh"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/lestrrat-3d/r3"
)

// RevolveWall is a resolved meridian walk and the surface it sweeps.
type RevolveWall struct {
	Walk survey2d.SideWalk
	Kind revolveaxis.WallKind
}

// RevolveCarrierInput is the validated sweep geometry read by the clearance kernel.
type RevolveCarrierInput struct {
	Lift       revolvemesh.RevolveLift
	Transform  r3.Transform
	Full       bool
	Phi0, Phi1 float64
	Walls      []RevolveWall
}

// RevolveCarrierResult holds ordered carrier faces and singular axis points.
type RevolveCarrierResult struct {
	Faces           []*CFace
	Vertices        []r3.Vec
	AxisRadiusUpper float64
}

// BuildRevolveCarriers constructs trimmed carriers from resolved meridian walks.
// The caller MUST have validated the walks and charged their work budget.
// Every axis point it lifts through in.Lift's basis and in.Transform — an
// anchor, a plane origin, a box end, a synthesized singular point — widens the
// LiftRound of the face it builds by that lift's own exact rounding.
func BuildRevolveCarriers(in RevolveCarrierInput) RevolveCarrierResult {
	b := in.Lift.Basis()
	a3p := in.Transform.Apply(b.A3)
	a3Round := in.Lift.ExactPointRound(in.Transform, 0, 0, 1, 0, a3p)
	wp := in.Transform.ApplyDir(b.W)
	e0p := in.Transform.ApplyDir(b.E0)
	e1p := in.Transform.ApplyDir(b.E1)
	sweep := AngWindow{Full: in.Full}
	if !in.Full {
		sweep = NewAngWindow(in.Phi0, in.Phi1)
	}
	midPhi := (in.Phi0 + in.Phi1) / 2
	onAxis := func(f *CFace, z float64) r3.Vec {
		held := a3p.Add(wp.Scale(z))
		f.LiftRound = math.Max(f.LiftRound, in.Lift.ExactPointRound(in.Transform, z, 0, 1, 0, held))
		return held
	}
	// sample places a witness: a float sample the kernel reads as an
	// approximate surface point, never a recorded one, so it records no
	// LiftRound. Every recorded axis point goes through onAxis instead.
	sample := func(z, rho, phi float64) r3.Vec {
		sin, cos := math.Sincos(phi)
		radial := b.E0.Scale(cos).Add(b.E1.Scale(sin))
		return in.Transform.Apply(b.A3.Add(b.W.Scale(z)).Add(radial.Scale(rho)))
	}

	var out RevolveCarrierResult
	var capElems []survey2d.SurveyElem
	for _, wall := range in.Walls {
		w := wall.Walk
		if el, ok := survey2d.WalkElem(w.SegmentWalk); ok {
			capElems = append(capElems, el)
		}
		out.AxisRadiusUpper = math.Max(out.AxisRadiusUpper, w.AxisRadiusUpper)
		switch wall.Kind {
		case revolveaxis.WallAxis:
			continue
		case revolveaxis.WallCylinder:
			r := (w.StartV + w.EndV) / 2
			f := &CFace{
				Kind: CkCylinder, Anchor: a3p, Axis: wp, RefU: e0p, RefV: e1p,
				Radius: r, ZWin: NewLinWindow(w.StartU, w.EndU), Sweep: sweep,
				LiftRound: a3Round,
			}
			f.Box = BoxUnion(CircleBox(onAxis(f, w.StartU), wp, r), CircleBox(onAxis(f, w.EndU), wp, r))
			f.Wit = append(f.Wit, sample((w.StartU+w.EndU)/2, r, midPhi), sample(w.StartU, r, in.Phi0))
			out.Faces = append(out.Faces, f)
		case revolveaxis.WallPlane:
			rlo := math.Min(w.StartV, w.EndV)
			rhi := math.Max(w.StartV, w.EndV)
			sign := 1.0
			if w.TanInV < 0 {
				sign = -1
			}
			f := &CFace{
				Kind: CkPlane,
				U:    e0p, V: e1p,
				N: wp.Scale(sign),
			}
			f.O = onAxis(f, w.StartU)
			f.Region = AnnularRegion(rlo, rhi, in.Phi0, in.Phi1, in.Full)
			f.Box = CapBox(f)
			f.Wit = CapWitnesses(f)
			out.Faces = append(out.Faces, f)
		case revolveaxis.WallCone:
			dz, dr := w.EndU-w.StartU, w.EndV-w.StartV
			apexZ := w.StartU - w.StartV*dz/dr
			growth := 1.0
			if dz*dr < 0 {
				growth = -1
			}
			f := &CFace{
				Kind: CkCone,
				Axis: wp.Scale(growth),
				RefU: e0p, RefV: e1p,
				Half:  math.Atan2(math.Abs(dr), math.Abs(dz)),
				ZWin:  NewLinWindow(math.Abs(w.StartU-apexZ), math.Abs(w.EndU-apexZ)),
				Sweep: sweep,
			}
			f.Anchor = onAxis(f, apexZ)
			f.Box = BoxUnion(CircleBox(onAxis(f, w.StartU), wp, w.StartV), CircleBox(onAxis(f, w.EndU), wp, w.EndV))
			f.Wit = append(f.Wit, sample((w.StartU+w.EndU)/2, (w.StartV+w.EndV)/2, midPhi))
			out.Faces = append(out.Faces, f)
			if w.StartV <= 0 || w.EndV <= 0 {
				// The apex sits on the trimmed face: a surface singular
				// point, synthesized as a vertex-like candidate (§3).
				out.Vertices = append(out.Vertices, f.Anchor)
			}
		case revolveaxis.WallSphere:
			f := &CFace{
				Kind: CkSphere,
				Axis: wp,
				RefU: e0p, RefV: e1p,
				Radius: w.Radius,
				Merid:  AngWindow{Full: w.Closed},
				Sweep:  sweep,
			}
			f.Anchor = onAxis(f, w.CU)
			if !w.Closed {
				f.Merid = NewAngWindow(w.Th0, w.Th1)
			}
			f.Box = [2]r3.Vec{
				f.Anchor.Sub(r3.NewVec(w.Radius, w.Radius, w.Radius)),
				f.Anchor.Add(r3.NewVec(w.Radius, w.Radius, w.Radius)),
			}
			midTh := (w.Th0 + w.Th1) / 2
			f.Wit = append(f.Wit, sample(w.CU+w.Radius*math.Cos(midTh), math.Max(0, w.Radius*math.Sin(midTh)), midPhi))
			out.Faces = append(out.Faces, f)
		case revolveaxis.WallTorus:
			f := &CFace{
				Kind: CkTorus,
				Axis: wp,
				RefU: e0p, RefV: e1p,
				Radius:  w.Radius,
				Major:   w.CV,
				Merid:   AngWindow{Full: w.Closed},
				Sweep:   sweep,
				Spindle: w.Radius >= w.CV-ClrAngTol*math.Max(1, w.CV),
			}
			f.Anchor = onAxis(f, w.CU)
			if !w.Closed {
				f.Merid = NewAngWindow(w.Th0, w.Th1)
			}
			spineBox := CircleBox(f.Anchor, wp, f.Major)
			pad := r3.NewVec(w.Radius, w.Radius, w.Radius)
			f.Box = [2]r3.Vec{spineBox[0].Sub(pad), spineBox[1].Add(pad)}
			midTh := (w.Th0 + w.Th1) / 2
			f.Wit = append(f.Wit, sample(w.CU+w.Radius*math.Cos(midTh), w.CV+w.Radius*math.Sin(midTh), midPhi))
			out.Faces = append(out.Faces, f)
			// A spindle patch reaching the axis at a walk endpoint has a
			// singular axis-collapse point there, synthesized like a cone
			// apex (§3).
			if !w.Closed && w.StartV == 0 {
				out.Vertices = append(out.Vertices, onAxis(f, w.CU+w.Radius*math.Cos(w.Th0)))
			}
			if !w.Closed && w.EndV == 0 {
				out.Vertices = append(out.Vertices, onAxis(f, w.CU+w.Radius*math.Cos(w.Th1)))
			}
		}
	}

	if !in.Full {
		region := NewRegion2(capElems)
		for _, cap := range []struct {
			phi  float64
			sign float64
		}{{phi: in.Phi0, sign: -1}, {phi: in.Phi1, sign: 1}} {
			sin, cos := math.Sincos(cap.phi)
			radial := e0p.Scale(cos).Add(e1p.Scale(sin))
			vel := e0p.Scale(-sin).Add(e1p.Scale(cos))
			f := &CFace{
				Kind:      CkPlane,
				O:         a3p,
				U:         wp,
				V:         radial,
				N:         vel.Scale(cap.sign),
				Region:    region,
				LiftRound: a3Round,
			}
			f.Box = CapBox(f)
			f.Wit = CapWitnesses(f)
			out.Faces = append(out.Faces, f)
		}
	}
	return out
}
