package clearance

import (
	"math"

	"github.com/lestrrat-3d/decad/internal/survey2d"

	"github.com/lestrrat-3d/decad/internal/proofbound"

	"github.com/lestrrat-3d/r3"
)

// CKind discriminates the kernel's carrier variants.
type CKind int

const (
	CkPlane CKind = iota
	CkCylinder
	CkCone
	CkSphere
	CkTorus
)

// CFace is one trimmed face in world coordinates: the carrier plus its
// closed-form trim, a containing box, and on-face witness points.
type CFace struct {
	Kind CKind

	// plane: o + x·u + y·v, outward normal n; trim = region.
	O, U, V, N r3.Vec
	Region     Region2

	// axis-symmetric carriers: anchor + axis (unit) + azimuth frame.
	Anchor     r3.Vec
	Axis       r3.Vec
	RefU, RefV r3.Vec
	Radius     float64 // cylinder/sphere radius, torus minor
	Major      float64 // torus major
	Half       float64 // cone half-angle
	Sweep      AngWindow
	ZWin       LinWindow // cylinder: axial range; cone: axial range from apex
	Merid      AngWindow // sphere/torus meridian window
	Spindle    bool      // torus minor >= major: off the polynomial path (§4)

	Box [2]r3.Vec
	Wit []r3.Vec

	// LiftRound is the largest exact rounding the payload's own frame lift and
	// accumulated placement committed on any recorded point this carrier was
	// built from (its anchor or origin, the other end of its axis and, for a
	// revolve, any singular axis point it synthesized) — measured per point
	// over exact rationals (proofbound.ExactFrameLiftRound,
	// revolvemesh.RevolveLift.ExactPointRound), so a lift that is exact for the
	// coordinates at hand contributes zero. Witnesses are float samples and
	// record nothing. The caller folds it into the body's own displacement
	// (decad's bodyGeom.delta); the kernel itself never reads it.
	LiftRound float64
}

// AdmitState folds two per-side admission classifications: any −1 rejects,
// else any 0 downgrades to a straddle.
func AdmitState(cs ...int) int {
	out := 1
	for _, c := range cs {
		if c == -1 {
			return -1
		}
		if c == 0 {
			out = 0
		}
	}
	return out
}

// planeCoords maps a world point into the plane face's 2D frame.
func (f *CFace) PlaneCoords(p r3.Vec) (float64, float64) {
	rel := p.Sub(f.O)
	return rel.Dot(f.U), rel.Dot(f.V)
}

// planeOffset is the face plane's offset along its unit normal.
func (f *CFace) PlaneOffset() float64 { return f.N.Dot(f.O) }

// axisCoords maps a world point into (z along axis from anchor, ρ, azimuth).
func (f *CFace) AxisCoords(p r3.Vec) (float64, float64, float64) {
	rel := p.Sub(f.Anchor)
	z := rel.Dot(f.Axis)
	rad := rel.Sub(f.Axis.Scale(z))
	rho := rad.Len()
	return z, rho, math.Atan2(rad.Dot(f.RefV), rad.Dot(f.RefU))
}

// admitPoint classifies a world point claimed to lie on the face's carrier
// against the trim: +1 admitted with margin, −1 rejected with margin, 0
// ambiguous. margin is a length.
func (f *CFace) AdmitPoint(p r3.Vec, margin float64) int {
	switch f.Kind {
	case CkPlane:
		x, y := f.PlaneCoords(p)
		return f.Region.Classify(x, y, margin)
	case CkCylinder:
		z, _, phi := f.AxisCoords(p)
		angMargin := margin / math.Max(f.Radius, 1e-30)
		return AdmitState(f.ZWin.Classify(z, margin), f.Sweep.Classify(phi, angMargin))
	case CkCone:
		z, rho, phi := f.AxisCoords(p)
		angMargin := margin / math.Max(rho, 1e-30)
		return AdmitState(f.ZWin.Classify(z, margin), f.Sweep.Classify(phi, angMargin))
	case CkSphere:
		z, rho, phi := f.AxisCoords(p)
		th := math.Atan2(rho, z)
		angMargin := margin / math.Max(f.Radius, 1e-30)
		c := f.Merid.Classify(th, angMargin)
		if rho <= margin {
			// A pole: every azimuth matches; only the meridian decides.
			return c
		}
		return AdmitState(c, f.Sweep.Classify(phi, margin/math.Max(rho, 1e-30)))
	default: // CkTorus
		z, rho, phi := f.AxisCoords(p)
		mu := math.Atan2(rho-f.Major, z)
		angMargin := margin / math.Max(f.Radius, 1e-30)
		c := f.Merid.Classify(mu, angMargin)
		if rho <= margin {
			return c
		}
		return AdmitState(c, f.Sweep.Classify(phi, margin/math.Max(rho, 1e-30)))
	}
}

// BoxOf grows a box over points.
func BoxOf(pts ...r3.Vec) [2]r3.Vec {
	lo := r3.NewVec(math.Inf(1), math.Inf(1), math.Inf(1))
	hi := r3.NewVec(math.Inf(-1), math.Inf(-1), math.Inf(-1))
	for _, p := range pts {
		lo = r3.NewVec(math.Min(lo.X, p.X), math.Min(lo.Y, p.Y), math.Min(lo.Z, p.Z))
		hi = r3.NewVec(math.Max(hi.X, p.X), math.Max(hi.Y, p.Y), math.Max(hi.Z, p.Z))
	}
	return [2]r3.Vec{lo, hi}
}

func BoxUnion(a, b [2]r3.Vec) [2]r3.Vec {
	return [2]r3.Vec{
		r3.NewVec(math.Min(a[0].X, b[0].X), math.Min(a[0].Y, b[0].Y), math.Min(a[0].Z, b[0].Z)),
		r3.NewVec(math.Max(a[1].X, b[1].X), math.Max(a[1].Y, b[1].Y), math.Max(a[1].Z, b[1].Z)),
	}
}

// CircleBox is the exact box of a full 3D circle.
func CircleBox(c, axis r3.Vec, r float64) [2]r3.Vec {
	ext := r3.NewVec(
		r*math.Sqrt(math.Max(0, 1-axis.X*axis.X)),
		r*math.Sqrt(math.Max(0, 1-axis.Y*axis.Y)),
		r*math.Sqrt(math.Max(0, 1-axis.Z*axis.Z)),
	)
	return [2]r3.Vec{c.Sub(ext), c.Add(ext)}
}

// ClrBoxDist is the distance between two boxes (zero when they meet).
func ClrBoxDist(a, b [2]r3.Vec) float64 {
	gap := func(alo, ahi, blo, bhi float64) float64 {
		if ahi < blo {
			return blo - ahi
		}
		if bhi < alo {
			return alo - bhi
		}
		return 0
	}
	dx := gap(a[0].X, a[1].X, b[0].X, b[1].X)
	dy := gap(a[0].Y, a[1].Y, b[0].Y, b[1].Y)
	dz := gap(a[0].Z, a[1].Z, b[0].Z, b[1].Z)
	return math.Sqrt(dx*dx + dy*dy + dz*dz)
}

// FaceFootExtent bounds |p − f.o| over every point p a planar face's own trim
// can admit. f.box contains f.region by construction (every arm that builds a
// CkPlane face's box — CapBox, BoxOf's own callers — grows it to cover the
// region), so the farthest of the box's eight corners from the foot o is a
// sound, if not tight, upper bound on the face's own extent.
func FaceFootExtent(f *CFace) float64 {
	best := 0.0
	for _, x := range []float64{f.Box[0].X, f.Box[1].X} {
		for _, y := range []float64{f.Box[0].Y, f.Box[1].Y} {
			for _, z := range []float64{f.Box[0].Z, f.Box[1].Z} {
				if d := r3.NewVec(x, y, z).Sub(f.O).Len(); d > best {
					best = d
				}
			}
		}
	}
	return best
}

// PlaneTiltAllow bounds the displacement one rounded carrier PLANE commits
// across its own extent from its foot f.o. addPrismFaces and addRevolveFaces
// build a plane carrier's u, v and outward normal n through the payload's own
// `dir`-style construction in raw float64 (docs/clearance-design.md §2), so n
// is not exactly perpendicular to the plane it is meant to describe. For a
// point p genuinely on the TRUE
// plane through o with the TRUE normal n_true, the rounded plane's own
// equation reads n·(p−o) = (n−n_true)·(p−o) + n_true·(p−o) = (n−n_true)·(p−o)
// — the second term vanishes by definition of the true plane — which
// Cauchy–Schwarz bounds by |n−n_true|·|p−o|. proofbound.DirRoundAllow bounds the first
// factor for the frame/placement composition every such normal is built
// through (a unit-scale input, since every direction the carrier model feeds it
// is a unit or axis vector); the ×4 safety factor is prismPointBound's own rule
// for a value that passes through more than one held linear map — here the
// frame lift, a cross product and a normalize — before publication, so it is
// sound rather than tight. FaceFootExtent bounds the second factor.
func PlaneTiltAllow(f *CFace, frame r3.Frame, xform r3.Transform) float64 {
	if f.Kind != CkPlane {
		return 0
	}
	normalErr := proofbound.ProductUpper(4, proofbound.DirRoundAllow(frame, xform, 1))
	return proofbound.ProductUpper(normalErr, FaceFootExtent(f))
}

// BodyFaceTiltDelta is the worst PlaneTiltAllow over every planar face a
// body's carrier model built — the one per-body term addPrismFaces and
// addRevolveFaces fold into bodyGeom.delta beside the point and axial/angular
// terms (bodyGeom's own doc comment).
func BodyFaceTiltDelta(faces []*CFace, frame r3.Frame, xform r3.Transform) float64 {
	best := 0.0
	for _, f := range faces {
		if t := PlaneTiltAllow(f, frame, xform); t > best {
			best = t
		}
	}
	return best
}

// CEdge is one boundary edge: a segment or a circular arc/circle in world
// coordinates.
type CEdge struct {
	Line   bool
	A, B   r3.Vec // segment endpoints
	Center r3.Vec
	Axis   r3.Vec
	RefU   r3.Vec
	RefV   r3.Vec
	Radius float64
	Ang    AngWindow
	Box    [2]r3.Vec
}

func (e *CEdge) At(th float64) r3.Vec {
	s, c := math.Sincos(th)
	return e.Center.Add(e.RefU.Scale(e.Radius * c)).Add(e.RefV.Scale(e.Radius * s))
}

// PerpTo returns a deterministic unit vector perpendicular to a unit vector.
func PerpTo(a r3.Vec) r3.Vec {
	seed := r3.NewVec(1, 0, 0)
	if math.Abs(a.X) > 0.9 {
		seed = r3.NewVec(0, 1, 0)
	}
	p, _ := a.Cross(seed).Normalize()
	return p
}

// CapBox is the world box of a planar face's region boundary (arcs taken as
// their full circles — conservative, and a box only needs to contain).
func CapBox(f *CFace) [2]r3.Vec {
	at := func(x, y float64) r3.Vec { return f.O.Add(f.U.Scale(x)).Add(f.V.Scale(y)) }
	box := [2]r3.Vec{
		r3.NewVec(math.Inf(1), math.Inf(1), math.Inf(1)),
		r3.NewVec(math.Inf(-1), math.Inf(-1), math.Inf(-1)),
	}
	for _, e := range f.Region.Elems {
		if e.Kind == survey2d.SurveyLine {
			box = BoxUnion(box, BoxOf(at(e.Ax, e.Ay), at(e.Bx, e.By)))
			continue
		}
		axis := f.U.Cross(f.V)
		box = BoxUnion(box, CircleBox(at(e.Qx, e.Qy), axis, e.Rr))
	}
	return box
}

// CapWitnesses returns on-face points: boundary samples plus a verified
// interior point when one is found.
func CapWitnesses(f *CFace) []r3.Vec {
	var out []r3.Vec
	for _, s := range f.Region.Samples() {
		out = append(out, f.O.Add(f.U.Scale(s[0])).Add(f.V.Scale(s[1])))
	}
	if p, ok := f.Region.InteriorPoint(); ok {
		out = append(out, f.O.Add(f.U.Scale(p[0])).Add(f.V.Scale(p[1])))
	}
	return out
}

// AnnularRegion builds the 2D region of a revolve wallPlane face in its
// (e0, e1) frame: an annular sector, a disk sector, an annulus or a disk.
func AnnularRegion(rlo, rhi, phi0, phi1 float64, full bool) Region2 {
	var elems []survey2d.SurveyElem
	if full {
		if e, ok := survey2d.ArcElem(0, 0, rhi, 0, 2*math.Pi, true); ok {
			elems = append(elems, e)
		}
		if rlo > 0 {
			if e, ok := survey2d.ArcElem(0, 0, rlo, 0, 2*math.Pi, true); ok {
				elems = append(elems, e)
			}
		}
		return NewRegion2(elems)
	}
	if e, ok := survey2d.ArcElem(0, 0, rhi, phi0, phi1, false); ok {
		elems = append(elems, e)
	}
	if rlo > 0 {
		if e, ok := survey2d.ArcElem(0, 0, rlo, phi0, phi1, false); ok {
			elems = append(elems, e)
		}
	}
	s0, c0 := math.Sincos(phi0)
	s1, c1 := math.Sincos(phi1)
	if e, ok := survey2d.LineElem(rlo*c0, rlo*s0, rhi*c0, rhi*s0); ok {
		elems = append(elems, e)
	}
	if e, ok := survey2d.LineElem(rlo*c1, rlo*s1, rhi*c1, rhi*s1); ok {
		elems = append(elems, e)
	}
	return NewRegion2(elems)
}
