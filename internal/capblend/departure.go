package capblend

import (
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/capband"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
)

// patchNormalAllow adapts built cap-band faces to the exact departure
// bounds in internal/capband/departure.go.

// ruledNormalAllowUnbounded is the trivial bound on the distance between two
// unit vectors: any two of them differ by at most 2. It is what
// patchNormalAllow returns wherever its own derivation degenerates — an
// honest "this reading proves nothing" rather than a smaller number nothing
// supports. Every consumer treats it as undecided rather than as a width.
const ruledNormalAllowUnbounded = 2.0

// patchNormalAllow is the file's answer for one patch: the proven bound on
// how far the RULED surface the build assembles can carry a normal differing
// from the surface the patch publishes.
//
// Both patch kinds answer from their own held corners, and neither is exempt.
// A straight wall's offset family is affine in the offset amount, so the
// surface a flat patch denotes is a plane — but the corners the build emits are
// each rounded once more, and the tag is fixed through three of the four, so
// the built quad still leaves the plane by an amount only a world-space reading
// of those four corners states. How far the built patch turns from the denoted
// one is a separate term (capband.DenotedNormalAllow), which setPatchReadings
// adds.
func patchNormalAllow(f *topo.Face, g capband.Patch, b capband.PatchBuilt) float64 {
	departure := planeDeparture
	if g.Circular {
		departure = patchDeparture
	}
	allow, ok := departure(f, b)
	if !ok || proofbound.IsNonFinite(allow) || allow < 0 {
		return ruledNormalAllowUnbounded
	}
	return math.Min(ruledNormalAllowUnbounded, allow)
}

// planeDeparture reads the published plane tag before bounding its built patch.
func planeDeparture(f *topo.Face, b capband.PatchBuilt) (float64, bool) {
	tag, okTag := f.Surface.(surfacegeom.Plane)
	if !okTag {
		return 0, false
	}
	return capband.PlaneDeparture(tag.Frame.U(), tag.Frame.V(), b)
}

// patchDeparture reads the published cone tag before bounding its built patch.
func patchDeparture(f *topo.Face, b capband.PatchBuilt) (float64, bool) {
	sinH, cosH, originVec, axisVec, ok := ConeTagTerms(f)
	if !ok {
		return 0, false
	}
	tagRadius, okR := coneTagRadius(f)
	if !okR {
		return 0, false
	}
	return capband.ConeDeparture(sinH, cosH, originVec, axisVec, tagRadius, b)
}

// coneTagRadius is the radius a circular patch's own tag carries at its origin
// — zero for every tag the cap band builds, since coneSurface anchors a Cone at
// its apex. It is read rather than assumed, so the signed distance κ this file
// measures stays the tag's own implicit function whatever radius a tag states.
func coneTagRadius(f *topo.Face) (*big.Rat, bool) {
	var value units.Value
	switch s := f.Surface.(type) {
	case surfacegeom.Cone:
		value = s.Radius
	case surfacegeom.Cylinder:
		value = s.Radius
	default:
		return nil, false
	}
	mm, err := value.In(units.Millimeter)
	if err != nil {
		return nil, false
	}
	r := proofarith.FloatRat(mm)
	if r == nil {
		return nil, false
	}
	return r, true
}

// builtPatch names one band patch's built ruled surface from the two
// published directrix edges and the boundary vertices that pair them: ruling k
// joins sideEnds[k] to capEnds[k]. A directrix this evaluator cannot read
// leaves its own reference empty, which either departure derivation refuses and
// the caller publishes the trivial bound for, rather than a number no reading
// supports.
func builtPatch(sideEdge, capEdge *topo.Edge, sideEnds, capEnds []*topo.Vertex) capband.PatchBuilt {
	return capband.PatchBuilt{
		SideDir: directrixFromEdge(sideEdge, sideEnds...),
		CapDir:  directrixFromEdge(capEdge, capEnds...),
	}
}

// builtApexPatch is the same for a reflex corner's apex patch, whose side
// directrix is the single original corner VERTEX the rulings all leave: a
// circle of radius zero, which patchDeparture admits only where that vertex
// is the tag's own origin held for held.
func builtApexPatch(apex *topo.Vertex, capEdge *topo.Edge, capEnds []*topo.Vertex) capband.PatchBuilt {
	capped := directrixFromEdge(capEdge, capEnds...)
	if apex == nil {
		return capband.PatchBuilt{CapDir: capped}
	}
	ends := make([]r3.Vec, len(capped.Ends))
	for i := range ends {
		ends[i] = apex.Position
	}
	return capband.PatchBuilt{
		SideDir: capband.DirectrixRef{Center: apex.Position, Ends: ends},
		CapDir:  capped,
	}
}

// directrixFromEdge reads one published directrix edge. Only the three curve
// kinds a band's directrix is ever built as are admitted — the two circular ones
// a wall over an arc gives, and the `Line3` a straight wall gives, which carries
// no circle and is read through its ruling endpoints alone; anything else leaves
// the patch without a stated built surface and the caller publishes the trivial
// bound.
func directrixFromEdge(e *topo.Edge, ends ...*topo.Vertex) capband.DirectrixRef {
	var out capband.DirectrixRef
	var radius units.Value
	switch c := e.Curve.(type) {
	case surfacegeom.Arc3:
		out.Center, out.Axis, radius = c.Center, c.Axis, c.Radius
	case surfacegeom.Circle3:
		out.Center, out.Axis, radius = c.Center, c.Axis, c.Radius
	case surfacegeom.Line3:
		out.Straight = true
	default:
		return capband.DirectrixRef{}
	}
	if !out.Straight {
		mm, err := radius.In(units.Millimeter)
		if err != nil {
			return capband.DirectrixRef{}
		}
		out.Radius = mm
	}
	for _, v := range ends {
		if v == nil {
			return capband.DirectrixRef{}
		}
		out.Ends = append(out.Ends, v.Position)
	}
	return out
}

// ConeTagTerms reads the sine and cosine of a circular patch's own half angle
// beside the axis they lean against. A `Cylinder` is the zero-half-angle member
// of the same family — `Face.NormalAt` hands back the bare radial direction
// there — so it takes the exact pair rather than a second code path. Any other
// tag refuses: a patch whose surface this file cannot state exactly gets no
// model at all, and DX7 answers undecided.
func ConeTagTerms(f *topo.Face) (proofbound.RatInterval, proofbound.RatInterval, r3.Vec, r3.Vec, bool) {
	switch s := f.Surface.(type) {
	case surfacegeom.Cone:
		half, err := s.HalfAngle.In(units.Radian)
		if err != nil {
			return proofbound.RatInterval{}, proofbound.RatInterval{}, r3.Vec{}, r3.Vec{}, false
		}
		rHalf := proofarith.FloatRat(half)
		if rHalf == nil {
			return proofbound.RatInterval{}, proofbound.RatInterval{}, r3.Vec{}, r3.Vec{}, false
		}
		sin, cos, ok := proofbound.RadSinCosInterval(rHalf)
		if !ok {
			return proofbound.RatInterval{}, proofbound.RatInterval{}, r3.Vec{}, r3.Vec{}, false
		}
		return sin, cos, s.Origin, s.Axis, true
	case surfacegeom.Cylinder:
		return proofbound.PointInterval(new(big.Rat)), proofbound.PointInterval(big.NewRat(1, 1)), s.Origin, s.Axis, true
	default:
		return proofbound.RatInterval{}, proofbound.RatInterval{}, r3.Vec{}, r3.Vec{}, false
	}
}
